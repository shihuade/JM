
/*!
 ***************************************************************************
 * \file md_low.c
 *
 * \brief
 *    Main macroblock mode decision functions and helpers
 *
 **************************************************************************
 */

#include <math.h>
#include <limits.h>
#include <float.h>

#include "global.h"
#include "rdopt_coding_state.h"
#include "mb_access.h"
#include "intrarefresh.h"
#include "image.h"
#include "transform8x8.h"
#include "ratectl.h"
#include "mode_decision.h"
#include "mode_decision_p8x8.h"
#include "fmo.h"
#include "me_umhex.h"
#include "me_umhexsmp.h"
#include "macroblock.h"
#include "q_around.h"
#include "vlc.h"
#include "md_common.h"
#include "rdopt.h"
#include "memalloc.h"
#include "mc_prediction.h"
#include "rd_intra_jm.h"

/*!
 ************************************************************************
 * \brief
 *    RDOff decision between 4x4 and 8x8 transform
 ************************************************************************
 */
static int get_best_transform_8x8(Macroblock *currMB)
{ 
  if(currMB->p_Inp->Transform8x8Mode == 2) //always use the 8x8 transform
    return 1;
  else
  {
    VideoParameters *p_Vid = currMB->p_Vid;  
    RDOPTStructure  *p_RDO = currMB->p_Slice->p_RDO;

    short diff16[4][16];
    short diff64[64];
    int     pic_pix_y, pic_pix_x, i, j;
    int     mb_y, mb_x, block8x8;
    distblk cost8x8=0, cost4x4=0;

    for (block8x8 = 0; block8x8 < 4; block8x8++)
    {
      short *tmp64 = diff64;
      short *tmp16[4]; //{diff16[0], diff16[1], diff16[2], diff16[3]};
      short source_pel, index;

      tmp16[0] = diff16[0];
      tmp16[1] = diff16[1];
      tmp16[2] = diff16[2];
      tmp16[3] = diff16[3];

      mb_y = (block8x8 >> 1) << 3;
      mb_x = (block8x8 & 0x01) << 3;

      //===== loop over 8x8 blocks =====
      pic_pix_y = currMB->opix_y;
      pic_pix_x = currMB->pix_x;
      //===== get displaced frame difference ======
      for (j = mb_y; j < 8 + mb_y; j++)
      {
        for (i = mb_x; i < 8 + mb_x; i++)
        {
          index = (short) (2 * ((j - mb_y)> 3) + ((i - mb_x)> 3));
          source_pel = p_Vid->pCurImg[pic_pix_y + j][pic_pix_x + i];
          *(tmp16[index])++ = (short) (source_pel - p_RDO->tr4x4->mpr8x8[j][i]);
          *tmp64++          = (short) (source_pel - p_RDO->tr8x8->mpr8x8[j][i]);
        }
      }

      cost4x4 += p_Vid->distortion4x4 (diff16[0], DISTBLK_MAX);
      cost4x4 += p_Vid->distortion4x4 (diff16[1], DISTBLK_MAX);
      cost4x4 += p_Vid->distortion4x4 (diff16[2], DISTBLK_MAX);
      cost4x4 += p_Vid->distortion4x4 (diff16[3], DISTBLK_MAX);
      cost8x8 += p_Vid->distortion8x8 (diff64   , DISTBLK_MAX);
    }

    return (cost8x8 < cost4x4);
  }
}

/*!
*************************************************************************************
* \brief
*    Mode Decision for a macroblock
*************************************************************************************
*/
void encode_one_macroblock_low (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  imgpel ***temp_img; // to temp store the Y data for 8x8 transform

  int         block, mode, j;
  RD_PARAMS   enc_mb;
  distblk       bmcost[5] = {DISTBLK_MAX};
  distblk       rd_cost = 0;
  distblk       cost = 0;
  distblk       min_cost = DISTBLK_MAX;
  distblk       cost_direct=0;
  distblk       cost8x8_direct = 0;
  int         have_direct=0;
  int         intra1 = 0;
  int         temp_cpb = 0;
  byte        best_transform_flag = (byte) 0;
  short       islice      = (short) (currSlice->slice_type == I_SLICE);
  short       bslice      = (short) (currSlice->slice_type == B_SLICE);
  short       pslice      = (short) ((currSlice->slice_type == P_SLICE) || (currSlice->slice_type == SP_SLICE));
  short       intra       = (short) (islice || (currSlice->slice_type == SI_SLICE) || (pslice && currMB->mb_y == p_Vid->mb_y_upd && p_Vid->mb_y_upd!=p_Vid->mb_y_intra));
  int         lambda_mf[3];
  Block8x8Info *b8x8info   = p_Vid->b8x8info;
  //int         mb_available[3] = { 1, 1, 1};

  char   **ipredmodes = p_Vid->ipredmode;
  MotionVector *allmvs = (currSlice->slice_type == I_SLICE || (currSlice->slice_type == SI_SLICE)) ? NULL: &currSlice->all_mv[0][0][0][0][0];
  int     ****i4p;  //for non-RD-opt. mode
  imgpel  **mb_pred = currSlice->mb_pred[0];

  byte tmp_8x8_flag, tmp_no_mbpart;

  BestMode    md_best;
  Info8x8 best = init_info_8x8_struct();

  init_md_best(&md_best);
  
  get_mem3Dpel(&temp_img, 3, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
  
  intra |= RandomIntra (p_Vid, currMB->mbAddrX);    // Forced Pseudo-Random Intra
#if (MVC_EXTENSION_ENABLE)
  if(p_Inp->num_of_views==2)
    intra |= (currSlice->num_ref_idx_active[LIST_0]==0 && currSlice->num_ref_idx_active[LIST_1]==0);  // force intra if no ref available
#endif

  //===== Setup Macroblock encoding parameters =====
  init_enc_mb_params(currMB, &enc_mb, intra);
  if (p_Inp->AdaptiveRounding)
  {
    reset_adaptive_rounding(p_Vid);
  }

  if (currSlice->mb_aff_frame_flag)
  {
    reset_mb_nz_coeff(p_Vid, currMB->mbAddrX);
  }

  //=====   S T O R E   C O D I N G   S T A T E   =====
  //---------------------------------------------------
  currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_cm);

  if (!intra)
  {
    //===== set direct motion vectors =====
    currMB->best_mode = 10;  // let us set best_mode to an intra mode to avoid possible bug with RDOQ
    if (bslice && enc_mb.valid[0])
    {
      currSlice->Get_Direct_Motion_Vectors (currMB);
    }

    if (p_Inp->CtxAdptLagrangeMult == 1)
    {
      get_initial_mb16x16_cost(currMB);
    }

    //===== MOTION ESTIMATION FOR 16x16, 16x8, 8x16 BLOCKS =====
    for (mode = 1; mode < 4; mode++)
    {
      best.bipred = 0;
      best.mode = (char) mode;
      b8x8info->best[mode][0].bipred = 0;
      if (enc_mb.valid[mode])
      {
        for (cost=0, block=0; block<(mode==1?1:2); block++)
        {
          update_lambda_costs(currMB, &enc_mb, lambda_mf);
          PartitionMotionSearch (currMB, mode, block, lambda_mf);

          //--- set 4x4 block indizes (for getting MV) ---
          j = (block==1 && mode==2 ? 2 : 0);

          //--- get cost and reference frame for List 0 prediction ---
          bmcost[LIST_0] = DISTBLK_MAX;
          list_prediction_cost(currMB, LIST_0, block, mode, &enc_mb, bmcost, best.ref);

          if (bslice)
          {
            //--- get cost and reference frame for List 1 prediction ---
            bmcost[LIST_1] = DISTBLK_MAX;
            list_prediction_cost(currMB, LIST_1, block, mode, &enc_mb, bmcost, best.ref);

            // Compute bipredictive cost between best list 0 and best list 1 references
            list_prediction_cost(currMB, BI_PRED, block, mode, &enc_mb, bmcost, best.ref);

            // currently Bi predictive ME is only supported for modes 1, 2, 3 and ref 0
            if (is_bipred_enabled(p_Vid, mode))
            {
              list_prediction_cost(currMB, BI_PRED_L0, block, mode, &enc_mb, bmcost, 0);
              list_prediction_cost(currMB, BI_PRED_L1, block, mode, &enc_mb, bmcost, 0);
            }
            else
            {
              bmcost[BI_PRED_L0] = DISTBLK_MAX;
              bmcost[BI_PRED_L1] = DISTBLK_MAX;
            }

            // Determine prediction list based on mode cost
            determine_prediction_list(bmcost, &best, &cost);
          }
          else // if (bslice)
          {
            best.pdir  = 0;
            cost      += bmcost[LIST_0];
          }

          assign_enc_picture_params(currMB, mode, &best, 2 * block);

          //----- set reference frame and direction parameters -----
          set_block8x8_info(b8x8info, mode, block, &best);

          //--- set reference frames and motion vectors ---
          if (mode>1 && block==0)
            currSlice->set_ref_and_motion_vectors (currMB, motion, &best, block);
        } // for (block=0; block<(mode==1?1:2); block++)

        currMB->luma_transform_size_8x8_flag = FALSE;
        if (p_Inp->Transform8x8Mode) //for inter rd-off, set 8x8 to do 8x8 transform
        {
          currSlice->set_modes_and_refs_for_blocks(currMB, (short) mode);
          currMB->luma_transform_size_8x8_flag = (byte) transform_decision(currMB, -1, &cost);
        }

        if (cost < min_cost)
        {
          currMB->best_mode = (short) mode;
          min_cost  = cost;
          best_transform_flag = currMB->luma_transform_size_8x8_flag;

          if (p_Inp->CtxAdptLagrangeMult == 1)
          {
            adjust_mb16x16_cost(currMB, cost);
          }
        }
      } // if (enc_mb.valid[mode])
    } // for (mode=1; mode<4; mode++)

    if (enc_mb.valid[P8x8])
    {
      //===== store coding state of macroblock =====
      currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_mb);
      memset( currSlice->cofAC[0][0][0], 0, 2080 * sizeof(int)); // 4 * 4 * 2 * 65

      currMB->valid_8x8 = FALSE;

      if (p_Inp->Transform8x8Mode)
      {
        ResetRD8x8Data(p_Vid, p_RDO->tr8x8);
        //===========================================================
        // Check 8x8 partition with transform size 8x8
        //===========================================================
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
        for (cost_direct =  0, block = 0; block < 4; block++)
        {
          submacroblock_mode_decision_low(currMB, &enc_mb, p_RDO->tr8x8, p_RDO->cofAC8x8ts[block], 
            &have_direct, block, &cost_direct, &cost, &cost8x8_direct, 1);

          set_subblock8x8_info(b8x8info, P8x8, block, p_RDO->tr8x8);
        }

        currMB->luma_transform_size_8x8_flag = FALSE; //switch to 4x4 transform size
      }// if (p_Inp->Transform8x8Mode)


      if (p_Inp->Transform8x8Mode != 2)
      {
        ResetRD8x8Data(p_Vid, p_RDO->tr4x4);
        //=================================================================
        // Check 8x8, 8x4, 4x8 and 4x4 partitions with transform size 4x4
        //=================================================================
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
        for (cost_direct = 0, block=0; block<4; block++)
        {
          submacroblock_mode_decision_low(currMB, &enc_mb, p_RDO->tr4x4, p_RDO->coefAC8x8[block],
            &have_direct, block, &cost_direct, &cost, &cost8x8_direct, 0);

          set_subblock8x8_info(b8x8info, P8x8, block, p_RDO->tr4x4);
        }
      }// if (p_Inp->Transform8x8Mode != 2)

      if (p_Inp->RCEnable)
        rc_store_diff(currSlice->diffy, &p_Vid->pCurImg[currMB->opix_y], currMB->pix_x, mb_pred);

      //check cost for P8x8 for non-rdopt mode
      if (((p_Inp->Transform8x8Mode < 2) && (p_RDO->tr4x4->mb_p8x8_cost < min_cost)) || 
        ((p_Inp->Transform8x8Mode >  0) && (p_RDO->tr8x8->mb_p8x8_cost < min_cost)))
      {
        currMB->best_mode = P8x8;
        if (p_Inp->Transform8x8Mode == 2)
        {
          min_cost = p_RDO->tr8x8->mb_p8x8_cost;
          currMB->luma_transform_size_8x8_flag = TRUE;
        }
        else if (p_Inp->Transform8x8Mode)
        {
          if (p_RDO->tr8x8->mb_p8x8_cost < p_RDO->tr4x4->mb_p8x8_cost)
          {
            min_cost = p_RDO->tr8x8->mb_p8x8_cost;
            currMB->luma_transform_size_8x8_flag = TRUE;
          }
          else if(p_RDO->tr4x4->mb_p8x8_cost < p_RDO->tr8x8->mb_p8x8_cost)
          {
            min_cost = p_RDO->tr4x4->mb_p8x8_cost;
            currMB->luma_transform_size_8x8_flag = FALSE;
          }
          else
          {
            if (get_best_transform_8x8(currMB) == 0)
            {
              min_cost = p_RDO->tr4x4->mb_p8x8_cost;
              currMB->luma_transform_size_8x8_flag = FALSE;
            }
            else
            {
              min_cost = p_RDO->tr8x8->mb_p8x8_cost;
              currMB->luma_transform_size_8x8_flag = TRUE;
            }
          }
        }
        else
        {
          min_cost = p_RDO->tr4x4->mb_p8x8_cost;
          currMB->luma_transform_size_8x8_flag = FALSE;
        }
      }// if ((p_RDO->tr4x4->mb_p8x8_cost < min_cost || p_RDO->tr8x8->mb_p8x8_cost < min_cost))
      p_Vid->giRDOpt_B8OnlyFlag = FALSE;
    }

    // Find a motion vector for the Skip mode
    if(pslice)
      FindSkipModeMotionVector (currMB);
  }
  else // if (!intra)
  {
    min_cost = DISTBLK_MAX;
  }


  //========= C H O O S E   B E S T   M A C R O B L O C K   M O D E =========
  //-------------------------------------------------------------------------
  tmp_8x8_flag  = currMB->luma_transform_size_8x8_flag;  //save 8x8_flag
  tmp_no_mbpart = currMB->NoMbPartLessThan8x8Flag;      //save no-part-less
  if ((p_Vid->yuv_format != YUV400) && (p_Vid->yuv_format != YUV444))
    // precompute all chroma intra prediction modes
    currSlice->intra_chroma_prediction(currMB, NULL, NULL, NULL);

  if (enc_mb.valid[0] && bslice) // check DIRECT MODE
  {
    if(have_direct)
    {
      switch(p_Inp->Transform8x8Mode)
      {
      case 1: // Mixture of 8x8 & 4x4 transform
        cost = ((cost8x8_direct < cost_direct) || !(enc_mb.valid[5] && enc_mb.valid[6] && enc_mb.valid[7]))
          ? cost8x8_direct : cost_direct;
        break;
      case 2: // 8x8 Transform only
        cost = cost8x8_direct;
        break;
      default: // 4x4 Transform only
        cost = cost_direct;
        break;
      }
    }
    else
    { //!have_direct
      cost = GetDirectCostMB (currMB);
    }
    if (cost!=DISTBLK_MAX)
    {
      cost -= weighted_cost(enc_mb.lambda_mdfp, 16);
    }

    if (cost <= min_cost)
    {
      if(p_Vid->active_sps->direct_8x8_inference_flag && p_Inp->Transform8x8Mode)
      {
        if(p_Inp->Transform8x8Mode==2)
          currMB->luma_transform_size_8x8_flag = TRUE;
        else
        {
          if(cost8x8_direct < cost_direct)
            currMB->luma_transform_size_8x8_flag = TRUE;
          else
            currMB->luma_transform_size_8x8_flag = FALSE;
        }
      }
      else
        currMB->luma_transform_size_8x8_flag = FALSE;

      //Rate control
      if (p_Inp->RCEnable)
        rc_store_diff(currSlice->diffy, &p_Vid->pCurImg[currMB->opix_y], currMB->pix_x, mb_pred);

      min_cost  = cost;
      currMB->best_mode = 0;
      tmp_8x8_flag = currMB->luma_transform_size_8x8_flag;
    }
    else
    {
      currMB->luma_transform_size_8x8_flag = tmp_8x8_flag; // restore if not best
      currMB->NoMbPartLessThan8x8Flag = tmp_no_mbpart; // restore if not best
    }
  }
  currMB->min_rdcost = min_cost;
  if (enc_mb.valid[I8MB]) // check INTRA8x8
  {
    currMB->luma_transform_size_8x8_flag = TRUE; // at this point cost will ALWAYS be less than min_cost

    currMB->mb_type = currMB->ar_mode = I8MB;
    temp_cpb = mode_decision_for_I8x8_MB (currMB, enc_mb.lambda_mdfp, &rd_cost);

    if (rd_cost <= currMB->min_rdcost) //HYU_NOTE. bug fix. 08/15/07
    {
      currMB->cbp = temp_cpb;
      if (p_Vid->P444_joined)
      {
        currSlice->curr_cbp[0] = currSlice->cmp_cbp[1];  
        currSlice->curr_cbp[1] = currSlice->cmp_cbp[2];
      }

      if(enc_mb.valid[I4MB])
      {
        //coeffs
        if (p_Inp->Transform8x8Mode != 2) 
        {
          i4p = p_RDO->cofAC; 
          p_RDO->cofAC = currSlice->cofAC; 
          currSlice->cofAC = i4p;
        }
      }

      copy_image_data_16x16(temp_img[0], &p_Vid->enc_picture->imgY[currMB->pix_y], 0, currMB->pix_x);

      if (p_Vid->P444_joined)
      {
        copy_image_data_16x16(temp_img[1], &p_Vid->enc_picture->imgUV[0][currMB->pix_y], 0, currMB->pix_x);
        copy_image_data_16x16(temp_img[2], &p_Vid->enc_picture->imgUV[1][currMB->pix_y], 0, currMB->pix_x);
      }

      //Rate control
      if (p_Inp->RCEnable)
        rc_store_diff(currSlice->diffy, &p_Vid->pCurImg[currMB->opix_y], currMB->pix_x, mb_pred);

      currMB->min_rdcost  = rd_cost; 
      currMB->best_mode = I8MB;
      tmp_8x8_flag = currMB->luma_transform_size_8x8_flag;
    }
    else
    {
      currMB->luma_transform_size_8x8_flag = tmp_8x8_flag; // restore if not best
      if (p_Vid->P444_joined)
      {
        currMB->cbp |= currSlice->curr_cbp[0];    
        currMB->cbp |= currSlice->curr_cbp[1];    
        currSlice->cmp_cbp[1] = currMB->cbp;   
        currSlice->cmp_cbp[2] = currMB->cbp;
      }
    }
  }

  if (enc_mb.valid[I4MB]) // check INTRA4x4
  {
    currMB->luma_transform_size_8x8_flag = FALSE;
    currMB->mb_type = currMB->ar_mode = I4MB;
    temp_cpb = mode_decision_for_I4x4_MB (currMB, enc_mb.lambda_mdfp, &rd_cost);
    if (rd_cost <= currMB->min_rdcost) 
    {
      currMB->cbp = temp_cpb;

      //Rate control
      if (p_Inp->RCEnable)
        rc_store_diff(currSlice->diffy, &p_Vid->pCurImg[currMB->opix_y], currMB->pix_x, mb_pred);

      currMB->min_rdcost  = rd_cost; 
      currMB->best_mode = I4MB;
      tmp_8x8_flag = currMB->luma_transform_size_8x8_flag;
    }
    else
    {
      currMB->luma_transform_size_8x8_flag = tmp_8x8_flag; // restore if not best
      if (p_Vid->P444_joined)
      {
        currMB->cbp |= currSlice->curr_cbp[0];
        currMB->cbp |= currSlice->curr_cbp[1];    
        currSlice->cmp_cbp[1] = currMB->cbp;   
        currSlice->cmp_cbp[2] = currMB->cbp;
      }
      //coeffs
      i4p = p_RDO->cofAC; 
      p_RDO->cofAC = currSlice->cofAC; 
      currSlice->cofAC=i4p;
    }
  }
  if (enc_mb.valid[I16MB]) // check INTRA16x16
  {
    rd_cost = find_best_mode_I16x16_MB (currMB, enc_mb.lambda_mdfp, currMB->min_rdcost);

    if (rd_cost < currMB->min_rdcost)
    {
      //Rate control      
      if (p_Inp->RCEnable)
        rc_store_diff(currSlice->diffy, &p_Vid->pCurImg[currMB->opix_y], currMB->pix_x, currSlice->mpr_16x16[0][(short) currMB->i16mode]);

      currMB->best_mode   = I16MB;      
      currMB->min_rdcost  = rd_cost; 
      currMB->cbp = currMB->residual_transform_quant_luma_16x16 (currMB, PLANE_Y);

      if (p_Vid->P444_joined)
      {
        select_plane(p_Vid, PLANE_U);
        currSlice->cmp_cbp[1] = currMB->residual_transform_quant_luma_16x16(currMB, PLANE_U);
        select_plane(p_Vid, PLANE_V);
        currSlice->cmp_cbp[2] = currMB->residual_transform_quant_luma_16x16(currMB, PLANE_V);   

        select_plane(p_Vid, PLANE_Y);
        currMB->cbp |= currSlice->cmp_cbp[1];    
        currMB->cbp |= currSlice->cmp_cbp[2];    
        currSlice->cmp_cbp[1] = currMB->cbp;   
        currSlice->cmp_cbp[2] = currMB->cbp;
      }

    }
    else
    {
      currMB->luma_transform_size_8x8_flag = tmp_8x8_flag; // restore
      currMB->NoMbPartLessThan8x8Flag = tmp_no_mbpart;     // restore
    }
  }

  intra1 = is_intra(currMB);

  //=====  S E T   F I N A L   M A C R O B L O C K   P A R A M E T E R S ======
  //---------------------------------------------------------------------------
  {
    //===== set parameters for chosen mode =====
    currSlice->set_modes_and_refs_for_blocks (currMB, currMB->best_mode);

    if (currMB->best_mode == P8x8)
    {
      if (currMB->luma_transform_size_8x8_flag && (p_RDO->tr8x8->cbp8x8 == 0) && p_Inp->Transform8x8Mode != 2)
        currMB->luma_transform_size_8x8_flag = FALSE;

      currSlice->set_coeff_and_recon_8x8 (currMB);

      memset(currMB->intra_pred_modes, DC_PRED, MB_BLOCK_PARTITIONS * sizeof(char));
      for (j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
        memset(&ipredmodes[j][currMB->block_x], DC_PRED, BLOCK_MULTIPLE * sizeof(char));
    }
    else
    {
      //===== set parameters for chosen mode =====
      if (currMB->best_mode == I8MB)
      {
        memcpy(currMB->intra_pred_modes,currMB->intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
        for(j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
          memcpy(&p_Vid->ipredmode[j][currMB->block_x],&p_Vid->ipredmode8x8[j][currMB->block_x], BLOCK_MULTIPLE * sizeof(char));

        //--- restore reconstruction for 8x8 transform ---
        copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], temp_img[0], currMB->pix_x, 0);

        if (p_Vid->P444_joined)
        {
          copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], temp_img[1], currMB->pix_x, 0);
          copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], temp_img[2], currMB->pix_x, 0);
        }
      }

      if ((currMB->best_mode!=I4MB)&&(currMB->best_mode != I8MB))
      {
        memset(currMB->intra_pred_modes,DC_PRED, MB_BLOCK_PARTITIONS * sizeof(char));
        for(j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
          memset(&ipredmodes[j][currMB->block_x],DC_PRED, BLOCK_MULTIPLE * sizeof(char));
        currMB->ar_mode = currMB->best_mode;

        if (currMB->best_mode!=I16MB)
        {
          if((currMB->best_mode >= 1) && (currMB->best_mode <= 3))
            currMB->luma_transform_size_8x8_flag = best_transform_flag;
          
          currSlice->luma_residual_coding(currMB);
          if (currSlice->P444_joined)
          {            
            if((currMB->cbp==0 && currSlice->cmp_cbp[1] == 0 && currSlice->cmp_cbp[2] == 0) &&(currMB->best_mode == 0))
              currMB->luma_transform_size_8x8_flag = FALSE;
          }
          else
          {            
            if((currMB->cbp==0)&&(currMB->best_mode == 0))
              currMB->luma_transform_size_8x8_flag = FALSE;
          }

          //Rate control
          if (p_Inp->RCEnable)
            rc_store_diff(currSlice->diffy, &p_Vid->pCurImg[currMB->opix_y], currMB->pix_x, mb_pred);
        }
      }
    }
    //check luma cbp for transform size flag
    if (((currMB->cbp&15) == 0) && currMB->mb_type != I4MB && currMB->mb_type != I8MB)
      currMB->luma_transform_size_8x8_flag = FALSE;

    // precompute all chroma intra prediction modes
    if ((p_Vid->yuv_format != YUV400) && (p_Vid->yuv_format != YUV444))
      currSlice->intra_chroma_prediction(currMB, NULL, NULL, NULL);

    currMB->i16offset = 0;

    if ((p_Vid->yuv_format != YUV400) && (p_Vid->yuv_format != YUV444))
      currSlice->chroma_residual_coding (currMB);

    if (currMB->best_mode == I16MB)
    {
      currMB->i16offset = I16Offset  (currMB->cbp, currMB->i16mode);
    }

    currSlice->set_motion_vectors_mb (currMB);

    //===== check for SKIP mode =====
    if(p_Vid->P444_joined)
    {
      if ((pslice) && currMB->best_mode == 1 && currMB->cbp==0 && currSlice->cmp_cbp[1] == 0 && currSlice->cmp_cbp[2] == 0 &&
        motion[currMB->block_y][currMB->block_x].ref_idx[LIST_0]    == 0 &&
        motion[currMB->block_y][currMB->block_x].mv     [LIST_0].mv_x == allmvs->mv_x &&
        motion[currMB->block_y][currMB->block_x].mv     [LIST_0].mv_y == allmvs->mv_y)
      {
        currMB->mb_type = currMB->b8x8[0].mode = currMB->b8x8[1].mode = currMB->b8x8[2].mode = currMB->b8x8[3].mode = 0;
        currMB->luma_transform_size_8x8_flag = FALSE;
      }
    }
    else if ((pslice) && currMB->best_mode == 1 && currMB->cbp==0 &&
      motion[currMB->block_y][currMB->block_x].ref_idx[LIST_0]    == 0 &&
      motion[currMB->block_y][currMB->block_x].mv     [LIST_0].mv_x == allmvs->mv_x &&
      motion[currMB->block_y][currMB->block_x].mv     [LIST_0].mv_y == allmvs->mv_y)
    {
      currMB->mb_type = currMB->b8x8[0].mode = currMB->b8x8[1].mode = currMB->b8x8[2].mode = currMB->b8x8[3].mode = 0;
      currMB->luma_transform_size_8x8_flag = FALSE;
    }

    if (currSlice->mb_aff_frame_flag || (p_Inp->UseRDOQuant && currSlice->RDOQ_QP_Num > 1))
      set_mbaff_parameters(currMB);
  }

  // Rate control
  if(p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE)
    rc_store_mad(currMB);

  //===== Decide if this MB will restrict the reference frames =====
  if (p_Inp->RestrictRef)
    update_refresh_map(currMB, intra, intra1);


  /*update adaptive rounding offset p_Inp*/
  if (p_Vid->AdaptiveRounding)
  {
    update_offset_params(currMB, currMB->best_mode, currMB->luma_transform_size_8x8_flag);
  }

  free_mem3Dpel(temp_img);
}

