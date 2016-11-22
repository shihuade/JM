/*!
***************************************************************************
* \file md_highfast.c
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
#include "ratectl.h"
#include "mode_decision.h"
#include "mode_decision_p8x8.h"
#include "fmo.h"
#include "me_umhex.h"
#include "me_umhexsmp.h"
#include "macroblock.h"
#include "md_common.h"
#include "conformance.h"
#include "vlc.h"
#include "rdopt.h"
#include "mv_search.h"

/*!
*************************************************************************************
* \brief
*    Fast intra decision
*************************************************************************************
*/

static void fast_mode_intra_decision(Macroblock *currMB, short *intra_skip)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int i;
  int mb_available_up, mb_available_left;
  long SBE;
  double AR = 0, ABE = 0;
  PixelPos up;       //!< pixel position p(0,-1)
  PixelPos left[2];  //!< pixel positions p(-1, -1..0)

  p_Vid->getNeighbour(currMB,  0 ,  -1 , p_Vid->mb_size[IS_LUMA], &up);
  p_Vid->getNeighbour(currMB, -1 ,  -1 , p_Vid->mb_size[IS_LUMA], &left[0]);
  p_Vid->getNeighbour(currMB, -1 ,   0 , p_Vid->mb_size[IS_LUMA], &left[1]);

  mb_available_up       = up.available;
  mb_available_left     = left[1].available;

  AR=(1.0/384)*currMB->min_rate;

  SBE = 0;

  if( (currMB->mb_y != (int)p_Vid->FrameHeightInMbs-1) && (currMB->mb_x != (int)p_Vid->PicWidthInMbs-1) && mb_available_left && mb_available_up)
  {
    for(i = 0; i < MB_BLOCK_SIZE; i++)
    {
      SBE += iabs(p_Vid->pCurImg[currMB->opix_y][currMB->pix_x + i] - p_Vid->enc_picture->imgY[currMB->pix_y - 1][currMB->pix_x + i]);
      SBE += iabs(p_Vid->pCurImg[currMB->opix_y + i][currMB->pix_x] - p_Vid->enc_picture->imgY[currMB->pix_y + i][currMB->pix_x - 1]);
    }

    for(i = 0; i < 8; i++)
    {
      SBE += iabs(p_Vid->pImgOrg[1][currMB->opix_c_y][currMB->pix_c_x + i] - p_Vid->enc_picture->imgUV[0][currMB->pix_c_y - 1][currMB->pix_c_x + i]);
      SBE += iabs(p_Vid->pImgOrg[1][currMB->opix_c_y+i][currMB->pix_c_x] - p_Vid->enc_picture->imgUV[0][currMB->pix_c_y + i][currMB->pix_c_x - 1]);
      SBE += iabs(p_Vid->pImgOrg[2][currMB->opix_c_y][currMB->pix_c_x + i] - p_Vid->enc_picture->imgUV[1][currMB->pix_c_y - 1][currMB->pix_c_x + i]);
      SBE += iabs(p_Vid->pImgOrg[2][currMB->opix_c_y+i][currMB->pix_c_x] - p_Vid->enc_picture->imgUV[1][currMB->pix_c_y + i][currMB->pix_c_x - 1]);
    }
    ABE = 1.0/64 * SBE;
  }
  else  // Image boundary
  {
    ABE = 0;
  }

  if(AR <= ABE)
  {
    *intra_skip = 1;
  }
}

/*!
*************************************************************************************
* \brief
*    Mode Decision for a macroblock
*************************************************************************************
*/
void encode_one_macroblock_highfast (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  int         max_index;
  int         block, index, mode, i, j;
  RD_PARAMS   enc_mb;
  distblk       max_rdcost = DISTBLK_MAX;
  distblk       bmcost[5] = {DISTBLK_MAX};
  distblk       cost=0;
  distblk       min_cost = DISTBLK_MAX;
  int         intra1 = 0;
  int         mb_available[3];

  short       islice      = (short) (currSlice->slice_type == I_SLICE);
  short       bslice      = (short) (currSlice->slice_type == B_SLICE);
  short       pslice      = (short) ((currSlice->slice_type == P_SLICE) || (currSlice->slice_type == SP_SLICE));
  short       intra       = (short) ((currSlice->slice_type == I_SLICE) || (currSlice->slice_type == SI_SLICE) || (pslice && currMB->mb_y == p_Vid->mb_y_upd && p_Vid->mb_y_upd != p_Vid->mb_y_intra));
  int         lambda_mf[3];

  imgpel    **mb_pred  = currSlice->mb_pred[0];
  Block8x8Info *b8x8info = p_Vid->b8x8info;

  char       chroma_pred_mode_range[2];


  MotionVector *allmvs = (currSlice->slice_type == I_SLICE) ? NULL: &currSlice->all_mv[0][0][0][0][0];

  // Fast Mode Decision
  short inter_skip = 0, intra_skip = 0;
  int mode16 = 0;
  distblk RDCost16 = DISTBLK_MAX;
  BestMode    md_best;
  Info8x8     best;

  init_md_best(&md_best);

  // Init best (need to create simple function)
  best.pdir = 0;
  best.bipred = 0;
  best.ref[LIST_0] = 0;
  best.ref[LIST_1] = -1;

  intra |= RandomIntra (p_Vid, currMB->mbAddrX);    // Forced Pseudo-Random Intra

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
    if (bslice && enc_mb.valid[0])
    {
      currSlice->Get_Direct_Motion_Vectors (currMB);
      currMB->best_mode = 0;
      currMB->c_ipred_mode=DC_PRED_8;
      currMB->min_rdcost = max_rdcost;
      compute_mode_RD_cost(currMB, &enc_mb, 0, &inter_skip);
    }
    if (p_Inp->CtxAdptLagrangeMult == 1)
    {
      get_initial_mb16x16_cost(currMB);
    }

    //===== MOTION ESTIMATION FOR 16x16, 16x8, 8x16 BLOCKS =====
    for (mode = 1; mode < 4; mode++)
    {
      best.mode = (char) mode;
      best.bipred = 0;
      b8x8info->best[mode][0].bipred = 0;

      if (enc_mb.valid[mode] && !inter_skip)
      {
        for (cost=0, block=0; block<(mode==1?1:2); block++)
        {
          update_lambda_costs(currMB, &enc_mb, lambda_mf);
          PartitionMotionSearch (currMB, mode, block, lambda_mf);

          //--- set 4x4 block indizes (for getting MV) ---
          j = (block==1 && mode==2 ? 2 : 0);
          i = (block==1 && mode==3 ? 2 : 0);

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
              get_bipred_cost(currMB, mode, block, i, j, &best, &enc_mb, bmcost);
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
            best.pdir = 0;
            cost      += bmcost[LIST_0];
          }

          assign_enc_picture_params(currMB, mode, &best, 2 * block);

          //----- set reference frame and direction parameters -----
          set_block8x8_info(b8x8info, mode, block, &best);

          //--- set reference frames and motion vectors ---
          if (mode>1 && block == 0)
            currSlice->set_ref_and_motion_vectors (currMB, motion, &best, block);
        } // for (block=0; block<(mode==1?1:2); block++)


        if(mode == 1)
        {
          if(pslice)
            currMB->min_rdcost = max_rdcost;

          currMB->c_ipred_mode=DC_PRED_8;
          compute_mode_RD_cost(currMB, &enc_mb, (short) mode, &inter_skip);

          if(pslice)
          {
            // Get SKIP motion vector and compare SKIP_MV with best motion vector of 16x16
            FindSkipModeMotionVector (currMB);

            if(p_Inp->EarlySkipEnable)
            {
              //===== check for SKIP mode =====
              if ( currMB->cbp==0 && motion[currMB->block_y][currMB->block_x].ref_idx[LIST_0]==0 &&
                motion[currMB->block_y][currMB->block_x].mv[LIST_0].mv_x == allmvs->mv_x &&
                motion[currMB->block_y][currMB->block_x].mv[LIST_0].mv_y == allmvs->mv_y)
              {
                inter_skip = 1;
                currMB->best_mode = 0;
              }
            } // if(p_Inp->EarlySkipEnable)
          }

          // store variables.
          RDCost16 = currMB->min_rdcost;
          mode16 = currMB->best_mode;
        } // if(mode == 1)

        if ((!inter_skip) && (cost < min_cost))
        {
          md_best.mode = (byte) mode;
          md_best.cost = cost;
          currMB->best_mode = (short) mode;
          min_cost  = cost;
          if (p_Inp->CtxAdptLagrangeMult == 1)
          {
            adjust_mb16x16_cost(currMB, cost);
          }
        }
      } // if (enc_mb.valid[mode])
    } // for (mode=1; mode<4; mode++)

    if ((!inter_skip) && enc_mb.valid[P8x8])
    {
      currMB->valid_8x8 = FALSE;

      if (p_Inp->Transform8x8Mode)
      {
        ResetRD8x8Data(p_Vid, p_RDO->tr8x8);
        currMB->luma_transform_size_8x8_flag = TRUE; //switch to 8x8 transform size
        //===========================================================
        // Check 8x8 partition with transform size 8x8
        //===========================================================
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
        for (block = 0; block < 4; block++)
        {
          currSlice->submacroblock_mode_decision(currMB, &enc_mb, p_RDO->tr8x8, p_RDO->cofAC8x8ts[block], block, &cost);
          if(!currMB->valid_8x8)
            break;
          set_subblock8x8_info(b8x8info, P8x8, block, p_RDO->tr8x8);
        }
      }// if (p_Inp->Transform8x8Mode)

      currMB->valid_4x4 = FALSE;
      if (p_Inp->Transform8x8Mode != 2)
      {
        currMB->luma_transform_size_8x8_flag = FALSE; //switch to 8x8 transform size
        ResetRD8x8Data(p_Vid, p_RDO->tr4x4);
        //=================================================================
        // Check 8x8, 8x4, 4x8 and 4x4 partitions with transform size 4x4
        //=================================================================
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
        for (block = 0; block < 4; block++)
        {
          currSlice->submacroblock_mode_decision(currMB, &enc_mb, p_RDO->tr4x4, p_RDO->coefAC8x8[block], block, &cost);
          if(!currMB->valid_4x4)
            break;
          set_subblock8x8_info(b8x8info, P8x8, block, p_RDO->tr4x4);
        }
      }// if (p_Inp->Transform8x8Mode != 2)

      if (p_Inp->RCEnable)
        rc_store_diff(currSlice->diffy, &p_Vid->pCurImg[currMB->opix_y], currMB->pix_x, mb_pred);

      p_Vid->giRDOpt_B8OnlyFlag = FALSE;
    }
  }
  else // if (!intra)
  {
    min_cost = DISTBLK_MAX;
  }

  //========= C H O O S E   B E S T   M A C R O B L O C K   M O D E =========
  //-------------------------------------------------------------------------
  {
    if (!inter_skip)
    {
      if(currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE)
      {
        currMB->min_rdcost = RDCost16;
        currMB->best_mode  = (short) mode16;
      }
      else
        currMB->min_rdcost = max_rdcost;

      // if Fast High mode, compute  inter modes separate process for inter/intra
      max_index = ((!intra && p_Inp->SelectiveIntraEnable ) ? 5 : 9);

      // Set Chroma mode
      set_chroma_pred_mode(currMB, enc_mb, mb_available, chroma_pred_mode_range);

      //========= C H O O S E   B E S T   M A C R O B L O C K   M O D E =========
      //-------------------------------------------------------------------------

      for (currMB->c_ipred_mode = chroma_pred_mode_range[0]; currMB->c_ipred_mode<=chroma_pred_mode_range[1]; currMB->c_ipred_mode++)
      {
        // bypass if c_ipred_mode is not allowed
        if ( (p_Vid->yuv_format != YUV400) &&
          (  ((!intra || !p_Inp->IntraDisableInterOnly) && p_Inp->ChromaIntraDisable == 1 && currMB->c_ipred_mode!=DC_PRED_8)
          || (currMB->c_ipred_mode == VERT_PRED_8 && !mb_available[0]) 
          || (currMB->c_ipred_mode == HOR_PRED_8  && !mb_available[1]) 
          || (currMB->c_ipred_mode == PLANE_8     && (!mb_available[1] || !mb_available[0] || !mb_available[2]))))
          continue;

        //===== GET BEST MACROBLOCK MODE =====
        for (index=0; index < max_index; index++)
        {
          mode = mb_mode_table[index];
          if (mode == 1)
            continue;
          if (enc_mb.valid[mode])
          {
            if (p_Vid->yuv_format != YUV400)
            {           
              currMB->i16mode = 0; 
            }

            // Skip intra modes in inter slices if best mode is inter <P8x8 with cbp equal to 0    
            if (currSlice->P444_joined)
            {
              if (p_Inp->SkipIntraInInterSlices && !intra && mode >= I16MB 
                && currMB->best_mode <=3 && currMB->best_cbp == 0 && currSlice->cmp_cbp[1] == 0 && currSlice->cmp_cbp[2] == 0 && (currMB->min_rdcost < weighted_cost(enc_mb.lambda_mdfp, 5)))
                continue;
            }
            else
            {
              if (p_Inp->SkipIntraInInterSlices && !intra && mode >= I4MB && currMB->best_mode <=3 && currMB->best_cbp == 0 && (currMB->min_rdcost < weighted_cost(enc_mb.lambda_mdfp,5)))
                continue;
            }

            compute_mode_RD_cost(currMB, &enc_mb, (short) mode, &inter_skip);

          }
        }// for (index=0; index<max_index; index++)
      }// for (currMB->c_ipred_mode=DC_PRED_8; currMB->c_ipred_mode<=chroma_pred_mode_range[1]; currMB->c_ipred_mode++)                     

      // Selective Intra Coding
      if((currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE) && p_Inp->SelectiveIntraEnable && !is_FREXT_profile(p_Inp->ProfileIDC))
      {
        fast_mode_intra_decision(currMB, &intra_skip);

        if(!intra_skip)
        {
          // Set Chroma mode
          set_chroma_pred_mode(currMB, enc_mb, mb_available, chroma_pred_mode_range);

          max_index = 9;

          for (currMB->c_ipred_mode=chroma_pred_mode_range[0]; currMB->c_ipred_mode<=chroma_pred_mode_range[1]; currMB->c_ipred_mode++)
          {

            // bypass if c_ipred_mode is not allowed
            if ( (p_Vid->yuv_format != YUV400) &&
              (  ((!intra || !p_Inp->IntraDisableInterOnly) && p_Inp->ChromaIntraDisable == 1 && currMB->c_ipred_mode!=DC_PRED_8)
              || (currMB->c_ipred_mode == VERT_PRED_8 && !mb_available[0])
              || (currMB->c_ipred_mode == HOR_PRED_8  && !mb_available[1])
              || (currMB->c_ipred_mode == PLANE_8     && (!mb_available[0] || !mb_available[1]|| !mb_available[2]))))
              continue;

            //===== GET BEST MACROBLOCK MODE =====
            for (index = 5; index < max_index; index++)
            {
              mode = mb_mode_table[index];

              // Skip intra modes in inter slices if best mode is inter <P8x8 with cbp equal to 0
              if (p_Inp->SkipIntraInInterSlices && !intra && mode >= I4MB && currMB->best_mode <=3 && currMB->best_cbp == 0)
                continue;

              if (p_Vid->yuv_format != YUV400)
              {
                currMB->i16mode = 0;
                // RDcost of mode 1 in P-slice and mode 0, 1 in B-slice are already available
                if(((bslice && mode == 0) || (!islice && mode == 1)))
                  continue;
              }

              if (enc_mb.valid[mode])
                compute_mode_RD_cost(currMB, &enc_mb, (short) mode, &inter_skip);
            } // for (index = 5; index < max_index; index++)
          }
        }
      }
    }
    restore_nz_coeff(currMB);

  }

  intra1 = is_intra(currMB);

  //=====  S E T   F I N A L   M A C R O B L O C K   P A R A M E T E R S ======
  //---------------------------------------------------------------------------
  update_qp_cbp_tmp(currMB, p_RDO->cbp);
  currSlice->set_stored_mb_parameters (currMB);

  // Rate control
  if(p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE)
    rc_store_mad(currMB);

  //===== Decide if this MB will restrict the reference frames =====
  if (p_Inp->RestrictRef)
    update_refresh_map(currMB, intra, intra1);

}


