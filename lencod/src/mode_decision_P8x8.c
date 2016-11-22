
/*!
 ***************************************************************************
 * \file mode_decision_8x8.c
 *
 * \brief
 *    Mode decision functions for 8x8 submacroblock
 *
 **************************************************************************
 */

#include <math.h>
#include <limits.h>
#include <float.h>

#include "global.h"
#include "image.h"
#include "mode_decision.h"
#include "macroblock.h"
#include "errdo.h"
#include "q_around.h"
#include "md_common.h"
#include "rdopt.h"


void copy_part_info(Info8x8 *b8x8, Info8x8 *part)
{
  b8x8[0] = part[0];
  b8x8[1] = part[1];
  b8x8[2] = part[2];
  b8x8[3] = part[3];
}

/*!
*************************************************************************************
* \brief
*    Mode Decision for an 8x8 sub-macroblock
*************************************************************************************
*/
void submacroblock_mode_decision_p_slice(Macroblock *currMB,
                                         RD_PARAMS *enc_mb,
                                         RD_8x8DATA *dataTr,
                                         int ****cofACtr,
                                         int block,
                                         distblk *cost)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  int transform8x8 = currMB->luma_transform_size_8x8_flag;
  int64 curr_cbp_blk;
  int j, k;
  int index;
  int mode;
  distblk min_rdcost, rdcost = 0;
  distblk min_cost8x8;
  distblk bmcost[5] = {DISTBLK_MAX};
  int cnt_nonz = 0;
  int best_cnt_nonz = 0;
  int maxindex =  (transform8x8) ? 2 : 5;
  int block_x, block_y;
  int lambda_mf[3];

  int ****fadjust = transform8x8? p_Vid->ARCofAdj8x8 : p_Vid->ARCofAdj4x4;

  //--- set coordinates ---
  int j0 = ((block>>1)<<3);
  int j1 = (j0>>2);
  int i0 = ((block&0x01)<<3);
  int i1 = (i0>>2);

  Boolean valid_8x8 = FALSE;
  Boolean stored_state_8x8 = FALSE;

#ifdef BEST_NZ_COEFF
  int best_nz_coeff[2][2];
#endif

  Info8x8 *partition = &(dataTr->part[block]);
  Info8x8 best = init_info_8x8_struct();

  *partition = best;
  if (transform8x8)
    currMB->valid_8x8 = FALSE;
  else
    currMB->valid_4x4 = FALSE;

#ifdef BEST_NZ_COEFF
  for(j = 0; j <= 1; j++)
  {
    for(i = 0; i <= 1; i++)
      best_nz_coeff[i][j] = p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1 + j] = 0;
  }
#endif

  if(p_Inp->subMBCodingState == 2)
    currSlice->store_coding_state(currMB, currSlice->p_RDO->cs_tmp);

  //=====  LOOP OVER POSSIBLE CODING MODES FOR 8x8 SUB-PARTITION  =====
  for (min_cost8x8 = DISTBLK_MAX, min_rdcost = DISTBLK_MAX, index = 1; index < maxindex; index++)
  {

    mode = b8_mode_table[index];

    best.mode = (char) mode;
    *cost = 0;

    if (enc_mb->valid[mode] && (!(transform8x8 == 1 && mode > 4)) && (transform8x8 == 0 || mode != 0 ))
    {
      if (transform8x8)
      {
        currMB->valid_8x8 = TRUE;
      }
      else
        currMB->valid_4x4 = TRUE;
      valid_8x8 = TRUE;
      curr_cbp_blk = 0;

      {
        short b_ref;

        //======= motion estimation for all reference frames ========
        //-----------------------------------------------------------
        memcpy(lambda_mf, enc_mb->lambda_mf, 3 * sizeof(int));

        if (p_Inp->CtxAdptLagrangeMult == 1)
        {
          RDOPTStructure *p_RDO = currSlice->p_RDO;
          lambda_mf[F_PEL] = (int)(lambda_mf[F_PEL] * p_RDO->lambda_mf_factor);
          lambda_mf[H_PEL] = (int)(lambda_mf[H_PEL] * p_RDO->lambda_mf_factor);
          lambda_mf[Q_PEL] = (int)(lambda_mf[Q_PEL] * p_RDO->lambda_mf_factor);
        }

        SubPartitionMotionSearch (currMB, mode, block, lambda_mf);

        //--- get cost and reference frame for LIST 0 prediction ---
        bmcost[LIST_0] = DISTBLK_MAX;
        list_prediction_cost(currMB, LIST_0, block, mode, enc_mb, bmcost, best.ref);

        //store LIST 0 reference index for every block
        block_x = currMB->block_x + (block & 0x01)*2;
        block_y = currMB->block_y + (block & 0x02);
        b_ref = best.ref[LIST_0];        

        motion[block_y    ][block_x    ].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
        motion[block_y    ][block_x    ].mv      [LIST_0] = currSlice->all_mv[LIST_0][b_ref][mode][j1    ][i1    ];
        motion[block_y    ][block_x    ].ref_idx [LIST_0] = (char) b_ref;
        motion[block_y    ][block_x + 1].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
        motion[block_y    ][block_x + 1].mv      [LIST_0] = currSlice->all_mv[LIST_0][b_ref][mode][j1    ][i1 + 1];
        motion[block_y    ][block_x + 1].ref_idx [LIST_0] = (char) b_ref;
        motion[block_y + 1][block_x    ].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
        motion[block_y + 1][block_x    ].mv      [LIST_0] = currSlice->all_mv[LIST_0][b_ref][mode][j1 + 1][i1    ];
        motion[block_y + 1][block_x    ].ref_idx [LIST_0] = (char) b_ref;
        motion[block_y + 1][block_x + 1].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
        motion[block_y + 1][block_x + 1].mv      [LIST_0] = currSlice->all_mv[LIST_0][b_ref][mode][j1 + 1][i1 + 1];
        motion[block_y + 1][block_x + 1].ref_idx [LIST_0] = (char) b_ref;

        best.pdir = 0;
        *cost     = bmcost[LIST_0];
      } // if (mode!=0)

      //--- get and check rate-distortion cost ---
      rdcost = rdcost_for_8x8blocks (currMB, dataTr, &cnt_nonz, &curr_cbp_blk, enc_mb->lambda_mdfp, block, (short) mode, &best, min_rdcost);

      //--- set variables if best mode has changed ---
      if (rdcost < min_rdcost)
      {
        min_cost8x8              = *cost;
        min_rdcost               = rdcost;        
        *partition               = best;
        partition->mode          = (char) mode;
        currMB->b8x8[block].mode = (char) mode;

#ifdef BEST_NZ_COEFF
        if (cnt_nonz)
        {
          for(i = 0; i <= 1; i++)
          {
            best_nz_coeff[i][0]= p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1    ];
            best_nz_coeff[i][1]= p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1 + 1];
          }
        }
        else
        {
          for(i = 0; i <= 1; i++)
          {
            best_nz_coeff[i][0]= 0;
            best_nz_coeff[i][1]= 0;
          }
        }
#endif

        //--- store number of nonzero coefficients ---
        best_cnt_nonz  = cnt_nonz;

        //--- store block cbp ---
        dataTr->cbp_blk8x8 &= (~(0x33 << (((block>>1)<<3)+((block & 0x01)<<1)))); // delete bits for block
        dataTr->cbp_blk8x8 |= curr_cbp_blk;

        //--- store coefficients ---
        memcpy(&cofACtr[0][0][0][0],&currSlice->cofAC[block][0][0][0], 4 * 2 * 65 * sizeof(int));

        if( currSlice->P444_joined ) 
        {
          //--- store coefficients ---
          memcpy(&cofACtr[1][0][0][0],&currSlice->cofAC[block + 4][0][0][0], 4 * 2 * 65 * sizeof(int));
          memcpy(&cofACtr[2][0][0][0],&currSlice->cofAC[block + 8][0][0][0], 4 * 2 * 65 * sizeof(int));
        }

        //--- store reconstruction and prediction ---
        copy_image_data_8x8(&dataTr->rec_mbY8x8[j0], &p_Vid->enc_picture->imgY[currMB->pix_y + j0], i0, currMB->pix_x + i0);
        copy_image_data_8x8(&dataTr->mpr8x8[j0], &currSlice->mb_pred[0][j0], i0, i0);

        if (p_Inp->rdopt == 3)
        {
          errdo_store_best_b8x8(currMB, transform8x8, block);
        }

        if(currSlice->slice_type == SP_SLICE)
        {
          for (j = j0; j < j0 + BLOCK_SIZE_8x8; j++)
          {
            memcpy(&dataTr->lrec[j][i0],&p_Vid->lrec[currMB->pix_y + j][currMB->pix_x + i0], BLOCK_SIZE_8x8 * sizeof(int));
          }
        }

        if(currSlice->P444_joined) 
        {
          copy_image_data_8x8(&dataTr->rec_mb8x8_cr[0][j0], &p_Vid->enc_picture->imgUV[0][currMB->pix_y + j0], i0, currMB->pix_x + i0);
          copy_image_data_8x8(&dataTr->mpr8x8CbCr[0][j0], &currSlice->mb_pred[1][j0], i0, i0);

          copy_image_data_8x8(&dataTr->rec_mb8x8_cr[1][j0], &p_Vid->enc_picture->imgUV[1][currMB->pix_y + j0], i0, currMB->pix_x + i0);
          copy_image_data_8x8(&dataTr->mpr8x8CbCr[1][j0], &currSlice->mb_pred[2][j0], i0, i0);
        }

        //--- store best 8x8 coding state ---
        if (block < 3)
        {
          currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_b8);
          stored_state_8x8 = TRUE;
        }
      } // if (rdcost <= min_rdcost)

      //--- re-set coding state as it was before coding with current mode was performed ---
      if (index != maxindex - 1)
      {
        if(p_Inp->subMBCodingState == 1)
          currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);
        else if(p_Inp->subMBCodingState == 2)
          currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_tmp);
      }
    } // if ((enc_mb->valid[mode] && (transform8x8 == 0 || mode != 0 || (mode == 0 && active_sps->direct_8x8_inference_flag)))
  } // for (min_rdcost=1e30, index = 1; index<6; index++)

  if (valid_8x8 == TRUE)
  {
#ifdef BEST_NZ_COEFF
    for(i = 0; i <= 1; i++)  
    {
      for(j = 0; j <= 1; j++)
        p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1 + j] = best_nz_coeff[i][j];
    }
#endif

    if (!transform8x8)
    {
      if (min_cost8x8 != DISTBLK_MAX)
        dataTr->mb_p8x8_cost += min_cost8x8;
      else
        dataTr->mb_p8x8_cost = DISTBLK_MAX;
    }

    //----- set cbp and count of nonzero coefficients ---
    if (best_cnt_nonz)
    {
      dataTr->cbp8x8       |= (1 << block);
      dataTr->cnt_nonz_8x8 += best_cnt_nonz;
    }

    if (!transform8x8)
    {
      if (block < 3)
      {
        //===== re-set reconstructed block =====
        j0   = 8*(block >> 1);
        i0   = 8*(block & 0x01);

        // need to double check code since original did not use i0
        copy_image_data_8x8(&p_Vid->enc_picture->imgY[currMB->pix_y + j0], &dataTr->rec_mbY8x8[j0], currMB->pix_x + i0, i0);

        if (p_Inp->rdopt == 3)
        {
          errdo_get_best_b8x8(currMB, transform8x8, block);
        }

        if(currSlice->slice_type == SP_SLICE)
        {
          for (j = j0; j < j0 + BLOCK_SIZE_8x8; j++)
          {
            memcpy(&p_Vid->lrec[currMB->pix_y + j][currMB->pix_x], dataTr->lrec[j], 2 * BLOCK_SIZE * sizeof(int)); // reset the coefficients for SP slice
          }
        }

        if(currSlice->P444_joined) 
        {

          for (k=0; k<2; k++)
          {
            copy_image_data_8x8(&p_Vid->enc_picture->imgUV[k][currMB->pix_y + j0], &dataTr->rec_mb8x8_cr[k][j0], currMB->pix_x + i0, i0);
          }
        }
      } // if (block<3)
    }
    else
    {
      //======= save motion data for 8x8 partition for transform size 8x8 ========    
      currSlice->store_8x8_motion_vectors(currSlice, 0, block, partition);
    }

    //===== set motion vectors and reference frames (prediction) =====    
    currSlice->set_ref_and_motion_vectors (currMB, motion, partition, block);

    //===== set the coding state after current block =====
    //if (transform8x8 == 0 || block < 3)
    if (stored_state_8x8 == TRUE)
      currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_b8);
    else
    {
      currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);
      update_adaptive_rounding_8x8(p_Vid, p_Inp, dataTr, fadjust);
    }
  }
}

/*!
*************************************************************************************
* \brief
*    Mode Decision for an 8x8 sub-macroblock
*************************************************************************************
*/
void submacroblock_mode_decision_b_slice(Macroblock *currMB,
                                         RD_PARAMS *enc_mb,
                                         RD_8x8DATA *dataTr,
                                         int ****cofACtr,
                                         int block,
                                         distblk *cost)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  int transform8x8 = currMB->luma_transform_size_8x8_flag;
  int maxindex =  (transform8x8) ? 2 : 5;
  int64 curr_cbp_blk;
  int k;
  int index;
  int mode;
  distblk min_rdcost, rdcost = 0;
  distblk min_cost8x8;
  distblk bmcost[5] = {DISTBLK_MAX};
  int cnt_nonz = 0;
  int best_cnt_nonz = 0;  
  int block_x, block_y;
  int lambda_mf[3];

  int ****fadjust = transform8x8? p_Vid->ARCofAdj8x8 : p_Vid->ARCofAdj4x4;

  //--- set coordinates ---
  int j0 = ((block>>1)<<3);
  int j1 = (j0>>2);
  int i0 = ((block&0x01)<<3);
  int i1 = (i0>>2);

  Boolean valid_8x8 = FALSE;
  Boolean stored_state_8x8 = FALSE;

#ifdef BEST_NZ_COEFF
  int best_nz_coeff[2][2];
#endif


  Info8x8 *partition = &(dataTr->part[block]);
  Info8x8 best = init_info_8x8_struct();  
  *partition = best;
  if (transform8x8)
    currMB->valid_8x8 = FALSE;
  else
    currMB->valid_4x4 = FALSE;

#ifdef BEST_NZ_COEFF
  for(j = 0; j <= 1; j++)
  {
    for(i = 0; i <= 1; i++)
      best_nz_coeff[i][j] = p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1 + j] = 0;
  }
#endif

  if(p_Inp->subMBCodingState == 2)
    currSlice->store_coding_state(currMB, currSlice->p_RDO->cs_tmp);

  //=====  LOOP OVER POSSIBLE CODING MODES FOR 8x8 SUB-PARTITION  =====
  for (min_cost8x8 = DISTBLK_MAX, min_rdcost = DISTBLK_MAX, index = 0; index < maxindex; index++)
  {

    mode = b8_mode_table[index];

    best.mode = (char) mode;
    *cost = 0;

    if (enc_mb->valid[mode] && (!(transform8x8 == 1 && mode > 4)) && (transform8x8 == 0 || mode != 0 || (mode == 0 && p_Vid->active_sps->direct_8x8_inference_flag)))
    {
      if (transform8x8)
      {
        currMB->valid_8x8 = TRUE;
      }
      else
        currMB->valid_4x4 = TRUE;

      valid_8x8 = TRUE;
      curr_cbp_blk = 0;

      if (mode==0)  //--- Direct8x8 Mode ---
      {        
        block_x = currMB->block_x + (block & 0x01)*2;
        block_y = currMB->block_y + (block & 0x02);
        best.ref[LIST_0] = currSlice->direct_ref_idx[block_y][block_x][LIST_0];
        best.ref[LIST_1] = currSlice->direct_ref_idx[block_y][block_x][LIST_1];
        best.pdir        = currSlice->direct_pdir[block_y][block_x];
      } // if (mode==0)
      else
      {
        short b_ref;

        //======= motion estimation for all reference frames ========
        //-----------------------------------------------------------
        memcpy(lambda_mf, enc_mb->lambda_mf, 3 * sizeof(int));

        if (p_Inp->CtxAdptLagrangeMult == 1)
        {
          RDOPTStructure *p_RDO = currSlice->p_RDO;
          lambda_mf[F_PEL] = (int)(lambda_mf[F_PEL] * p_RDO->lambda_mf_factor);
          lambda_mf[H_PEL] = (int)(lambda_mf[H_PEL] * p_RDO->lambda_mf_factor);
          lambda_mf[Q_PEL] = (int)(lambda_mf[Q_PEL] * p_RDO->lambda_mf_factor);
        }

        SubPartitionMotionSearch (currMB, mode, block, lambda_mf);

        //--- get cost and reference frame for LIST 0 prediction ---
        bmcost[LIST_0] = DISTBLK_MAX;
        list_prediction_cost(currMB, LIST_0, block, mode, enc_mb, bmcost, best.ref);

        //store LIST 0 reference index for every block
        block_x = currMB->block_x + (block & 0x01)*2;
        block_y = currMB->block_y + (block & 0x02);
        b_ref = best.ref[LIST_0];

        motion[block_y    ][block_x    ].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
        motion[block_y    ][block_x    ].mv      [LIST_0] = currSlice->all_mv[LIST_0][b_ref][mode][j1    ][i1    ];
        motion[block_y    ][block_x    ].ref_idx [LIST_0] = (char) b_ref;
        motion[block_y    ][block_x + 1].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
        motion[block_y    ][block_x + 1].mv      [LIST_0] = currSlice->all_mv[LIST_0][b_ref][mode][j1    ][i1 + 1];
        motion[block_y    ][block_x + 1].ref_idx [LIST_0] = (char) b_ref;
        motion[block_y + 1][block_x    ].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
        motion[block_y + 1][block_x    ].mv      [LIST_0] = currSlice->all_mv[LIST_0][b_ref][mode][j1 + 1][i1    ];
        motion[block_y + 1][block_x    ].ref_idx [LIST_0] = (char) b_ref;
        motion[block_y + 1][block_x + 1].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
        motion[block_y + 1][block_x + 1].mv      [LIST_0] = currSlice->all_mv[LIST_0][b_ref][mode][j1 + 1][i1 + 1];
        motion[block_y + 1][block_x + 1].ref_idx [LIST_0] = (char) b_ref;

        //--- get cost and reference frame for LIST 1 prediction ---
        bmcost[LIST_1] = DISTBLK_MAX;
        bmcost[BI_PRED] = DISTBLK_MAX;
        list_prediction_cost(currMB, LIST_1, block, mode, enc_mb, bmcost, best.ref);

        // Compute bipredictive cost between best list 0 and best list 1 references
        list_prediction_cost(currMB, BI_PRED, block, mode, enc_mb, bmcost, best.ref);

        // currently Bi prediction ME is only supported for modes 1, 2, 3 and only for ref 0 and only for ref 0
        if (is_bipred_enabled(p_Vid, mode))
        {
          get_bipred_cost(currMB, mode, block, i1, j1, &best, enc_mb, bmcost);
        }
        else
        {
          bmcost[BI_PRED_L0] = DISTBLK_MAX;
          bmcost[BI_PRED_L1] = DISTBLK_MAX;
        }

        //--- get prediction direction ----
        determine_prediction_list(bmcost, &best, cost);

        //store backward reference index for every block
        for (k = LIST_0; k <= LIST_1; k++)
        {
          motion[block_y     ][block_x    ].ref_idx[k] = best.ref[k];
          motion[block_y     ][block_x + 1].ref_idx[k] = best.ref[k];
          motion[block_y  + 1][block_x    ].ref_idx[k] = best.ref[k];
          motion[block_y  + 1][block_x + 1].ref_idx[k] = best.ref[k];

          if (best.bipred)
          {              
            motion[block_y     ][block_x    ].mv[k] = currSlice->bipred_mv[best.bipred - 1][k][(short) best.ref[k]][mode][j1    ][i1    ];
            motion[block_y     ][block_x + 1].mv[k] = currSlice->bipred_mv[best.bipred - 1][k][(short) best.ref[k]][mode][j1    ][i1 + 1];
            motion[block_y  + 1][block_x    ].mv[k] = currSlice->bipred_mv[best.bipred - 1][k][(short) best.ref[k]][mode][j1 + 1][i1    ];
            motion[block_y  + 1][block_x + 1].mv[k] = currSlice->bipred_mv[best.bipred - 1][k][(short) best.ref[k]][mode][j1 + 1][i1 + 1];
          }
          else
          {
            motion[block_y     ][block_x    ].mv[k] = currSlice->all_mv[k][(short) best.ref[k]][mode][j1    ][i1    ];
            motion[block_y     ][block_x + 1].mv[k] = currSlice->all_mv[k][(short) best.ref[k]][mode][j1    ][i1 + 1];
            motion[block_y  + 1][block_x    ].mv[k] = currSlice->all_mv[k][(short) best.ref[k]][mode][j1 + 1][i1    ];
            motion[block_y  + 1][block_x + 1].mv[k] = currSlice->all_mv[k][(short) best.ref[k]][mode][j1 + 1][i1 + 1];
          }
        }                
      } // if (mode!=0)

      //--- get and check rate-distortion cost ---
      rdcost = rdcost_for_8x8blocks (currMB, dataTr, &cnt_nonz, &curr_cbp_blk, enc_mb->lambda_mdfp, block, (short) mode, &best, min_rdcost);

      //--- set variables if best mode has changed ---
      if (rdcost < min_rdcost)
      {
        min_cost8x8              = *cost;
        min_rdcost               = rdcost;        
        *partition               = best;
        partition->mode          = (char) mode;
        currMB->b8x8[block].mode = (char) mode;

#ifdef BEST_NZ_COEFF
        if (cnt_nonz)
        {
          for(i = 0; i <= 1; i++)
          {
            best_nz_coeff[i][0]= p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1    ];
            best_nz_coeff[i][1]= p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1 + 1];
          }
        }
        else
        {
          for(i = 0; i <= 1; i++)
          {
            best_nz_coeff[i][0]= 0;
            best_nz_coeff[i][1]= 0;
          }
        }
#endif

        //--- store number of nonzero coefficients ---
        best_cnt_nonz  = cnt_nonz;

        //--- store block cbp ---
        dataTr->cbp_blk8x8 &= (~(0x33 << (((block>>1)<<3)+((block & 0x01)<<1)))); // delete bits for block
        dataTr->cbp_blk8x8 |= curr_cbp_blk;

        //--- store coefficients ---
        memcpy(&cofACtr[0][0][0][0],&currSlice->cofAC[block][0][0][0], 4 * 2 * 65 * sizeof(int));

        if( currSlice->P444_joined ) 
        {
          //--- store coefficients ---
          memcpy(&cofACtr[1][0][0][0],&currSlice->cofAC[block + 4][0][0][0], 4 * 2 * 65 * sizeof(int));
          memcpy(&cofACtr[2][0][0][0],&currSlice->cofAC[block + 8][0][0][0], 4 * 2 * 65 * sizeof(int));
        }

        //--- store reconstruction and prediction ---
        copy_image_data_8x8(&dataTr->rec_mbY8x8[j0], &p_Vid->enc_picture->imgY[currMB->pix_y + j0], i0, currMB->pix_x + i0);
        copy_image_data_8x8(&dataTr->mpr8x8[j0], &currSlice->mb_pred[0][j0], i0, i0);

        if (p_Inp->rdopt == 3)
        {
          errdo_store_best_b8x8(currMB, transform8x8, block); 
        }        

        if(currSlice->P444_joined) 
        {
          copy_image_data_8x8(&dataTr->rec_mb8x8_cr[0][j0], &p_Vid->enc_picture->imgUV[0][currMB->pix_y + j0], i0, currMB->pix_x + i0);
          copy_image_data_8x8(&dataTr->mpr8x8CbCr[0][j0], &currSlice->mb_pred[1][j0], i0, i0);

          copy_image_data_8x8(&dataTr->rec_mb8x8_cr[1][j0], &p_Vid->enc_picture->imgUV[1][currMB->pix_y + j0], i0, currMB->pix_x + i0);
          copy_image_data_8x8(&dataTr->mpr8x8CbCr[1][j0], &currSlice->mb_pred[2][j0], i0, i0);
        }

        //--- store best 8x8 coding state ---
        if (block < 3)
        {
          currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_b8);
          stored_state_8x8 = TRUE;
        }
      } // if (rdcost <= min_rdcost)

      //--- re-set coding state as it was before coding with current mode was performed ---
      if (index != maxindex - 1)
      {
        if(p_Inp->subMBCodingState == 1)
          currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);
        else if(p_Inp->subMBCodingState == 2)
          currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_tmp);
      }
    } // if ((enc_mb->valid[mode] && (transform8x8 == 0 || mode != 0 || (mode == 0 && active_sps->direct_8x8_inference_flag)))
  } // for (min_rdcost=1e30, index = 0; index<6; index++)

  if (valid_8x8 == TRUE)
  {
#ifdef BEST_NZ_COEFF
    for(i = 0; i <= 1; i++)  
    {
      for(j = 0; j <= 1; j++)
        p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1 + j] = best_nz_coeff[i][j];
    }
#endif

    if (!transform8x8)
    {
      if (min_cost8x8 != DISTBLK_MAX)
        dataTr->mb_p8x8_cost += min_cost8x8;
      else
        dataTr->mb_p8x8_cost = DISTBLK_MAX;
    }

    //----- set cbp and count of nonzero coefficients ---
    if (best_cnt_nonz)
    {
      dataTr->cbp8x8       |= (1 << block);
      dataTr->cnt_nonz_8x8 += best_cnt_nonz;
    }

    if (!transform8x8)
    {
      if (block < 3)
      {
        //===== re-set reconstructed block =====
        j0   = 8*(block >> 1);
        i0   = 8*(block & 0x01);

        // need to double check code since original did not use i0
        copy_image_data_8x8(&p_Vid->enc_picture->imgY[currMB->pix_y + j0], &dataTr->rec_mbY8x8[j0], currMB->pix_x + i0, i0);

        if (p_Inp->rdopt == 3)
        {
          errdo_get_best_b8x8(currMB, transform8x8, block);
        }        

        if(currSlice->P444_joined) 
        {

          for (k=0; k<2; k++)
          {
            copy_image_data_8x8(&p_Vid->enc_picture->imgUV[k][currMB->pix_y + j0], &dataTr->rec_mb8x8_cr[k][j0], currMB->pix_x + i0, i0);
          }
        }
      } // if (block<3)
    }
    else
    {
      //======= save motion data for 8x8 partition for transform size 8x8 ========    
      currSlice->store_8x8_motion_vectors(currSlice, 0, block, partition);
    }

    //===== set motion vectors and reference frames (prediction) =====    
    currSlice->set_ref_and_motion_vectors (currMB, motion, partition, block);

    //===== set the coding state after current block =====
    //if (transform8x8 == 0 || block < 3)
    if (stored_state_8x8 == TRUE)
      currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_b8);
    else
    {
      currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);
      update_adaptive_rounding_8x8(p_Vid, p_Inp, dataTr, fadjust);
    }
  }
}

/*!
*************************************************************************************
* \brief
*    Low Complexity Mode Decision for an 8x8 sub-macroblock
*************************************************************************************
*/
void submacroblock_mode_decision_low(Macroblock *currMB,
                                     RD_PARAMS *enc_mb,
                                     RD_8x8DATA *dataTr,                                     
                                     int ****cofACtr,
                                     int *have_direct,
                                     int block,
                                     distblk *cost_direct,
                                     distblk *cost,
                                     distblk *cost8x8_direct,
                                     int transform8x8)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  int64 curr_cbp_blk;
  //double rdcost = 0.0;
  int j0, i0;
  int i, j;
  int index;
  int mode;
  distblk min_cost8x8;
  distblk direct4x4_tmp, direct8x8_tmp;
  distblk bmcost[5] = {DISTBLK_MAX};
  int cnt_nonz = 0;
  int dummy;
  int best_cnt_nonz = 0;
  int maxindex =  (transform8x8) ? 2 : 5;
  int block_x, block_y;
  int lambda_mf[3];

  int ****fadjust = transform8x8? p_Vid->ARCofAdj8x8 : p_Vid->ARCofAdj4x4;
  short pdir;

  Boolean valid_8x8 = FALSE;
  Boolean stored_state_8x8 = FALSE;

#ifdef BEST_NZ_COEFF
  int j1, i1;
  int best_nz_coeff[2][2];
#endif

  Info8x8 best = init_info_8x8_struct();
  dataTr->part[block].mode = 0;
  if (transform8x8)
    currMB->valid_8x8 = FALSE;
  else
    currMB->valid_4x4 = FALSE;

  //--- set coordinates ---
  j0 = ((block>>1)<<3);
  i0 = ((block&0x01)<<3);

#ifdef BEST_NZ_COEFF
  j1 = (j0>>2);
  i1 = (i0>>2);
  for(j = 0; j <= 1; j++)
  {
    for(i = 0; i <= 1; i++)
      best_nz_coeff[i][j] = p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1 + j] = 0;
  }
#endif

  if (transform8x8)
    currMB->luma_transform_size_8x8_flag = TRUE; //switch to transform size 8x8

  //--- store coding state before coding ---
  currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_cm);

  //=====  LOOP OVER POSSIBLE CODING MODES FOR 8x8 SUB-PARTITION  =====
  for (min_cost8x8 = DISTBLK_MAX, index = (currSlice->slice_type == B_SLICE ? 0 : 1); index < maxindex; index++)
  {
    mode = b8_mode_table[index];
    best.mode = (char) mode;
    *cost = 0;

    if (enc_mb->valid[mode] && (!(transform8x8 == 1 && mode > 4)) && (transform8x8 == 0 || mode != 0 || (mode == 0 && p_Vid->active_sps->direct_8x8_inference_flag)))
    {
      if (transform8x8)
      {
        currMB->valid_8x8 = TRUE;
      }
      else 
        currMB->valid_4x4 = TRUE;

      valid_8x8 = TRUE;
      curr_cbp_blk = 0;

      if (mode==0)
      {
        //--- Direct Mode ---       
        direct4x4_tmp = 0;
        direct8x8_tmp = 0;
        direct4x4_tmp = GetDirectCost8x8 ( currMB, block, &direct8x8_tmp);
        if ((direct4x4_tmp==DISTBLK_MAX)||(*cost_direct==DISTBLK_MAX))
        {
          *cost_direct = DISTBLK_MAX;
          if (transform8x8)
            *cost8x8_direct = DISTBLK_MAX;
        }
        else
        {
          *cost_direct += direct4x4_tmp;
          if (transform8x8)
            *cost8x8_direct += direct8x8_tmp;
        }
        (*have_direct) ++;

        if (transform8x8)
        {
          switch(p_Inp->Transform8x8Mode)
          {
          case 1: // Mixture of 8x8 & 4x4 transform
            if((direct8x8_tmp < direct4x4_tmp) || !(enc_mb->valid[5] && enc_mb->valid[6] && enc_mb->valid[7]))
              *cost = direct8x8_tmp;
            else
              *cost = direct4x4_tmp;
            break;
          case 2: // 8x8 Transform only
            *cost = direct8x8_tmp;
            break;
          default: // 4x4 Transform only
            *cost = direct4x4_tmp;
            break;
          }
          if (p_Inp->Transform8x8Mode==2)
            *cost = DISTBLK_MAX;
        }
        else
        {
          *cost = direct4x4_tmp;
        }

        block_x = currMB->block_x + (block & 0x01)*2;
        block_y = currMB->block_y + (block & 0x02);
        best.ref[LIST_0] = currSlice->direct_ref_idx[block_y][block_x][LIST_0];
        best.ref[LIST_1] = currSlice->direct_ref_idx[block_y][block_x][LIST_1];
        best.pdir        = currSlice->direct_pdir[block_y][block_x];
      } // if (mode==0)
      else
      {
        short b_ref;

        //======= motion estimation for all reference frames ========
        //-----------------------------------------------------------
        memcpy(lambda_mf, enc_mb->lambda_mf, 3 * sizeof(int));
        if (p_Inp->CtxAdptLagrangeMult == 1)
        {
          RDOPTStructure *p_RDO = currSlice->p_RDO;
          lambda_mf[F_PEL] = (int)(lambda_mf[F_PEL] * p_RDO->lambda_mf_factor);
          lambda_mf[H_PEL] = (int)(lambda_mf[H_PEL] * p_RDO->lambda_mf_factor);
          lambda_mf[Q_PEL] = (int)(lambda_mf[Q_PEL] * p_RDO->lambda_mf_factor);
        }

        SubPartitionMotionSearch (currMB, mode, block, lambda_mf);

        //--- get cost and reference frame for LIST 0 prediction ---
        bmcost[LIST_0] = DISTBLK_MAX;
        list_prediction_cost(currMB, LIST_0, block, mode, enc_mb, bmcost, best.ref);

        //store LIST 0 reference index for every block
        block_x = currMB->block_x + (block & 0x01)*2;
        block_y = currMB->block_y + (block & 0x02);
        b_ref = best.ref[LIST_0];

        for (j = block_y; j< block_y + 2; j++)
        {
          for (i = block_x; i< block_x + 2; i++)
          {
            motion[j][i].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
            motion[j][i].ref_idx [LIST_0] = (char) b_ref;
          }
        }

        if (currSlice->slice_type == B_SLICE)
        {
          //--- get cost and reference frame for LIST 1 prediction ---
          bmcost[LIST_1] = DISTBLK_MAX;
          bmcost[BI_PRED] = DISTBLK_MAX;
          list_prediction_cost(currMB, LIST_1, block, mode, enc_mb, bmcost, best.ref);

          // Compute bipredictive cost between best list 0 and best list 1 references
          list_prediction_cost(currMB, BI_PRED, block, mode, enc_mb, bmcost, best.ref);

          // currently Bi prediction ME is only supported for modes 1, 2, 3 and only for ref 0 and only for ref 0
          if (is_bipred_enabled(p_Vid, mode))
          {
            list_prediction_cost(currMB, BI_PRED_L0, block, mode, enc_mb, bmcost, 0);
            list_prediction_cost(currMB, BI_PRED_L1, block, mode, enc_mb, bmcost, 0);
          }
          else
          {
            bmcost[BI_PRED_L0] = DISTBLK_MAX;
            bmcost[BI_PRED_L1] = DISTBLK_MAX;
          }

          //--- get prediction direction ----
          determine_prediction_list(bmcost, &best, cost);

          //store backward reference index for every block
          for (j = block_y; j< block_y + 2; j++)
          {
            for (i = block_x; i < block_x + 2; i++)
            {
              //motion[j][i].ref_pic [LIST_0] = currSlice->listX[currMB->list_offset][b_ref];
              motion[j][i].ref_idx [LIST_0] = (char) best.ref[LIST_0];
              motion[j][i].ref_idx [LIST_0] = (char) best.ref[LIST_1];
            }
          }
        } // if (currSlice->slice_type == B_SLICE)
        else
        {
          best.pdir = 0;
          *cost     = bmcost[LIST_0];
        }
      } // if (mode!=0)
      if (*cost != DISTBLK_MAX)
        *cost += (ref_cost(currSlice, enc_mb->lambda_mf[Q_PEL], (short) B8Mode2Value (currSlice, (short) mode, best.pdir), 
        (best.pdir < 1 ? currMB->list_offset : currMB->list_offset + LIST_1)) - 1);

      //--- set variables if best mode has changed ---
      if (*cost < min_cost8x8)
      {
        min_cost8x8             = *cost;
        
        dataTr->part[block] = best;
        currMB->b8x8[block].mode = (char) mode;

#ifdef BEST_NZ_COEFF
        if (cnt_nonz)
        {
          best_nz_coeff[0][0]= p_Vid->nz_coeff[currMB->mbAddrX][i1    ][j1    ];
          best_nz_coeff[0][1]= p_Vid->nz_coeff[currMB->mbAddrX][i1    ][j1 + 1];
          best_nz_coeff[1][0]= p_Vid->nz_coeff[currMB->mbAddrX][i1 + 1][j1    ];
          best_nz_coeff[1][1]= p_Vid->nz_coeff[currMB->mbAddrX][i1 + 1][j1 + 1];
        }
        else
        {
          best_nz_coeff[0][0]= 0;
          best_nz_coeff[0][1]= 0;
          best_nz_coeff[1][0]= 0;
          best_nz_coeff[1][1]= 0;
        }
#endif

        //--- store number of nonzero coefficients ---
        best_cnt_nonz  = cnt_nonz;                

        //--- store best 8x8 coding state ---
        if (block < 3)
        {
          currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_b8);
          stored_state_8x8 = TRUE;
        }
      } // if (rdcost <= min_rdcost)

      //--- re-set coding state as it was before coding with current mode was performed ---
      currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);
    } // if ((enc_mb->valid[mode] && (transform8x8 == 0 || mode != 0 || (mode == 0 && active_sps->direct_8x8_inference_flag)))
  } // for (min_rdcost=1e30, index=(currSlice->slice_type == B_SLICE ? 0 : 1); index<6; index++)

  if (valid_8x8 == TRUE)
  {
    int list_mode[2];
#ifdef BEST_NZ_COEFF
    for(i = 0; i <= 1; i++)  
    {
      for(j = 0; j <= 1; j++)
        p_Vid->nz_coeff[currMB->mbAddrX][i1 + i][j1 + j] = best_nz_coeff[i][j];
    }
#endif

    if (!transform8x8)
    {
      if (min_cost8x8 != DISTBLK_MAX)
        dataTr->mb_p8x8_cost += min_cost8x8;
      else
        dataTr->mb_p8x8_cost = DISTBLK_MAX;
    }


    if (transform8x8)
    {
      if (min_cost8x8 != DISTBLK_MAX)
        dataTr->mb_p8x8_cost += min_cost8x8;
      else
        dataTr->mb_p8x8_cost = DISTBLK_MAX;

      mode = dataTr->part[block].mode;
      pdir = dataTr->part[block].pdir;
    }
    else
    {
      mode = dataTr->part[block].mode;
      pdir = dataTr->part[block].pdir;
    }

    curr_cbp_blk  = 0;
    currMB->b8x8[block].bipred = dataTr->part[block].bipred;
    currMB->ar_mode = (short) ((mode != 0)? mode: P8x8);

    list_mode[0] = (pdir == 0 || pdir == 2 ? mode : 0);
    list_mode[1] = (pdir == 1 || pdir == 2 ? mode : 0);

    best_cnt_nonz = currSlice->luma_residual_coding_8x8 (currMB, &dummy, &curr_cbp_blk, block, pdir, list_mode, dataTr->part[block].ref);

    if (currSlice->P444_joined)
    {
      best_cnt_nonz += currSlice->coeff_cost_cr[1] + currSlice->coeff_cost_cr[2];
    }

    dataTr->cbp_blk8x8   &= (~(0x33 << (((block>>1)<<3)+((block & 0x01)<<1)))); // delete bits for block
    dataTr->cbp_blk8x8   |= curr_cbp_blk;

    //--- store coefficients ---
    memcpy(cofACtr[0][0][0],currSlice->cofAC[block][0][0], 4 * 2 * 65 * sizeof(int));

    if(currSlice->P444_joined) 
    {
      //--- store coefficients ---
      memcpy(cofACtr[1][0][0],currSlice->cofAC[block + 4][0][0], 4 * 2 * 65 * sizeof(int));
      memcpy(cofACtr[2][0][0],currSlice->cofAC[block + 8][0][0], 4 * 2 * 65 * sizeof(int));
    }


    //--- store reconstruction and prediction ---
    copy_image_data_8x8(&dataTr->rec_mbY8x8[j0], &p_Vid->enc_picture->imgY[currMB->pix_y + j0], i0, currMB->pix_x + i0);
    copy_image_data_8x8(&dataTr->mpr8x8[j0], &currSlice->mb_pred[0][j0], i0, i0);


    //--- store reconstruction and prediction ---
    if(currSlice->slice_type == SP_SLICE)
    {
      for (j=j0; j < j0 + BLOCK_SIZE_8x8; j++)
      {
        memcpy(&dataTr->lrec[j][i0],&p_Vid->lrec[currMB->pix_y+j][currMB->pix_x+i0],BLOCK_SIZE_8x8 * sizeof(int)); // store coefficients for primary SP slice
      }
    }
    if(currSlice->P444_joined) 
    {
      copy_image_data_8x8(&dataTr->rec_mb8x8_cr[0][j0], &p_Vid->enc_picture->imgUV[0][currMB->pix_y + j0], i0, currMB->pix_x + i0);
      copy_image_data_8x8(&dataTr->mpr8x8CbCr[0][j0], &currSlice->mb_pred[1][j0], i0, i0);
      copy_image_data_8x8(&dataTr->rec_mb8x8_cr[1][j0], &p_Vid->enc_picture->imgUV[1][currMB->pix_y + j0], i0, currMB->pix_x + i0);
      copy_image_data_8x8(&dataTr->mpr8x8CbCr[1][j0], &currSlice->mb_pred[2][j0], i0, i0);
    }   


    //----- set cbp and count of nonzero coefficients ---
    if (best_cnt_nonz)
    {
      dataTr->cbp8x8       |= (1 << block);
      dataTr->cnt_nonz_8x8 += best_cnt_nonz;
    }

    if (!transform8x8)
    {
      if (block < 3)
      {
        //===== re-set reconstructed block =====
        j0   = 8*(block >> 1);
        i0   = 8*(block & 0x01);

        copy_image_data_8x8(&p_Vid->enc_picture->imgY[currMB->pix_y + j0], &dataTr->rec_mbY8x8[j0], currMB->pix_x + i0, i0);

        if(currSlice->slice_type == SP_SLICE)
        {
          for (j = j0; j < j0 + BLOCK_SIZE_8x8; j++)
          {
            memcpy(&p_Vid->lrec[currMB->pix_y + j][currMB->pix_x], dataTr->lrec[j], 2 * BLOCK_SIZE * sizeof(int)); // reset the coefficients for SP slice
          }
        }

        if(currSlice->P444_joined) 
        {
          copy_image_data_8x8(&p_Vid->enc_picture->imgUV[0][currMB->pix_y + j0], &dataTr->rec_mb8x8_cr[0][j0], currMB->pix_x + i0, i0);
          copy_image_data_8x8(&p_Vid->enc_picture->imgUV[1][currMB->pix_y + j0], &dataTr->rec_mb8x8_cr[1][j0], currMB->pix_x + i0, i0);
        }
      } // if (block<3)
    }
    else
    {
      //======= save motion data for 8x8 partition for transform size 8x8 ========    
      currSlice->store_8x8_motion_vectors(currSlice, 0, block, &dataTr->part[block]);
    }

    //===== set motion vectors and reference frames (prediction) =====
    currSlice->set_ref_and_motion_vectors (currMB, motion, &dataTr->part[block], block);

    //===== set the coding state after current block =====
    //if (transform8x8 == 0 || block < 3)
    if (stored_state_8x8 == TRUE)
      currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_b8);
    else
    {
      update_adaptive_rounding_8x8(p_Vid, p_Inp, dataTr, fadjust);
    }
  }
}

