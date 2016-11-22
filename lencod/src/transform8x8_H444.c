/*!
 ***************************************************************************
 * \file transform8x8_H444.c
 *
 * \brief
 *    8x8 transform functions
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Yuri Vatis
 *    - Jan Muenster
 *    - Lowell Winger                   <lwinger@lsil.com>
 * \date
 *    12. October 2003
 **************************************************************************
 */

#include <math.h>
#include <limits.h>

#include "global.h"

#include "image.h"
#include "mb_access.h"
#include "elements.h"
#include "vlc.h"
#include "transform8x8.h"
#include "transform.h"
#include "macroblock.h"
#include "symbol.h"
#include "mc_prediction.h"
#include "quant8x8.h"
#include "rdoq.h"
#include "q_matrix.h"
#include "q_offsets.h"
#include "rdopt.h"
#include "md_common.h"
#include "intra8x8.h"
#include "rdopt_coding_state.h"
#include "blk_prediction.h"

/*!
 *************************************************************************************
 * \brief
 *    8x8 Intra mode decision for a macroblock - Low complexity
 *************************************************************************************
 */
int mode_decision_for_I8x8_blocks_JM_Low444 (Macroblock *currMB, int b8, int lambda, distblk *min_cost)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;

  int     ipmode, best_ipmode = 0, j, dummy;
  distblk   cost;
  int     nonzero = 0;
  int     block_x     = (b8 & 0x01) << 3;
  int     block_y     = (b8 >> 1) << 3;
  int     pic_pix_x   = currMB->pix_x + block_x;
  int     pic_pix_y   = currMB->pix_y + block_y;
  int     pic_opix_x  = currMB->pix_x + block_x;
  int     pic_opix_y  = currMB->opix_y + block_y;
  int     pic_block_x = pic_pix_x >> 2;
  int     pic_block_y = pic_pix_y >> 2;
  int     mb_block_y  = (currMB->block_y) + ((b8 >> 1) << 1);
  int     mb_block_x  = (currMB->block_x) + ((b8 & 0x01) << 1);

  //imgpel    *img_org, *img_prd;
  //int       *residual;
  int left_available, up_available, all_available;
  int    **mb_ores = currSlice->mb_ores[0]; 
  imgpel **mb_pred = currSlice->mb_pred[0];
  int *mb_size = p_Vid->mb_size[IS_LUMA];

  char   upMode, leftMode;
  int    mostProbableMode;

  PixelPos left_block, top_block;

  get4x4Neighbour(currMB, block_x - 1, block_y    , mb_size, &left_block);
  get4x4Neighbour(currMB, block_x,     block_y - 1, mb_size, &top_block );

  if (p_Inp->UseConstrainedIntraPred)
  {
    top_block.available  = top_block.available  ? p_Vid->intra_block [top_block.mb_addr] : 0;
    left_block.available = left_block.available ? p_Vid->intra_block [left_block.mb_addr] : 0;
  }

  if(b8 >> 1)
    upMode    =  top_block.available ? p_Vid->ipredmode8x8[top_block.pos_y ][top_block.pos_x ] : -1;
  else
    upMode    =  top_block.available ? p_Vid->ipredmode   [top_block.pos_y ][top_block.pos_x ] : -1;

  if(b8 & 0x01)
    leftMode  = left_block.available ? p_Vid->ipredmode8x8[left_block.pos_y][left_block.pos_x] : -1;
  else
    leftMode  = left_block.available ? p_Vid->ipredmode[left_block.pos_y][left_block.pos_x] : -1;

  mostProbableMode  = (upMode < 0 || leftMode < 0) ? DC_PRED : upMode < leftMode ? upMode : leftMode;
  *min_cost = DISTBLK_MAX;
  currMB->ipmode_DPCM = NO_INTRA_PMODE; //For residual DPCM

  //===== INTRA PREDICTION FOR 8x8 BLOCK =====
  currSlice->set_intrapred_8x8(currMB, PLANE_Y, pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);

  if(currSlice->P444_joined)
  { 
    select_plane(p_Vid, PLANE_U);
    currSlice->set_intrapred_8x8(currMB, PLANE_U, pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);
    select_plane(p_Vid, PLANE_V);
    currSlice->set_intrapred_8x8(currMB, PLANE_V, pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);
    select_plane(p_Vid, PLANE_Y);
  }

  //===== LOOP OVER ALL 8x8 INTRA PREDICTION MODES =====
  for (ipmode = 0; ipmode < NO_INTRA_PMODE; ipmode++)
  {
    if( (ipmode==DC_PRED) ||
      ((ipmode==VERT_PRED||ipmode==VERT_LEFT_PRED||ipmode==DIAG_DOWN_LEFT_PRED) && up_available ) ||
      ((ipmode==HOR_PRED||ipmode==HOR_UP_PRED) && left_available ) ||
      (all_available) )
    {
      get_intrapred_8x8(currMB, PLANE_Y, ipmode, left_available, up_available);
      cost  = (ipmode == mostProbableMode) ? 0 : ( weighted_cost(lambda, 4) );
      cost += currSlice->compute_cost8x8(p_Vid, &p_Vid->pImgOrg[0][pic_opix_y], currSlice->mpr_8x8[0][ipmode], pic_opix_x, *min_cost - cost);

      if(currSlice->P444_joined)
      {
        get_intrapred_8x8(currMB, PLANE_U, ipmode, left_available, up_available);
        cost += currSlice->compute_cost8x8(p_Vid, &p_Vid->pImgOrg[PLANE_U][pic_opix_y], currSlice->mpr_8x8[PLANE_U][ipmode], pic_opix_x, *min_cost - cost);
        get_intrapred_8x8(currMB, PLANE_V, ipmode, left_available, up_available);
        cost += currSlice->compute_cost8x8(p_Vid, &p_Vid->pImgOrg[PLANE_V][pic_opix_y], currSlice->mpr_8x8[PLANE_V][ipmode], pic_opix_x, *min_cost - cost);
      }

      if (cost < *min_cost)
      {
        best_ipmode = ipmode;
        *min_cost   = cost;
      }
    }
  }

  //===== set intra mode prediction =====
  p_Vid->ipredmode8x8[pic_block_y][pic_block_x] = (char) best_ipmode;
  currMB->ipmode_DPCM = (char) best_ipmode; //For residual DPCM

  if(currSlice->P444_joined)
  {
    ColorPlane k;
    p_Vid->CbCr_predmode_8x8[b8] = best_ipmode; 
    for (k = PLANE_U; k <= PLANE_V; k++)
    {
      currMB->cr_cbp[k] = 0; 
      select_plane(p_Vid, k);
      /*
      for (j=0; j<8; j++)
      {
        for (i=0; i<8; i++)
        {
          currSlice->mb_pred[k][block_y+j][block_x+i] = currSlice->mpr_8x8[k][best_ipmode][j][i]; 
          currSlice->mb_ores[k][block_y+j][block_x+i] = p_Vid->pImgOrg[k][currMB->pix_y + block_y + j][currMB->pix_x + block_x + i] - currSlice->mpr_8x8[k][best_ipmode][j][i];
        }
      }
      */
      copy_image_data_8x8(&currSlice->mb_pred[k][block_y], currSlice->mpr_8x8[k][best_ipmode], block_x, 0);
      compute_residue(&(p_Vid->pImgOrg[k][currMB->pix_y+block_y]), &currSlice->mb_pred[k][block_y], &currSlice->mb_ores[k][block_y], block_x, currMB->pix_x+block_x, 8, 8);

      currMB->ipmode_DPCM = (short) best_ipmode; 
      if (currMB->residual_transform_quant_luma_8x8(currMB, k, b8, &dummy, 1))
        currMB->cr_cbp[k] = 1;
    }
    select_plane(p_Vid, PLANE_Y);
  }

  currMB->intra_pred_modes8x8[4*b8] = (char) ((mostProbableMode == best_ipmode)
    ? -1
    : (best_ipmode < mostProbableMode ? best_ipmode : best_ipmode-1));

  for(j = mb_block_y; j < mb_block_y + 2; j++)   //loop 4x4s in the subblock for 8x8 prediction setting
    memset(&p_Vid->ipredmode8x8[j][mb_block_x], best_ipmode, 2 * sizeof(char));

  // get prediction and prediction error
  /*
  for (j = block_y; j < block_y + 8; j++)
  {
    memcpy(&mb_pred[j][block_x],currSlice->mpr_8x8[0][best_ipmode][j - block_y], 8 * sizeof(imgpel));
    img_org  = &p_Vid->pCurImg[currMB->opix_y + j][pic_opix_x];
    img_prd  = &mb_pred[j][block_x];
    residual = &mb_ores[j][block_x];
    for (i=0; i<8; i++)
    {
      *residual++ = *img_org++ - *img_prd++;
    }
  }
  */
  generate_pred_error_8x8(&p_Vid->pCurImg[currMB->opix_y+block_y], currSlice->mpr_8x8[0][best_ipmode], &mb_pred[block_y], &mb_ores[block_y], pic_opix_x, block_x);

  currMB->ipmode_DPCM = (short) best_ipmode;
  nonzero = currMB->residual_transform_quant_luma_8x8 (currMB, PLANE_Y, b8, &dummy, 1);    
  return nonzero;
}

/*!
*************************************************************************************
* \brief
*    8x8 Intra mode decision for a macroblock - High complexity
*************************************************************************************
*/
int mode_decision_for_I8x8_blocks_JM_High444 (Macroblock *currMB, int b8, int lambda, distblk *min_cost)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  int     ipmode, best_ipmode = 0, j, dummy;
  int     c_nz, nonzero = 0; 
  distblk  rdcost = 0;
  distblk  min_rdcost  = DISTBLK_MAX;
  int     block_x     = (b8 & 0x01) << 3;
  int     block_y     = (b8 >> 1) << 3;
  int     pic_pix_x   = currMB->pix_x + block_x;
  int     pic_pix_y   = currMB->pix_y + block_y;
  int     pic_opix_x  = currMB->pix_x + block_x;
  int     pic_opix_y  = currMB->opix_y + block_y;
  int     pic_block_x = pic_pix_x >> 2;
  int     pic_block_y = pic_pix_y >> 2;
  int     mb_block_y  = (currMB->block_y) + ((b8 >> 1) << 1);
  int     mb_block_x  = (currMB->block_x) + ((b8 & 0x01) << 1);

  int uv;
  int left_available, up_available, all_available;

  char   upMode, leftMode;
  int    mostProbableMode;

  PixelPos left_block, top_block;

  int *mb_size = p_Vid->mb_size[IS_LUMA];

  get4x4Neighbour(currMB, block_x - 1, block_y    , mb_size, &left_block);
  get4x4Neighbour(currMB, block_x,     block_y - 1, mb_size, &top_block );

  if (p_Inp->UseConstrainedIntraPred)
  {
    top_block.available  = top_block.available  ? p_Vid->intra_block [top_block.mb_addr ] : 0;
    left_block.available = left_block.available ? p_Vid->intra_block [left_block.mb_addr] : 0;
  }

  if(b8 >> 1)
    upMode    =  top_block.available ? p_Vid->ipredmode8x8[top_block.pos_y ][top_block.pos_x ] : -1;
  else
    upMode    =  top_block.available ? p_Vid->ipredmode   [top_block.pos_y ][top_block.pos_x ] : -1;

  if(b8 & 0x01)
    leftMode  = left_block.available ? p_Vid->ipredmode8x8[left_block.pos_y][left_block.pos_x] : -1;
  else
    leftMode  = left_block.available ? p_Vid->ipredmode[left_block.pos_y][left_block.pos_x] : -1;

  mostProbableMode  = (upMode < 0 || leftMode < 0) ? DC_PRED : upMode < leftMode ? upMode : leftMode;
  *min_cost = DISTBLK_MAX;
  currMB->ipmode_DPCM = NO_INTRA_PMODE; //For residual DPCM

  //===== INTRA PREDICTION FOR 8x8 BLOCK =====
  currSlice->set_intrapred_8x8(currMB, PLANE_Y, pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);

  if(currSlice->P444_joined)
  { 
    select_plane(p_Vid, PLANE_U);
    currSlice->set_intrapred_8x8(currMB, PLANE_U, pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);
    select_plane(p_Vid, PLANE_V);
    currSlice->set_intrapred_8x8(currMB, PLANE_V, pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);
    select_plane(p_Vid, PLANE_Y);
  }

  //===== LOOP OVER ALL 8x8 INTRA PREDICTION MODES =====
  for (ipmode = 0; ipmode < NO_INTRA_PMODE; ipmode++)
  {
    if( (ipmode==DC_PRED) ||
      ((ipmode==VERT_PRED||ipmode==VERT_LEFT_PRED||ipmode==DIAG_DOWN_LEFT_PRED) && up_available ) ||
      ((ipmode==HOR_PRED||ipmode==HOR_UP_PRED) && left_available ) ||
      (all_available) )
    {
      get_intrapred_8x8(currMB, PLANE_Y, ipmode, left_available, up_available);

      // get prediction and prediction error
      generate_pred_error_8x8(&p_Vid->pImgOrg[0][pic_opix_y], currSlice->mpr_8x8[0][ipmode], &currSlice->mb_pred[0][block_y], &currSlice->mb_ores[0][block_y], pic_opix_x, block_x);

      if(currSlice->P444_joined) 
      {
        get_intrapred_8x8(currMB, PLANE_U, ipmode, left_available, up_available);
        generate_pred_error_8x8(&p_Vid->pImgOrg[1][pic_opix_y], currSlice->mpr_8x8[1][ipmode], &currSlice->mb_pred[1][block_y], &currSlice->mb_ores[1][block_y], pic_opix_x, block_x);
        get_intrapred_8x8(currMB, PLANE_V, ipmode, left_available, up_available);
        generate_pred_error_8x8(&p_Vid->pImgOrg[2][pic_opix_y], currSlice->mpr_8x8[2][ipmode], &currSlice->mb_pred[2][block_y], &currSlice->mb_ores[2][block_y], pic_opix_x, block_x);
      }

      currMB->ipmode_DPCM = (short) ipmode;

      // get and check rate-distortion cost

      rdcost = currSlice->rdcost_for_8x8_intra_blocks (currMB, &c_nz, b8, ipmode, lambda, min_rdcost, mostProbableMode);
      if ((rdcost < min_rdcost) || (rdcost == min_rdcost && ipmode == mostProbableMode))
      {
        //--- set coefficients ---
        memcpy(p_RDO->coefAC8x8intra[b8][0][0][0],currSlice->cofAC[b8][0][0], 4 * 2 * 65 * sizeof(int));

        //--- set reconstruction ---
        copy_image_data_8x8(p_RDO->rec8x8[PLANE_Y], &p_Vid->enc_picture->imgY[pic_pix_y], 0, pic_pix_x);

        if (p_Vid->AdaptiveRounding)
        {
          for (j = block_y; j < block_y + 8; j++)
            memcpy(&p_Vid->ARCofAdj8x8[0][DUMMY][j][block_x],&p_Vid->ARCofAdj8x8[0][I8MB][j][block_x], 8 * sizeof(int));

          if (currSlice->P444_joined)
          {
            for (j = block_y; j < block_y + 8; j++)
            {
              memcpy(&p_Vid->ARCofAdj8x8[1][DUMMY][j][block_x],&p_Vid->ARCofAdj8x8[1][I8MB][j][block_x], 8 * sizeof(int));
              memcpy(&p_Vid->ARCofAdj8x8[2][DUMMY][j][block_x],&p_Vid->ARCofAdj8x8[2][I8MB][j][block_x], 8 * sizeof(int));
            }
          }            
        }

        if (currSlice->P444_joined) 
        { 
          //--- set coefficients ---
          for (uv=0; uv < 2; uv++)
          {
            memcpy(p_RDO->coefAC8x8intra[b8][uv + 1][0][0],currSlice->cofAC[4+b8+4*uv][0][0], 2 * 4 * 65 * sizeof(int));

            currMB->cr_cbp[uv + 1] = currMB->c_nzCbCr[uv + 1];
            //--- set reconstruction ---
            copy_image_data_8x8(p_RDO->rec8x8[uv + 1], &p_Vid->enc_picture->imgUV[uv][pic_pix_y], 0, pic_pix_x);
          }
        }

        //--- flag if transform coefficients must be coded ---
        nonzero = c_nz;

        //--- set best mode update minimum cost ---
        *min_cost   = rdcost;
        min_rdcost  = rdcost;
        best_ipmode = ipmode;
      }
    }
  }

  //===== set intra mode prediction =====
  p_Vid->ipredmode8x8[pic_block_y][pic_block_x] = (char) best_ipmode;
  currMB->ipmode_DPCM = (short) best_ipmode; //For residual DPCM

  if(currSlice->P444_joined)
  {
    ColorPlane k;
    p_Vid->CbCr_predmode_8x8[b8] = best_ipmode; 
    for (k = PLANE_U; k <= PLANE_V; k++)
    {
      currMB->cr_cbp[k] = 0; 
      select_plane(p_Vid, k);
      /*
      for (j=0; j<8; j++)
      {
        for (i=0; i<8; i++)
        {
          currSlice->mb_pred[k][block_y+j][block_x+i] = currSlice->mpr_8x8[k][best_ipmode][j][i]; 
          currSlice->mb_ores[k][block_y+j][block_x+i] = p_Vid->pImgOrg[k][currMB->pix_y + block_y + j][currMB->pix_x + block_x + i] - currSlice->mpr_8x8[k][best_ipmode][j][i];
        }
      }
      */
      generate_pred_error_8x8(&p_Vid->pImgOrg[k][currMB->pix_y + block_y], currSlice->mpr_8x8[k][best_ipmode], &currSlice->mb_pred[k][block_y], &currSlice->mb_ores[k][block_y], currMB->pix_x + block_x, block_x);
      currMB->ipmode_DPCM = (short) best_ipmode; 

      if (currMB->residual_transform_quant_luma_8x8(currMB, k, b8, &dummy, 1))
        currMB->cr_cbp[k] = 1;
    }
    select_plane(p_Vid, PLANE_Y);
  }

  currMB->intra_pred_modes8x8[4*b8] = (char) ((mostProbableMode == best_ipmode)
    ? -1
    : (best_ipmode < mostProbableMode ? best_ipmode : best_ipmode-1));

  memset(&p_Vid->ipredmode8x8[mb_block_y    ][mb_block_x], best_ipmode, 2 * sizeof(char));
  memset(&p_Vid->ipredmode8x8[mb_block_y + 1][mb_block_x], best_ipmode, 2 * sizeof(char));

  //===== restore coefficients =====
  memcpy(currSlice->cofAC[b8][0][0], p_RDO->coefAC8x8intra[b8][0][0][0], 4 * 2 * 65 * sizeof(int));

  if (p_Vid->AdaptiveRounding)
  {
    for (j=block_y; j< block_y + 8; j++)
      memcpy(&p_Vid->ARCofAdj8x8[0][I8MB][j][block_x], &p_Vid->ARCofAdj8x8[0][DUMMY][j][block_x], 8 * sizeof(int));

    if (currSlice->P444_joined)
    {
      for (j=0; j<8; j++)
      {
        memcpy(&p_Vid->ARCofAdj8x8[1][I8MB][block_y + j][block_x], &p_Vid->ARCofAdj8x8[1][DUMMY][block_y + j][block_x], 8 * sizeof(int));
        memcpy(&p_Vid->ARCofAdj8x8[2][I8MB][block_y + j][block_x], &p_Vid->ARCofAdj8x8[2][DUMMY][block_y + j][block_x], 8 * sizeof(int));
      }
    }
  }

  //===== restore reconstruction and prediction (needed if single coeffs are removed) =====
  copy_image_data_8x8(&p_Vid->enc_picture->imgY[pic_pix_y], p_RDO->rec8x8[0], pic_pix_x, 0);
  copy_image_data_8x8(&currSlice->mb_pred[0][block_y], currSlice->mpr_8x8[0][best_ipmode], block_x, 0);

  if (currSlice->P444_joined)
  {
    //===== restore coefficients =====
    memcpy(currSlice->cofAC[4 + b8 + 4*0][0][0], p_RDO->coefAC8x8intra[b8][1][0][0], 4 * 2 * 65 * sizeof(int));
    memcpy(currSlice->cofAC[4 + b8 + 4*1][0][0], p_RDO->coefAC8x8intra[b8][2][0][0], 4 * 2 * 65 * sizeof(int));

    //===== restore reconstruction and prediction (needed if single coeffs are removed) =====
    copy_image_data_8x8(&p_Vid->enc_picture->imgUV[0][pic_pix_y], p_RDO->rec8x8[1], pic_pix_x, 0);
    copy_image_data_8x8(&p_Vid->enc_picture->imgUV[1][pic_pix_y], p_RDO->rec8x8[2], pic_pix_x, 0);
    copy_image_data_8x8(&currSlice->mb_pred[1][block_y], currSlice->mpr_8x8[1][best_ipmode], block_x, 0);
    copy_image_data_8x8(&currSlice->mb_pred[2][block_y], currSlice->mpr_8x8[2][best_ipmode], block_x, 0);
  }

  return nonzero;
}

