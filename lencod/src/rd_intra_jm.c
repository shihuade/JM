/*!
 ***************************************************************************
 * \file rd_intra_jm.c
 *
 * \brief
 *    Rate-Distortion optimized mode decision
 *
 * \author
 *    - Heiko Schwarz
 *    - Valeri George
 *    - Lowell Winger              <lwinger@lsil.com>
 *    - Alexis Michael Tourapis    <alexismt@ieee.org>
 * \date
 *    12. April 2001
 **************************************************************************
 */

#include <limits.h>

#include "global.h"

#include "image.h"
#include "macroblock.h"
#include "mb_access.h"
#include "rdopt_coding_state.h"
#include "mode_decision.h"
#include "rdopt.h"
#include "rd_intra_jm.h"
#include "q_around.h"
#include "intra4x4.h"
#include "intra8x8.h"
#include "md_common.h"
#include "transform8x8.h"
#include "md_distortion.h"
#include "elements.h"
#include "symbol.h"
#include "intra16x16.h"
#include "intra4x4.h"
#include "intra8x8.h"

extern int MBType2Value (Macroblock* currMB);

/*!
 *************************************************************************************
 * \brief
 *    Mode Decision for an 4x4 Intra block
 *************************************************************************************
 */
int mode_decision_for_I4x4_blocks_JM_High (Macroblock *currMB, int  b8,  int  b4,  int  lambda,  distblk*  min_cost)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  int     ipmode, best_ipmode = 0, y;
  int     available_mode;
  int     c_nz, nonzero = 0;
  int*    ACLevel = currSlice->cofAC[b8][b4][0];
  int*    ACRun   = currSlice->cofAC[b8][b4][1];
  distblk rdcost = 0;
  distblk min_rdcost  = DISTBLK_MAX;
  int    block_x     = ((b8 & 0x01) << 3) + ((b4 & 0x01) << 2);
  int    block_y     = ((b8 >> 1) << 3)  + ((b4 >> 1) << 2);
  int    pic_pix_x   = currMB->pix_x  + block_x;
  int    pic_pix_y   = currMB->pix_y  + block_y;
  int    pic_opix_x  = currMB->pix_x + block_x;
  int    pic_opix_y  = currMB->opix_y + block_y;
  int    pic_block_x = pic_pix_x >> 2;
  int    pic_block_y = pic_pix_y >> 2;

  int left_available, up_available, all_available;
  int *mb_size = p_Vid->mb_size[IS_LUMA];

  char   upMode, leftMode;
  int    mostProbableMode;

  PixelPos left_block, top_block;

  int  lrec4x4[4][4];
  int best_nz_coeff = 0;
  int block_x4 = block_x>>2;
  int block_y4 = block_y>>2;

#ifdef BEST_NZ_COEFF
  int best_coded_block_flag = 0;
  int bit_pos = 1 + ((((b8>>1)<<1)+(b4>>1))<<2) + (((b8&1)<<1)+(b4&1));
  int64 cbp_bits;

  if (b8==0 && b4==0)
    cbp_bits = 0;  
#endif

  get4x4Neighbour(currMB, block_x - 1, block_y    , mb_size, &left_block);
  get4x4Neighbour(currMB, block_x,     block_y - 1, mb_size, &top_block );

  // constrained intra pred
  if (p_Inp->UseConstrainedIntraPred)
  {
    left_block.available = left_block.available ? p_Vid->intra_block[left_block.mb_addr] : 0;
    top_block.available  = top_block.available  ? p_Vid->intra_block[top_block.mb_addr]  : 0;
  }

  upMode            =  top_block.available ? p_Vid->ipredmode[top_block.pos_y ][top_block.pos_x ] : (char) -1;
  leftMode          = left_block.available ? p_Vid->ipredmode[left_block.pos_y][left_block.pos_x] : (char) -1;

  mostProbableMode  = (upMode < 0 || leftMode < 0) ? DC_PRED : upMode < leftMode ? upMode : leftMode;
  *min_cost = DISTBLK_MAX;

  currMB->ipmode_DPCM = NO_INTRA_PMODE; ////For residual DPCM

  //===== INTRA PREDICTION FOR 4x4 BLOCK =====
  // set intra prediction values for 4x4 intra prediction
  currSlice->set_intrapred_4x4(currMB, PLANE_Y, pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);  

  //===== LOOP OVER ALL 4x4 INTRA PREDICTION MODES =====
  for (ipmode = 0; ipmode < NO_INTRA_PMODE; ipmode++)
  {
    available_mode =  (all_available) || (ipmode==DC_PRED) ||
      (up_available && (ipmode==VERT_PRED||ipmode==VERT_LEFT_PRED||ipmode==DIAG_DOWN_LEFT_PRED)) ||
      (left_available && (ipmode==HOR_PRED||ipmode==HOR_UP_PRED));

    if (valid_intra_mode(currSlice, ipmode) == 0)
      continue;

    if( available_mode)
    {
      // generate intra 4x4 prediction block given availability
      get_intrapred_4x4(currMB, PLANE_Y, ipmode, block_x, block_y, left_available, up_available);

      // get prediction and prediction error
      generate_pred_error_4x4(&p_Vid->pCurImg[pic_opix_y], currSlice->mpr_4x4[0][ipmode], &currSlice->mb_pred[0][block_y], &currSlice->mb_ores[0][block_y], pic_opix_x, block_x);     

      // get and check rate-distortion cost
#ifdef BEST_NZ_COEFF
      currMB->cbp_bits[0] = cbp_bits;
#endif      

      rdcost = currSlice->rdcost_for_4x4_intra_blocks (currMB, &c_nz, b8, b4, ipmode, lambda, mostProbableMode, min_rdcost);
      if ((rdcost < min_rdcost) || (rdcost == min_rdcost && ipmode == mostProbableMode))
      {
        //--- set coefficients ---
        memcpy(p_RDO->cofAC4x4[0], ACLevel, 18 * sizeof(int));
        memcpy(p_RDO->cofAC4x4[1], ACRun,   18 * sizeof(int));

        //--- set reconstruction ---
        copy_4x4block(p_RDO->rec4x4[PLANE_Y], &p_Vid->enc_picture->imgY[pic_pix_y], 0, pic_pix_x);

        // SP/SI reconstruction
        if(currSlice->slice_type == SP_SLICE && !currSlice->sp2_frame_indicator)
        {
          for (y=0; y<4; y++)
          {
            memcpy(lrec4x4[y],&p_Vid->lrec[pic_pix_y+y][pic_pix_x], BLOCK_SIZE * sizeof(int));// stores the mode coefficients
          }
        }

        //--- flag if transform-coefficients must be coded ---
        nonzero = c_nz;

        //--- set best mode update minimum cost ---
        *min_cost     = rdcost;
        min_rdcost    = rdcost;
        best_ipmode   = ipmode;

        best_nz_coeff = p_Vid->nz_coeff [currMB->mbAddrX][block_x4][block_y4];
#ifdef BEST_NZ_COEFF
        best_coded_block_flag = (int)((currMB->cbp_bits[0] >> bit_pos)&(int64)(1));
#endif
        if (p_Vid->AdaptiveRounding)
        {
          store_adaptive_rounding_4x4 (p_Vid, p_Vid->ARCofAdj4x4, I4MB, block_y, block_x);
        }
      }
    }
  }
#if INTRA_RDCOSTCALC_NNZ
  p_Vid->nz_coeff [currMB->mbAddrX][block_x4][block_y4] = best_nz_coeff;
#endif
#ifdef BEST_NZ_COEFF
  cbp_bits &= (~(int64)(1<<bit_pos));
  cbp_bits |= (int64)(best_coded_block_flag<<bit_pos);
#endif

  //===== set intra mode prediction =====
  p_Vid->ipredmode[pic_block_y][pic_block_x] = (char) best_ipmode;
  currMB->intra_pred_modes[4*b8+b4] =
    (char) (mostProbableMode == best_ipmode ? -1 : (best_ipmode < mostProbableMode ? best_ipmode : best_ipmode-1)); 

  //===== restore coefficients =====
  memcpy (ACLevel, p_RDO->cofAC4x4[0], 18 * sizeof(int));
  memcpy (ACRun,   p_RDO->cofAC4x4[1], 18 * sizeof(int));

  //===== restore reconstruction and prediction (needed if single coeffs are removed) =====
  copy_4x4block(&p_Vid->enc_picture->imgY[pic_pix_y], p_RDO->rec4x4[PLANE_Y], pic_pix_x, 0);
  copy_4x4block(&currSlice->mb_pred[0][block_y], currSlice->mpr_4x4[0][best_ipmode], block_x, 0);

  // SP/SI reconstuction
  if(currSlice->slice_type == SP_SLICE && !currSlice->sp2_frame_indicator)
  {
    for (y=0; y<BLOCK_SIZE; y++)
    {
      memcpy (&p_Vid->lrec[pic_pix_y+y][pic_pix_x], lrec4x4[y], BLOCK_SIZE * sizeof(int));//restore coefficients when encoding primary SP frame
    }
  }

  if (p_Vid->AdaptiveRounding)
  {
    update_adaptive_rounding_4x4 (p_Vid,p_Vid->ARCofAdj4x4, I4MB, block_y, block_x);
  }

  return nonzero;
}

/*!
*************************************************************************************
* \brief
*    8x8 Intra mode decision for a macroblock - High complexity
*************************************************************************************
*/
int mode_decision_for_I8x8_blocks_JM_High (Macroblock *currMB, int b8, int lambda, distblk *min_cost)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  int     ipmode, best_ipmode = 0, j;
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
  int     *p_AC8x8 = p_RDO->coefAC8x8intra[b8][0][0][0];
  int     *cofAC   = currSlice->cofAC[b8][0][0];

  int    left_available, up_available, all_available;
  int    **mb_ores = currSlice->mb_ores[0]; 
  imgpel **mb_pred = currSlice->mb_pred[0];

  char   upMode, leftMode;
  int    mostProbableMode;

  PixelPos left_block, top_block;

  int *mb_size = p_Vid->mb_size[IS_LUMA];

  get4x4Neighbour(currMB, block_x - 1, block_y    , mb_size, &left_block);
  get4x4Neighbour(currMB, block_x,     block_y - 1, mb_size, &top_block );

  // constrained intra pred
  if (p_Inp->UseConstrainedIntraPred)
  {
    left_block.available = left_block.available ? p_Vid->intra_block [left_block.mb_addr] : 0;
    top_block.available  = top_block.available  ? p_Vid->intra_block [top_block.mb_addr ] : 0;
  }

  if(b8 >> 1)
    upMode    =  top_block.available ? p_Vid->ipredmode8x8[top_block.pos_y ][top_block.pos_x ] : (char) -1;
  else
    upMode    =  top_block.available ? p_Vid->ipredmode   [top_block.pos_y ][top_block.pos_x ] : (char) -1;

  if(b8 & 0x01)
    leftMode  = left_block.available ? p_Vid->ipredmode8x8[left_block.pos_y][left_block.pos_x] : (char) -1;
  else
    leftMode  = left_block.available ? p_Vid->ipredmode[left_block.pos_y][left_block.pos_x] : (char) -1;

  mostProbableMode  = (upMode < 0 || leftMode < 0) ? DC_PRED : upMode < leftMode ? upMode : leftMode;
  *min_cost = DISTBLK_MAX;
  currMB->ipmode_DPCM = NO_INTRA_PMODE; //For residual DPCM

  //===== INTRA PREDICTION FOR 8x8 BLOCK =====
  currSlice->set_intrapred_8x8(currMB, PLANE_Y, pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);

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
      generate_pred_error_8x8(&p_Vid->pCurImg[pic_opix_y], currSlice->mpr_8x8[0][ipmode], &mb_pred[block_y], &mb_ores[block_y], pic_opix_x, block_x);     

      currMB->ipmode_DPCM = (short) ipmode;

      // get and check rate-distortion cost

      rdcost = currSlice->rdcost_for_8x8_intra_blocks (currMB, &c_nz, b8, ipmode, lambda, min_rdcost, mostProbableMode);
      if ((rdcost < min_rdcost) || (rdcost == min_rdcost && ipmode == mostProbableMode))
      {
        //--- set coefficients ---
        memcpy(p_AC8x8, cofAC, 4 * 2 * 65 * sizeof(int));

        //--- set reconstruction ---
        copy_image_data_8x8(p_RDO->rec8x8[PLANE_Y], &p_Vid->enc_picture->imgY[pic_pix_y], 0, pic_pix_x);

        if (p_Vid->AdaptiveRounding)
        {
          for (j = block_y; j < block_y + 8; j++)
            memcpy(&p_Vid->ARCofAdj8x8[0][DUMMY][j][block_x],&p_Vid->ARCofAdj8x8[0][I8MB][j][block_x], 8 * sizeof(int));
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

  currMB->intra_pred_modes8x8[4*b8] = (char) ((mostProbableMode == best_ipmode)
    ? -1
    : (best_ipmode < mostProbableMode ? best_ipmode : best_ipmode - 1));

  memset(&p_Vid->ipredmode8x8[mb_block_y    ][mb_block_x], best_ipmode, 2 * sizeof(char));
  memset(&p_Vid->ipredmode8x8[mb_block_y + 1][mb_block_x], best_ipmode, 2 * sizeof(char));

  //===== restore coefficients =====
  memcpy(cofAC, p_AC8x8, 4 * 2 * 65 * sizeof(int));

  if (p_Vid->AdaptiveRounding)
  {
    for (j=block_y; j< block_y + 8; j++)
      memcpy(&p_Vid->ARCofAdj8x8[0][I8MB][j][block_x], &p_Vid->ARCofAdj8x8[0][DUMMY][j][block_x], 8 * sizeof(int));
  }

  //===== restore reconstruction and prediction (needed if single coeffs are removed) =====
  copy_image_data_8x8(&p_Vid->enc_picture->imgY[pic_pix_y], p_RDO->rec8x8[0], pic_pix_x, 0);
  copy_image_data_8x8(&mb_pred[block_y], currSlice->mpr_8x8[0][best_ipmode], block_x, 0);

  return nonzero;
}

/*!
 *************************************************************************************
 * \brief
 *    Mode Decision for an 8x8 Intra block
 *************************************************************************************
 */
int Mode_Decision_for_IntraSubMBlocks(Macroblock *currMB, int b8, int lambda, distblk *cost, int non_zero[3])
{
  Slice *currSlice = currMB->p_Slice;
  int  b4;
  distblk  cost4x4;
  currMB->cr_cbp[0] = 0;
  currMB->cr_cbp[1] = 0;
  currMB->cr_cbp[2] = 0;

  memset(non_zero, 0, 3 * sizeof(int));
 *cost = weighted_cost(lambda, 6);        //6 * lambda;
  for (b4=0; b4<4; b4++)
  {
    non_zero[0] |= currSlice->mode_decision_for_I4x4_blocks (currMB, b8, b4, lambda, &cost4x4);
    non_zero[1] |= currMB->cr_cbp[1];
    non_zero[2] |= currMB->cr_cbp[2];
    *cost += cost4x4;
  }

  return non_zero[0];
}

/*!
 *************************************************************************************
 * \brief
 *    4x4 Intra mode decision for an macroblock
 *************************************************************************************
 */
int mode_decision_for_I4x4_MB (Macroblock *currMB, int lambda,  distblk* cost)
{
  Slice *currSlice = currMB->p_Slice;
  int  cbp=0, b8;
  distblk cost8x8;
  int non_zero[3] = {0, 0, 0};

  currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;
  
  for (*cost=0, b8=0; b8<4; b8++)
  {
    if (Mode_Decision_for_IntraSubMBlocks (currMB, b8, lambda, &cost8x8, non_zero))
    {
      cbp |= (1<<b8);
    }
    *cost += cost8x8;
    if (non_zero[1])
    {
      currSlice->cmp_cbp[1] |= (1<<b8);
      cbp |= currSlice->cmp_cbp[1];
      currSlice->cmp_cbp[1] = cbp;
      currSlice->cmp_cbp[2] = cbp;
    }
    if (non_zero[2])
    {
      currSlice->cmp_cbp[2] |= (1<<b8);
      cbp |= currSlice->cmp_cbp[2];
      currSlice->cmp_cbp[1] = cbp;
      currSlice->cmp_cbp[2] = cbp;
    }
  }
  return cbp;
}

int find_best_mode_I16x16_MB (Macroblock *currMB, int lambda,  distblk min_cost)
{
  Slice *currSlice = currMB->p_Slice;
  currMB->luma_transform_size_8x8_flag = FALSE;
  return (int) currSlice->find_sad_16x16 (currMB);
}

/*!
*************************************************************************************
* \brief
*    Intra 16x16 mode decision
*************************************************************************************
*/
int mode_decision_for_I16x16_MB (Macroblock* currMB, int lambda)
{
  find_best_mode_I16x16_MB (currMB, lambda, DISTBLK_MAX);
  return currMB->residual_transform_quant_luma_16x16 (currMB, PLANE_Y);    
}


/************************************************************************************
* \brief
*    Intra 16x16 mode decision using Rate-Distortion Optimization
*************************************************************************************
*/
int mode_decision_for_I16x16_MB_RDO (Macroblock* currMB, int lambda)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp; 
  Slice *currSlice = currMB->p_Slice;

  SyntaxElement   se;
  const int*      partMap    = assignSE2partition[currSlice->partition_mode];
  DataPartition*  dataPart   = &(currSlice->partArr[partMap[SE_MBTYPE]]);

  distblk min_rdcost = DISTBLK_MAX, rdcost;
  distblk distortionY; 
  int rate; 
  int i,k;
  int b8, b4; 


  int up_avail, left_avail, left_up_avail;

  int  best_mode = 0, best_cbp = 0; 
  int  bestCofAC[4][4][2][65];
  int  bestCofDC[2][18]; 
  imgpel bestRec[MB_BLOCK_SIZE][MB_BLOCK_SIZE];

  currMB->mb_type = I16MB; 
  
  currSlice->set_intrapred_16x16(currMB, PLANE_Y, &left_avail, &up_avail, &left_up_avail);

  for (k = 0;k < 4; k++)
  {
    if (p_Inp->IntraDisableInterOnly == 0 || (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE))
    {
      if (p_Inp->Intra16x16ParDisable && (k==VERT_PRED_16||k==HOR_PRED_16))
        continue;

      if (p_Inp->Intra16x16PlaneDisable && k==PLANE_16)
        continue;
    }

    if ((k==0 && !up_avail) || (k==1 && !left_avail) || (k==3 && (!left_avail || !up_avail || !left_up_avail)))
      continue; 

    get_intrapred_16x16(currMB, PLANE_Y, k, left_avail, up_avail);

    currMB->i16mode = (char) k; 
    currMB->cbp = currMB->residual_transform_quant_luma_16x16(currMB, PLANE_Y);
    distortionY = compute_SSE16x16_thres(&p_Vid->pCurImg[currMB->opix_y], &p_Vid->enc_picture->p_curr_img[currMB->pix_y], currMB->pix_x, currMB->pix_x, min_rdcost);

    if (distortionY < min_rdcost - weighted_cost(lambda, 4))
    {
      currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_tmp);
      currMB->i16offset = I16Offset  (currMB->cbp, currMB->i16mode);
      se.value1  = MBType2Value (currMB);
      se.value2  = 0;
      se.type    = SE_MBTYPE;

      currSlice->writeMB_typeInfo (currMB, &se, dataPart);

      rate = se.len;
      if (distortionY + weighted_cost(lambda, rate) < min_rdcost)
      {
        rate += currSlice->writeCoeff16x16   (currMB, PLANE_Y);
        currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_tmp);

        rdcost = distortionY + weighted_cost(lambda, rate); 

        if(rdcost < min_rdcost)
        {
          min_rdcost = rdcost; 
          best_mode = k; 
          best_cbp = currMB->cbp; 
          for(b8 = 0; b8 < 4; b8++)
          {
            for(b4 = 0; b4 < 4; b4++)
            {
              memcpy(bestCofAC[b8][b4][0], currSlice->cofAC[b8][b4][0], sizeof(int) * 65);
              memcpy(bestCofAC[b8][b4][1], currSlice->cofAC[b8][b4][1], sizeof(int) * 65);
            }
          }

          memcpy(bestCofDC[0], currSlice->cofDC[0][0], sizeof(int)*18);
          memcpy(bestCofDC[1], currSlice->cofDC[0][1], sizeof(int)*18);

          for(i = 0; i < MB_BLOCK_SIZE; i++)
            memcpy(bestRec[i], &p_Vid->enc_picture->p_curr_img[currMB->pix_y + i][currMB->pix_x], sizeof(imgpel)*MB_BLOCK_SIZE);
        }
      }
      else
      {
        currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_tmp);
      }
    }
  }

  currMB->i16mode = (char) best_mode;
  currMB->cbp = best_cbp; 
  currMB->i16offset = I16Offset  (currMB->cbp, currMB->i16mode);

  for(b8 = 0; b8 < 4; b8++)
  {
    for(b4 = 0; b4 < 4; b4++)
    {
      memcpy(currSlice->cofAC[b8][b4][0], bestCofAC[b8][b4][0], sizeof(int)*65);
      memcpy(currSlice->cofAC[b8][b4][1], bestCofAC[b8][b4][1], sizeof(int)*65);
    }
  }

  memcpy(currSlice->cofDC[0][0], bestCofDC[0], sizeof(int)*18);
  memcpy(currSlice->cofDC[0][1], bestCofDC[1], sizeof(int)*18);

  for(i = 0; i < MB_BLOCK_SIZE; i++)
  {
    memcpy(&p_Vid->enc_picture->p_curr_img[currMB->pix_y+i][currMB->pix_x], bestRec[i], MB_BLOCK_SIZE*sizeof(imgpel));
  }

  return currMB->cbp;
}


