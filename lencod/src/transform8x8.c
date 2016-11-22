/*!
 ***************************************************************************
 * \file transform8x8.c
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
#include "blk_prediction.h"
#include "elements.h"
#include "vlc.h"
#include "transform8x8.h"
#include "transform.h"
#include "macroblock.h"
#include "symbol.h"
#include "mc_prediction.h"
#include "md_distortion.h"
#include "quant8x8.h"
#include "rdoq.h"
#include "q_matrix.h"
#include "q_offsets.h"
#include "rdopt.h"
#include "md_common.h"
#include "intra8x8.h"
#include "rdopt_coding_state.h"

//! single scan pattern
static const byte SNGL_SCAN8x8[64][2] = {
  {0,0}, {1,0}, {0,1}, {0,2}, {1,1}, {2,0}, {3,0}, {2,1},
  {1,2}, {0,3}, {0,4}, {1,3}, {2,2}, {3,1}, {4,0}, {5,0},
  {4,1}, {3,2}, {2,3}, {1,4}, {0,5}, {0,6}, {1,5}, {2,4},
  {3,3}, {4,2}, {5,1}, {6,0}, {7,0}, {6,1}, {5,2}, {4,3},
  {3,4}, {2,5}, {1,6}, {0,7}, {1,7}, {2,6}, {3,5}, {4,4},
  {5,3}, {6,2}, {7,1}, {7,2}, {6,3}, {5,4}, {4,5}, {3,6},
  {2,7}, {3,7}, {4,6}, {5,5}, {6,4}, {7,3}, {7,4}, {6,5},
  {5,6}, {4,7}, {5,7}, {6,6}, {7,5}, {7,6}, {6,7}, {7,7}
};

static const byte SNGL_SCAN8x8_CAVLC[64][2] = {
  {0,0}, {1,1}, {1,2}, {2,2}, {4,1}, {0,5}, {3,3}, {7,0}, {3,4}, {1,7}, {5,3}, {6,3}, {2,7}, {6,4}, {5,6}, {7,5},
  {1,0}, {2,0}, {0,3}, {3,1}, {3,2}, {0,6}, {4,2}, {6,1}, {2,5}, {2,6}, {6,2}, {5,4}, {3,7}, {7,3}, {4,7}, {7,6},
  {0,1}, {3,0}, {0,4}, {4,0}, {2,3}, {1,5}, {5,1}, {5,2}, {1,6}, {3,5}, {7,1}, {4,5}, {4,6}, {7,4}, {5,7}, {6,7},
  {0,2}, {2,1}, {1,3}, {5,0}, {1,4}, {2,4}, {6,0}, {4,3}, {0,7}, {4,4}, {7,2}, {3,6}, {5,5}, {6,5}, {6,6}, {7,7}
};

//! field scan pattern
static const byte FIELD_SCAN8x8[64][2] = {   // 8x8
  {0,0}, {0,1}, {0,2}, {1,0}, {1,1}, {0,3}, {0,4}, {1,2},
  {2,0}, {1,3}, {0,5}, {0,6}, {0,7}, {1,4}, {2,1}, {3,0},
  {2,2}, {1,5}, {1,6}, {1,7}, {2,3}, {3,1}, {4,0}, {3,2},
  {2,4}, {2,5}, {2,6}, {2,7}, {3,3}, {4,1}, {5,0}, {4,2},
  {3,4}, {3,5}, {3,6}, {3,7}, {4,3}, {5,1}, {6,0}, {5,2},
  {4,4}, {4,5}, {4,6}, {4,7}, {5,3}, {6,1}, {6,2}, {5,4},
  {5,5}, {5,6}, {5,7}, {6,3}, {7,0}, {7,1}, {6,4}, {6,5},
  {6,6}, {6,7}, {7,2}, {7,3}, {7,4}, {7,5}, {7,6}, {7,7}
};

static const byte FIELD_SCAN8x8_CAVLC[64][2] = {
  {0,0}, {1,1}, {2,0}, {0,7}, {2,2}, {2,3}, {2,4}, {3,3}, {3,4}, {4,3}, {4,4}, {5,3}, {5,5}, {7,0}, {6,6}, {7,4}, 
  {0,1}, {0,3}, {1,3}, {1,4}, {1,5}, {3,1}, {2,5}, {4,1}, {3,5}, {5,1}, {4,5}, {6,1}, {5,6}, {7,1}, {6,7}, {7,5}, 
  {0,2}, {0,4}, {0,5}, {2,1}, {1,6}, {4,0}, {2,6}, {5,0}, {3,6}, {6,0}, {4,6}, {6,2}, {5,7}, {6,4}, {7,2}, {7,6}, 
  {1,0}, {1,2}, {0,6}, {3,0}, {1,7}, {3,2}, {2,7}, {4,2}, {3,7}, {5,2}, {4,7}, {5,4}, {6,3}, {6,5}, {7,3}, {7,7}
};


//! array used to find expensive coefficients
static const byte COEFF_COST8x8[2][64] =
{
  {3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,
   1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
   9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
   9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
   9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9}
};


// Predictor array index definitions
#define P_Z (PredPel[0])
#define P_A (PredPel[1])
#define P_B (PredPel[2])
#define P_C (PredPel[3])
#define P_D (PredPel[4])
#define P_E (PredPel[5])
#define P_F (PredPel[6])
#define P_G (PredPel[7])
#define P_H (PredPel[8])
#define P_I (PredPel[9])
#define P_J (PredPel[10])
#define P_K (PredPel[11])
#define P_L (PredPel[12])
#define P_M (PredPel[13])
#define P_N (PredPel[14])
#define P_O (PredPel[15])
#define P_P (PredPel[16])
#define P_Q (PredPel[17])
#define P_R (PredPel[18])
#define P_S (PredPel[19])
#define P_T (PredPel[20])
#define P_U (PredPel[21])
#define P_V (PredPel[22])
#define P_W (PredPel[23])
#define P_X (PredPel[24])



/*!
************************************************************************
* \brief
*    Residual DPCM for Intra lossless coding
*
* \par Input:
*    block_x,block_y: Block position inside a macro block (0,8).
************************************************************************
*/
//For residual DPCM
static int Residual_DPCM_8x8(int ipmode, int **ores, int **rres,int block_y, int block_x)
{
  int i,j;
  int temp[8][8];

  if(ipmode==VERT_PRED)
  { 
    for (j=0; j<8; j++)
     temp[0][j] = ores[block_y][block_x+j];

    for (i=1; i<8; i++) 
      for (j=0; j<8; j++)
        temp[i][j] =  ores[block_y+i][block_x+j] - ores[block_y+i-1][block_x+j];

    for (i = 0; i < 8; i++)
      for (j = 0; j < 8; j++)
        rres[block_y+i][block_x+j] = temp[i][j];
  }
  else  //HOR_PRED
  {
    for (i=0; i<8; i++)
     temp[i][0] = ores[block_y + i][block_x];

    for (i=0; i<8; i++)
      for (j=1; j<8; j++)
        temp[i][j] = ores[block_y+i][block_x+j] - ores[block_y+i][block_x+j-1];

    for (i=0; i<8; i++)
      for (j=0; j<8; j++)
        rres[block_y+i][block_x+j] = temp[i][j];
  }
  return 0;
}

/*!
************************************************************************
* \brief
*    Inverse residual DPCM for Intra lossless coding
*
* \par Input:
*    block_x,block_y: Block position inside a macro block (0,8).
************************************************************************
*/
//For residual DPCM
static int Inv_Residual_DPCM_8x8(Macroblock *currMB, int **m7, int block_y, int block_x)  
{
  int i;
  int temp[8][8];

  if(currMB->ipmode_DPCM == VERT_PRED)
  {
    for(i=0; i<8; i++)
    {
      temp[0][i] = m7[block_y+0][block_x+i];
      temp[1][i] = temp[0][i] + m7[block_y+1][block_x+i];
      temp[2][i] = temp[1][i] + m7[block_y+2][block_x+i];
      temp[3][i] = temp[2][i] + m7[block_y+3][block_x+i];
      temp[4][i] = temp[3][i] + m7[block_y+4][block_x+i];
      temp[5][i] = temp[4][i] + m7[block_y+5][block_x+i];
      temp[6][i] = temp[5][i] + m7[block_y+6][block_x+i];
      temp[7][i] = temp[6][i] + m7[block_y+7][block_x+i];
    }
    for(i=0; i<8; i++)
    {
      m7[block_y+1][block_x+i] = temp[1][i];
      m7[block_y+2][block_x+i] = temp[2][i];
      m7[block_y+3][block_x+i] = temp[3][i];
      m7[block_y+4][block_x+i] = temp[4][i];
      m7[block_y+5][block_x+i] = temp[5][i];
      m7[block_y+6][block_x+i] = temp[6][i];
      m7[block_y+7][block_x+i] = temp[7][i];
    }
  }
  else //HOR_PRED
  {
    for(i=0; i<8; i++)
    {
      temp[i][0] = m7[block_y+i][block_x+0];
      temp[i][1] = temp[i][0] + m7[block_y+i][block_x+1];
      temp[i][2] = temp[i][1] + m7[block_y+i][block_x+2];
      temp[i][3] = temp[i][2] + m7[block_y+i][block_x+3];
      temp[i][4] = temp[i][3] + m7[block_y+i][block_x+4];
      temp[i][5] = temp[i][4] + m7[block_y+i][block_x+5];
      temp[i][6] = temp[i][5] + m7[block_y+i][block_x+6];
      temp[i][7] = temp[i][6] + m7[block_y+i][block_x+7];
    }
    for(i=0; i<8; i++)
    {
      m7[block_y+i][block_x+1] = temp[i][1];
      m7[block_y+i][block_x+2] = temp[i][2];
      m7[block_y+i][block_x+3] = temp[i][3];
      m7[block_y+i][block_x+4] = temp[i][4];
      m7[block_y+i][block_x+5] = temp[i][5];
      m7[block_y+i][block_x+6] = temp[i][6];
      m7[block_y+i][block_x+7] = temp[i][7];
    }
  }
  return 0;
}

/*!
 *************************************************************************************
 * \brief
 *    8x8 Intra mode decision for a macroblock
 *************************************************************************************
 */
int mode_decision_for_I8x8_MB (Macroblock *currMB, int lambda, distblk *min_cost)
{
  Slice *currSlice = currMB->p_Slice;
  int cur_cbp = 0, b8;
  //int cr_cbp[3] = { 0, 0, 0}; 
  distblk cost8x8;
  *min_cost = weighted_cost(lambda, 6); //6 bits overhead;

  if (currSlice->P444_joined == 0)
  {
    for (b8=0; b8<4; b8++)
    {
      if (currSlice->mode_decision_for_I8x8_blocks (currMB, b8, lambda, &cost8x8))
      {
        cur_cbp |= (1<<b8);
      }
      *min_cost += cost8x8;      
    }
  }
  else
  { 
    int k;
    currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;
    currMB->cr_cbp[0] = 0;
    currMB->cr_cbp[1] = 0;
    currMB->cr_cbp[2] = 0;

    for (b8 = 0; b8 < 4; b8++)
    {
      if (currSlice->mode_decision_for_I8x8_blocks (currMB, b8, lambda, &cost8x8))
      {
        cur_cbp |= (1<<b8);
      }
      *min_cost += cost8x8;

      for (k = 1; k < 3; k++)
      {
        if(currMB->cr_cbp[k]) //if (cr_cbp[k])
        {
          currSlice->cmp_cbp[k] |= (1 << b8);
          cur_cbp |= currSlice->cmp_cbp[k];
          currSlice->cmp_cbp[k] = cur_cbp;
        }
      }      
    }
  }

  return cur_cbp;
}


/*!
 *************************************************************************************
 * \brief
 *    R-D Cost for an 8x8 Intra block
 *************************************************************************************
 */
distblk rdcost_for_8x8_intra_blocks(Macroblock *currMB, int *nonzero, int b8, int ipmode, int lambda, distblk min_rdcost, int mostProbableMode)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  distblk  rdcost = 0;
  int     dummy = 0;
  int     rate;
  distblk   distortion  = 0;
  int     block_x     = (b8 & 0x01) << 3;
  int     block_y     = (b8 >> 1) << 3;
  int     pic_pix_x   = currMB->pix_x + block_x;
  int     pic_pix_y   = currMB->pix_y + block_y;
  int     pic_opix_y  = currMB->opix_y + block_y;

  SyntaxElement  se;
  const int      *partMap   = assignSE2partition[currSlice->partition_mode];
  DataPartition  *dataPart;

  //===== perform forward transform, Q, IQ, inverse transform, Reconstruction =====
  *nonzero = currMB->residual_transform_quant_luma_8x8 (currMB, PLANE_Y, b8, &dummy, 1);

  //===== get distortion (SSD) of 8x8 block =====
  distortion += compute_SSE8x8(&p_Vid->pCurImg[pic_opix_y], &p_Vid->enc_picture->imgY[pic_pix_y], pic_pix_x, pic_pix_x);
  if (distortion > min_rdcost)
  {
    //currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);
    return distortion;
  }
  currMB->ipmode_DPCM = NO_INTRA_PMODE;  

  //===== RATE for INTRA PREDICTION MODE  (SYMBOL MODE MUST BE SET TO CAVLC) =====
  se.value1 = (mostProbableMode == ipmode) ? -1 : ipmode < mostProbableMode ? ipmode : ipmode-1;

  //--- set position and type ---
  se.context = b8;
  se.type    = SE_INTRAPREDMODE;

  //--- choose data partition ---
  if (currSlice->slice_type != B_SLICE)
    dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);
  else
    dataPart = &(currSlice->partArr[partMap[SE_BFRAME]]);

  //--- encode and update rate ---
  currSlice->writeIntraPredMode (&se, dataPart);

  rate = se.len;

  //===== RATE for LUMINANCE COEFFICIENTS =====

  if (currSlice->symbol_mode == CAVLC)
  {      
    rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, 0, 0);
    rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, 1, 0);
    rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, 2, 0);
    rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, 3, 0);
  }
  else
  {
    rate  += writeCoeff8x8_CABAC (currMB, PLANE_Y, b8, 1);
  }
  rdcost = distortion + weight_cost(lambda, rate);
  currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);

  return rdcost;
}


/*!
 *************************************************************************************
 * \brief
 *    R-D Cost for an 8x8 Intra block
 *************************************************************************************
 */
distblk  rdcost_for_8x8_intra_blocks_444(Macroblock *currMB, int *nonzero, int b8, int ipmode, int lambda, distblk min_rdcost, int mostProbableMode)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  distblk  rdcost = 0;
  int     dummy = 0;
  int     rate;
  distblk   distortion  = 0;
  int     block_x     = (b8 & 0x01) << 3;
  int     block_y     = (b8 >> 1) << 3;
  int     pic_pix_x   = currMB->pix_x + block_x;
  int     pic_pix_y   = currMB->pix_y + block_y;
  int     pic_opix_y  = currMB->opix_y + block_y;

  SyntaxElement  se;
  const int      *partMap   = assignSE2partition[currSlice->partition_mode];
  DataPartition  *dataPart;

  if(currSlice->P444_joined == 0) 
  {
    //===== perform forward transform, Q, IQ, inverse transform, Reconstruction =====
    *nonzero = currMB->residual_transform_quant_luma_8x8 (currMB, PLANE_Y, b8, &dummy, 1);

    //===== get distortion (SSD) of 8x8 block =====
    distortion += compute_SSE8x8(&p_Vid->pCurImg[pic_opix_y], &p_Vid->enc_picture->imgY[pic_pix_y], pic_pix_x, pic_pix_x);

    currMB->ipmode_DPCM = NO_INTRA_PMODE;  

    //===== RATE for INTRA PREDICTION MODE  (SYMBOL MODE MUST BE SET TO CAVLC) =====
    se.value1 = (mostProbableMode == ipmode) ? -1 : ipmode < mostProbableMode ? ipmode : ipmode-1;

    //--- set position and type ---
    se.context = b8;
    se.type    = SE_INTRAPREDMODE;

    //--- choose data partition ---
    if (currSlice->slice_type != B_SLICE)
      dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);
    else
      dataPart = &(currSlice->partArr[partMap[SE_BFRAME]]);

    //--- encode and update rate ---
    currSlice->writeIntraPredMode (&se, dataPart);

    rate = se.len;

    //===== RATE for LUMINANCE COEFFICIENTS =====

    if (currSlice->symbol_mode == CAVLC)
    {      
      rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, 0, 0);
      rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, 1, 0);
      rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, 2, 0);
      rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, 3, 0);
    }
    else
    {
      rate  += writeCoeff8x8_CABAC (currMB, PLANE_Y, b8, 1);
    }
  }
  else
  {
    ColorPlane k;
    //===== perform forward transform, Q, IQ, inverse transform, Reconstruction =====
    *nonzero = currMB->residual_transform_quant_luma_8x8 (currMB, PLANE_Y, b8, &dummy, 1);

    //===== get distortion (SSD) of 8x8 block =====
    distortion += compute_SSE8x8(&p_Vid->pCurImg[pic_opix_y], &p_Vid->enc_picture->imgY[pic_pix_y], pic_pix_x, pic_pix_x);

    for (k = PLANE_U; k <= PLANE_V; k++)
    {
      select_plane(p_Vid, k);
      currMB->c_nzCbCr[k ]= currMB->residual_transform_quant_luma_8x8(currMB, k, b8, &dummy,1);
      distortion += compute_SSE8x8(&p_Vid->pImgOrg[k][pic_opix_y], &p_Vid->enc_picture->p_curr_img[pic_pix_y], pic_pix_x, pic_pix_x);
    }
    currMB->ipmode_DPCM = NO_INTRA_PMODE;
    select_plane(p_Vid, PLANE_Y);

    //===== RATE for INTRA PREDICTION MODE  (SYMBOL MODE MUST BE SET TO CAVLC) =====
    se.value1 = (mostProbableMode == ipmode) ? -1 : ipmode < mostProbableMode ? ipmode : ipmode-1;

    //--- set position and type ---
    se.context = b8;
    se.type    = SE_INTRAPREDMODE;

    //--- choose data partition ---
    if (currSlice->slice_type != B_SLICE)
      dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);
    else
      dataPart = &(currSlice->partArr[partMap[SE_BFRAME]]);

    //--- encode and update rate ---
    currSlice->writeIntraPredMode (&se, dataPart);
    rate = se.len;

    //===== RATE for LUMINANCE COEFFICIENTS =====

    if (currSlice->symbol_mode == CAVLC)
    {      
      int b4;
      for(b4=0; b4<4; b4++)
      {
        rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, b4, 0);
        rate  += currSlice->writeCoeff4x4_CAVLC (currMB, CB, b8, b4, 0);
        rate  += currSlice->writeCoeff4x4_CAVLC (currMB, CR, b8, b4, 0);
      }
    }
    else
    {
      rate  += writeCoeff8x8_CABAC (currMB, PLANE_Y, b8, 1);
      rate  += writeCoeff8x8_CABAC (currMB, PLANE_U, b8, 1);
      rate  += writeCoeff8x8_CABAC (currMB, PLANE_V, b8, 1);
    }
  }
  rdcost =  distortion + weight_cost(lambda, rate);
  currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);

  return rdcost;
}

static inline int check_zero(int **mb_ores, int block_x)
{
  int i, j, k = 0;

  for (j = 0; (j < BLOCK_SIZE_8x8) && (k == 0); j++)
  {
    for (i = block_x; (i< block_x + BLOCK_SIZE_8x8) && (k == 0); i++)
    {
      //k |= (mb_ores[j][i] != 0);
      k |= mb_ores[j][i];
    }
  }
  return k;
}

/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    b8: Block position inside a macro block (0,1,2,3).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.
 *    coeff_cost: Counter for nonzero coefficients, used to discard expensive levels.
 ************************************************************************
 */
int residual_transform_quant_luma_8x8(Macroblock *currMB, ColorPlane pl, int b8, int *coeff_cost, int intra)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int nonzero = FALSE; 

  int block_x = 8*(b8 & 0x01);
  int block_y = 8*(b8 >> 1);
  int pl_off = b8+ (pl<<2);
  Slice *currSlice = currMB->p_Slice;
  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;

  imgpel **mb_pred = currSlice->mb_pred[pl];
  int    **mb_ores = currSlice->mb_ores[pl];
  int    **mb_rres = currSlice->mb_rres[pl];

  int max_imgpel_value   = p_Vid->max_imgpel_value;

  if (check_zero(&mb_ores[block_y], block_x) != 0)
  {
    int qp = (p_Vid->yuv_format==YUV444 && !currSlice->P444_joined)? currMB->qp_scaled[(int)(p_Vid->colour_plane_id)]: currMB->qp_scaled[pl];

    // Variable p_Quant and some of its parameters could be all set outside 
    // to speed up the code (e.g. field mode, coeff_cost, etc). 
    QuantParameters *p_Quant = p_Vid->p_Quant;

    QuantMethods quant_methods;
    quant_methods.block_x = block_x;
    quant_methods.block_y = block_y;

    quant_methods.ACLevel = currSlice->cofAC[pl_off][0][0];
    quant_methods.ACRun   = currSlice->cofAC[pl_off][0][1];

    quant_methods.qp         = qp;
    quant_methods.q_params   = p_Quant->q_params_8x8[pl][intra][qp]; 
    quant_methods.fadjust    = p_Vid->AdaptiveRounding ? (&p_Vid->ARCofAdj8x8[pl][currMB->ar_mode][block_y]) : NULL;
    quant_methods.coeff_cost = coeff_cost;
    quant_methods.pos_scan   = currMB->is_field_mode ? FIELD_SCAN8x8 : SNGL_SCAN8x8;    
    quant_methods.c_cost     = COEFF_COST8x8[currSlice->disthres];

    // Forward 8x8 transform
    forward8x8(mb_ores, mb_rres, block_y, block_x);

    // Quantization process
    nonzero = currSlice->quant_8x8(currMB, &mb_rres[block_y], &quant_methods);
  }
  else
  {
    currSlice->cofAC[pl_off][0][0][0] = 0;
  }

  if (nonzero)
  {
    // Inverse 8x8 transform
    inverse8x8(&mb_rres[block_y], &mb_rres[block_y], block_x);

    // generate final block
    sample_reconstruct (&img_enc[currMB->pix_y + block_y], &mb_pred[block_y], &mb_rres[block_y], block_x, currMB->pix_x + block_x, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, max_imgpel_value, DQ_BITS_8);
  }
  else // if (nonzero) => No transformed residual. Just use prediction.
  {
    copy_image_data_8x8 (&img_enc[currMB->pix_y + block_y], &mb_pred[block_y], currMB->pix_x + block_x, block_x);
  }  

  //  Decoded block moved to frame memory
  return nonzero;
}

/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also. Used for CAVLC.
 *
 * \par Input:
 *    b8: Block position inside a macro block (0,1,2,3).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.
 *    coeff_cost: Counter for nonzero coefficients, used to discard expensive levels.
 ************************************************************************
 */
int residual_transform_quant_luma_8x8_cavlc(Macroblock *currMB, ColorPlane pl, int b8, int *coeff_cost, int intra)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int nonzero = FALSE; 

  int block_x = 8*(b8 & 0x01);
  int block_y = 8*(b8 >> 1);
  int pl_off = b8+ (pl<<2);
  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;
  Slice *currSlice = currMB->p_Slice;
  imgpel **mb_pred = currSlice->mb_pred[pl];  
  int    **mb_ores = currSlice->mb_ores[pl];   
  int    **mb_rres = currSlice->mb_rres[pl];   

  int max_imgpel_value   = p_Vid->max_imgpel_value;
  
  int qp = currMB->qp_scaled[pl];

  //if (check_zero(&mb_ores[block_y], block_x) != 0)
  {
    // Variable p_Quant and some of its parameters could be all set outside 
    // to speed up the code (e.g. field mode, coeff_cost, etc). 
    QuantParameters *p_Quant = p_Vid->p_Quant;

    QuantMethods quant_methods;
    quant_methods.block_x    = block_x;
    quant_methods.block_y    = block_y;
    quant_methods.qp         = qp;
    quant_methods.q_params   = p_Quant->q_params_8x8[pl][intra][qp]; 
    quant_methods.fadjust    = p_Vid->AdaptiveRounding ? (&p_Vid->ARCofAdj8x8[pl][currMB->ar_mode][block_y]) : NULL;
    quant_methods.coeff_cost = coeff_cost;
    // quant_methods.pos_scan   = currMB->is_field_mode ? FIELD_SCAN8x8 : SNGL_SCAN8x8;    
    quant_methods.pos_scan   = currMB->is_field_mode ? FIELD_SCAN8x8_CAVLC : SNGL_SCAN8x8_CAVLC;    
    quant_methods.c_cost     = COEFF_COST8x8[currSlice->disthres];

    // Forward 8x8 transform
    forward8x8(mb_ores, mb_rres, block_y, block_x);

    // Quantization process
    nonzero = currSlice->quant_8x8cavlc(currMB, &mb_rres[block_y], &quant_methods, currSlice->cofAC[pl_off]);
  }

  if (nonzero)
  {
    // Inverse 8x8 transform
    inverse8x8(&mb_rres[block_y], &mb_rres[block_y], block_x);

    // generate final block
    sample_reconstruct (&img_enc[currMB->pix_y + block_y], &mb_pred[block_y], &mb_rres[block_y], block_x, currMB->pix_x + block_x, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, max_imgpel_value, DQ_BITS_8);
  }
  else // if (nonzero) => No transformed residual. Just use prediction.
  {
    copy_image_data_8x8(&img_enc[currMB->pix_y + block_y], &mb_pred[block_y], currMB->pix_x + block_x, block_x);
  }  

  //  Decoded block moved to frame memory
  return nonzero;
}

int residual_transform_quant_luma_8x8_ls(Macroblock *currMB, ColorPlane pl, int b8, int *coeff_cost, int intra)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int i,j,coeff_ctr;
  int scan_pos = 0,run = -1;
  int nonzero = FALSE;  

  int block_x = 8*(b8 & 0x01);
  int block_y = 8*(b8 >> 1);
  int pl_off = b8 + (pl<<2);
  Slice *currSlice = currMB->p_Slice;
  int*  ACLevel = currSlice->cofAC[pl_off][0][0];
  int*  ACRun   = currSlice->cofAC[pl_off][0][1];  
  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;
  imgpel **mb_pred = currSlice->mb_pred[pl];
  int    **mb_ores = currSlice->mb_ores[pl];
  int    **mb_rres = currSlice->mb_rres[pl];

  int scan_poss[4] = { 0 }, runs[4] = { -1, -1, -1, -1 };
  int MCcoeff = 0;
  int *m7;
  int is_cavlc = (currSlice->symbol_mode == CAVLC);

  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN8x8 : SNGL_SCAN8x8;

  int **fadjust8x8 = p_Vid->AdaptiveRounding ? (&p_Vid->ARCofAdj8x8[pl][currMB->ar_mode][block_y]) :NULL;
  
  runs[0]=runs[1]=runs[2]=runs[3]=-1;
  scan_poss[0] = scan_poss[1] = scan_poss[2] = scan_poss[3] = 0;

  if( (currMB->ipmode_DPCM < 2)&&(intra))
  {
    Residual_DPCM_8x8(currMB->ipmode_DPCM, mb_ores, mb_rres, block_y, block_x);
  }
  else
  { 
    for (j = block_y ; j < block_y + BLOCK_SIZE_8x8 ; j ++)
      for (i = block_x ; i < block_x + BLOCK_SIZE_8x8 ; i ++)
        mb_rres[j][i] = mb_ores[j][i] ;
  }

  for (coeff_ctr=0; coeff_ctr < 64; coeff_ctr++)
  {
    i=pos_scan[coeff_ctr][0];
    j=pos_scan[coeff_ctr][1];

    run++;

    if (currMB->luma_transform_size_8x8_flag && is_cavlc)
    {
      MCcoeff = (coeff_ctr & 3);
      runs[MCcoeff]++;
    }

    m7 = &mb_rres[block_y + j][block_x + i];

    if (p_Vid->AdaptiveRounding)
    {
      fadjust8x8[j][block_x+i] = 0;
    }

    if (*m7 != 0)
    {
      nonzero = TRUE;

      if (currMB->luma_transform_size_8x8_flag && is_cavlc)
      {
        *m7 = iClip3(-CAVLC_LEVEL_LIMIT, CAVLC_LEVEL_LIMIT, *m7);
        *coeff_cost += MAX_VALUE;

        currSlice->cofAC[pl_off][MCcoeff][0][scan_poss[MCcoeff]  ] = *m7;
        currSlice->cofAC[pl_off][MCcoeff][1][scan_poss[MCcoeff]++] = runs[MCcoeff];
        ++scan_pos;
        runs[MCcoeff]=-1;
      }
      else
      {
        *coeff_cost += MAX_VALUE;
        ACLevel[scan_pos  ] = *m7;
        ACRun  [scan_pos++] = run;
        run=-1;                     // reset zero level counter
      }
    }
  }

  if (!currMB->luma_transform_size_8x8_flag || !is_cavlc)
    ACLevel[scan_pos] = 0;
  else
  {
    for(i=0; i<4; i++)
      currSlice->cofAC[pl_off][i][0][scan_poss[i]] = 0;
  }

  if( (currMB->ipmode_DPCM < 2) && (intra))
  {
    Inv_Residual_DPCM_8x8(currMB, mb_rres, block_y, block_x);
  }

  for( j=block_y; j<block_y + BLOCK_SIZE_8x8; j++)
  {            
    for( i=block_x; i< block_x + BLOCK_SIZE_8x8; i++)
    {
      mb_rres[j][i] += (int) mb_pred[j][i];
      img_enc[currMB->pix_y + j][currMB->pix_x + i]= (imgpel) mb_rres[j][i];
    }
  }  

  //  Decoded block moved to frame memory
  return nonzero;
}


/*static inline void compute_diff(int *diff, imgpel *cimg, imgpel *cmpr, int width)
{
  int i;
  for (i = 0; i < width; i++)
  {
    *(diff++) = *(cimg++) - *(cmpr++);
  }
}*/

/*!
*************************************************************************************
* \brief
*     distortion for an 8x8 Intra block 
*************************************************************************************
*/
distblk compute_comp8x8_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **mpr8x8, int pic_opix_x, distblk min_cost)
{
  short diff64[64];

  int i, j;
  short *diff = &diff64[0];
  imgpel *cimg, *cmpr;

  for (j=0; j<8; j++)
  {
  //  compute_diff(diff, &cur_img[j][pic_opix_x], &mpr8x8[j][0], BLOCK_SIZE_8x8);

    cimg = &cur_img[j][pic_opix_x];
    cmpr = &mpr8x8[j][0];
    for (i=0; i<8; i++)
    {
      *diff++ = *cimg++ - *cmpr++;
    }

  }
  return p_Vid->distortion8x8 (diff64, min_cost);
}


/*!
*************************************************************************************
* \brief
*     SAD distortion for an 8x8 Intra block 
*************************************************************************************
*/
distblk compute_sad8x8_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **mpr8x8, int pic_opix_x, distblk min_cost)
{
  imgpel *cimg, *cmpr;
  int i32Cost = 0;
  int i, j;
  int imin_cost = dist_down(min_cost);

  for (j=0; j<8; j++)
  {    
    cimg = &cur_img[j][pic_opix_x];
    cmpr = &mpr8x8[j][0];
    for (i=0; i<8; i++)
    {
      i32Cost += iabs(*cimg++ - *cmpr++);
    }

    if (i32Cost > imin_cost)
    {
      return min_cost;
    }
  }
  return dist_scale(i32Cost);
}

/*!
*************************************************************************************
* \brief
*     SSE distortion for an 8x8 Intra block 
*************************************************************************************
*/
distblk compute_sse8x8_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **mpr8x8, int pic_opix_x, distblk min_cost)
{
  int i, j;
  imgpel *cimg, *cmpr;
  int imin_cost = dist_down(min_cost);
  int distortion = 0;

  for (j=0; j<8; j++)
  {
    cimg = &cur_img[j][pic_opix_x];
    cmpr = &mpr8x8[j][0];

    for (i=0; i<8; i++)
    {
      distortion += iabs2(*cimg++ - *cmpr++);
    }

    if (distortion > imin_cost)
    {
      return min_cost;
    }
  }
  return dist_scale(distortion);
}
/*!
*************************************************************************************
* \brief
*     SATD distortion for an 8x8 Intra block 
*************************************************************************************
*/
distblk compute_satd8x8_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **mpr8x8, int pic_opix_x, distblk min_cost)
{
  int i, j;
  short diff64[64];

  short *diff = &diff64[0];
  imgpel *cimg, *cmpr;

  for (j=0; j<8; j++)
  {
    cimg = &cur_img[j][pic_opix_x];
    cmpr = &mpr8x8[j][0];
    for (i=0; i<8; i++)
    {
      *diff++ = *cimg++ - *cmpr++;
    }
  }

  return (dist_scale(HadamardSAD8x8 (diff64)));
}

