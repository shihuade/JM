
/*!
 *************************************************************************************
 * \file block.c
 *
 * \brief
 *    Process one block
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Inge Lille-Langoy               <inge.lille-langoy@telenor.com>
 *    - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *    - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *    - Jani Lainema                    <jani.lainema@nokia.com>
 *    - Detlev Marpe
 *    - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *    - Ragip Kurceren                  <ragip.kurceren@nokia.com>
 *    - Greg Conklin                    <gregc@real.com>
 *    - Alexis Michael Tourapis         <alexismt@ieee.org>
 *************************************************************************************
 */

#include "contributors.h"

#include <math.h>

#include "global.h"
#include "blk_prediction.h"
#include "enc_statistics.h"
#include "memalloc.h"
#include "image.h"
#include "mb_access.h"
#include "block.h"
#include "vlc.h"
#include "transform.h"
#include "mc_prediction.h"
#include "q_offsets.h"
#include "q_matrix.h"
#include "quant4x4.h"
#include "quantChroma.h"
#include "md_common.h"
#include "transform8x8.h"


// Notation for comments regarding prediction and predictors.
// The pels of the 4x4 block are labelled a..p. The predictor pels above
// are labelled A..H, from the left I..P, and from above left X, as follows:
//
//  X A B C D E F G H
//  I a b c d
//  J e f g h
//  K i j k l
//  L m n o p
//

// Predictor array index definitions
#define P_X (PredPel[0])
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

//! array used to find expensive coefficients
static const byte COEFF_COST4x4[3][16] =
{
  {3,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0},
  {9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9},
  {3,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0},
};

static const byte SCAN_YUV420  [4][2] =
{
  {0,0},
  {0,1},
  {0,2},
  {0,3}
};

//! single scan pattern
static const byte SCAN_YUV422  [8][2] =
{
  {0,0},{0,1},
  {1,0},{0,2},
  {0,3},{1,1},
  {1,2},{1,3}
};

//! look up tables for FRExt-chroma support
static const unsigned char hor_offset[4][4][4] =  {
  {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
  },
  {
    {0, 4, 0, 4},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
  },
  {
    {0, 4, 0, 4},
    {0, 4, 0, 4},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
  },
  {
    {0, 4, 0, 4},
    {8,12, 8,12},
    {0, 4, 0, 4},
    {8,12, 8,12}
  }
};

static const unsigned char ver_offset[4][4][4] =  { 
  {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
  },
  {
    {0, 0, 4, 4},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
  },
  {
    {0, 0, 4, 4},
    {8, 8,12,12},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
  },
  {
    {0, 0, 4, 4},
    {0, 0, 4, 4},
    {8, 8,12,12},
    {8, 8,12,12}
  }
};

static const int A[4][4] = {
  { 16, 20, 16, 20},
  { 20, 25, 20, 25},
  { 16, 20, 16, 20},
  { 20, 25, 20, 25}
};

static const unsigned char cbp_blk_chroma[8][4] = {
  {16, 17, 18, 19},
  {20, 21, 22, 23},
  {24, 25, 26, 27},
  {28, 29, 30, 31},
  {32, 33, 34, 35},
  {36, 37, 38, 39},
  {40, 41, 42, 43},
  {44, 45, 46, 47} 
};

//! single scan pattern
static const byte SNGL_SCAN[16][2] =
{
  {0,0},{1,0},{0,1},{0,2},
  {1,1},{2,0},{3,0},{2,1},
  {1,2},{0,3},{1,3},{2,2},
  {3,1},{3,2},{2,3},{3,3}
};

//! field scan pattern
static const byte FIELD_SCAN[16][2] =
{
  {0,0},{0,1},{1,0},{0,2},
  {0,3},{1,1},{1,2},{1,3},
  {2,0},{2,1},{2,2},{2,3},
  {3,0},{3,1},{3,2},{3,3}
};


//For residual DPCM
static int Residual_DPCM_16x16(int **mb_ores, int ipmode);
static int Inv_Residual_DPCM_16x16(int **mb_ores, int ipmode);
static int Residual_DPCM_4x4(int ipmode, int **mb_ores, int **mb_rres, int block_y, int block_x);  
static int Inv_Residual_DPCM_4x4 (Macroblock *currMB, int **m7, int block_y, int block_x);
static int Residual_DPCM_Chroma(int ipmode, int **ores, int **rres, int width, int height);
static int Inv_Residual_DPCM_Chroma(int ipmode, int **m7, int width, int height);

/*!
 ************************************************************************
 * \brief
 *    For new intra pred routines
 *
 * \par Input:
 *    Image par, 16x16 based intra mode
 *
 * \par Output:
 *    none
 ************************************************************************
 */
int residual_transform_quant_luma_16x16(Macroblock *currMB, ColorPlane pl)
{
  int i,j;
  int ii,jj;

  int ac_coef = 0;
  imgpel *img_Y, *predY;
  int nonzero = FALSE;

  int   jpos, ipos;
  int   b8, b4;

  //begin the changes
  int   pl_off = pl<<2;
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  QuantParameters *p_Quant = p_Vid->p_Quant;

  int*  DCLevel = currSlice->cofDC[pl][0];
  int*  DCRun   = currSlice->cofDC[pl][1];
  int   ****cofAC = &currSlice->cofAC[pl_off];
  int coeff_cost;
  int   new_intra_mode      = currMB->i16mode;
  imgpel **img_enc          = p_Vid->enc_picture->p_curr_img;
  int    max_imgpel_value   = p_Vid->max_imgpel_value;
  int    qp                 = (p_Vid->yuv_format==YUV444 && !currSlice->P444_joined)? currMB->qp_scaled[(int)(p_Vid->colour_plane_id)]: currMB->qp_scaled[pl]; //currMB->qp_scaled[pl]; 
  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;
  imgpel  ***curr_mpr_16x16 = currSlice->mpr_16x16[pl];

  int qp_per = p_Quant->qp_per_matrix[qp];

  // select scaling parameters
  // needs a fix for 444 modes (pl should be current color plane)

  QuantMethods quant_methods;
  // set quantization parameters

  quant_methods.qp         = qp; 
  quant_methods.q_params   = p_Quant->q_params_4x4[pl][1][qp]; 
  quant_methods.fadjust    = p_Vid->AdaptiveRounding ? (&p_Vid->ARCofAdj4x4[pl][I16MB][0]): NULL;
  quant_methods.pos_scan   = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;
  quant_methods.c_cost     = COEFF_COST4x4[currSlice->disthres];
  quant_methods.coeff_cost = &coeff_cost;
  quant_methods.type       = LUMA_16AC;


  for (j = 0; j < 16; ++j)
  {
    predY = curr_mpr_16x16[new_intra_mode][j];
    img_Y = &p_Vid->pCurImg[currMB->opix_y + j][currMB->pix_x];
    for (i = 0; i < 16; ++i)
    {
      currSlice->tblk16x16[j][i] = img_Y[i] - predY[i];
    }
  }

  // forward 4x4 integer transform
  for (j = 0; j < 16; j+=4)
  {
    for (i = 0;i < 16; i+=4)
    {
      forward4x4(currSlice->tblk16x16, currSlice->tblk16x16, j, i);
    }
  }

  // pick out DC coeff
  for (j = 0; j < 4; ++j)
    for (i = 0; i < 4; ++i)
      currSlice->tblk4x4[j][i]= currSlice->tblk16x16[j << 2][i << 2];

  // hadamard of DC coefficients
  hadamard4x4(currSlice->tblk4x4, currSlice->tblk4x4);

  nonzero = currSlice->quant_dc4x4(currMB, &currSlice->tblk4x4[0], qp, DCLevel, DCRun, &quant_methods.q_params[0][0], pos_scan);


  // inverse DC transform
  if (nonzero)
  {
    ihadamard4x4(currSlice->tblk4x4, currSlice->tblk4x4);

    // inverse quantization for the DC coefficients
    for (j = 0; j < MB_BLOCK_SIZE; j += BLOCK_SIZE)
    {
      for (i = 0; i < MB_BLOCK_SIZE;i += BLOCK_SIZE)
      {
        currSlice->tblk16x16[j][i] = rshift_rnd_sf(((currSlice->tblk4x4[j>>2][i>>2]) * quant_methods.q_params[0][0].InvScaleComp) << qp_per, 6);
      }
    }
  }
  else // All DC equal to 0.
  {
    for (j = 0; j < MB_BLOCK_SIZE; j += BLOCK_SIZE)
    {
      for (i = 0; i < MB_BLOCK_SIZE; i += BLOCK_SIZE)
      {
        currSlice->tblk16x16[j][i] = 0;
      }
    }
  }

  // AC processing for MB
  for (jj=0;jj<4;++jj)
  {
    jpos = (jj << 2);
    currMB->subblock_y = (short) jpos;
    for (ii=0;ii<4;++ii)
    {
      ipos = (ii << 2);
      currMB->subblock_x = (short) ipos;

      b8       = 2*(jj >> 1) + (ii >> 1);
      b4       = 2*(jj & 0x01) + (ii & 0x01);
      quant_methods.block_x = ipos;
      quant_methods.block_y = jpos;

      quant_methods.ACLevel  = cofAC[b8][b4][0];
      quant_methods.ACRun    = cofAC[b8][b4][1];

      // Quantization process
      nonzero = currSlice->quant_ac4x4(currMB, &currSlice->tblk16x16[jpos], &quant_methods);

      if (nonzero)
        ac_coef = 15;

      //inverse transform
      if (currSlice->tblk16x16[jpos][ipos]!= 0 || nonzero)
        inverse4x4(currSlice->tblk16x16, currSlice->tblk16x16, jpos, ipos);
    }
  }


  // Reconstruct samples
  sample_reconstruct (&img_enc[currMB->pix_y], curr_mpr_16x16[new_intra_mode], currSlice->tblk16x16, 0, currMB->pix_x, 16, 16, max_imgpel_value, DQ_BITS);

  if(currSlice->slice_type == SP_SLICE || currSlice->slice_type == SI_SLICE)
  {
    for (j = currMB->pix_y; j < currMB->pix_y + 16;++j)
      for (i = currMB->pix_x; i < currMB->pix_x + 16;++i)
        p_Vid->lrec[j][i]=-16; //signals an I16 block in the SP frame
  }

  return ac_coef;
}

static int Residual_DPCM_16x16(int **m7, int ipmode)
{
  int i,j;
  int temp[16][16];

  if(ipmode==VERT_PRED_16)
  {   
    for (i=1; i<16; ++i) 
      for (j=0; j<16; ++j)
        temp[i][j] = m7[i][j] - m7[i-1][j];

    for (i=1; i<16; ++i)
      for (j=0; j<16; ++j)
        m7[i][j] = temp[i][j];
  }
  else  //HOR_PRED_16
  {
    for (i=0; i<16; ++i)
      for (j=1; j<16; ++j)
        temp[i][j] = m7[i][j] - m7[i][j-1];

    for (i=0; i<16; ++i)
      for (j=1; j<16; ++j)
        m7[i][j] = temp[i][j];
  }
  return 0;
}

static int Inv_Residual_DPCM_16x16(int **m7, int ipmode)
{
  int i;
  int temp[16][16];

  if(ipmode==VERT_PRED_16)
  {
    for (i=0; i<16; ++i) 
    {
      temp[0][i] = m7[0][i];
      temp[1][i] = m7[1][i] + temp[0][i];
      temp[2][i] = m7[2][i] + temp[1][i];
      temp[3][i] = m7[3][i] + temp[2][i];
      temp[4][i] = m7[4][i] + temp[3][i];
      temp[5][i] = m7[5][i] + temp[4][i];
      temp[6][i] = m7[6][i] + temp[5][i];
      temp[7][i] = m7[7][i] + temp[6][i];
      temp[8][i] = m7[8][i] + temp[7][i];
      temp[9][i] = m7[9][i] + temp[8][i];
      temp[10][i] = m7[10][i] + temp[9][i];
      temp[11][i] = m7[11][i] + temp[10][i];
      temp[12][i] = m7[12][i] + temp[11][i];
      temp[13][i] = m7[13][i] + temp[12][i];
      temp[14][i] = m7[14][i] + temp[13][i];
      temp[15][i] = m7[15][i] + temp[14][i];
    }
    // These could now just use a memcpy
    for (i=0; i<16; ++i)
    {
      m7[1][i] = temp[1][i];
      m7[2][i] = temp[2][i];
      m7[3][i] = temp[3][i];
      m7[4][i] = temp[4][i];
      m7[5][i] = temp[5][i];
      m7[6][i] = temp[6][i];
      m7[7][i] = temp[7][i];
      m7[8][i] = temp[8][i];
      m7[9][i] = temp[9][i];
      m7[10][i] = temp[10][i];
      m7[11][i] = temp[11][i];
      m7[12][i] = temp[12][i];
      m7[13][i] = temp[13][i];
      m7[14][i] = temp[14][i];
      m7[15][i] = temp[15][i];
    }
  }
  else  //HOR_PRED_16
  {
    for(i=0; i<16; ++i)
    {
      temp[i][0] = m7[i][0];
      temp[i][1] = m7[i][1] + temp[i][0];
      temp[i][2] = m7[i][2] + temp[i][1];
      temp[i][3] = m7[i][3] + temp[i][2];
      temp[i][4] = m7[i][4] + temp[i][3];
      temp[i][5] = m7[i][5] + temp[i][4];
      temp[i][6] = m7[i][6] + temp[i][5];
      temp[i][7] = m7[i][7] + temp[i][6];
      temp[i][8] = m7[i][8] + temp[i][7];
      temp[i][9] = m7[i][9] + temp[i][8];
      temp[i][10] = m7[i][10] + temp[i][9];
      temp[i][11] = m7[i][11] + temp[i][10];
      temp[i][12] = m7[i][12] + temp[i][11];
      temp[i][13] = m7[i][13] + temp[i][12];
      temp[i][14] = m7[i][14] + temp[i][13];
      temp[i][15] = m7[i][15] + temp[i][14];
    }
    for (i=0; i<16; ++i)
    {
      m7[i][1] = temp[i][1];
      m7[i][2] = temp[i][2];
      m7[i][3] = temp[i][3];
      m7[i][4] = temp[i][4];
      m7[i][5] = temp[i][5];
      m7[i][6] = temp[i][6];
      m7[i][7] = temp[i][7];
      m7[i][8] = temp[i][8];
      m7[i][9] = temp[i][9];
      m7[i][10] = temp[i][10];
      m7[i][11] = temp[i][11];
      m7[i][12] = temp[i][12];
      m7[i][13] = temp[i][13];
      m7[i][14] = temp[i][14];
      m7[i][15] = temp[i][15];
    }    
  }
  return 0;
}


/*!
 ************************************************************************
 * \brief
 *    For new intra pred routines
 *
 * \par Input:
 *    Image par, 16x16 based intra mode
 *
 * \par Output:
 *    none
 ************************************************************************
 */
int residual_transform_quant_luma_16x16_ls(Macroblock *currMB, ColorPlane pl)
{
  int i,j;
  int ii,jj;

  int run,scan_pos,coeff_ctr;
  int ac_coef = 0;
  imgpel *img_Y, *predY;

  int   b8, b4;

  //begin the changes
  int   pl_off = pl<<2;
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  Boolean is_cavlc = (Boolean) (currSlice->symbol_mode == CAVLC);
  int*  DCLevel = currSlice->cofDC[pl][0];
  int*  DCRun   = currSlice->cofDC[pl][1];
  int*  ACLevel;
  int*  ACRun;  
  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;
  int   *m7;
  int   new_intra_mode = currMB->i16mode;

  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;
  imgpel  ***curr_mpr_16x16  = currSlice->mpr_16x16[pl];

  for (j = 0; j < 16; ++j)
  {
    predY = curr_mpr_16x16[new_intra_mode][j];
    img_Y = &p_Vid->pCurImg[currMB->opix_y + j][currMB->pix_x];
    for (i = 0; i < 16; ++i)
    {
      currSlice->tblk16x16[j][i] = img_Y[i] - predY[i];
    }
  }

  if (new_intra_mode < 2)
  {
    Residual_DPCM_16x16(currSlice->tblk16x16, new_intra_mode);
  }

  // pick out DC coeff
  for (j = 0; j < 4;++j)
    for (i = 0; i < 4;++i)
      currSlice->tblk4x4[j][i]= currSlice->tblk16x16[j << 2][i << 2];

  run=-1;
  scan_pos=0;

  for (coeff_ctr=0;coeff_ctr<16;++coeff_ctr)
  {
    i=pos_scan[coeff_ctr][0];
    j=pos_scan[coeff_ctr][1];

    ++run;

    m7 = &currSlice->tblk4x4[j][i];

    if (*m7 != 0)
    {
      if (is_cavlc)
        *m7 = iClip3(-CAVLC_LEVEL_LIMIT, CAVLC_LEVEL_LIMIT, *m7);

      DCLevel[scan_pos  ] = *m7;
      DCRun  [scan_pos++] = run;
      run=-1;
    }
  }
  DCLevel[scan_pos]=0;

  // replace DC coeff. This is needed in case of out of limits for CAVLC. Could be done only for CAVLC
  for (j = 0; j < 4;++j)
    for (i = 0; i < 4;++i)
      currSlice->tblk16x16[j << 2][i << 2] = currSlice->tblk4x4[j][i];

  // AC inverse trans/quant for MB
  for (jj = 0; jj < 4; ++jj)
  {
    for (ii = 0; ii < 4; ++ii)
    {
      for (j=0;j<4;++j)
      {
        memcpy(currSlice->tblk4x4[j],&currSlice->tblk16x16[(jj<<2)+j][(ii<<2)], BLOCK_SIZE * sizeof(int));
      }

      run      = -1;
      scan_pos =  0;
      b8       = 2*(jj >> 1) + (ii >> 1);
      b4       = 2*(jj & 0x01) + (ii & 0x01);
      ACLevel  = currSlice->cofAC [b8+pl_off][b4][0];
      ACRun    = currSlice->cofAC [b8+pl_off][b4][1];

      for (coeff_ctr=1;coeff_ctr<16;coeff_ctr++) // set in AC coeff
      {
        i=pos_scan[coeff_ctr][0];
        j=pos_scan[coeff_ctr][1];

        run++;
        m7 = &currSlice->tblk4x4[j][i];

        if (*m7 != 0)
        {
          if (is_cavlc)
            *m7 = iClip3(-CAVLC_LEVEL_LIMIT, CAVLC_LEVEL_LIMIT, *m7);

          ac_coef = 15;
          ACLevel[scan_pos  ] = *m7;
          ACRun  [scan_pos++] = run;
          run=-1;
        }
        // set adaptive rounding p_Inp to 0 since process is not meaningful here.
      }
      ACLevel[scan_pos] = 0;

      for (j=0;j<4;++j)
        memcpy(&currSlice->tblk16x16[(jj<<2)+j][(ii<<2)],currSlice->tblk4x4[j], BLOCK_SIZE * sizeof(int)); 
    }
  }

  if (new_intra_mode < 2)
  {
    Inv_Residual_DPCM_16x16(currSlice->tblk16x16, new_intra_mode);
  }


  for (j = 0; j < 16; ++j)
  {
    img_Y = &img_enc[currMB->pix_y + j][currMB->pix_x];
    predY = curr_mpr_16x16[new_intra_mode][j];        
    for (i = 0; i < 16; ++i)
      img_Y[i]=(imgpel)(currSlice->tblk16x16[j][i] + predY[i]);
  }

  if(currSlice->slice_type == SP_SLICE || currSlice->slice_type == SI_SLICE)
  {
    for (j = currMB->pix_y; j < currMB->pix_y + 16;++j)
      for (i = currMB->pix_x; i < currMB->pix_x + 16;++i)
        p_Vid->lrec[j][i]=-16; //signals an I16 block in the SP frame
  }

  return ac_coef;
}

static inline int check_zero(int **mb_ores, int block_x)
{
  int i, j, k = 0;

  for (j = 0; (j < BLOCK_SIZE) && (k == 0); ++j)
  {
    for (i = block_x; (i< block_x + BLOCK_SIZE) && (k == 0); ++i)
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
*    The routine performs transform,quantization,inverse transform, 
*    adds the diff to the prediction and writes the result to the 
*    decoded luma frame. 
*
* \par Input:
*    currMB:          Current macroblock.
*    pl:              Color plane for 4:4:4 coding.
*    block_x,block_y: Block position inside a macro block (0,4,8,12).
*    intra:           Intra block indicator.
*
* \par Output_
*    nonzero:         0 if no levels are nonzero. \n
*                     1 if there are nonzero levels.\n
*    coeff_cost:      Coeff coding cost for thresholding consideration.\n
************************************************************************
*/
int residual_transform_quant_luma_4x4(Macroblock *currMB, ColorPlane pl, int block_x,int block_y, int *coeff_cost, int intra)
{
  int nonzero = FALSE;

  int   pos_x   = block_x >> BLOCK_SHIFT;
  int   pos_y   = block_y >> BLOCK_SHIFT;
  int   b8      = 2*(pos_y >> 1) + (pos_x >> 1) + (pl<<2);
  int   b4      = 2*(pos_y & 0x01) + (pos_x & 0x01);
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;

  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;
  imgpel **mb_pred = currSlice->mb_pred[pl];
  int    **mb_ores = currSlice->mb_ores[pl];  

  if (check_zero(&mb_ores[block_y], block_x) != 0) // check if any coefficients in block
  {
    int   **mb_rres = currSlice->mb_rres[pl];   
    int   max_imgpel_value = p_Vid->max_imgpel_value;
    int   qp = (p_Vid->yuv_format==YUV444 && !currSlice->P444_joined)? currMB->qp_scaled[(int)(p_Vid->colour_plane_id)]: currMB->qp_scaled[pl]; 
    QuantParameters   *p_Quant = p_Vid->p_Quant;
    QuantMethods quant_methods;
    quant_methods.ACLevel = currSlice->cofAC[b8][b4][0];
    quant_methods.ACRun   = currSlice->cofAC[b8][b4][1];

    quant_methods.block_x    = block_x;
    quant_methods.block_y    = block_y;
    quant_methods.qp         = qp;
    quant_methods.q_params   = p_Quant->q_params_4x4[pl][intra][qp]; 
    quant_methods.fadjust    = p_Vid->AdaptiveRounding ? (&p_Vid->ARCofAdj4x4[pl][currMB->ar_mode][block_y]) : NULL;
    quant_methods.coeff_cost = coeff_cost;
    quant_methods.pos_scan   = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;    
    quant_methods.c_cost     = COEFF_COST4x4[currSlice->disthres];

    currMB->subblock_x = ((b8&0x1)==0) ? (((b4&0x1)==0)? 0: 4) : (((b4&0x1)==0)? 8: 12); // horiz. position for coeff_count context
    currMB->subblock_y = (b8<2)        ? ((b4<2)       ? 0: 4) : ((b4<2)       ? 8: 12); // vert.  position for coeff_count context

    //  Forward 4x4 transform
    forward4x4(mb_ores, currSlice->tblk16x16, block_y, block_x);

    // Quantization process
    nonzero = currSlice->quant_4x4(currMB, &currSlice->tblk16x16[block_y], &quant_methods);

    //  Decoded block moved to frame memory
    if (nonzero)
    {
      // Inverse 4x4 transform
      inverse4x4(currSlice->tblk16x16, mb_rres, block_y, block_x);

      // generate final block
      sample_reconstruct (&img_enc[currMB->pix_y + block_y], &mb_pred[block_y], &mb_rres[block_y], block_x, currMB->pix_x + block_x, BLOCK_SIZE, BLOCK_SIZE, max_imgpel_value, DQ_BITS);
    }
    else // if (nonzero) => No transformed residual. Just use prediction.
    {
      copy_image_data_4x4(&img_enc[currMB->pix_y + block_y], &mb_pred[block_y], currMB->pix_x + block_x, block_x);
    }
  }
  else
  {
    currSlice->cofAC[b8][b4][0][0] = 0;
    copy_image_data_4x4(&img_enc[currMB->pix_y + block_y], &mb_pred[block_y], currMB->pix_x + block_x, block_x);
  }

  return nonzero;
}



/*!
************************************************************************
* \brief
*    Process for lossless coding of coefficients.
*    The routine performs transform, quantization,inverse transform, 
*    adds the diff to the prediction and writes the result to the 
*    decoded luma frame. 
*
* \par Input:
*    currMB:          Current macroblock.
*    pl:              Color plane for 4:4:4 coding.
*    block_x,block_y: Block position inside a macro block (0,4,8,12).
*    intra:           Intra block indicator.
*
* \par Output_
*    nonzero:         0 if no levels are nonzero. \n
*                     1 if there are nonzero levels.\n
*    coeff_cost:      Coeff coding cost for thresholding consideration.\n
************************************************************************
*/
int residual_transform_quant_luma_4x4_ls(Macroblock *currMB, ColorPlane pl, int block_x,int block_y,int *coeff_cost, int intra)
{
  int i,j, coeff_ctr;
  int run = -1;
  int nonzero = FALSE;  

  int   pos_x   = block_x >> BLOCK_SHIFT;
  int   pos_y   = block_y >> BLOCK_SHIFT;
  int   b8      = 2*(pos_y >> 1) + (pos_x >> 1) + (pl<<2);
  int   b4      = 2*(pos_y & 0x01) + (pos_x & 0x01);
  
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  Boolean is_cavlc = (Boolean) (currSlice->symbol_mode == CAVLC);
  int*  ACL = &currSlice->cofAC[b8][b4][0][0];
  int*  ACR = &currSlice->cofAC[b8][b4][1][0];

  int   pix_y, pix_x;
  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;
  imgpel **mb_pred = currSlice->mb_pred[pl];
  int    **mb_ores = currSlice->mb_ores[pl];
  int    **mb_rres = currSlice->mb_rres[pl]; 
  int   *m7;

  const byte *p_scan = currMB->is_field_mode ? &FIELD_SCAN[0][0] : &SNGL_SCAN[0][0];

  // select scaling parameters
  int **fadjust4x4    = p_Vid->AdaptiveRounding ? (&p_Vid->ARCofAdj4x4[pl][currMB->ar_mode][block_y]) : NULL;

  if( (currMB->ipmode_DPCM < 2) && (intra))
  {
    Residual_DPCM_4x4(currMB->ipmode_DPCM, mb_ores, mb_rres, block_y, block_x);
  }
  else
  {
    for (j=block_y; j < block_y + BLOCK_SIZE; ++j)
      for (i=block_x; i < block_x + BLOCK_SIZE; ++i)
        mb_rres[j][i] = mb_ores[j][i];
  }

  for (coeff_ctr=0;coeff_ctr < 16;coeff_ctr++)
  {
    i = *p_scan++;
    j = *p_scan++;

    run++;

    m7 = &mb_rres[block_y + j][block_x + i]; 

    if (p_Vid->AdaptiveRounding)
      fadjust4x4[j][block_x+i] = 0;

    if (*m7 != 0)
    {
      if (is_cavlc)
        *m7 = iClip3(-CAVLC_LEVEL_LIMIT, CAVLC_LEVEL_LIMIT, *m7);

      nonzero=TRUE;
      *coeff_cost += MAX_VALUE;
      *ACL++ = *m7;
      *ACR++ = run;
      run=-1;                     // reset zero level counter        
    }
  }
  *ACL = 0;

  if( (currMB->ipmode_DPCM < 2) && (intra))
  {
    Inv_Residual_DPCM_4x4(currMB, mb_rres, block_y, block_x);
  }

  for (j=0; j < BLOCK_SIZE; ++j)
  {
    pix_y = currMB->pix_y + block_y + j;
    pix_x = currMB->pix_x+block_x;
    for (i=0; i < BLOCK_SIZE; ++i)
    {
      img_enc[pix_y][pix_x+i] = (imgpel) (mb_rres[j+block_y][i+block_x] + mb_pred[j+block_y][i+block_x]);
    }
  }

  return nonzero;
}

/*!
************************************************************************
* \brief
*    Residual DPCM for Intra lossless coding
*
* \par Input:
*    block_x,block_y: Block position inside a macro block (0,4,8,12).
************************************************************************
*/
static int Residual_DPCM_4x4(int ipmode, int **mb_ores, int **mb_rres, int block_y, int block_x)
{
  int i;
  int temp[4][4];

  if(ipmode==VERT_PRED)
  {
    for (i=0; i<4; ++i)
    {
      temp[0][i] = mb_ores[block_y + 0][block_x + i];
      temp[1][i] = mb_ores[block_y + 1][block_x + i] - mb_ores[block_y    ][block_x + i];
      temp[2][i] = mb_ores[block_y + 2][block_x + i] - mb_ores[block_y + 1][block_x + i];
      temp[3][i] = mb_ores[block_y + 3][block_x + i] - mb_ores[block_y + 2][block_x + i];
    }

    for (i = 0; i < 4; ++i)
    {
      mb_rres[block_y + 0][block_x + i] = temp[0][i];
      mb_rres[block_y + 1][block_x + i] = temp[1][i];
      mb_rres[block_y + 2][block_x + i] = temp[2][i];
      mb_rres[block_y + 3][block_x + i] = temp[3][i];
    }
  }
  else  //HOR_PRED
  {
    for (i=0; i<4; ++i)
    {
      temp[i][0] = mb_ores[block_y + i][block_x];
      temp[i][1] = mb_ores[block_y + i][block_x + 1] - mb_ores[block_y + i][block_x    ];
      temp[i][2] = mb_ores[block_y + i][block_x + 2] - mb_ores[block_y + i][block_x + 1];
      temp[i][3] = mb_ores[block_y + i][block_x + 3] - mb_ores[block_y + i][block_x + 2];
    }

    for (i=0; i<4; ++i)
    {
      mb_rres[block_y + i][block_x + 0] = temp[i][0];
      mb_rres[block_y + i][block_x + 1] = temp[i][1];
      mb_rres[block_y + i][block_x + 2] = temp[i][2];
      mb_rres[block_y + i][block_x + 3] = temp[i][3];
    }
  }
  return 0;
}

/*!
************************************************************************
* \brief
*    Inverse residual DPCM for Intra lossless coding
*
* \par Input:
*    block_x,block_y: Block position inside a macro block (0,4,8,12).
************************************************************************
*/
//For residual DPCM
static int Inv_Residual_DPCM_4x4(Macroblock *currMB, int **m7, int block_y, int block_x)  
{
  int i;
  int temp[4][4];

  if(currMB->ipmode_DPCM == VERT_PRED)
  {
    for(i=0; i<4; ++i)
    {
      temp[0][i] = m7[block_y + 0][block_x + i];
      temp[1][i] = m7[block_y + 1][block_x + i] + temp[0][i];
      temp[2][i] = m7[block_y + 2][block_x + i] + temp[1][i];
      temp[3][i] = m7[block_y + 3][block_x + i] + temp[2][i];
    }
    for(i=0; i<4; ++i)
    {      
      m7[block_y + 0][block_x + i] = temp[0][i];
      m7[block_y + 1][block_x + i] = temp[1][i];
      m7[block_y + 2][block_x + i] = temp[2][i];
      m7[block_y + 3][block_x + i] = temp[3][i];
    }
  }
  else //HOR_PRED
  {
    for(i=0; i<4; ++i)
    {
      temp[i][0] = m7[block_y + i][block_x + 0];
      temp[i][1] = m7[block_y + i][block_x + 1] + temp[i][0];
      temp[i][2] = m7[block_y + i][block_x + 2] + temp[i][1];
      temp[i][3] = m7[block_y + i][block_x + 3] + temp[i][2];    
    }
    for(i=0; i<4; ++i)
    {
      m7[block_y+i][block_x  ] = temp[i][0];
      m7[block_y+i][block_x+1] = temp[i][1];
      m7[block_y+i][block_x+2] = temp[i][2];
      m7[block_y+i][block_x+3] = temp[i][3];
    }
  }
  return 0;
}

/*!
 ************************************************************************
 * \brief
 *    Transform,quantization,inverse transform for chroma.
 *    The main reason why this is done in a separate routine is the
 *    additional 2x2 transform of DC-coeffs. This routine is called
 *    once for each of the chroma components.
 *
 * \par Input:
 *    uv    : Make difference between the U and V chroma component  \n
 *    cr_cbp: chroma coded block pattern
 *
 * \par Output:
 *    cr_cbp: Updated chroma coded block pattern.
 ************************************************************************
 */
int residual_transform_quant_chroma_4x4(Macroblock *currMB, int uv, int cr_cbp)
{
  int i, j, n2, n1, coeff_ctr;
  int *m1;
  int coeff_cost = 0;
  int cr_cbp_tmp = 0;
  int DCzero = FALSE;
  int nonzero[4][4] = {{FALSE}};
  int nonezero = FALSE;
  //int empty_block = TRUE;
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  QuantParameters *p_Quant = p_Vid->p_Quant;

  int   b4;
  int*  DCLevel = currSlice->cofDC[uv+1][0];
  int*  DCRun   = currSlice->cofDC[uv+1][1];
  int   intra = is_intra (currMB);
  int   uv_scale = uv * (p_Vid->num_blk8x8_uv >> 1);

  //FRExt
  static const int64 cbpblk_pattern[4]={0, 0xf0000, 0xff0000, 0xffff0000};
  int yuv = p_Vid->yuv_format;
  int b8;  

  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;
  int cur_qp = currMB->qpc[uv] + currSlice->bitdepth_chroma_qp_scale;  

  int max_imgpel_value_uv = p_Vid->max_pel_value_comp[uv + 1];

  int    **mb_rres = currSlice->mb_rres[uv + 1]; 
  int    **mb_ores = currSlice->mb_ores[uv + 1];
  imgpel **mb_pred = currSlice->mb_pred[uv + 1]; 

  QuantMethods quant_methods;
  // set quantization parameters
  quant_methods.qp       = cur_qp; 
  quant_methods.q_params = p_Quant->q_params_4x4[uv + 1][intra][cur_qp]; 
  quant_methods.type     = CHROMA_AC;
  if (currMB->mb_type == P8x8 && currMB->luma_transform_size_8x8_flag)
  {
    //P8x8, transform 8x8 chroma adjustments must be stored in a different array to avoid conflict with transform 4x4
    quant_methods.fadjust = p_Vid->AdaptiveRounding ? p_Vid->ARCofAdj4x4[uv + 1][4] : NULL;
  }
  else
  {
    quant_methods.fadjust = p_Vid->AdaptiveRounding ? p_Vid->ARCofAdj4x4[uv + 1][currMB->ar_mode] : NULL;
  }

  quant_methods.pos_scan      = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;
  quant_methods.c_cost        = (currSlice->disthres == 2 && intra ) ? COEFF_COST4x4[1] : COEFF_COST4x4[currSlice->disthres];
  quant_methods.coeff_cost    = &coeff_cost;


  p_Vid->is_v_block = uv;

  //============= integer transform ===============
  for (n2=0; n2 < p_Vid->mb_cr_size_y; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 < p_Vid->mb_cr_size_x; n1 += BLOCK_SIZE)
    {
      if (check_zero(&mb_ores[n2], n1) == 0) // check if any coefficients in block
      {
        for (j = n2; j < n2 + BLOCK_SIZE; j++)
          for (i = n1; i < n1 + BLOCK_SIZE; i++)
            mb_rres[j][i] = 0;
      }
      else
      {
        forward4x4(mb_ores, mb_rres, n2, n1);
        //empty_block = FALSE;
      }
    }
  }

  if (yuv == YUV420)
  {
    int **fadjust2x2 = NULL;

    m1 = currSlice->tblk4x4[0];
    //================== CHROMA DC YUV420 ===================
  
    // forward 2x2 hadamard
    hadamard2x2(mb_rres, m1);

    // Quantization process of chroma 2X2 hadamard transformed DC coeffs.
    DCzero = currSlice->quant_dc_cr(currMB, &m1, cur_qp, DCLevel, DCRun, &quant_methods.q_params[0][0], fadjust2x2, SCAN_YUV420);

    if (DCzero) 
    {
        currMB->cbp_blk |= 0xf0000 << (uv << 2) ;    // if one of the 2x2-DC levels is != 0 set the
        cr_cbp=imax(1,cr_cbp);                     // coded-bit all 4 4x4 blocks (bit 16-19 or 20-23)
    }

    //  Inverse transform of 2x2 DC levels
    ihadamard2x2(m1, m1);

    mb_rres[0][0] = m1[0] >> 5;
    mb_rres[0][4] = m1[1] >> 5;
    mb_rres[4][0] = m1[2] >> 5;
    mb_rres[4][4] = m1[3] >> 5;
  }
  else if (yuv == YUV422)
  {
    int **fadjust4x2 = NULL; // note that this is in fact 2x4 but the coefficients have been transposed for better memory access
    //for YUV422 only
    int cur_qp_dc = currMB->qpc[uv] + 3 + currSlice->bitdepth_chroma_qp_scale;

    LevelQuantParams **quant_paramsDC = p_Quant->q_params_4x4[uv + 1][intra][cur_qp_dc]; 

    //================== CHROMA DC YUV422 ===================
    //pick out DC coeff    
    for (j=0; j < p_Vid->mb_cr_size_y; j+=BLOCK_SIZE)
    {
      for (i=0; i < p_Vid->mb_cr_size_x; i+=BLOCK_SIZE)
        currSlice->tblk4x4[i>>2][j>>2]= mb_rres[j][i];
    }

    // forward hadamard transform. Note that coeffs have been transposed (4x2 instead of 2x4) which makes transform a bit faster
    hadamard4x2(currSlice->tblk4x4, currSlice->tblk4x4);

    // Quantization process of chroma transformed DC coeffs.
    DCzero = currSlice->quant_dc_cr(currMB, currSlice->tblk4x4, cur_qp_dc, DCLevel, DCRun, &quant_paramsDC[0][0], fadjust4x2, SCAN_YUV422);

    if (DCzero)
    {
      currMB->cbp_blk |= 0xff0000 << (uv << 3) ;   // if one of the DC levels is != 0 set the
      cr_cbp=imax(1,cr_cbp);                       // coded-bit all 4 4x4 blocks (bit 16-31 or 32-47) //YUV444
    }

    //inverse DC transform. Note that now currSlice->tblk4x4 is transposed back
    ihadamard4x2(currSlice->tblk4x4, currSlice->tblk4x4);    

    // This code assumes sizeof(int) > 16. Therefore, no need to have conditional
    for (j = 0; j < 4; ++j)
    {
      mb_rres[j << 2 ][0] = rshift_rnd_sf(currSlice->tblk4x4[j][0], 6);
      mb_rres[j << 2 ][4] = rshift_rnd_sf(currSlice->tblk4x4[j][1], 6);
    }
  }

  //     Quant of chroma AC-coeffs.
  for (b8=0; b8 < (p_Vid->num_blk8x8_uv >> 1); b8++)
  {
    for (b4=0; b4 < 4; b4++)
    {
      int64 uv_cbpblk = ((int64)1) << cbp_blk_chroma[b8 + uv_scale][b4];      
      n1 = hor_offset[yuv][b8][b4];
      n2 = ver_offset[yuv][b8][b4];
      quant_methods.ACLevel = currSlice->cofAC[4 + b8 + uv_scale][b4][0];
      quant_methods.ACRun   = currSlice->cofAC[4 + b8 + uv_scale][b4][1];
      quant_methods.block_x  = n1;
      quant_methods.block_y  = n2;

      currMB->subblock_y = subblk_offset_y[p_Vid->yuv_format - 1][b8][b4];
      currMB->subblock_x = subblk_offset_x[p_Vid->yuv_format - 1][b8][b4];
      if (p_Vid->AdaptiveRounding)
      {
        if (currMB->mb_type == P8x8 && currMB->luma_transform_size_8x8_flag)
        {
          //P8x8, transform 8x8 chroma adjustments must be stored in a different array to avoid conflict with transform 4x4
          quant_methods.fadjust = &p_Vid->ARCofAdj4x4[uv + 1][4][n2];
        }
        else
        {
          quant_methods.fadjust = &p_Vid->ARCofAdj4x4[uv + 1][currMB->ar_mode][n2];
        }
      }
      else
      {
        quant_methods.fadjust = NULL;
      }

      // Quantization process
      nonzero[n2>>2][n1>>2] = currSlice->quant_ac4x4cr(currMB, &mb_rres[n2], &quant_methods);

      if (nonzero[n2>>2][n1>>2])
      {
        currMB->cbp_blk |= uv_cbpblk;
        cr_cbp_tmp = 2;
        nonezero = TRUE;
      }
    }
  }

  // Perform thresholding
  // * reset chroma coeffs
  if(nonezero && coeff_cost < _CHROMA_COEFF_COST_)
  {
    int64 uv_cbpblk = ((int64)cbpblk_pattern[yuv] << (uv << (1+yuv)));
    cr_cbp_tmp = 0;

    for (b8 = 0; b8 < (p_Vid->num_blk8x8_uv >> 1); b8++)
    {
      for (b4 = 0; b4 < 4; b4++)
      {
        n1 = hor_offset[yuv][b8][b4];
        n2 = ver_offset[yuv][b8][b4];
        if (nonzero[n2>>2][n1>>2] == TRUE)
        {
          nonzero[n2>>2][n1>>2] = FALSE;
          quant_methods.ACLevel = currSlice->cofAC[4 + b8 + uv_scale][b4][0];
          quant_methods.ACRun   = currSlice->cofAC[4 + b8 + uv_scale][b4][1];

          if (DCzero == 0)
            currMB->cbp_blk &= ~(uv_cbpblk);  // if no chroma DC's: then reset coded-bits of this chroma subblock

          quant_methods.ACLevel[0] = 0;

          for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// ac coeff
          {
            mb_rres[n2 + pos_scan[coeff_ctr][1]][n1 + pos_scan[coeff_ctr][0]] = 0;
            quant_methods.ACLevel[coeff_ctr]  = 0;
          }
        }
      }
    }
  }

  //     inverse transform.
  //     Horizontal.
  if(cr_cbp_tmp == 2)
    cr_cbp = 2;

  nonezero = FALSE;
  for (n2=0; n2 < p_Vid->mb_cr_size_y; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 < p_Vid->mb_cr_size_x; n1 += BLOCK_SIZE)
    {
      if (mb_rres[n2][n1] != 0 || nonzero[n2>>2][n1>>2] == TRUE)
      {
        inverse4x4(mb_rres, mb_rres, n2, n1);
        nonezero = TRUE;
      }
    }
  }

  //  Decoded block moved to memory
  if (nonezero == TRUE)
  {
    sample_reconstruct (&p_Vid->enc_picture->imgUV[uv][currMB->pix_c_y], mb_pred, mb_rres, 0, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y, max_imgpel_value_uv, DQ_BITS);
  }
  else
  {
    copy_image_data(&p_Vid->enc_picture->imgUV[uv][currMB->pix_c_y], mb_pred, currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
  }

  return cr_cbp;
}

static int Residual_DPCM_Chroma(int ipmode, int **ores, int **rres, int width, int height)
{
  int i,j;
  int temp[16][16];

  if(ipmode==VERT_PRED_8)
  { 
    for (j=0; j<width; j++)
      temp[0][j] = ores[0][j];

    for (j=0; j<width; j++)
      for (i=1; i<height; i++) 
        temp[i][j] =  ores[i][j] - ores[i-1][j];

    for (i = 0; i < height; i++)
      for (j = 0; j < width; j++)
        rres[i][j] = temp[i][j];
  }
  else  //HOR_PRED_8
  {
    for (i=0; i<height; i++)
      temp[i][0] = ores[i][0];

    for (i=0; i<height; i++)
      for (j=1; j<width; j++)
        temp[i][j] = ores[i][j] - ores[i][j-1];

    for (i=0; i<height; i++)
      for (j=0; j<width; j++)
        rres[i][j] = temp[i][j];
  }

  return 0;
}

static int Inv_Residual_DPCM_Chroma(int ipmode, int **m7, int width, int height)  
{
  int i, j;
  int temp[16][16];

  if(ipmode == VERT_PRED_8)
  {
    for(i=0; i<width; i++)
    {
      temp[0][i] = m7[0][i];
      for(j=1; j<height; j++)
      {
        temp[j][i] = temp[j-1][i] + m7[j][i];
      }
    }
    for(i=0; i<height; i++)
    {
      for(j = 0; j < width; j++)
      {
        m7[i][j] = temp[i][j];
      }
    }
  }
  else //HOR_PRED_8
  {
    for(i=0; i<height; i++)
    {
      temp[i][0] = m7[i][0];
      for(j = 1; j < width; j++)
      {
        temp[i][j] = temp[i][j-1] + m7[i][j];
      }
    }
    for(i=0; i<height; i++)
    {
      for(j = 0; j < width; j++)
      {
        m7[i][j] = temp[i][j];
      }
    }
  }
  return 0;
}

/*!
 ************************************************************************
 * \brief
 *    Transform,quantization,inverse transform for chroma.
 *    The main reason why this is done in a separate routine is the
 *    additional 2x2 transform of DC-coeffs. This routine is called
 *    once for each of the chroma components.
 *
 * \par Input:
 *    uv    : Make difference between the U and V chroma component  \n
 *    cr_cbp: chroma coded block pattern
 *
 * \par Output:
 *    cr_cbp: Updated chroma coded block pattern.
 ************************************************************************
 */
int residual_transform_quant_chroma_4x4_ls(Macroblock *currMB, int uv, int cr_cbp)
{
  int i,j,n2,n1,coeff_ctr,level ,scan_pos,run;
  int m1[BLOCK_SIZE];
  int coeff_cost;
  imgpel *orig_img, *pred_img;

  int   b4;
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  Boolean is_cavlc = (Boolean) (currSlice->symbol_mode == CAVLC);
  int*  DCLevel = currSlice->cofDC[uv+1][0];
  int*  DCRun   = currSlice->cofDC[uv+1][1];
  int*  ACLevel;
  int*  ACRun;
  int   uv_scale = uv * (p_Vid->num_blk8x8_uv >> 1);

  //FRExt
  int yuv = p_Vid->yuv_format;
  int b8;
  int *m7;
  int m3[4][4];

  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;  
  int    **mb_rres = currSlice->mb_rres[uv + 1]; 
  int    **mb_ores = currSlice->mb_ores[uv + 1];
  imgpel **mb_pred = currSlice->mb_pred[uv + 1]; 
  int **fadjust4x4;

  int intra = ((currMB->mb_type == I4MB) || (currMB->mb_type == I8MB) || (currMB->mb_type == I16MB)); 


  if (currMB->mb_type == P8x8 && currMB->luma_transform_size_8x8_flag)
  {
    //P8x8, transform 8x8 chroma adjustments must be stored in a different array to avoid conflict with transform 4x4
    fadjust4x4    = p_Vid->AdaptiveRounding ? p_Vid->ARCofAdj4x4[uv + 1][4] : NULL;
  }
  else
  {
    fadjust4x4    = p_Vid->AdaptiveRounding ? p_Vid->ARCofAdj4x4[uv + 1][currMB->ar_mode] : NULL;
  }

  if((currMB->c_ipred_mode == HOR_PRED_8 || currMB->c_ipred_mode == VERT_PRED_8) && (intra))
  {
    Residual_DPCM_Chroma(currMB->c_ipred_mode, mb_ores, mb_rres, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
  }
  else
  {
    for (j=0; j < p_Vid->mb_cr_size_y; j++)
    {
      for (i=0; i < p_Vid->mb_cr_size_x; i++)
      {
        mb_rres[j][i]  = mb_ores[j][i];
      }
    }
  }

  if (yuv == YUV420)
  {
    //================== CHROMA DC YUV420 ===================
    //     2X2 transform of DC coeffs.
    run=-1;
    scan_pos=0;    
    m1[0] = mb_rres[0][0] ;
    m1[1] = mb_rres[0][4] ;
    m1[2] = mb_rres[4][0] ;
    m1[3] = mb_rres[4][4] ;

    for (coeff_ctr=0; coeff_ctr < 4; coeff_ctr++)
    {
      run++;

      level =iabs(m1[coeff_ctr]);

      if (level  != 0)
      {
        if (is_cavlc)
          level = imin(level, CAVLC_LEVEL_LIMIT);

        currMB->cbp_blk |= 0xf0000 << (uv << 2) ;    // if one of the 2x2-DC levels is != 0 set the
        cr_cbp=imax(1, cr_cbp);                     // coded-bit all 4 4x4 blocks (bit 16-19 or 20-23)
        level = isignab(level, m1[coeff_ctr]);
        DCLevel[scan_pos  ] = level;
        DCRun  [scan_pos++] = run;
        run=-1;
      }
    }
    DCLevel[scan_pos] = 0;    
  }
  else if(yuv == YUV422)
  {
    //================== CHROMA DC YUV422 ===================
    //transform DC coeff
    //horizontal

    //pick out DC coeff
    for (j=0; j < p_Vid->mb_cr_size_y; j+=BLOCK_SIZE)
    {
      for (i=0; i < p_Vid->mb_cr_size_x; i+=BLOCK_SIZE)
      {
        m3[i>>2][j>>2] = mb_rres[j][i];
      }
    }


    run=-1;
    scan_pos=0;

    //quant of chroma DC-coeffs
    for (coeff_ctr=0;coeff_ctr<8;coeff_ctr++)
    {
      i=SCAN_YUV422[coeff_ctr][0];
      j=SCAN_YUV422[coeff_ctr][1];

      run++;

      level = iabs(m3[i][j]);
      currSlice->tblk4x4[i][j]=m3[i][j];

      if (level != 0)
      {
        //YUV422
        currMB->cbp_blk |= 0xff0000 << (uv << 3) ;   // if one of the DC levels is != 0 set the
        cr_cbp=imax(1,cr_cbp);                       // coded-bit all 4 4x4 blocks (bit 16-31 or 32-47) //YUV444

        DCLevel[scan_pos  ] = isignab(level,currSlice->tblk4x4[i][j]);
        DCRun  [scan_pos++] = run;
        run=-1;
      }
    }
    DCLevel[scan_pos]=0;

    //inverse DC transform
    //horizontal    
  }

  //     Quant of chroma AC-coeffs.
  coeff_cost=0;

  for (b8=0; b8 < (p_Vid->num_blk8x8_uv >> 1); b8++)
  {
    for (b4=0; b4 < 4; b4++)
    {
      int64 uv_cbpblk = ((int64)1) << cbp_blk_chroma[b8 + uv_scale][b4];
      n1 = hor_offset[yuv][b8][b4];
      n2 = ver_offset[yuv][b8][b4];
      ACLevel = currSlice->cofAC[4 + b8 + uv_scale][b4][0];
      ACRun   = currSlice->cofAC[4 + b8 + uv_scale][b4][1];
      run=-1;
      scan_pos=0;

      for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// start change rd_quant
      {
        i=pos_scan[coeff_ctr][0];
        j=pos_scan[coeff_ctr][1];

        ++run;

        level = iabs(mb_rres[n2+j][n1+i]);

        if (p_Vid->AdaptiveRounding)
        {
          fadjust4x4[n2+j][n1+i] = 0;
        }

        if (level  != 0)
        {
          cr_cbp=imax(2, cr_cbp);

          currMB->cbp_blk |= uv_cbpblk;
          coeff_cost += MAX_VALUE;                // set high cost, shall not be discarded

          ACLevel[scan_pos  ] = isignab(level, mb_rres[n2+j][n1+i]);
          ACRun  [scan_pos++] = run;
          run=-1;

          level = isignab(level, mb_rres[n2+j][n1+i]);          
        }
      }
      ACLevel[scan_pos] = 0;
    }
  }

  if((currMB->c_ipred_mode == HOR_PRED_8 || currMB->c_ipred_mode == VERT_PRED_8) && intra)
  {
    Inv_Residual_DPCM_Chroma(currMB->c_ipred_mode, mb_rres, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y) ;
  }

  for (j=0; j < p_Vid->mb_cr_size_y; ++j)
  {      
    orig_img = &p_Vid->enc_picture->imgUV[uv][currMB->pix_c_y + j][currMB->pix_c_x];
    m7 = mb_rres[j];
    pred_img = mb_pred[j];
    for (i=0; i < p_Vid->mb_cr_size_x; ++i)
    {        
      orig_img[i] = (imgpel) m7[i] + pred_img[i];
    }
  }  

  return cr_cbp;
}

/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.              \n
 *    coeff_cost: Counter for nonzero coefficients, used to discard expensive levels.
 *
 *
 ************************************************************************
 */
int residual_transform_quant_luma_4x4_sp(Macroblock *currMB, ColorPlane pl, int block_x,int block_y,int *coeff_cost, int intra)
{
  int i,j,coeff_ctr;
  int qp_const,ilev, level,scan_pos = 0,run = -1;
  int nonzero = FALSE;
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;

  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;
  imgpel **mb_pred = currSlice->mb_pred[pl];
  int    **mb_rres = currSlice->mb_rres[pl]; 
  int    **mb_ores = currSlice->mb_ores[pl];
  int c_err,qp_const2;

  int   qp = currMB->qp_scaled[pl]; 
  int   qp_sp = (currMB->qpsp);

  const byte *c_cost = COEFF_COST4x4[currSlice->disthres];
  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;

  int   pos_x   = block_x >> BLOCK_SHIFT;
  int   pos_y   = block_y >> BLOCK_SHIFT;
  int   b8      = 2*(pos_y >> 1) + (pos_x >> 1);
  int   b4      = 2*(pos_y & 0x01) + (pos_x & 0x01);
  int*  ACLevel = currSlice->cofAC[b8][b4][0];
  int*  ACRun   = currSlice->cofAC[b8][b4][1];

  // For encoding optimization
  int c_err1, c_err2, level1, level2;
  double D_dis1, D_dis2;
  int len, info;
  double lambda_mode   = 0.85 * pow (2, (qp - SHIFT_QP)/3.0) * 4;

  QuantParameters   *p_Quant = p_Vid->p_Quant;

  int qp_per    = p_Quant->qp_per_matrix[qp];
  int q_bits    = Q_BITS + qp_per;
  int qp_per_sp = p_Quant->qp_per_matrix[qp_sp];
  int q_bits_sp = Q_BITS + qp_per_sp;

  LevelQuantParams **q_params_4x4 = p_Quant->q_params_4x4[pl][intra][qp]; 
  LevelQuantParams **quant_params_sp = p_Quant->q_params_4x4[pl][intra][qp_sp]; 

  qp_const  = (1<<q_bits)/6;    // inter
  qp_const2 = (1<<q_bits_sp)>>1;  //sp_pred

  //  Horizontal transform
  for (j=block_y; j< block_y + BLOCK_SIZE; ++j)
  {
    for (i=block_x; i< block_x + BLOCK_SIZE; ++i)
    { 
      mb_rres[j][i] = mb_ores[j][i];
      mb_rres[j][i]+=mb_pred[j][i];
      currSlice->tblk16x16[j][i] = mb_pred[j][i]; 
    }
  }

  // 4x4 transform
  forward4x4(mb_rres, mb_rres, block_y, block_x);
  forward4x4(currSlice->tblk16x16, currSlice->tblk16x16, block_y, block_x);

  for (coeff_ctr = 0;coeff_ctr < 16;coeff_ctr++)     
  {
    i = pos_scan[coeff_ctr][0];
    j = pos_scan[coeff_ctr][1];

    run++;
    ilev=0;

    // decide prediction

    // case 1
    level1 = (iabs (currSlice->tblk16x16[j+block_y][i+block_x]) * quant_params_sp[j][i].ScaleComp + qp_const2) >> q_bits_sp;
    level1 = (level1 << q_bits_sp) / quant_params_sp[j][i].ScaleComp;
    c_err1 = mb_rres[j+block_y][i+block_x] - isignab(level1, currSlice->tblk16x16[j+block_y][i+block_x]);
    level1 = (iabs (c_err1) * q_params_4x4[j][i].ScaleComp + qp_const) >> q_bits;

    // case 2
    c_err2 = mb_rres[j+block_y][i+block_x] - currSlice->tblk16x16[j+block_y][i+block_x];
    level2 = (iabs (c_err2) * q_params_4x4[j][i].ScaleComp + qp_const) >> q_bits;

    // select prediction
    if ((level1 != level2) && (level1 != 0) && (level2 != 0))
    {
      D_dis1 = mb_rres[j+block_y][i+block_x] - 
        ((isignab(level1,c_err1) * (q_params_4x4[j][i].InvScaleComp >> 4) * A[j][i]<< qp_per) >>6) 
        - currSlice->tblk16x16[j+block_y][i+block_x];
      levrun_linfo_inter(level1, run, &len, &info);
      D_dis1 = D_dis1 * D_dis1 + lambda_mode * len;

      D_dis2 = mb_rres[j+block_y][i+block_x] - 
        ((isignab(level2,c_err2)* (q_params_4x4[j][i].InvScaleComp >> 4)* A[j][i]<< qp_per) >>6) 
        - currSlice->tblk16x16[j+block_y][i+block_x];
      levrun_linfo_inter(level2, run, &len, &info);
      D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;

      if (D_dis1 == D_dis2)
        level = (iabs(level1) < iabs(level2)) ? level1 : level2;
      else if (D_dis1 < D_dis2)
        level = level1;
      else
        level = level2;

      c_err = (level == level1) ? c_err1 : c_err2;
    }
    else if (level1 == level2)
    {
      level = level1;
      c_err = c_err1;
    }
    else
    {
      level = (level1 == 0) ? level1 : level2;
      c_err = (level1 == 0) ? c_err1 : c_err2;
    }

    if (level != 0)
    {
      nonzero = TRUE;

      *coeff_cost += (level > 1) ? MAX_VALUE : c_cost[run];

      level = isignab(level,c_err);
      ACLevel[scan_pos] = level;
      ACRun  [scan_pos] = run;
      ++scan_pos;
      run=-1;                     // reset zero level counter
      ilev=((level * (q_params_4x4[j][i].InvScaleComp >> 4)* A[j][i] << qp_per) >>6);
    }

    ilev += currSlice->tblk16x16[j+block_y][i+block_x];

    if(currSlice->slice_type != SI_SLICE && !p_Vid->sp2_frame_indicator)//stores the SP frame coefficients in p_Vid->lrec, will be useful to encode these and create SI or SP switching frame
    {
      p_Vid->lrec[currMB->pix_y+block_y+j][currMB->pix_x+block_x+i]=
        isignab((iabs(ilev) * quant_params_sp[j][i].ScaleComp + qp_const2) >> q_bits_sp, ilev);
    }    

    // Yrec Qs and DeQs
    mb_rres[j+block_y][i+block_x] = isignab((iabs(ilev) * quant_params_sp[j][i].ScaleComp + qp_const2)>> q_bits_sp, ilev) * (quant_params_sp[j][i].InvScaleComp >> 4) << qp_per_sp;
  }
  ACLevel[scan_pos] = 0;

  // inverse transform
  inverse4x4(mb_rres, mb_rres, block_y, block_x);

  for (j=block_y; j < block_y+BLOCK_SIZE; ++j)
  {
    for (i=block_x; i < block_x+BLOCK_SIZE; ++i)
    {
      mb_rres[j][i] = iClip1 (p_Vid->max_imgpel_value, rshift_rnd_sf(mb_rres[j][i], DQ_BITS));
   }
  }
  
  //  Decoded block moved to frame memory
  for (j=0; j < BLOCK_SIZE; ++j)
  {
    for (i=0; i < BLOCK_SIZE; ++i)
    {
      img_enc[currMB->pix_y+block_y+j][currMB->pix_x+block_x+i]= (imgpel) mb_rres[block_y+j][block_x+i];
    }
  }
         
  return nonzero;
}

/*!
 ************************************************************************
 * \brief
 *    Transform,quantization,inverse transform for chroma.
 *    The main reason why this is done in a separate routine is the
 *    additional 2x2 transform of DC-coeffs. This routine is called
 *    once for each of the chroma components.
 *
 * \par Input:
 *    uv    : Make difference between the U and V chroma component
 *    cr_cbp: chroma coded block pattern
 *
 * \par Output:
 *    cr_cbp: Updated chroma coded block pattern.
 ************************************************************************
 */
int residual_transform_quant_chroma_4x4_sp(Macroblock *currMB, int uv,int cr_cbp)
{
  int i,j,ilev,n2,n1,coeff_ctr,c_err,level ,scan_pos,run;
  int m1[BLOCK_SIZE];
  int coeff_cost;
  int cr_cbp_tmp;
  int mp1[BLOCK_SIZE];
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  Boolean is_cavlc = (Boolean) (currSlice->symbol_mode == CAVLC);
  const byte *c_cost = COEFF_COST4x4[currSlice->disthres];
  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;

  int   intra = is_intra (currMB);

  int   b4;
  int*  DCLevel = currSlice->cofDC[uv+1][0];
  int*  DCRun   = currSlice->cofDC[uv+1][1];

  int c_err1, c_err2, level1, level2;
  int len, info;
  double D_dis1, D_dis2;
  double lambda_mode   = 0.85 * pow (2, (currMB->qp -SHIFT_QP)/3.0) * 4;
  int max_imgpel_value_uv = p_Vid->max_pel_value_comp[1];

  int qpChroma = currMB->qpc[uv] + currSlice->bitdepth_chroma_qp_scale;       
  int qpc_sp = iClip3(-currSlice->bitdepth_chroma_qp_scale, 51, currMB->qpsp + p_Vid->active_pps->chroma_qp_index_offset);
  int qpChromaSP = qpc_sp < 0 ? qpc_sp : QP_SCALE_CR[qpc_sp];

  int    **mb_rres = currSlice->mb_rres[uv + 1]; 
  int    **mb_ores = currSlice->mb_ores[uv + 1]; 
  imgpel **mb_pred = currSlice->mb_pred[uv + 1]; 
  
  QuantMethods quant_methods;
  QuantParameters *p_Quant = p_Vid->p_Quant;    

  int qp_per    = p_Quant->qp_per_matrix[qpChroma];
  int q_bits    = Q_BITS + qp_per;
  int qp_const  = (1<<q_bits)/6;    // inter
  int qp_per_sp = p_Quant->qp_per_matrix[qpChromaSP];
  int q_bits_sp = Q_BITS + qp_per_sp;
  int qp_const2 = (1<<q_bits_sp)>>1;  //sp_pred
  
  LevelQuantParams **q_params_4x4 = p_Quant->q_params_4x4[uv + 1][intra][qpChroma]; 

  LevelQuantParams **quant_params_sp = p_Quant->q_params_4x4[uv + 1][intra][qpChromaSP]; 

  quant_methods.type     = CHROMA_AC;

  for (j=0; j < p_Vid->mb_cr_size_y; ++j)
  {
    for (i=0; i < p_Vid->mb_cr_size_x; ++i)
    {
      mb_rres[j][i]  = mb_ores[j][i];
      mb_rres[j][i] += mb_pred[j][i];
      currSlice->tblk16x16[j][i] = mb_pred[j][i];
    }
  }
  
  for (n2=0; n2 < p_Vid->mb_cr_size_y; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 < p_Vid->mb_cr_size_x; n1 += BLOCK_SIZE)
    {
      forward4x4(mb_rres, mb_rres, n2, n1);      
      forward4x4(currSlice->tblk16x16, currSlice->tblk16x16, n2, n1);
    }
  }

  //     2X2 transform of DC coeffs.
  hadamard2x2(mb_rres, m1);
  hadamard2x2(currSlice->tblk16x16, mp1);
  
  run=-1;
  scan_pos=0;

  for (coeff_ctr = 0; coeff_ctr < 4; coeff_ctr++)
  {
    run++;
    ilev=0;

    // case 1
    c_err1 = (iabs (mp1[coeff_ctr]) * quant_params_sp[0][0].ScaleComp + 2 * qp_const2) >> (q_bits_sp + 1);
    c_err1 = (c_err1 << (q_bits_sp + 1)) / quant_params_sp[0][0].ScaleComp;
    c_err1 = m1[coeff_ctr] - isignab(c_err1, mp1[coeff_ctr]);
    level1 = (iabs(c_err1) * q_params_4x4[0][0].ScaleComp + 2 * qp_const) >> (q_bits+1);

    // case 2
    c_err2 = m1[coeff_ctr] - mp1[coeff_ctr];
    level2 = (iabs(c_err2) * q_params_4x4[0][0].ScaleComp + 2 * qp_const) >> (q_bits+1);

    if (level1 != level2 && level1 != 0 && level2 != 0)
    {
      D_dis1 = m1[coeff_ctr] - ((isignab(level1,c_err1)* (q_params_4x4[0][0].InvScaleComp >> 4) * A[0][0]<< qp_per) >>5)- mp1[coeff_ctr];
      levrun_linfo_c2x2(level1, run, &len, &info);
      D_dis1 = D_dis1 * D_dis1 + lambda_mode * len;

      D_dis2 = m1[coeff_ctr] - ((isignab(level2,c_err2)* (q_params_4x4[0][0].InvScaleComp >> 4) * A[0][0]<< qp_per) >>5)- mp1[coeff_ctr];
      levrun_linfo_c2x2(level2, run, &len, &info);
      D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;

      if (D_dis1 == D_dis2)
        level = (iabs(level1) < iabs(level2)) ? level1 : level2;
      else if (D_dis1 < D_dis2)
        level = level1;
      else
        level = level2;

      c_err = (level == level1) ? c_err1 : c_err2;
    }
    else if (level1 == level2)
    {
      level = level1;
      c_err = c_err1;
    }
    else
    {
      level = (level1 == 0) ? level1 : level2;
      c_err = (level1 == 0) ? c_err1 : c_err2;
    }

    if (level  != 0)
    {
      if (is_cavlc)
        level = imin(level, CAVLC_LEVEL_LIMIT);
      
      currMB->cbp_blk |= 0xf0000 << (uv << 2) ;  // if one of the 2x2-DC levels is != 0 the coded-bit
      cr_cbp = imax(1, cr_cbp);
      DCLevel[scan_pos  ] = isignab(level ,c_err);
      DCRun  [scan_pos++] = run;
      run=-1;
      ilev=((isignab(level,c_err)* (q_params_4x4[0][0].InvScaleComp*A[0][0] >> 4) << qp_per) >>5);
    }

    ilev+= mp1[coeff_ctr];
    m1[coeff_ctr]=isignab((iabs(ilev)  * quant_params_sp[0][0].ScaleComp + 2 * qp_const2) >> (q_bits_sp+1), ilev) * (quant_params_sp[0][0].InvScaleComp >> 4) << qp_per_sp;
    if(currSlice->slice_type != SI_SLICE && !p_Vid->sp2_frame_indicator)
      p_Vid->lrec_uv[uv][currMB->pix_c_y + 4*(coeff_ctr & 0x01)][currMB->pix_c_x + 4*(coeff_ctr>>1)]=isignab((iabs(ilev)  * quant_params_sp[0][0].ScaleComp + 2 * qp_const2) >> (q_bits_sp+1), ilev);// stores the SP frames coefficients, will be useful to encode SI or switching SP frame
  }
  DCLevel[scan_pos] = 0;

  //  Inverse transform of 2x2 DC levels
  ihadamard2x2(m1, m1);

  mb_rres[0][0]=(m1[0])>>1;
  mb_rres[0][4]=(m1[1])>>1;
  mb_rres[4][0]=(m1[2])>>1;
  mb_rres[4][4]=(m1[3])>>1;

  //     Quant of chroma AC-coeffs.
  coeff_cost=0;
  cr_cbp_tmp=0;

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      b4      = 2*(n2 >> 2) + (n1 >> 2);
      quant_methods.ACLevel = currSlice->cofAC[uv+4][b4][0];
      quant_methods.ACRun   = currSlice->cofAC[uv+4][b4][1];

      run      = -1;
      scan_pos =  0;

      for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// start change rd_quant
      {
        i=pos_scan[coeff_ctr][0];
        j=pos_scan[coeff_ctr][1];

        ++run;
        ilev=0;

        // quantization on prediction
        c_err1 = (iabs(currSlice->tblk16x16[n2+j][n1+i]) * quant_params_sp[j][i].ScaleComp + qp_const2) >> q_bits_sp;
        c_err1 = (c_err1 << q_bits_sp) / quant_params_sp[j][i].ScaleComp;
        c_err1 = mb_rres[n2+j][n1+i] - isignab(c_err1, currSlice->tblk16x16[n2+j][n1+i]);
        level1 = (iabs(c_err1) * q_params_4x4[j][i].ScaleComp + qp_const) >> q_bits;

        // no quantization on prediction
        c_err2 = mb_rres[n2+j][n1+i] - currSlice->tblk16x16[n2+j][n1+i];
        level2 = (iabs(c_err2) * q_params_4x4[j][i].ScaleComp + qp_const) >> q_bits;

        if (level1 != level2 && level1 != 0 && level2 != 0)
        {
          D_dis1 = mb_rres[n2+j][n1+i] - ((isignab(level1,c_err1) * (q_params_4x4[j][i].InvScaleComp >> 4) * A[j][i]<< qp_per) >>6) - currSlice->tblk16x16[n2+j][n1+i];

          levrun_linfo_inter(level1, run, &len, &info);
          D_dis1 = D_dis1 * D_dis1 + lambda_mode * len;

          D_dis2 = mb_rres[n2+j][n1+i] - ((isignab(level2,c_err2)* (q_params_4x4[j][i].InvScaleComp*A[j][i] >> 4) << qp_per) >>6) - currSlice->tblk16x16[n2+j][n1+i];
          levrun_linfo_inter(level2, run, &len, &info);
          D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;

          if (D_dis1 == D_dis2)
            level = (iabs(level1) < iabs(level2)) ? level1 : level2;
          else
          {
            if (D_dis1 < D_dis2)
              level = level1;
            else
              level = level2;
          }
          c_err = (level == level1) ? c_err1 : c_err2;
        }
        else if (level1 == level2)
        {
          level = level1;
          c_err = c_err1;
        }
        else
        {
          level = (level1 == 0) ? level1 : level2;
          c_err = (level1 == 0) ? c_err1 : c_err2;
        }

        if (level  != 0)
        {
          currMB->cbp_blk |=  (int64)1 << (16 + (uv << 2) + ((n2 >> 1) + (n1 >> 2))) ;

          coeff_cost += (level > 1) ? MAX_VALUE : c_cost[run];  // set high cost, shall not be discarded

          cr_cbp_tmp=2;
          level = isignab(level,c_err);
          quant_methods.ACLevel[scan_pos] = level;
          quant_methods.ACRun  [scan_pos] = run;
          ++scan_pos;
          run=-1;
          ilev=((level * (q_params_4x4[j][i].InvScaleComp*A[j][i] >> 4) << qp_per) >>6);
        }
        ilev+=currSlice->tblk16x16[n2+j][n1+i];
        if(currSlice->slice_type != SI_SLICE && !p_Vid->sp2_frame_indicator)
          if(!( ((n2+j) & 0x03)==0 && ((n1+i) & 0x03) ==0 ))
            p_Vid->lrec_uv[uv][currMB->pix_c_y + n2+j][currMB->pix_c_x + n1 + i]=isignab((iabs(ilev) * quant_params_sp[j][i].ScaleComp + qp_const2) >> q_bits_sp,ilev);//stores the SP frames coefficients, will be useful to encode SI or switching SP frame
        mb_rres[n2+j][n1+i] = isignab((iabs(ilev) * quant_params_sp[j][i].ScaleComp + qp_const2) >> q_bits_sp,ilev) * (quant_params_sp[j][i].InvScaleComp >> 4) << qp_per_sp;
      }
      quant_methods.ACLevel[scan_pos] = 0;
    }
  }

  // * reset chroma coeffs

  if(cr_cbp_tmp==2)
    cr_cbp=2;
  
  //     inverse transform.
  //     Horizontal.
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      inverse4x4(mb_rres, mb_rres, n2, n1);

      for (j=0; j < BLOCK_SIZE; ++j)
        for (i=0; i < BLOCK_SIZE; ++i)
        {
          mb_rres[n2+j][n1+i] = iClip1 (max_imgpel_value_uv,rshift_rnd_sf(mb_rres[n2+j][n1+i], DQ_BITS));
        }
    }
  }

  //  Decoded block moved to memory
  for (j=0; j < BLOCK_SIZE*2; ++j)
    for (i=0; i < BLOCK_SIZE*2; ++i)
    {
      p_Vid->enc_picture->imgUV[uv][currMB->pix_c_y + j][currMB->pix_c_x + i]= (imgpel) mb_rres[j][i];
    }  

  return cr_cbp;
}


/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.            \n
 *    coeff_cost: Counter for nonzero coefficients, used to discard expensive levels.
 ************************************************************************
 */
void copyblock_sp(Macroblock *currMB, ColorPlane pl, int block_x,int block_y)
{
  int i, j;

  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  QuantParameters *p_Quant = p_Vid->p_Quant;
  int cur_qp = currMB->qpsp + p_Vid->bitdepth_luma_qp_scale;  
  int qp_per = p_Quant->qp_per_matrix[cur_qp];
  int q_bits = Q_BITS + qp_per;
  int qp_const2 = (1 << q_bits) >> 1;  //sp_pred
  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;
  imgpel **mb_pred = currSlice->mb_pred[pl]; 
  int    **mb_ores = currSlice->mb_ores[pl]; 
  int    **mb_rres = currSlice->mb_rres[pl]; 

  LevelQuantParams **q_params_4x4 = p_Quant->q_params_4x4[pl][0][cur_qp]; 

  //  Horizontal transform
  for (j=0; j< BLOCK_SIZE; ++j)
  {
    for (i=0; i< BLOCK_SIZE; ++i)
    {
      mb_rres[j][i] = mb_ores[j+block_y][i+block_x];
      currSlice->tblk16x16[j][i]=mb_pred[j+block_y][i+block_x];  
    }
  }

  forward4x4(currSlice->tblk16x16, currSlice->tblk16x16, 0, 0);

  // Quant
  for (j=0;j < BLOCK_SIZE; ++j)
  {
    for (i=0; i < BLOCK_SIZE; ++i)
    {
      mb_rres[j][i]=isignab((iabs(currSlice->tblk16x16[j][i])* q_params_4x4[j][i].ScaleComp+qp_const2)>> q_bits,
      currSlice->tblk16x16[j][i]) * (q_params_4x4[j][i].InvScaleComp >> 4) <<qp_per;
      if(currSlice->slice_type != SI_SLICE && !p_Vid->sp2_frame_indicator)
      {
        p_Vid->lrec[currMB->pix_y+block_y+j][currMB->pix_x+block_x+i] =
          isignab((iabs(currSlice->tblk16x16[j][i]) * q_params_4x4[j][i].ScaleComp + qp_const2) >> q_bits, currSlice->tblk16x16[j][i]);// stores the SP frames coefficients, will be useful to encode SI or switching SP frame
      }
    }
  }

  //     inverse transform.
  //     horizontal
  inverse4x4(mb_rres, mb_rres, 0, 0);

  //  Decoded block moved to frame memory
  for (j=0; j < BLOCK_SIZE; ++j)
  {
    for (i=0; i < BLOCK_SIZE; ++i)
    {
      mb_rres[j][i] = iClip1 (p_Vid->max_imgpel_value, rshift_rnd_sf(mb_rres[j][i], DQ_BITS));
      img_enc[currMB->pix_y+block_y+j][currMB->pix_x+block_x+i]=(imgpel) mb_rres[j][i];
    }
  }

}

/*!
 ************************************************************************
 * \brief Eric Setton
 * Encoding of a secondary SP / SI frame.
 * For an SI frame the predicted block should only come from spatial pred.
 * The original image signal is the error coefficients of a primary SP in the raw data stream
 * the difference with the primary SP are :
 *  - the prediction signal is transformed and quantized (qpsp) but not dequantized
 *  - only one kind of prediction is considered and not two
 *  - the resulting error coefficients are not quantized before being sent to the VLC
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.
 *    coeff_cost: Counter for nonzero coefficients, used to discard expensive levels.
 *
 *
 ************************************************************************
 */

int residual_transform_quant_luma_4x4_sp2(Macroblock *currMB, ColorPlane pl, int block_x,int block_y,int *coeff_cost, int intra)
{
  int i,j,ilev,coeff_ctr;
  int level,scan_pos = 0,run = -1;
  int nonzero = FALSE;

  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  QuantParameters *p_Quant = p_Vid->p_Quant;
  imgpel **img_enc = p_Vid->enc_picture->p_curr_img;
  imgpel **mb_pred = currSlice->mb_pred[pl];   
//  int    **mb_ores = currSlice->mb_ores[pl];   
  int    **mb_rres = currSlice->mb_rres[pl];   
  int c_err,qp_const2;

  int   pos_x   = block_x >> BLOCK_SHIFT;
  int   pos_y   = block_y >> BLOCK_SHIFT;
  int   b8      = 2*(pos_y >> 1) + (pos_x >> 1);
  int   b4      = 2*(pos_y & 0x01) + (pos_x & 0x01);
  const byte *c_cost = COEFF_COST4x4[currSlice->disthres];
  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;

  int level1;
  
  int   qp_sp = (currMB->qpsp);

  int qp_per_sp = p_Quant->qp_per_matrix[qp_sp];
  int q_bits_sp = Q_BITS + qp_per_sp;

  //LevelQuantParams **q_params_4x4 = p_Quant->q_params_4x4[pl][intra][qp]; 
  LevelQuantParams **quant_params_sp = p_Quant->q_params_4x4[pl][intra][qp_sp]; 
  QuantMethods quant_methods;
  
  quant_methods.ACLevel = currSlice->cofAC[b8][b4][0];
  quant_methods.ACRun   = currSlice->cofAC[b8][b4][1];

  qp_const2=(1<<q_bits_sp)>>1;  //sp_pred

  for (j=0; j< BLOCK_SIZE; ++j)
  {
    for (i=0; i< BLOCK_SIZE; ++i)
    {
      //Coefficients obtained from the prior encoding of the SP frame
      mb_rres[j][i] = p_Vid->lrec[currMB->pix_y+block_y+j][currMB->pix_x+block_x+i];
      //Predicted block
      currSlice->tblk16x16[j][i]=mb_pred[j+block_y][i+block_x];
    }
  }
  // forward transform
  forward4x4(currSlice->tblk16x16, currSlice->tblk16x16, 0, 0);

  for (coeff_ctr=0;coeff_ctr < 16;coeff_ctr++)     // 8 times if double scan, 16 normal scan
  {

    i=pos_scan[coeff_ctr][0];
    j=pos_scan[coeff_ctr][1];

    run++;
    ilev=0;

    //quantization of the predicted block
    level1 = (iabs (currSlice->tblk16x16[j][i]) * quant_params_sp[j][i].ScaleComp + qp_const2) >> q_bits_sp;
    //substracted from p_Vid->lrec
    c_err = mb_rres[j][i]-isignab(level1, currSlice->tblk16x16[j][i]);   //substracting the predicted block

    level = iabs(c_err);
    if (level != 0)
    {
      nonzero=TRUE;

      *coeff_cost += (level > 1) ? MAX_VALUE : c_cost[run];

      quant_methods.ACLevel[scan_pos] = isignab(level,c_err);
      quant_methods.ACRun  [scan_pos] = run;
      ++scan_pos;
      run=-1;                     // reset zero level counter
    }
    //from now on we are in decoder land
    ilev=c_err + isignab(level1,currSlice->tblk16x16[j][i]) ;  // adding the quantized predicted block
    mb_rres[j][i] = ilev * (quant_params_sp[j][i].InvScaleComp >> 4) << qp_per_sp;
  }
  quant_methods.ACLevel[scan_pos] = 0;

  //  Inverse transform
  inverse4x4(mb_rres, mb_rres, 0, 0);

  for (j=0; j < BLOCK_SIZE; ++j)
  {
    for (i=0; i < BLOCK_SIZE; ++i)
    {
      img_enc[currMB->pix_y+block_y+j][currMB->pix_x+block_x+i] = (imgpel) iClip3 (0, p_Vid->max_imgpel_value,rshift_rnd_sf(mb_rres[j][i], DQ_BITS));
    }
  }

  return nonzero;
}


/*!
 ************************************************************************
 * \brief Eric Setton
 * Encoding of the chroma of a  secondary SP / SI frame.
 * For an SI frame the predicted block should only come from spatial pred.
 * The original image signal is the error coefficients of a primary SP in the raw data stream
 * the difference with the primary SP are :
 *  - the prediction signal is transformed and quantized (qpsp) but not dequantized
 *  - the resulting error coefficients are not quantized before being sent to the VLC
 *
 * \par Input:
 *    uv    : Make difference between the U and V chroma component
 *    cr_cbp: chroma coded block pattern
 *
 * \par Output:
 *    cr_cbp: Updated chroma coded block pattern.
 *
 ************************************************************************
 */
int residual_transform_quant_chroma_4x4_sp2(Macroblock *currMB, int uv,int cr_cbp)
{
  int i,j,ilev,n2,n1,coeff_ctr,c_err,level ,scan_pos = 0,run = -1;
  int m1[BLOCK_SIZE];
  int coeff_cost;
  int cr_cbp_tmp;
  int mp1[BLOCK_SIZE];
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  QuantParameters *p_Quant = p_Vid->p_Quant;
  QuantMethods quant_methods;
  const byte *c_cost = COEFF_COST4x4[currSlice->disthres];
  const byte (*pos_scan)[2] = currMB->is_field_mode ? FIELD_SCAN : SNGL_SCAN;

  int   b4;
  int*  DCLevel = currSlice->cofDC[uv+1][0];
  int*  DCRun   = currSlice->cofDC[uv+1][1];
  int  level1;
  int    **mb_rres = currSlice->mb_rres[uv + 1]; 
  imgpel **mb_pred = currSlice->mb_pred[uv + 1]; 
  int   intra = is_intra (currMB);

  int qpChroma   = currMB->qpc[uv] + currSlice->bitdepth_chroma_qp_scale;   

  int qpc_sp = iClip3(-currSlice->bitdepth_chroma_qp_scale, 51, currMB->qpsp + p_Vid->active_pps->chroma_qp_index_offset);
  
  int qpChromaSP = qpc_sp < 0 ? qpc_sp : QP_SCALE_CR[qpc_sp];

  int qp_per    = p_Quant->qp_per_matrix[qpChroma];

  int qp_per_sp = p_Quant->qp_per_matrix[qpChromaSP];
  int q_bits_sp = Q_BITS + qp_per;
  int qp_const2 = (1 << q_bits_sp)>>1;  //sp_pred

  //LevelQuantParams **q_params_4x4 = p_Quant->q_params_4x4[uv + 1][intra][qpChroma]; 
  LevelQuantParams **quant_params_sp = p_Quant->q_params_4x4[uv + 1][intra][qpChromaSP]; 

  for (j=0; j < MB_BLOCK_SIZE>>1; ++j)
  {
    for (i=0; i < MB_BLOCK_SIZE>>1; ++i)
    {
      currSlice->tblk16x16[j][i]=mb_pred[j][i];
      mb_rres[j][i]=p_Vid->lrec_uv[uv][currMB->pix_c_y + j][currMB->pix_c_x + i];
    }
  }


  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      forward4x4(currSlice->tblk16x16, currSlice->tblk16x16, n2, n1);
    }
  }

  //   DC coefficients already transformed and quantized
  m1[0]= mb_rres[0][0];
  m1[1]= mb_rres[0][4];
  m1[2]= mb_rres[4][0];
  m1[3]= mb_rres[4][4];

  //     2X2 transform of predicted DC coeffs.
  hadamard2x2(currSlice->tblk16x16, mp1);

  for (coeff_ctr=0; coeff_ctr < 4; coeff_ctr++)
  {
    run++;
    ilev=0;

    //quantization of predicted DC coeff
    level1 = (iabs (mp1[coeff_ctr]) * quant_params_sp[0][0].ScaleComp + 2 * qp_const2) >> (q_bits_sp + 1);
    //substratcted from lrecUV
    c_err = m1[coeff_ctr] - isignab(level1, mp1[coeff_ctr]);
    level = iabs(c_err);

    if (level  != 0)
    {
      currMB->cbp_blk |= 0xf0000 << (uv << 2) ;  // if one of the 2x2-DC levels is != 0 the coded-bit
      cr_cbp=imax(1,cr_cbp);
      DCLevel[scan_pos] = isignab(level ,c_err);
      DCRun  [scan_pos] = run;
      scan_pos++;
      run=-1;
    }

    //from now on decoder world
    ilev = c_err + isignab(level1,mp1[coeff_ctr]) ; // we have perfect reconstruction here

    m1[coeff_ctr]= ilev  * (quant_params_sp[0][0].InvScaleComp >> 4) << qp_per_sp;

  }
  DCLevel[scan_pos] = 0;

  //  Inverse transform of 2x2 DC levels
  ihadamard2x2(m1, m1);

  mb_rres[0][0]=m1[0]>>1;
  mb_rres[0][4]=m1[1]>>1;
  mb_rres[4][0]=m1[2]>>1;
  mb_rres[4][4]=m1[3]>>1;

  //     Quant of chroma AC-coeffs.
  coeff_cost=0;
  cr_cbp_tmp=0;

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      b4      = 2*(n2 >> 2) + (n1 >> 2);
      quant_methods.ACLevel = currSlice->cofAC[uv+4][b4][0];
      quant_methods.ACRun   = currSlice->cofAC[uv+4][b4][1];

      run      = -1;
      scan_pos =  0;

      for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// start change rd_quant
      {

        i=pos_scan[coeff_ctr][0];
        j=pos_scan[coeff_ctr][1];

        ++run;
        ilev=0;
        // quantization on prediction
        level1 = (iabs(currSlice->tblk16x16[n2+j][n1+i]) * quant_params_sp[j][i].ScaleComp + qp_const2) >> q_bits_sp;
        //substracted from p_Vid->lrec
        c_err  = mb_rres[n2+j][n1+i] - isignab(level1, currSlice->tblk16x16[n2+j][n1+i]);
        level  = iabs(c_err) ;

        if (level  != 0)
        {
          currMB->cbp_blk |=  (int64)1 << (16 + (uv << 2) + ((n2 >> 1) + (n1 >> 2))) ;

          coeff_cost += (level > 1) ? MAX_VALUE : c_cost[run];

          cr_cbp_tmp=2;
          quant_methods.ACLevel[scan_pos] = isignab(level,c_err);
          quant_methods.ACRun  [scan_pos] = run;
          ++scan_pos;
          run=-1;
        }

        //from now on decoder land
        ilev=c_err + isignab(level1,currSlice->tblk16x16[n2+j][n1+i]);
        mb_rres[n2+j][n1+i] = ilev * (quant_params_sp[j][i].InvScaleComp >> 4) << qp_per_sp;
      }
      quant_methods.ACLevel[scan_pos] = 0;
    }
  }
  // * reset chroma coeffs

  if(cr_cbp_tmp==2)
    cr_cbp=2;
  
  //     inverse transform.
  //     Horizontal.
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      inverse4x4(mb_rres, mb_rres, n2, n1);

      //     Vertical.
      for (j=0; j < BLOCK_SIZE; ++j)
      {
        for (i=0; i < BLOCK_SIZE; ++i)
        {
          p_Vid->enc_picture->imgUV[uv][currMB->pix_c_y + j + n2][currMB->pix_c_x + i + n1 ] = 
            (imgpel) iClip3 (0, p_Vid->max_imgpel_value,rshift_rnd_sf(mb_rres[n2+j][n1+i], DQ_BITS));
        }
      }
    }
  }

  return cr_cbp;
}


void select_transform(Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;

  if (currSlice->slice_type != SP_SLICE && currSlice->slice_type != SI_SLICE)
  {
    VideoParameters *p_Vid = currMB->p_Vid;
    if (p_Vid->active_sps->lossless_qpprime_flag == 1)
    {
      if (currMB->qp_scaled[(short) p_Vid->colour_plane_id] == 0)
      {
        currMB->residual_transform_quant_luma_4x4   = residual_transform_quant_luma_4x4_ls;
        currMB->residual_transform_quant_luma_16x16 = residual_transform_quant_luma_16x16_ls;
        currMB->residual_transform_quant_luma_8x8   = residual_transform_quant_luma_8x8_ls;
      }
      else
      {
        currMB->residual_transform_quant_luma_4x4   = residual_transform_quant_luma_4x4;
        currMB->residual_transform_quant_luma_16x16 = residual_transform_quant_luma_16x16;
        if (currSlice->symbol_mode == CAVLC)
          currMB->residual_transform_quant_luma_8x8 = residual_transform_quant_luma_8x8_cavlc;
        else
          currMB->residual_transform_quant_luma_8x8 = residual_transform_quant_luma_8x8;
        currMB->residual_transform_quant_chroma_4x4[0] = residual_transform_quant_chroma_4x4;
        currMB->residual_transform_quant_chroma_4x4[1] = residual_transform_quant_chroma_4x4;
      }

      if (currMB->qp_scaled[PLANE_U] == 0)
      {
        currMB->residual_transform_quant_chroma_4x4[0] = residual_transform_quant_chroma_4x4_ls;
      }
      if (currMB->qp_scaled[PLANE_V] == 0)
      {
        currMB->residual_transform_quant_chroma_4x4[1] = residual_transform_quant_chroma_4x4_ls;
      }
    }
    else
    {
      currMB->residual_transform_quant_luma_4x4   = residual_transform_quant_luma_4x4;
      currMB->residual_transform_quant_luma_16x16 = residual_transform_quant_luma_16x16;
      if (currSlice->symbol_mode == CAVLC)
        currMB->residual_transform_quant_luma_8x8 = residual_transform_quant_luma_8x8_cavlc;
      else
        currMB->residual_transform_quant_luma_8x8 = residual_transform_quant_luma_8x8;

      currMB->residual_transform_quant_chroma_4x4[0] = residual_transform_quant_chroma_4x4;
      currMB->residual_transform_quant_chroma_4x4[1] = residual_transform_quant_chroma_4x4;
    }
  }
  else if(currSlice->slice_type != SI_SLICE && !currSlice->sp2_frame_indicator)
  {
    currMB->residual_transform_quant_luma_4x4 = residual_transform_quant_luma_4x4_sp;
    currMB->residual_transform_quant_luma_16x16 = residual_transform_quant_luma_16x16;
    if (currSlice->symbol_mode == CAVLC)
      currMB->residual_transform_quant_luma_8x8 = residual_transform_quant_luma_8x8_cavlc;
    else
      currMB->residual_transform_quant_luma_8x8 = residual_transform_quant_luma_8x8;

    currMB->residual_transform_quant_chroma_4x4[0]  = residual_transform_quant_chroma_4x4_sp;
    currMB->residual_transform_quant_chroma_4x4[1]  = residual_transform_quant_chroma_4x4_sp;
  }
  else
  {
    currMB->residual_transform_quant_luma_4x4 = residual_transform_quant_luma_4x4_sp2;
    currMB->residual_transform_quant_luma_16x16 = residual_transform_quant_luma_16x16;
    if (currSlice->symbol_mode == CAVLC)
      currMB->residual_transform_quant_luma_8x8 = residual_transform_quant_luma_8x8_cavlc;
    else
      currMB->residual_transform_quant_luma_8x8 = residual_transform_quant_luma_8x8;

    currMB->residual_transform_quant_chroma_4x4[0]  = residual_transform_quant_chroma_4x4_sp2;
    currMB->residual_transform_quant_chroma_4x4[1]  = residual_transform_quant_chroma_4x4_sp2;
  }
}
