
/*!
 *************************************************************************************
 * \file quant4x4_trellis.c
 *
 * \brief
 *    Quantization process for a 4x4 block using trellis based quantization
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Qualcomm      
 *    - Limin Liu                                <limin.liu@dolby.com>
 *    - Alexis Michael Tourapis                  <alexismt@ieee.org>
 *
 *************************************************************************************
 */

#include "contributors.h"

#include <math.h>

#include "global.h"

#include "image.h"
#include "mb_access.h"
#include "vlc.h"
#include "transform.h"
#include "mc_prediction.h"
#include "q_offsets.h"
#include "q_matrix.h"
#include "quant4x4.h"
#include "rdoq.h"

/*!
 ************************************************************************
 * \brief
 *    Quantization process for All coefficients for a 4x4 block
 *
 ************************************************************************
 */
int quant_4x4_trellis(Macroblock *currMB, int **tblock, struct quant_methods *q_method)
{
  int   block_x = q_method->block_x;

  int*  ACL = &q_method->ACLevel[0];
  int*  ACR = &q_method->ACRun[0];  
  Slice *currSlice = currMB->p_Slice;
  QuantParameters *p_Quant = currMB->p_Vid->p_Quant;
  int  qp = q_method->qp;
  LevelQuantParams **q_params_4x4 = q_method->q_params;
  const byte (*pos_scan)[2] = q_method->pos_scan;
  const byte *c_cost = q_method->c_cost;
  int *coeff_cost = q_method->coeff_cost;

  Boolean is_cavlc = (Boolean) (currSlice->symbol_mode == CAVLC);

  int i,j, coeff_ctr;

  int *m7;

  int   level, run = 0;
  int   nonzero = FALSE;
  int   qp_per = p_Quant->qp_per_matrix[qp];
  const byte *p_scan = &pos_scan[0][0];

  int levelTrellis[16];

  currSlice->rdoq_4x4(currMB, tblock, q_method, levelTrellis);

  // Quantization
  for (coeff_ctr = 0; coeff_ctr < 16; ++coeff_ctr)
  {
    i = *p_scan++;  // horizontal position
    j = *p_scan++;  // vertical position

    m7 = &tblock[j][block_x + i];

    if (*m7 != 0)
    {    
      /*
      scaled_coeff = iabs (*m7) * q_params_4x4[j][i].ScaleComp;
      level = (scaled_coeff + q_params_4x4[j][i].OffsetComp) >> q_bits;
      */
      level = levelTrellis[coeff_ctr];

      if (level != 0)
      {
        if (is_cavlc)
          level = imin(level, CAVLC_LEVEL_LIMIT);

        *coeff_cost += (level > 1) ? MAX_VALUE : c_cost[run];

        level   = isignab(level, *m7);
        *m7     = rshift_rnd_sf(((level * q_params_4x4[j][i].InvScaleComp) << qp_per), 4);
        *ACL++  = level;
        *ACR++  = run; 
        // reset zero level counter
        run     = 0;
        nonzero = TRUE;        
      }
      else
      {
        *m7 = 0;
        ++run;
      } 
    }
    else
    {
      ++run;
    } 
  }

  *ACL = 0;

  return nonzero;
}

/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a 4x4 block (CAVLC)
*
************************************************************************
*/
void rdoq_4x4_CAVLC(Macroblock *currMB, int **tblock, struct quant_methods *q_method, int levelTrellis[])
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int   block_x = q_method->block_x;
  int   block_y = q_method->block_y;
  LevelQuantParams **q_params_4x4 = q_method->q_params;
  const byte (*pos_scan)[2] = q_method->pos_scan;
  const byte *p_scan = &pos_scan[0][0];
  int  qp = q_method->qp;
  QuantParameters *p_Quant = currMB->p_Vid->p_Quant;
  int   qp_per = p_Quant->qp_per_matrix[qp];
  int   qp_rem = p_Quant->qp_rem_matrix[qp];

  levelDataStruct levelData[16];  
  double  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 

  int type = LUMA_4x4;
  int   pos_x   = block_x >> BLOCK_SHIFT;
  int   pos_y   = block_y >> BLOCK_SHIFT;
  int   b8      = 2*(pos_y >> 1) + (pos_x >> 1);
  int   b4      = 2*(pos_y & 0x01) + (pos_x & 0x01);

  init_trellis_data_4x4_CAVLC(currMB, tblock, block_x, qp_per, qp_rem, q_params_4x4, p_scan, &levelData[0], type);
  est_RunLevel_CAVLC(currMB, levelData, levelTrellis, LUMA, b8, b4, 16, lambda_md);
}
/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a 4x4 block (CABAC)
*
************************************************************************
*/
void rdoq_4x4_CABAC(Macroblock *currMB, int **tblock, struct quant_methods *q_method, int levelTrellis[])
{
  VideoParameters *p_Vid = currMB->p_Vid;
  
  const byte (*pos_scan)[2] = q_method->pos_scan;
  const byte *p_scan = &pos_scan[0][0];

  levelDataStruct levelData[16];
  int kStart=0, kStop=0, noCoeff = 0, estBits;

  double lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 

  noCoeff = init_trellis_data_4x4_CABAC(currMB, tblock, q_method, p_scan, &levelData[0], &kStart, &kStop, LUMA_4x4);
  estBits = est_write_and_store_CBP_block_bit(currMB, LUMA_4x4);
  est_writeRunLevel_CABAC(currMB, levelData, levelTrellis, LUMA_4x4, lambda_md, kStart, kStop, noCoeff, estBits);
}

/*!
 ************************************************************************
 * \brief
 *    Quantization process for All coefficients for a 4x4 block (LUMA_16AC or CHROMA_AC)
 *
 ************************************************************************
 */
int quant_ac4x4_trellis(Macroblock *currMB, int **tblock, struct quant_methods *q_method)
{
  int   block_x = q_method->block_x;

  int*  ACLevel = q_method->ACLevel;
  int*  ACRun   = q_method->ACRun;
  int qp = q_method->qp;
  LevelQuantParams **q_params_4x4 = q_method->q_params;
  const byte (*pos_scan)[2] = q_method->pos_scan;
  const byte *c_cost = q_method->c_cost;
  int *coeff_cost = q_method->coeff_cost;

  Slice *currSlice = currMB->p_Slice;
  QuantParameters *p_Quant = currMB->p_Vid->p_Quant;
  Boolean is_cavlc = (Boolean) (currSlice->symbol_mode == CAVLC);
  int i,j, coeff_ctr;

  int *m7;
  int   level, run = 0;
  int   nonzero = FALSE;  
  int   qp_per = p_Quant->qp_per_matrix[qp];
  const byte *p_scan = &pos_scan[1][0];
  int*  ACL = &ACLevel[0];
  int*  ACR = &ACRun[0];

  int levelTrellis[16]; 

  currSlice->rdoq_ac4x4(currMB, tblock, q_method, levelTrellis);

  // Quantization
  for (coeff_ctr = 1; coeff_ctr < 16; coeff_ctr++)
  {
    i = *p_scan++;  // horizontal position
    j = *p_scan++;  // vertical position

    m7 = &tblock[j][block_x + i];
    if (*m7 != 0)
    {    
      /*
      scaled_coeff = iabs (*m7) * q_params_4x4[j][i].ScaleComp;
      level = (scaled_coeff + q_params_4x4[j][i].OffsetComp) >> q_bits;
      */
      level=levelTrellis[coeff_ctr - 1];

      if (level != 0)
      {
        if (is_cavlc)
          level = imin(level, CAVLC_LEVEL_LIMIT);

        *coeff_cost += (level > 1) ? MAX_VALUE : c_cost[run];

        level  = isignab(level, *m7);
        *m7    = rshift_rnd_sf(((level * q_params_4x4[j][i].InvScaleComp) << qp_per), 4);
        // inverse scale can be alternative performed as follows to ensure 16bit
        // arithmetic is satisfied.
        // *m7 = (qp_per<4) ? rshift_rnd_sf((level*q_params_4x4[j][i].InvScaleComp),4-qp_per) : (level*q_params_4x4[j][i].InvScaleComp)<<(qp_per-4);
        *ACL++  = level;
        *ACR++  = run; 
        // reset zero level counter
        run     = 0;
        nonzero = TRUE;
      }
      else
      {
        run++;
        *m7 = 0;
      }
    }
    else
    {
      run++;
    }          
  }

  *ACL = 0;

  return nonzero;
}

/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a 4x4 block (CAVLC)
*
************************************************************************
*/
void rdoq_ac4x4_CAVLC(Macroblock *currMB, int **tblock, struct quant_methods *q_method, int levelTrellis[])
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int   block_x = q_method->block_x;
  int   block_y = q_method->block_y;
  LevelQuantParams **q_params_4x4 = q_method->q_params;
  const byte (*pos_scan)[2] = q_method->pos_scan;
  int  qp = q_method->qp;
  int  type = q_method->type;
  QuantParameters *p_Quant = p_Vid->p_Quant;
  int   qp_per = p_Quant->qp_per_matrix[qp];
  int   qp_rem = p_Quant->qp_rem_matrix[qp];

  const byte *p_scan = &pos_scan[1][0];
  levelDataStruct levelData[16];  
  double  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 

  int   pos_x   = block_x >> BLOCK_SHIFT;
  int   pos_y   = block_y >> BLOCK_SHIFT;
  int   b8      = 2*(pos_y >> 1) + (pos_x >> 1);
  int   b4      = 2*(pos_y & 0x01) + (pos_x & 0x01);
  int   block_type = ( (type == CHROMA_AC) ? CHROMA_AC : LUMA_INTRA16x16AC);

  init_trellis_data_4x4_CAVLC(currMB, tblock, block_x, qp_per, qp_rem, q_params_4x4, p_scan, &levelData[0], type);
  est_RunLevel_CAVLC(currMB, levelData, levelTrellis, block_type, b8, b4, 15, lambda_md);
}
/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a 4x4 block (LUMA_16AC or CHROMA_AC) - CABAC
*
************************************************************************
*/
void rdoq_ac4x4_CABAC(Macroblock *currMB, int **tblock, struct quant_methods *q_method, int levelTrellis[])
{
  VideoParameters *p_Vid = currMB->p_Vid;
  
  const byte (*pos_scan)[2] = q_method->pos_scan;
  int  type = q_method->type;

  const byte *p_scan = &pos_scan[1][0];
  levelDataStruct levelData[16];
  double  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 
  int kStart = 0, kStop = 0, noCoeff = 0, estBits;

  noCoeff = init_trellis_data_4x4_CABAC(currMB, tblock, q_method, p_scan, &levelData[0], &kStart, &kStop, type);
  estBits = est_write_and_store_CBP_block_bit(currMB, type);
  est_writeRunLevel_CABAC(currMB, levelData, levelTrellis, type, lambda_md, kStart, kStop, noCoeff, estBits);
}

/*!
 ************************************************************************
 * \brief
 *    Quantization process for All coefficients for a 4x4 DC block
 *
 ************************************************************************
 */
int quant_dc4x4_trellis(Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                       LevelQuantParams *q_params_4x4, const byte (*pos_scan)[2])
{
  Slice *currSlice = currMB->p_Slice;
  QuantParameters *p_Quant = currMB->p_Vid->p_Quant;
  Boolean is_cavlc = (Boolean) (currSlice->symbol_mode == CAVLC);
  int i,j, coeff_ctr;

  int *m7;

  int   level, run = 0;
  int   nonzero = FALSE;  
  int   qp_per = p_Quant->qp_per_matrix[qp];
  int   qp_rem = p_Quant->qp_rem_matrix[qp];
  const byte *p_scan = &pos_scan[0][0];
  int*  DCL = &DCLevel[0];
  int*  DCR = &DCRun[0];

  int levelTrellis[16];

  currSlice->rdoq_dc(currMB, tblock, qp_per, qp_rem, q_params_4x4, pos_scan, levelTrellis, LUMA_16DC);

  // Quantization
  for (coeff_ctr = 0; coeff_ctr < 16; coeff_ctr++)
  {
    i = *p_scan++;  // horizontal position
    j = *p_scan++;  // vertical position

    m7 = &tblock[j][i];

    if (*m7 != 0)
    {    
      level = levelTrellis[coeff_ctr];

      if (level != 0)
      {
        if (is_cavlc)
          level = imin(level, CAVLC_LEVEL_LIMIT);

        level   = isignab(level, *m7);
        *m7     = level;
        *DCL++  = level;
        *DCR++  = run;
        // reset zero level counter
        run     = 0;
        nonzero = TRUE;
      }
      else
      {
        run++;
        *m7 = 0;
      }
    }
    else
    {
      run++;
    }                    
  }

  *DCL = 0;

  return nonzero;
}

/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a luma DC block 
*
************************************************************************
*/
void rdoq_dc_CAVLC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, LevelQuantParams *q_params_4x4, 
                   const byte (*pos_scan)[2], int levelTrellis[], int type)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  const byte *p_scan = &pos_scan[0][0];
  levelDataStruct levelData[16];
  double  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 

  init_trellis_data_DC_CAVLC(currMB, tblock, qp_per, qp_rem, q_params_4x4, p_scan, &levelData[0]);
  est_RunLevel_CAVLC(currMB, levelData, levelTrellis, LUMA_INTRA16x16DC, 0, 0, 16, lambda_md);
}


/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a luma DC block 
*
************************************************************************
*/
void rdoq_dc_CABAC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, LevelQuantParams *q_params_4x4, const byte (*pos_scan)[2], int levelTrellis[], int type)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  const byte *p_scan = &pos_scan[0][0];
  levelDataStruct levelData[16];
  double  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 
  int kStart = 0, kStop = 0, noCoeff = 0, estBits;

  noCoeff = init_trellis_data_DC_CABAC(currMB, tblock, qp_per, qp_rem, q_params_4x4, p_scan, &levelData[0], &kStart, &kStop);
  estBits = est_write_and_store_CBP_block_bit(currMB, type);
  est_writeRunLevel_CABAC(currMB, levelData, levelTrellis, type, lambda_md, kStart, kStop, noCoeff, estBits);
}

