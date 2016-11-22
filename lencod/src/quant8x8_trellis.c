
/*!
 *************************************************************************************
 * \file quant8x8_trellis.c
 *
 * \brief
 *    Quantization process for a 4x4 block using trellis based quantization
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
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
#include "quant8x8.h"
#include "rdoq.h"

/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a 8x8 block
*
************************************************************************
*/
static void rdoq_8x8_CABAC(Macroblock *currMB, int **tblock, int block_x,int qp_per, int qp_rem, 
              LevelQuantParams **q_params_8x8, const byte *p_scan, int levelTrellis[64])
{
  VideoParameters *p_Vid = currMB->p_Vid;
  levelDataStruct levelData[64];
  double  lambda_md = 0.0;
  int kStart = 0, kStop = 0, noCoeff = 0;

  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 

  noCoeff = init_trellis_data_8x8_CABAC(currMB, tblock, block_x, qp_per, qp_rem, q_params_8x8, p_scan, &levelData[0], &kStart, &kStop);
  est_writeRunLevel_CABAC(currMB, levelData, levelTrellis, LUMA_8x8, lambda_md, kStart, kStop, noCoeff, 0);
}

/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a 8x8 block
*
************************************************************************
*/
static void rdoq_8x8_CAVLC(Macroblock *currMB, int **tblock, int block_y, int block_x, int qp_per, int qp_rem,
                    LevelQuantParams **q_params_8x8, const byte *p_scan, int levelTrellis[4][16])
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int k;
  levelDataStruct levelData[4][16]; 
  double lambda_md = 0.0;
  
  int b8 = ((block_y >> 3) << 1) + (block_x >> 3);

  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 

  init_trellis_data_8x8_CAVLC (currMB, tblock, block_x, qp_per, qp_rem, q_params_8x8, p_scan, levelData);

  for (k = 0; k < 4; k++)
    est_RunLevel_CAVLC(currMB, levelData[k], levelTrellis[k], LUMA, b8, k, 16, lambda_md);
}


/*!
 ************************************************************************
 * \brief
 *    Quantization process for All coefficients for a 8x8 block
 *
 * \par Input:
 *
 * \par Output:
 *
 ************************************************************************
 */
int quant_8x8_trellis(Macroblock *currMB, int **tblock, struct quant_methods *q_method)
{
  QuantParameters *p_Quant = currMB->p_Vid->p_Quant;
  int block_x = q_method->block_x;
  int  qp = q_method->qp;
  int*  ACLevel = q_method->ACLevel;
  int*  ACRun   = q_method->ACRun;
  LevelQuantParams **q_params_8x8 = q_method->q_params;
  const byte (*pos_scan)[2] = q_method->pos_scan;
  const byte *c_cost = q_method->c_cost;
  int *coeff_cost = q_method->coeff_cost;

  int i,j, coeff_ctr;

  int  *m7;
  int   level, run = 0;
  int   nonzero = FALSE;
  int   qp_per = p_Quant->qp_per_matrix[qp];
  int   qp_rem = p_Quant->qp_rem_matrix[qp];
  const byte *p_scan = &pos_scan[0][0];
  int*  ACL = &ACLevel[0];
  int*  ACR = &ACRun[0];
  int   levelTrellis[64];

  rdoq_8x8_CABAC(currMB, tblock, block_x, qp_per, qp_rem, q_params_8x8, p_scan, levelTrellis);

  // Quantization
  for (coeff_ctr = 0; coeff_ctr < 64; coeff_ctr++)
  {
    i = *p_scan++;  // horizontal position
    j = *p_scan++;  // vertical position

    m7 = &tblock[j][block_x + i];
    if (*m7 != 0)
    {    
      level = levelTrellis[coeff_ctr];

      if (level != 0)
      {
        nonzero = TRUE;

        *coeff_cost += (level > 1) ? MAX_VALUE : c_cost[run];

        level  = isignab(level, *m7);
        *m7    = rshift_rnd_sf(((level * q_params_8x8[j][i].InvScaleComp) << qp_per), 6);
        *ACL++ = level;
        *ACR++ = run; 
        // reset zero level counter
        run    = 0;
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
 *    Quantization process for All coefficients for a 8x8 block 
 *    CAVLC version
 *
 * \par Input:
 *
 * \par Output:
 *
 ************************************************************************
 */
int quant_8x8cavlc_trellis(Macroblock *currMB, int **tblock, struct quant_methods *q_method, int***  cofAC)
{
  QuantParameters *p_Quant = currMB->p_Vid->p_Quant;
  int block_x = q_method->block_x;
  int block_y = q_method->block_y;
  int  qp = q_method->qp;
  LevelQuantParams **q_params_8x8 = q_method->q_params;
  const byte (*pos_scan)[2] = q_method->pos_scan;
  const byte *c_cost = q_method->c_cost;
  int *coeff_cost = q_method->coeff_cost;

  int i,j, k, coeff_ctr;

  int *m7;
  int level, runs[4] = { 0 };
  int nonzero = FALSE; 
  int qp_per = p_Quant->qp_per_matrix[qp];  
  int qp_rem = p_Quant->qp_rem_matrix[qp];
  const byte *p_scan = &pos_scan[0][0];
  int*  ACL[4];  
  int*  ACR[4];

  int levelTrellis[4][16];

  rdoq_8x8_CAVLC(currMB, tblock, block_y, block_x, qp_per, qp_rem, q_params_8x8, p_scan, levelTrellis);

  for (k = 0; k < 4; k++)
  {
    ACL[k] = &cofAC[k][0][0];
    ACR[k] = &cofAC[k][1][0];
  }

  // Quantization
  for (k = 0; k < 4; k++)
  {
    for (coeff_ctr = 0; coeff_ctr < 16; coeff_ctr++)
    {
      i = *p_scan++;  // horizontal position
      j = *p_scan++;  // vertical position

      m7 = &tblock[j][block_x + i];

      if (m7 != 0)
      {
        level = levelTrellis[k][coeff_ctr];

        if (level != 0)
        {
          nonzero=TRUE;

          *coeff_cost += (level > 1) ? MAX_VALUE : c_cost[runs[k]];

          level  = isignab(level, *m7);
          *m7    = rshift_rnd_sf(((level * q_params_8x8[j][i].InvScaleComp) << qp_per), 6);

          *(ACL[k])++ = level;
          *(ACR[k])++ = runs[k];
          // reset zero level counter
          runs[k] = 0;
        }
        else
        {        
          runs[k]++;
          *m7 = 0;      
        }
      }
      else
      {
        runs[k]++;
      }
    }
  }

  for(k = 0; k < 4; k++)
    *(ACL[k]) = 0;

  return nonzero;
}

