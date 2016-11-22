
/*!
 *************************************************************************************
 * \file quant8x8_around.c
 *
 * \brief
 *    Quantization process for a 8x8 block with adaptive rounding
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
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
#include "quant8x8.h"


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
int quant_8x8_around(Macroblock *currMB, int **tblock, struct quant_methods *q_method)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  QuantParameters *p_Quant = p_Vid->p_Quant;
  int AdaptRndWeight = p_Vid->AdaptRndWeight;

  int block_x = q_method->block_x;

  int  qp = q_method->qp;
  int*  ACLevel = q_method->ACLevel;
  int*  ACRun   = q_method->ACRun;
  LevelQuantParams **q_params_8x8 = q_method->q_params;
  LevelQuantParams *q_params = NULL;

  int **fadjust8x8 = q_method->fadjust;
  const byte (*pos_scan)[2] = q_method->pos_scan;
  const byte *c_cost = q_method->c_cost;
  int *coeff_cost = q_method->coeff_cost;

  int i,j, coeff_ctr;

  int *m7;
  int scaled_coeff;

  int   level, run = 0;
  int   nonzero = FALSE;
  int   qp_per = p_Quant->qp_per_matrix[qp];
  int   q_bits = Q_BITS_8 + qp_per;
  const byte *p_scan = &pos_scan[0][0];
  int*  ACL = &ACLevel[0];
  int*  ACR = &ACRun[0];
  int*  padjust8x8;

  // Quantization
  for (coeff_ctr = 0; coeff_ctr < 64; ++coeff_ctr)
  {
    i = *p_scan++;  // horizontal position
    j = *p_scan++;  // vertical position
    
    padjust8x8 = &fadjust8x8[j][block_x + i];
    m7 = &tblock[j][block_x + i];
    if (*m7 != 0)
    {
      q_params = &q_params_8x8[j][i];
      scaled_coeff = iabs (*m7) * q_params->ScaleComp;
      level = (scaled_coeff + q_params->OffsetComp) >> q_bits;

      if (level != 0)
      {
        *padjust8x8 = rshift_rnd_sf((AdaptRndWeight * (scaled_coeff - (level << q_bits))), (q_bits + 1));

        nonzero = TRUE;

        *coeff_cost += (level > 1) ? MAX_VALUE : c_cost[run];

        level  = isignab(level, *m7);
        *m7    = rshift_rnd_sf(((level * q_params->InvScaleComp) << qp_per), 6);
        *ACL++ = level;
        *ACR++ = run; 
        // reset zero level counter
        run    = 0;
      }
      else
      {
        *padjust8x8 = 0;
        ++run;
        *m7 = 0;
      }
    }
    else
    {
      *padjust8x8 = 0;
      ++run;
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
int quant_8x8cavlc_around(Macroblock *currMB, int **tblock, struct quant_methods *q_method, int***  cofAC)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  QuantParameters *p_Quant = p_Vid->p_Quant;
  int AdaptRndWeight = p_Vid->AdaptRndWeight;
  int block_x = q_method->block_x;

  int  qp = q_method->qp;
  LevelQuantParams **q_params_8x8 = q_method->q_params;
  int **fadjust8x8 = q_method->fadjust;
  const byte (*pos_scan)[2] = q_method->pos_scan;
  const byte *c_cost = q_method->c_cost;
  int *coeff_cost = q_method->coeff_cost;  

  int i,j, k, coeff_ctr;

  int *m7;
  int scaled_coeff;  

  int level, runs[4] = { 0 };
  int nonzero = FALSE; 
  int qp_per = p_Quant->qp_per_matrix[qp];  
  int q_bits = Q_BITS_8 + qp_per;
  const byte *p_scan = &pos_scan[0][0];
  int*  ACL[4];  
  int*  ACR[4];
  int*  padjust8x8;

  for (k = 0; k < 4; ++k)
  {
    ACL[k] = &cofAC[k][0][0];
    ACR[k] = &cofAC[k][1][0];
  }

  // Quantization
  for (k = 0; k < 4; ++k)
  {
    for (coeff_ctr = 0; coeff_ctr < 16; ++coeff_ctr)
    {
      i = *p_scan++;  // horizontal position
      j = *p_scan++;  // vertical position

      padjust8x8 = &fadjust8x8[j][block_x + i];
      m7 = &tblock[j][block_x + i];
      if (*m7 != 0)
      {
        scaled_coeff = iabs (*m7) * q_params_8x8[j][i].ScaleComp;
        level = (scaled_coeff + q_params_8x8[j][i].OffsetComp) >> q_bits;

        if (level != 0)
        {
          level = imin(level, CAVLC_LEVEL_LIMIT);

          *padjust8x8 = rshift_rnd_sf((AdaptRndWeight * (scaled_coeff - (level << q_bits))), (q_bits + 1));

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
          *padjust8x8 = 0;
          *m7 = 0;    
          ++runs[k];
        }
      }
      else
      {
        *padjust8x8 = 0;
        ++runs[k];
      }
    }
  }

  for(k = 0; k < 4; ++k)
    *(ACL[k]) = 0;

  return nonzero;
}

