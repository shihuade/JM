
/*!
 *************************************************************************************
 * \file quantChroma_trellis.c
 *
 * \brief
 *    Quantization process for a Chroma block (trellis based)
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
#include "q_matrix.h"
#include "quant4x4.h"
#include "quantChroma.h"
#include "rdoq.h"

/*!
 ************************************************************************
 * \brief
 *    Quantization process for All coefficients for a 2x2 DC block
 *
 * \par Input:
 *
 * \par Output:
 *
 ************************************************************************
 */
int quant_dc2x2_trellis(Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                       LevelQuantParams *q_params_4x4, int **fadjust, const byte (*pos_scan)[2])
{
  Slice *currSlice = currMB->p_Slice;
  QuantParameters *p_Quant = currMB->p_Vid->p_Quant;
  Boolean is_cavlc = (Boolean) (currSlice->symbol_mode == CAVLC);
  int coeff_ctr;

  int *m7;

  int   level, run = 0;
  int   nonzero = FALSE;  
  int   qp_per = p_Quant->qp_per_matrix[qp];
  int   qp_rem = p_Quant->qp_rem_matrix[qp]; 
  //const byte *p_scan = &pos_scan[0][0];
  int*  DCL = &DCLevel[0];
  int*  DCR = &DCRun[0];

  int levelTrellis[16];

  currSlice->rdoq_dc_cr(currMB, tblock,qp_per,qp_rem, q_params_4x4, pos_scan, levelTrellis, CHROMA_DC);

  m7 = *tblock;

  // Quantization
  for (coeff_ctr=0; coeff_ctr < 4; coeff_ctr++)
  {
    // we need to update q_params_4x4->OffsetComp to a 4x1 array that would contain offset info for 
    // every 2x2 DC position
    if (*m7)
    {
      level = levelTrellis[coeff_ctr];

      if (level  != 0)
      {
        if (is_cavlc)
          level = imin(level, CAVLC_LEVEL_LIMIT);

        level = isignab(level, *m7);

        *m7++ = ((level * q_params_4x4->InvScaleComp) << qp_per);

        *DCL++  = level;
        *DCR++  = run;
        // reset zero level counter
        run     = 0;
        nonzero = TRUE;
      }
      else
      {
        run++;
        *m7++ = 0;
      }
    }
    else
    {
      run++;
      m7++;
    }
  }

  *DCL = 0;

  return nonzero;
}

/*!
 ************************************************************************
 * \brief
 *    Quantization process for All coefficients for a 2x2 DC block
 *
 * \par Input:
 *
 * \par Output:
 *
 ************************************************************************
 */
int quant_dc4x2_trellis(Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                       LevelQuantParams *q_params_4x4, int **fadjust, const byte (*pos_scan)[2])
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

  currSlice->rdoq_dc_cr(currMB, tblock,qp_per,qp_rem, q_params_4x4,pos_scan, levelTrellis, CHROMA_DC_2x4);

  for (coeff_ctr=0; coeff_ctr < 8; coeff_ctr++)
  {
    j = *p_scan++;  // note that in this part, somehow coefficients were transposed from 2x4 to 4x2.
    i = *p_scan++;  

    m7 = &tblock[j][i];

    if (*m7 != 0)
    {
      level = levelTrellis[coeff_ctr];

      if (level  != 0)
      {
        if (is_cavlc)
          level = imin(level, CAVLC_LEVEL_LIMIT);
        level = isignab(level, *m7);

        *m7 = ((level * q_params_4x4->InvScaleComp) << qp_per);

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
*    all coefficients in a chroma DC block
*
************************************************************************
*/
void rdoq_dc_cr_CAVLC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, LevelQuantParams *q_params_4x4, const byte (*pos_scan)[2], int levelTrellis[], int type)
{
  VideoParameters *p_Vid = currMB->p_Vid;

  const byte *p_scan = &pos_scan[0][0];
  levelDataStruct levelData[16];
  double  lambda_md = 0.0;

  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 

  init_trellis_data_DC_cr_CAVLC(currMB, tblock, qp_per, qp_rem, q_params_4x4, p_scan, &levelData[0]);
  est_RunLevel_CAVLC(currMB, levelData, levelTrellis, CHROMA_DC, 0, 0, p_Vid->num_cdc_coeff, lambda_md);
}

/*!
************************************************************************
* \brief
*    Rate distortion optimized Quantization process for 
*    all coefficients in a chroma DC block
*
************************************************************************
*/
void rdoq_dc_cr_CABAC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, LevelQuantParams *q_params_4x4, const byte (*pos_scan)[2], int levelTrellis[], int type)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  const byte *p_scan = &pos_scan[0][0];
  levelDataStruct levelData[16];
  double  lambda_md = 0.0;
  int kStart=0, kStop=0, noCoeff = 0, estBits;

  lambda_md = p_Vid->lambda_rdoq[p_Vid->type][p_Vid->masterQP]; 

  noCoeff = init_trellis_data_DC_cr_CABAC(currMB, tblock, qp_per, qp_rem, q_params_4x4, p_scan, &levelData[0], &kStart, &kStop);
  estBits = est_write_and_store_CBP_block_bit(currMB, type);
  est_writeRunLevel_CABAC(currMB, levelData, levelTrellis, type, lambda_md, kStart, kStop, noCoeff, estBits);
}


