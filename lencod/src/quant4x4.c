
/*!
 *************************************************************************************
 * \file quant4x4.c
 *
 * \brief
 *    Quantization process for a 4x4 block
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Alexis Michael Tourapis                  <alexismt@ieee.org>
 *
 *************************************************************************************
 */

#include "contributors.h"
#include "global.h"
#include "quant4x4.h"

/*!
************************************************************************
* \brief
*    Quantization initialization function
*
************************************************************************
*/
void init_quant_4x4(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;

  if (currSlice->UseRDOQuant == 1)
  {
    currSlice->quant_4x4     = quant_4x4_trellis;
    if (p_Inp->RDOQ_DC == 1)
      currSlice->quant_dc4x4 = quant_dc4x4_trellis;
    else
      currSlice->quant_dc4x4 = quant_dc4x4_normal;
    currSlice->quant_ac4x4   = quant_ac4x4_trellis;
    if (currSlice->symbol_mode == CAVLC)
    {
      currSlice->rdoq_4x4   = rdoq_4x4_CAVLC;
      currSlice->rdoq_dc    = rdoq_dc_CAVLC;
      currSlice->rdoq_ac4x4 = rdoq_ac4x4_CAVLC;
    }
    else
    {
      currSlice->rdoq_4x4   = rdoq_4x4_CABAC;
      currSlice->rdoq_dc    = rdoq_dc_CABAC;
      currSlice->rdoq_ac4x4 = rdoq_ac4x4_CABAC;
    }
  }
  else if (p_Vid->AdaptiveRounding)
  {
    currSlice->quant_4x4     = quant_4x4_around;
    currSlice->quant_dc4x4   = quant_dc4x4_normal;
    currSlice->quant_ac4x4   = quant_ac4x4_around;
  }
  else
  {
    currSlice->quant_4x4   = quant_4x4_normal;
    currSlice->quant_dc4x4 = quant_dc4x4_normal;
    currSlice->quant_ac4x4 = quant_ac4x4_normal;
  }
}

