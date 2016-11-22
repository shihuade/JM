
/*!
 *************************************************************************************
 * \file quantChroma.c
 *
 * \brief
 *    Quantization initialization function for Chroma blocks
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
#include "quantChroma.h"

/*!
************************************************************************
* \brief
*    Quantization initialization function
*
************************************************************************
*/
void init_quant_Chroma(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;

  if (p_Inp->UseRDOQuant == 1 && p_Inp->RDOQ_CR == 1)
  {
    currSlice->quant_ac4x4cr = quant_ac4x4_trellis;
    if (p_Inp->RDOQ_DC_CR)
    {
      if (p_Vid->yuv_format == YUV422)
        currSlice->quant_dc_cr = quant_dc4x2_trellis;
      else
        currSlice->quant_dc_cr = quant_dc2x2_trellis;
    }
    else
    {
      if (p_Vid->yuv_format == YUV422)
        currSlice->quant_dc_cr   = quant_dc4x2_normal;
      else
        currSlice->quant_dc_cr   = quant_dc2x2_normal;
    }
    if (currSlice->symbol_mode == CABAC)
      currSlice->rdoq_dc_cr = rdoq_dc_cr_CABAC;
    else
      currSlice->rdoq_dc_cr = rdoq_dc_cr_CAVLC;
  }
  else if (p_Inp->UseRDOQuant == 1 || (!(currSlice->p_Vid)->AdaptiveRounding))
  {
    currSlice->quant_ac4x4cr = quant_ac4x4_normal;
    if (p_Vid->yuv_format == YUV422)
      currSlice->quant_dc_cr   = quant_dc4x2_normal;
    else
      currSlice->quant_dc_cr   = quant_dc2x2_normal;
  }
  else
  {
    currSlice->quant_ac4x4cr = quant_ac4x4_around;
    if (p_Vid->yuv_format == YUV422)
      currSlice->quant_dc_cr   = quant_dc4x2_around;
    else
      currSlice->quant_dc_cr   = quant_dc2x2_around;
  }
}


