
/*!
 *************************************************************************************
 * \file img_dist_snr.c
 *
 * \brief
 *    Compute signal to noise ratio (SNR) between the encoded image and the reference image
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Woo-Shik Kim                    <wooshik.kim@usc.edu>
 *     - Alexis Michael Tourapis         <alexismt@ieee.org>
 *************************************************************************************
 */
#include "contributors.h"

#include <math.h>

#include "enc_statistics.h"
#include "global.h"
#include "img_distortion.h"
#include "md_distortion.h"

/*!
 ************************************************************************
 * \brief
 *    Find SNR for all three components
 ************************************************************************
 */
void find_snr(VideoParameters *p_Vid, ImageStructure *imgREF, ImageStructure *imgSRC, DistMetric *metricSSE, DistMetric *metricPSNR)
{
  DistortionParams *p_Dist = p_Vid->p_Dist;
  FrameFormat *format = &imgREF->format;
  // Luma.
  metricSSE ->value[0] = (float) compute_SSE(imgREF->data[0], imgSRC->data[0], 0, 0, format->height[0], format->width[0]);
  metricPSNR->value[0] = psnr(format->max_value_sq[0], format->size_cmp[0], metricSSE->value[0]);  
  // Chroma.
  if (format->yuv_format != YUV400)
  {   
    metricSSE ->value[1] = (float) compute_SSE(imgREF->data[1], imgSRC->data[1], 0, 0, format->height[1], format->width[1]);
    metricPSNR->value[1] = psnr(format->max_value_sq[1], format->size_cmp[1], metricSSE->value[1]);
    metricSSE ->value[2] = (float) compute_SSE(imgREF->data[2], imgSRC->data[2], 0, 0, format->height[1], format->width[1]);
    metricPSNR->value[2] = psnr(format->max_value_sq[2], format->size_cmp[2], metricSSE->value[2]);
  }
#if (MVC_EXTENSION_ENABLE)
  if (p_Vid->p_Inp->num_of_views == 2)
  {
    // Luma
    p_Vid->p_Dist->metric_v[p_Vid->view_id][SSE].value[0]  = metricSSE ->value[0];
    p_Vid->p_Dist->metric_v[p_Vid->view_id][PSNR].value[0] = metricPSNR->value[0];
    // Chroma
    if (format->yuv_format != YUV400)
    {
      p_Vid->p_Dist->metric_v[p_Vid->view_id][SSE].value[1]  = metricSSE ->value[1];
      p_Vid->p_Dist->metric_v[p_Vid->view_id][PSNR].value[1] = metricPSNR->value[1];
      p_Vid->p_Dist->metric_v[p_Vid->view_id][SSE].value[2]  = metricSSE ->value[2];
      p_Vid->p_Dist->metric_v[p_Vid->view_id][PSNR].value[2] = metricPSNR->value[2];
    }
  }
#endif
   
  {
    accumulate_average(metricSSE,  p_Dist->frame_ctr);
    accumulate_average(metricPSNR, p_Dist->frame_ctr);

    accumulate_avslice(metricSSE,  p_Vid->type, p_Vid->p_Stats->frame_ctr[p_Vid->type]);
    accumulate_avslice(metricPSNR, p_Vid->type, p_Vid->p_Stats->frame_ctr[p_Vid->type]);
#if (MVC_EXTENSION_ENABLE)
    if (p_Vid->p_Inp->num_of_views == 2)
    {
      accumulate_average(&p_Dist->metric_v[p_Vid->view_id][SSE],  p_Dist->frame_ctr_v[p_Vid->view_id]);
      accumulate_average(&p_Dist->metric_v[p_Vid->view_id][PSNR], p_Dist->frame_ctr_v[p_Vid->view_id]);
    }
#endif
  }
}
