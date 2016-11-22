
/*!
 *************************************************************************************
 * \file img_distortion.c
 *
 * \brief
 *    Compute distortion for encoded image
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Woo-Shik Kim                    <wooshik.kim@usc.edu>
 *     - Alexis Michael Tourapis         <alexismt@ieee.org> 
 *************************************************************************************
 */

#include <math.h>
#include <time.h>

#include "global.h"

#include "refbuf.h"
#include "mbuffer.h"
#include "img_luma.h"
#include "img_chroma.h"
#include "intrarefresh.h"
#include "fmo.h"
#include "sei.h"
#include "memalloc.h"
#include "nalu.h"
#include "ratectl.h"
#include "mb_access.h"
#include "md_distortion.h"
#include "output.h"
#include "context_ini.h"
#include "conformance.h"
#include "enc_statistics.h"

#include "q_matrix.h"
#include "q_offsets.h"
#include "quant4x4.h"
#include "quant8x8.h"
#include "wp.h"
#include "input.h"
#include "image.h"
#include "img_distortion.h"
#include "img_dist_snr.h"
#include "img_dist_ssim.h"
#include "img_dist_ms_ssim.h"
#include "cconv_yuv2rgb.h"


/*!
 ************************************************************************
 * \brief
 *    Metric accumulator
 ************************************************************************
 */
void accumulate_metric(float *ave_metric, float cur_metric, int frames)
{
  *ave_metric = (float) (*ave_metric * (frames - 1) + cur_metric) / frames;
}

/*!
 ************************************************************************
 * \brief
 *    Accumulate distortion for all components
 ************************************************************************
 */
void accumulate_average(DistMetric *metric, int frames)
{
  accumulate_metric(&metric->average[0],  metric->value[0],  frames);
  accumulate_metric(&metric->average[1],  metric->value[1],  frames);
  accumulate_metric(&metric->average[2],  metric->value[2],  frames);
}

/*!
 ************************************************************************
 * \brief
 *    Accumulate distortion for all components for slice_type
 ************************************************************************
 */
void accumulate_avslice(DistMetric *metric, int slice_type, int frames)
{
  accumulate_metric(&metric->avslice[slice_type][0], metric->value[0],  frames);
  accumulate_metric(&metric->avslice[slice_type][1], metric->value[1],  frames);
  accumulate_metric(&metric->avslice[slice_type][2], metric->value[2],  frames);
}

/*!
 ************************************************************************
 * \brief
 *    Find distortion for all three components
 ************************************************************************
 */
void find_distortion (VideoParameters *p_Vid, ImageData *imgData)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  DistortionParams *p_Dist = p_Vid->p_Dist;
  int64 diff_cmp[3] = {0};

  //  Calculate SSE for Y, U and V.
  if (p_Vid->structure!=FRAME)
  {
    // Luma.
    diff_cmp[0] += compute_SSE(p_Vid->pCurImg, p_Vid->imgY_com, 0, 0, p_Inp->output.height[0], p_Inp->output.width[0]);

    // Chroma.
    if (p_Vid->yuv_format != YUV400)
    {
      diff_cmp[1] += compute_SSE(p_Vid->pImgOrg[1], p_Vid->imgUV_com[0], 0, 0, p_Inp->output.height[1], p_Inp->output.width[1]);
      diff_cmp[2] += compute_SSE(p_Vid->pImgOrg[2], p_Vid->imgUV_com[1], 0, 0, p_Inp->output.height[1], p_Inp->output.width[1]);
    }
  }
  else
  {
    if( (p_Inp->separate_colour_plane_flag != 0) )
    {
      p_Vid->enc_picture = p_Vid->enc_frame_picture[0];     
    }
    p_Vid->pCurImg   = imgData->frm_data[0];
    p_Vid->pImgOrg[0] = imgData->frm_data[0];

    // Luma.
    diff_cmp[0] += compute_SSE(p_Vid->pImgOrg[0], p_Vid->enc_picture->imgY, 0, 0, p_Inp->output.height[0], p_Inp->output.width[0]);

    // Chroma.
    if (p_Vid->yuv_format != YUV400)
    {
      p_Vid->pImgOrg[1] = imgData->frm_data[1];
      p_Vid->pImgOrg[2] = imgData->frm_data[2]; 

      diff_cmp[1] += compute_SSE(p_Vid->pImgOrg[1], p_Vid->enc_picture->imgUV[0], 0, 0, p_Inp->output.height[1], p_Inp->output.width[1]);
      diff_cmp[2] += compute_SSE(p_Vid->pImgOrg[2], p_Vid->enc_picture->imgUV[1], 0, 0, p_Inp->output.height[1], p_Inp->output.width[1]);
    }
  }

  // This should be assigned to the SSE structure. Should double check code.
  p_Dist->metric[SSE].value[0] = (float) diff_cmp[0];
  p_Dist->metric[SSE].value[1] = (float) diff_cmp[1];
  p_Dist->metric[SSE].value[2] = (float) diff_cmp[2];
}

void select_img(VideoParameters *p_Vid, ImageStructure *imgSRC, ImageStructure *imgREF, ImageData *imgData)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  if (p_Vid->fld_flag != FALSE)
  {
    imgSRC->format = p_Inp->output;
    imgREF->format = p_Inp->output;

    imgREF->data[0] = p_Vid->pCurImg;
    imgSRC->data[0] = p_Vid->imgY_com;

    if (p_Vid->yuv_format != YUV400)
    {
      imgREF->data[1] = p_Vid->pImgOrg[1];
      imgREF->data[2] = p_Vid->pImgOrg[2];
      imgSRC->data[1] = p_Vid->imgUV_com[0];
      imgSRC->data[2] = p_Vid->imgUV_com[1];
    }
  }
  else
  {
    imgSRC->format = p_Inp->output;
    imgREF->format = p_Inp->output;

    imgREF->data[0] = imgData->frm_data[0];

    if ((p_Inp->PicInterlace == ADAPTIVE_CODING) || (p_Inp->separate_colour_plane_flag != 0))
    {
      p_Vid->enc_picture = p_Vid->enc_frame_picture[0];
    }
    imgSRC->data[0] = p_Vid->enc_picture->imgY;

    if (p_Vid->yuv_format != YUV400)
    {
      imgREF->data[1] = imgData->frm_data[1];
      imgREF->data[2] = imgData->frm_data[2];

      imgSRC->data[1] = p_Vid->enc_picture->imgUV[0];
      imgSRC->data[2] = p_Vid->enc_picture->imgUV[1];
    }
  }
}

void compute_distortion(VideoParameters *p_Vid, ImageData *imgData)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  DistortionParams *p_Dist = p_Vid->p_Dist;
  if (p_Inp->Verbose != 0)
  {
    select_img(p_Vid, &p_Vid->imgSRC, &p_Vid->imgREF, imgData);

    find_snr (p_Vid, &p_Vid->imgREF, &p_Vid->imgSRC, &p_Dist->metric[SSE], &p_Dist->metric[PSNR]);
    if (p_Inp->Distortion[SSIM] == 1)
      find_ssim (p_Vid, p_Inp, &p_Vid->imgREF, &p_Vid->imgSRC, &p_Dist->metric[SSIM]);
    if (p_Inp->Distortion[MS_SSIM] == 1)
      find_ms_ssim(p_Vid, p_Inp, &p_Vid->imgREF, &p_Vid->imgSRC, &p_Dist->metric[MS_SSIM]);
    // RGB Distortion
    if(p_Inp->DistortionYUVtoRGB == 1)
    {
      YUVtoRGB(p_Vid, &p_Vid->imgREF, &p_Vid->imgRGB_ref);
      YUVtoRGB(p_Vid, &p_Vid->imgSRC, &p_Vid->imgRGB_src);
      find_snr (p_Vid, &p_Vid->imgRGB_ref, &p_Vid->imgRGB_src, &p_Dist->metric[SSE_RGB], &p_Dist->metric[PSNR_RGB]);
      if (p_Inp->Distortion[SSIM] == 1)
        find_ssim (p_Vid, p_Inp, &p_Vid->imgRGB_ref, &p_Vid->imgRGB_src, &p_Dist->metric[SSIM_RGB]);
      if (p_Inp->Distortion[MS_SSIM] == 1)
        find_ms_ssim(p_Vid, p_Inp, &p_Vid->imgRGB_ref, &p_Vid->imgRGB_src, &p_Dist->metric[MS_SSIM_RGB]);
    }
  }
}
