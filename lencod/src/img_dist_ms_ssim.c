
/*!
 *************************************************************************************
 * \file img_dist_ms_ssim.c
 *
 * \brief
 *    Compute structural similarity (SSIM) index using the encoded image and the reference image
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Woo-Shik Kim                    <wooshik.kim@usc.edu>
 *     - Peshala Pahalawatta             <ppaha@dolby.com>
 *     - Zhen Li                         <zli@dolby.com> 
 *     - Alexis Michael Tourapis         <alexismt@ieee.org>
 *************************************************************************************
 */
#include "contributors.h"
#include "global.h"
#include "img_distortion.h"
#include "enc_statistics.h"
#include "memalloc.h"
#include "math.h"

#ifndef UNBIASED_VARIANCE
  #define UNBIASED_VARIANCE // unbiased estimation of the variance
#endif

#define MS_SSIM_PAD  6
#define MS_SSIM_PAD2 3

#define MS_SSIM_BETA0 0.0448
#define MS_SSIM_BETA1 0.2856
#define MS_SSIM_BETA2 0.3001
#define MS_SSIM_BETA3 0.2363
#define MS_SSIM_BETA4 0.1333

#define MAX_SSIM_LEVELS 5

//Computes the product of the contrast and structure componenents of the structural similarity metric.
float compute_structural_components (VideoParameters *p_Vid, InputParameters *p_Inp, imgpel **refImg, imgpel **encImg, int height, int width, int win_height, int win_width, int comp)
{
  static const float K2 = 0.03f;
  float max_pix_value_sqd;
  float C2;
  float win_pixels = (float) (win_width * win_height);
#ifdef UNBIASED_VARIANCE
  float win_pixels_bias = win_pixels - 1;
#else
  float win_pixels_bias = win_pixels;
#endif
  float mb_ssim, meanOrg, meanEnc;
  float varOrg, varEnc, covOrgEnc;
  int imeanOrg, imeanEnc, ivarOrg, ivarEnc, icovOrgEnc;
  float cur_distortion = 0.0;
  int i, j, n, m, win_cnt = 0;
  int overlapSize = p_Inp->SSIMOverlapSize;

  max_pix_value_sqd = (float) (p_Vid->max_pel_value_comp[comp] * p_Vid->max_pel_value_comp[comp]);
  C2 = K2 * K2 * max_pix_value_sqd;

  for (j = 0; j <= height - win_height; j += overlapSize)
  {
    for (i = 0; i <= width - win_width; i += overlapSize)
    {
      imeanOrg = 0;
      imeanEnc = 0; 
      ivarOrg  = 0;
      ivarEnc  = 0;
      icovOrgEnc = 0;

      for ( n = j;n < j + win_height;n ++)
      {
        for (m = i;m < i + win_width;m ++)
        {
          imeanOrg   += refImg[n][m];
          imeanEnc   += encImg[n][m];
          ivarOrg    += refImg[n][m] * refImg[n][m];
          ivarEnc    += encImg[n][m] * encImg[n][m];
          icovOrgEnc += refImg[n][m] * encImg[n][m];
        }
      }

      meanOrg = (float) imeanOrg / win_pixels;
      meanEnc = (float) imeanEnc / win_pixels;

      varOrg    = ((float) ivarOrg - ((float) imeanOrg) * meanOrg) / win_pixels_bias;
      varEnc    = ((float) ivarEnc - ((float) imeanEnc) * meanEnc) / win_pixels_bias;
      covOrgEnc = ((float) icovOrgEnc - ((float) imeanOrg) * meanEnc) / win_pixels_bias;

      mb_ssim  = (float) (2.0 * covOrgEnc + C2);
      mb_ssim /= (float) (varOrg + varEnc + C2);

      cur_distortion += mb_ssim;
      win_cnt++;
    }
  }

  cur_distortion /= (float) win_cnt;

  if (cur_distortion >= 1.0 && cur_distortion < 1.01) // avoid float accuracy problem at very low QP(e.g.2)
    cur_distortion = 1.0;

  return cur_distortion;
}

float compute_luminance_component (VideoParameters *p_Vid, InputParameters *p_Inp, imgpel **refImg, imgpel **encImg, int height, int width, int win_height, int win_width, int comp)
{
  static const float K1 = 0.01f;
  float max_pix_value_sqd;
  float C1;
  float win_pixels = (float) (win_width * win_height);
  float mb_ssim, meanOrg, meanEnc;
  int imeanOrg, imeanEnc;
  float cur_distortion = 0.0;
  int i, j, n, m, win_cnt = 0;
  int overlapSize = p_Inp->SSIMOverlapSize;

  max_pix_value_sqd = (float) (p_Vid->max_pel_value_comp[comp] * p_Vid->max_pel_value_comp[comp]);
  C1 = K1 * K1 * max_pix_value_sqd;

  for (j = 0; j <= height - win_height; j += overlapSize)
  {
    for (i = 0; i <= width - win_width; i += overlapSize)
    {
      imeanOrg = 0;
      imeanEnc = 0; 
      
      for ( n = j;n < j + win_height;n ++)
      {
        for (m = i;m < i + win_width;m ++)
        {
          imeanOrg   += refImg[n][m];
          imeanEnc   += encImg[n][m];
        }
      }

      meanOrg = (float) imeanOrg / win_pixels;
      meanEnc = (float) imeanEnc / win_pixels;

      mb_ssim  = (float) (2.0 * meanOrg * meanEnc + C1);
      mb_ssim /= (float) (meanOrg * meanOrg + meanEnc * meanEnc + C1);

      cur_distortion += mb_ssim;
      win_cnt++;
    }
  }

  cur_distortion /= (float) win_cnt;

  if (cur_distortion >= 1.0 && cur_distortion < 1.01) // avoid float accuracy problem at very low QP(e.g.2)
    cur_distortion = 1.0;

  return cur_distortion;
}

void horizontal_symmetric_extension(int **buffer, int width, int height )
{
  int j;
  int* buf;
 
  int height_plus_pad2 = height + MS_SSIM_PAD2;
  int width_plus_pad2_minus_one  = width  + MS_SSIM_PAD2 - 1;

  // horizontal PSE
  for (j = MS_SSIM_PAD2; j < height_plus_pad2; j++ ) {
    // left column
    buf = &buffer[j][MS_SSIM_PAD2];
    buf[-1] = buf[1];
    buf[-2] = buf[2];

    // right column
    buf = &buffer[j][width_plus_pad2_minus_one];
    buf[1] = buf[-1];
    buf[2] = buf[-2];
    buf[3] = buf[-3];
  }
}

void vertical_symmetric_extension(int **buffer, int width, int height)
{
  int i;

  int *bufminus1 = &buffer[MS_SSIM_PAD2-1][MS_SSIM_PAD2];
  int *bufminus2 = &buffer[MS_SSIM_PAD2-2][MS_SSIM_PAD2];
  int *bufplus1  = &buffer[MS_SSIM_PAD2+1][MS_SSIM_PAD2];
  int *bufplus2  = &buffer[MS_SSIM_PAD2+2][MS_SSIM_PAD2];

  int *bufhminus1 = &buffer[height + MS_SSIM_PAD2-2][MS_SSIM_PAD2];
  int *bufhminus2 = &buffer[height + MS_SSIM_PAD2-3][MS_SSIM_PAD2];
  int *bufhminus3 = &buffer[height + MS_SSIM_PAD2-4][MS_SSIM_PAD2];
  int *bufhplus1  = &buffer[height + MS_SSIM_PAD2][MS_SSIM_PAD2];
  int *bufhplus2  = &buffer[height + MS_SSIM_PAD2+1][MS_SSIM_PAD2];
  int *bufhplus3  = &buffer[height + MS_SSIM_PAD2+2][MS_SSIM_PAD2];
  
  for (i = 0; i < width; i++)
  {
    //Top Rows
    bufminus1[i] = bufplus1[i];
    bufminus2[i] = bufplus2[i];
    //Bottom Rows
    bufhplus1[i] = bufhminus1[i];
    bufhplus2[i] = bufhminus2[i];
    bufhplus3[i] = bufhminus3[i];
  }
}

static void imgpel_to_padded_int(imgpel** src, int **buffer, int width, int height)
{
  int i, j;
  int* tmpDst;
  imgpel* tmpSrc;

  tmpDst = buffer[MS_SSIM_PAD2];
  for (j = 0; j < height; j++)
  {
    tmpSrc = src[j];
    tmpDst = &buffer[j + MS_SSIM_PAD2][MS_SSIM_PAD2];
    for (i = 0; i < width; i++)
    {
      tmpDst[i] = (int)tmpSrc[i];
    }
  }
}

void downsample(imgpel** src, imgpel** out, int height, int width)
{
  int height2 = height >> 1;
  int width2  = width  >> 1;
  int i, j;
  int ii, jj;
  int iDst;
  int tmp, tmp1, tmp2;
  int* tmpDst;
  int* tmpSrc;
  
  int** itemp;
  int** dest;

  get_mem2Dint(&itemp, height + MS_SSIM_PAD, width + MS_SSIM_PAD);
  get_mem2Dint(&dest, height + MS_SSIM_PAD, width2 + MS_SSIM_PAD);
  
  imgpel_to_padded_int(src, itemp, width, height);
  horizontal_symmetric_extension(itemp, width, height);

  for (j = 0; j < height; j++)
  {
    tmpDst = dest[j+MS_SSIM_PAD2];
    tmpSrc = itemp[j+MS_SSIM_PAD2];
    iDst   = MS_SSIM_PAD2;
    for (i = 0; i < width2; i++, iDst++)
    {
      ii = (i << 1) + MS_SSIM_PAD2;
      tmp1 = tmpSrc[ii-1] + tmpSrc[ii+2];
      tmp2 = tmpSrc[ii] + tmpSrc[ii+1];
      tmpDst[iDst] = tmpSrc[ii-2] + tmpSrc[ii+3] + (tmp1 << 1) + tmp1 + (tmp2 << 5) - (tmp2 << 2);
      tmpDst[iDst] >>= 6;
    }
  }
 
  //Periodic extension
  vertical_symmetric_extension(dest, width2, height);

  for (i = 0; i < width2; i++) 
  {
    ii = i + MS_SSIM_PAD2;
    for (j = 0; j < height2; j++)
    {
      jj = (j << 1) + MS_SSIM_PAD2;
      tmp1 = dest[jj-1][ii] + dest[jj+2][ii];
      tmp2 = dest[jj][ii]   + dest[jj+1][ii];
      tmp = dest[jj-2][ii] + dest[jj+3][ii] + (tmp1 << 1) + tmp1 + (tmp2 << 5) - (tmp2 << 2);
      out[j][i] = (unsigned char) (tmp >> 6);   //Note: Should change for different bit depths
    }
  }
  free_mem2Dint(itemp);
  free_mem2Dint(dest);
}

float compute_ms_ssim(VideoParameters *p_Vid, InputParameters *p_Inp, imgpel **refImg, imgpel **encImg, int height, int width, int win_height, int win_width, int comp)
{
  float structural[MAX_SSIM_LEVELS];
  float cur_distortion;
  float luminance;
  imgpel** dsRef;
  imgpel** dsEnc;
  int m;
  static const int max_ssim_levels_minus_one = MAX_SSIM_LEVELS - 1;
  static const float exponent[5] = {(float)MS_SSIM_BETA0, (float)MS_SSIM_BETA1, (float)MS_SSIM_BETA2, (float)MS_SSIM_BETA3, (float)MS_SSIM_BETA4};

  dsRef = NULL;
  dsEnc = NULL;
  get_mem2Dpel(&dsRef, height>>1, width>>1);
  get_mem2Dpel(&dsEnc, height>>1, width>>1);

  structural[0] = compute_structural_components(p_Vid, p_Inp, refImg, encImg, height, width, win_height, win_width, comp);
  cur_distortion = (float)pow(structural[0], exponent[0]);

  downsample(refImg, dsRef, height, width);
  downsample(encImg, dsEnc, height, width);

  for (m = 1; m < MAX_SSIM_LEVELS; m++)
  {
    height >>= 1;
    width >>= 1;
    structural[m] = compute_structural_components(p_Vid, p_Inp, dsRef, dsEnc, height, width, imin(win_height,height), imin(win_width,width), comp);
    cur_distortion *= (float)pow(structural[m], exponent[m]);
    if (m < max_ssim_levels_minus_one)
    {
      downsample(dsRef, dsRef, height, width);
      downsample(dsEnc, dsEnc, height, width);
    }
    else
    {
      luminance = compute_luminance_component(p_Vid, p_Inp, dsRef, dsEnc, height, width, imin(win_height,height), imin(win_width,width), comp);
      cur_distortion *= (float)pow(luminance, exponent[m]);
    }
  }
  free_mem2Dpel(dsRef);
  free_mem2Dpel(dsEnc);
  dsRef = NULL;
  dsEnc = NULL;

  return cur_distortion;
}
  
/*!
 ************************************************************************
 * \brief
 *    Find MS-SSIM for all three components
 ************************************************************************
 */
void find_ms_ssim (VideoParameters *p_Vid, InputParameters *p_Inp, ImageStructure *ref, ImageStructure *src, DistMetric *metricSSIM)
{
  DistortionParams *p_Dist = p_Vid->p_Dist;
  FrameFormat *format = &ref->format;

  metricSSIM->value[0] = compute_ms_ssim (p_Vid, p_Inp, ref->data[0], src->data[0], format->height[0], format->width[0], BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, 0);
  // Chroma.
  if (format->yuv_format != YUV400)
  {     
    metricSSIM->value[1]  = compute_ms_ssim (p_Vid, p_Inp, ref->data[1], src->data[1], format->height[1], format->width[1], p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x, 1);
    metricSSIM->value[2]  = compute_ms_ssim (p_Vid, p_Inp, ref->data[2], src->data[2], format->height[1], format->width[1], p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x, 2);
  }

  {
    accumulate_average(metricSSIM,  p_Dist->frame_ctr);
    accumulate_avslice(metricSSIM,  p_Vid->type, p_Vid->p_Stats->frame_ctr[p_Vid->type]);
  }
}
