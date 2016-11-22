/*!
*************************************************************************************
* \file me_distortion.c
*
* \brief
*    Motion estimation error calculation functions
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*      - Alexis Michael Tourapis <alexis.tourapis@dolby.com>
*      - Athanasios Leontaris    <aleon@dolby.com>
*
*************************************************************************************
*/

#include "contributors.h"

#include <limits.h>

#include "global.h"
#include "image.h"
#include "memalloc.h"
#include "mb_access.h"
#include "refbuf.h"
#include "mv_search.h"
#include "me_distortion.h"


//#define CHECKOVERFLOW(mcost) assert(mcost>=0)
#define CHECKOVERFLOW(mcost) 

/*!
***********************************************************************
* \brief
*    Calculate SAD
***********************************************************************
*/
distblk distortion4x4SAD(short* diff, distblk min_dist)
{
  int distortion = 0, k;
  for (k = 0; k < 16; k++)
  {
    distortion += iabs(*diff++);
  }
  return (dist_scale((distblk) distortion));
}

/*!
***********************************************************************
* \brief
*    Calculate SSE 4x4
***********************************************************************
*/
distblk distortion4x4SSE(short* diff, distblk min_dist)
{
  int distortion = 0, k;
  for (k = 0; k < 16; k++)
  {
    distortion += iabs2(*diff++);
  }
  return (dist_scale((distblk) distortion));
}

/*!
***********************************************************************
* \brief
*    Calculate SATD
***********************************************************************
*/
distblk distortion4x4SATD(short* diff, distblk min_dist)
{
  distblk i64Ret = HadamardSAD4x4( diff );
  return (dist_scale(i64Ret));
}

/*!
***********************************************************************
* \brief
*    Calculate SAD for 8x8 (with a threshold)
***********************************************************************
*/
distblk distortion8x8SADthres(short* diff, distblk min_cost)
{
  int distortion = 0;
  int i, j;  
  int imin_cost = dist_down(min_cost);

  for (j = 0; j < 8; j++)
  {
    for (i = 0; i < 8; i++)
    {
      distortion += iabs(*diff++);
    }
    if (distortion > imin_cost)
      break;
  }

  return (dist_scale((distblk) distortion));
}

/*!
***********************************************************************
* \brief
*    Calculate SAD for 8x8
***********************************************************************
*/
distblk distortion8x8SAD(short* diff, distblk min_dist)
{
  int distortion = 0;
  int k;

  for (k = 0; k < 64; k++)
  {
    distortion += iabs(*diff++);
  }
  return (dist_scale((distblk) distortion));
}

/*!
***********************************************************************
* \brief
*    Calculate SSE for 8x8
***********************************************************************
*/
distblk distortion8x8SSE(short* diff, distblk min_dist)
{
  distblk distortion = 0;
  int k;
  for (k = 0; k < 64; k++)
  {
    distortion += iabs2(*diff++);
  }
  return (dist_scale(distortion));
}

/*!
***********************************************************************
* \brief
*    Calculate SATD for 8x8
***********************************************************************
*/
distblk distortion8x8SATD(short* diff, distblk min_dist)
{
  distblk i64Ret = HadamardSAD8x8( diff );
  return (dist_scale(i64Ret));
}

void select_distortion(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  switch(p_Inp->ModeDecisionMetric)
  {
  case ERROR_SAD:
    p_Vid->distortion4x4 = distortion4x4SAD;
    p_Vid->distortion8x8 = distortion8x8SAD;
    break;
  case ERROR_SSE:   
    p_Vid->distortion4x4 = distortion4x4SSE;
    p_Vid->distortion8x8 = distortion8x8SSE;
    break;
  case ERROR_SATD :
  default:
    p_Vid->distortion4x4 = distortion4x4SATD;
    p_Vid->distortion8x8 = distortion8x8SATD;
    break;
  }
}


/*!
***********************************************************************
* \brief
*    Calculate 4x4 Hadamard-Transformed SAD
***********************************************************************
*/
int HadamardSAD4x4 (short* diff)
{
  int k, satd = 0;
  int m[16], d[16];

  /*===== hadamard transform =====*/
  m[ 0] = diff[ 0] + diff[12];
  m[ 1] = diff[ 1] + diff[13];
  m[ 2] = diff[ 2] + diff[14];
  m[ 3] = diff[ 3] + diff[15];
  m[ 4] = diff[ 4] + diff[ 8];
  m[ 5] = diff[ 5] + diff[ 9];
  m[ 6] = diff[ 6] + diff[10];
  m[ 7] = diff[ 7] + diff[11];
  m[ 8] = diff[ 4] - diff[ 8];
  m[ 9] = diff[ 5] - diff[ 9];
  m[10] = diff[ 6] - diff[10];
  m[11] = diff[ 7] - diff[11];
  m[12] = diff[ 0] - diff[12];
  m[13] = diff[ 1] - diff[13];
  m[14] = diff[ 2] - diff[14];
  m[15] = diff[ 3] - diff[15];

  d[ 0] = m[ 0] + m[ 4];
  d[ 1] = m[ 1] + m[ 5];
  d[ 2] = m[ 2] + m[ 6];
  d[ 3] = m[ 3] + m[ 7];
  d[ 4] = m[ 8] + m[12];
  d[ 5] = m[ 9] + m[13];
  d[ 6] = m[10] + m[14];
  d[ 7] = m[11] + m[15];
  d[ 8] = m[ 0] - m[ 4];
  d[ 9] = m[ 1] - m[ 5];
  d[10] = m[ 2] - m[ 6];
  d[11] = m[ 3] - m[ 7];
  d[12] = m[12] - m[ 8];
  d[13] = m[13] - m[ 9];
  d[14] = m[14] - m[10];
  d[15] = m[15] - m[11];

  m[ 0] = d[ 0] + d[ 3];
  m[ 1] = d[ 1] + d[ 2];
  m[ 2] = d[ 1] - d[ 2];
  m[ 3] = d[ 0] - d[ 3];
  m[ 4] = d[ 4] + d[ 7];
  m[ 5] = d[ 5] + d[ 6];
  m[ 6] = d[ 5] - d[ 6];
  m[ 7] = d[ 4] - d[ 7];
  m[ 8] = d[ 8] + d[11];
  m[ 9] = d[ 9] + d[10];
  m[10] = d[ 9] - d[10];
  m[11] = d[ 8] - d[11];
  m[12] = d[12] + d[15];
  m[13] = d[13] + d[14];
  m[14] = d[13] - d[14];
  m[15] = d[12] - d[15];

  d[ 0] = m[ 0] + m[ 1];
  d[ 1] = m[ 0] - m[ 1];
  d[ 2] = m[ 2] + m[ 3];
  d[ 3] = m[ 3] - m[ 2];
  d[ 4] = m[ 4] + m[ 5];
  d[ 5] = m[ 4] - m[ 5];
  d[ 6] = m[ 6] + m[ 7];
  d[ 7] = m[ 7] - m[ 6];
  d[ 8] = m[ 8] + m[ 9];
  d[ 9] = m[ 8] - m[ 9];
  d[10] = m[10] + m[11];
  d[11] = m[11] - m[10];
  d[12] = m[12] + m[13];
  d[13] = m[12] - m[13];
  d[14] = m[14] + m[15];
  d[15] = m[15] - m[14];

  //===== sum up =====
  // Table lookup is faster than abs macro
  for (k=0; k<16; ++k)
  {
    satd += iabs(d [k]);
  }


  return ((satd+1)>>1);
}

/*!
***********************************************************************
* \brief
*    Calculate 8x8 Hadamard-Transformed SAD
***********************************************************************
*/
int HadamardSAD8x8 (short* diff)
{
  int i, j, jj, sad=0;

  // Hadamard related arrays
  int m1[8][8], m2[8][8], m3[8][8];


  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }
  for (j=0; j < 8; j++)
    for (i=0; i < 8; i++) 
      sad += iabs (m2[j][i]);

  return ((sad+2)>>2);
}

/*!
************************************************************************
* \brief
*    SAD computation
************************************************************************
*/
distblk computeSAD(StorablePicture *ref1,
               MEBlock *mv_block,
               distblk min_mcost,
               MotionVector *cand)
{
  int mcost = 0;
  int imin_cost = dist_down(min_mcost);
  int y,x;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  VideoParameters *p_Vid = mv_block->p_Vid;
  int pad_size_x = p_Vid->padded_size_x - blocksize_x;
#if (JM_MEM_DISTORTION)
  int *imgpel_abs = p_Vid->imgpel_abs;
#endif

  imgpel *src_line, *ref_line;
  src_line = mv_block->orig_pic[0];
  ref_line = UMVLine4X (ref1, cand->mv_y, cand->mv_x);
  for (y=0; y<blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
#if (JM_MEM_DISTORTION)
      mcost += imgpel_abs[ *src_line++ - *ref_line++ ];
      mcost += imgpel_abs[ *src_line++ - *ref_line++ ];
      mcost += imgpel_abs[ *src_line++ - *ref_line++ ];
      mcost += imgpel_abs[ *src_line++ - *ref_line++ ];
#else
      mcost += iabs( *src_line++ - *ref_line++ );
      mcost += iabs( *src_line++ - *ref_line++ );
      mcost += iabs( *src_line++ - *ref_line++ );
      mcost += iabs( *src_line++ - *ref_line++ );
#endif
    }
    if(mcost > imin_cost) 
      return (dist_scale_f((distblk)mcost));
    ref_line += pad_size_x;
  }
  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int cr_pad_size_x = p_Vid->cr_padded_size_x - blocksize_x_cr;
    int k;
    int mcr_cost = 0; // chroma me cost

    for (k=0; k < 2; k++)
    {
      src_line = mv_block->orig_pic[k+1];
      ref_line = UMVLine8X_chroma ( ref1, k+1, cand->mv_y, cand->mv_x);
      mcr_cost = 0;

      for (y = 0; y < blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x += 2)
        {
#if (JM_MEM_DISTORTION)
          mcr_cost += imgpel_abs[ *src_line++ - *ref_line++ ];
          mcr_cost += imgpel_abs[ *src_line++ - *ref_line++ ];
#else
          mcr_cost += iabs( *src_line++ - *ref_line++ );
          mcr_cost += iabs( *src_line++ - *ref_line++ );
#endif
        }
        ref_line += cr_pad_size_x;
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;

      if(mcost >imin_cost)
        return (dist_scale_f((distblk)mcost));
    }
  }

  CHECKOVERFLOW(mcost);
  return (dist_scale((distblk)mcost));
}

/*!
************************************************************************
* \brief
*    SAD computation for weighted samples
************************************************************************
*/
distblk computeSADWP(StorablePicture *ref1,
                 MEBlock *mv_block,
                 distblk min_mcost,
                 MotionVector *cand
                 )
{
  int mcost = 0;
  int imin_cost = dist_down(min_mcost);
  int y, x;
  int weighted_pel;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;

  VideoParameters *p_Vid = mv_block->p_Vid;
  Slice *currSlice = mv_block->p_Slice;  
  int pad_size_x = p_Vid->padded_size_x - blocksize_x;
  int max_imgpel_value = p_Vid->max_imgpel_value;
  short weight = mv_block->weight_luma;
  short offset = mv_block->offset_luma;

  int wp_luma_round = currSlice->wp_luma_round;
  short luma_log_weight_denom = currSlice->luma_log_weight_denom;

  imgpel *src_line = mv_block->orig_pic[0];
  imgpel *ref_line = UMVLine4X (ref1, cand->mv_y, cand->mv_x);

  for (y=0; y<blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
      weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
      mcost += iabs( *src_line++ -  weighted_pel );
      weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
      mcost += iabs( *src_line++ -  weighted_pel );
      weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
      mcost += iabs( *src_line++ -  weighted_pel );
      weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
      mcost += iabs( *src_line++ -  weighted_pel );
    }
    if(mcost > imin_cost)
      return (dist_scale_f((distblk)mcost));
    ref_line += pad_size_x;
  }
  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int cr_pad_size_x = p_Vid->cr_padded_size_x - blocksize_x_cr;
    int k;
    int mcr_cost = 0;
    int max_imgpel_value_uv = p_Vid->max_pel_value_comp[1];
    int wp_chroma_round = currSlice->wp_chroma_round;
    short chroma_log_weight_denom = currSlice->chroma_log_weight_denom;

    for (k=0; k < 2; k++)
    {
      weight = mv_block->weight_cr[k];
      offset = mv_block->offset_cr[k];

      mcr_cost = 0;
      src_line = mv_block->orig_pic[k+1];
      ref_line = UMVLine8X_chroma ( ref1, k+1, cand->mv_y, cand->mv_x);
      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          weighted_pel = iClip1( max_imgpel_value_uv, ((weight * *ref_line++  + wp_chroma_round) >> chroma_log_weight_denom) + offset);
          mcr_cost += iabs( *src_line++ -  weighted_pel );
          weighted_pel = iClip1( max_imgpel_value_uv, ((weight * *ref_line++  + wp_chroma_round) >> chroma_log_weight_denom) + offset);
          mcr_cost += iabs( *src_line++ -  weighted_pel );
        }
        ref_line += cr_pad_size_x;
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;

      if(mcost >imin_cost)
        return (dist_scale_f((distblk)mcost));
    }
  }

  CHECKOVERFLOW(mcost);
  return (dist_scale((distblk)mcost));
}

/*!
************************************************************************
* \brief
*    BiPred SAD computation (no weights)
************************************************************************
*/
distblk computeBiPredSAD1(StorablePicture *ref1, 
                      StorablePicture *ref2, 
                      MEBlock *mv_block,
                      distblk min_mcost,
                      MotionVector *cand1,
                      MotionVector *cand2)
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int bi_diff;
  int y,x;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  VideoParameters *p_Vid = mv_block->p_Vid;
  int pad_size_x = p_Vid->padded_size_x - blocksize_x;
#if (JM_MEM_DISTORTION)
  int *imgpel_abs = p_Vid->imgpel_abs;
#endif

  imgpel *src_line   = mv_block->orig_pic[0];
  imgpel *ref2_line  = UMVLine4X(ref2, cand2->mv_y, cand2->mv_x);
  imgpel *ref1_line  = UMVLine4X(ref1, cand1->mv_y, cand1->mv_x);

  for (y = 0; y < blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
#if (JM_MEM_DISTORTION)
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += imgpel_abs[ bi_diff ];
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += imgpel_abs[ bi_diff ];
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += imgpel_abs[ bi_diff ];
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += imgpel_abs[ bi_diff ];
#else
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += iabs(bi_diff);
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += iabs(bi_diff);
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += iabs(bi_diff);
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += iabs(bi_diff);
#endif
    }
    if(mcost > imin_cost)
      return (dist_scale_f((distblk)mcost));

    ref2_line += pad_size_x;
    ref1_line += pad_size_x;
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int cr_pad_size_x = p_Vid->cr_padded_size_x - blocksize_x_cr;
    int k;
    int mcr_cost = 0;

    for (k=1; k<3; k++)
    {
      mcr_cost = 0;
      src_line = mv_block->orig_pic[k];
      ref2_line = UMVLine8X_chroma ( ref2, k, cand2->mv_y, cand2->mv_x);
      ref1_line = UMVLine8X_chroma ( ref1, k, cand1->mv_y, cand1->mv_x);

      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          mcr_cost += iabs(bi_diff);
          bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          mcr_cost += iabs(bi_diff);
        }        
        ref2_line += cr_pad_size_x;
        ref1_line += cr_pad_size_x;
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;

      if(mcost > imin_cost)
        return (dist_scale_f((distblk)mcost));
    }
  }

  CHECKOVERFLOW(mcost);
  return (dist_scale((distblk)mcost));
}

/*!
************************************************************************
* \brief
*    BiPred SAD computation (with weights)
************************************************************************
*/
distblk computeBiPredSAD2(StorablePicture *ref1, 
                      StorablePicture *ref2, 
                      MEBlock *mv_block,
                      distblk min_mcost,
                      MotionVector *cand1,
                      MotionVector *cand2)
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int bi_diff;
  VideoParameters *p_Vid = mv_block->p_Vid;
  Slice *currSlice = mv_block->p_Slice;
  int denom = currSlice->luma_log_weight_denom + 1;
  int lround = 2 * currSlice->wp_luma_round;  
  int max_imgpel_value = p_Vid->max_imgpel_value;
  int y,x;
  int weighted_pel, pixel1, pixel2;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  short weight1 = mv_block->weight1;
  short weight2 = mv_block->weight2;
  short offsetBi = mv_block->offsetBi;

  int pad_size_x = p_Vid->padded_size_x - blocksize_x;

  imgpel *src_line   = mv_block->orig_pic[0];
  imgpel *ref2_line  = UMVLine4X(ref2, cand2->mv_y, cand2->mv_x);
  imgpel *ref1_line  = UMVLine4X(ref1, cand1->mv_y, cand1->mv_x);

  for (y=0; y<blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
      pixel1 = weight1 * (*ref1_line++);
      pixel2 = weight2 * (*ref2_line++);
      weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
      bi_diff = (*src_line++) - weighted_pel;
      mcost += iabs(bi_diff);

      pixel1 = weight1 * (*ref1_line++);
      pixel2 = weight2 * (*ref2_line++);
      weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
      bi_diff = (*src_line++) - weighted_pel;
      mcost += iabs(bi_diff);

      pixel1 = weight1 * (*ref1_line++);
      pixel2 = weight2 * (*ref2_line++);
      weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
      bi_diff = (*src_line++) - weighted_pel;
      mcost += iabs(bi_diff);

      pixel1 = weight1 * (*ref1_line++);
      pixel2 = weight2 * (*ref2_line++);
      weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
      bi_diff = (*src_line++) - weighted_pel;
      mcost += iabs(bi_diff);
    }

    if(mcost > imin_cost)
      return dist_scale_f((distblk)mcost);
    ref2_line += pad_size_x;
    ref1_line += pad_size_x;
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int cr_pad_size_x  = p_Vid->cr_padded_size_x - blocksize_x_cr;
    int k;
    int mcr_cost = 0;
    int max_imgpel_value_uv = p_Vid->max_pel_value_comp[1];

    for (k=0; k<2; k++)
    {
      weight1  = mv_block->weight1_cr[k];
      weight2  = mv_block->weight2_cr[k];
      offsetBi = mv_block->offsetBi_cr[k];

      mcr_cost = 0;
      src_line = mv_block->orig_pic[k+1];
      ref2_line = UMVLine8X_chroma ( ref2, k+1, cand2->mv_y, cand2->mv_x);
      ref1_line = UMVLine8X_chroma ( ref1, k+1, cand1->mv_y, cand1->mv_x);

      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel = iClip1( max_imgpel_value_uv, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          bi_diff = (*src_line++) - weighted_pel;
          mcr_cost += iabs(bi_diff);

          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel = iClip1( max_imgpel_value_uv, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          bi_diff = (*src_line++) - weighted_pel;
          mcr_cost += iabs(bi_diff);
        }
        ref2_line += cr_pad_size_x;
        ref1_line += cr_pad_size_x;
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;
      
      if(mcost > imin_cost) 
        return dist_scale_f((distblk)mcost);
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}

/*!
************************************************************************
* \brief
*    SAD computation _with_ Hadamard Transform
************************************************************************
*/
distblk computeSATD(StorablePicture *ref1,
                MEBlock *mv_block,
                distblk min_mcost,
                MotionVector *cand
                )
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int y, x, y4;
  int src_size_x, src_size_mul;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  VideoParameters *p_Vid = mv_block->p_Vid;
  imgpel *src_tmp = mv_block->orig_pic[0];
  short *d, diff[MB_PIXELS];
  imgpel *src_line, *ref_line;
  
  if ( !mv_block->test8x8 )
  { // 4x4 TRANSFORM
    src_size_x = blocksize_x - BLOCK_SIZE;
    src_size_mul = blocksize_x * BLOCK_SIZE;
    for (y = cand->mv_y; y < cand->mv_y + (blocksize_y<<2); y += (BLOCK_SIZE_SP))
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE)
      {
        d    = diff;
        ref_line = UMVLine4X (ref1, y, cand->mv_x + (x<<2));
        src_line = src_tmp + x;
        for (y4 = 0; y4 < BLOCK_SIZE; y4++ )
        {
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;

          ref_line += p_Vid->padded_size_x_m4x4;
          src_line += src_size_x;
        }
        mcost += HadamardSAD4x4 (diff);
        if(mcost > imin_cost)
          return dist_scale_f((distblk)mcost);
      }
      src_tmp += src_size_mul;
    }
  }
  else
  { // 8x8 TRANSFORM    
    src_size_x = (blocksize_x - BLOCK_SIZE_8x8);
    src_size_mul = blocksize_x * BLOCK_SIZE_8x8;
    for (y = cand->mv_y; y < cand->mv_y + (blocksize_y<<2); y += (BLOCK_SIZE_8x8_SP) )
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE_8x8 )
      {
        d = diff;
        ref_line = UMVLine4X (ref1, y, cand->mv_x + (x<<2));
        src_line = src_tmp + x;
        for (y4 = 0; y4 < BLOCK_SIZE_8x8; y4++ )
        {
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;

          ref_line += p_Vid->padded_size_x_m8x8;
          src_line += src_size_x;
        }
        mcost += HadamardSAD8x8 (diff);
        if(mcost > imin_cost)
          return dist_scale_f((distblk)mcost);
      }
      src_tmp += src_size_mul;
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}

/*!
************************************************************************
* \brief
*    SAD computation of weighted samples _with_ Hadamard Transform
************************************************************************
*/
distblk computeSATDWP(StorablePicture *ref1,
                  MEBlock *mv_block,
                  distblk min_mcost,
                  MotionVector *cand
                  )
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int y, x, y4;
  int weighted_pel;
  int src_size_x, src_size_mul;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;

  imgpel *src_tmp = mv_block->orig_pic[0];
  VideoParameters *p_Vid = mv_block->p_Vid;
  Slice *currSlice = mv_block->p_Slice;
  short luma_log_weight_denom = currSlice->luma_log_weight_denom;
  short weight = mv_block->weight_luma;
  short offset = mv_block->offset_luma; 

  int wp_luma_round = currSlice->wp_luma_round;  
  int max_imgpel_value = p_Vid->max_imgpel_value;
  short *d, diff[MB_PIXELS];
  imgpel *src_line, *ref_line;

  if ( !mv_block->test8x8 )
  { // 4x4 TRANSFORM
    src_size_x = (blocksize_x - BLOCK_SIZE);
    src_size_mul = blocksize_x * BLOCK_SIZE;
    for (y = cand->mv_y; y < cand->mv_y + (blocksize_y<<2); y += (BLOCK_SIZE_SP))
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE)
      {
        d    = diff;
        ref_line = UMVLine4X (ref1, y, cand->mv_x + (x<<2));
        src_line = src_tmp + x;
        for (y4 = 0; y4 < BLOCK_SIZE; y4++ )
        {
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);

          ref_line += p_Vid->padded_size_x_m4x4;
          src_line += src_size_x;
        }
        mcost += HadamardSAD4x4 (diff);
        
        if(mcost > imin_cost) 
          return dist_scale_f((distblk)mcost);
      }
      src_tmp += src_size_mul;
    }
  }
  else
  { // 8x8 TRANSFORM
    src_size_x = (blocksize_x - BLOCK_SIZE_8x8);
    src_size_mul = blocksize_x * BLOCK_SIZE_8x8;
    for (y = cand->mv_y; y < cand->mv_y + (blocksize_y<<2); y += (BLOCK_SIZE_8x8_SP) )
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE_8x8 )
      {
        d = diff;
        ref_line = UMVLine4X (ref1, y, cand->mv_x + (x<<2));
        src_line = src_tmp + x;
        for (y4 = 0; y4 < BLOCK_SIZE_8x8; y4++ )
        {
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);
          weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
          *d++ = (short) (*src_line++ - weighted_pel);

          ref_line += p_Vid->padded_size_x_m8x8;
          src_line += src_size_x;
        }
        mcost += HadamardSAD8x8 (diff);
        if(mcost > imin_cost) 
          return dist_scale_f((distblk)mcost);
      }
      src_tmp += src_size_mul;
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}

/*!
************************************************************************
* \brief
*    BiPred (w/o weights) SATD computation
************************************************************************
*/
distblk computeBiPredSATD1(StorablePicture *ref1, 
                       StorablePicture *ref2, 
                       MEBlock *mv_block,
                       distblk min_mcost,
                       MotionVector *cand1,
                       MotionVector *cand2)
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int y, x, y4;
  int src_size_x, src_size_mul;
  imgpel *src_tmp = mv_block->orig_pic[0];
  short *d, diff[MB_PIXELS];
  imgpel *src_line, *ref1_line, *ref2_line;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  VideoParameters *p_Vid = mv_block->p_Vid;

  if ( !mv_block->test8x8 )
  { // 4x4 TRANSFORM
    src_size_x = (blocksize_x - BLOCK_SIZE);
    src_size_mul = blocksize_x * BLOCK_SIZE;
    for (y=0; y<(blocksize_y<<2); y += (BLOCK_SIZE_SP))
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE)
      {
        d    = diff;
        src_line   = src_tmp + x;
        ref2_line  = UMVLine4X(ref2, cand2->mv_y + y, cand2->mv_x + (x<<2));
        ref1_line  = UMVLine4X(ref1, cand1->mv_y + y, cand1->mv_x + (x<<2));
        for (y4 = 0; y4 < BLOCK_SIZE; y4++ )
        {
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);

          ref1_line += p_Vid->padded_size_x_m4x4;
          ref2_line += p_Vid->padded_size_x_m4x4;
          src_line  += src_size_x;
        }
        mcost += HadamardSAD4x4 (diff);
        if(mcost > imin_cost) 
          return dist_scale_f((distblk)mcost);
      }
      src_tmp += src_size_mul;
    }
  }
  else
  { // 8x8 TRANSFORM
    src_size_x = (blocksize_x - BLOCK_SIZE_8x8);
    src_size_mul = blocksize_x * BLOCK_SIZE_8x8;
    for (y=0; y<(blocksize_y << 2); y += BLOCK_SIZE_8x8_SP )
    {
      int y_pos2 = cand2->mv_y + y;
      int y_pos1 = cand1->mv_y + y;
      for (x=0; x<blocksize_x; x += BLOCK_SIZE_8x8 )
      {
        d = diff;
        src_line   = src_tmp + x;
        ref2_line  = UMVLine4X(ref2, y_pos2, cand2->mv_x + (x<<2));
        ref1_line  = UMVLine4X(ref1, y_pos1, cand1->mv_x + (x<<2));
        for (y4 = 0; y4 < BLOCK_SIZE_8x8; y4++ )
        {
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);

          ref1_line += p_Vid->padded_size_x_m8x8;
          ref2_line += p_Vid->padded_size_x_m8x8;
          src_line += src_size_x;
        }
        mcost += HadamardSAD8x8 (diff);
        if(mcost > imin_cost)
          return dist_scale_f((distblk)mcost);
      }
      src_tmp += src_size_mul;
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}

/*!
************************************************************************
* \brief
*    BiPred (w/ weights) SATD computation
************************************************************************
*/
distblk computeBiPredSATD2(StorablePicture *ref1, 
                       StorablePicture *ref2, 
                       MEBlock *mv_block,
                       distblk min_mcost,
                       MotionVector *cand1,
                       MotionVector *cand2)
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int y, x, y4;
  int weighted_pel, pixel1, pixel2;
  VideoParameters *p_Vid = mv_block->p_Vid;
  Slice *currSlice = mv_block->p_Slice;
  int denom = currSlice->luma_log_weight_denom + 1;
  int lround = 2 * currSlice->wp_luma_round;
  short weight1 = mv_block->weight1;
  short weight2 = mv_block->weight2;
  short offsetBi = mv_block->offsetBi;


  int max_imgpel_value = p_Vid->max_imgpel_value;
  int src_size_x, src_size_mul;
  imgpel *src_tmp = mv_block->orig_pic[0];
  short *d, diff[MB_PIXELS];
  imgpel *src_line, *ref1_line, *ref2_line;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;

  if ( !mv_block->test8x8 )
  { // 4x4 TRANSFORM
    src_size_x = (blocksize_x - BLOCK_SIZE);
    src_size_mul = blocksize_x * BLOCK_SIZE;
    for (y=0; y<(blocksize_y<<2); y += BLOCK_SIZE_SP)
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE)
      {
        d    = diff;
        src_line   = src_tmp + x;
        ref2_line  = UMVLine4X(ref2, cand2->mv_y + y, cand2->mv_x + (x<<2));
        ref1_line  = UMVLine4X(ref1, cand1->mv_y + y, cand1->mv_x + (x<<2));
        for (y4 = 0; y4 < BLOCK_SIZE; y4++ )
        {
          // 0
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 1
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 2
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 3
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);

          ref1_line += p_Vid->padded_size_x_m4x4;
          ref2_line += p_Vid->padded_size_x_m4x4;
          src_line  += src_size_x;
        }
        mcost += HadamardSAD4x4 (diff);
        if(mcost > imin_cost)
          return dist_scale_f((distblk)mcost);
      }
      src_tmp += src_size_mul;
    }
  }
  else
  { // 8x8 TRANSFORM
    src_size_x = (blocksize_x - BLOCK_SIZE_8x8);
    src_size_mul = blocksize_x * BLOCK_SIZE_8x8;
    for (y=0; y < (blocksize_y << 2); y += BLOCK_SIZE_8x8_SP )
    {
      int y_pos2 = cand2->mv_y + y;
      int y_pos1 = cand1->mv_y + y;
      for (x=0; x<blocksize_x; x += BLOCK_SIZE_8x8 )
      {
        d = diff;
        src_line   = src_tmp + x;
        ref2_line  = UMVLine4X(ref2, y_pos2, cand2->mv_x + (x<<2));
        ref1_line  = UMVLine4X(ref1, y_pos1, cand1->mv_x + (x<<2));
        for (y4 = 0; y4 < BLOCK_SIZE_8x8; y4++ )
        {
          // 0
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 1
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 2
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 3
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 4
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 5
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 6
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line++) - weighted_pel);
          // 7
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          *d++ =  (short) ((*src_line) - weighted_pel);

          ref1_line += p_Vid->padded_size_x_m8x8;
          ref2_line += p_Vid->padded_size_x_m8x8;
          src_line  += src_size_x;
        }
        mcost += HadamardSAD8x8 (diff);
        if(mcost > imin_cost)
          return dist_scale_f((distblk)mcost);
      }
      src_tmp += src_size_mul;
    }
  }
  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}

/*!
************************************************************************
* \brief
*    SSE computation
************************************************************************
*/
distblk computeSSE(StorablePicture *ref1,
               MEBlock *mv_block,
               distblk min_mcost,
               MotionVector *cand
               )
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int y,x;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  VideoParameters *p_Vid = mv_block->p_Vid;
  int pad_size_x = p_Vid->padded_size_x - blocksize_x;

  imgpel *src_line = mv_block->orig_pic[0];
  imgpel *ref_line = UMVLine4X (ref1, cand->mv_y, cand->mv_x);
  
  for (y=0; y<blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
      mcost += iabs2( *src_line++ - *ref_line++ );
      mcost += iabs2( *src_line++ - *ref_line++ );
      mcost += iabs2( *src_line++ - *ref_line++ );
      mcost += iabs2( *src_line++ - *ref_line++ );
    }
    if(mcost > imin_cost)
      return dist_scale_f((distblk)mcost);
    ref_line += pad_size_x;
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int cr_pad_size_x = p_Vid->cr_padded_size_x - blocksize_x_cr;
    int k;
    int mcr_cost = 0;

    for (k=0; k<2; k++)
    {
      mcr_cost = 0;
      src_line = mv_block->orig_pic[k+1];
      ref_line = UMVLine8X_chroma ( ref1, k+1, cand->mv_y, cand->mv_x);
      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          mcr_cost += iabs2( *src_line++ - *ref_line++ );
          mcr_cost += iabs2( *src_line++ - *ref_line++ );
        }
        ref_line += cr_pad_size_x;
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;
      if(mcost > imin_cost)
        return dist_scale_f((distblk)mcost);
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}


/*!
************************************************************************
* \brief
*    SSE computation of weighted samples
************************************************************************
*/
distblk computeSSEWP(StorablePicture *ref1,
                 MEBlock *mv_block,
                 distblk min_mcost,
                 MotionVector *cand
                 )
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int y,x;
  int weighted_pel;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  VideoParameters *p_Vid = mv_block->p_Vid;
  Slice *currSlice = mv_block->p_Slice;  
  short weight = mv_block->weight_luma;
  short offset = mv_block->offset_luma; 

  int wp_luma_round = currSlice->wp_luma_round;  
  int pad_size_x = p_Vid->padded_size_x - blocksize_x;
  int max_imgpel_value = p_Vid->max_imgpel_value;
  short luma_log_weight_denom = currSlice->luma_log_weight_denom;

  imgpel *src_line = mv_block->orig_pic[0];
  imgpel *ref_line = UMVLine4X (ref1, cand->mv_y, cand->mv_x);

  for (y=0; y<blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
      weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
      mcost += iabs2( *src_line++ - weighted_pel );
      weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
      mcost += iabs2( *src_line++ - weighted_pel );
      weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
      mcost += iabs2( *src_line++ - weighted_pel );
      weighted_pel = iClip1( max_imgpel_value, ((weight * *ref_line++  + wp_luma_round) >> luma_log_weight_denom) + offset);
      mcost += iabs2( *src_line++ - weighted_pel );
    }
    if(mcost > imin_cost)
      return dist_scale_f((distblk)mcost);
    ref_line += pad_size_x;
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    // These could be made global to reduce computations
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int cr_pad_size_x  = p_Vid->cr_padded_size_x - blocksize_x_cr;
    int k;
    int mcr_cost = 0;
    int max_imgpel_value_uv = p_Vid->max_pel_value_comp[1];
    int wp_chroma_round = currSlice->wp_chroma_round;
    short chroma_log_weight_denom = currSlice->chroma_log_weight_denom;

    for (k=0; k<2; k++)
    {
      weight = mv_block->weight_cr[k];
      offset = mv_block->offset_cr[k]; 

      mcr_cost = 0;
      src_line = mv_block->orig_pic[k+1];
      ref_line = UMVLine8X_chroma ( ref1, k+1, cand->mv_y, cand->mv_x);
      for (y=0; y<blocksize_y_cr; y++)
      {

        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          weighted_pel = iClip1( max_imgpel_value_uv, ((weight * *ref_line++  + wp_chroma_round) >> chroma_log_weight_denom) + offset);
          mcr_cost += iabs2( *src_line++ - weighted_pel );
          weighted_pel = iClip1( max_imgpel_value_uv, ((weight * *ref_line++  + wp_chroma_round) >> chroma_log_weight_denom) + offset);
          mcr_cost += iabs2( *src_line++ - weighted_pel );
        }        
        ref_line += cr_pad_size_x;
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;
      if(mcost > imin_cost)
        return dist_scale_f((distblk)mcost);
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}

/*!
************************************************************************
* \brief
*    BiPred SSE computation (no weights)
************************************************************************
*/
distblk computeBiPredSSE1(StorablePicture *ref1, 
                      StorablePicture *ref2, 
                      MEBlock *mv_block,
                      distblk min_mcost,
                      MotionVector *cand1,
                      MotionVector *cand2)
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int bi_diff;
  int y,x;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  VideoParameters *p_Vid = mv_block->p_Vid;
  int pad_size_x = p_Vid->padded_size_x - blocksize_x;

  imgpel *src_line   = mv_block->orig_pic[0];
  imgpel *ref2_line  = UMVLine4X(ref2, cand2->mv_y, cand2->mv_x);
  imgpel *ref1_line  = UMVLine4X(ref1, cand1->mv_y, cand1->mv_x);

  for (y = 0; y < blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += iabs2(bi_diff);
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += iabs2(bi_diff);
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += iabs2(bi_diff);
      bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
      mcost += iabs2(bi_diff);
    }
    if(mcost > imin_cost)
      return dist_scale_f((distblk)mcost);

    ref2_line += pad_size_x;
    ref1_line += pad_size_x;
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int cr_pad_size_x  = p_Vid->cr_padded_size_x - blocksize_x_cr;
    int k;
    int mcr_cost = 0;

    for (k=0; k<2; k++)
    {
      mcr_cost = 0;
      src_line = mv_block->orig_pic[k+1];
      ref2_line = UMVLine8X_chroma ( ref2, k+1, cand2->mv_y, cand2->mv_x);
      ref1_line = UMVLine8X_chroma ( ref1, k+1, cand1->mv_y, cand1->mv_x);

      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          mcr_cost += iabs2(bi_diff);
          bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          mcr_cost += iabs2(bi_diff);
        }        
        ref2_line += cr_pad_size_x;
        ref1_line += cr_pad_size_x;
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;
      if(mcost > imin_cost)
        return dist_scale_f((distblk)mcost);
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}


/*!
************************************************************************
* \brief
*    BiPred SSE computation (with weights)
************************************************************************
*/
distblk computeBiPredSSE2(StorablePicture *ref1, 
                      StorablePicture *ref2, 
                      MEBlock *mv_block,
                      distblk min_mcost,
                      MotionVector *cand1,
                      MotionVector *cand2)
{
  int imin_cost = dist_down(min_mcost);
  int mcost = 0;
  int bi_diff;
  VideoParameters *p_Vid = mv_block->p_Vid;
  Slice *currSlice = mv_block->p_Slice;  
  int denom = currSlice->luma_log_weight_denom + 1;
  int lround = 2 * currSlice->wp_luma_round;
  int max_imgpel_value = p_Vid->max_imgpel_value;
  int y,x;
  int weighted_pel, pixel1, pixel2;
  short weight1 = mv_block->weight1;
  short weight2 = mv_block->weight2;
  short offsetBi = mv_block->offsetBi;

  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  int pad_size_x = p_Vid->padded_size_x - blocksize_x;

  imgpel *src_line   = mv_block->orig_pic[0];
  imgpel *ref2_line  = UMVLine4X(ref2, cand2->mv_y, cand2->mv_x);
  imgpel *ref1_line  = UMVLine4X(ref1, cand1->mv_y, cand1->mv_x);
  for (y=0; y<blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
      pixel1 = weight1 * (*ref1_line++);
      pixel2 = weight2 * (*ref2_line++);
      weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
      bi_diff = (*src_line++) - weighted_pel;
      mcost += bi_diff * bi_diff;

      pixel1 = weight1 * (*ref1_line++);
      pixel2 = weight2 * (*ref2_line++);
      weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
      bi_diff = (*src_line++) - weighted_pel;
      mcost += bi_diff * bi_diff;

      pixel1 = weight1 * (*ref1_line++);
      pixel2 = weight2 * (*ref2_line++);
      weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
      bi_diff = (*src_line++) - weighted_pel;
      mcost += bi_diff * bi_diff;

      pixel1 = weight1 * (*ref1_line++);
      pixel2 = weight2 * (*ref2_line++);
      weighted_pel =  iClip1( max_imgpel_value, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
      bi_diff = (*src_line++) - weighted_pel;
      mcost += bi_diff * bi_diff;
    }
    if(mcost > imin_cost)
      return dist_scale_f((distblk)mcost);

    ref2_line += pad_size_x;
    ref1_line += pad_size_x;
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int cr_pad_size_x  = p_Vid->cr_padded_size_x - blocksize_x_cr;
    int k;
    int mcr_cost = 0;
    int max_imgpel_value_uv = p_Vid->max_pel_value_comp[1];

    for (k=0; k<2; k++)
    {
      weight1  = mv_block->weight1_cr[k];
      weight2  = mv_block->weight2_cr[k];
      offsetBi = mv_block->offsetBi_cr[k];

      mcr_cost = 0;
      src_line = mv_block->orig_pic[k+1];
      ref2_line = UMVLine8X_chroma ( ref2, k+1, cand2->mv_y, cand2->mv_x);
      ref1_line = UMVLine8X_chroma ( ref1, k+1, cand1->mv_y, cand1->mv_x);

      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel = iClip1( max_imgpel_value_uv, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          bi_diff = (*src_line++) - weighted_pel;
          mcr_cost += bi_diff * bi_diff;

          pixel1 = weight1 * (*ref1_line++);
          pixel2 = weight2 * (*ref2_line++);
          weighted_pel = iClip1( max_imgpel_value_uv, ((pixel1 + pixel2 + lround) >> denom) + offsetBi);
          bi_diff = (*src_line++) - weighted_pel;
          mcr_cost += bi_diff * bi_diff;
        }        
        ref2_line += cr_pad_size_x;
        ref1_line += cr_pad_size_x;
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;
      if(mcost > imin_cost)
        return dist_scale_f((distblk)mcost);
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}


void calcDifference(imgpel **origImg, int ox, int oy, imgpel **predImg, int px, int py, int width, int height, short *diff)
{
  int i, j;
  imgpel *orig, *pred;
  for (j = 0; j < height; j++)
  {
    orig = origImg[oy+j]+ox;
    pred = predImg[py+j]+px;
    for (i = 0; i < width; i++)
    {
      *diff++ = *orig++ - *pred++;
    }
  }
}
