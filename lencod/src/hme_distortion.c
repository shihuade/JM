/*!
*************************************************************************************
* \file hme_distortion.c
*
* \brief
*    Motion estimation error calculation functions
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*      - Alexis Michael Tourapis <alexismt@gmail.com>
*      - Yuwen He               
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
#include "hme_distortion.h"


/*!
************************************************************************
* \brief
*    SAD computation
************************************************************************
*/
distblk computeSAD_hme(StorablePicture *ref1,
               MEBlock *mv_block,
               distblk min_mcost,
               MotionVector *cand)
{
  int mcost = 0;
  int imin_cost = dist_down(min_mcost);
  int y,x;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;

  int pad_size_x = mv_block->hme_ref_size_x_pad - blocksize_x;

  imgpel *src_line = mv_block->orig_pic[0];
  imgpel *ref_line = UMVLine4X_int_hme (ref1, mv_block, cand->mv_y, cand->mv_x);

  for (y=0; y<blocksize_y; y++)
  {
    for (x = 0; x < blocksize_x; x+=4)
    {
      mcost += iabs( *src_line++ - *ref_line++ );
      mcost += iabs( *src_line++ - *ref_line++ );
      mcost += iabs( *src_line++ - *ref_line++ );
      mcost += iabs( *src_line++ - *ref_line++ );
    }
    if(mcost > imin_cost) 
      return (dist_scale_f((distblk)mcost));
    ref_line += pad_size_x;
  }

  return (dist_scale((distblk)mcost));
}

/*!
************************************************************************
* \brief
*    SAD computation for weighted samples
************************************************************************
*/
distblk computeSADWP_hme(StorablePicture *ref1,
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
  int pad_size_x = mv_block->hme_ref_size_x_pad - blocksize_x;
  int max_imgpel_value = p_Vid->max_imgpel_value;
  short weight = mv_block->weight_luma;
  short offset = mv_block->offset_luma;

  int wp_luma_round = currSlice->wp_luma_round;
  short luma_log_weight_denom = currSlice->luma_log_weight_denom;

  imgpel *src_line = mv_block->orig_pic[0];
  imgpel *ref_line = UMVLine4X_int_hme (ref1, mv_block, cand->mv_y, cand->mv_x);

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

  return (dist_scale((distblk)mcost));
}


