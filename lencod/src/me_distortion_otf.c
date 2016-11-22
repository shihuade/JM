/*!
*************************************************************************************
* \file me_distortion_otf.c
*
* \brief
*    Motion estimation error calculation functions with on-the-fly interpolation
*
*
*************************************************************************************
*/

#include "contributors.h"

#include <limits.h>

#include "global.h"
#include "image.h"
#include "memalloc.h"
#include "mb_access.h"
#include "mv_search.h"
#include "me_distortion.h"
#include "me_distortion_otf.h"
#include "get_block_otf.h"

#include "refbuf_otf.h"


//#define CHECKOVERFLOW(mcost) assert(mcost>=0)
#define CHECKOVERFLOW(mcost) 


/*!
************************************************************************
* \brief
*  JLT :  SAD computation  (on-the-fly)
************************************************************************
*/
distblk computeSAD_otf(StorablePicture *ref1,
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
#if (JM_MEM_DISTORTION)
  int *imgpel_abs = p_Vid->imgpel_abs;
#endif
  imgpel  *src_line, *ref_line ; 
  imgpel  data[MB_PIXELS];                                  // local allocation could be optimized by a global instanciation
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;
  
  src_line = mv_block->orig_pic[0];
  ref_line = data ;
  // get block with interpolation on-the-fly
  p_Dpb->pf_get_block_luma( p_Vid, ref_line, tmp_line, cand->mv_x, cand->mv_y, blocksize_x, blocksize_y, ref1, 0 )  ;

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
  }
  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int k;
    int mcr_cost = 0; // chroma me cost

    for (k=0; k < 2; k++)
    {
      src_line = mv_block->orig_pic[k+1];
      ref_line = data ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref_line, tmp_line, cand->mv_x, cand->mv_y, blocksize_x_cr, blocksize_y_cr, ref1, k+1 ) ;
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
*  JLT :  SADWP computation for weighted samples ( on-the-fly )
************************************************************************
*/
distblk computeSADWP_otf(StorablePicture *ref1,
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
  int max_imgpel_value = p_Vid->max_imgpel_value;
  short weight = mv_block->weight_luma;
  short offset = mv_block->offset_luma;

  int wp_luma_round = currSlice->wp_luma_round;
  short luma_log_weight_denom = currSlice->luma_log_weight_denom;

  imgpel  *src_line, *ref_line ; 
  imgpel  data[MB_PIXELS];                                  // local allocation could be optimized by a global instanciation
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;

  src_line = mv_block->orig_pic[0];
  ref_line = data ;
  // get block with interpolation on-the-fly
  p_Dpb->pf_get_block_luma( p_Vid, ref_line, tmp_line, cand->mv_x, cand->mv_y, blocksize_x, blocksize_y, ref1, 0 );

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
  }
  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
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
      ref_line = data ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref_line, tmp_line, cand->mv_x, cand->mv_y, blocksize_x_cr, blocksize_y_cr, ref1, k+1 ) ;

      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          weighted_pel = iClip1( max_imgpel_value_uv, ((weight * *ref_line++  + wp_chroma_round) >> chroma_log_weight_denom) + offset);
          mcr_cost += iabs( *src_line++ -  weighted_pel );
          weighted_pel = iClip1( max_imgpel_value_uv, ((weight * *ref_line++  + wp_chroma_round) >> chroma_log_weight_denom) + offset);
          mcr_cost += iabs( *src_line++ -  weighted_pel );
        }
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
*  JLT :  BiPred SAD computation (no weights) ( on-the-fly )
************************************************************************
*/
distblk computeBiPredSAD1_otf(StorablePicture *ref1, 
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
#if (JM_MEM_DISTORTION)
  int *imgpel_abs = p_Vid->imgpel_abs;
#endif

  imgpel  *src_line, *ref2_line, *ref1_line ; 
  imgpel  data2[MB_PIXELS], data1[MB_PIXELS];                                  // local allocation could be optimized by a global instanciation
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;

  src_line = mv_block->orig_pic[0];
  ref2_line = data2 ;
  ref1_line = data1 ;

  p_Dpb->pf_get_block_luma( p_Vid, ref2_line, tmp_line, cand2->mv_x, cand2->mv_y, blocksize_x, blocksize_y, ref2, 0 );
  p_Dpb->pf_get_block_luma( p_Vid, ref1_line, tmp_line, cand1->mv_x, cand1->mv_y, blocksize_x, blocksize_y, ref1, 0 );

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
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int k;
    int mcr_cost = 0;

    for (k=1; k<3; k++)
    {
      mcr_cost = 0;
      
      src_line = mv_block->orig_pic[k];
      ref2_line = data2 ;
      ref1_line = data1 ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref2_line, tmp_line, cand2->mv_x, cand2->mv_y, blocksize_x_cr, blocksize_y_cr, ref2, k ) ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref1_line, tmp_line, cand1->mv_x, cand1->mv_y, blocksize_x_cr, blocksize_y_cr, ref1, k ) ;

      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          mcr_cost += iabs(bi_diff);
          bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          mcr_cost += iabs(bi_diff);
        }
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
*  JLT :  BiPred SAD computation (with weights) ( on-the-fly )
************************************************************************
*/
distblk computeBiPredSAD2_otf(StorablePicture *ref1, 
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
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

  imgpel  *src_line, *ref2_line, *ref1_line ; 
  imgpel  data2[MB_PIXELS], data1[MB_PIXELS];                                  // local allocation could be optimized by a global instanciation
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;
  src_line = mv_block->orig_pic[0];
  ref2_line = data2 ;
  ref1_line = data1 ;
  
  p_Dpb->pf_get_block_luma( p_Vid, ref2_line, tmp_line, cand2->mv_x, cand2->mv_y, blocksize_x, blocksize_y, ref2, 0 );
  p_Dpb->pf_get_block_luma( p_Vid, ref1_line, tmp_line, cand1->mv_x, cand1->mv_y, blocksize_x, blocksize_y, ref1, 0 );

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
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
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
      ref2_line = data2 ;
      ref1_line = data1 ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref2_line, tmp_line, cand2->mv_x, cand2->mv_y, blocksize_x_cr, blocksize_y_cr, ref2, k+1 ) ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref1_line, tmp_line, cand1->mv_x, cand1->mv_y, blocksize_x_cr, blocksize_y_cr, ref1, k+1 ) ;

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
*   JLT : SAD computation _with_ Hadamard Transform (on-the-fly)
************************************************************************
*/
distblk computeSATD_otf(StorablePicture *ref1,
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];

  imgpel  *src_tmp = mv_block->orig_pic[0];
  short   *d, diff[MB_PIXELS];
  imgpel  *src_line, *ref_line, data[MB_PIXELS] ;
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;


  if ( !mv_block->test8x8 )
  { // 4x4 TRANSFORM
    src_size_x = blocksize_x - BLOCK_SIZE;
    src_size_mul = blocksize_x * BLOCK_SIZE;
    for (y = cand->mv_y; y < cand->mv_y + (blocksize_y<<2); y += (BLOCK_SIZE_SP))
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE)
      {
        d    = diff;
        ref_line = data ;
        p_Dpb->pf_get_block_luma( p_Vid, ref_line, tmp_line, cand->mv_x + (x<<2) , y, BLOCK_SIZE, BLOCK_SIZE, ref1, 0 );
        src_line = src_tmp + x;
        for (y4 = 0; y4 < BLOCK_SIZE; y4++ )
        {
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;
          *d++ = *src_line++ - *ref_line++ ;

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
        ref_line = data ;
        p_Dpb->pf_get_block_luma( p_Vid, ref_line, tmp_line, cand->mv_x + (x<<2) , y, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, ref1, 0 );
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
*  JLT :  SAD computation of weighted samples _with_ Hadamard Transform ( on-the-fly )
************************************************************************
*/
distblk computeSATDWP_otf(StorablePicture *ref1,
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
  short luma_log_weight_denom = currSlice->luma_log_weight_denom;
  short weight = mv_block->weight_luma;
  short offset = mv_block->offset_luma; 

  int wp_luma_round = currSlice->wp_luma_round;  
  int max_imgpel_value = p_Vid->max_imgpel_value;
  short *d, diff[MB_PIXELS];
  imgpel *src_line, *ref_line, data[MB_PIXELS];
  int tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;

  if ( !mv_block->test8x8 )
  { // 4x4 TRANSFORM
    src_size_x = (blocksize_x - BLOCK_SIZE);
    src_size_mul = blocksize_x * BLOCK_SIZE;
    for (y = cand->mv_y; y < cand->mv_y + (blocksize_y<<2); y += (BLOCK_SIZE_SP))
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE)
      {
        d    = diff;
        ref_line = data;
        p_Dpb->pf_get_block_luma( p_Vid, ref_line, tmp_line, cand->mv_x + (x<<2) , y, BLOCK_SIZE, BLOCK_SIZE, ref1, 0 );
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
        ref_line = data;
        p_Dpb->pf_get_block_luma( p_Vid, ref_line, tmp_line, cand->mv_x + (x<<2) , y, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, ref1, 0 );
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
*  JLT :  BiPred (w/o weights) SATD computation ( on-the-fly )
************************************************************************
*/
distblk computeBiPredSATD1_otf(StorablePicture *ref1, 
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
  imgpel *src_line, *ref1_line, *ref2_line, data1[MB_PIXELS], data2[MB_PIXELS] ;
  int tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;
  short blocksize_x = mv_block->blocksize_x;
  short blocksize_y = mv_block->blocksize_y;
  VideoParameters *p_Vid = mv_block->p_Vid;
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];

  if ( !mv_block->test8x8 )
  { // 4x4 TRANSFORM
    src_size_x = (blocksize_x - BLOCK_SIZE);
    src_size_mul = blocksize_x * BLOCK_SIZE;
    for (y=0; y<(blocksize_y<<2); y += (BLOCK_SIZE_SP))
    {
      for (x=0; x<blocksize_x; x += BLOCK_SIZE)
      {
        d    = diff;
        ref2_line = data2;
        ref1_line = data1;
        src_line   = src_tmp + x;
        p_Dpb->pf_get_block_luma( p_Vid, ref2_line, tmp_line, cand2->mv_x + (x<<2) , cand2->mv_y + y, BLOCK_SIZE, BLOCK_SIZE, ref2, 0 );
        p_Dpb->pf_get_block_luma( p_Vid, ref1_line, tmp_line, cand1->mv_x + (x<<2) , cand1->mv_y + y, BLOCK_SIZE, BLOCK_SIZE, ref1, 0 );
        
        for (y4 = 0; y4 < BLOCK_SIZE; y4++ )
        {
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          *d++ = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);

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
        ref2_line  = data2;
        ref1_line  = data1;
        src_line   = src_tmp + x;
        p_Dpb->pf_get_block_luma( p_Vid, ref2_line, tmp_line, cand2->mv_x + (x<<2) , y_pos2, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, ref2, 0 );
        p_Dpb->pf_get_block_luma( p_Vid, ref1_line, tmp_line, cand1->mv_x + (x<<2) , y_pos1, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, ref1, 0 );

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
*  JLT :  BiPred (w/ weights) SATD computation ( on-the-fly )
************************************************************************
*/
distblk computeBiPredSATD2_otf(StorablePicture *ref1, 
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
  int denom = currSlice->luma_log_weight_denom + 1;
  int lround = 2 * currSlice->wp_luma_round;
  short weight1 = mv_block->weight1;
  short weight2 = mv_block->weight2;
  short offsetBi = mv_block->offsetBi;


  int max_imgpel_value = p_Vid->max_imgpel_value;
  int src_size_x, src_size_mul;
  imgpel *src_tmp = mv_block->orig_pic[0];
  short *d, diff[MB_PIXELS];
  imgpel *src_line, *ref1_line, *ref2_line, data1[MB_PIXELS], data2[MB_PIXELS] ;
  int tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;
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
        ref2_line = data2;
        ref1_line = data1;
        src_line   = src_tmp + x;
        p_Dpb->pf_get_block_luma( p_Vid, ref2_line, tmp_line, cand2->mv_x + (x<<2) , cand2->mv_y + y, BLOCK_SIZE, BLOCK_SIZE, ref2, 0 );
        p_Dpb->pf_get_block_luma( p_Vid, ref1_line, tmp_line, cand1->mv_x + (x<<2) , cand1->mv_y + y, BLOCK_SIZE, BLOCK_SIZE, ref1, 0 );

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
        ref2_line  = data2;
        ref1_line  = data1;
        src_line   = src_tmp + x;
        p_Dpb->pf_get_block_luma( p_Vid, ref2_line, tmp_line, cand2->mv_x + (x<<2) , y_pos2, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, ref2, 0 );
        p_Dpb->pf_get_block_luma( p_Vid, ref1_line, tmp_line, cand1->mv_x + (x<<2) , y_pos1, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8, ref1, 0 );

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
*   JLT : SSE computation ( on-the-fly )
************************************************************************
*/
distblk computeSSE_otf(StorablePicture *ref1,
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];

  imgpel *src_line = mv_block->orig_pic[0];
  imgpel *ref_line , data[MB_PIXELS] ; 
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;
  ref_line = data;

  p_Dpb->pf_get_block_luma( p_Vid, ref_line, tmp_line, cand->mv_x, cand->mv_y, blocksize_x, blocksize_y, ref1, 0 );

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
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int k;
    int mcr_cost = 0;

    for (k=0; k<2; k++)
    {
      mcr_cost = 0;
      src_line = mv_block->orig_pic[k+1];
      ref_line = data ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref_line, tmp_line, cand->mv_x, cand->mv_y, blocksize_x_cr, blocksize_y_cr, ref1, k+1 ) ;
      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          mcr_cost += iabs2( *src_line++ - *ref_line++ );
          mcr_cost += iabs2( *src_line++ - *ref_line++ );
        }
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
*   JLT : SSE computation of weighted samples ( on-the-fly )
************************************************************************
*/
distblk computeSSEWP_otf(StorablePicture *ref1,
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
  short weight = mv_block->weight_luma;
  short offset = mv_block->offset_luma; 

  int wp_luma_round = currSlice->wp_luma_round;  
  int max_imgpel_value = p_Vid->max_imgpel_value;
  short luma_log_weight_denom = currSlice->luma_log_weight_denom;

  imgpel *src_line = mv_block->orig_pic[0];
  imgpel *ref_line , data[MB_PIXELS] ; 
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;

  ref_line = data;
  p_Dpb->pf_get_block_luma( p_Vid, ref_line, tmp_line, cand->mv_x, cand->mv_y, blocksize_x, blocksize_y, ref1, 0 );

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
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    // These could be made global to reduce computations
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
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
      ref_line = data ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref_line, tmp_line, cand->mv_x, cand->mv_y, blocksize_x_cr, blocksize_y_cr, ref1, k+1 ) ;
      for (y=0; y<blocksize_y_cr; y++)
      {

        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          weighted_pel = iClip1( max_imgpel_value_uv, ((weight * *ref_line++  + wp_chroma_round) >> chroma_log_weight_denom) + offset);
          mcr_cost += iabs2( *src_line++ - weighted_pel );
          weighted_pel = iClip1( max_imgpel_value_uv, ((weight * *ref_line++  + wp_chroma_round) >> chroma_log_weight_denom) + offset);
          mcr_cost += iabs2( *src_line++ - weighted_pel );
        }
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
*  JLT :  BiPred SSE computation (no weights) ( on-the-fly )
************************************************************************
*/
distblk computeBiPredSSE1_otf(StorablePicture *ref1, 
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];

  imgpel  *src_line, *ref2_line, *ref1_line ; 
  imgpel  data2[MB_PIXELS], data1[MB_PIXELS];                                  // local allocation could be optimized by a global instanciation
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;
  src_line = mv_block->orig_pic[0];
  ref2_line = data2 ;
  ref1_line = data1 ;

  p_Dpb->pf_get_block_luma( p_Vid, ref2_line, tmp_line, cand2->mv_x, cand2->mv_y, blocksize_x, blocksize_y, ref2, 0 );
  p_Dpb->pf_get_block_luma( p_Vid, ref1_line, tmp_line, cand1->mv_x, cand1->mv_y, blocksize_x, blocksize_y, ref1, 0 );

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
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
    int k;
    int mcr_cost = 0;

    for (k=0; k<2; k++)
    {
      mcr_cost = 0;
      src_line = mv_block->orig_pic[k+1];
      ref2_line = data2 ;
      ref1_line = data1 ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref2_line, tmp_line, cand2->mv_x, cand2->mv_y, blocksize_x_cr, blocksize_y_cr, ref2, k+1 ) ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref1_line, tmp_line, cand1->mv_x, cand1->mv_y, blocksize_x_cr, blocksize_y_cr, ref1, k+1 ) ;

      for (y=0; y<blocksize_y_cr; y++)
      {
        for (x = 0; x < blocksize_x_cr; x+=2)
        {
          bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          mcr_cost += iabs2(bi_diff);
          bi_diff = (*src_line++) - ((*ref1_line++ + *ref2_line++ + 1)>>1);
          mcr_cost += iabs2(bi_diff);
        }
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
*  JLT :  BiPred SSE computation (with weights) ( on-the-fly )
************************************************************************
*/
distblk computeBiPredSSE2_otf(StorablePicture *ref1, 
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
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
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

  imgpel  *src_line, *ref2_line, *ref1_line ; 
  imgpel  data2[MB_PIXELS], data1[MB_PIXELS];                                  // local allocation could be optimized by a global instanciation
  int     tmp_line[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;
  src_line = mv_block->orig_pic[0];
  ref2_line = data2 ;
  ref1_line = data1 ;

  p_Dpb->pf_get_block_luma( p_Vid, ref2_line, tmp_line, cand2->mv_x, cand2->mv_y, blocksize_x, blocksize_y, ref2, 0 );
  p_Dpb->pf_get_block_luma( p_Vid, ref1_line, tmp_line, cand1->mv_x, cand1->mv_y, blocksize_x, blocksize_y, ref1, 0 );

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
  }

  if ( mv_block->ChromaMEEnable ) 
  {
    // calculate chroma conribution to motion compensation error
    int blocksize_x_cr = mv_block->blocksize_cr_x;
    int blocksize_y_cr = mv_block->blocksize_cr_y;
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
      ref2_line = data2 ;
      ref1_line = data1 ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref2_line, tmp_line, cand2->mv_x, cand2->mv_y, blocksize_x_cr, blocksize_y_cr, ref2, k+1 ) ;
      p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, ref1_line, tmp_line, cand1->mv_x, cand1->mv_y, blocksize_x_cr, blocksize_y_cr, ref1, k+1 ) ;

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
      }
      mcost += mv_block->ChromaMEWeight * mcr_cost;
      if(mcost > imin_cost)
        return dist_scale_f((distblk)mcost);
    }
  }

  CHECKOVERFLOW(mcost);
  return dist_scale((distblk)mcost);
}

