/*!
*************************************************************************************
* \file me_fullfast_otf.c
*
* \brief
*    Motion Estimation using Full Search Fast (extention for on-the-fly interpolation)
*
*
*************************************************************************************
*/

// Includes
#include "contributors.h"
#include <limits.h>

#include "global.h"
#include "image.h"
#include "memalloc.h"
#include "mb_access.h"

#include "me_distortion.h"
#include "get_block_otf.h"

#include "me_fullsearch.h"
#include "me_fullfast.h"
#include "me_fullfast_otf.h"
#include "conformance.h"
#include "mv_search.h"

/*!
 ***********************************************************************
 * \brief
 *    Setup the fast search for an macroblock (interpolation on-the-fly)
 ***********************************************************************
 */
void SetupFastFullPelSearch_otf (Macroblock *currMB, MEBlock *mv_block, int list)  // <--  reference frame parameter, list0 or 1
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
  int (*dist_method) (int x) = p_Inp->MEErrorMetric[0 + (( p_Vid->dpb_layer_id >= 1 ) ? 3 : 0)] ? iabs2 : iabs;
#if (JM_MEM_DISTORTION)
  int*    imgpel_dist   = p_Inp->MEErrorMetric[0 + p_Vid->layer_offset] ? p_Vid->imgpel_quad : p_Vid->imgpel_abs;
#endif
  MotionVector   pmv;
  imgpel orig_pels[768];
  imgpel data[MB_PIXELS] ;
  int    tmp_data[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ] ;
  imgpel  *srcptr = orig_pels, *refptr = data ;
  int     k, x, y;
  MotionVector cand, offset;
  int     ref_x, ref_y, pos, bindex, blky;
  distblk  LineSadBlk0, LineSadBlk1, LineSadBlk2, LineSadBlk3;
  PixelPos block[4];  // neighbor blocks
  MEFullFast *p_me_ffast = p_Vid->p_ffast_me;


  short ref = mv_block->ref_idx;
  distpel**   block_sad = p_me_ffast->BlockSAD[list][ref][7];
  int     search_range  = p_me_ffast->max_search_range[list][ref];
  int     max_pos       = (2*search_range+1) * (2*search_range+1);

  int     list_offset   = p_Vid->mb_data[currMB->mbAddrX].list_offset;
  int     apply_weights = ( (p_Vid->active_pps->weighted_pred_flag && (currSlice->slice_type == P_SLICE || (currSlice->slice_type == SP_SLICE))) ||
    (p_Vid->active_pps->weighted_bipred_idc && (currSlice->slice_type == B_SLICE))) && p_Inp->UseWeightedReferenceME;
  int     weighted_pel;
  StorablePicture *ref_picture = currSlice->listX[list+list_offset][ref];

  int   wp_luma_round = 0;
  short luma_log_weight_denom  = 5;
  short chroma_log_weight_denom = 5;
  int   wp_chroma_round = 0;
  short weight_luma = mv_block->weight_luma, weight_cr[2];
  short offset_luma = mv_block->offset_luma, offset_cr[2];
  search_range <<= 2;  

  //===== get search center: predictor of 16x16 block =====
  get_neighbors(currMB, block, 0, 0, 16);
  currMB->GetMVPredictor (currMB, block, &pmv, ref, p_Vid->enc_picture->mv_info, list, 0, 0, 16, 16);

#if (JM_INT_DIVIDE)
  p_Vid->p_ffast_me->search_center[list][ref].mv_x = ((pmv.mv_x + 2) >> 2) * 4;
  p_Vid->p_ffast_me->search_center[list][ref].mv_y = ((pmv.mv_y + 2) >> 2) * 4;
#else
  p_Vid->p_ffast_me->search_center[list][ref].mv_x = (pmv.mv_x / 4) * 4;
  p_Vid->p_ffast_me->search_center[list][ref].mv_y = (pmv.mv_y / 4) * 4;
#endif
  if (!p_Inp->rdopt)
  {
    //--- correct center so that (0,0) vector is inside ---
    p_Vid->p_ffast_me->search_center[list][ref].mv_x = (short) iClip3(-search_range, search_range, p_Vid->p_ffast_me->search_center[list][ref].mv_x);
    p_Vid->p_ffast_me->search_center[list][ref].mv_y = (short) iClip3(-search_range, search_range, p_Vid->p_ffast_me->search_center[list][ref].mv_y);
  }

  p_Vid->p_ffast_me->search_center[list][ref].mv_x = (short) iClip3(p_Vid->MaxHmvR[4] + search_range, p_Vid->MaxHmvR[5] - search_range, p_Vid->p_ffast_me->search_center[list][ref].mv_x);
  p_Vid->p_ffast_me->search_center[list][ref].mv_y = (short) iClip3(p_Vid->MaxVmvR[4] + search_range, p_Vid->MaxVmvR[5] - search_range, p_Vid->p_ffast_me->search_center[list][ref].mv_y);

  p_Vid->p_ffast_me->search_center_padded[list][ref] = pad_MVs(p_Vid->p_ffast_me->search_center[list][ref], mv_block);

  offset = p_Vid->p_ffast_me->search_center_padded[list][ref];
  //===== copy original block for fast access =====
  for   (y = currMB->opix_y; y < currMB->opix_y + MB_BLOCK_SIZE; y++)
  {
    memcpy(srcptr, &p_Vid->pCurImg[y][currMB->pix_x], MB_BLOCK_SIZE * sizeof(imgpel));
    srcptr += MB_BLOCK_SIZE;
  }

  if ( mv_block->ChromaMEEnable)
  {
    for (k = 1; k < 3; k++)
    {
      for   (y = currMB->opix_c_y; y < currMB->opix_c_y + p_Vid->mb_cr_size_y; y++)
      {
        memcpy(srcptr, &p_Vid->pImgOrg[k][y][currMB->pix_c_x], p_Vid->mb_cr_size_x * sizeof(imgpel));
        srcptr += p_Vid->mb_cr_size_x;
      }
    }
  }


  //===== determine position of (0,0)-vector =====
  if (!p_Inp->rdopt)
  {
    ref_x = mv_block->pos_x_padded - offset.mv_x;
    ref_y = mv_block->pos_y_padded - offset.mv_y;

    for (pos = 0; pos < max_pos; pos++)
    {
      if (ref_x == p_Vid->spiral_qpel_search[pos].mv_x && ref_y == p_Vid->spiral_qpel_search[pos].mv_y)
      {
        p_me_ffast->pos_00[list][ref] = pos;
        break;
      }
    }
  }

  //===== loop over search range (spiral search): get blockwise SAD =====
  if (apply_weights)
  {
    weight_luma = currSlice->wp_weight[list + list_offset][ref][0];
    offset_luma = currSlice->wp_offset[list + list_offset][ref][0];
    wp_luma_round = currSlice->wp_luma_round;
    luma_log_weight_denom = currSlice->luma_log_weight_denom;
    chroma_log_weight_denom = currSlice->chroma_log_weight_denom;
    wp_chroma_round = currSlice->wp_chroma_round;

    if (mv_block->ChromaMEEnable)
    {
      weight_cr[0] = currSlice->wp_weight[list + list_offset][ref][1];
      weight_cr[1] = currSlice->wp_weight[list + list_offset][ref][2];
      offset_cr[0] = currSlice->wp_offset[list + list_offset][ref][1];
      offset_cr[1] = currSlice->wp_offset[list + list_offset][ref][2];
    }

    for (pos = 0; pos < max_pos; pos++)
    {
      cand = add_MVs(offset, &p_Vid->spiral_qpel_search[pos]);      

      srcptr = orig_pels;
      bindex = 0;
      refptr = data ;
      p_Dpb->pf_get_block_luma ( p_Vid, refptr, tmp_data, cand.mv_x, cand.mv_y, MB_BLOCK_SIZE, MB_BLOCK_SIZE, ref_picture, 0 ) ;

      for (blky = 0; blky < 4; blky++)
      {
        LineSadBlk0 = LineSadBlk1 = LineSadBlk2 = LineSadBlk3 = 0;

        for (y = 0; y < 4; y++)
        {
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk0 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk0 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk0 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk0 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk1 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk1 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk1 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk1 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk2 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk2 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk2 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk2 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk3 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk3 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk3 += dist_method (weighted_pel - *srcptr++);
          weighted_pel = iClip1( p_Vid->max_imgpel_value, ((weight_luma * *refptr++  + wp_luma_round) >> luma_log_weight_denom) + offset_luma);
          LineSadBlk3 += dist_method (weighted_pel - *srcptr++); 
        }
        block_sad[bindex++][pos] = (distpel) LineSadBlk0;
        block_sad[bindex++][pos] = (distpel) LineSadBlk1;
        block_sad[bindex++][pos] = (distpel) LineSadBlk2;
        block_sad[bindex++][pos] = (distpel) LineSadBlk3;

      }
      if (mv_block->ChromaMEEnable)
      {
        int max_imgpel_value_uv = p_Vid->max_pel_value_comp[1];

        for (k = 0; k < 2; k ++)
        {
          bindex = 0;
          refptr = data ;
          p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, refptr, tmp_data, cand.mv_x, cand.mv_y, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y, ref_picture, k+1 );

          for (blky = 0; blky < 4; blky++)
          {
            LineSadBlk0 = LineSadBlk1 = LineSadBlk2 = LineSadBlk3 = 0;

            for (y = 0; y < p_Vid->mb_cr_size_y; y+=BLOCK_SIZE)
            {
              for (x = 0; x < p_Vid->mb_cr_size_x; x += BLOCK_SIZE)
              {
                weighted_pel = iClip1( max_imgpel_value_uv, ((weight_cr[k] * *refptr++  + wp_chroma_round) >> chroma_log_weight_denom) + offset_cr[k]);
                LineSadBlk0 += dist_method (weighted_pel - *srcptr++);
              }
              for (x = 0; x < p_Vid->mb_cr_size_x; x += BLOCK_SIZE)
              {
                weighted_pel = iClip1( max_imgpel_value_uv, ((weight_cr[k] * *refptr++  + wp_chroma_round) >> chroma_log_weight_denom) + offset_cr[k]);
                LineSadBlk1 += dist_method (weighted_pel - *srcptr++);
              }
              for (x = 0; x < p_Vid->mb_cr_size_x; x += BLOCK_SIZE)
              {
                weighted_pel = iClip1( max_imgpel_value_uv, ((weight_cr[k] * *refptr++  + wp_chroma_round) >> chroma_log_weight_denom) + offset_cr[k]);
                LineSadBlk2 += dist_method (weighted_pel - *srcptr++);
              }
              for (x = 0; x < p_Vid->mb_cr_size_x; x += BLOCK_SIZE)
              {
                weighted_pel = iClip1( max_imgpel_value_uv, ((weight_cr[k] * *refptr++  + wp_chroma_round) >> chroma_log_weight_denom) + offset_cr[k]);
                LineSadBlk3 += dist_method (weighted_pel - *srcptr++);
              }
            }
            block_sad[bindex][pos] = block_sad[bindex][pos] + ((distpel) LineSadBlk0);
            ++bindex;
            block_sad[bindex][pos] = block_sad[bindex][pos] + ((distpel) LineSadBlk1);
            ++bindex;
            block_sad[bindex][pos] = block_sad[bindex][pos] + ((distpel) LineSadBlk2);
            ++bindex;
            block_sad[bindex][pos] = block_sad[bindex][pos] + ((distpel) LineSadBlk3);
            ++bindex;
          }
        }
      }
    }
  }
  else
  {
    for (pos = 0; pos < max_pos; pos++)
    {
      cand = add_MVs(offset, &p_Vid->spiral_qpel_search[pos]);
      srcptr = orig_pels;
      bindex = 0;

      refptr = data;
      p_Dpb->pf_get_block_luma( p_Vid, refptr, tmp_data, cand.mv_x, cand.mv_y, MB_BLOCK_SIZE, MB_BLOCK_SIZE, ref_picture, 0 ) ;
      
      for (blky = 0; blky < 4; blky++)
      {
        LineSadBlk0 = LineSadBlk1 = LineSadBlk2 = LineSadBlk3 = 0;

        for (y = 0; y < 4; y++)
        {
#if (JM_MEM_DISTORTION)
          // Distortion for first 4x4 block
          LineSadBlk0 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk0 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk0 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk0 += imgpel_dist[ *refptr++ - *srcptr++ ];
          // Distortion for second 4x4 block
          LineSadBlk1 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk1 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk1 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk1 += imgpel_dist[ *refptr++ - *srcptr++ ];
          // Distortion for third 4x4 block
          LineSadBlk2 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk2 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk2 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk2 += imgpel_dist[ *refptr++ - *srcptr++ ];
          // Distortion for fourth 4x4 block
          LineSadBlk3 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk3 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk3 += imgpel_dist[ *refptr++ - *srcptr++ ];
          LineSadBlk3 += imgpel_dist[ *refptr++ - *srcptr++ ];
#else
          // Distortion for first 4x4 block
          LineSadBlk0 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk0 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk0 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk0 += dist_method (*refptr++ - *srcptr++);
          // Distortion for second 4x4 block
          LineSadBlk1 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk1 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk1 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk1 += dist_method (*refptr++ - *srcptr++);
          // Distortion for third 4x4 block
          LineSadBlk2 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk2 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk2 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk2 += dist_method (*refptr++ - *srcptr++);
          // Distortion for fourth 4x4 block
          LineSadBlk3 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk3 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk3 += dist_method (*refptr++ - *srcptr++);
          LineSadBlk3 += dist_method (*refptr++ - *srcptr++);
#endif
        }
        block_sad[bindex++][pos] = (distpel) LineSadBlk0;
        block_sad[bindex++][pos] = (distpel) LineSadBlk1;
        block_sad[bindex++][pos] = (distpel) LineSadBlk2;
        block_sad[bindex++][pos] = (distpel) LineSadBlk3;
      }

      if (mv_block->ChromaMEEnable)
      {
        for (k = 0; k < 2; k ++)
        {
          bindex = 0;

          refptr = data ;
          p_Dpb->pf_get_block_chroma[OTF_ME]( p_Vid, refptr, tmp_data, cand.mv_x, cand.mv_y, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y, ref_picture, k+1 );

          for (blky = 0; blky < 4; blky++)
          {
            LineSadBlk0 = LineSadBlk1 = LineSadBlk2 = LineSadBlk3 = 0;

            for (y = 0; y < p_Vid->mb_cr_size_y; y+=BLOCK_SIZE)
            {
              for (x = 0; x < p_Vid->mb_cr_size_x; x += BLOCK_SIZE)
              {
                LineSadBlk0 += dist_method (*refptr++ - *srcptr++);
              }
              for (x = 0; x < p_Vid->mb_cr_size_x; x += BLOCK_SIZE)
              {
                LineSadBlk1 += dist_method (*refptr++ - *srcptr++);
              }
              for (x = 0; x < p_Vid->mb_cr_size_x; x += BLOCK_SIZE)
              {
                LineSadBlk2 += dist_method (*refptr++ - *srcptr++);
              }
              for (x = 0; x < p_Vid->mb_cr_size_x; x += BLOCK_SIZE)
              {
                LineSadBlk3 += dist_method (*refptr++ - *srcptr++);
              }
            }
            block_sad[bindex][pos] = block_sad[bindex][pos] + ((distpel) LineSadBlk0);
            ++bindex;
            block_sad[bindex][pos] = block_sad[bindex][pos] + ((distpel) LineSadBlk1);
            ++bindex;
            block_sad[bindex][pos] = block_sad[bindex][pos] + ((distpel) LineSadBlk2);
            ++bindex;
            block_sad[bindex][pos] = block_sad[bindex][pos] + ((distpel) LineSadBlk3);
            ++bindex;
          }
        }
      }
    }
  }

  //===== combine SAD's for larger block types =====
  update_full_search_large_blocks (p_Vid->p_ffast_me, list, ref, max_pos);

  //===== set flag marking that search setup have been done =====
  p_Vid->p_ffast_me->search_setup_done[list][ref] = 1;
}
