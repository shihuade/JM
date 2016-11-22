
/*!
*************************************************************************************
* \file me_fullfast.c
*
* \brief
*    Motion Estimation using Full Search Fast
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*      - Alexis Michael Tourapis <alexismt@ieee.org>
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
#include "refbuf.h"

#include "me_distortion.h"
#include "me_fullsearch.h"
#include "me_fullfast.h"
#include "conformance.h"
#include "mv_search.h"


// Functions
/*!
 ***********************************************************************
 * \brief
 *    Full pixel block motion search
 ***********************************************************************
 */


/*!
 ***********************************************************************
 * \brief
 *    function creating arrays for fast integer motion estimation
 ***********************************************************************
 */
void initialize_fast_full_search (VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int  i, j, k, list;
  int  search_range = p_Inp->SearchMode[0] == FAST_FULL_SEARCH ? p_Inp->search_range[0] : p_Inp->search_range[1];
  int  max_pos      = (2*search_range+1) * (2*search_range+1);
  MEFullFast *p_me_ffast = NULL;
   
   if ((p_Vid->p_ffast_me = calloc (1, sizeof (MEFullFast))) == NULL) 
    no_mem_exit ("initialize_fast_full_search: p_Vid->p_ffast_me");
   p_me_ffast = p_Vid->p_ffast_me;

  if ((p_me_ffast->BlockSAD = (distpel*****)malloc (2 * sizeof(distpel****))) == NULL)
    no_mem_exit ("initialize_fast_full_search: p_me_ffast->BlockSAD");

  for (list=0; list<2;list++)
  {
    if ((p_me_ffast->BlockSAD[list] = (distpel****)malloc ((p_Vid->max_num_references) * sizeof(distpel***))) == NULL)
      no_mem_exit ("initialize_fast_full_search: p_me_ffast->BlockSAD");
    for (i = 0; i < p_Vid->max_num_references; i++)
    {
      if ((p_me_ffast->BlockSAD[list][i] = (distpel***)malloc (8 * sizeof(distpel**))) == NULL)
        no_mem_exit ("initialize_fast_full_search: p_me_ffast->BlockSAD");
      for (j = 1; j < 8; j++)
      {
        if ((p_me_ffast->BlockSAD[list][i][j] = (distpel**)malloc (16 * sizeof(distpel*))) == NULL)
          no_mem_exit ("initialize_fast_full_search: p_me_ffast->BlockSAD");
        for (k = 0; k < 16; k++)
        {
          if ((p_me_ffast->BlockSAD[list][i][j][k] = (distpel*)malloc (max_pos * sizeof(distpel))) == NULL)
            no_mem_exit ("initialize_fast_full_search: p_me_ffast->BlockSAD");
        }
      }
    }
  }

  if ((p_me_ffast->search_setup_done = (int**)malloc (2*sizeof(int*)))==NULL)
    no_mem_exit ("initialize_fast_full_search: p_me_ffast->search_setup_done");
  if ((p_me_ffast->search_center = (MotionVector**) malloc (2 * sizeof(MotionVector*)))==NULL)
    no_mem_exit ("initialize_fast_full_search: p_me_ffast->search_center");
  if ((p_me_ffast->search_center_padded = (MotionVector**) malloc (2 * sizeof(MotionVector*)))==NULL)
    no_mem_exit ("initialize_fast_full_search: p_me_ffast->search_center_padded");

  if ((p_me_ffast->pos_00 = (int**)malloc (2*sizeof(int*)))==NULL)
    no_mem_exit ("initialize_fast_full_search: p_me_ffast->pos_00");
  if ((p_me_ffast->max_search_range = (int**)malloc (2*sizeof(int*)))==NULL)
    no_mem_exit ("initialize_fast_full_search: p_me_ffast->max_search_range");

  for (list=0; list<2; list++)
  {
    if ((p_me_ffast->search_setup_done[list] = (int*)malloc ((p_Vid->max_num_references)*sizeof(int)))==NULL)
      no_mem_exit ("initialize_fast_full_search: p_me_ffast->search_setup_done");
    if ((p_me_ffast->search_center[list] = (MotionVector*) malloc ((p_Vid->max_num_references) * sizeof(MotionVector)))==NULL)
      no_mem_exit ("initialize_fast_full_search: p_me_ffast->search_center");
    if ((p_me_ffast->search_center_padded[list] = (MotionVector*) malloc ((p_Vid->max_num_references) * sizeof(MotionVector)))==NULL)
      no_mem_exit ("initialize_fast_full_search: p_me_ffast->search_center_padded");


    if ((p_me_ffast->pos_00[list] = (int*)malloc ((p_Vid->max_num_references)*sizeof(int)))==NULL)
      no_mem_exit ("initialize_fast_full_search: p_me_ffast->pos_00");
    if ((p_me_ffast->max_search_range[list] = (int*)malloc ((p_Vid->max_num_references)*sizeof(int)))==NULL)
      no_mem_exit ("initialize_fast_full_search: p_me_ffast->max_search_range");
  }

  // assign max search ranges for reference frames
  if (p_Inp->full_search == 2)
  {
    for (list=0;list<2;list++)
      for (i=0; i<p_Vid->max_num_references; i++)
        p_me_ffast->max_search_range[list][i] = search_range;
  }
  else
  {
    for (list=0;list<2;list++)
    {
      p_me_ffast->max_search_range[list][0] = search_range;
      for (i=1; i< p_Vid->max_num_references; i++)  p_me_ffast->max_search_range[list][i] = search_range / 2;
    }
  }
}

/*!
 ***********************************************************************
 * \brief
 *    function for deleting the arrays for fast integer motion estimation
 ***********************************************************************
 */
void
clear_fast_full_search (VideoParameters *p_Vid)
{
  int  i, j, k, list;
  MEFullFast *p_me_ffast = p_Vid->p_ffast_me;

  for (list=0; list<2; list++)
  {
    for (i = 0; i < p_Vid->max_num_references; i++)
    {
      for (j = 1; j < 8; j++)
      {
        for (k = 0; k < 16; k++)
        {
          free (p_me_ffast->BlockSAD[list][i][j][k]);
        }
        free (p_me_ffast->BlockSAD[list][i][j]);
      }
      free (p_me_ffast->BlockSAD[list][i]);
    }
    free (p_me_ffast->BlockSAD[list]);
  }
  free (p_me_ffast->BlockSAD);

  for (list=0; list<2; list++)
  {
    free (p_me_ffast->search_setup_done[list]);
    free (p_me_ffast->search_center_padded[list]);
    free (p_me_ffast->search_center[list]);
    free (p_me_ffast->pos_00[list]);
    free (p_me_ffast->max_search_range[list]);
  }
  free (p_me_ffast->search_setup_done);
  free (p_me_ffast->search_center_padded);
  free (p_me_ffast->search_center);
  free (p_me_ffast->pos_00);
  free (p_me_ffast->max_search_range);
  free (p_me_ffast);

}


/*!
 ***********************************************************************
 * \brief
 *    function resetting flags for fast integer motion estimation
 *    (have to be called in start_macroblock())
 ***********************************************************************
 */
void reset_fast_full_search (VideoParameters *p_Vid)
{
  int list;
  for (list=0; list<2; list++)
    memset(&p_Vid->p_ffast_me->search_setup_done [list][0], 0, p_Vid->max_num_references * sizeof(int));
}
/*!
 ***********************************************************************
 * \brief
 *    calculation of SAD for larger blocks on the basis of 4x4 blocks
 ***********************************************************************
 */
void
update_full_search_large_blocks (MEFullFast *p_ffast_me, int list, int refindex, int max_pos)
{
#define ADD_UP_BLOCKS()   _o=*_bo; _i=*_bi; _j=*_bj; for(pos=0;pos<max_pos;pos++) _o[pos] = _i[pos] + _j[pos];
#define INCREMENT(inc)    _bo+=inc; _bi+=inc; _bj+=inc;
  distpel  *****BlockSAD = p_ffast_me->BlockSAD;

  int       pos;
  distpel   **_bo, **_bi, **_bj;
  distpel *_o,   *_i,   *_j;

  //--- blocktype 6 ---
  _bo = BlockSAD[list][refindex][6];
  _bi = BlockSAD[list][refindex][7];
  _bj = _bi + 4;
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(5);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS();

  //--- blocktype 5 ---
  _bo = BlockSAD[list][refindex][5];
  _bi = BlockSAD[list][refindex][7];
  _bj = _bi + 1;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 4 ---
  _bo = BlockSAD[list][refindex][4];
  _bi = BlockSAD[list][refindex][6];
  _bj = _bi + 1;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(6);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 3 ---
  _bo = BlockSAD[list][refindex][3];
  _bi = BlockSAD[list][refindex][4];
  _bj = _bi + 8;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 2 ---
  _bo = BlockSAD[list][refindex][2];
  _bi = BlockSAD[list][refindex][4];
  _bj = _bi + 2;
  ADD_UP_BLOCKS(); INCREMENT(8);
  ADD_UP_BLOCKS();

  //--- blocktype 1 ---
  _bo = BlockSAD[list][refindex][1];
  _bi = BlockSAD[list][refindex][3];
  _bj = _bi + 2;
  ADD_UP_BLOCKS();
}


/*!
 ***********************************************************************
 * \brief
 *    Setup the fast search for an macroblock
 ***********************************************************************
 */
void setup_fast_full_search (Macroblock *currMB, MEBlock *mv_block, int list)  // <--  reference frame parameter, list0 or 1
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int (*dist_method) (int x) = p_Inp->MEErrorMetric[0] ? iabs2 : iabs;
#if (JM_MEM_DISTORTION)
  int*    imgpel_dist   = p_Inp->MEErrorMetric[0] ? p_Vid->imgpel_quad : p_Vid->imgpel_abs;
#endif
  MotionVector   pmv;
  imgpel orig_pels[768];
  imgpel  *srcptr = orig_pels, *refptr;
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

      refptr = UMVLine4X (ref_picture, cand.mv_y, cand.mv_x);

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
          refptr += p_Vid->padded_size_x - MB_BLOCK_SIZE;
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

          refptr = UMVLine8X_chroma (ref_picture, k+1, cand.mv_y, cand.mv_x);
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
              refptr += p_Vid->cr_padded_size_x - p_Vid->mb_cr_size_x;
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

      refptr = UMVLine4X (ref_picture, cand.mv_y, cand.mv_x);

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

          refptr += p_Vid->padded_size_x - MB_BLOCK_SIZE;
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

          refptr = UMVLine8X_chroma (ref_picture, k+1, cand.mv_y, cand.mv_x);
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
              refptr += p_Vid->cr_padded_size_x - p_Vid->mb_cr_size_x;
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


/*!
 ***********************************************************************
 * \brief
 *    Fast Full pixel block motion search
 ***********************************************************************
 */
distblk                                                          //  ==> minimum motion cost after search
fast_full_search_motion_estimation (Macroblock   *currMB,        // <--  current Macroblock
                                    MotionVector *pred_mv,       // <--  motion vector predictor (x) in sub-pel units
                                    MEBlock      *mv_block,      // <--  motion estimation parameters
                                    distblk       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                    int           lambda_factor  // <--  lagrangian parameter for determining motion cost
                                    )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  MEFullFast *p_me_ffast = p_Vid->p_ffast_me;
  distblk mcost;
  int   pos;  
  int  search_range   = imax(mv_block->searchRange.max_x, mv_block->searchRange.max_y) >> 2;
  int   max_pos       = (2*search_range+1)*(2*search_range+1);              // number of search positions
  int   best_pos      = 0;                                                  // position with minimum motion cost
  int   block_index;                                                        // block index for indexing SAD array
  distpel*  block_sad;                                                          // pointer to SAD array
  int   list = mv_block->list;
  short ref  = mv_block->ref_idx;
  MotionVector cand = {0, 0}, *offset = &p_Vid->p_ffast_me->search_center[list][ref];
  int max_mvd = p_Vid->max_mvd-1;

  block_index = (mv_block->block_y << 2) + (mv_block->block_x); // block index for indexing SAD array
  block_sad   = p_me_ffast->BlockSAD[list][ref][mv_block->blocktype][block_index];         // pointer to SAD array
  
  //===== set up fast full integer search if needed / set search center =====
  if (!p_Vid->p_ffast_me->search_setup_done[list][ref])
  {
    currMB->p_SetupFastFullPelSearch (currMB, mv_block, list);
  }

  //===== cost for (0,0)-vector: it is done before, because MVCost can be negative =====
  if (!p_Inp->rdopt && GetMaxMVD(&cand, pred_mv)<max_mvd)
  {
    min_mcost = block_sad[p_me_ffast->pos_00[list][ref]];
    up_scale(&min_mcost);
    min_mcost += mv_cost (p_Vid, lambda_factor, &cand, pred_mv);

    best_pos = p_me_ffast->pos_00[list][ref];
  }

  //===== loop over all search positions =====
  for (pos=0; pos<max_pos; pos++, block_sad++)
  {
    mcost  = dist_scale(*block_sad);
    cand = add_MVs (p_Vid->spiral_qpel_search[pos], offset);
    /* //only for debug purpose;
    if(GetMaxMVD(&cand, pred_mv) >= max_mvd)
    {
      printf("\n\n%d: out of range(%s%d)!\n\n", p_Vid->frame_no, __FILE__, __LINE__);
    }
    */
    //--- check residual cost ---
    if (mcost < min_mcost && GetMaxMVD(&cand, pred_mv)<max_mvd)
    {
      //--- get motion vector cost ---
      mcost += mv_cost (p_Vid, lambda_factor, &cand, pred_mv);

      //--- check motion cost ---
      if (mcost < min_mcost)
      {
        min_mcost = mcost;
        best_pos  = pos;
      }
    }
  }

  //===== set best motion vector and return minimum motion cost =====
  mv_block->mv[list] = add_MVs (p_Vid->spiral_qpel_search[best_pos], offset);

  return min_mcost;
}

