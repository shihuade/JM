/*!
 **************************************************************************************
 * \file
 *    slice.c
 * \brief
 *    generate the slice header, setup the bit buffer for slices,
 *    and generates the slice NALU(s)

 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Athanasios Leontaris            <aleon@dolby.com>
 *     - Karsten Suehring
 *     - Alexis Michael Tourapis         <alexismt@ieee.org> 
 ***************************************************************************************
 */

#include "contributors.h"

#include <math.h>
#include <float.h>

#include "global.h"
#include "image.h"
#include "wp.h"
#include "list_reorder.h"
#include "me_hme.h"

static void poc_ref_pic_reorder_dummy(Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no);
static void reorder_lists_dummy( Slice *currSlice );

/*!
 ************************************************************************
 * \brief
 *    init_ref_pic_list_reordering initializations should go here
 ************************************************************************
 */
void init_ref_pic_list_reordering(Slice* currSlice, int refReorderMethod, int useDistortionReordering)
{
  currSlice->ref_pic_list_reordering_flag[LIST_0] = 0;
  currSlice->ref_pic_list_reordering_flag[LIST_1] = 0;

  if (refReorderMethod == 1 && useDistortionReordering == 1)
  {
    currSlice->poc_ref_pic_reorder_frame = mse_ref_pic_reorder_frame;
    currSlice->reorder_lists             = reorder_lists;
  }
  else if (refReorderMethod == 2)
  {
    currSlice->poc_ref_pic_reorder_frame = tlyr_ref_pic_reorder_frame_default;
    currSlice->reorder_lists             = reorder_lists;
  }
#if (MVC_EXTENSION_ENABLE)
  else if ((refReorderMethod == 1) && (currSlice->layer_id > 0) && (currSlice->p_Inp->MVCInterViewReorder > 0 ))
  {
    currSlice->poc_ref_pic_reorder_frame = poc_ref_pic_reorder_frame_enh;
    currSlice->reorder_lists             = reorder_lists;
  }
  else if ((refReorderMethod == 0) && (currSlice->layer_id > 0) && (currSlice->p_Inp->MVCInterViewReorder > 0 ))
  {
    currSlice->poc_ref_pic_reorder_frame = poc_ref_pic_reorder_frame_enh;
    currSlice->reorder_lists             = reorder_lists;
  }
#endif
  else if ((refReorderMethod == 0) || (currSlice->slice_type == I_SLICE) || (currSlice->slice_type == SI_SLICE))
  {
    currSlice->poc_ref_pic_reorder_frame = poc_ref_pic_reorder_dummy;
    currSlice->reorder_lists             = reorder_lists_dummy;
  }
  else
  {
    currSlice->poc_ref_pic_reorder_frame = poc_ref_pic_reorder_frame_enh;
    currSlice->reorder_lists             = reorder_lists;
  }
}

/*!
************************************************************************
* \brief
*    decide reference picture reordering, Frame only
************************************************************************
*/
void poc_ref_pic_reorder_frame_default(Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no)
{
  StorablePicture **list = currSlice->listX[list_no];
  int *modification_of_pic_nums_idc = currSlice->modification_of_pic_nums_idc[list_no]; 
  int *abs_diff_pic_num_minus1 = currSlice->abs_diff_pic_num_minus1[list_no]; 
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  StorablePicture *p_Enc_Pic = p_Vid->enc_picture;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;

  int i,j,k;

  int currPicNum, picNumLXPred;

  int default_order[32];
  int re_order[32];
  int tmp_reorder[32];
  int list_sign[32];
  int reorder_stop, no_reorder;
  int poc_diff[32];
  int diff;

  int abs_poc_dist;
  int maxPicNum;
  unsigned int num_refs;

  maxPicNum  = p_Vid->max_frame_num;
  currPicNum = p_Vid->frame_num;

  picNumLXPred = currPicNum;

  // First assign default list order.
  for (i=0; i<(int)num_ref_idx_lX_active; i++)
  {
    default_order[i] = list[i]->pic_num;
  }

  // Now access all references in buffer and assign them
  // to a potential reordering list. For each one of these
  // references compute the poc distance compared to current
  // frame.
  num_refs = currSlice->p_Dpb->ref_frames_in_buffer;
  for (i = 0; i < (int)p_Dpb->ref_frames_in_buffer; i++)
  {
    poc_diff[i] = 0xFFFF;
    re_order[i] = p_Dpb->fs_ref[i]->frame->pic_num;

    if (p_Dpb->fs_ref[i]->is_used==3 && (p_Dpb->fs_ref[i]->frame->used_for_reference)&&(!p_Dpb->fs_ref[i]->frame->is_long_term))
    {
      abs_poc_dist = iabs(p_Dpb->fs_ref[i]->frame->poc - p_Enc_Pic->poc) ;
      poc_diff[i] = abs_poc_dist;
      if (list_no == LIST_0)
      {
        list_sign[i] = (p_Enc_Pic->poc < p_Dpb->fs_ref[i]->frame->poc) ? +1 : -1;
      }
      else
      {
        list_sign[i] = (p_Enc_Pic->poc > p_Dpb->fs_ref[i]->frame->poc) ? +1 : -1;
      }
    }
  }

  // now sort these references based on poc (temporal) distance
  for (i = 0; i < ((int)num_refs - 1); i++)
  {
    for (j = i + 1; j < (int)num_refs; j++)
    {
      if (poc_diff[i]>poc_diff[j] || (poc_diff[i] == poc_diff[j] && list_sign[j] > list_sign[i]))
      {
        i32_swap(&poc_diff[i], &poc_diff[j]);
        i32_swap(&re_order[i], &re_order[j]);
        i32_swap(&list_sign[i], &list_sign[j]);
      }
    }
  }

  // populate list with selections from the pre-analysis stage
  if ( p_Inp->WPMCPrecision 
    && p_Vid->pWPX->curr_wp_rd_pass->algorithm != WP_REGULAR
    && p_Vid->pWPX->num_wp_ref_list[list_no] )
  {
    for (i=0; i<(int)num_ref_idx_lX_active; i++)
    {
      re_order[i] = p_Vid->pWPX->wp_ref_list[list_no][i].PicNum;
    }
  }

  if (p_Inp->EnableOpenGOP)
  {
    int counter = 0;
    for (i = 0; i < currSlice->listXsize[list_no]; i++)
    {
      if (currSlice->listX[list_no][i]->poc < p_Vid->last_valid_reference && p_Vid->ThisPOC > p_Vid->last_valid_reference)
      {
        counter++;
      }
    }

    currSlice->listXsize[list_no] -= counter;
    num_ref_idx_lX_active = currSlice->listXsize[list_no];
  }

  // Check versus default list to see if any
  // change has happened
  no_reorder = 1;
  for (i=0; i<(int)num_ref_idx_lX_active; i++)
  {
    if (default_order[i] != re_order[i])
    {
      no_reorder = 0;
    }
  }

  // If different, then signal reordering
  if (no_reorder==0)
  {
    for (i=0; i<(int)num_ref_idx_lX_active; i++)
    {
      diff = re_order[i]-picNumLXPred;
      if (diff <= 0)
      {
        modification_of_pic_nums_idc[i] = 0;
        abs_diff_pic_num_minus1[i] = iabs(diff)-1;
        if (abs_diff_pic_num_minus1[i] < 0)
          abs_diff_pic_num_minus1[i] = maxPicNum -1;
      }
      else
      {
        modification_of_pic_nums_idc[i] = 1;
        abs_diff_pic_num_minus1[i] = iabs(diff)-1;
      }
      picNumLXPred = re_order[i];

      tmp_reorder[i] = re_order[i];

      k = i;
      for (j=i; j<(int)num_ref_idx_lX_active; j++)
      {
        if (default_order[j] != re_order[i])
        {
          ++k;
          tmp_reorder[k] = default_order[j];
        }
      }
      reorder_stop = 1;
      for(j=i+1; j<(int)num_ref_idx_lX_active; j++)
      {
        if (tmp_reorder[j] != re_order[j])
        {
          reorder_stop = 0;
          break;
        }
      }

      if (reorder_stop==1)
      {
        ++i;
        break;
      }
      memcpy ( default_order, tmp_reorder, num_ref_idx_lX_active * sizeof(int));

    }
    modification_of_pic_nums_idc[i] = 3;

    memcpy ( default_order, tmp_reorder, num_ref_idx_lX_active * sizeof(int));

    if (list_no==0)
    {
      currSlice->ref_pic_list_reordering_flag[LIST_0] = 1;
    }
    else
    {
      currSlice->ref_pic_list_reordering_flag[LIST_1] = 1;
    }
  }
}

/*!
************************************************************************
* \brief
*    Dummy reference reordering decision function
************************************************************************
*/
static void poc_ref_pic_reorder_dummy(Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no)
{
}

/*!
************************************************************************
* \brief
*    decide reference picture reordering, Frame only, for enh layer.
************************************************************************
*/
void poc_ref_pic_reorder_frame_enh(Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no)
{
  StorablePicture **list = currSlice->listX[list_no];
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  StorablePicture *p_Enc_Pic = p_Vid->enc_picture;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;

  int i, j, k;
  int found = 0;

  StorablePicture *re_order[32];
  StorablePicture *inter_layer_ref[32];
  StorablePicture *tmp_pic_value;

  int list_sign[32];
  int poc_diff[32];
  int inter_layer_ref_idx[32];

  int abs_poc_dist;
  int num_refs;
  int pic_num;

  // First find inter view ref/s
  // This code is idiotic since this can be found immediately given that reference is in p_Dpb->fs_ilref

#if (MVC_EXTENSION_ENABLE)
  if (currSlice->layer_id > 0)
  {
    for (i = 0; i < (int) num_ref_idx_lX_active; i++)
    {
      if (list[i]->view_id < currSlice->layer_id)
      {
        inter_layer_ref[found] = list[i];
        inter_layer_ref_idx[found] = i;
        found++;
      }
    }
  }
#endif

  // Now access all references in buffer and assign them
  // to a potential reordering list. For each one of these
  // references compute the poc distance compared to current
  // frame.  
  num_refs = p_Dpb->ref_frames_in_buffer;
  for (i = 0; i < num_refs; i++)
  {
    poc_diff[i] = 0xFFFF;
    re_order[i] = p_Dpb->fs_ref[i]->frame;

    // following code seems to exclude long term references
    if (p_Dpb->fs_ref[i]->is_used==3 && (p_Dpb->fs_ref[i]->frame->used_for_reference) && (!p_Dpb->fs_ref[i]->frame->is_long_term))
    {
      abs_poc_dist = iabs(p_Dpb->fs_ref[i]->frame->poc - p_Enc_Pic->poc) ;
      poc_diff[i] = abs_poc_dist;
      if (list_no == LIST_0)
      {
        list_sign[i] = (p_Enc_Pic->poc < p_Dpb->fs_ref[i]->frame->poc) ? +1 : -1;
      }
      else
      {
        list_sign[i] = (p_Enc_Pic->poc > p_Dpb->fs_ref[i]->frame->poc) ? +1 : -1;
      }
      //printf("Reference %d %d %d %d %d\n", i, p_Dpb->fs_ref[i]->frame->poc, p_Enc_Pic->poc, poc_diff[i], list_sign[i]);
    }
  }

  // now sort these references based on poc (temporal) distance
  for (i = 0; i < (num_refs - 1 ); i++)
  {
    for (j = i + 1; j < num_refs; j++)
    {
      if (poc_diff[i] > poc_diff[j] || (poc_diff[i] == poc_diff[j] && list_sign[j] > list_sign[i]))
      {
        i32_swap(&poc_diff[i], &poc_diff[j]);
        tmp_pic_value = re_order[i];
        re_order[i]   = re_order[j];
        re_order[j]   = tmp_pic_value ;
        i32_swap(&list_sign[i], &list_sign[j]);
      }
    }
  }
  
  // populate list with selections from the pre-analysis stage
  if ( p_Inp->WPMCPrecision 
    && p_Vid->pWPX->curr_wp_rd_pass->algorithm != WP_REGULAR
    && p_Vid->pWPX->num_wp_ref_list[list_no] )
  {
    for (i = 0; i < (int) num_ref_idx_lX_active; i++)
    {
      pic_num = p_Vid->pWPX->wp_ref_list[list_no][i].PicNum;
      for (j = 0; j < (int) p_Dpb->ref_frames_in_buffer; j++)
      {
        if (pic_num == p_Dpb->fs_ref[i]->frame->pic_num)
        {
          re_order[i] = p_Dpb->fs_ref[i]->frame;
          break;
        }
      }
    }
  }

  if (currSlice->layer_id > 0)
  {
    // This code currently only supports a single inter-layer reference. 
    // We need to have a flag in the code that points to how many such references
    // exist and appropriately scan fl_ilref.
    j = 0;
    k = 0;
    for (i = 0; i < (int) num_ref_idx_lX_active+found; i++)
    {
      if ((found > 0) && (i == inter_layer_ref_idx[k]))
      {
        list[i] = inter_layer_ref[k++];
        //printf("values %d %d", (int) list[i],  (int) p_Dpb->fs_ilref[0]->frame);
        found--;
      }
      else
      {
        list[i] = re_order[j++];
      }
    }
  }
  else
  {
    for (i = 0; i < (int) num_ref_idx_lX_active; i++)
    {
      list[i] = re_order[i];
    }
  }
}

/*!
************************************************************************
* \brief
*    decide reference picture reordering, Field.
************************************************************************
*/
void poc_ref_pic_reorder_field_enh(Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no)
{
  StorablePicture **list = currSlice->listX[list_no];
  VideoParameters *p_Vid = currSlice->p_Vid;
  //InputParameters *p_Inp = currSlice->p_Inp;
  StorablePicture *p_Enc_Pic = p_Vid->enc_picture;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;

  int i,j,k;
  int found = 0;

  StorablePicture *re_order[32];
  StorablePicture *tmp_reorder[32];
  StorablePicture *inter_layer_ref[32];
  StorablePicture *tmp_pic_value;
  StorablePicture *pField[2]; // 0: TOP_FIELD, 1: BOTTOM_FIELD
  FrameStore      *pFrameStore; 

  int list_sign[32];
  int poc_diff[32];
  int inter_layer_ref_idx[32];

  int abs_poc_dist;
  int num_refs;

  int field_used[2] = {1, 2};
  int fld_type[32];
  int num_flds, fld, idx;
  int list_size = 0;
  int top_idx = 0;
  int bot_idx = 0;


  // First find inter view ref
#if (MVC_EXTENSION_ENABLE)
  if (currSlice->layer_id > 0)
  {
    for (i = 0; i < (int) num_ref_idx_lX_active; i++)
    {
      if (list[i]->view_id < currSlice->layer_id)
      {
        inter_layer_ref[found] = list[i];
        inter_layer_ref_idx[found] = i;
        found++;
      }
    }
  }
#endif

  // Now access all references in buffer and assign them
  // to a potential reordering list. For each one of these
  // references compute the poc distance compared to current
  // frame.
  idx = 0;
  num_refs = p_Dpb->ref_frames_in_buffer;
  for (i = 0; i < num_refs; i++)
  {
    pFrameStore = p_Dpb->fs_ref[i];
    pField[0]   = pFrameStore->top_field;
    pField[1]   = pFrameStore->bottom_field;
    num_flds    = (currSlice->structure == BOTTOM_FIELD && (p_Enc_Pic->poc == (pField[0]->poc + 1) ) ) ? 1 : 2;

    poc_diff[2*i    ] = 0xFFFF;
    poc_diff[2*i + 1] = 0xFFFF;

    for ( fld = 0; fld < num_flds; fld++ )
    {
      if ( (pFrameStore->is_used & field_used[fld]) && pField[fld]->used_for_reference && !(pField[fld]->is_long_term))
      {
        abs_poc_dist = iabs(pField[fld]->poc - p_Enc_Pic->poc) ;
        poc_diff[idx] = abs_poc_dist;
        re_order[idx] = pField[fld];
        fld_type[idx] = fld + 1;

        if (list_no == LIST_0)
        {
          list_sign[idx] = (p_Enc_Pic->poc < pField[fld]->poc) ? +1 : -1;
        }
        else
        {
          list_sign[idx] = (p_Enc_Pic->poc > pField[fld]->poc) ? +1 : -1;
        }
        idx++;
      }
    }
  }
  num_refs = idx;

  // now sort these references based on poc (temporal) distance
  for (i=0; i < num_refs-1; i++)
  {
    for (j = (i + 1); j < num_refs; j++)
    {
      if (poc_diff[i] > poc_diff[j] || (poc_diff[i] == poc_diff[j] && list_sign[j] > list_sign[i]))
      {
        // poc_diff
        i32_swap (&poc_diff[i], &poc_diff[j]);
        // re_order (PicNum)
        tmp_pic_value   = re_order[i];
        re_order[i] = re_order[j];
        re_order[j] = tmp_pic_value;
        // list_sign
        i32_swap (&list_sign[i], &list_sign[j]);
        // fld_type
        i32_swap (&fld_type[i], &fld_type[j]);
      }
    }
  }

  if (currSlice->structure == TOP_FIELD)
  {
    while ((top_idx < num_refs)||(bot_idx < num_refs))
    {
      for ( ; top_idx < num_refs; top_idx++)
      {
        if ( fld_type[top_idx] == TOP_FIELD )
        {
          tmp_reorder[list_size] = re_order[top_idx];
          list_size++;
          top_idx++;
          break;
        }
      }
      for ( ; bot_idx < num_refs; bot_idx++)
      {
        if ( fld_type[bot_idx] == BOTTOM_FIELD )
        {
          tmp_reorder[list_size] = re_order[bot_idx];
          list_size++;
          bot_idx++;
          break;
        }
      }
    }
  }

  if (currSlice->structure == BOTTOM_FIELD)
  {
    while ((top_idx < num_refs)||(bot_idx < num_refs))
    {
      for ( ; bot_idx < num_refs; bot_idx++)
      {
        if ( fld_type[bot_idx] == BOTTOM_FIELD )
        {
          tmp_reorder[list_size] = re_order[bot_idx];
          list_size++;
          bot_idx++;
          break;
        }
      }
      for ( ; top_idx < num_refs; top_idx++)
      {
        if ( fld_type[top_idx] == TOP_FIELD )
        {
          tmp_reorder[list_size] = re_order[top_idx];
          list_size++;
          top_idx++;
          break;
        }
      }
    }
  }

  j = 0;
  k = 0;
  for (i = 0; i < (int) num_ref_idx_lX_active; i++)
  {
    if ((found > 0) && (i == inter_layer_ref_idx[k]))
    {
      list[i] = inter_layer_ref[k++];
      found--;
    }
    else
    {
      list[i] = tmp_reorder[j++];
    }
  }
}

/*!
************************************************************************
* \brief
*    decide reference picture reordering, Field only
************************************************************************
*/
void poc_ref_pic_reorder_field(Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no)
{
  StorablePicture **list = currSlice->listX[list_no];
  int *modification_of_pic_nums_idc = currSlice->modification_of_pic_nums_idc[list_no]; 
  int *abs_diff_pic_num_minus1 = currSlice->abs_diff_pic_num_minus1[list_no]; 

  VideoParameters *p_Vid = currSlice->p_Vid;
  //InputParameters *p_Inp = currSlice->p_Inp;
  StorablePicture *p_Enc_Pic = p_Vid->enc_picture;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;

  unsigned int i,j,k;

  int currPicNum, picNumLXPred;

  int default_order[32];
  int re_order[32];
  int tmp_reorder[32];
  int list_sign[32];  
  int poc_diff[32];
  int fld_type[32];

  int reorder_stop, no_reorder;
  int tmp_value, diff;

  int abs_poc_dist;
  int maxPicNum;
  unsigned int num_refs;

  int field_used[2] = {1, 2};
  int fld, idx, num_flds;

  unsigned int top_idx = 0;
  unsigned int bot_idx = 0;
  unsigned int list_size = 0;

  StorablePicture *pField[2]; // 0: TOP_FIELD, 1: BOTTOM_FIELD
  FrameStore      *pFrameStore; 

  maxPicNum  = 2 * p_Vid->max_frame_num;
  currPicNum = 2 * p_Vid->frame_num + 1;

  picNumLXPred = currPicNum;

  // First assign default list order.
  for (i=0; i<num_ref_idx_lX_active; i++)
  {
    default_order[i] = list[i]->pic_num;
  }

  // Now access all references in buffer and assign them
  // to a potential reordering list. For each one of these
  // references compute the poc distance compared to current
  // frame.  
  // look for eligible fields
  idx = 0;

  for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
  {
    pFrameStore = p_Dpb->fs_ref[i];
    pField[0]   = pFrameStore->top_field;
    pField[1]   = pFrameStore->bottom_field;
    num_flds    = (currSlice->structure == BOTTOM_FIELD && (p_Enc_Pic->poc == (pField[0]->poc + 1) ) ) ? 1 : 2;

    poc_diff[2*i    ] = 0xFFFF;
    poc_diff[2*i + 1] = 0xFFFF;

    for ( fld = 0; fld < num_flds; fld++ )
    {
#if (MVC_EXTENSION_ENABLE)
      if ( (pFrameStore->is_used & field_used[fld]) && pField[fld]->used_for_reference && !(pField[fld]->is_long_term) 
        && (p_Vid->p_Inp->num_of_views==1 || pField[fld]->view_id==p_Vid->view_id))
#else
      if ( (pFrameStore->is_used & field_used[fld]) && pField[fld]->used_for_reference && !(pField[fld]->is_long_term))
#endif
      {
        abs_poc_dist = iabs(pField[fld]->poc - p_Enc_Pic->poc) ;
        poc_diff[idx] = abs_poc_dist;
        re_order[idx] = pField[fld]->pic_num;
        fld_type[idx] = fld + 1;

        if (list_no == LIST_0)
        {
          list_sign[idx] = (p_Enc_Pic->poc < pField[fld]->poc) ? +1 : -1;
        }
        else
        {
          list_sign[idx] = (p_Enc_Pic->poc > pField[fld]->poc) ? +1 : -1;
        }
        idx++;
      }
    }
  }
  num_refs = idx;

  // now sort these references based on poc (temporal) distance
  for (i=0; i < num_refs-1; i++)
  {
    for (j = (i + 1); j < num_refs; j++)
    {
      if (poc_diff[i] > poc_diff[j] || (poc_diff[i] == poc_diff[j] && list_sign[j] > list_sign[i]))
      {
        // poc_diff
        tmp_value   = poc_diff[i];
        poc_diff[i] = poc_diff[j];
        poc_diff[j] = tmp_value;
        // re_order (PicNum)
        tmp_value   = re_order[i];
        re_order[i] = re_order[j];
        re_order[j] = tmp_value;
        // list_sign
        tmp_value    = list_sign[i];
        list_sign[i] = list_sign[j];
        list_sign[j] = tmp_value;
        // fld_type
        tmp_value   = fld_type[i];
        fld_type[i] = fld_type[j];
        fld_type[j] = tmp_value ;
      }
    }
  }

  if (currSlice->structure == TOP_FIELD)
  {
    while ((top_idx < num_refs)||(bot_idx < num_refs))
    {
      for ( ; top_idx < num_refs; top_idx++)
      {
        if ( fld_type[top_idx] == TOP_FIELD )
        {
          tmp_reorder[list_size] = re_order[top_idx];
          list_size++;
          top_idx++;
          break;
        }
      }
      for ( ; bot_idx < num_refs; bot_idx++)
      {
        if ( fld_type[bot_idx] == BOTTOM_FIELD )
        {
          tmp_reorder[list_size] = re_order[bot_idx];
          list_size++;
          bot_idx++;
          break;
        }
      }
    }
  }
  if (currSlice->structure == BOTTOM_FIELD)
  {
    while ((top_idx < num_refs)||(bot_idx < num_refs))
    {
      for ( ; bot_idx < num_refs; bot_idx++)
      {
        if ( fld_type[bot_idx] == BOTTOM_FIELD )
        {
          tmp_reorder[list_size] = re_order[bot_idx];
          list_size++;
          bot_idx++;
          break;
        }
      }
      for ( ; top_idx < num_refs; top_idx++)
      {
        if ( fld_type[top_idx] == TOP_FIELD )
        {
          tmp_reorder[list_size] = re_order[top_idx];
          list_size++;
          top_idx++;
          break;
        }
      }
    }
  }

  // copy to final matrix
  list_size = imin( list_size, 32 );
  for ( i = 0; i < list_size; i++ )
  {
    re_order[i] = tmp_reorder[i];
  }

  // Check versus default list to see if any
  // change has happened
  no_reorder = 1;
  for (i=0; i<num_ref_idx_lX_active; i++)
  {
    if (default_order[i] != re_order[i])
    {
      no_reorder = 0;
    }
  }

  // If different, then signal reordering
  if (no_reorder == 0)
  {
    for (i=0; i<num_ref_idx_lX_active; i++)
    {
      diff = re_order[i] - picNumLXPred;
      if (diff <= 0)
      {
        modification_of_pic_nums_idc[i] = 0;
        abs_diff_pic_num_minus1[i] = iabs(diff)-1;
        if (abs_diff_pic_num_minus1[i] < 0)
          abs_diff_pic_num_minus1[i] = maxPicNum -1;
      }
      else
      {
        modification_of_pic_nums_idc[i] = 1;
        abs_diff_pic_num_minus1[i] = iabs(diff)-1;
      }
      picNumLXPred = re_order[i];

      tmp_reorder[i] = re_order[i];

      k = i;
      for (j = i; j < num_ref_idx_lX_active; j++)
      {
        if (default_order[j] != re_order[i])
        {
          ++k;
          tmp_reorder[k] = default_order[j];
        }
      }
      reorder_stop = 1;
      for(j=i+1; j<num_ref_idx_lX_active; j++)
      {
        if (tmp_reorder[j] != re_order[j])
        {
          reorder_stop = 0;
          break;
        }
      }

      if (reorder_stop==1)
      {
        ++i;
        break;
      }

      memcpy ( default_order, tmp_reorder, num_ref_idx_lX_active * sizeof(int));
    }
    modification_of_pic_nums_idc[i] = 3;

    memcpy ( default_order, tmp_reorder, num_ref_idx_lX_active * sizeof(int));

    if (list_no==0)
    {
      currSlice->ref_pic_list_reordering_flag[LIST_0] = 1;
    }
    else
    {
      currSlice->ref_pic_list_reordering_flag[LIST_1] = 1;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Decide reference picture reordering based on mse distortion given zero mvs; 
 *    Frame only
 ************************************************************************
 */
void mse_ref_pic_reorder_frame(Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no)
{
  StorablePicture **list = currSlice->listX[list_no];
  VideoParameters *p_Vid = currSlice->p_Vid;
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;
  InputParameters *p_Inp = currSlice->p_Inp;
  //StorablePicture *p_Enc_Pic = p_Vid->enc_picture;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;
  
  int i, j, k, l, m;
  int found = 0, diff;
  int64 iDistortion = 0;

  StorablePicture *re_order[32];
  StorablePicture *inter_layer_ref[32];
  StorablePicture *tmp_pic_value;
  
  int list_sign[32];
  int64 mse_diff[32];
  int inter_layer_ref_idx[32];
  
  int   num_refs = p_Dpb->ref_frames_in_buffer;
  
  //printf("num %d (%d %d)\n", num_refs, p_Vid->ThisPOC, p_Vid->last_valid_reference);
  if (num_refs > 1) // pointless to reorder if only one reference
  {
    // access all references in dpb and extract their distortion information versus the current reference
    for (i = 0; i < num_refs; i++)
    {
      // initialization value
      mse_diff[i] = 0x7FFFFFFFFFFFFFFF;
      if (p_Dpb->fs_ref[i])
        re_order[i] = p_Dpb->fs_ref[i]->frame;
      else
      {
        re_order[i] = NULL;
        //printf("null reference found\n");
        continue;
      }
      
      // open gop test
      if (p_Inp->EnableOpenGOP)
      {
        
        if (p_Dpb->fs_ref[i]->poc < p_Vid->last_valid_reference && p_Vid->ThisPOC >= p_Vid->last_valid_reference)
        {
          //printf("invalid poc %d\n", p_Dpb->fs_ref[i]->poc);
          continue;
        }
      }
      
      
      // following code seems to exclude long term references
      if (p_Dpb->fs_ref[i]->is_used==3 && (p_Dpb->fs_ref[i]->frame->used_for_reference))
      {
        iDistortion = 0;
        if (p_Vid->p_Inp->HMEEnable == 1)
        {
          int ii;
          for (ii = 0; ii < num_refs; ii++)
          {
            if (p_Dpb->fs_ref[i]->frame->poc == pHMEInfo->poc[0][0][ii])
            {
              mse_diff[i] = pHMEInfo->hme_distortion[0][0][ii];
              break;
            }
          }
          //printf("distortion (%d) %lld \n", p_Dpb->fs_ref[i]->poc, mse_diff[i]);
        }
        else
        {
          for (l = 0; l < p_Vid->height; l++)
          {
            for (m = 0; m < p_Vid->width; m++) {
              diff = p_Vid->pCurImg[l][l] - p_Dpb->fs_ref[i]->frame->imgY[l][m];
              iDistortion += diff * diff;
            }
          }
          //abs_poc_dist = iabs(p_Dpb->fs_ref[i]->frame->poc - p_Enc_Pic->poc) ;
          mse_diff[i] = iDistortion;
          list_sign[i] = 0;
        }
        //printf("distortion %d %lld %d\n", i, mse_diff[i], p_Dpb->fs_ref[i]->frame->poc);
      }
    }
    // now sort these references based on distortion
    for (i = 0; i < (num_refs - 1); i++)
    {
      for (j = i + 1; j < num_refs; j++)
      {
        if (mse_diff[i] > mse_diff[j])
        {
          i64_swap(&mse_diff[i], &mse_diff[j]);
          tmp_pic_value = re_order[i];
          re_order[i]   = re_order[j];
          re_order[j]   = tmp_pic_value ;
        }
      }
    }
    if (currSlice->layer_id > 0)
    {
      // This code currently only supports a single inter-layer reference.
      // We need to have a flag in the code that points to how many such references
      // exist and appropriately scan fl_ilref.
      j = 0;
      k = 0;
      for (i = 0; i < (int) num_ref_idx_lX_active+found; i++)
      {
        if ((found > 0) && (i == inter_layer_ref_idx[k]))
        {
          list[i] = inter_layer_ref[k++];
          //printf("values %d %d", (int) list[i],  (int) p_Dpb->fs_ilref[0]->frame);
          found--;
        }
        else
        {
          list[i] = re_order[j++];
        }
      }
    }
    else
    {
      //printf("references : {");
      for (i = 0; i < (int) num_ref_idx_lX_active; i++)
      //for (i = 0; i < (int) num_refs; i++)
      {
        list[i] = re_order[i];
        //printf("(%d)", (int) list[i]->poc);
      }
      //printf("}\n");
    }
  }
}

/*!
************************************************************************
* \brief
*    decide reference picture reordering based on temporal layer, Frame only
************************************************************************
*/
void tlyr_ref_pic_reorder_frame_default(Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no)
{
  StorablePicture **list = currSlice->listX[list_no];
  int *modification_of_pic_nums_idc = currSlice->modification_of_pic_nums_idc[list_no]; 
  int *abs_diff_pic_num_minus1 = currSlice->abs_diff_pic_num_minus1[list_no]; 

  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  StorablePicture *p_Enc_Pic = p_Vid->enc_picture;
  struct decoded_picture_buffer *p_Dpb = currSlice->p_Dpb;

  unsigned int i,j,k;


  int default_order[32];
  int re_order[32];
  int tmp_reorder[32];
  int list_sign[32];
  int reorder_stop, no_reorder;
  int poc_diff[32];

  int temporal_layer[32];

  int diff;

  int abs_poc_dist;
  unsigned int num_refs;

  int tmp_poc_diff, tmp_re_order, tmp_list_sign, tmp_temporal_layer;
  unsigned int beginIdx = 0;
  char newSize = 0;


  int maxPicNum  = p_Vid->max_frame_num;
  int currPicNum = p_Vid->frame_num;

  int picNumLXPred = currPicNum;

  // First assign default list order.
  for (i=0; i<num_ref_idx_lX_active; i++)
  {
    default_order[i] = list[i]->pic_num;
  }

  // Now access all references in buffer and assign them
  // to a potential reordering list. For each one of these
  // references compute the poc distance compared to current
  // frame.
  num_refs = p_Dpb->ref_frames_in_buffer;
  for (i = 0; i < num_refs; i++)
  {
    poc_diff[i] = 0xFFFF;
    re_order[i] = p_Dpb->fs_ref[i]->frame->pic_num;
    
    temporal_layer[i] = p_Dpb->fs_ref[i]->frame->temporal_layer; 

    if (p_Dpb->fs_ref[i]->is_used==3 && (p_Dpb->fs_ref[i]->frame->used_for_reference)&&(!p_Dpb->fs_ref[i]->frame->is_long_term))
    {
      abs_poc_dist = iabs(p_Dpb->fs_ref[i]->frame->poc - p_Enc_Pic->poc) ;
      poc_diff[i] = abs_poc_dist;
      if (list_no == LIST_0)
      {
        list_sign[i] = (p_Enc_Pic->poc < p_Dpb->fs_ref[i]->frame->poc) ? +1 : -1;
      }
      else
      {
        list_sign[i] = (p_Enc_Pic->poc > p_Dpb->fs_ref[i]->frame->poc) ? +1 : -1;
      }
    }
  }

  // first, sort the references based on poc (temporal) distance 
  for (i = 0; i < (num_refs - 1); i++)
  {
    for (j = i + 1; j < num_refs; j++)
    {
      if (list_sign[i] == list_sign[j] && poc_diff[i] > poc_diff[j])
      {
        tmp_poc_diff = poc_diff[i];
        tmp_re_order = re_order[i];
        tmp_list_sign = list_sign[i];
        tmp_temporal_layer = temporal_layer[i];
        poc_diff[i] = poc_diff[j];
        re_order[i] = re_order[j];
        list_sign[i] = list_sign[j];
        temporal_layer[i] = temporal_layer[j];
        poc_diff[j] = tmp_poc_diff;
        re_order[j] = tmp_re_order ;
        list_sign[j] = tmp_list_sign;
        temporal_layer[j] = tmp_temporal_layer;
      }
    }
  }

  // use the closest valid reference frame + 3 temporal layer 0 frames
  for (i = 0; i < num_refs; i++)
  {  
    /*
    if ( (i == 0 && temporal_layer[i] <= p_Enc_Pic->temporal_layer) ||
         (i && !temporal_layer[i]) ) // valid reference pic
    */
    if (temporal_layer[i] <= p_Enc_Pic->temporal_layer) 
    {
      tmp_poc_diff = poc_diff[i];
      tmp_re_order = re_order[i];
      tmp_list_sign = list_sign[i];
      tmp_temporal_layer = temporal_layer[i];
      for (j = i; j > beginIdx; j--) 
      {
        poc_diff[j] = poc_diff[j-1];
        re_order[j] = re_order[j-1];
        list_sign[j] = list_sign[j-1];
        temporal_layer[j] = temporal_layer[j-1];
      }                   
      poc_diff[beginIdx] = tmp_poc_diff;
      re_order[beginIdx] = tmp_re_order;
      list_sign[beginIdx] = tmp_list_sign;
      temporal_layer[beginIdx] = tmp_temporal_layer;
      beginIdx += 1;

      newSize++;
    }
  }
  //currSlice->listXsize[0] = newSize; // update with valid size

  // populate list with selections from the pre-analysis stage
  if ( p_Inp->WPMCPrecision 
    && p_Vid->pWPX->curr_wp_rd_pass->algorithm != WP_REGULAR
    && p_Vid->pWPX->num_wp_ref_list[list_no] )
  {
    for (i=0; i<num_ref_idx_lX_active; i++)
    {
      re_order[i] = p_Vid->pWPX->wp_ref_list[list_no][i].PicNum;
    }
  }

  // Check versus default list to see if any
  // change has happened
  no_reorder = 1;
  for (i=0; i<num_ref_idx_lX_active; i++)
  {
    if (default_order[i] != re_order[i])
    {
      no_reorder = 0;
    }
  }

  // If different, then signal reordering
  if (no_reorder==0)
  {
    for (i=0; i<num_ref_idx_lX_active; i++)
    {
      diff = re_order[i]-picNumLXPred;
      if (diff <= 0)
      {
        modification_of_pic_nums_idc[i] = 0;
        abs_diff_pic_num_minus1[i] = iabs(diff)-1;
        if (abs_diff_pic_num_minus1[i] < 0)
          abs_diff_pic_num_minus1[i] = maxPicNum -1;
      }
      else
      {
        modification_of_pic_nums_idc[i] = 1;
        abs_diff_pic_num_minus1[i] = iabs(diff)-1;
      }
      picNumLXPred = re_order[i];

      tmp_reorder[i] = re_order[i];

      k = i;
      for (j=i; j<num_ref_idx_lX_active; j++)
      {
        if (default_order[j] != re_order[i])
        {
          ++k;
          tmp_reorder[k] = default_order[j];
        }
      }
      reorder_stop = 1;
      for(j=i+1; j<num_ref_idx_lX_active; j++)
      {
        if (tmp_reorder[j] != re_order[j])
        {
          reorder_stop = 0;
          break;
        }
      }

      if (reorder_stop==1)
      {
        ++i;
        break;
      }
      memcpy ( default_order, tmp_reorder, num_ref_idx_lX_active * sizeof(int));

    }
    modification_of_pic_nums_idc[i] = 3;

    memcpy ( default_order, tmp_reorder, num_ref_idx_lX_active * sizeof(int));

    if (list_no==0)
    {
      currSlice->ref_pic_list_reordering_flag[LIST_0] = 1;
    }
    else
    {
      currSlice->ref_pic_list_reordering_flag[LIST_1] = 1;
    }
  }
}


/*!
************************************************************************
* \brief
*    Dummy function for reference list reordering. Used when
*    p_Inp->ReferenceReorder is 0 or if an I/SI slice
************************************************************************
*/
static void reorder_lists_dummy( Slice *currSlice )
{
}

/*!
************************************************************************
* \brief
*    Reorder lists
************************************************************************
*/

void reorder_lists( Slice *currSlice )
{
  InputParameters *p_Inp = currSlice->p_Inp;

  // Perform reordering based on poc distances for HierarchicalCoding
  if ( p_Inp->ReferenceReorder &&
       ((currSlice->slice_type == P_SLICE  || currSlice->slice_type == SP_SLICE) || 
        (p_Inp->UseDistortionReorder == 1 && p_Inp->EnableReorderBslice == 1 && currSlice->slice_type == B_SLICE)) )
  {
    int i, num_ref;

    for (i = 0; i < currSlice->num_ref_idx_active[LIST_0] + 1; i++)
    {
      currSlice->modification_of_pic_nums_idc[LIST_0][i] = 3;
      currSlice->abs_diff_pic_num_minus1[LIST_0][i] = 0;
      currSlice->long_term_pic_idx[LIST_0][i] = 0;
    }

    num_ref = currSlice->num_ref_idx_active[LIST_0];
    if ( p_Inp->ReferenceReorder == 2 ) // temporal layer based
    {
      if ( currSlice->structure == FRAME )
        currSlice->poc_ref_pic_reorder_frame(currSlice, num_ref, LIST_0);
    }
    else
    {
      if ( currSlice->structure == FRAME )
        currSlice->poc_ref_pic_reorder_frame(currSlice, num_ref, LIST_0);
      else
      {
        poc_ref_pic_reorder_field_enh(currSlice, num_ref, LIST_0);
      }
    }

    currSlice->num_ref_idx_active[LIST_0] = currSlice->listXsize[0]; 

    if (p_Inp->ReferenceReorder > 1)
    {
      free_ref_pic_list_reordering_buffer(currSlice);
      alloc_ref_pic_list_reordering_buffer(currSlice);
      reorder_ref_pic_list ( currSlice, LIST_0);
    }
  }
  currSlice->num_ref_idx_active[LIST_0] = currSlice->listXsize[0]; 

#if KEEP_B_SAME_LIST
  if ( currSlice->slice_type == B_SLICE && currSlice->p_Dpb->ref_frames_in_buffer >= 2 )
  {
    int iSendAddtionalRPLR = 0;
    iSendAddtionalRPLR = ( p_Inp->BIdenticalList == 1 && ( (currSlice->ThisPOC/2) % (p_Inp->NumberBFrames+1) ) == 0 ) ||
                         ( p_Inp->BIdenticalList == 2 );

    if ( iSendAddtionalRPLR )
    {
      int i, diff_num, last_frame_num;

      alloc_ref_pic_list_reordering_buffer(currSlice);

      for (i = 0; i < currSlice->num_ref_idx_active[LIST_1] + 1; i++)
      {
        currSlice->modification_of_pic_nums_idc[LIST_1][i] = 3;
        currSlice->abs_diff_pic_num_minus1[LIST_1][i] = 0;
        currSlice->long_term_pic_idx[LIST_1][i] = 0;
      }

      last_frame_num = currSlice->frame_num;
      for ( i=0; i<currSlice->num_ref_idx_active[LIST_1]; i++ )
      {
        diff_num = currSlice->listX[LIST_0][i]->frame_num - last_frame_num;
        last_frame_num = currSlice->listX[LIST_0][i]->frame_num;
        if ( diff_num <= 0 )
        {
          currSlice->modification_of_pic_nums_idc[LIST_1][i] = 0;
          currSlice->abs_diff_pic_num_minus1[LIST_1][i] = iabs(diff_num) - 1;
          if ( currSlice->abs_diff_pic_num_minus1[LIST_1][i] < 0 )
          {
            currSlice->abs_diff_pic_num_minus1[LIST_1][i] = currSlice->max_frame_num - 1;
          }
        }
        else
        {
          currSlice->modification_of_pic_nums_idc[LIST_1][i] = 1;
          currSlice->abs_diff_pic_num_minus1[LIST_1][i] = iabs(diff_num) - 1;
        }
      }

      currSlice->ref_pic_list_reordering_flag[LIST_1] = 1;
      currSlice->num_ref_idx_active[LIST_1] = currSlice->listXsize[1];
      reorder_ref_pic_list ( currSlice, LIST_1);
      currSlice->num_ref_idx_active[LIST_1] = currSlice->listXsize[1];

      return;
    }
  }
#endif

#if CRA
  if ( p_Inp->useCRA )
  {
    if ( currSlice->slice_type == B_SLICE && currSlice->p_Dpb->ref_frames_in_buffer >= 2 )
    {
      if ( ( (currSlice->ThisPOC/2) - (p_Inp->NumberBFrames+1) ) % p_Inp->intra_period == 0 )
      {
        int i, diff_num;
        alloc_ref_pic_list_reordering_buffer(currSlice);
        for (i = 0; i < currSlice->num_ref_idx_active[LIST_1] + 1; i++)
        {
          currSlice->modification_of_pic_nums_idc[LIST_1][i] = 3;
          currSlice->abs_diff_pic_num_minus1[LIST_1][i] = 0;
          currSlice->long_term_pic_idx[LIST_1][i] = 0;
        }

        diff_num = currSlice->listX[LIST_0][0]->frame_num - currSlice->frame_num;
        if ( diff_num <= 0 )
        {
          currSlice->modification_of_pic_nums_idc[LIST_1][0] = 0;
          currSlice->abs_diff_pic_num_minus1[LIST_1][0] = iabs(diff_num) - 1;
          if ( currSlice->abs_diff_pic_num_minus1[LIST_1][0] < 0 )
          {
            currSlice->abs_diff_pic_num_minus1[LIST_1][0] = currSlice->max_frame_num - 1;
          }
        }
        else
        {
          currSlice->modification_of_pic_nums_idc[LIST_1][0] = 1;
          currSlice->abs_diff_pic_num_minus1[LIST_1][0] = iabs(diff_num) - 1;
        }

        currSlice->ref_pic_list_reordering_flag[LIST_1] = 1;
        currSlice->num_ref_idx_active[LIST_1] = currSlice->listXsize[1]; 
        reorder_ref_pic_list ( currSlice, LIST_1);
        currSlice->num_ref_idx_active[LIST_1] = currSlice->listXsize[1]; 
      }
    }
  }
#endif
}

/*!
************************************************************************
* \brief
*    Reorder lists
************************************************************************
*/

void wp_mcprec_reorder_lists( Slice *currSlice )
{

  int i, num_ref;

  wpxModifyRefPicList( currSlice );

  alloc_ref_pic_list_reordering_buffer(currSlice);

  for (i = 0; i < currSlice->num_ref_idx_active[LIST_0] + 1; i++)
  {
    currSlice->modification_of_pic_nums_idc[LIST_0][i] = 3;
    currSlice->abs_diff_pic_num_minus1[LIST_0][i] = 0;
    currSlice->long_term_pic_idx[LIST_0][i] = 0;
  }

  if (currSlice->slice_type == B_SLICE) // type should be part of currSlice not p_Vid
  {
    for (i = 0; i < currSlice->num_ref_idx_active[LIST_1] + 1; i++)
    {
      currSlice->modification_of_pic_nums_idc[LIST_1][i] = 3;
      currSlice->abs_diff_pic_num_minus1[LIST_1][i] = 0;
      currSlice->long_term_pic_idx[LIST_1][i] = 0;
    }
  }

  // LIST_0
  num_ref = currSlice->num_ref_idx_active[LIST_0];

  currSlice->poc_ref_pic_reorder_frame(currSlice, num_ref, LIST_0);
  // reference picture reordering
  reorder_ref_pic_list ( currSlice, LIST_0);

  if ( currSlice->slice_type == B_SLICE )
  {
    // LIST_1
    num_ref = currSlice->num_ref_idx_active[LIST_1];

    currSlice->poc_ref_pic_reorder_frame(currSlice, num_ref, LIST_1);
    // reference picture reordering
    reorder_ref_pic_list ( currSlice, LIST_1);
  }
}

void set_default_ref_pic_lists(Slice *currSlice)
{
  int list, ref;

  for(list = LIST_0; list <= (currSlice->slice_type == P_SLICE ? LIST_0:LIST_1); list++)
  {
    for(ref = 0; ref < currSlice->listXsize[list]; ref++)
    {
      currSlice->default_pic_num[list][ref] = currSlice->listX[list][ref]->pic_num;
#if MVC_EXTENSION_ENABLE
      currSlice->default_view_id[list][ref] = currSlice->listX[list][ref]->view_id;
#endif
    }
  }
}

void reorder_against_default_ref_pic_lists(Slice *currSlice, int cur_list)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  int *modification_of_pic_nums_idc = currSlice->modification_of_pic_nums_idc[cur_list];
  int *abs_diff_pic_num_minus1 = currSlice->abs_diff_pic_num_minus1[cur_list];
#if MVC_EXTENSION_ENABLE
  int *abs_diff_view_idx_minus1 = currSlice->abs_diff_view_idx_minus1[cur_list];
  int picViewIdxLX;
#endif
  int i;

  int maxPicNum, currPicNum, picNumLXNoWrap, picNumLXPred;
  int maxViewIdx =0;
  int currViewIdx = -1;
  int picViewIdxLXPred=-1;
  
  int diff;

  if (p_Vid->structure==FRAME)
  {
    maxPicNum  = p_Vid->max_frame_num;
    currPicNum = p_Vid->frame_num;
  }
  else
  {
    maxPicNum  = 2*p_Vid->max_frame_num;
    currPicNum = 2*p_Vid->frame_num+1;
  }

  currViewIdx = p_Vid->view_id;
  maxViewIdx = 1;
  picViewIdxLXPred=-1;

  picNumLXPred = currPicNum;

  currSlice->ref_pic_list_reordering_flag[cur_list] = 0;
  for (i = 0; i < currSlice->listXsize[cur_list]; i++)
  {
    //printf("list %d %d : %d %d %d\n",currSlice->listXsize[cur_list], cur_list,i,currSlice->default_pic_num[cur_list][i], currSlice->listX[cur_list][i]->pic_num);
    //if(currSlice->default_pic_num[cur_list][i] != currSlice->listX[cur_list][i]->pic_num)
    if(
#if MVC_EXTENSION_ENABLE
      currSlice->default_view_id[cur_list][i] != currSlice->listX[cur_list][i]->view_id || 
#endif
       currSlice->default_pic_num[cur_list][i] != currSlice->listX[cur_list][i]->pic_num)
    {
      currSlice->ref_pic_list_reordering_flag[cur_list] = 1;
      break;
    }
  }

  if(!currSlice->ref_pic_list_reordering_flag[cur_list])
    return;

  // initialization
  for (i = 0; i < currSlice->listXsize[cur_list]+1; i++)
    modification_of_pic_nums_idc[i] = 3;

  for (i=0; i < currSlice->listXsize[cur_list]; i++)
  {
    StorablePicture *ref_pic = currSlice->listX[cur_list][i];
    // inter_view ref reordering
#if MVC_EXTENSION_ENABLE
    if(ref_pic->view_id != currViewIdx)
    {
      diff = 1;
      
      modification_of_pic_nums_idc[i] = 5;
      abs_diff_view_idx_minus1[i] = iabs(diff)-1;
    }
    // temporal ref reordering
    else
#endif
    {
      diff = ref_pic->pic_num-picNumLXPred;
      if (diff <= 0)
      {
        modification_of_pic_nums_idc[i] = 0;
        abs_diff_pic_num_minus1[i] = iabs(diff)-1;
        if (abs_diff_pic_num_minus1[i] < 0)
          abs_diff_pic_num_minus1[i] = maxPicNum-1;
      }
      else
      {
        modification_of_pic_nums_idc[i] = 1;
        abs_diff_pic_num_minus1[i] = iabs(diff)-1;
      }
    }

    if (modification_of_pic_nums_idc[i] == 0 || modification_of_pic_nums_idc[i] == 1)
    {
      if (modification_of_pic_nums_idc[i] == 0)
      {
        if( picNumLXPred - ( abs_diff_pic_num_minus1[i] + 1 ) < 0 )
          picNumLXNoWrap = picNumLXPred - ( abs_diff_pic_num_minus1[i] + 1 ) + maxPicNum;
        else
          picNumLXNoWrap = picNumLXPred - ( abs_diff_pic_num_minus1[i] + 1 );
      }
      else // (modification_of_pic_nums_idc[i] == 1)
      {
        if( picNumLXPred + ( abs_diff_pic_num_minus1[i] + 1 )  >=  maxPicNum )
          picNumLXNoWrap = picNumLXPred + ( abs_diff_pic_num_minus1[i] + 1 ) - maxPicNum;
        else
          picNumLXNoWrap = picNumLXPred + ( abs_diff_pic_num_minus1[i] + 1 );
      }
      picNumLXPred = picNumLXNoWrap;
    }
#if MVC_EXTENSION_ENABLE
    else if (modification_of_pic_nums_idc[i] == 4 || modification_of_pic_nums_idc[i] == 5)
    {
      if(modification_of_pic_nums_idc[i] == 4) 
      {
        picViewIdxLX = picViewIdxLXPred - (abs_diff_view_idx_minus1[i] + 1);
        if( picViewIdxLX <0)
          picViewIdxLX += maxViewIdx;
      }
      else //if(modification_of_pic_nums_idc[i] == 5)
      {
        picViewIdxLX = picViewIdxLXPred + (abs_diff_view_idx_minus1[i] + 1);
        if( picViewIdxLX >= maxViewIdx)
          picViewIdxLX -= maxViewIdx;
      }
      picViewIdxLXPred = picViewIdxLX;
    }
#endif
  }
}

