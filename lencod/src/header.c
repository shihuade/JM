/*!
 *************************************************************************************
 * \file header.c
 *
 * \brief
 *    H.264 Slice and Sequence headers
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Karsten Suehring
 *************************************************************************************
 */

#include <math.h>

#include "global.h"

#include "elements.h"
#include "header.h"
#include "rtp.h"
#include "mbuffer.h"
#include "vlc.h"
#include "parset.h"
#include "list_reorder.h"

// A little trick to avoid those horrible #if TRACE all over the source code
#if TRACE
#define SYMTRACESTRING(s) strncpy(sym.tracestring,s,TRACESTRING_SIZE)
#else
#define SYMTRACESTRING(s) // do nothing
#endif

const int * assignSE2partition[2] ;
const int assignSE2partition_NoDP[SE_MAX_ELEMENTS] =
  {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const int assignSE2partition_DP[SE_MAX_ELEMENTS] =
  // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17
  {  0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 0, 0, 0 } ;

#if (MVC_EXTENSION_ENABLE)
static int mvc_ref_pic_list_reordering(Slice *currSlice, Bitstream *bitstream);
#endif
static int ref_pic_list_reordering(Slice *currSlice, Bitstream *bitstream);
static int pred_weight_table      (Slice *currSlice, Bitstream *bitstream);
static int get_picture_type       (Slice *currSlice);
/*!
 ********************************************************************************************
 * \brief
 *    Write a slice header
 *
 * \return
 *    number of bits used
 ********************************************************************************************
*/
int SliceHeader(Slice* currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  seq_parameter_set_rbsp_t *active_sps = currSlice->active_sps;
  pic_parameter_set_rbsp_t *active_pps = currSlice->active_pps;

  int dP_nr = assignSE2partition[currSlice->partition_mode][SE_HEADER];
  Bitstream *bitstream = currSlice->partArr[dP_nr].bitstream;   
  int len = 0;
  unsigned int field_pic_flag = 0; 
  byte bottom_field_flag = 0;

  int num_bits_slice_group_change_cycle;
  float numtmp;

  if (currSlice->mb_aff_frame_flag)
    len  = write_ue_v("SH: first_mb_in_slice", p_Vid->current_mb_nr >> 1,   bitstream);
  else
    len  = write_ue_v("SH: first_mb_in_slice", p_Vid->current_mb_nr,   bitstream);

  len += write_ue_v("SH: slice_type", get_picture_type (currSlice),   bitstream);

  len += write_ue_v("SH: pic_parameter_set_id" , active_pps->pic_parameter_set_id ,bitstream);

  if( active_sps->separate_colour_plane_flag == 1 )
    len += write_u_v( 2, "SH: colour_plane_id", p_Vid->colour_plane_id, bitstream );

#if (MVC_EXTENSION_ENABLE)
  //if(p_Vid->num_of_layers==2)
  //  len += write_u_v (p_Vid->log2_max_frame_num_minus4 + 4,"SH: frame_num", p_Vid->frame_num/2, bitstream);
  //else
    len += write_u_v (p_Vid->log2_max_frame_num_minus4 + 4,"SH: frame_num", p_Vid->frame_num, bitstream);
#else
  len += write_u_v (p_Vid->log2_max_frame_num_minus4 + 4,"SH: frame_num", p_Vid->frame_num, bitstream);
#endif

  if (!currSlice->active_sps->frame_mbs_only_flag)
  {
    // field_pic_flag    u(1)
    field_pic_flag = (currSlice->structure == TOP_FIELD || currSlice->structure ==BOTTOM_FIELD)?1:0;
    assert( field_pic_flag == p_Vid->fld_flag );
    len += write_u_1("SH: field_pic_flag", field_pic_flag, bitstream);

    if (field_pic_flag)
    {
      //bottom_field_flag     u(1)
      bottom_field_flag = (byte) (currSlice->structure == BOTTOM_FIELD);
      len += write_u_1("SH: bottom_field_flag" , bottom_field_flag ,bitstream);
    }
  }

#if (MVC_EXTENSION_ENABLE)
  if (currSlice->idr_flag || (p_Vid->view_id && !p_Vid->non_idr_flag[currSlice->structure == BOTTOM_FIELD ? 1 : 0]) )
#else
  if (currSlice->idr_flag)
#endif
  {
    // idr_pic_id
    len += write_ue_v ("SH: idr_pic_id", (p_Vid->number & 0x01), bitstream);
  }

  if (p_Vid->pic_order_cnt_type == 0)
  {
    if (active_sps->frame_mbs_only_flag)
    {
      p_Vid->pic_order_cnt_lsb = (p_Vid->toppoc & ~((((unsigned int)(-1)) << (p_Vid->log2_max_pic_order_cnt_lsb_minus4+4))) );
    }
    else
    {
      if (!field_pic_flag || currSlice->structure == TOP_FIELD)
        p_Vid->pic_order_cnt_lsb = (p_Vid->toppoc & ~((((unsigned int)(-1)) << (p_Vid->log2_max_pic_order_cnt_lsb_minus4+4))) );
      else if ( currSlice->structure == BOTTOM_FIELD )
        p_Vid->pic_order_cnt_lsb = (p_Vid->bottompoc & ~((((unsigned int)(-1)) << (p_Vid->log2_max_pic_order_cnt_lsb_minus4+4))) );
    }

    len += write_u_v (p_Vid->log2_max_pic_order_cnt_lsb_minus4+4, "SH: pic_order_cnt_lsb", p_Vid->pic_order_cnt_lsb, bitstream);

    if (p_Vid->bottom_field_pic_order_in_frame_present_flag && !field_pic_flag)
    {
      len += write_se_v ("SH: delta_pic_order_cnt_bottom", p_Vid->delta_pic_order_cnt_bottom, bitstream);
    }
  }
  if (p_Vid->pic_order_cnt_type == 1 && !p_Vid->delta_pic_order_always_zero_flag)
  {
    len += write_se_v ("SH: delta_pic_order_cnt[0]", p_Vid->delta_pic_order_cnt[0], bitstream);

    if (p_Vid->bottom_field_pic_order_in_frame_present_flag && !field_pic_flag)
    {
      len += write_se_v ("SH: delta_pic_order_cnt[1]", p_Vid->delta_pic_order_cnt[1], bitstream);
    }
  }

  if (active_pps->redundant_pic_cnt_present_flag)
  {
    len += write_ue_v ("SH: redundant_pic_cnt", p_Vid->redundant_pic_cnt, bitstream);
  }

  // Direct Mode Type selection for B pictures
  if (currSlice->slice_type == B_SLICE)
  {
    len +=  write_u_1 ("SH: direct_spatial_mv_pred_flag", p_Vid->direct_spatial_mv_pred_flag, bitstream);
  }

  if ((currSlice->slice_type == P_SLICE) || (currSlice->slice_type == B_SLICE) || currSlice->slice_type==SP_SLICE)
  {
    int override_flag;
    if ((currSlice->slice_type == P_SLICE) || (currSlice->slice_type == SP_SLICE))
    {
      override_flag = (currSlice->num_ref_idx_active[LIST_0] != (active_pps->num_ref_idx_l0_default_active_minus1 +1)) ? 1 : 0;
    }
    else
    {
      override_flag = ((currSlice->num_ref_idx_active[LIST_0] != (active_pps->num_ref_idx_l0_default_active_minus1 +1))
                      || (currSlice->num_ref_idx_active[LIST_1] != (active_pps->num_ref_idx_l1_default_active_minus1 +1))) ? 1 : 0;
    }

    len +=  write_u_1 ("SH: num_ref_idx_active_override_flag", override_flag, bitstream);

    if (override_flag)
    {
      len += write_ue_v ("SH: num_ref_idx_l0_active_minus1", currSlice->num_ref_idx_active[LIST_0]-1, bitstream);
      if (currSlice->slice_type == B_SLICE)
      {
        len += write_ue_v ("SH: num_ref_idx_l1_active_minus1", currSlice->num_ref_idx_active[LIST_1]-1, bitstream);
      }
    }

  }
  
  // add reference list modification info if needed
#if (MVC_EXTENSION_ENABLE)
  if(currSlice->layer_id > 0)
    len += mvc_ref_pic_list_reordering(currSlice, bitstream);
  else
#endif
    len += ref_pic_list_reordering(currSlice, bitstream);

  if (((currSlice->slice_type == P_SLICE || currSlice->slice_type == SP_SLICE) && active_pps->weighted_pred_flag) ||
     ((currSlice->slice_type == B_SLICE) && currSlice->weighted_prediction == 1))
  {
    len += pred_weight_table(currSlice, bitstream);
  }

  if (p_Vid->nal_reference_idc)
  {
    p_Vid->adaptive_ref_pic_buffering_flag = (p_Vid->dec_ref_pic_marking_buffer != NULL);
    len += dec_ref_pic_marking(bitstream, 
      p_Vid->dec_ref_pic_marking_buffer, 
#if (MVC_EXTENSION_ENABLE)
      (currSlice->idr_flag || (p_Vid->view_id && !p_Vid->non_idr_flag[currSlice->structure == BOTTOM_FIELD ? 1 : 0]) ) ? 1 : 0,
#else
      currSlice->idr_flag, 
#endif
      p_Vid->no_output_of_prior_pics_flag, 
      p_Vid->long_term_reference_flag );
  }

  if(currSlice->symbol_mode==CABAC && currSlice->slice_type!=I_SLICE /*&& currSlice->slice_type!=SI_IMG*/)
  {
    len += write_ue_v("SH: cabac_init_idc", currSlice->model_number, bitstream);
  }

  len += write_se_v("SH: slice_qp_delta", (currSlice->qp - 26 - active_pps->pic_init_qp_minus26), bitstream);

  if (currSlice->slice_type == SP_SLICE || currSlice->slice_type == SI_SLICE) // SP or SI
  {
    if (currSlice->slice_type == SP_SLICE) // Switch Flag only for SP pictures
    {
      len += write_u_1 ("SH: sp_for_switch_flag", currSlice->sp2_frame_indicator, bitstream);   // 1 for switching SP, 0 for normal SP
    }
    len += write_se_v ("SH: slice_qs_delta", (p_Vid->qpsp - 26), bitstream );
  }

  if (active_pps->deblocking_filter_control_present_flag)
  {
    len += write_ue_v("SH: disable_deblocking_filter_idc",currSlice->DFDisableIdc, bitstream);  // Turn deblocking filter on/off on slice basis

    if (currSlice->DFDisableIdc!=1)
    {
      len += write_se_v ("SH: slice_alpha_c0_offset_div2", currSlice->DFAlphaC0Offset / 2, bitstream);
      len += write_se_v ("SH: slice_beta_offset_div2    ", currSlice->DFBetaOffset / 2, bitstream);
    }
  }

  if ( active_pps->num_slice_groups_minus1 > 0 &&
    active_pps->slice_group_map_type >= 3 && active_pps->slice_group_map_type <= 5)
  {
    numtmp=p_Vid->PicHeightInMapUnits*p_Vid->PicWidthInMbs/(float)(active_pps->slice_group_change_rate_minus1+1)+1;
    num_bits_slice_group_change_cycle = (int)ceil(log(numtmp)/log(2));

    //! p_Vid->slice_group_change_cycle can be changed before calling FmoInit()
    len += write_u_v (num_bits_slice_group_change_cycle, "SH: slice_group_change_cycle", p_Vid->slice_group_change_cycle, bitstream);
  }

  // NOTE: The following syntax element is actually part
  //        Slice data bitstream A RBSP syntax

  if(currSlice->partition_mode && !currSlice->idr_flag)
  {
    len += write_ue_v("DPA: slice_id", currSlice->slice_nr, bitstream);
  }

  return len;
}

/*!
 ********************************************************************************************
 * \brief
 *    writes the ref_pic_list_reordering syntax
 *    based on content of according fields in p_Vid structure
 *
 * \return
 *    number of bits used
 ********************************************************************************************
*/
static int ref_pic_list_reordering(Slice *currSlice, Bitstream *bitstream)
{
  int list; 
  int i, len=0;

  for(list = LIST_0; list <= LIST_1; list++)
  {
    if(currSlice->num_ref_idx_active[list]==0)
      continue;

    reorder_against_default_ref_pic_lists(currSlice, list);

    if (currSlice->slice_type!=I_SLICE && currSlice->slice_type!=SI_SLICE)
    {
      len += write_u_1 ("SH: ref_pic_list_reordering_flag", currSlice->ref_pic_list_reordering_flag[list], bitstream);
      if (currSlice->ref_pic_list_reordering_flag[list])
      {
        i=-1;
        do
        {
          i++;
          len += write_ue_v ("SH: modification_of_pic_nums_idc", currSlice->modification_of_pic_nums_idc[list][i], bitstream);
          if (currSlice->modification_of_pic_nums_idc[list][i] == 0 || currSlice->modification_of_pic_nums_idc[list][i] == 1)
          {
            len += write_ue_v ("SH: abs_diff_pic_num_minus1", currSlice->abs_diff_pic_num_minus1[list][i], bitstream);
          }
          else if (currSlice->modification_of_pic_nums_idc[list][i] == 2)
          {
            len += write_ue_v ("SH: long_term_pic_idx", currSlice->long_term_pic_idx[list][i], bitstream);
          }
        } while (currSlice->modification_of_pic_nums_idc[list][i] != 3);
      }
    }
  }

  return len;
}

#if (MVC_EXTENSION_ENABLE)
/*!
********************************************************************************************
* \brief
*    writes the mvc_ref_pic_list_reordering syntax
*    for both LIST_0 and LIST_1
*
* \return
*    number of bits used
********************************************************************************************
*/

static int mvc_ref_pic_list_reordering(Slice *currSlice, Bitstream *bitstream)
{
  int list; 
  int i, len=0;

  for(list = LIST_0; list <= LIST_1; list++)
  {
    if(currSlice->num_ref_idx_active[list]==0)
      continue;

    reorder_against_default_ref_pic_lists(currSlice, list);

    if (currSlice->slice_type!=I_SLICE && currSlice->slice_type!=SI_SLICE)
    {
      len += write_u_1 ("SH: ref_pic_list_reordering_flag", currSlice->ref_pic_list_reordering_flag[list], bitstream);
      if (currSlice->ref_pic_list_reordering_flag[list])
      {
        i=-1;
        do
        {
          i++;
          len += write_ue_v ("SH: modification_of_pic_nums_idc", currSlice->modification_of_pic_nums_idc[list][i], bitstream);
          if (currSlice->modification_of_pic_nums_idc[list][i] == 0 || currSlice->modification_of_pic_nums_idc[list][i] == 1)
          {
            len += write_ue_v ("SH: abs_diff_pic_num_minus1", currSlice->abs_diff_pic_num_minus1[list][i], bitstream);
          }
          else if (currSlice->modification_of_pic_nums_idc[list][i] == 2)
          {
            len += write_ue_v ("SH: long_term_pic_idx", currSlice->long_term_pic_idx[list][i], bitstream);
          }
          else if (currSlice->modification_of_pic_nums_idc[list][i] == 4 || currSlice->modification_of_pic_nums_idc[list][i] == 5)
          {
            len += write_ue_v ("SH: abs_diff_view_idx_minus1", currSlice->abs_diff_view_idx_minus1[list][i], bitstream);
          }
        } while (currSlice->modification_of_pic_nums_idc[list][i] != 3);
      }
    }
  }

  return len;
}
#endif

/*!
 ************************************************************************
 * \brief
 *    write the memory management control operations
 *
 * \return
 *    number of bits used
 ************************************************************************
 */
int dec_ref_pic_marking(Bitstream *bitstream, DecRefPicMarking_t *p_drpm, int idr_flag, int no_output_of_prior_pics_flag, int long_term_reference_flag )
{
  DecRefPicMarking_t *tmp_drpm;

  int val, len=0;
  int adaptive_ref_pic_buffering_flag;

  if (idr_flag)
  {
    len += write_u_1("SH: no_output_of_prior_pics_flag", no_output_of_prior_pics_flag, bitstream);
    len += write_u_1("SH: long_term_reference_flag", long_term_reference_flag, bitstream);
  }
  else
  {
    adaptive_ref_pic_buffering_flag = (p_drpm != NULL);

    len += write_u_1("SH: adaptive_ref_pic_buffering_flag", adaptive_ref_pic_buffering_flag, bitstream);

    if (adaptive_ref_pic_buffering_flag)
    {
      tmp_drpm = p_drpm;
      // write Memory Management Control Operation
      do
      {
        if (tmp_drpm==NULL) error ("Error encoding MMCO commands", 500);

        val = tmp_drpm->memory_management_control_operation;
        len += write_ue_v("SH: memory_management_control_operation", val, bitstream);

        if ((val==1)||(val==3))
        {
          len += 1 + write_ue_v("SH: difference_of_pic_nums_minus1", tmp_drpm->difference_of_pic_nums_minus1, bitstream);
        }
        if (val==2)
        {
          len+= write_ue_v("SH: long_term_pic_num", tmp_drpm->long_term_pic_num, bitstream);
        }
        if ((val==3)||(val==6))
        {
          len+= write_ue_v("SH: long_term_frame_idx", tmp_drpm->long_term_frame_idx, bitstream);
        }
        if (val==4)
        {
          len += write_ue_v("SH: max_long_term_pic_idx_plus1", tmp_drpm->max_long_term_frame_idx_plus1, bitstream);
        }

        tmp_drpm=tmp_drpm->Next;

      } while (val != 0);

    }
  }
  return len;
}


/*!
 ************************************************************************
 * \brief
 *    write prediction weight table
 *
 * \return
 *    number of bits used
 ************************************************************************
 */
static int pred_weight_table(Slice *currSlice, Bitstream *bitstream)
{
  int len = 0;
  int i,j;

  len += write_ue_v("SH: luma_log_weight_denom", currSlice->luma_log_weight_denom, bitstream);

  if ( 0 != currSlice->active_sps->chroma_format_idc)
  {
    len += write_ue_v("SH: chroma_log_weight_denom", currSlice->chroma_log_weight_denom, bitstream);
  }

  for (i=0; i< currSlice->num_ref_idx_active[LIST_0]; i++)
  {
    if ( (currSlice->wp_weight[0][i][0] != 1<<currSlice->luma_log_weight_denom) || (currSlice->wp_offset[0][i][0] != 0) )
    {
      len += write_u_1  ("SH: luma_weight_flag_l0", 1, bitstream);
      len += write_se_v ("SH: luma_weight_l0", currSlice->wp_weight[0][i][0], bitstream);
      len += write_se_v ("SH: luma_offset_l0", (currSlice->wp_offset[0][i][0]>>(currSlice->bitdepth_luma - 8)), bitstream);
    }
    else
    {
      len += write_u_1  ("SH: luma_weight_flag_l0", 0, bitstream);
    }

    if (currSlice->active_sps->chroma_format_idc!=0)
    {
      if ( (currSlice->wp_weight[0][i][1] != 1<<currSlice->chroma_log_weight_denom) || (currSlice->wp_offset[0][i][1] != 0) ||
        (currSlice->wp_weight[0][i][2] != 1<<currSlice->chroma_log_weight_denom) || (currSlice->wp_offset[0][i][2] != 0)  )
      {
        len += write_u_1 ("chroma_weight_flag_l0", 1, bitstream);
        for (j=1; j<3; j++)
        {
          len += write_se_v ("chroma_weight_l0", currSlice->wp_weight[0][i][j] ,bitstream);
          len += write_se_v ("chroma_offset_l0", (currSlice->wp_offset[0][i][j]>>(currSlice->bitdepth_chroma-8)) ,bitstream);
        }
      }
      else
      {
        len += write_u_1 ("chroma_weight_flag_l0", 0, bitstream);
      }
    }
  }

  if (currSlice->slice_type == B_SLICE)
  {
    for (i=0; i< currSlice->num_ref_idx_active[LIST_1]; i++)
    {
      if ( (currSlice->wp_weight[1][i][0] != 1<<currSlice->luma_log_weight_denom) || (currSlice->wp_offset[1][i][0] != 0) )
      {
        len += write_u_1  ("SH: luma_weight_flag_l1", 1, bitstream);
        len += write_se_v ("SH: luma_weight_l1", currSlice->wp_weight[1][i][0], bitstream);
        len += write_se_v ("SH: luma_offset_l1", (currSlice->wp_offset[1][i][0]>>(currSlice->bitdepth_luma-8)), bitstream);
      }
      else
      {
        len += write_u_1  ("SH: luma_weight_flag_l1", 0, bitstream);
      }

      if (currSlice->active_sps->chroma_format_idc!=0)
      {
        if ( (currSlice->wp_weight[1][i][1] != 1<<currSlice->chroma_log_weight_denom) || (currSlice->wp_offset[1][i][1] != 0) ||
          (currSlice->wp_weight[1][i][2] != 1<<currSlice->chroma_log_weight_denom) || (currSlice->wp_offset[1][i][2] != 0) )
        {
          len += write_u_1 ("chroma_weight_flag_l1", 1, bitstream);
          for (j=1; j<3; j++)
          {
            len += write_se_v ("chroma_weight_l1", currSlice->wp_weight[1][i][j] ,bitstream);
            len += write_se_v ("chroma_offset_l1", (currSlice->wp_offset[1][i][j]>>(currSlice->bitdepth_chroma-8)) ,bitstream);
          }
        }
        else
        {
          len += write_u_1 ("chroma_weight_flag_l1", 0, bitstream);
        }
      }
    }
  }
  return len;
}


/*!
 ************************************************************************
 * \brief
 *    Selects picture type and codes it to symbol
 *
 * \return
 *    symbol value for picture type
 ************************************************************************
 */
static int get_picture_type(Slice *currSlice)
{
  // set this value to zero for transmission without signaling
  // that the whole picture has the same slice type
  int same_slicetype_for_whole_frame = 5;

#if (MVC_EXTENSION_ENABLE)
  if(currSlice->p_Inp->num_of_views==2)
  {
    same_slicetype_for_whole_frame = 0;
  }
#endif

  switch (currSlice->slice_type)
  {
  case I_SLICE:
    return 2 + same_slicetype_for_whole_frame;
    break;
  case P_SLICE:
    return 0 + same_slicetype_for_whole_frame;
    break;
  case B_SLICE:
    return 1 + same_slicetype_for_whole_frame;
    break;
  case SP_SLICE:
    return 3 + same_slicetype_for_whole_frame;
    break;
  case SI_SLICE:
    return 4 + same_slicetype_for_whole_frame;
    break;
  default:
    error("Picture Type not supported!",1);
    break;
  }

  return 0;
}



/*!
 *****************************************************************************
 *
 * \brief
 *    int Partition_BC_Header () write the Partition type B, C header
 *
 * \return
 *    Number of bits used by the partition header
 *
 * \par Parameters
 *    PartNo: Partition Number to which the header should be written
 *
 * \par Side effects
 *    Partition header as per VCEG-N72r2 is written into the appropriate
 *    partition bit buffer
 *
 * \par Limitations/Shortcomings/Tweaks
 *    The current code does not support the change of picture parameters within
 *    one coded sequence, hence there is only one parameter set necessary.  This
 *    is hard coded to zero.
 *
 * \date
 *    October 24, 2001
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 *****************************************************************************/
int Partition_BC_Header(Slice *currSlice, int PartNo)
{
  DataPartition *partition = &((currSlice)->partArr[PartNo]);
  SyntaxElement sym;

  assert (PartNo > 0 && PartNo < currSlice->max_part_nr);

  sym.type = SE_HEADER;         // This will be true for all symbols generated here
  sym.value2  = 0;

  SYMTRACESTRING("RTP-PH: Slice ID");
  sym.value1 = currSlice->slice_nr;
  writeSE_UVLC (&sym, partition);

  return sym.len;
}

