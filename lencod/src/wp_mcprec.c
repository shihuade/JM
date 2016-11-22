
/*!
*************************************************************************************
* \file wp_mcprec.c
*
* \brief
*    Improved Motion Compensation Precision Scheme using Weighted Prediction
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*     - Athanasios Leontaris            <aleon@dolby.com>
*     - Alexis Michael Tourapis         <alexismt@ieee.org>
*************************************************************************************
*/
#include "contributors.h"

#include "global.h"
#include "image.h"
#include "slice.h"
#include "list_reorder.h"
#include "wp_mcprec.h"
#include "memalloc.h"

/*!
************************************************************************
* \brief
*    Initialize WPR object structure
************************************************************************
*/

void wpxInitWPXObject( VideoParameters *p_Vid )
{
  p_Vid->pWPX = (WPXObject *)malloc( sizeof( WPXObject ) );
  if ( p_Vid->pWPX == NULL )
  {
    fprintf( stderr, "\n Error initializing memory for WPXObject. Exiting...\n" );
    exit(1);
  }

  // wp_ref_l0
  if ((p_Vid->pWPX->wp_ref_list[LIST_0] = (WeightedPredRefX *) calloc(MAX_REFERENCE_PICTURES, sizeof(WeightedPredRefX))) == NULL)
    no_mem_exit("wpxInitWPXObject: p_Vid->pWPX->wp_ref_list[0]");
  // wp_ref_l1
  if ((p_Vid->pWPX->wp_ref_list[LIST_1] = (WeightedPredRefX *) calloc(MAX_REFERENCE_PICTURES, sizeof(WeightedPredRefX))) == NULL)
    no_mem_exit("wpxInitWPXObject: p_Vid->pWPX->wp_ref_list[1]");
}

/*!
************************************************************************
* \brief
*    Free WPR object structure
************************************************************************
*/

void wpxFreeWPXObject( VideoParameters *p_Vid )
{
  // wp_ref_l0
  if (p_Vid->pWPX->wp_ref_list[LIST_0])
    free(p_Vid->pWPX->wp_ref_list[LIST_0]);
  p_Vid->pWPX->wp_ref_list[LIST_0] = NULL;
  // wp_ref_l1
  if (p_Vid->pWPX->wp_ref_list[LIST_1])
    free(p_Vid->pWPX->wp_ref_list[LIST_1]);
  p_Vid->pWPX->wp_ref_list[LIST_1] = NULL;

  if ( p_Vid->pWPX != NULL )
  {
    free( p_Vid->pWPX );
  }
}

/*!
************************************************************************
* \brief
*    Initialize WPR coding passes
************************************************************************
*/

void wpxInitWPXPasses( VideoParameters *p_Vid, InputParameters *p_Inp )
{
  // initialize number of wp reference frames in each list
  p_Vid->pWPX->num_wp_ref_list[LIST_0] = 0;
  p_Vid->pWPX->num_wp_ref_list[LIST_1] = 0;

  switch( p_Inp->WPMCPrecision )
  {
  default:
  case 0:
    break;
  case 1:
    p_Vid->pWPX->wp_rd_passes[0].algorithm = WP_REGULAR;
    p_Vid->pWPX->wp_rd_passes[1].algorithm = WP_MCPREC_MINUS0;      
    break;
  case 2:
    p_Vid->pWPX->wp_rd_passes[0].algorithm = WP_REGULAR;
    p_Vid->pWPX->wp_rd_passes[1].algorithm = WP_MCPREC_MINUS0;
    p_Vid->pWPX->wp_rd_passes[2].algorithm = WP_MCPREC_MINUS1;      
    break;
  }
}

/*!
************************************************************************
* \brief
*    Modifies ref_pic_list for all lists
************************************************************************
*/

void wpxModifyRefPicList( Slice *currSlice )
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  unsigned int i, j, cloned_refs;
  int   default_order_list0[32];
  int   default_order_list1[32];
  int   re_order[32], *list_order;  
  int          pred_list;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;

  StorablePicture **list;

  // set memory
  memset( (void *)default_order_list0, 1<<20, 32 * sizeof( int ) );
  memset( (void *)default_order_list1, 1<<20, 32 * sizeof( int ) );
  memset( (void *)re_order,            1<<20, 32 * sizeof( int ) );

  // First assign default list orders
  list = currSlice->listX[LIST_0];
  for (i=0; i<(unsigned int)(currSlice->num_ref_idx_active[LIST_0]); i++)
  {
    default_order_list0[i] = list[i]->pic_num;
  }
  if ( currSlice->slice_type == B_SLICE )
  {
    list = currSlice->listX[LIST_1];
    for (i=0; i<(unsigned int)(currSlice->num_ref_idx_active[LIST_1]); i++)
    {
      default_order_list1[i] = list[i]->pic_num;
    }
  }

  // obtain the ref_pic_list using POC-based reordering if currSlice->slice_type != B_SLICE
  if ( currSlice->slice_type == P_SLICE )
  {
    int  list_sign[32];
    int  poc_diff[32];
    int  tmp_value;
    int  abs_poc_dist;
    
    for (i=0; i < p_Dpb->ref_frames_in_buffer; i++)
    {
      re_order[i] = p_Dpb->fs_ref[i]->frame->pic_num;
      if (p_Dpb->fs_ref[i]->is_used==3 && (p_Dpb->fs_ref[i]->frame->used_for_reference)&&(!p_Dpb->fs_ref[i]->frame->is_long_term))
      {
        abs_poc_dist = iabs(p_Dpb->fs_ref[i]->frame->poc - p_Vid->enc_picture->poc) ;
        poc_diff[i]  = abs_poc_dist;
        list_sign[i] = (p_Vid->enc_picture->poc < p_Dpb->fs_ref[i]->frame->poc) ? +1 : -1;
      }
    }

    // sort these references based on poc (temporal) distance
    for (i=0; i< p_Dpb->ref_frames_in_buffer-1; i++)
    {
      for (j=i+1; j< p_Dpb->ref_frames_in_buffer; j++)
      {
        if (poc_diff[i]>poc_diff[j] || (poc_diff[i] == poc_diff[j] && list_sign[j] > list_sign[i]))
        {
          tmp_value = poc_diff[i];
          poc_diff[i] = poc_diff[j];
          poc_diff[j] = tmp_value;
          tmp_value  = re_order[i];
          re_order[i] = re_order[j];
          re_order[j] = tmp_value ;
          tmp_value  = list_sign[i];
          list_sign[i] = list_sign[j];
          list_sign[j] = tmp_value ;
        }
      }
    }
  }
  // end of POC-based reordering of P_SLICE

  // loop over two lists
  for ( pred_list = 0; pred_list < (1 + ( currSlice->slice_type == B_SLICE ? 1 : 0 )); pred_list++ )
  {
    if ( pred_list == LIST_0 )
    {
      list_order = (currSlice->slice_type == P_SLICE) ? re_order : default_order_list0;
    }
    else
    {
      list_order = default_order_list1;
    }

    // check algorithms (more flexibility for the future...)
    switch( p_Vid->pWPX->curr_wp_rd_pass->algorithm )
    {
    default:
    case WP_MCPREC_PLUS0:
    case WP_MCPREC_PLUS1:
    case WP_MCPREC_MINUS0:
    case WP_MCPREC_MINUS1:
      // ref 0 and 1 point to to the same reference
      cloned_refs = p_Vid->pWPX->num_wp_ref_list[pred_list] = 2;
      for ( j = 0; j < cloned_refs; j++ )
      {
        p_Vid->pWPX->wp_ref_list[pred_list][j].PicNum = list_order[0];
      }
      // shift the rest
      for ( j = cloned_refs; j < p_Dpb->ref_frames_in_buffer; j++ )
      {
        p_Vid->pWPX->wp_ref_list[pred_list][j].PicNum = list_order[j - (cloned_refs - 1)];
        p_Vid->pWPX->num_wp_ref_list[pred_list]++;
      }
      break;
    case WP_MCPREC_MINUS_PLUS0:
      // ref 0 and 1 point to to the same reference
      cloned_refs = p_Vid->pWPX->num_wp_ref_list[pred_list] = 3;
      for ( j = 0; j < cloned_refs; j++ )
      {
        p_Vid->pWPX->wp_ref_list[pred_list][j].PicNum = list_order[0];
      }
      // shift the rest
      for ( j = cloned_refs; j < p_Dpb->ref_frames_in_buffer; j++ )
      {
        p_Vid->pWPX->wp_ref_list[pred_list][j].PicNum = list_order[j - (cloned_refs - 1)];
        p_Vid->pWPX->num_wp_ref_list[pred_list]++;
      }      
      break;
    }
    // constrain list length
    p_Vid->pWPX->num_wp_ref_list[pred_list] = imin( p_Vid->pWPX->num_wp_ref_list[pred_list],
      ( pred_list == LIST_0 ) ? currSlice->num_ref_idx_active[LIST_0] : currSlice->num_ref_idx_active[LIST_1] );
  }
}

/*!
************************************************************************
* \brief
*    Determine whether it is fine to determine WP parameters
************************************************************************
*/

int wpxDetermineWP( Slice *currSlice, int clist, int n )
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int i, j, determine_wp = 0;
  short default_weight[3];
  // we assume it's the same as in the WP functions
  short luma_log_weight_denom   = 5;
  short chroma_log_weight_denom = 5;
  int cur_list = LIST_0;

  default_weight[0] = 1 << luma_log_weight_denom;
  default_weight[1] = 1 << chroma_log_weight_denom;
  default_weight[2] = 1 << chroma_log_weight_denom;

  if ( !p_Inp->WPMCPrecision )
  {
    determine_wp = 1;
  }
  else
  {
    determine_wp = 0;
    // check slice type
    if ( currSlice->slice_type == P_SLICE )
    {
      for (i = 0; i < 3; i++)
      {
        currSlice->wp_weight[clist][n][i] = default_weight[i];
      }      
    }
    else if ( currSlice->slice_type == B_SLICE )
    {
      cur_list = ((clist & 0x01) ==  LIST_1) ? LIST_0 : LIST_1;

      // single-list prediction
      for (i = 0; i < 3; i++)
      {
        currSlice->wp_weight[clist][n][i] = default_weight[i];
      }
      // bi-pred
      for (j = 0; j < currSlice->listXsize[cur_list]; j++)
      {
        for (i = 0; i < 3; i++)
          currSlice->wbp_weight[clist][n][j][i] = default_weight[i];
      }      
    }
    // algorithm consideration
    switch( p_Vid->pWPX->curr_wp_rd_pass->algorithm )
    {
    case WP_MCPREC_PLUS0:
      for (i = 0; i < 3; i++)
        currSlice->wp_offset[clist][n][i] = (n == 1) ? 1 : 0;
      break;
    case WP_MCPREC_MINUS0:
      for (i = 0; i < 3; i++)
        currSlice->wp_offset[clist][n][i] = (n == 1) ? -1 : 0;
      break;
    case WP_MCPREC_PLUS1:
      for (i = 0; i < 3; i++)
        currSlice->wp_offset[clist][n][i] = (n == 0) ? 1 : 0;
      break;
    case WP_MCPREC_MINUS1:
      for (i = 0; i < 3; i++)
        currSlice->wp_offset[clist][n][i] = (n == 0) ? -1 : 0;
      break;
    case WP_MCPREC_MINUS_PLUS0:
      for (i = 0; i < 3; i++)
        currSlice->wp_offset[clist][n][i] = (n == 1) ? -1 : ((n == 2) ? 1 : 0);
      break;
    default:
    case WP_REGULAR:
      determine_wp = 1;
      break;
    }    
    // check list (play with the WP factors)
    if ( currSlice->slice_type == B_SLICE && cur_list == LIST_0 )
    {
      for (i = 0; i < 3; i++)
        currSlice->wp_offset[clist][n][i] *= 2;
    }
    // zero out chroma offsets
    for (i = 1; i < 3; i++)
    {
      currSlice->wp_offset[clist][n][i] = 0;
    }
  }

  return determine_wp;
}

/*!
************************************************************************
* \brief
*    Modify number of references
************************************************************************
*/

void wpxAdaptRefNum( Slice *currSlice )
{
  InputParameters *p_Inp = currSlice->p_Inp;
  VideoParameters *p_Vid = currSlice->p_Vid;
#if (MVC_EXTENSION_ENABLE)
  int view_id = p_Vid->view_id;
#else
  int view_id = 0;
#endif

  if ( p_Vid->pWPX->curr_wp_rd_pass->algorithm == WP_REGULAR )
  {
    switch( currSlice->slice_type )
    {
    default:
    case I_SLICE:
      break;
    case P_SLICE:
      if ( currSlice->num_ref_idx_active[LIST_0] == ( p_Inp->P_List0_refs[view_id] * ((currSlice->structure !=0) + 1) ) )
      {
        currSlice->listXsize[LIST_0] = currSlice->num_ref_idx_active[LIST_0] = (char) imax( ((currSlice->structure !=0) + 1), currSlice->num_ref_idx_active[LIST_0] - ((currSlice->structure !=0) + 1) );
      }
      break;
    case B_SLICE:
      if ( currSlice->num_ref_idx_active[LIST_0] == ( p_Inp->B_List0_refs[view_id] * ((currSlice->structure !=0) + 1) ) )
      {
        currSlice->listXsize[LIST_0] = currSlice->num_ref_idx_active[LIST_0] = (char) imax( ((currSlice->structure !=0) + 1), currSlice->num_ref_idx_active[LIST_0] - ((currSlice->structure !=0) + 1) );
      }
      if ( currSlice->num_ref_idx_active[LIST_1] == ( p_Inp->B_List1_refs[view_id] * ((currSlice->structure !=0) + 1) ) )
      {
        currSlice->listXsize[LIST_1] = currSlice->num_ref_idx_active[LIST_1] = (char) imax( ((currSlice->structure !=0) + 1), currSlice->num_ref_idx_active[LIST_1] - ((currSlice->structure !=0) + 1) );
      }
      break;
    }    
  }
}
