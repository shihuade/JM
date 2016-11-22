
/*!
*************************************************************************************
* \file wp_periodic.c
*
* \brief
*    Generate periodic weights for WP
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*     - Alexis Michael Tourapis         <alexismt@ieee.org>
*************************************************************************************
*/
#include "contributors.h"

#include "global.h"
#include "image.h"
#include "wp_periodic.h"
#include "wp.h"
#include <math.h>

#define PI 3.14159265

/*!
************************************************************************
* \brief
*    Estimates reference picture weighting factors for P slices
************************************************************************
*/

void EstimateWPPSlicePeriodic(Slice *currSlice, int select_offset)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;

  int n;
  short default_weight[3];
  int list_offset   = ((currSlice->mb_aff_frame_flag) && (p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  int clist;
  int start_mb, end_mb; 
  int comp;
  int bitdepth[3] = { (p_Vid->bitdepth_luma - 8), (p_Vid->bitdepth_chroma - 8), (p_Vid->bitdepth_chroma - 8)};

  short weight[2][MAX_REFERENCE_PICTURES][3];
  short offset[2][MAX_REFERENCE_PICTURES][3];

  int cur_slice;
  int cur_weight, cur_offset;

  currSlice->luma_log_weight_denom   = 5;
  currSlice->chroma_log_weight_denom = 5;

  currSlice->wp_luma_round           = 1 << (currSlice->luma_log_weight_denom - 1);
  currSlice->wp_chroma_round         = 1 << (currSlice->chroma_log_weight_denom - 1);
  default_weight[0]       = 1 << currSlice->luma_log_weight_denom;
  default_weight[1]       = default_weight[2] = 1 << currSlice->chroma_log_weight_denom;


  if(p_Inp->slice_mode == 1)
  {
    cur_slice = p_Vid->current_mb_nr / p_Inp->slice_argument; 
  }
  else
    cur_slice = 0;

 
  if(p_Inp->slice_mode == 1)
  {
    start_mb = cur_slice * p_Inp->slice_argument;
    end_mb = start_mb + p_Inp->slice_argument;
    if(end_mb > (int)p_Vid->FrameSizeInMbs)
      end_mb = p_Vid->FrameSizeInMbs;
  }
  else
  {
    start_mb = 0;
    end_mb = p_Vid->FrameSizeInMbs;
  }

  for (clist = 0; clist < 2 + list_offset; clist++)
  {
    for (n = 0; n < currSlice->listXsize[clist]; n++)
    {          
      for (comp=0; comp < 3; comp ++)
      {
        cur_weight = (int) ((127.0 * cos (p_Vid->number *PI / 30)) + (4.0 * (((float) rand()) / RAND_MAX)) - 4.0);
        weight[clist][n][comp] = sClip3(-128, 127, cur_weight);
        cur_offset = (int) ((127.0 * cos ((p_Vid->number + 15) * PI / 45)) + (4.0 * (((float) rand()) / RAND_MAX)) - 4.0);
        offset[clist][n][comp] = (sClip3(-128, 127, cur_offset) << bitdepth[comp]);
      }
    }
  }

  for (clist = 0; clist < 2 + list_offset; clist++)
  {
    for (n = 0; n < currSlice->listXsize[clist]; n++)
    {
      for (comp=0; comp < 3; comp ++)
      {
        currSlice->wp_weight[clist][n][comp] = weight[clist][n][comp];
        currSlice->wp_offset[clist][n][comp] = offset[clist][n][comp];
        if(p_Vid->wp_weights)
          p_Vid->wp_weights[comp][clist][n][cur_slice] = weight[clist][n][comp];
        if(p_Vid->wp_offsets)
          p_Vid->wp_offsets[comp][clist][n][cur_slice] = offset[clist][n][comp];
      }
#if DEBUG_WP 
      for(comp = 0; comp < 3; comp++)
        printf("slice %d: index %d component %d weight %d offset %d\n", cur_slice, n,comp,currSlice->wp_weight[clist][n][comp],currSlice->wp_offset[clist][n][comp]);
#endif
    }
  }
}

/*!
************************************************************************
* \brief
*    Estimates reference picture weighting factors for B slices
************************************************************************
*/
void EstimateWPBSlicePeriodic(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int i, j, n;

  int comp;
  int k;

  short default_weight[3];
  int list_offset   = ((currSlice->mb_aff_frame_flag) && (p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  int clist;
  int cur_slice = 0;
  int cur_weight, cur_offset;
  int bitdepth[3] = { (p_Vid->bitdepth_luma - 8), (p_Vid->bitdepth_chroma - 8), (p_Vid->bitdepth_chroma - 8)};

  if (p_Vid->active_pps->weighted_bipred_idc == 2) //! implicit mode. Values are fixed and it is important to show it here
  {
    currSlice->luma_log_weight_denom = 5;
    currSlice->chroma_log_weight_denom = 5;
  }
  else                                     //! explicit mode. Values can be changed for higher precision.
  {
    currSlice->luma_log_weight_denom = 5;
    currSlice->chroma_log_weight_denom = 5;
  }

  currSlice->wp_luma_round   = 1 << (currSlice->luma_log_weight_denom - 1);
  currSlice->wp_chroma_round = 1 << (currSlice->chroma_log_weight_denom - 1);
  default_weight[0] = 1 << currSlice->luma_log_weight_denom;
  default_weight[1] = 1 << currSlice->chroma_log_weight_denom;
  default_weight[2] = 1 << currSlice->chroma_log_weight_denom;


  if(p_Inp->slice_mode == 1)
  {
    cur_slice = p_Vid->current_mb_nr / p_Inp->slice_argument ; 
  }
  else
    cur_slice = 0;

  if (p_Vid->active_pps->weighted_bipred_idc == 2) //! implicit mode
  {
    short im_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3];

    ComputeImplicitWeights(currSlice, default_weight, im_weight);

    for (k = 0; k < 2; k++)
    {
      for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
      {
        for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
        {
          for (comp = 0; comp < 3; comp++)
          {
            currSlice->wbp_weight[k][i][j][comp] = im_weight[k][i][j][comp];
          }
        }
      }
    }

    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (i = 0; i < currSlice->listXsize[clist]; i++)
      {
        for (comp = 0; comp < 3; comp++)
        {
          currSlice->wp_weight[clist][i][comp] = default_weight[comp];
          currSlice->wp_offset[clist][i][comp] = 0;
        }
      }
    }
  }
  else  // explicit mode
  {
    int start_mb, end_mb;

    int comp;

    short wp_weight[6][MAX_REFERENCE_PICTURES][3];
    short wp_offset[6][MAX_REFERENCE_PICTURES][3];

    if(p_Inp->slice_mode == 1)
    {
      start_mb = cur_slice * p_Inp->slice_argument;
      end_mb = start_mb + p_Inp->slice_argument;
      if(end_mb > (int)p_Vid->FrameSizeInMbs)
        end_mb = p_Vid->FrameSizeInMbs;
    }
    else
    {
      start_mb = 0;
      end_mb = p_Vid->FrameSizeInMbs;
    }

    for (clist = 0; clist < 2 + list_offset; clist++)
    {
      for (n = 0; n < currSlice->listXsize[clist]; n++)
      {          
        for (comp = 0; comp < 3; comp ++)
        {
          cur_weight = (int) ((127.0 * cos (p_Vid->number *PI / 30)) + (4.0 * (((float) rand()) / RAND_MAX)) - 4.0);
          wp_weight[clist][n][comp] = sClip3(-128, 127, cur_weight);
          cur_offset = (int) ((127.0 * cos ((p_Vid->number + 45) * PI / 45)) + (4.0 * (((float) rand()) / RAND_MAX)) - 4.0);
          wp_offset[clist][n][comp] = (sClip3(-128, 127, cur_offset) << bitdepth[comp]);
          /*
          printf("(%d %d %d) weights %d %d %d %d %d %d\n", 
            clist, n, comp,
            cur_offset, wp_offset[clist][n][comp], bitdepth[comp],
            cur_weight, wp_weight[clist][n][comp], bitdepth[comp]);
            */
        }
      }
    }

    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (n = 0; n < currSlice->listXsize[clist]; n++)
      {
        for (comp=0; comp < 3; comp ++)
        {
          currSlice->wp_weight[clist][n][comp] = wp_weight[clist][n][comp];
          currSlice->wp_offset[clist][n][comp] = wp_offset[clist][n][comp];
          if(p_Vid->wp_weights)
            p_Vid->wp_weights[comp][clist][n][cur_slice] = wp_weight[clist][n][comp];
          if(p_Vid->wp_offsets)
            p_Vid->wp_offsets[comp][clist][n][cur_slice] = wp_offset[clist][n][comp];
          /*
          printf("(%d %d %d) single weights %d %d\n", 
            clist, n, comp,
            currSlice->wp_weight[clist][n][comp],
            currSlice->wp_offset[clist][n][comp]);
            */
        }
      }
    }

    for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
    {
      for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
      {
        for (comp = 0; comp < 3; comp++)
        {
          currSlice->wbp_weight[0][i][j][comp] = currSlice->wp_weight[0][i][comp];
          currSlice->wbp_weight[1][i][j][comp] = currSlice->wp_weight[1][j][comp];
          /*
          printf("(%d %d %d) bi weights %d %d\n", 
            i, j, comp,
            currSlice->wbp_weight[0][i][j][comp],
            currSlice->wbp_weight[1][i][j][comp]);
            */
        }
      }
    }
  }    
}


/*!
************************************************************************
* \brief
*    Tests P slice weighting factors to perform or not WP RD decision
************************************************************************
*/
int TestWPPSlicePeriodic(Slice *currSlice, int select_offset)
{  
  VideoParameters *p_Vid = currSlice->p_Vid;
  p_Vid->wp_parameters_set = 1; 

  return 1;
}


/*!
************************************************************************
* \brief
*    TestWPBSliceRandom:
*    Tests B slice weighting prediction
************************************************************************
*/
int TestWPBSlicePeriodic(Slice *currSlice, int select_method)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  p_Vid->wp_parameters_set = 1; 

  return 1;
}


