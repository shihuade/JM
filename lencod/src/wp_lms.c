
/*!
*************************************************************************************
* \file wp_lms.c
*
* \brief
*    Estimate weights for WP using LMS method
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*     - Alexis Michael Tourapis         <alexismt@ieee.org>
*     - Athanasios Leontaris            <aleon@dolby.com>
*     - Yan Ye                          <yye@dolby.com>
*************************************************************************************
*/
#include "contributors.h"

#include "global.h"
#include "image.h"
#include "wp_lms.h"
#include "wp.h"


static double ComputeBlockNormMean(imgpel **CurrentImage, 
                                   int height_in_blk, int width_in_blk, 
                                   int blk_size_y, int blk_size_x, 
                                   int cur_blk, double dc_mean)
{
  int blkx, blky; 
  int x, y;
  double sum = 0.; 
  imgpel *img_ptr; 

  blkx = cur_blk % width_in_blk;
  blky = cur_blk / width_in_blk;

  for(y = 0; y < blk_size_y; y++)
  {
    img_ptr = &CurrentImage[blky*blk_size_y+y][blkx*blk_size_x];
    for(x = 0; x < blk_size_x; x++)
    {
      sum += dabs((double)img_ptr[x]-dc_mean);
    }
  }

  return sum; 
}

static void ComputeNormMeanBlockBased(imgpel **CurrentImage, int height_in_blk, int width_in_blk, 
                                      int blk_size_y, int blk_size_x, 
                                      int start_blk, int end_blk, 
                                      double dc_mean, double *norm_slice)
{
  int cur_blk;
  double block_norm; 
  double norm;
  
  {
    norm = 0.;
    for(cur_blk = start_blk; cur_blk < end_blk; cur_blk++)
    {
      block_norm = ComputeBlockNormMean(CurrentImage, height_in_blk, width_in_blk, blk_size_y, blk_size_x, cur_blk, dc_mean);
      norm += block_norm; 
    }
    (*norm_slice) = (double) norm;
  }
}

void ComputeExplicitWPParamsLMS(Slice *currSlice,
                                int select_offset,
                                int start_mb,
                                int end_mb, 
                                short default_weight[3],
                                short weight[6][MAX_REFERENCE_PICTURES][3],
                                short offset[6][MAX_REFERENCE_PICTURES][3])
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int clist, n, k, comp;
  int slice_size, slice_size_cr; 

  double dc_org[3];
  double dc_ref[3];
  double norm_org[3];
  double norm_ref[3]; 
  double numer[3];
  double denom[3]; 
  short  cur_weight, cur_offset;
  int   bit_depth = (p_Vid->bitdepth_luma - 8);

  imgpel **cur_ref; 

  for (clist=0; clist< 2; clist++)
  {
    for (n = 0; n < currSlice->listXsize[clist]; n++)
    {
      for (comp = 0; comp < 3; comp++)
      {
        weight[clist][n][comp] = default_weight[comp];
        offset[clist][n][comp] = 0;
      }
    }
  }

  slice_size    = (end_mb-start_mb)*MB_BLOCK_SIZE*MB_BLOCK_SIZE;
  slice_size_cr = (end_mb-start_mb)*p_Vid->mb_cr_size_x*p_Vid->mb_cr_size_y;

  ComputeImgSumBlockBased(p_Vid->pCurImg, p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, MB_BLOCK_SIZE, MB_BLOCK_SIZE,
    start_mb, end_mb, &dc_org[0]);  

  norm_org[0] = dc_org[0]/((double) slice_size);
  ComputeNormMeanBlockBased(p_Vid->pCurImg, p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, MB_BLOCK_SIZE, MB_BLOCK_SIZE, 
    start_mb, end_mb, norm_org[0], &numer[0]);  

  if (p_Inp->ChromaWeightSupport == 1)
  {
    for (k = 0; k < 2; k++)
    {
      ComputeImgSumBlockBased(p_Vid->pImgOrg[k + 1], p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x,
        start_mb, end_mb, &dc_org[k+1]);  
    } 
  }

  for (clist = 0; clist < 2; clist++)
  {
    for (n = 0; n < currSlice->listXsize[clist]; n++)
    {
      cur_ref = currSlice->listX[clist][n]->p_curr_img;
      ComputeImgSumBlockBased(cur_ref, p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, MB_BLOCK_SIZE, MB_BLOCK_SIZE, 
        start_mb, end_mb, &dc_ref[0]);  
      norm_ref[0] = dc_ref[0] / ((double) slice_size);
      ComputeNormMeanBlockBased(cur_ref, p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, MB_BLOCK_SIZE, MB_BLOCK_SIZE, 
        start_mb, end_mb, norm_ref[0], &denom[0]);  

      if (p_Inp->ChromaWeightSupport == 1)
      {
        for (k = 0; k < 2; k++)
        {
          cur_ref = currSlice->listX[clist][n]->imgUV[k];
          ComputeImgSumBlockBased(cur_ref, p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x, 
            start_mb, end_mb, &dc_ref[k+1]);  
        }        
      }

      if (select_offset == 0)
      {
        if (denom[0] != 0)
          cur_weight = (short) (default_weight[0] * numer[0] / denom[0] + 0.5);
        else
          cur_weight = default_weight[0];  // only used when reference picture is black
        cur_weight = sClip3(-128, 127, cur_weight);

        cur_offset = (short) (norm_org[0] - ((double) cur_weight * norm_ref[0] / (double) default_weight[0]) + 0.5);
        cur_offset = (short) ((cur_offset + (bit_depth>>1)) >> bit_depth);
        cur_offset = sClip3( -128, 127, cur_offset);
        cur_offset = (short) (cur_offset << bit_depth);
        weight[clist][n][0] = cur_weight;
        offset[clist][n][0] = cur_offset;

        if (p_Inp->ChromaWeightSupport == 1)
        {
          if (dc_ref[1] != 0)
            cur_weight = (short) (default_weight[1] * dc_org[1] / dc_ref[1] + 0.5);
          else
            cur_weight = default_weight[1];  // only used when reference picture is black
          weight[clist][n][1] = sClip3(-128, 127, cur_weight);

          if (dc_ref[2] != 0)
            cur_weight = (short) (default_weight[2] * dc_org[2] / dc_ref[2] + 0.5);
          else
            cur_weight = default_weight[2];  // only used when reference picture is black
          weight[clist][n][2] = sClip3(-128, 127, cur_weight);
        }
      }
      else
      {
        cur_offset = (short) ((dc_org[0] - dc_ref[0])/(slice_size)+0.5);
        cur_offset = (cur_offset+((p_Vid->bitdepth_luma-8)>>1))>>(p_Vid->bitdepth_luma-8);
        cur_offset = sClip3( -128, 127, cur_offset);
        cur_offset = cur_offset<<(p_Vid->bitdepth_luma-8);
        cur_weight = default_weight[0];

        weight[clist][n][0] = cur_weight;
        offset[clist][n][0] = cur_offset;

        if (p_Inp->ChromaWeightSupport == 1)
        {
          cur_offset = (short) ((dc_org[1] - dc_ref[1])/(slice_size_cr)+0.5);
          cur_offset = (cur_offset + ((p_Vid->bitdepth_chroma - 8)>>1))>>(p_Vid->bitdepth_chroma-8);
          cur_offset = sClip3( -128, 127, cur_offset);
          offset[clist][n][1] = cur_offset<<(p_Vid->bitdepth_chroma - 8);

          weight[clist][n][1] = default_weight[1];

          cur_offset = (short) ((dc_org[2] - dc_ref[2])/(slice_size_cr)+0.5);
          cur_offset = (cur_offset + ((p_Vid->bitdepth_chroma - 8)>>1))>>(p_Vid->bitdepth_chroma-8);
          cur_offset = sClip3( -128, 127, cur_offset);
          offset[clist][n][2] = cur_offset<<(p_Vid->bitdepth_chroma - 8);

          weight[clist][n][2] = default_weight[2];
        }
      }
    }
  }
}


void ComputeExplicitWPParamsJNT(Slice *currSlice,                                  
                                int start_mb,
                                int end_mb, 
                                short default_weight[3],
                                short weight[6][MAX_REFERENCE_PICTURES][3],
                                short offset[6][MAX_REFERENCE_PICTURES][3])
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int clist, n, k, comp;

  // all these should be converted to int64
  int64 dc_org[3];
  int64 dc_cur_ref[2][MAX_REFERENCE_PICTURES][3];
  short w0 = default_weight[0], w1 = default_weight[0];

  imgpel **cur_ref; 

  for (clist=0; clist< 2; clist++)
  {
    for (n = 0; n < currSlice->listXsize[clist]; n++)
    {
      for (comp = 0; comp < 3; comp++)
      {
        weight[clist][n][comp] = default_weight[comp];
        offset[clist][n][comp] = 0;
      }
    }
  }

  dc_org[0] = ComputeSumBlockBased   (p_Vid->pCurImg, p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, MB_BLOCK_SIZE, MB_BLOCK_SIZE,
    start_mb, end_mb);  

  if (p_Inp->ChromaWeightSupport == 1)
  {
    for (k = 0; k < 2; k++)
    {
      dc_org[k+1] = ComputeSumBlockBased(p_Vid->pImgOrg[k + 1], p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x,
        start_mb, end_mb);  
    } 
  }

  for (clist = 0; clist < 2; clist++)
  {
    for (n = 0; n < currSlice->listXsize[clist]; n++)
    {
      cur_ref = currSlice->listX[clist][n]->p_curr_img;
      dc_cur_ref[clist][n][0] = ComputeSumBlockBased (cur_ref, p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, MB_BLOCK_SIZE, MB_BLOCK_SIZE, start_mb, end_mb);  

      if (p_Inp->ChromaWeightSupport == 1)
      {
        for (k = 0; k < 2; k++)
        {
          cur_ref = currSlice->listX[clist][n]->imgUV[k];
          dc_cur_ref[clist][n][k+1] = ComputeSumBlockBased (cur_ref, p_Vid->FrameHeightInMbs, p_Vid->PicWidthInMbs, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x, start_mb, end_mb);  
        }        
      }
    }
  }

  // setup list 1 references using zero ref of LIST_0
  for (n = 0; n < currSlice->listXsize[LIST_1]; n++)
  {

    double temp_weight = ( (double)(dc_cur_ref[LIST_1][n][0] - dc_org[0]) / (double) (dc_org[0] - dc_cur_ref[LIST_0][0][0]));

    w0 = (short) (((double) default_weight[0] * 2 * temp_weight) / ( 1.0 + temp_weight) + 0.5);
    w1 = (short) (((double) default_weight[0] * 2) / ( 1.0 + temp_weight) + 0.5);

    //printf("%.2f %.2f %.2f %.2f %d %d\n",temp_weight,dc_ref[1][0],dc_ref[0][0], dc_org[0], cm_weight[0 + list_offset][i][j][0] ,cm_weight[1 + list_offset][i][j][0] );
    if (w0 <= 0 || w1 <= 0)
    {
      w0 = default_weight[0];
      w1 = default_weight[0];
    }

    if (w1 < -64 || w1 > 128 || w0 < -64 || w0 >128)
    {
      w0 = default_weight[0];
      w1 = default_weight[0];
    }

    weight[LIST_1][n][0] = w1;
    offset[LIST_1][n][0] = 0;

    if (p_Inp->ChromaWeightSupport == 1)
    {
      weight[LIST_1][n][1] = (short) ((double)(default_weight[1] * w1) / (double) default_weight[0]);
      weight[LIST_1][n][2] = (short) ((double)(default_weight[2] * w1) / (double) default_weight[0]);
    }
  }

  // now set LIST_0 weights with 0 reference of LIST_1
  for (n = 0; n < currSlice->listXsize[LIST_0]; n++)
  {
    double temp_weight = ( (double)(dc_cur_ref[LIST_1][0][0] - dc_org[0]) / (double) (dc_org[0] - dc_cur_ref[LIST_0][n][0]));

    w0 = (short) (((double) default_weight[0] * 2 * temp_weight) / ( 1.0 + temp_weight) + 0.5);
    w1 = (short) (((double) default_weight[0] * 2) / ( 1.0 + temp_weight) + 0.5);

    if (w0 <= 0 || w1 <= 0)
    {
      w0 = default_weight[0];
      w1 = default_weight[0];
    }

    if (w1 < -64 || w1 > 128 || w0 < -64 || w0 >128)
    {
      w0 = default_weight[0];
      w1 = default_weight[0];
    }

    weight[LIST_0][n][0] = w0;
    offset[LIST_0][n][0] = 0;

    if (p_Inp->ChromaWeightSupport == 1)
    {
      weight[LIST_0][n][1] = (short) ((double)(default_weight[1] * w0) / (double) default_weight[0]);
      weight[LIST_0][n][2] = (short) ((double)(default_weight[2] * w0) / (double) default_weight[0]);
    }
  }
}

/*!
************************************************************************
* \brief
*    Estimates reference picture weighting factors for P slices
************************************************************************
*/

void EstimateWPPSliceAlg1(Slice *currSlice, int select_offset)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;

  int i, n;
  short default_weight[3];
  int list_offset   = ((currSlice->mb_aff_frame_flag) && (p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  int clist;

  int cur_slice;

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

  if(p_Vid->wp_parameters_set == 0)
  {
    int start_mb, end_mb; 
    int comp;

    short weight[2][MAX_REFERENCE_PICTURES][3];
    short offset[2][MAX_REFERENCE_PICTURES][3];

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

    ComputeExplicitWPParamsLMS(currSlice, select_offset, start_mb, end_mb, default_weight, weight, offset);

    for (clist = 0; clist < 2 + list_offset; clist++)
    {
      for (n = 0; n < currSlice->listXsize[clist]; n++)
      {
        if ( wpxDetermineWP( currSlice, clist, n ) )
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
  }
  else
  {
    for (clist = 0; clist < 2 + list_offset; clist++)
    {
      for (n = 0; n < currSlice->listXsize[clist]; n++)
      {
        if ( wpxDetermineWP( currSlice, clist, n ) )
        {
          for (i=0; i < 3; i ++)
          {
            currSlice->wp_weight[clist][n][i] = p_Vid->wp_weights[i][clist][n][cur_slice];
            currSlice->wp_offset[clist][n][i] = p_Vid->wp_offsets[i][clist][n][cur_slice];
#if DEBUG_WP
          printf("slice %d: index %d component %d weight %d offset %d\n", cur_slice, n,i,currSlice->wp_weight[clist][n][i],currSlice->wp_offset[clist][n][i]);
#endif
          }
        }
      }
    }
  }
}

/*!
************************************************************************
* \brief
*    Estimates reference picture weighting factors for B slices
************************************************************************
*/
void EstimateWPBSliceAlg1(Slice *currSlice)
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

  if (p_Vid->wp_parameters_set == 0)    // single-pass coding
  {
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

      if (p_Inp->EnhancedBWeightSupport == 2)
        ComputeExplicitWPParamsJNT(currSlice, start_mb, end_mb, default_weight, wp_weight, wp_offset);
      else
        ComputeExplicitWPParamsLMS(currSlice, 0, start_mb, end_mb, default_weight, wp_weight, wp_offset);

      for (clist=0; clist<2 + list_offset; clist++)
      {
        for (n = 0; n < currSlice->listXsize[clist]; n++)
        {
          if ( wpxDetermineWP( currSlice, clist, n ) )  
          {
            for (comp=0; comp < 3; comp ++)
            {
              currSlice->wp_weight[clist][n][comp] = wp_weight[clist][n][comp];
              currSlice->wp_offset[clist][n][comp] = wp_offset[clist][n][comp];
              if(p_Vid->wp_weights)
                p_Vid->wp_weights[comp][clist][n][cur_slice] = wp_weight[clist][n][comp];
              if(p_Vid->wp_offsets)
                p_Vid->wp_offsets[comp][clist][n][cur_slice] = wp_offset[clist][n][comp];
            }
          }
        }
      }

      if (p_Vid->active_pps->weighted_bipred_idc != 1)
      {
        for (clist=0; clist<2 + list_offset; clist++)
        {
          for (n = 0; n < currSlice->listXsize[clist]; n++)
          {
            memcpy(currSlice->wp_weight[clist][n], default_weight, 3 * sizeof(short));
            memset(currSlice->wp_offset[clist][n], 0, 3 * sizeof(short));
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
          }
        }
      }
    }
  }
  else    // multiple-pass coding, WP parameters have been decided and stored
  if (p_Vid->active_pps->weighted_bipred_idc == 2) //! implicit mode
  {
    for (k = 0; k < 2; k++)
    {
      for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
      {
        for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
        {
          for (comp = 0; comp < 3; comp++)
          {
            currSlice->wbp_weight[k][i][j][comp] = p_Vid->wbp_weight[comp][k][i][j][cur_slice];
          }
        }
      }
    }
  }
  else // explicit WP mode (or no WP at all)
  {
    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (n = 0; n < currSlice->listXsize[clist]; n++)
      {
        if ( wpxDetermineWP( currSlice, clist, n ) )  // use WP with pre-stored parameters
        {
          for (i=0; i < 3; i ++)
          {
            currSlice->wp_weight[clist][n][i] = p_Vid->wp_weights[i][clist][n][cur_slice];
            currSlice->wp_offset[clist][n][i] = p_Vid->wp_offsets[i][clist][n][cur_slice];
          }
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
int TestWPPSliceAlg1(Slice *currSlice, int select_offset)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int i, j, n;

  int index;
  int comp;

  short default_weight[3];

  int list_offset   = ((p_Vid->mb_aff_frame_flag)&&(p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  short weight[6][MAX_REFERENCE_PICTURES][3];
  short offset[6][MAX_REFERENCE_PICTURES][3];
  int clist;
  int perform_wp = 0;

  int cur_slice, num_slice;
  int start_mb, end_mb; 

  short luma_log_weight_denom   = 5;
  short chroma_log_weight_denom = 5;

  default_weight[0] = 1 << luma_log_weight_denom;
  default_weight[1] = default_weight[2] = 1 << chroma_log_weight_denom;

  /* set all values to defaults */
  for (i = 0; i < 2 + list_offset; i++)
  {
    for (j = 0; j < currSlice->listXsize[i]; j++)
    {
      for (n = 0; n < 3; n++)
      {
        weight[i][j][n] = default_weight[n];
        offset[i][j][n] = 0;
      }
    }
  }

  num_slice = p_Vid->num_slices_wp;
  for(cur_slice = 0; cur_slice < num_slice; cur_slice++)
  {
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

    ComputeExplicitWPParamsLMS(currSlice, select_offset, start_mb, end_mb, default_weight, weight, offset);

    for (clist = 0; clist < 2; clist++)
    {
      for (n = 0; n < currSlice->listXsize[clist]; n++)
      {
        for (comp=0; comp < 3; comp ++)
        {
          p_Vid->wp_weights[comp][clist][n][cur_slice] = weight[clist][n][comp];
          p_Vid->wp_offsets[comp][clist][n][cur_slice] = offset[clist][n][comp];
        }
#if DEBUG_WP
        printf ("weight[%d][%d][0] = %d\n", clist, n, weight[clist][n][0]);
        printf ("offset[%d][%d][0] = %d\n", clist, n, offset[clist][n][0]);
        printf ("weight[%d][%d][1] = %d\n", clist, n, weight[clist][n][1]);
        printf ("offset[%d][%d][1] = %d\n", clist, n, offset[clist][n][1]);
        printf ("weight[%d][%d][2] = %d\n", clist, n, weight[clist][n][2]);
        printf ("offset[%d][%d][2] = %d\n", clist, n, offset[clist][n][2]);
#endif
      }
    }
  }

  perform_wp = 0;
  for(cur_slice = 0; cur_slice < num_slice; cur_slice++)
  {
    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (index = 0; index < currSlice->listXsize[clist]; index++)
      {
        for (comp=0; comp < 3; comp ++)
        {
          int offset_test = p_Inp->RDPSliceBTest && p_Vid->active_sps->profile_idc != BASELINE
            ? iabs(p_Vid->wp_offsets[comp][clist][index][cur_slice]) > 2
            : p_Vid->wp_offsets[comp][clist][index][cur_slice] != 0;

          if (p_Vid->wp_weights[comp][clist][index][cur_slice] != default_weight[comp] || offset_test)
          {
            perform_wp = 1;
            break;
          }
        }
        if (perform_wp == 1) break;
      }
      if (perform_wp == 1) break;
    }
    if(perform_wp == 1) break;
  }

  p_Vid->wp_parameters_set = 1; 

  return perform_wp;
}


/*!
************************************************************************
* \brief
*    TestWPBSliceAlg1:
*    Tests B slice weighting prediction
************************************************************************
*/
int TestWPBSliceAlg1(Slice *currSlice, int select_method)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int i, j, n;

  int index, comp;

  short default_weight[3];
  // this needs to be fixed.
  int list_offset   = ((p_Vid->mb_aff_frame_flag) && (p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  short im_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3];
  short wp_weight[6][MAX_REFERENCE_PICTURES][3];
  short wp_offset[6][MAX_REFERENCE_PICTURES][3];

  int cur_slice, num_slice;
  int start_mb, end_mb; 

  int clist;
  int perform_wp = 1;

  short luma_log_weight_denom   = 5;
  short chroma_log_weight_denom = 5;

  default_weight[0] = 1 << luma_log_weight_denom;
  default_weight[1] = 1 << chroma_log_weight_denom;
  default_weight[2] = 1 << chroma_log_weight_denom;

  if (select_method == 1) //! implicit WP
  {
    cur_slice = 0;
    num_slice = p_Vid->num_slices_wp;

    ComputeImplicitWeights(currSlice, default_weight, im_weight);

    perform_wp = 1;
    for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
    {
      for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
      {
        for (comp = 0; comp < 3; comp++)
        {
          short weight0, weight1; 

          if(perform_wp)
          {
            weight0 = im_weight[0][i][j][comp];
            weight1 = im_weight[1][i][j][comp];
          }
          else  
          {
            weight0 = default_weight[comp];
            weight1 = default_weight[comp];
          }

          // propagate implicit weights to all slices
          for(cur_slice = 0; cur_slice < num_slice; cur_slice++)
          {
            p_Vid->wbp_weight[comp][0][i][j][cur_slice] = weight0;
            p_Vid->wbp_weight[comp][1][i][j][cur_slice] = weight1;
          }
        }
      }
    }
  }
  else    // explicit WP
  {
    num_slice  = p_Vid->num_slices_wp;
    for(cur_slice = 0; cur_slice < num_slice; cur_slice++)
    {
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
  
      if (p_Inp->EnhancedBWeightSupport == 2)
        ComputeExplicitWPParamsJNT(currSlice, start_mb, end_mb, default_weight, wp_weight, wp_offset);
      else
        ComputeExplicitWPParamsLMS(currSlice, 0, start_mb, end_mb, default_weight, wp_weight, wp_offset);

      for (clist=0; clist<2 + list_offset; clist++)
      {
        for (n = 0; n < currSlice->listXsize[clist]; n++)
        {
          for (comp=0; comp < 3; comp ++)
          {
            p_Vid->wp_weights[comp][clist][n][cur_slice] = wp_weight[clist][n][comp];
            p_Vid->wp_offsets[comp][clist][n][cur_slice] = wp_offset[clist][n][comp];
          }
#if DEBUG_WP
          printf ("weight[%d][%d] = %d\n", clist, n, wp_weight[clist][n][0]);
          printf ("offset[%d][%d] = %d\n", clist, n, wp_offset[clist][n][0]);
#endif
        }
      }
    }

    perform_wp = 0; 
    for(cur_slice = 0; cur_slice < num_slice; cur_slice++)
    {
      for (clist=0; clist<2 + list_offset; clist++)
      {
        for (index = 0; index < currSlice->listXsize[clist]; index++)
        {
          for (comp=0; comp < 3; comp ++)
          {
            if (p_Vid->wp_weights[comp][clist][index][cur_slice] != default_weight[comp] || 
              p_Vid->wp_offsets[comp][clist][index][cur_slice] != 0)
            {
              perform_wp = 1;
              break;
            }
          }
          if (perform_wp == 1) break;
        }
        if (perform_wp == 1) break;
      }
      if(perform_wp == 1) break;
    }
  }

  p_Vid->wp_parameters_set = 1; 

  return perform_wp;
}


