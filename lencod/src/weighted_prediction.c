
/*!
*************************************************************************************
* \file weighted_prediction.c
*
* \brief
*    Estimate weights for WP using DC method
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*     - Alexis Michael Tourapis         <alexismt@ieee.org>
*     - Athanasios Leontaris            <aleon@dolby.com>
*************************************************************************************
*/
#include "contributors.h"

#include "global.h"
#include "image.h"
#include "wp.h"

/*!
************************************************************************
* \brief
*    Initialize weighting parameter functions
************************************************************************
*/
void InitWP(VideoParameters *p_Vid, InputParameters *p_Inp, int force_wp_method)
{
  if(force_wp_method)
  {
      p_Vid->EstimateWPPSlice = EstimateWPPSliceAlg1;
      p_Vid->EstimateWPBSlice = EstimateWPBSliceAlg1;
      p_Vid->TestWPPSlice = TestWPPSliceAlg1;
      p_Vid->TestWPBSlice = TestWPBSliceAlg1;
  }
  else
  if (p_Inp->WPIterMC && p_Vid->nal_reference_idc)
  {
    p_Vid->EstimateWPPSlice = EstimateWPPSliceAlg2;
    p_Vid->EstimateWPBSlice = EstimateWPBSliceAlg2;
    p_Vid->TestWPPSlice = TestWPPSliceAlg2;
    p_Vid->TestWPBSlice = TestWPBSliceAlg2;
  }
  else
  {
    int wp_method = p_Inp->WPMethod;
    switch (wp_method)
    {
    default:
    case 0:
      p_Vid->EstimateWPPSlice = EstimateWPPSliceAlg0;
      p_Vid->EstimateWPBSlice = EstimateWPBSliceAlg0;
      p_Vid->TestWPPSlice = TestWPPSliceAlg0;
      p_Vid->TestWPBSlice = TestWPBSliceAlg0;
      break;
    case 1:
      p_Vid->EstimateWPPSlice = EstimateWPPSliceAlg1;
      p_Vid->EstimateWPBSlice = EstimateWPBSliceAlg1;
      p_Vid->TestWPPSlice = TestWPPSliceAlg1;
      p_Vid->TestWPBSlice = TestWPBSliceAlg1;
      break;
    case 2:
      p_Vid->EstimateWPPSlice = EstimateWPPSliceRandom;
      p_Vid->EstimateWPBSlice = EstimateWPBSliceRandom;
      p_Vid->TestWPPSlice = TestWPPSliceRandom;
      p_Vid->TestWPBSlice = TestWPBSliceRandom;
      break;
    case 3:
      p_Vid->EstimateWPPSlice = EstimateWPPSlicePeriodic;
      p_Vid->EstimateWPBSlice = EstimateWPBSlicePeriodic;
      p_Vid->TestWPPSlice = TestWPPSlicePeriodic;
      p_Vid->TestWPBSlice = TestWPBSlicePeriodic;
      break;

    }
  }
}

void ResetWP(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  short default_weights[3];
  short luma_log_weight_denom   = 5;
  short chroma_log_weight_denom = 5;
  int cur_slice, clist, n;

  default_weights[0] = 1 << luma_log_weight_denom;
  default_weights[1] = default_weights[2] = 1 << chroma_log_weight_denom;
  
  p_Vid->wp_parameters_set = 0;
  for (cur_slice = 0; cur_slice < p_Vid->num_slices_wp; cur_slice++)
  {
    for (clist = 0; clist < 2 ; clist++)
    {
      for (n = 0; n < p_Inp->num_ref_frames; n++)
      {
        p_Vid->wp_weights[0][clist][n][cur_slice] = default_weights[0];
        p_Vid->wp_offsets[0][clist][n][cur_slice] = 0;
        p_Vid->wp_weights[1][clist][n][cur_slice] = default_weights[1];
        p_Vid->wp_offsets[1][clist][n][cur_slice] = 0;
        p_Vid->wp_weights[2][clist][n][cur_slice] = default_weights[2];
        p_Vid->wp_offsets[2][clist][n][cur_slice] = 0;
      }
    }
  }
}

void ComputeImplicitWeights(Slice *currSlice,
                            short default_weight[3],
                            short im_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3])
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  int i, j, comp;
  short tx, td, tb, DistScaleFactor;

  for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
  {
    for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
    {
      td = (short) iClip3(-128, 127,(currSlice->listX[LIST_1][j]->poc - currSlice->listX[LIST_0][i]->poc));
      tb = (short) iClip3(-128, 127,(p_Vid->ThisPOC - currSlice->listX[LIST_0][i]->poc));
      for (comp = 0; comp < 3; comp++)
      {
        // implicit weights
        if (td == 0)
        {
          im_weight[0][i][j][comp] = default_weight[comp];
          im_weight[1][i][j][comp] = default_weight[comp];
        }
        else
        {
          tx = (short) (16384 + iabs(td >> 1))/td;
          DistScaleFactor = sClip3(-1024, 1023, (tx*tb + 32 )>>6);
          im_weight[1][i][j][comp] = DistScaleFactor >> 2;
          if (im_weight[1][i][j][comp] < -64 || im_weight[1][i][j][comp] >128)
            im_weight[1][i][j][comp] = default_weight[comp];
          im_weight[0][i][j][comp] = 64 - im_weight[1][i][j][comp];
        }
      }
    }
  }
}


/*!
************************************************************************
* \brief
*    Compute sum of samples in a picture
************************************************************************
*/
double ComputeImgSum(imgpel **CurrentImage, int height, int width)
{
  int i, j;
  double sum_value = 0.0;
  imgpel *p_tmp;

  for (i = 0; i < height; i++)
  {
    p_tmp = CurrentImage[i];
    for (j = 0; j < width; j++)
    {
      sum_value += (double) *(p_tmp++);
    }
  }
  return sum_value;
}

static int ComputeBlockSum(imgpel **CurrentImage, int height_in_blk, int width_in_blk, int blk_size_y, int blk_size_x, int cur_blk)
{
  int x, y;
  int sum = 0; 

  int blkx = cur_blk % width_in_blk;
  int blky = cur_blk / width_in_blk;
  int blk_start_x = blkx*blk_size_x;
  int blk_start_y = blky*blk_size_y;
  imgpel *img_ptr;

  for(y = 0; y < blk_size_y; y++)
  {
    img_ptr = &CurrentImage[blk_start_y + y][blk_start_x];
    for(x = 0; x < blk_size_x; x++)
    {
      sum += img_ptr[x];
    }
  }

  return sum; 
}

void ComputeImgSumBlockBased(imgpel **CurrentImage, 
                             int height_in_blk, int width_in_blk, 
                             int blk_size_y, int blk_size_x, 
                             int start_blk, int end_blk, double *dc)
{
  int cur_blk;
  int block_sum; 
  int64 slice_sum = 0;

  for(cur_blk = start_blk; cur_blk < end_blk; cur_blk++)
  {
    block_sum = ComputeBlockSum(CurrentImage, height_in_blk, width_in_blk, blk_size_y, blk_size_x, cur_blk);
    slice_sum += block_sum; 
  }
  (*dc) = (double) slice_sum;
}

int64 ComputeSumBlockBased(imgpel **CurrentImage, 
                             int height_in_blk, int width_in_blk, 
                             int blk_size_y, int blk_size_x, 
                             int start_blk, int end_blk)
{
  int cur_blk;
  int64 slice_sum = 0;

  for(cur_blk = start_blk; cur_blk < end_blk; cur_blk++)
  {
    slice_sum += ComputeBlockSum(CurrentImage, height_in_blk, width_in_blk, blk_size_y, blk_size_x, cur_blk);
  }
  return slice_sum;
}

/*!
************************************************************************
* \brief
*    Estimates reference picture weighting factors for P slices
************************************************************************
*/

void EstimateWPPSliceAlg0(Slice *currSlice, int select_offset)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  double dc_org = 0.0;
  double dc_org_UV[2] = {0.0};
  double dc_ref[MAX_REFERENCE_PICTURES] = { 0.0 };
  double dc_ref_UV[MAX_REFERENCE_PICTURES][2] = { {0.0}};

  int i, n, k;
  short default_weight[3];
  short cur_weight, cur_offset;
  int list_offset   = ((currSlice->mb_aff_frame_flag)&&(p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  short weight[2][MAX_REFERENCE_PICTURES][3];
  short offset[2][MAX_REFERENCE_PICTURES][3];
  int clist;

  imgpel **tmpPtr;

  currSlice->luma_log_weight_denom   = 5;
  currSlice->chroma_log_weight_denom = 5;

  currSlice->wp_luma_round           = 1 << (currSlice->luma_log_weight_denom - 1);
  currSlice->wp_chroma_round         = 1 << (currSlice->chroma_log_weight_denom - 1);
  default_weight[0]       = 1 << currSlice->luma_log_weight_denom;
  default_weight[1]       = default_weight[2] = 1 << currSlice->chroma_log_weight_denom;
  
  dc_org = ComputeImgSum(p_Vid->pCurImg, p_Vid->height, p_Vid->width);

  if (p_Inp->ChromaWeightSupport == 1)
  {
    for (k = 0; k < 2; k++)
    {
      dc_org_UV[k] = ComputeImgSum(p_Vid->pImgOrg[k + 1], p_Vid->height_cr, p_Vid->width_cr);
    } 
  }

  for (clist = 0; clist < 2 + list_offset; clist++)
  {
    for (n = 0; n < currSlice->listXsize[clist]; n++)
    {
      if ( wpxDetermineWP( currSlice, clist, n ) )
      {
        /* set all values to defaults */
        for (i = 0; i < 3; i++)
        {
          weight[clist][n][i]    = default_weight[i];
          currSlice->wp_weight[clist][n][i] = default_weight[i];
          currSlice->wp_offset[clist][n][i] = 0;
          offset[clist][n][i]    = 0;
        }

        // Y
        tmpPtr = currSlice->listX[clist][n]->p_curr_img;      
        dc_ref[n] = ComputeImgSum(tmpPtr, p_Vid->height, p_Vid->width);

        if (p_Inp->ChromaWeightSupport == 1)
        {
          for (k = 0; k < 2; k++)
          {
            // UV
            tmpPtr = currSlice->listX[clist][n]->imgUV[k];
            dc_ref_UV[n][k] = ComputeImgSum(tmpPtr, p_Vid->height_cr, p_Vid->width_cr);
          }        
        }

        if (select_offset == 0)
        {
          if (dc_ref[n] != 0.0)
           cur_weight  = (short) (default_weight[0] * dc_org / dc_ref[n] + 0.5);
          else
            cur_weight = default_weight[0];  // only used when reference picture is black
          cur_weight = sClip3(-128, 127, cur_weight);
          cur_offset = 0;

          weight[clist][n][0] = cur_weight;
          offset[clist][n][0] = cur_offset;

          if (p_Inp->ChromaWeightSupport == 1)
          {
            if (dc_ref_UV[n][0] != 0)
              weight[clist][n][1] = (short) (default_weight[1] * dc_org_UV[0] / dc_ref_UV[n][0] + 0.5);
            else
              weight[clist][n][1] = default_weight[1];  // only used when reference picture is black
            weight[clist][n][1] = sClip3(-128, 127, weight[clist][n][1]);

            if (dc_ref_UV[n][1] != 0)
              weight[clist][n][2] = (short) (default_weight[2] * dc_org_UV[1] / dc_ref_UV[n][1] + 0.5);
            else
              weight[clist][n][2] = default_weight[2];  // only used when reference picture is black
            weight[clist][n][2] = sClip3(-128, 127, weight[clist][n][2]);
          }
        }
        else
        {
          cur_offset = (short) ((dc_org - dc_ref[n])/(p_Vid->size)+0.5);
          cur_offset = (cur_offset+((p_Vid->bitdepth_luma-8)>>1))>>(p_Vid->bitdepth_luma-8);
          cur_offset = sClip3( -128, 127, cur_offset);
          cur_offset = cur_offset<<(p_Vid->bitdepth_luma-8);
          cur_weight = default_weight[0];

          weight[clist][n][0] = cur_weight;
          offset[clist][n][0] = cur_offset;

          if (p_Inp->ChromaWeightSupport == 1)
          {
            offset[clist][n][1] = (short) ((dc_org_UV[0] - dc_ref_UV[n][0])/(p_Vid->size_cr)+0.5);
            offset[clist][n][1] = (offset[clist][n][1] + ((p_Vid->bitdepth_chroma - 8)>>1))>>(p_Vid->bitdepth_chroma-8);
            offset[clist][n][1] = sClip3( -128, 127, offset[clist][n][1]);
            offset[clist][n][1] = offset[clist][n][1]<<(p_Vid->bitdepth_chroma - 8);
            
            weight[clist][n][1] = default_weight[1];

            offset[clist][n][2] = (short) ((dc_org_UV[1] - dc_ref_UV[n][1])/(p_Vid->size_cr)+0.5);
            offset[clist][n][2] = (offset[clist][n][2] + ((p_Vid->bitdepth_chroma - 8)>>1))>>(p_Vid->bitdepth_chroma-8);
            offset[clist][n][2] = sClip3( -128, 127, offset[clist][n][2]);
            offset[clist][n][2] = offset[clist][n][2]<<(p_Vid->bitdepth_chroma - 8);

            weight[clist][n][2] = default_weight[2];
          }
        }

        for (i=0; i < 3; i ++)
        {
          currSlice->wp_weight[clist][n][i] = weight[clist][n][i];
          currSlice->wp_offset[clist][n][i] = offset[clist][n][i];
#if DEBUG_WP
          printf("index %d component %d weight %d offset %d\n",n,i,weight[0][n][i],offset[0][n][i]);
#endif
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
void EstimateWPBSliceAlg0(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int i, j, k, n;

  int index;
  int comp;
  double dc_org = 0.0;
  double dc_org_UV[2] = { 0.0 };
  double dc_ref[6][MAX_REFERENCE_PICTURES] = { {0.0} };
  double dc_ref_UV[6][MAX_REFERENCE_PICTURES][2] = { {{0.0}} };

  short default_weight[3];
  int list_offset   = ((currSlice->mb_aff_frame_flag)&&(p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  short weight[6][MAX_REFERENCE_PICTURES][3];
  short offset[6][MAX_REFERENCE_PICTURES][3];
  short im_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3];
  int clist;
  short wf_weight, wf_offset;
  imgpel **tmpPtr;

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

  currSlice->wp_luma_round     = 1 << (currSlice->luma_log_weight_denom - 1);
  currSlice->wp_chroma_round   = 1 << (currSlice->chroma_log_weight_denom - 1);
  default_weight[0] = 1 << currSlice->luma_log_weight_denom;
  default_weight[1] = 1 << currSlice->chroma_log_weight_denom;
  default_weight[2] = 1 << currSlice->chroma_log_weight_denom;

  if (p_Vid->active_pps->weighted_bipred_idc == 2) //! implicit mode
  {
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
      for (index = 0; index < currSlice->listXsize[clist]; index++)
      {
        for (comp = 0; comp < 3; comp++)
        {
          currSlice->wp_weight[clist][index][comp] = default_weight[comp];
          currSlice->wp_offset[clist][index][comp] = 0;
        }
      }
    }
  }
  else
  {
    dc_org = ComputeImgSum(p_Vid->pCurImg, p_Vid->height, p_Vid->width);

    if (p_Inp->ChromaWeightSupport == 1)
    {
      for (k = 0; k < 2; k++)
      {
        dc_org_UV[k] = ComputeImgSum(p_Vid->pImgOrg[k + 1], p_Vid->height_cr, p_Vid->width_cr);
      } 
    }

    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (n = 0; n < currSlice->listXsize[clist]; n++)
      {
        if ( wpxDetermineWP( currSlice, clist, n ) )
        {
          /* set all values to defaults */
          for (i = 0; i < 3; i++)
          {
            currSlice->wp_weight[clist][n][i] = default_weight[i];
            currSlice->wp_offset[clist][n][i] = 0;
            offset   [clist][n][i] = 0;
            weight   [clist][n][i] = default_weight[i];
          }
          // To simplify these computations we may wish to perform these after a reference is 
          // stored in the reference buffer and attach them to the storedimage structure!!!
          // Y
          tmpPtr = currSlice->listX[clist][n]->p_curr_img;
          dc_ref[clist][n] = ComputeImgSum(tmpPtr, p_Vid->height, p_Vid->width);

          if (dc_ref[clist][n] != 0.0)
            wf_weight = (short) (default_weight[0] * dc_org / dc_ref[clist][n] + 0.5);
          else
            wf_weight = default_weight[0];  // only used when reference picture is black
          wf_weight = sClip3(-128, 127, wf_weight);
          wf_offset = 0;

          //    printf("dc_org = %d, dc_ref = %d, weight[%d] = %d\n",dc_org, dc_ref[n],n,weight[n][0]);

          weight[clist][n][0] = wf_weight;
          offset[clist][n][0] = wf_offset;

          // UV
          if (p_Inp->ChromaWeightSupport == 1)
          {          
            for (k = 0; k < 2; k++)
            {        	
              tmpPtr = currSlice->listX[clist][n]->imgUV[k];
              dc_ref_UV[clist][n][k] = ComputeImgSum(tmpPtr, p_Vid->height_cr, p_Vid->width_cr);

              if (dc_ref_UV[clist][n][k] != 0.0)
                wf_weight = (short) (default_weight[k + 1] * dc_org_UV[k] / dc_ref_UV[clist][n][k] + 0.5);
              else
                wf_weight = default_weight[k + 1];  // only used when reference picture is black
              wf_weight = sClip3(-128, 127, wf_weight);
              wf_offset = 0;

              weight[clist][n][k + 1] = wf_weight;
              offset[clist][n][k + 1] = wf_offset;
            }
          }
          else
          {
            weight[clist][n][1] = default_weight[1];
            weight[clist][n][2] = default_weight[2];        
            offset[clist][n][1] = 0;
            offset[clist][n][2] = 0;
          }

          for (i = 0; i < 3; i++)
          {
            currSlice->wp_weight[clist][n][i] = weight[clist][n][i];
            currSlice->wp_offset[clist][n][i] = offset[clist][n][i];
#if DEBUG_WP
            printf("%d %d\n",currSlice->wp_weight[clist][index][comp],currSlice->wp_offset[clist][index][comp]);
#endif
          }
        }
      }
    }

    if (p_Vid->active_pps->weighted_bipred_idc != 1)
    {
      for (clist=0; clist<2 + list_offset; clist++)
      {
        for (index = 0; index < currSlice->listXsize[clist]; index++)
        {
          memcpy(currSlice->wp_weight[clist][index], default_weight, 3 * sizeof(short));
          memset(currSlice->wp_offset[clist][index], 0, 3 * sizeof(short));
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
#if DEBUG_WP
        printf ("bpw weight[%d][%d] = %d  , %d (%d %d %d) (%d %d) (%d %d)\n", i, j, currSlice->wbp_weight[0][i][j][0], currSlice->wbp_weight[1][i][j][0],
          p_Vid->enc_picture->poc,currSlice->listX[LIST_0][i]->poc, currSlice->listX[LIST_1][j]->poc,
          DistScaleFactor ,tx,tx,tx);
#endif
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

int TestWPPSliceAlg0(Slice *currSlice, int select_offset)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int i, j, k, n;

  int index;
  int comp;
  double dc_org = 0.0;
  double dc_org_UV[2] = {0.0};  
  double dc_ref[MAX_REFERENCE_PICTURES] = { 0.0 };
  double dc_ref_UV[MAX_REFERENCE_PICTURES][2] = { {0.0}};

  short default_weight[3];

  int list_offset   = ((p_Vid->mb_aff_frame_flag)&&(p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  short weight[2][MAX_REFERENCE_PICTURES][3];
  short offset[2][MAX_REFERENCE_PICTURES][3];
  //short wp_weight[6][MAX_REFERENCE_PICTURES][3];
  //short wp_offset[6][MAX_REFERENCE_PICTURES][3];
  int clist;
  int perform_wp = 0;
  imgpel **tmpPtr;

  short luma_log_weight_denom = 5;
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
        //wp_weight[i][j][n] = default_weight[n];
        //wp_offset[i][j][n] = 0;
        offset[i][j][n] = 0;
      }
    }
  }

  dc_org = ComputeImgSum(p_Vid->pCurImg, p_Vid->height, p_Vid->width);

  if (p_Inp->ChromaWeightSupport == 1)
  {
    for (k = 0; k < 2; k++)
    {
      dc_org_UV[k] = ComputeImgSum(p_Vid->pImgOrg[k + 1], p_Vid->height_cr, p_Vid->width_cr);
    } 
  }

  for (clist = 0; clist < 2 + list_offset; clist++)
  {
    for (n = 0; n < currSlice->listXsize[clist]; n++)
    {
      tmpPtr = currSlice->listX[clist][n]->p_curr_img;
      dc_ref[n] = ComputeImgSum(tmpPtr, p_Vid->height, p_Vid->width);

      if (p_Inp->ChromaWeightSupport == 1)
      {
        for (k = 0; k < 2; k++)
        {
          tmpPtr = currSlice->listX[clist][n]->imgUV[k];
          dc_ref_UV[n][k] = ComputeImgSum(tmpPtr, p_Vid->height_cr, p_Vid->width_cr);
        }        
      }

      if (select_offset == 0)
      {
        if (dc_ref[n] != 0.0)
          weight[clist][n][0] = (short) (default_weight[0] * dc_org / dc_ref[n] + 0.5);
        else
          weight[clist][n][0] = default_weight[0];  // only used when reference picture is black
        weight[clist][n][0] = sClip3(-128, 127, weight[clist][n][0]);

        if (p_Inp->ChromaWeightSupport == 1)
        {
          if (dc_ref_UV[n][0] != 0)
            weight[clist][n][1] = (short) (default_weight[1] * dc_org_UV[0] / dc_ref_UV[n][0] + 0.5);
          else
            weight[clist][n][1] = default_weight[1];  // only used when reference picture is black
          weight[clist][n][1] = sClip3(-128, 127, weight[clist][n][1]);

          if (dc_ref_UV[n][1] != 0)
            weight[clist][n][2] = (short) (default_weight[2] * dc_org_UV[1] / dc_ref_UV[n][1] + 0.5);
          else
            weight[clist][n][2] = default_weight[2];  // only used when reference picture is black
          weight[clist][n][2] = sClip3(-128, 127, weight[clist][n][2]);
        }
      }
      else
      {
        offset[clist][n][0] = (short) ((dc_org - dc_ref[n])/(p_Vid->size)+0.5);
        offset[clist][n][0] = (offset[clist][n][0]+((p_Vid->bitdepth_luma-8)>>1))>>(p_Vid->bitdepth_luma-8);
        offset[clist][n][0] = sClip3( -128, 127, offset[clist][n][0]);
        offset[clist][n][0] = offset[clist][n][0]<<(p_Vid->bitdepth_luma-8);
        weight[clist][n][0] = default_weight[0];

        if (p_Inp->ChromaWeightSupport == 1)
        {
          offset[clist][n][1] = (short) ((dc_org_UV[0] - dc_ref_UV[n][0])/(p_Vid->size_cr)+0.5);
          offset[clist][n][1] = (offset[clist][n][1] + ((p_Vid->bitdepth_chroma - 8)>>1))>>(p_Vid->bitdepth_chroma-8);
          offset[clist][n][1] = sClip3( -128, 127, offset[clist][n][1]);
          offset[clist][n][1] = offset[clist][n][1]<<(p_Vid->bitdepth_chroma - 8);

          weight[clist][n][1] = default_weight[1];

          offset[clist][n][2] = (short) ((dc_org_UV[1] - dc_ref_UV[n][1])/(p_Vid->size_cr)+0.5);
          offset[clist][n][2] = (offset[clist][n][2] + ((p_Vid->bitdepth_chroma - 8)>>1))>>(p_Vid->bitdepth_chroma-8);
          offset[clist][n][2] = sClip3( -128, 127, offset[clist][n][2]);
          offset[clist][n][2] = offset[clist][n][2]<<(p_Vid->bitdepth_chroma - 8);

          weight[clist][n][2] = default_weight[2];
        }
      }
    }
  }

  for (clist=0; clist<2 + list_offset; clist++)
  {
    for (index = 0; index < currSlice->listXsize[clist]; index++)
    {
      for (comp=0; comp < 3; comp ++)
      {
        int offset_test = p_Inp->RDPSliceBTest && p_Vid->active_sps->profile_idc != BASELINE
          ? iabs(offset[clist][index][comp]) > 2
          : offset[clist][index][comp] != 0;

        if (weight[clist][index][comp] != default_weight[0] || offset_test)
        {
          perform_wp = 1;
          break;
        }
      }
      if (perform_wp == 1) break;
    }
    if (perform_wp == 1) break;
  }

  return perform_wp;
}

/*!
************************************************************************
* \brief
*    TestWPBSliceAlg0:
*    Tests B slice weighting prediction
************************************************************************
*/
int TestWPBSliceAlg0(Slice *currSlice, int select_method)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int i, j, k, n;
#if (MVC_EXTENSION_ENABLE)
  int view_id = p_Vid->view_id;
#else
  int view_id = 0;
#endif

  int index;
  int comp;
  double dc_org = 0.0;
  double dc_org_UV[2] = { 0.0 };    
  double dc_ref[6][MAX_REFERENCE_PICTURES] = { {0.0} };  
  double dc_ref_UV[6][MAX_REFERENCE_PICTURES][2] = { {{0.0}} };

  short default_weight[3];
  // this needs to be fixed.
  int list_offset   = ((p_Vid->mb_aff_frame_flag)&&(p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  short weight[6][MAX_REFERENCE_PICTURES][3];
  short offset[6][MAX_REFERENCE_PICTURES][3];
  short im_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3];
  short wp_weight[6][MAX_REFERENCE_PICTURES][3];
  short wp_offset[6][MAX_REFERENCE_PICTURES][3];
  //short wbp_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3];

  int clist;
  short wf_weight, wf_offset;
  int perform_wp = 1;
  imgpel **tmpPtr;

  short luma_log_weight_denom = 5;
  short chroma_log_weight_denom = 5;

  default_weight[0] = 1 << luma_log_weight_denom;
  default_weight[1] = 1 << chroma_log_weight_denom;
  default_weight[2] = 1 << chroma_log_weight_denom;

  /* set all values to defaults */
  for (i = 0; i < 2 + list_offset; i++)
  {
    for (j = 0; j < currSlice->listXsize[i]; j++)
    {
      for (n = 0; n < 3; n++)
      {
        wp_weight[i][j][n] = default_weight[n];
        wp_offset[i][j][n] = 0;
        weight   [i][j][n] = default_weight[n];
        offset   [i][j][n] = 0;
      }
    }
  }

  if (select_method == 1) //! implicit mode
  {
    ComputeImplicitWeights(currSlice, default_weight, im_weight);

    perform_wp = 0;
    for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
    {
      for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
      {
        if (im_weight[1][i][j][0] != default_weight[0] || im_weight[1][i][j][0] != default_weight[0])
        {
          perform_wp = 1;
          break;
        }
      }
      if (perform_wp == 1) break;
    }
  }
  else
  {
    dc_org = ComputeImgSum(p_Vid->pCurImg, p_Vid->height, p_Vid->width);

    if (p_Inp->ChromaWeightSupport == 1)
    {
      for (k = 0; k < 2; k++)
      {
        dc_org_UV[k] = ComputeImgSum(p_Vid->pImgOrg[k + 1], p_Vid->height_cr, p_Vid->width_cr);
      } 
    }

    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (n = 0; n < currSlice->listXsize[clist]; n++)
      {
        // To simplify these computations we may wish to perform these after a reference is 
        // stored in the reference buffer and attach them to the storedimage structure!!!
        // Y
        tmpPtr = currSlice->listX[clist][n]->p_curr_img;
        dc_ref[clist][n] = ComputeImgSum(tmpPtr, p_Vid->height, p_Vid->width);

        if (dc_ref[clist][n] != 0.0)
          wf_weight = (short) (default_weight[0] * dc_org / dc_ref[clist][n] + 0.5);
        else
          wf_weight = default_weight[0];  // only used when reference picture is black
        wf_weight = sClip3(-128, 127, wf_weight);
        wf_offset = 0;

        weight[clist][n][0] = wf_weight;
        offset[clist][n][0] = wf_offset;

        // UV
        if (p_Inp->ChromaWeightSupport == 1)
        {          
          for (k = 0; k < 2; k++)
          {
            tmpPtr = currSlice->listX[clist][n]->imgUV[k];
            dc_ref_UV[clist][n][k] = ComputeImgSum(tmpPtr, p_Vid->height_cr, p_Vid->width_cr);

            if (dc_ref_UV[clist][n][k] != 0.0)
              wf_weight = (short) (default_weight[k + 1] * dc_org_UV[k] / dc_ref_UV[clist][n][k] + 0.5);
            else
              wf_weight = default_weight[k + 1];  // only used when reference picture is black
            wf_weight = sClip3(-128, 127, wf_weight);
            wf_offset = 0;

            weight[clist][n][k + 1] = wf_weight;
            offset[clist][n][k + 1] = wf_offset;
          }
        }
        else
        {
          weight[clist][n][1] = default_weight[1];
          weight[clist][n][2] = default_weight[2];        
          offset[clist][n][1] = 0;
          offset[clist][n][2] = 0;
        }
      }
    }

    if (select_method == 0) //! explicit mode
    {
      for (clist=0; clist<2 + list_offset; clist++)
      {
        for (index = 0; index < currSlice->listXsize[clist]; index++)
        {
          memcpy(p_Vid->currentSlice->wp_weight[clist][index], weight[clist][index], 3 * sizeof(short));
          memcpy(p_Vid->currentSlice->wp_offset[clist][index], offset[clist][index], 3 * sizeof(short));
        }
      }
    }
    else
    {
      for (clist=0; clist<2 + list_offset; clist++)
      {
        for (index = 0; index < currSlice->listXsize[clist]; index++)
        {
          memcpy(wp_weight[clist][index], default_weight, 3 * sizeof(short));
          memset(wp_offset[clist][index], 0, 3 * sizeof(short));
        }
      }
    }
  }

  if (select_method == 0) //! explicit mode
  {
    int active_refs[2];

    active_refs[0] = (p_Inp->B_List0_refs == 0 ? currSlice->listXsize[0] : imin(p_Inp->B_List0_refs[view_id], currSlice->listXsize[0]));
    active_refs[1] = (p_Inp->B_List1_refs == 0 ? currSlice->listXsize[1] : imin(p_Inp->B_List1_refs[view_id], currSlice->listXsize[1]));

    perform_wp = 0;
    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (index = 0; index < active_refs[clist]; index++)
      {
        for (comp=0; comp < 3; comp ++)
        {
          if (wp_weight[clist][index][comp] != default_weight[comp])
          {
            perform_wp = 1;
            break;
          }
        }
        if (perform_wp == 1) break;
      }
      if (perform_wp == 1) break;
    }
  }
  return perform_wp;
}
