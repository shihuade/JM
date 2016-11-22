/*!
*************************************************************************************
* \file wp_mciter.c
*
* \brief
*    Estimate weights for WP using iterative MC method
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*************************************************************************************
*/
#include "contributors.h"

#include <math.h>
#include <float.h>

#include "global.h"
#include "image.h"
#include "wp.h"

/*!
************************************************************************
* \brief
*    Estimates reference picture weighting factors for P slices
************************************************************************
*/

void EstimateWPPSliceAlg2(Slice *currSlice, int select_offset)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  double dc_org = 0.0;
  double dc_org_UV[2] = {0.0};
  double dc_ref[MAX_REFERENCE_PICTURES] = { 0.0 };
  double dc_ref_UV[MAX_REFERENCE_PICTURES][2] = { {0.0}};

  int i, n, k;
  short default_weight[3];
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
          offset[clist][n][0] = p_Vid->frameOffset[clist][n];

          offset[clist][n][0] = (offset[clist][n][0]+((p_Vid->bitdepth_luma-8)>>1))>>(p_Vid->bitdepth_luma - 8);
          offset[clist][n][0] = sClip3( -128, 127, offset[clist][n][0]);
          offset[clist][n][0] = offset[clist][n][0]<<(p_Vid->bitdepth_luma-8);
          weight[clist][n][0] = default_weight[0];

          if (p_Inp->ChromaWeightSupport == 1)
          {
            offset[clist][n][1] = (short) ((dc_org_UV[0] - dc_ref_UV[n][0])/(p_Vid->size_cr)+0.5);
            offset[clist][n][1] = (offset[clist][n][1] + ((p_Vid->bitdepth_chroma - 8)>>1))>>(p_Vid->bitdepth_chroma - 8);
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
void EstimateWPBSliceAlg2(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int i, j, k, n;

  short tx, tb, td, DistScaleFactor;

  int index;
  int comp;
  //double dc_org = 0.0;
  double dc_org_UV[2] = { 0.0 };
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
    for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
    {
      for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
      {
        td = (short) iClip3(-128, 127,(currSlice->listX[LIST_1][j]->poc - currSlice->listX[LIST_0][i]->poc));
        tb = (short)iClip3(-128, 127,(p_Vid->enc_picture->poc - currSlice->listX[LIST_0][i]->poc));
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
            DistScaleFactor = (short) iClip3(-1024, 1023, (tx*tb + 32 )>>6);
            im_weight[1][i][j][comp] = DistScaleFactor>>2;
            if (im_weight[1][i][j][comp] < -64 || im_weight[1][i][j][comp] >128)
              im_weight[1][i][j][comp] = default_weight[comp];
            im_weight[0][i][j][comp] = 64 - im_weight[1][i][j][comp];
          }
        }
#if DEBUG_WP
        printf ("%d imp weight[%d][%d] = %d  , %d (%d %d %d) (%d %d) (%d %d)\n",p_Vid->enc_picture->poc, i, j,  im_weight[0][i][j][0], im_weight[1][i][j][0],
          p_Vid->enc_picture->poc,currSlice->listX[LIST_0][i]->poc, currSlice->listX[LIST_1][j]->poc,
          DistScaleFactor ,tx,td,tb);
#endif
      }
    }

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
    //dc_org = ComputeImgSum(p_Vid->pCurImg, p_Vid->height, p_Vid->width);

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

          offset[clist][n][0] = wf_offset = p_Vid->frameOffset[clist][n];
          weight[clist][n][0] = wf_weight = default_weight[0];


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

int TestWPPSliceAlg2(Slice *currSlice, int select_offset)
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
  short wp_weight[6][MAX_REFERENCE_PICTURES][3];
  short wp_offset[6][MAX_REFERENCE_PICTURES][3];
  int clist;
  int perform_wp = 0;
  imgpel **tmpPtr;

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
        wp_weight[i][j][n] = default_weight[n];
        wp_offset[i][j][n] = 0;
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
        offset[clist][n][0] = p_Vid->frameOffset[clist][n]; 

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
*    TestWPBSliceAlg2:
*    Tests B slice weighting prediction
************************************************************************
*/
int TestWPBSliceAlg2(Slice *currSlice, int select_method)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int i, j, k, n;
#if (MVC_EXTENSION_ENABLE)
  int view_id = p_Vid->view_id;
#else
  int view_id = 0;
#endif

  short tx, td, tb, DistScaleFactor;

  int index;
  int comp;
  double dc_org = 0.0;
  double dc_org_UV[2] = { 0.0 };    
  double dc_ref_UV[6][MAX_REFERENCE_PICTURES][2] = { {{0.0}} };

  short default_weight[3];
  // this needs to be fixed.
  int list_offset   = ((p_Vid->mb_aff_frame_flag)&&(p_Vid->mb_data[p_Vid->current_mb_nr].mb_field))? (p_Vid->current_mb_nr & 0x01) ? 4 : 2 : 0;
  short weight[6][MAX_REFERENCE_PICTURES][3];
  short offset[6][MAX_REFERENCE_PICTURES][3];
  short im_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3];
  short wp_weight[6][MAX_REFERENCE_PICTURES][3];
  short wp_offset[6][MAX_REFERENCE_PICTURES][3];
  short wbp_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3];

  int clist;
  short wf_weight, wf_offset;
  int perform_wp = 0;
  imgpel **tmpPtr;

  short luma_log_weight_denom   = 5;
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
        offset   [i][j][n] = 0;
        weight   [i][j][n] = default_weight[n];
      }
    }
  }

  for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
  {
    for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
    {
      td = (short) iClip3(-128, 127,(currSlice->listX[LIST_1][j]->poc - currSlice->listX[LIST_0][i]->poc));
      tb = (short) iClip3(-128, 127,(p_Vid->enc_picture->poc - currSlice->listX[LIST_0][i]->poc));
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


  if (select_method == 1) //! implicit mode
  {
    for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
    {
      for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
      {
        for (comp = 0; comp < 3; comp++)
        {
          wbp_weight[1][i][j][comp] = im_weight[1][i][j][comp] ;
          wbp_weight[0][i][j][comp] = im_weight[0][i][j][comp];
        }
      }
    }

    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (index = 0; index < currSlice->listXsize[clist]; index++)
      {
        for (comp = 0; comp < 3; comp++)
        {
          wp_weight[clist][index][comp] = default_weight[comp];
          wp_offset[clist][index][comp] = 0;
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
        offset[clist][n][0] = wf_offset = p_Vid->frameOffset[clist][n];          
        weight[clist][n][0] = wf_weight = default_weight[0];         

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

    for (i = 0; i < currSlice->listXsize[LIST_0]; i++)
    {
      for (j = 0; j < currSlice->listXsize[LIST_1]; j++)
      {
        for (comp = 0; comp < 3; comp++)
        {
          wbp_weight[0][i][j][comp] = wp_weight[0][i][comp];
          wbp_weight[1][i][j][comp] = wp_weight[1][j][comp];
        }
#if DEBUG_WP
        printf ("bpw weight[%d][%d] = %d  , %d (%d %d %d) (%d %d) (%d %d)\n", i, j, p_Vid->currentSlice->wbp_weight[0][i][j][0], p_Vid->currentSlice->wbp_weight[1][i][j][0],
        p_Vid->enc_picture->poc,currSlice->listX[LIST_0][i]->poc, currSlice->listX[LIST_1][j]->poc,
        DistScaleFactor ,tx,tx,tx);
#endif
      }
    }
  }

  if (select_method == 0) //! implicit mode
  {
    int active_refs[2];

    active_refs[0] = (p_Inp->B_List0_refs == 0 ? currSlice->listXsize[0] : imin(p_Inp->B_List0_refs[view_id], currSlice->listXsize[0]));
    active_refs[1] = (p_Inp->B_List1_refs == 0 ? currSlice->listXsize[1] : imin(p_Inp->B_List1_refs[view_id], currSlice->listXsize[1]));

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

void compute_offset(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  Macroblock *currMB;
  int i, j, x, y, xj, yi, temp, valOrg;
  int mvx=0,  mvy=0;
  int ref_frame=0;
  int subblock=0;
  int x_orig, y_orig;       
  int x_pos, y_pos;
  int out4Y_width  = (p_Vid->width  + IMG_PAD_SIZE_X) * 4 - 1;  
  int out4Y_height = (p_Vid->height + IMG_PAD_SIZE_Y) * 4 - 1;

  int frame, list, offset; 
  double dtemp;
  int numlists  = (p_Vid->type == B_SLICE) ? 2 : 1;


  for(list = 0; list < 2; list++)
  {
    for(frame = 0; frame < MAX_REFERENCE_PICTURES; frame++)
    {
      p_Vid->frameOffsetTotal[list][frame] = 0;
      p_Vid->frameOffsetCount[list][frame] = 0;
    }
  }


  for(i=0; i<p_Vid->height >> 4; i++) //y
  {
    for(j=0; j<p_Vid->width >> 4; j++)  //x
    {
      { 
        currMB = &p_Vid->mb_data[((i*p_Vid->width) >> 4)+j];
        if(is_intra(currMB)) //intra macroblocks are not used for calculation of the filter coeffs.
          continue;

        x_orig = MB_BLOCK_SIZE*j;
        y_orig = MB_BLOCK_SIZE*i;

        for(subblock = 0; subblock < 16; subblock++)
        {
          //List 0
          x = x_orig+4*(subblock & 0x03);
          y = y_orig+4*(subblock >> 2);
          mvx = motion[y >> 2][x >> 2].mv[LIST_0].mv_x;
          mvy = motion[y >> 2][x >> 2].mv[LIST_0].mv_y;
          ref_frame = motion[y >> 2][x >> 2].ref_idx[LIST_0];

          if(ref_frame != -1)
          {
            for(yi = 0; yi < 4; yi++)
            {    //y
              for(xj = 0; xj < 4; xj++)
              {  //x
                valOrg   = p_Vid->pCurImg[y+yi][x+xj];

                //y_pos = imax(0,imin(out4Y_height,4*(y+yi)+4*IMG_PAD_SIZE+mvy));
                y_pos = imax(-4*IMG_PAD_SIZE_Y, imin(out4Y_height, 4*(y+yi)+mvy));
                //x_pos = imax(0,imin(out4Y_width, 4*(x+xj)+4*IMG_PAD_SIZE+mvx));
                x_pos = imax(-4*IMG_PAD_SIZE_X, imin(out4Y_width, 4*(x+xj)+mvx));

                temp=currSlice->listX[LIST_0][ref_frame]->p_curr_img_sub[(y_pos & 0x03)][(x_pos & 0x03)][y_pos >> 2][x_pos >> 2];
                p_Vid->frameOffsetTotal[LIST_0][ref_frame]+=(valOrg-temp);
                p_Vid->frameOffsetCount[LIST_0][ref_frame]++;          
              }
            }
          } 


          //List 1
          mvx = motion[y >> 2][x >> 2].mv[LIST_1].mv_x;
          mvy = motion[y >> 2][x >> 2].mv[LIST_1].mv_y;
          ref_frame = motion[y >> 2][x >> 2].ref_idx[LIST_1];
          if(ref_frame != -1)
          {
            for(yi = 0; yi < 4; yi++)
            {    //y
              for(xj = 0; xj < 4; xj++)
              {  //x
                valOrg   = p_Vid->pCurImg[y+yi][x+xj];

                //y_pos = imax(0,imin(out4Y_height,4*(y+yi)+4*IMG_PAD_SIZE+mvy));
                y_pos = imax(-4*IMG_PAD_SIZE_Y, imin(out4Y_height, 4*(y+yi) + mvy));
                //x_pos = imax(0, imin(out4Y_width, 4*(x+xj)+4*IMG_PAD_SIZE+mvx));
                x_pos = imax(-4*IMG_PAD_SIZE_X, imin(out4Y_width, 4*(x+xj) + mvx));

                temp=currSlice->listX[LIST_0][ref_frame]->p_curr_img_sub[(y_pos & 0x03)][(x_pos & 0x03)][y_pos >> 2][x_pos >> 2];
                p_Vid->frameOffsetTotal[LIST_1][ref_frame]+=(valOrg-temp);
                p_Vid->frameOffsetCount[LIST_1][ref_frame]++;          
              }
            }
          }
        }//  for(subblock = 0; subblock < 16; subblock++)
      }
    }

    for(list = 0; list < numlists; list++)
    {
      for(frame = 0; frame < currSlice->listXsize[list]; frame++)
      {
        dtemp=(double)p_Vid->frameOffsetTotal[list][frame];

        if (p_Vid->frameOffsetCount[list][frame]>0)
        {
          offset=(int)(fabs(dtemp)/(double)p_Vid->frameOffsetCount[list][frame]+0.5);
          if (p_Vid->frameOffsetTotal[list][frame]>=0)
          {
            p_Vid->frameOffset[list][frame] = (short) offset;
          }
          else
          {
            p_Vid->frameOffset[list][frame] = (short) -offset;
          }
          //printf("list %d frame %d offset %d frameOffsetCount %d, frameOffsetTotal %d\n", list, frame, p_Vid->frameOffset[list][frame], p_Vid->frameOffsetCount[list][frame], p_Vid->frameOffsetTotal[list][frame]);
        }
        //else
        //{
        //printf("list %d frame %d, frameOffsetCount=0\n", list, frame);
        //}
      }
    }
  }
}
