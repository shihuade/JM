/*!
 *************************************************************************************
 * \file mc_prediction_otf.c
 *
 * \brief
 *    Motion Compensation
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Julien Le Tanou 
 *
 *************************************************************************************
 */

#include "contributors.h"

#include <stdlib.h>
#include <assert.h>
#include <limits.h>
#include <math.h>

#include "global.h"
#include "block.h"

#include "macroblock.h"
#include "mc_prediction.h"
#include "mc_prediction_otf.h"

#include "refbuf.h"
#include "image.h"
#include "mb_access.h"
#include "me_distortion.h"
#include "get_block_otf.h"

/*!
 ************************************************************************
 * \brief
 *    block single list prediction
 ************************************************************************
 */
static void mc_prediction(imgpel** mb_pred, imgpel* lpred, int block_size_y, int block_size_x, int ioff)
{
  int j;
  for (j = 0; j < block_size_y; j++)
  {
    memcpy(&(mb_pred[j][ioff]), lpred, block_size_x * sizeof(imgpel));
    lpred += block_size_x;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Weighted Prediction
 ************************************************************************
 */
static void weighted_mc_prediction(imgpel** mb_pred, 
                                   imgpel* lpred, 
                                   int block_size_y, 
                                   int block_size_x,
                                   int ioff, 
                                   int max_imgpel_value,
                                   short wp_scale, 
                                   short wp_offset,
                                   short wp_round, 
                                   short weight_denom)
{
  int i, j;
  int result;
  int block_x4 = ioff + block_size_x;

  for   (j = 0; j < block_size_y; j++)
  {
    for (i = ioff; i < block_x4; i++)
    {
      result = rshift_rnd((wp_scale * *lpred++), weight_denom) + wp_offset;      
      mb_pred[j][i] = (imgpel) iClip1( max_imgpel_value, result);
     }
  }
}

/*!
 ************************************************************************
 * \brief
 *    block biprediction
 ************************************************************************
 */
static void bi_prediction(imgpel **mb_pred, 
                          imgpel *l0pred, 
                          imgpel *l1pred, 
                          int block_size_y, 
                          int block_size_x, 
                          int ioff)
{
  int i, j;
  int block_x4 = ioff + block_size_x;

  for   (j = 0; j < block_size_y; j++)
  {
    for (i=ioff; i<block_x4; i++)
      mb_pred[j][i] = (*l0pred++ + *l1pred++ + 1) >> 1;
  }
}



/*!
 ************************************************************************
 * \brief
 *    block weighted biprediction
 ************************************************************************
 */
static void weighted_bi_prediction(imgpel** mb_pred, 
                                   imgpel *block_l0, 
                                   imgpel *block_l1, 
                                   int block_size_y, 
                                   int block_x, 
                                   int block_size_x,
                                   int max_imgpel_value,
                                   int wp_scale_l0, 
                                   int wp_scale_l1, 
                                   int wp_offset, 
                                   int weight_denom)
{
  int i, j, result;
  int block_x4 = block_x + block_size_x;

  for   (j = 0; j< block_size_y; j++)
  {
    for (i=block_x; i<block_x4; i++)
    {
      result = rshift_rnd_sf((wp_scale_l0 * *(block_l0++) + wp_scale_l1 * *(block_l1++)),  weight_denom);
      mb_pred[j][i] = (imgpel) iClip1( max_imgpel_value, result + wp_offset); 
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Predict one Luma block on-the-fly
 ************************************************************************
 */
void luma_prediction_otf ( Macroblock* currMB, //!< Current Macroblock
                     int   block_x,      //!< relative horizontal block coordinate of block
                     int   block_y,      //!< relative vertical   block coordinate of block
                     int   block_size_x, //!< relative horizontal block coordinate of block
                     int   block_size_y, //!< relative vertical   block coordinate of block
                     int   p_dir,        //!< prediction direction (0=list0, 1=list1, 2=bipred)
                     int   list_mode[2], //!< list prediction mode (1-7, 0=DIRECT)
                     char  *ref_idx,     //!< reference pictures
                     short bipred_me     //!< use bi prediction mv (0=no bipred, 1 = use set 1, 2 = use set 2)
                     )
{
  VideoParameters *p_Vid     = currMB->p_Vid;
  Slice           *currSlice = currMB->p_Slice;
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
  imgpel l0_pred[MB_PIXELS];
  imgpel l1_pred[MB_PIXELS];
  int    tmp_pred[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ]; // temporary pred block to compute on-the-fly interpolation 

  int  pic_opix_x   = ((currMB->pix_x + block_x) << 2);
  int  pic_opix_y   = ((currMB->opix_y + block_y) << 2);
  int  bx           = block_x >> 2;
  int  by           = block_y >> 2;
  MotionVector ***** mv_array = currSlice->all_mv;
  MotionVector *curr_mv = NULL;
  imgpel **mb_pred = currSlice->mb_pred[0];
  int  apply_weights = ( (currSlice->weighted_prediction == 1) || (currSlice->weighted_prediction == 2 && p_dir == 2) );

  if (bipred_me && ref_idx[0] == 0 && ref_idx[1] == 0 && p_dir == 2 && is_bipred_enabled(p_Vid, list_mode[0]) && is_bipred_enabled(p_Vid, list_mode[1]))
    mv_array = currSlice->bipred_mv[bipred_me - 1]; 

  switch (p_dir)
  {
  case 0:
    curr_mv = &mv_array[LIST_0][(short) ref_idx[0]][list_mode[0]][by][bx];
    p_Dpb->pf_get_block_luma (p_Vid, l0_pred, tmp_pred, pic_opix_x + curr_mv->mv_x, pic_opix_y + curr_mv->mv_y, block_size_x, block_size_y, currSlice->listX[LIST_0 + currMB->list_offset][(short) ref_idx[0]], 0 );
    break;
  case 1:
    curr_mv = &mv_array[LIST_1][(short) ref_idx[1]][list_mode[1]][by][bx];
    p_Dpb->pf_get_block_luma (p_Vid, l1_pred, tmp_pred, pic_opix_x + curr_mv->mv_x, pic_opix_y + curr_mv->mv_y, block_size_x, block_size_y, currSlice->listX[LIST_1 + currMB->list_offset][(short)ref_idx[1]], 0 );
    break;
  case 2:
    curr_mv = &mv_array[LIST_0][(short) ref_idx[0]][list_mode[0]][by][bx];
    p_Dpb->pf_get_block_luma (p_Vid, l0_pred, tmp_pred, pic_opix_x + curr_mv->mv_x, pic_opix_y + curr_mv->mv_y, block_size_x, block_size_y, currSlice->listX[LIST_0 + currMB->list_offset][(short)ref_idx[0]], 0);
    curr_mv = &mv_array[LIST_1][(short) ref_idx[1]][list_mode[1]][by][bx];
    p_Dpb->pf_get_block_luma (p_Vid, l1_pred, tmp_pred, pic_opix_x + curr_mv->mv_x, pic_opix_y + curr_mv->mv_y, block_size_x, block_size_y, currSlice->listX[LIST_1 + currMB->list_offset][(short)ref_idx[1]], 0);
    break;
  default:
    break;
  }

  if (apply_weights)
  {
    if (p_dir==2)
    {
      weighted_bi_prediction(&mb_pred[block_y], l0_pred, l1_pred, block_size_y, block_x, block_size_x, 
        p_Vid->max_imgpel_value,
        currSlice->wbp_weight[0][(short)ref_idx[0]][(short)ref_idx[1]][0], currSlice->wbp_weight[1][(short)ref_idx[0]][(short)ref_idx[1]][0],
        (currSlice->wp_offset[0][(short)ref_idx[0]][0] + currSlice->wp_offset[1][(short)ref_idx[1]][0] + 1)>>1, 
         currSlice->luma_log_weight_denom + 1);
    }
    else if (p_dir==0)
    {
      weighted_mc_prediction(&mb_pred[block_y], l0_pred, block_size_y, block_size_x, block_x,
        p_Vid->max_imgpel_value,
        currSlice->wp_weight[0][(short)ref_idx[0]][0], currSlice->wp_offset[0][(short)ref_idx[0]][0], currSlice->wp_luma_round, currSlice->luma_log_weight_denom);
    }
    else // (p_dir==1)
    {
      weighted_mc_prediction(&mb_pred[block_y], l1_pred, block_size_y, block_size_x, block_x,
        p_Vid->max_imgpel_value,
        currSlice->wp_weight[1][(short)ref_idx[1]][0], currSlice->wp_offset[1][(short)ref_idx[1]][0], currSlice->wp_luma_round, currSlice->luma_log_weight_denom);
    }
  }
  else
  {
    if (p_dir==2)
    {
      bi_prediction(&mb_pred[block_y], l0_pred, l1_pred, block_size_y, block_size_x, block_x);    
    }
    else if (p_dir==0)
    {
      mc_prediction(&mb_pred[block_y], l0_pred, block_size_y, block_size_x, block_x);
    }
    else // (p_dir==1)
    {
      mc_prediction(&mb_pred[block_y], l1_pred, block_size_y, block_size_x, block_x);
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Predict one Luma block on-the-fly 
 ************************************************************************
 */
void luma_prediction_bi_otf ( Macroblock* currMB, //!< Current Macroblock
                       int   block_x,      //!< relative horizontal block coordinate of 4x4 block
                       int   block_y,      //!< relative vertical   block coordinate of 4x4 block
                       int   block_size_x, //!< horizontal block size
                       int   block_size_y, //!< vertical   block size
                       int   l0_mode,      //!< list0 prediction mode (1-7, 0=DIRECT if l1_mode=0)
                       int   l1_mode,      //!< list1 prediction mode (1-7, 0=DIRECT if l0_mode=0)
                       short l0_ref_idx,   //!< reference frame for list0 prediction (-1: Intra4x4 pred. with l0_mode)
                       short l1_ref_idx,   //!< reference frame for list1 prediction 
                       int   list          //!< current list for prediction.
                       )
{
  VideoParameters *p_Vid     = currMB->p_Vid;
  Slice           *currSlice = currMB->p_Slice;
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];

  imgpel l0_pred[MB_PIXELS];
  imgpel l1_pred[MB_PIXELS];

  int    tmp_pred[ (MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5) ]; // temporary pred block for on-the-fly interpolation calculation
  int  pic_opix_x   = ((currMB->pix_x + block_x) << 2);
  int  pic_opix_y   = ((currMB->opix_y + block_y) << 2);
  int  bx        = block_x >> 2;
  int  by        = block_y >> 2;

  int  apply_weights = ( currSlice->weighted_prediction != 0 );

  MotionVector *****mv_array = currSlice->bipred_mv[list]; 
  MotionVector *mv_arrayl0 = &mv_array[LIST_0][l0_ref_idx][l0_mode][by][bx];
  MotionVector *mv_arrayl1 = &mv_array[LIST_1][l1_ref_idx][l1_mode][by][bx];
  imgpel **mb_pred = currSlice->mb_pred[0];

  p_Dpb->pf_get_block_luma(p_Vid, l0_pred, tmp_pred, pic_opix_x + mv_arrayl0->mv_x, pic_opix_y + mv_arrayl0->mv_y, block_size_x, block_size_y, currSlice->listX[0+currMB->list_offset][l0_ref_idx], 0);
  p_Dpb->pf_get_block_luma(p_Vid, l1_pred, tmp_pred, pic_opix_x + mv_arrayl1->mv_x, pic_opix_y + mv_arrayl1->mv_y, block_size_x, block_size_y, currSlice->listX[1+currMB->list_offset][l1_ref_idx], 0);

  if (apply_weights)
  {
    weighted_bi_prediction(&mb_pred[block_y], l0_pred, l1_pred, block_size_y, block_x, block_size_x, 
      p_Vid->max_imgpel_value,
      currSlice->wbp_weight[0][l0_ref_idx][l1_ref_idx][0], currSlice->wbp_weight[1][l0_ref_idx][l1_ref_idx][0],
      (currSlice->wp_offset[0][l0_ref_idx][0] + currSlice->wp_offset[1][l1_ref_idx][0] + 1)>>1, 
      currSlice->luma_log_weight_denom + 1);
  }
  else
  {
    bi_prediction(&mb_pred[block_y], l0_pred, l1_pred, block_size_y, block_size_x, block_x);
  }
}

/*!
 ************************************************************************
 * \brief
 *    Predict one chroma block on-the-fly
 ************************************************************************
 */
void chroma_prediction_otf ( Macroblock* currMB, // <-- Current Macroblock
                       int   uv,            // <-- colour component
                       int   block_x,       // <-- relative horizontal block coordinate of block
                       int   block_y,       // <-- relative vertical   block coordinate of block
                       int   block_size_x,  // <-- relative horizontal block coordinate of block
                       int   block_size_y,  // <-- relative vertical   block coordinate of block                        
                       int   p_dir,         // <-- prediction direction (0=list0, 1=list1, 2=bipred)
                       int   l0_mode,       // <-- list0  prediction mode (1-7, 0=DIRECT if l1_mode=0)
                       int   l1_mode,       // <-- list1 prediction mode (1-7, 0=DIRECT if l0_mode=0)
                       short l0_ref_idx,    // <-- reference frame for list0 prediction (if (<0) -> intra prediction)
                       short l1_ref_idx,    // <-- reference frame for list1 prediction 
                       short bipred_me      // <-- use bi prediction mv (0=no bipred, 1 = use set 1, 2 = use set 2)
                       )    
{
  VideoParameters *p_Vid     = currMB->p_Vid;
  Slice           *currSlice = currMB->p_Slice;
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];

  imgpel l0_pred[MB_PIXELS];
  imgpel l1_pred[MB_PIXELS];
  int    tmp_pred[(MB_BLOCK_SIZE+5)*(MB_BLOCK_SIZE+5)];

  int  pic_opix_x   = ((currMB->pix_c_x + block_x) << 2);
  int  pic_opix_y   = ((currMB->opix_c_y + block_y) << 2);

  int  bx           = block_x >> 2;
  int  by           = block_y >> 2;
  MotionVector ***** mv_array = currSlice->all_mv;    
  int uv_comp = uv + 1;
  imgpel **mb_pred = currSlice->mb_pred[ uv_comp];

  int  apply_weights = ( (currSlice->weighted_prediction == 1) || (currSlice->weighted_prediction == 2 && p_dir == 2) );

  if (bipred_me && l0_ref_idx == 0 && l1_ref_idx == 0 && p_dir == 2 && is_bipred_enabled(p_Vid, l0_mode)  && is_bipred_enabled(p_Vid, l1_mode))
    mv_array = currSlice->bipred_mv[bipred_me - 1]; 

  //===== INTER PREDICTION =====
  switch (p_dir)
  {
  case 0:
    p_Dpb->pf_get_block_chroma[OTF_MC] (p_Vid, l0_pred, tmp_pred, pic_opix_x + mv_array[LIST_0][l0_ref_idx][l0_mode][by][bx].mv_x, pic_opix_y + mv_array[LIST_0][l0_ref_idx][l0_mode][by][bx].mv_y, block_size_x, block_size_y, currSlice->listX[0+currMB->list_offset][l0_ref_idx], uv+1 );
    break;
  case 1: 
    p_Dpb->pf_get_block_chroma[OTF_MC] (p_Vid, l1_pred, tmp_pred, pic_opix_x + mv_array[LIST_1][l1_ref_idx][l1_mode][by][bx].mv_x, pic_opix_y + mv_array[LIST_1][l1_ref_idx][l1_mode][by][bx].mv_y, block_size_x, block_size_y, currSlice->listX[1+currMB->list_offset][l1_ref_idx], uv+1 );
    break;
  case 2:
    p_Dpb->pf_get_block_chroma[OTF_MC] (p_Vid, l0_pred, tmp_pred, pic_opix_x + mv_array[LIST_0][l0_ref_idx][l0_mode][by][bx].mv_x, pic_opix_y + mv_array[LIST_0][l0_ref_idx][l0_mode][by][bx].mv_y, block_size_x, block_size_y, currSlice->listX[0+currMB->list_offset][l0_ref_idx], uv+1 );
    p_Dpb->pf_get_block_chroma[OTF_MC] (p_Vid, l1_pred, tmp_pred, pic_opix_x + mv_array[LIST_1][l1_ref_idx][l1_mode][by][bx].mv_x, pic_opix_y + mv_array[LIST_1][l1_ref_idx][l1_mode][by][bx].mv_y, block_size_x, block_size_y, currSlice->listX[1+currMB->list_offset][l1_ref_idx], uv+1 );
    break;
  default:
    break;
  }

  if (apply_weights)
  {
    if (p_dir==2)
    {
      weighted_bi_prediction(&mb_pred[block_y], l0_pred, l1_pred, block_size_y, block_x, block_size_x, 
      p_Vid->max_pel_value_comp[1],
        currSlice->wbp_weight[0][l0_ref_idx][l1_ref_idx][uv_comp], currSlice->wbp_weight[1][l0_ref_idx][l1_ref_idx][uv_comp],
        (currSlice->wp_offset[0][l0_ref_idx][uv_comp] + currSlice->wp_offset[1][l1_ref_idx][uv_comp] + 1)>>1, 
         currSlice->chroma_log_weight_denom + 1);
    }
    else if (p_dir==0)
    {
      weighted_mc_prediction(&mb_pred[block_y], l0_pred, block_size_y, block_size_x, block_x,
      p_Vid->max_pel_value_comp[1],
        currSlice->wp_weight[0][l0_ref_idx][uv_comp], currSlice->wp_offset[0][l0_ref_idx][uv_comp], currSlice->wp_chroma_round, currSlice->chroma_log_weight_denom );
    }
    else // (p_dir==1)
    {
      weighted_mc_prediction(&mb_pred[block_y], l1_pred, block_size_y, block_size_x, block_x,
      p_Vid->max_pel_value_comp[1],
        currSlice->wp_weight[1][l1_ref_idx][uv_comp], currSlice->wp_offset[1][l1_ref_idx][uv_comp], currSlice->wp_chroma_round, currSlice->chroma_log_weight_denom );
    }
  }
  else
  {
    if (p_dir==2)
    {
      bi_prediction(&mb_pred[block_y], l0_pred, l1_pred, block_size_y, block_size_x, block_x);
    }
    else if (p_dir==0)
    {
      mc_prediction(&mb_pred[block_y], l0_pred, block_size_y, block_size_x, block_x);
    }
    else // (p_dir==1)
    {
      mc_prediction(&mb_pred[block_y], l1_pred, block_size_y, block_size_x, block_x);
    }
  }
}

