/*!
 *************************************************************************************
 * \file errdo_dist_estimation_multihyp.c
 * \brief
 *    Expected distortion estimation using multiple decoder based method
 *    refer to the paper "Optimized transmission of H.26L/JVT coded video over packet-lossy networks"
 *    by T. Stockhammer, T. Wiegand, and S. Wenger in 2002
 * \date
 *    October 22nd, 2001
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and
 *    affiliation details)
 *    - Dimitrios Kontopodis                    <dkonto@eikon.tum.de>
 *    Code revamped July 2008 by:
 *    - Peshala Pahalawatta (ppaha@dolby.com)
 *    - Alexis Tourapis (atour@dolby.com)
 *    Code modularized to support more distortion estimation algorithms in June 2009 by 
 *    - Zhifeng Chen (zzchen@dolby.com)
 *************************************************************************************
 */

//For distortion estimation
#include "global.h"
#include "mbuffer.h"
#include "errdo.h"
#include "md_distortion.h"
#include "md_common.h"
#include "lln_mc_prediction.h"

static void add_residue     (Macroblock *currMB, StorablePicture *enc_pic, int decoder, int pl, int block8x8, int x_size, int y_size);
static void Build_Status_Map(VideoParameters *p_Vid, InputParameters *p_Inp, byte **s_map);
static void get_predicted_mb(Macroblock *currMB, StorablePicture *enc_pic, int decoder);
static void copy_conceal_mb (Macroblock *currMB, StorablePicture *enc_pic, int decoder, int mb_error, StorablePicture* refPic);
static void decode_one_b8block      (Macroblock* currMB, StorablePicture *enc_pic, int decoder, int block8x8, short mv_mode, int pred_dir);
static void decode_one_mb           (Macroblock* currMB, StorablePicture *enc, int decoder);

extern void UpdateDecoders          (VideoParameters *p_Vid, InputParameters *p_Inp, StorablePicture *enc_pic);
extern void DeblockFrame(VideoParameters *p_Vid, imgpel **, imgpel ***);

/*!
 *************************************************************************************
 * \brief
 *  Performs distortion estimation using multiple decoder based algorithm.
 *
 *************************************************************************************
 */
distblk errdo_distortion_estimation_multihyp(Macroblock *currMB, int block, int block_size, short mode, short pdir, distblk min_rdcost)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  seq_parameter_set_rbsp_t *active_sps = p_Vid->active_sps;

  int  pax     = 8*(block & 0x01);
  int  pay     = 8*(block >> 1);
  distblk distortion=0;
  distblk temp_dist, ddistortion = 0;
  int  k;

  //Note that in rdcost_for_8x8blocks() no chroma distortion is calculated
  //Refer to the function reset_adaptive_rounding(), 
  //maxplane = (p_Vid->yuv_format == YUV400)?  1 : 3; for p_Vid->ARCofAdj4x4 
  //maxplane = (p_Vid->yuv_format == YUV400)?  1 : (p_Vid->P444_joined ? 3 : 1); for p_Vid->ARCofAdj8x8
  if (mode >= 4 && mode <= 7) //to be called by rdcost_for_8x8blocks()
  {
    //===== get residue =====
    // We need the reconstructed prediction residue for the simulated decoders.
    //compute_residue_b8block (p_Vid, &p_Vid->enc_picture->p_img[0][currMB->pix_y], p_Vid->p_decs->res_img[0], block, -1);
    errdo_compute_residue (currMB, &p_Vid->enc_picture->p_img[0][currMB->pix_y], p_Vid->p_decs->res_img[0], currSlice->mb_pred[0], block, 8);

    //=====   GET DISTORTION
    /*
    //To keep exactly same as former version, below lines are used
    for (k=0; k<p_Inp->NoOfDecoders ;k++)
    {
    decode_one_b8block (currMB, p_Vid->enc_picture, k, block, mode, pdir);
    distortion += compute_SSE8x8(&p_Vid->pCurImg[currMB->opix_y + pay], &p_Vid->enc_picture->p_dec_img[0][k][currMB->opix_y + pay], currMB->pix_x + pax, currMB->pix_x + pax);
    }
    distortion /= p_Inp->NoOfDecoders;
    */

    for (k = 0; k < p_Inp->NoOfDecoders; k++)
    {
      decode_one_b8block (currMB, p_Vid->enc_picture, k, block, mode, pdir);
      ddistortion += compute_SSE8x8(&p_Vid->pCurImg[currMB->opix_y + pay], &p_Vid->enc_picture->de_mem->p_dec_img[0][k][currMB->opix_y + pay], currMB->pix_x + pax, currMB->pix_x + pax);

      {
        temp_dist = min_rdcost * p_Inp->NoOfDecoders;
      }
      if (ddistortion > temp_dist)
      {
        //distortion = (distblk) ((ddistortion) / (k+1) + 0.5);
        return DISTBLK_MAX;
      }
    }

    distortion = (distblk) (ddistortion / p_Inp->NoOfDecoders);
  }
  else if (mode != P8x8)  //to be called by RDCost_for_macroblocks()
  {
    errdo_compute_residue (currMB, &p_Vid->enc_picture->p_curr_img[currMB->pix_y], p_Vid->p_decs->res_img[0], mode == I16MB ? currSlice->mpr_16x16[0][ (short) currMB->i16mode] : currSlice->mb_pred[0], 0, 16);
    for (k = 0; k<p_Inp->NoOfDecoders ;k++)
    {
      decode_one_mb (currMB, p_Vid->enc_picture, k);
      ddistortion += compute_SSE16x16(&p_Vid->pImgOrg[0][currMB->opix_y], &p_Vid->enc_picture->de_mem->p_dec_img[0][k][currMB->pix_y], currMB->pix_x, currMB->pix_x);

      //Use integer calculation
      if (min_rdcost < DISTBLK_MAX)
      {
        {
          temp_dist = min_rdcost * p_Inp->NoOfDecoders;
        }
      }
      else
        temp_dist = DISTBLK_MAX;

      if (ddistortion > temp_dist)
      {
        //distortion = (distblk) ((ddistortion) / (k+1) + 0.5);
        return DISTBLK_MAX;
      }
    }

    distortion = (distblk) (ddistortion / p_Inp->NoOfDecoders);

    if ((p_Vid->yuv_format != YUV400) && (active_sps->chroma_format_idc != YUV444))
    {
      // CHROMA
      distortion += compute_SSE_cr(&p_Vid->pImgOrg[1][currMB->opix_c_y], &p_Vid->enc_picture->imgUV[0][currMB->pix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x);
      distortion += compute_SSE_cr(&p_Vid->pImgOrg[2][currMB->opix_c_y], &p_Vid->enc_picture->imgUV[1][currMB->pix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x);
    }
  }
  else  //to be called by RDCost_for_macroblocks()
  {
    for (k = 0; k<p_Inp->NoOfDecoders ;k++)
    {
      ddistortion += compute_SSE16x16(&p_Vid->pImgOrg[0][currMB->opix_y], &p_Vid->enc_picture->de_mem->p_dec_img[0][k][currMB->pix_y], currMB->pix_x, currMB->pix_x);

      //Use integeter calculation
      if (min_rdcost < DISTBLK_MAX)
      {
        {
          temp_dist = min_rdcost * p_Inp->NoOfDecoders;
        }
      }
      else
        temp_dist = DISTBLK_MAX;

      if (ddistortion > temp_dist)
      {
        //distortion = (distblk) ((ddistortion) / (k+1) + 0.5);
        return DISTBLK_MAX;
      }
    }
    distortion = (distblk) (ddistortion / p_Inp->NoOfDecoders);

    if ((p_Vid->yuv_format != YUV400) && (active_sps->chroma_format_idc != YUV444))
    {
      // CHROMA
      distortion += compute_SSE_cr(&p_Vid->pImgOrg[1][currMB->opix_c_y], &p_Vid->enc_picture->imgUV[0][currMB->pix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x);
      distortion += compute_SSE_cr(&p_Vid->pImgOrg[2][currMB->opix_c_y], &p_Vid->enc_picture->imgUV[1][currMB->pix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x);
    }
  }

  return distortion;
}

/*!
 *************************************************************************************
 * \brief
 *    Performs the simulation of the packet losses, calls the error concealment funcs
 *    and deblocks the error concealed pictures
 *
 *************************************************************************************
 */
void UpdateDecoders(VideoParameters *p_Vid, InputParameters *p_Inp, StorablePicture *enc_pic)
{
  int k;
  for (k = 0; k < p_Inp->NoOfDecoders; k++)
  {
    Build_Status_Map(p_Vid, p_Inp, enc_pic->de_mem->mb_error_map[k]); // simulates the packet losses
    p_Vid->error_conceal_picture(p_Vid, enc_pic, k); 
    DeblockFrame (p_Vid, enc_pic->de_mem->p_dec_img[0][k], NULL);
  }
}



/*!
 *************************************************************************************
 * \brief
 *    Builds a random status map showing whether each MB is received or lost, based
 *    on the packet loss rate and the slice structure.
 *
 * \param s_map
 *    The status map to be filled
 *************************************************************************************
 */
static void Build_Status_Map(VideoParameters *p_Vid, InputParameters *p_Inp, byte **s_map)
{
  int i,j,slice=-1,mb=0,jj,ii;
  byte packet_lost=0;

  jj = p_Vid->height/MB_BLOCK_SIZE;
  ii = p_Vid->width/MB_BLOCK_SIZE;

  for (j = 0; j < jj; j++)
  {
    for (i = 0; i < ii; i++)
    {
      if (!p_Inp->slice_mode || p_Vid->mb_data[mb].slice_nr != slice) /* new slice */
      {
        packet_lost=0;
        if ((double)rand()/(double)RAND_MAX*100 < p_Inp->LossRateC)   packet_lost += 3;
        if ((double)rand()/(double)RAND_MAX*100 < p_Inp->LossRateB)   packet_lost += 2;
        if ((double)rand()/(double)RAND_MAX*100 < p_Inp->LossRateA)   packet_lost  = 1;
        slice++;
      }
      if (!packet_lost)
      {
        s_map[j][i]=0;  //! Packet OK
      }
      else
      {
        s_map[j][i] = packet_lost;
        if(p_Inp->partition_mode == 0)  s_map[j][i]=1;
      }
      mb++;
    }
  }
}


/*!
 *************************************************************************************
 * \brief
 *    Stores the pel values for the current best mode.
 *************************************************************************************
 */
void errdo_store_best_block_multihyp(InputParameters *p_Inp, imgpel*** mbY, imgpel*** dec_img, int block, int img_i, int img_j, int block_size)
{
  int i0 = ((block&0x01)<<3);
  int j0 = ((block>>1)<<3);
  int j, k;
  int j1 = j0 + block_size;
  
  for (k = 0; k < p_Inp->NoOfDecoders; k++)
  {
    for (j = j0; j < j1; j++)
    {
      memcpy(&mbY[k][j][i0], &dec_img[k][img_j+j][img_i+i0], block_size * sizeof(imgpel));
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Restores the pel values from the current best mode.
 *************************************************************************************
 */
void errdo_get_best_block_multihyp(Macroblock *currMB, imgpel*** dec_img, imgpel*** mbY, int block, int block_size)
{
  int i0 = ((block&0x01)<<3);
  int j0 = ((block>>1)<<3);
  int j, k;
  int j1 = j0 + block_size;

  for (k = 0; k < currMB->p_Inp->NoOfDecoders; k++)
  {
    for (j = j0; j < j1; j++)
    {
      memcpy(&dec_img[k][currMB->pix_y + j][currMB->pix_x+i0], &mbY[k][j][i0], block_size * sizeof(imgpel));
    }
  }
}

/*!
**************************************************************************************
* \brief 
*      Decodes one macroblock for error resilient RDO.  
*    Currently does not support:
*    1) B coded pictures
*    2) Chroma components
*    3) Potential error propagation due to intra prediction
*    4) Field coding
**************************************************************************************
*/
void decode_one_mb (Macroblock* currMB, StorablePicture *enc_pic, int decoder)
{
  int i0, j;
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  imgpel** curComp;
  imgpel** oldComp;

  //printf("currMB->mb_type %d\n", currMB->mb_type);
  if (currMB->mb_type > P8x8) //Intra MB
  {
    curComp = &enc_pic->de_mem->p_dec_img[0][decoder][currMB->pix_y];
    oldComp = &enc_pic->p_curr_img[currMB->pix_y];
    i0 = currMB->pix_x;
    copy_image_data_16x16(curComp, oldComp, i0, i0);
  }
  else if ((currMB->mb_type == 0) && (currSlice->slice_type != B_SLICE))
  {
    get_predicted_mb(currMB, enc_pic, decoder);
    curComp = &enc_pic->de_mem->p_dec_img[0][decoder][currMB->pix_y];
    for(j = 0; j < p_Vid->mb_size[0][1]; j++)
    {
      memcpy(&(curComp[j][currMB->pix_x]), &(p_Vid->p_decs->dec_mb_pred[decoder][j][0]), p_Vid->mb_size[0][0] * sizeof(imgpel));
    }
  }
  else 
  {
    get_predicted_mb(currMB, enc_pic, decoder);
    add_residue(currMB, enc_pic, decoder, PLANE_Y, 0, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
  }
}

/*!
**************************************************************************************
* \brief 
*      Finds predicted macroblock values 
*   and copies them to currSlice->mb_pred[0][][]
**************************************************************************************
*/
static void get_predicted_mb(Macroblock *currMB, StorablePicture *enc_pic, int decoder)
{
  Slice *currSlice = currMB->p_Slice;
  int i,j,k;
  int block_size_x, block_size_y;
  int mv_mode, pred_dir;
  static const byte decode_block_scan[16] = {0,1,4,5,2,3,6,7,8,9,12,13,10,11,14,15};
  int block8x8;
  int k_start, k_end, k_inc;

  if (currSlice->slice_type == P_SLICE && !currMB->mb_type)
  {
    block_size_x = MB_BLOCK_SIZE;
    block_size_y = MB_BLOCK_SIZE;
    perform_mc(currMB, decoder, PLANE_Y, enc_pic, LIST_0, 0, 0, enc_pic->mv_info, 0, 0, block_size_x, block_size_y, 0);
  }
  else if (currMB->mb_type == 1)
  {
    block_size_x = MB_BLOCK_SIZE;
    block_size_y = MB_BLOCK_SIZE;
    pred_dir = currMB->b8x8[0].pdir;   
    perform_mc(currMB, decoder, PLANE_Y, enc_pic, pred_dir, 1, 1, enc_pic->mv_info, 0, 0, block_size_x, block_size_y, currMB->b8x8[0].bipred);
  }
  else if (currMB->mb_type == 2)
  {   
    block_size_x = MB_BLOCK_SIZE;
    block_size_y = 8;    

    for (block8x8 = 0; block8x8 < 4; block8x8 += 2)
    {
      pred_dir = currMB->b8x8[block8x8].pdir;
      perform_mc(currMB, decoder, PLANE_Y, enc_pic, pred_dir, 2, 2, enc_pic->mv_info, 0, block8x8, block_size_x, block_size_y, currMB->b8x8[block8x8].bipred);
    }
  }
  else if (currMB->mb_type == 3)
  {   
    block_size_x = 8;
    block_size_y = 16;

    for (block8x8 = 0; block8x8 < 2; block8x8++)
    {
      i = block8x8<<1;
      j = 0;      
      pred_dir = currMB->b8x8[block8x8].pdir;
      perform_mc(currMB, decoder, PLANE_Y, enc_pic, pred_dir, 3, 3, enc_pic->mv_info, i, j, block_size_x, block_size_y, currMB->b8x8[block8x8].bipred);
    }
  }
  else 
  {
    for (block8x8 = 0; block8x8 < 4; block8x8++)
    {
      mv_mode  = currMB->b8x8[block8x8].mode;
      pred_dir = currMB->b8x8[block8x8].pdir;

      k_start = (block8x8 << 2);
      k_inc = (mv_mode == 5) ? 2 : 1;

      if (mv_mode == 0)
      {
        k_end = k_start + 1;
        block_size_x = 8;
        block_size_y = 8;
      }
      else
      {
        k_end = (mv_mode == 4) ? k_start + 1 : ((mv_mode == 7) ? k_start + 4 : k_start + k_inc + 1);
        block_size_x = ( mv_mode == 5 || mv_mode == 4 ) ? 8 : 4;
        block_size_y = ( mv_mode == 6 || mv_mode == 4 ) ? 8 : 4;
      }
      
      for (k = k_start; k < k_end; k += k_inc)
      {
        i =  (decode_block_scan[k] & 3);
        j = ((decode_block_scan[k] >> 2) & 3);
        perform_mc(currMB, decoder, PLANE_Y, enc_pic, pred_dir, mv_mode, mv_mode, enc_pic->mv_info, i, j, block_size_x, block_size_y, currMB->b8x8[block8x8].bipred);
      }
    }
  }
}


/*!
**************************************************************************************
* \brief 
*      Decodes one 8x8 partition for error resilient RDO.  
*    Currently does not support:
*    1) B coded pictures
*    2) Chroma components
*    3) Potential error propagation due to intra prediction
*    4) Field coding
**************************************************************************************
*/
void decode_one_b8block (Macroblock* currMB, StorablePicture *enc_pic, int decoder, int block8x8, short mv_mode, int pred_dir) 
{
  int i,j,k;
  int block_size_x, block_size_y;
  int i0 = (block8x8 & 0x01)<<3;
  int j0 = (block8x8 >> 1)<<3,   j1 = j0+8;
  //char*** ref_idx_buf = enc_pic->motion.ref_idx;
  imgpel **curComp;
  imgpel **oldComp;
  int k_start, k_end, k_inc;
  static const byte decode_block_scan[16] = {0,1,4,5,2,3,6,7,8,9,12,13,10,11,14,15};

  if (mv_mode > 8)  //Intra
  {
    for(j = j0; j < j1; j++)
    {
      curComp = &enc_pic->de_mem->p_dec_img[0][decoder][currMB->pix_y];
      oldComp = &enc_pic->p_curr_img[currMB->pix_y];
      memcpy(&(curComp[j][i0]), &(oldComp[j][i0]), sizeof(imgpel)*8);
    }
  }
  else
  {
    k_start = (block8x8 << 2);

    if (mv_mode == 0) //Direct
    {
      k_inc = 1;
      k_end = k_start+1;
      block_size_x = 8;
      block_size_y = 8;
      //ref_idx_buf = currSlice->direct_ref_idx;
    }
    else //mv_mode from 4 to 7
    {
      k_inc = (mv_mode == 5) ? 2 : 1;
      k_end = (mv_mode == 4) ? k_start + 1 : ((mv_mode == 7) ? k_start + 4 : k_start + k_inc + 1);

      block_size_x = ( mv_mode == 5 || mv_mode == 4 ) ? 8 : 4;
      block_size_y = ( mv_mode == 6 || mv_mode == 4 ) ? 8 : 4;
    }

    for (k = k_start; k < k_end; k += k_inc)
    {
      i =  (decode_block_scan[k] & 3);
      j = ((decode_block_scan[k] >> 2) & 3);
      perform_mc(currMB, decoder, PLANE_Y, enc_pic, pred_dir, mv_mode, mv_mode, enc_pic->mv_info, i, j, block_size_x, block_size_y, currMB->b8x8[block8x8].bipred);
    }        

    add_residue(currMB, enc_pic, decoder, PLANE_Y, block8x8, 8, 8);
  }
}

/*!
**************************************************************************************
* \brief 
*      Add residual to motion predicted block
**************************************************************************************
*/
static void add_residue (Macroblock *currMB, StorablePicture *enc_pic, int decoder, int pl, int block8x8, int x_size, int y_size) 
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int max_pel_value = currMB->p_Vid->max_pel_value_comp[pl];
  int i,j;
  int i0 = (block8x8 & 0x01)<<3, i1 = i0 + x_size;
  int j0 = (block8x8 >> 1)<<3,   j1 = j0 + y_size;

  imgpel **p_dec_img = &enc_pic->de_mem->p_dec_img[pl][decoder][currMB->pix_y];
  int    **res_img   = p_Vid->p_decs->res_img[0];
  imgpel** mpr       = p_Vid->p_decs->dec_mb_pred[decoder];

  for (j = j0; j < j1; j++)
  {
    for (i = i0; i < i1; i++)
    {
      p_dec_img[j][currMB->pix_x + i] = (imgpel) iClip3(0, max_pel_value, (mpr[j][i] + res_img[j][i])); 
    } 
  }
}


/*!
**************************************************************************************
* \brief 
*      Finds predicted macroblock values for error concealment
*   
*   Requires enc_pic->motion.mv and enc_pic->motion.ref_idx to be correct for 
*   current picture.
**************************************************************************************
*/
static void get_predicted_concealment_mb(Macroblock* currMB, StorablePicture* enc_pic, int decoder)
{
  Slice *currSlice = currMB->p_Slice;
  int i,j,k;
  int block_size_x, block_size_y;
  int mv_mode, pred_dir;
  static const byte decode_block_scan[16] = {0,1,4,5,2,3,6,7,8,9,12,13,10,11,14,15};
  int block8x8;
  int k_start, k_end, k_inc;

  if (currSlice->slice_type == P_SLICE && !currMB->mb_type)
  {
    block_size_x = MB_BLOCK_SIZE;
    block_size_y = MB_BLOCK_SIZE;
    perform_mc_concealment(currMB, decoder, PLANE_Y, enc_pic, LIST_0, 0, 0, enc_pic->mv_info, 0, 0, block_size_x, block_size_y);
  }
  else if (currMB->mb_type == 1)
  {
    block_size_x = MB_BLOCK_SIZE;
    block_size_y = MB_BLOCK_SIZE;
    pred_dir = currMB->b8x8[0].pdir;   
    perform_mc_concealment(currMB, decoder, PLANE_Y, enc_pic, pred_dir, 1, 1, enc_pic->mv_info, 0, 0, block_size_x, block_size_y);
  }
  else if (currMB->mb_type == 2)
  {   
    block_size_x = MB_BLOCK_SIZE;
    block_size_y = 8;    

    for (block8x8 = 0; block8x8 < 4; block8x8 += 2)
    {
      pred_dir = currMB->b8x8[block8x8].pdir;
      perform_mc_concealment(currMB, decoder, PLANE_Y, enc_pic, pred_dir, 2, 2, enc_pic->mv_info, 0, block8x8, block_size_x, block_size_y);
    }
  }
  else if (currMB->mb_type == 3)
  {   
    block_size_x = 8;
    block_size_y = 16;

    for (block8x8 = 0; block8x8 < 2; block8x8++)
    {
      i = block8x8<<1;
      j = 0;      
      pred_dir = currMB->b8x8[block8x8].pdir;
      perform_mc_concealment(currMB, decoder, PLANE_Y, enc_pic, pred_dir, 3, 3, enc_pic->mv_info, i, j, block_size_x, block_size_y);
    }
  }
  else 
  {
    for (block8x8 = 0; block8x8 < 4; block8x8++)
    {
      mv_mode  = currMB->b8x8[block8x8].mode;
      pred_dir = currMB->b8x8[block8x8].pdir;

      k_start = (block8x8 << 2);
      k_inc = (mv_mode == 5) ? 2 : 1;

      if (mv_mode == 0)
      {
        k_end = k_start + 1;
        block_size_x = 8;
        block_size_y = 8;
      }
      else
      {
        k_end = (mv_mode == 4) ? k_start + 1 : ((mv_mode == 7) ? k_start + 4 : k_start + k_inc + 1);
        block_size_x = ( mv_mode == 5 || mv_mode == 4 ) ? 8 : 4;
        block_size_y = ( mv_mode == 6 || mv_mode == 4 ) ? 8 : 4;
      }
      
      for (k = k_start; k < k_end; k += k_inc)
      {
        i =  (decode_block_scan[k] & 3);
        j = ((decode_block_scan[k] >> 2) & 3);
        perform_mc_concealment(currMB, decoder, PLANE_Y, enc_pic, pred_dir, mv_mode, mv_mode, enc_pic->mv_info, i, j, block_size_x, block_size_y);
      }
    }
  }
}


/*!
 *************************************************************************************
 * \brief
 *    Performs copy error concealment for macroblocks with errors.
 *  Note: Currently assumes that the reference picture lists remain the same for all 
 *        slices of a picture. 
 *  
 *************************************************************************************
 */
void copy_conceal_picture(VideoParameters *p_Vid, StorablePicture *enc_pic, int decoder)
{
  unsigned int mb;
  Macroblock* currMB;
  int mb_error;
  byte** mb_error_map = enc_pic->de_mem->mb_error_map[decoder];
  StorablePicture* refPic;
  BlockPos *PicPos = p_Vid->PicPos;

  refPic = find_nearest_ref_picture(p_Vid->p_Dpb_layer[0], enc_pic->poc); //Used for concealment if actual reference pic is not known.
  for (mb = 0; mb < p_Vid->PicSizeInMbs; mb++)
  {
    currMB = &p_Vid->mb_data[mb];
    currMB->mb_x = PicPos[mb].x;
    currMB->mb_y = PicPos[mb].y;
    mb_error = mb_error_map[currMB->mb_y][currMB->mb_x];
    if (mb_error)
    {      
      currMB->block_x = currMB ->mb_x << 2;
      currMB->block_y = currMB->mb_y << 2;
      currMB->pix_x   = currMB->block_x << 2;
      currMB->pix_y   = currMB->block_y << 2;
      copy_conceal_mb(currMB, enc_pic, decoder, mb_error, refPic);
    }
  }
}

/******************************************************************************************
*
* Perform copy error concealment for macroblock.
*   
*******************************************************************************************
*/
static void copy_conceal_mb(Macroblock* currMB, StorablePicture *enc_pic, int decoder, int mb_error, StorablePicture* refPic)
{
  Slice *currSlice = currMB->p_Slice;
  int j, i, i0 = currMB->pix_x;
  imgpel** concealed_img = &(enc_pic->de_mem->p_dec_img[0][decoder][currMB->pix_y]);
  imgpel** ref_img;

  if (mb_error == 1 || (mb_error != 3 && currMB->mb_type > P8x8)) //All partitions lost, or intra mb lost
  {
    if (refPic != NULL) //Use nearest reference picture for concealment
    {
      ref_img = &(refPic->de_mem->p_dec_img[0][decoder][currMB->pix_y]);
      for (j = 0; j < MB_BLOCK_SIZE; j++)
      {
        memcpy(&(concealed_img[j][i0]), &(ref_img[j][i0]), sizeof(imgpel)*MB_BLOCK_SIZE);
      }
    }
    else //No ref picture available
    {
      for(j=0; j<MB_BLOCK_SIZE; j++)
      {
        for(i=i0; i<i0+MB_BLOCK_SIZE; i++)
        {
          concealed_img[j][i] = 128;
        }
      }
    }
  }
  else if (mb_error != 2 && currSlice->slice_type != I_SLICE && currMB->mb_type < P8x8) //Only partition 3 lost, and P/B macroblock
  {
    VideoParameters *p_Vid = currMB->p_Vid;
    get_predicted_concealment_mb(currMB, enc_pic, decoder);
    for(j = 0; j < MB_BLOCK_SIZE; j++)
    {  
      memcpy(&(concealed_img[j][i0]), &(p_Vid->p_decs->dec_mb_pred[decoder][j][0]), MB_BLOCK_SIZE * sizeof(imgpel));
    }
  }
}
