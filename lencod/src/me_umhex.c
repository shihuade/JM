
/*!
 ************************************************************************
 *
 * \file me_umhex.c
 *
 * \brief
 *   Fast integer pel motion estimation and fractional pel motion estimation
 *   algorithms are described in this file.
 *   1. UMHEX_get_mem() and UMHEX_free_mem() are functions for allocation and release
 *      of memories about motion estimation
 *   2. UMHEX_BlockMotionSearch() is the function for fast integer pel motion
 *      estimation and fractional pel motion estimation
 *   3. UMHEX_DefineThreshold() defined thresholds for early termination
 * \author
 *    Main contributors: (see contributors.h for copyright, address and affiliation details)
 *    - Zhibo Chen         <chenzhibo@tsinghua.org.cn>
 *    - JianFeng Xu        <fenax@video.mdc.tsinghua.edu.cn>
 *    - Wenfang Fu         <fwf@video.mdc.tsinghua.edu.cn>
 *    - Xiaozhong Xu       <xxz@video.mdc.tsinghua.edu.cn>
 * \date
 *    2006.1
 ************************************************************************
 */

#include <limits.h>

#include "global.h"
#include "memalloc.h"
#include "me_umhex.h"
#include "refbuf.h"
#include "mb_access.h"
#include "image.h"
#include "enc_statistics.h"
#include "macroblock.h"
#include "me_distortion.h"
#include "mv_search.h"
#include "me_fullsearch.h"

#define Q_BITS          15
#define MIN_IMG_WIDTH   176

static const MotionVector Diamond[4] = {{-4, 0}, {4, 0}, {0, -4}, {0, 4}};
static const MotionVector Hexagon[6] = {{-8, 0}, {8, 0},{-4, -8}, {4, 8}, {-4, 8}, {4 , -8}};
static const short Big_Hexagon_X[16] = {0,-8, -16,-16,-16, -16, -16, -8,  0,  8,  16, 16, 16, 16, 16, 8};
static const short Big_Hexagon_Y[16] = {8, 12, 8,  4, 0, -4, -8, -12, -16, -12, -8, -4, 0, 4, 8, 12};

static const int   Multi_Ref_Thd[8]    = {0,  300,  120,  120,  60,  30,   30,  15};
static const int   Big_Hexagon_Thd[8]  = {0, 3000, 1500, 1500, 800, 400,  400, 200};
static const int   Median_Pred_Thd[8]  = {0,  750,  350,  350, 170,  80,   80,  40};
static const int   Threshold_DSR[8]    = {0, 2200, 1000, 1000, 500, 250,  250, 120};


void UMHEX_DefineThreshold(VideoParameters *p_Vid)
{
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;

  p_UMHex->AlphaFourth_1[1] = 0.01f;
  p_UMHex->AlphaFourth_1[2] = 0.01f;
  p_UMHex->AlphaFourth_1[3] = 0.01f;
  p_UMHex->AlphaFourth_1[4] = 0.02f;
  p_UMHex->AlphaFourth_1[5] = 0.03f;
  p_UMHex->AlphaFourth_1[6] = 0.03f;
  p_UMHex->AlphaFourth_1[7] = 0.04f;

  p_UMHex->AlphaFourth_2[1] = 0.06f;
  p_UMHex->AlphaFourth_2[2] = 0.07f;
  p_UMHex->AlphaFourth_2[3] = 0.07f;
  p_UMHex->AlphaFourth_2[4] = 0.08f;
  p_UMHex->AlphaFourth_2[5] = 0.12f;
  p_UMHex->AlphaFourth_2[6] = 0.11f;
  p_UMHex->AlphaFourth_2[7] = 0.15f;

  p_UMHex->BlockType_LUT[0][0] = 7; // 4x4
  p_UMHex->BlockType_LUT[0][1] = 6; // 4x8
  p_UMHex->BlockType_LUT[1][0] = 5; // 8x4
  p_UMHex->BlockType_LUT[1][1] = 4; // 8x8
  p_UMHex->BlockType_LUT[1][3] = 3; // 8x16
  p_UMHex->BlockType_LUT[3][1] = 2; // 16x8
  p_UMHex->BlockType_LUT[3][3] = 1; // 16x16

  return;
}
/*!
************************************************************************
* \brief
*    Set MB thresholds for fast motion estimation
*    Those thresholds may be adjusted to trade off rate-distortion
*    performance and UMHEX speed
************************************************************************
*/

void UMHEX_DefineThresholdMB(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;
  int gb_qp_per    = (p_Inp->qp[P_SLICE])/6;
  int gb_qp_rem    = (p_Inp->qp[P_SLICE])%6;

  int gb_q_bits    = Q_BITS+gb_qp_per;
  int gb_qp_const,Thresh4x4;

  float Quantize_step;
  int i;
  // scale factor: defined for different image sizes
  float scale_factor = (float)((1-p_Inp->UMHexScale*0.1)+p_Inp->UMHexScale*0.1*(p_Vid->width/MIN_IMG_WIDTH));
  // QP factor: defined for different quantization steps
  float QP_factor = (float)((1.0-0.90*(p_Inp->qp[P_SLICE]/51.0f)));
  distblk dbScalar = dist_scale(1);

  gb_qp_const=(1<<gb_q_bits)/6;
  Thresh4x4 =   ((1<<gb_q_bits) - gb_qp_const)/imax(1, p_Vid->p_Quant->q_params_4x4[0][1][gb_qp_rem][0][0].ScaleComp);
  Quantize_step = Thresh4x4/(4*5.61f)*2.0f*scale_factor;
  p_UMHex->Bsize[7]=(16*16)*Quantize_step*dbScalar;

  p_UMHex->Bsize[6] = p_UMHex->Bsize[7]*4*dbScalar;
  p_UMHex->Bsize[5] = p_UMHex->Bsize[7]*4*dbScalar;
  p_UMHex->Bsize[4] = p_UMHex->Bsize[5]*4*dbScalar;
  p_UMHex->Bsize[3] = p_UMHex->Bsize[4]*4*dbScalar;
  p_UMHex->Bsize[2] = p_UMHex->Bsize[4]*4*dbScalar;
  p_UMHex->Bsize[1] = p_UMHex->Bsize[2]*4*dbScalar;

  for(i=1;i<8;i++)
  {
    //ET_Thd1: early termination after median prediction
    p_UMHex->Median_Pred_Thd_MB[i]  = (distblk) (Median_Pred_Thd[i]* scale_factor*QP_factor*dbScalar);
    //ET_thd2: early termination after every circle of 16 points Big-Hex Search
    p_UMHex->Big_Hexagon_Thd_MB[i]  = (distblk) (Big_Hexagon_Thd[i]* scale_factor*QP_factor*dbScalar);
    //threshold for multi ref case
    p_UMHex->Multi_Ref_Thd_MB[i]    = (distblk) (Multi_Ref_Thd[i]  * scale_factor*QP_factor*dbScalar);
    //threshold for usage of DSR technique. DSR ref to JVT-R088
    p_UMHex->Threshold_DSR_MB[i]    = (distblk) (Threshold_DSR[i]  * scale_factor*QP_factor*dbScalar);
  }
}

/*!
************************************************************************
* \brief
*    Allocation of space for fast motion estimation
************************************************************************
*/
int UMHEX_get_mem(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;

  int memory_size = 0;
  int search_range = p_Inp->SearchMode[0] == UM_HEX ? p_Inp->search_range[0] : p_Inp->search_range[1];

  if (NULL==(p_UMHex->flag_intra = calloc ((p_Vid->width>>4)+1,sizeof(byte)))) no_mem_exit("UMHEX_get_mem: p_UMHex->flag_intra"); //fwf 20050330

  memory_size += get_mem2D(&p_UMHex->McostState, 2*search_range+1, 2*search_range+1);

  memory_size += get_mem4Ddistblk(&(p_UMHex->fastme_ref_cost), p_Vid->max_num_references, 9, 4, 4);
  memory_size += get_mem3Ddistblk(&(p_UMHex->fastme_l0_cost), 9, p_Vid->height >> 2, p_Vid->width >> 2);
  memory_size += get_mem3Ddistblk(&(p_UMHex->fastme_l1_cost), 9, p_Vid->height >> 2, p_Vid->width >> 2);
  memory_size += get_mem2Ddistblk(&(p_UMHex->fastme_best_cost), 7, p_Vid->width >> 2);

  memory_size += get_mem2D(&p_UMHex->SearchState, 7, 7);
  if(p_Inp->BiPredMotionEstimation == 1)//memory allocation for bipred mode
  {
    memory_size += get_mem3Ddistblk(&(p_UMHex->fastme_l0_cost_bipred), 9, p_Vid->height >> 2, p_Vid->width >> 2);//for bipred
    memory_size += get_mem3Ddistblk(&(p_UMHex->fastme_l1_cost_bipred), 9, p_Vid->height >> 2, p_Vid->width >> 2);//for bipred
  }

  return memory_size;
}

/*!
************************************************************************
* \brief
*    Free space for fast motion estimation
************************************************************************
*/
void UMHEX_free_mem(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;
  free_mem2D(p_UMHex->McostState);

  free_mem4Ddistblk(p_UMHex->fastme_ref_cost);
  free_mem3Ddistblk(p_UMHex->fastme_l0_cost );
  free_mem3Ddistblk(p_UMHex->fastme_l1_cost);
  free_mem2Ddistblk(p_UMHex->fastme_best_cost);

  free_mem2D(p_UMHex->SearchState);
  free (p_UMHex->flag_intra);
  if(p_Inp->BiPredMotionEstimation == 1)
  {
    free_mem3Ddistblk(p_UMHex->fastme_l0_cost_bipred);//for bipred
    free_mem3Ddistblk(p_UMHex->fastme_l1_cost_bipred);//for bipred
  }
  free(p_UMHex);
}

/*!
************************************************************************
* \brief
*    UMHEXIntegerPelBlockMotionSearch: fast pixel block motion search
*    this algorithm is called UMHexagonS(see JVT-D016),which includes
*    four steps with different kinds of search patterns
* \par Input:
* imgpel*   orig_pic,     // <--  original picture
* int       ref,          // <--  reference frame (0... or -1 (backward))
* int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
* int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
* int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
* int       pred_mv[2],   // <--  motion vector predictor (x|y) in sub-pel units
* MotionVector   *mv,        //  --> motion vector (x|y) - in sub-pel units
* int       search_range, // <--  1-d search range in sub-pel units
* int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
* int       lambda_factor // <--  lagrangian parameter for determining motion cost
* \par
* Two macro definitions defined in this program:
* 1. EARLY_TERMINATION: early termination algrithm, refer to JVT-D016.doc
* 2. SEARCH_ONE_PIXEL: search one pixel in search range
* \author
*   Main contributors: (see contributors.h for copyright, address and affiliation details)
*   - Zhibo Chen         <chenzhibo@tsinghua.org.cn>
*   - JianFeng Xu        <fenax@video.mdc.tsinghua.edu.cn>
*   - Xiaozhong Xu       <xxz@video.mdc.tsinghua.edu.cn>
* \date   :
*   2006.1
************************************************************************
*/
distblk                                     //  ==> minimum motion cost after search
UMHEXIntegerPelBlockMotionSearch  (Macroblock *currMB,     // <--  current Macroblock
                                   MotionVector *pred_mv,    // < <--  motion vector predictor (x|y) in sub-pel units
                                   MEBlock *mv_block,
                                   distblk     min_mcost,     // < <--  minimum motion cost (cost for center or huge value)
                                   int       lambda_factor  // < <--  lagrangian parameter for determining motion cost
                                   )
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;

  int   blocktype     = mv_block->blocktype;
  short blocksize_x   = mv_block->blocksize_x;  // horizontal block size
  short blocksize_y   = mv_block->blocksize_y;  // vertical block size
  short pic_pix_x2    = mv_block->pos_x2;
  short block_x       = mv_block->block_x;
  short block_y       = mv_block->block_y;

  int   list = mv_block->list;
  int   cur_list = list + currMB->list_offset;
  short ref = mv_block->ref_idx;
  StorablePicture *ref_picture = currSlice->listX[cur_list][ref];

  MotionVector *mv = &mv_block->mv[list];
  MotionVector iMinNow, cand, center, pred, best = {0, 0};

  int   search_step;
  int   pos;
  distblk mcost;
  distblk   *SAD_prediction = p_UMHex->fastme_best_cost[blocktype-1];//multi ref SAD prediction
  int   i, j, m;
  float betaFourth_1,betaFourth_2;
  int  temp_Big_Hexagon_X[16];//  temp for Big_Hexagon_X;
  int  temp_Big_Hexagon_Y[16];//  temp for Big_Hexagon_Y;
  distblk ET_Thred = p_UMHex->Median_Pred_Thd_MB[blocktype];//ET threshold in use
  int  search_range = mv_block->searchRange.max_x >> 2;

  short pic_pix_x = mv_block->pos_x_padded;
  short pic_pix_y = mv_block->pos_y_padded;
  pred.mv_x   = pic_pix_x + pred_mv->mv_x;       // predicted position x (in sub-pel units)
  pred.mv_y   = pic_pix_y + pred_mv->mv_y;       // predicted position y (in sub-pel units)
  center.mv_x = pic_pix_x + mv->mv_x;            // center position x (in sub-pel units)
  center.mv_y = pic_pix_y + mv->mv_y;            // center position y (in sub-pel units)



  //////allocate memory for search state//////////////////////////
  memset(p_UMHex->McostState[0],0,(2*p_Inp->search_range[p_Vid->view_id]+1)*(2*p_Inp->search_range[p_Vid->view_id]+1));


  //check the center median predictor

  cand = center;
  mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred);

  mcost += mv_block->computePredFPel(ref_picture, mv_block, min_mcost - mcost, &cand);

  p_UMHex->McostState[search_range][search_range] = 1;
  if (mcost < min_mcost)
  {
    min_mcost = mcost;
    best = cand;
  }

  iMinNow = best;

  for (m = 0; m < 4; m++)
  {
    cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
    cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
    SEARCH_ONE_PIXEL
  }

  if(center.mv_x != pic_pix_x || center.mv_y != pic_pix_y)
  {
    cand.mv_x = pic_pix_x ;
    cand.mv_y = pic_pix_y ;
    SEARCH_ONE_PIXEL
      iMinNow = best;  
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL
    }
  }
  /***********************************init process*************************/
  //for multi ref
  if(ref>0 && currSlice->structure == FRAME  && min_mcost > ET_Thred && SAD_prediction[pic_pix_x2] < p_UMHex->Multi_Ref_Thd_MB[blocktype])
    goto terminate_step;

  //ET_Thd1: early termination for low motion case
  if( min_mcost < ET_Thred)
  {
    goto terminate_step;
  }
  else // hybrid search for main search loop
  {
    /****************************(MV and SAD prediction)********************************/
    UMHEX_setup(currMB, ref, mv_block->list, block_y, block_x, blocktype, currSlice->all_mv );
    ET_Thred = p_UMHex->Big_Hexagon_Thd_MB[blocktype];  // ET_Thd2: early termination Threshold for strong motion

    // Threshold defined for EARLY_TERMINATION
    if (p_UMHex->pred_SAD == 0)
    {
      betaFourth_1=0;
      betaFourth_2=0;
    }
    else
    {
      betaFourth_1 = p_UMHex->Bsize[blocktype]/((float)p_UMHex->pred_SAD * p_UMHex->pred_SAD)-p_UMHex->AlphaFourth_1[blocktype];
      betaFourth_2 = p_UMHex->Bsize[blocktype]/((float)p_UMHex->pred_SAD * p_UMHex->pred_SAD)-p_UMHex->AlphaFourth_2[blocktype];

    }
    /*********************************************end of init ***********************************************/
  }
  // first_step: initial start point prediction

  if(blocktype>1)
  {
    cand.mv_x = (short) (pic_pix_x + (p_UMHex->pred_MV_uplayer[0] / 4) * 4);
    cand.mv_y = (short) (pic_pix_y + (p_UMHex->pred_MV_uplayer[1] / 4) * 4);
    SEARCH_ONE_PIXEL
  }


  //prediction using mV of last ref moiton vector
  if(p_UMHex->pred_MV_ref_flag == 1)      //Notes: for interlace case, ref==1 should be added
  {
    cand.mv_x = (short) (pic_pix_x + (p_UMHex->pred_MV_ref[0] / 4) * 4);
    cand.mv_y = (short) (pic_pix_y + (p_UMHex->pred_MV_ref[1] / 4) * 4);
    SEARCH_ONE_PIXEL
  }
  // Small local search
  iMinNow = best;
  for (m = 0; m < 4; m++)
  {
    cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
    cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
    SEARCH_ONE_PIXEL
  }

  //early termination algorithm, refer to JVT-G016
  EARLY_TERMINATION

    if(blocktype>6)
      goto fourth_1_step;
    else
      goto sec_step;

sec_step: //Unsymmetrical-cross search
  iMinNow = best;

  for(i = 4; i < search_range << 2; i+=8)
  {
    search_step = i;
    cand.mv_x = (short) (iMinNow.mv_x + search_step);
    cand.mv_y = iMinNow.mv_y ;
    SEARCH_ONE_PIXEL
      cand.mv_x = (short) (iMinNow.mv_x - search_step);
    cand.mv_y = iMinNow.mv_y ;
    SEARCH_ONE_PIXEL
  }
  for(i = 4; i < (search_range << 1);i+=8)
  {
    search_step = i;
    cand.mv_x = iMinNow.mv_x ;
    cand.mv_y = (short) (iMinNow.mv_y + search_step);
    SEARCH_ONE_PIXEL
      cand.mv_x = iMinNow.mv_x ;
    cand.mv_y = (short) (iMinNow.mv_y - search_step);
    SEARCH_ONE_PIXEL
  }


  //early termination alogrithm, refer to JVT-G016
  EARLY_TERMINATION

    iMinNow = best;

  //third_step:    // Uneven Multi-Hexagon-grid Search
  //sub step 1: 5x5 squre search
  for(pos=1;pos<25;pos++)
  {
    cand.mv_x = iMinNow.mv_x + p_Vid->spiral_qpel_search[pos].mv_x;
    cand.mv_y = iMinNow.mv_y + p_Vid->spiral_qpel_search[pos].mv_y;
    SEARCH_ONE_PIXEL
  }

  //early termination alogrithm, refer to JVT-G016
  EARLY_TERMINATION

    //sub step 2:  Multi-Hexagon-grid search
    memcpy(temp_Big_Hexagon_X,Big_Hexagon_X,64);
  memcpy(temp_Big_Hexagon_Y,Big_Hexagon_Y,64);
  for(i=1;i<=(search_range >> 2); i++)
  {

    for (m = 0; m < 16; m++)
    {
      cand.mv_x = (short) (iMinNow.mv_x + temp_Big_Hexagon_X[m]);
      cand.mv_y = (short) (iMinNow.mv_y + temp_Big_Hexagon_Y[m]);
      temp_Big_Hexagon_X[m] += Big_Hexagon_X[m];
      temp_Big_Hexagon_Y[m] += Big_Hexagon_Y[m];

      SEARCH_ONE_PIXEL
    }
    // ET_Thd2: early termination Threshold for strong motion
    if(min_mcost < ET_Thred)
    {
      goto terminate_step;
    }
  }


  //fourth_step:  //Extended Hexagon-based Search
  // the fourth step with a small search pattern
fourth_1_step:  //sub step 1: small Hexagon search
  for(i = 0; i < search_range; i++)
  {
    iMinNow = best;
    for (m = 0; m < 6; m++)
    {
      cand.mv_x = iMinNow.mv_x + Hexagon[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Hexagon[m].mv_y;
      SEARCH_ONE_PIXEL
    }

    if (best.mv_x == iMinNow.mv_x && best.mv_y == iMinNow.mv_y)
    {
      break;
    }
  }
fourth_2_step: //sub step 2: small Diamond search

  for(i = 0; i < search_range; i++)
  {
    iMinNow = best;
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL
    }
    if(best.mv_x == iMinNow.mv_x && best.mv_y == iMinNow.mv_y)
      break;
  }

terminate_step:

  // store SAD infomation for prediction
  //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
  for (i=0; i < (blocksize_x>>2); i++)
  {
    for (j=0; j < (blocksize_y>>2); j++)
    {
      if(mv_block->list == 0)
      {
        p_UMHex->fastme_ref_cost[ref][blocktype][block_y+j][block_x+i] = min_mcost;
        if (ref==0)
          p_UMHex->fastme_l0_cost[blocktype][(currMB->block_y)+block_y+j][(currMB->block_x)+block_x+i] = min_mcost;
      }
      else
      {
        p_UMHex->fastme_l1_cost[blocktype][(currMB->block_y)+block_y+j][(currMB->block_x)+block_x+i] = min_mcost;
      }
    }
  }
  //for multi ref SAD prediction
  if ((ref==0) || (SAD_prediction[pic_pix_x2] > min_mcost))
    SAD_prediction[pic_pix_x2] = min_mcost;

  mv->mv_x = (short) (best.mv_x - pic_pix_x);
  mv->mv_y = (short) (best.mv_y - pic_pix_y);
  return min_mcost;
}

distblk                                                   //  ==> minimum motion cost after search
UMHEXSubPelBlockMotionSearch (Macroblock *currMB,     // <--  current Macroblock
                              MotionVector *pred_mv,    // < <--  motion vector predictor (x|y) in sub-pel units
                              MEBlock *mv_block,
                              distblk     min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                              int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                              )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;
  static const MotionVector DiamondQ[4] = {{-1, 0}, { 0, 1}, { 1, 0}, { 0, -1}};
  distblk mcost;
  MotionVector cand, iMinNow, currmv = {0, 0}, cand_pad;

  int   list          = mv_block->list;        
  int   list_offset   = currMB->list_offset;
  short ref = mv_block->ref_idx;
  MotionVector *mv    = &mv_block->mv[list];

  StorablePicture *ref_picture = currSlice->listX[list+list_offset][ref];

  int   dynamic_search_range = 3, i;
  int   m;  
  int   pred_frac_mv_x,pred_frac_mv_y,abort_search;

  //int   pred_frac_up_mv_x, pred_frac_up_mv_y;

  pred_frac_mv_x = (pred_mv->mv_x - mv->mv_x) & 0x03;
  pred_frac_mv_y = (pred_mv->mv_y - mv->mv_y) & 0x03;

  //pred_frac_up_mv_x = (p_UMHex->pred_MV_uplayer[0] - mv->mv_x) & 0x03;
  //pred_frac_up_mv_y = (p_UMHex->pred_MV_uplayer[1] - mv->mv_y) & 0x03;


  memset(p_UMHex->SearchState[0], 0,(2 * dynamic_search_range + 1)*(2 * dynamic_search_range + 1));

  if( !p_Vid->start_me_refinement_hp )
  {
    p_UMHex->SearchState[dynamic_search_range][dynamic_search_range] = 1;
    cand = *mv;
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred_mv);
    cand_pad = pad_MVs (cand, mv_block); //cand = pad_MVs (cand, mv_block);
    mcost += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cand_pad); //&cand);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      currmv = cand;
    }
  }
  else
  {
    p_UMHex->SearchState[dynamic_search_range][dynamic_search_range] = 1;
    currmv = *mv;
  }

  if(pred_frac_mv_x!=0 || pred_frac_mv_y!=0)
  {
    cand.mv_x = (short) (mv->mv_x + pred_frac_mv_x);
    cand.mv_y = (short) (mv->mv_y + pred_frac_mv_y);
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred_mv);
    p_UMHex->SearchState[cand.mv_y -mv->mv_y + dynamic_search_range][cand.mv_x - mv->mv_x + dynamic_search_range] = 1;
    cand_pad = pad_MVs (cand, mv_block); //cand = pad_MVs (cand, mv_block);

    mcost += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cand_pad); //&cand);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      currmv = cand;
    }
  }

  iMinNow = currmv;

  for(i = 0; i < dynamic_search_range; i++)
  {
    abort_search=1;
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + DiamondQ[m].mv_x;
      cand.mv_y = iMinNow.mv_y + DiamondQ[m].mv_y;

      if(iabs(cand.mv_x - mv->mv_x) <= dynamic_search_range && iabs(cand.mv_y - mv->mv_y) <= dynamic_search_range)
      {
        if(!p_UMHex->SearchState[cand.mv_y -mv->mv_y + dynamic_search_range][cand.mv_x -mv->mv_x + dynamic_search_range])
        {
          p_UMHex->SearchState[cand.mv_y -mv->mv_y + dynamic_search_range][cand.mv_x -mv->mv_x + dynamic_search_range] = 1;
          mcost = mv_cost (p_Vid, lambda_factor, &cand, pred_mv);
          cand_pad = pad_MVs (cand, mv_block); //cand = pad_MVs (cand, mv_block);

          mcost += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cand_pad); // &cand);                    
          if (mcost < min_mcost)
          {
            min_mcost = mcost;
            currmv = cand;
            abort_search = 0;
          }
        }
      }
    }
    iMinNow = currmv;
    if(abort_search)
    {
      break;
    }
  }

  *mv = currmv;

  //===== return minimum motion cost =====
  return min_mcost;
}

distblk                                                   //  ==> minimum motion cost after search
UMHEXSubPelBlockME (Macroblock *currMB,        // <-- Current Macroblock
                    MotionVector  *pred_mv,    // <--  motion vector predictor (x|y) in sub-pel units
                    MEBlock *mv_block, 
                    distblk     min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                    int*      lambda
                    )
{  
  if(mv_block->blocktype >3)
  {
    min_mcost =  UMHEXSubPelBlockMotionSearch (currMB, pred_mv, mv_block, min_mcost, lambda[Q_PEL]);
  }
  else
  {
    min_mcost =  sub_pel_motion_estimation (currMB, pred_mv, mv_block, min_mcost, lambda);
  }

  return min_mcost;
}

/*!
************************************************************************
* \brief
* Functions for SAD prediction of intra block cases.
* 1. void UMHEX_decide_intrabk_SAD() judges the block coding type(intra/inter)
*    of neibouring blocks
* 2. void UMHEX_skip_intrabk_SAD() set the SAD to zero if neigouring block coding
*    type is intra
* \date
*    2003.4
************************************************************************
*/
void UMHEX_decide_intrabk_SAD(Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;
  if (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE)
  {
    if (currMB->pix_x == 0 && currMB->pix_y == 0)
    {
      p_UMHex->flag_intra_SAD = 0;
    }
    else if (currMB->pix_x == 0)
    {
      p_UMHex->flag_intra_SAD = p_UMHex->flag_intra[(currMB->pix_x)>>4];
    }
    else if (currMB->pix_y == 0)
    {
      p_UMHex->flag_intra_SAD = p_UMHex->flag_intra[((currMB->pix_x)>>4)-1];
    }
    else
    {
      p_UMHex->flag_intra_SAD = ((p_UMHex->flag_intra[(currMB->pix_x)>>4])||(p_UMHex->flag_intra[((currMB->pix_x)>>4)-1])||(p_UMHex->flag_intra[((currMB->pix_x)>>4)+1])) ;
    }
  }
  return;
}

void UMHEX_skip_intrabk_SAD(Macroblock *currMB, int ref_max)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;
  int i,j,k, ref;
  if (p_Vid->number > 0)
    p_UMHex->flag_intra[(currMB->pix_x)>>4] = (currMB->best_mode == 9 || currMB->best_mode == 10) ? 1:0;

  if (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE && (currMB->best_mode == 9 || currMB->best_mode == 10))
  {
    for (k=0; k < 9;k++)
    {
      for (j=0; j < 4; j++)
      {
        for (i=0; i < 4; i++)
        {
          p_UMHex->fastme_l0_cost[k][j][i] = 0;
          p_UMHex->fastme_l1_cost[k][j][i] = 0;

          for (ref=0; ref < ref_max;ref++)
          {
            p_UMHex->fastme_ref_cost[ref][k][j][i] = 0;
          }
        }
      }
    }

  }
  return;
}


void UMHEX_setup(Macroblock *currMB, short ref, int list, int block_y, int block_x, int blocktype, MotionVector  *****all_mv)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;

  int  N_Bframe=0;
  int n_Bframe=0;
  int temp_blocktype = 0;
  int indication_blocktype[8]={0,0,1,1,2,4,4,5};
  InputParameters *p_Inp = currMB->p_Inp;
  N_Bframe = p_Inp->NumberBFrames;
  n_Bframe =(N_Bframe) ? (p_Vid->p_Stats->frame_ctr[B_SLICE]%(N_Bframe+1)): 0;


  /**************************** MV prediction **********************/
  //MV uplayer prediction
  if (blocktype>1)
  {
    temp_blocktype = indication_blocktype[blocktype];
    p_UMHex->pred_MV_uplayer[0] = all_mv[list][ref][temp_blocktype][block_y][block_x].mv_x;
    p_UMHex->pred_MV_uplayer[1] = all_mv[list][ref][temp_blocktype][block_y][block_x].mv_y;
  }


  //MV ref-frame prediction
  p_UMHex->pred_MV_ref_flag = 0;
  if(list==0)
  {
    if (p_Vid->field_picture)
    {
      if ( ref > 1)
      {
        p_UMHex->pred_MV_ref[0] = all_mv[0][ref-2][blocktype][block_y][block_x].mv_x;
        p_UMHex->pred_MV_ref[0] = (int)(p_UMHex->pred_MV_ref[0]*((ref>>1)+1)/(float)((ref>>1)));
        p_UMHex->pred_MV_ref[1] = all_mv[0][ref-2][blocktype][block_y][block_x].mv_y;
        p_UMHex->pred_MV_ref[1] = (int)(p_UMHex->pred_MV_ref[1]*((ref>>1)+1)/(float)((ref>>1)));
        p_UMHex->pred_MV_ref_flag = 1;
      }
      if (currSlice->slice_type == B_SLICE &&  (ref==0 || ref==1) )
      {
        p_UMHex->pred_MV_ref[0] =(int) (all_mv[1][0][blocktype][block_y][block_x].mv_x * (-n_Bframe)/(N_Bframe-n_Bframe+1.0f));
        p_UMHex->pred_MV_ref[1] =(int) (all_mv[1][0][blocktype][block_y][block_x].mv_y * (-n_Bframe)/(N_Bframe-n_Bframe+1.0f));
        p_UMHex->pred_MV_ref_flag = 1;
      }
    }
    else //frame case
    {
      if ( ref > 0)
      {
        p_UMHex->pred_MV_ref[0] = all_mv[0][ref-1][blocktype][block_y][block_x].mv_x;
        p_UMHex->pred_MV_ref[0] = (int)(p_UMHex->pred_MV_ref[0]*(ref+1)/(float)(ref));
        p_UMHex->pred_MV_ref[1] = all_mv[0][ref-1][blocktype][block_y][block_x].mv_y;
        p_UMHex->pred_MV_ref[1] = (int)(p_UMHex->pred_MV_ref[1]*(ref+1)/(float)(ref));
        p_UMHex->pred_MV_ref_flag = 1;
      }
      if (currSlice->slice_type == B_SLICE && (ref==0)) //B frame forward prediction, first ref
      {
        p_UMHex->pred_MV_ref[0] =(int) (all_mv[1][0][blocktype][block_y][block_x].mv_x * (-n_Bframe)/(N_Bframe-n_Bframe+1.0f));
        p_UMHex->pred_MV_ref[1] =(int) (all_mv[1][0][blocktype][block_y][block_x].mv_y * (-n_Bframe)/(N_Bframe-n_Bframe+1.0f));
        p_UMHex->pred_MV_ref_flag = 1;
      }
    }
  }
  /******************************SAD prediction**********************************/
  if (list==0 && ref>0)  //pred_SAD_ref
  {

    if (p_UMHex->flag_intra_SAD) //add this for irregular motion
    {
      p_UMHex->pred_SAD = 0;
    }
    else
    {
      if (p_Vid->field_picture)
      {
        if (ref > 1)
        {
          p_UMHex->pred_SAD = p_UMHex->fastme_ref_cost[ref-2][blocktype][block_y][block_x];
        }
        else
        {
          p_UMHex->pred_SAD = p_UMHex->fastme_ref_cost[0][blocktype][block_y][block_x];
        }
      }
      else
      {
        p_UMHex->pred_SAD = p_UMHex->fastme_ref_cost[ref-1][blocktype][block_y][block_x];
      }

    }
  }
  else if (blocktype>1)  // pred_SAD_uplayer
  {
    if (p_UMHex->flag_intra_SAD)
    {
      p_UMHex->pred_SAD = 0;
    }
    else
    {
      p_UMHex->pred_SAD = (list==1) ? (p_UMHex->fastme_l1_cost[temp_blocktype][(currMB->block_y)+block_y][(currMB->block_x)+block_x]) : (p_UMHex->fastme_l0_cost[temp_blocktype][(currMB->block_y)+block_y][(currMB->block_x)+block_x]);
      p_UMHex->pred_SAD /= 2;
    }
  }
  else p_UMHex->pred_SAD = 0 ;  // pred_SAD_space

}

/*!
************************************************************************
* \brief
*    UMHEXBipredIntegerPelBlockMotionSearch: fast pixel block motion search for bipred mode
*    this algrithm is called UMHexagonS(see JVT-D016),which includes
*    four steps with different kinds of search patterns
* \author
*   Main contributors: (see contributors.h for copyright, address and affiliation details)
*   - Zhibo Chen         <chenzhibo@tsinghua.org.cn>
*   - JianFeng Xu        <fenax@video.mdc.tsinghua.edu.cn>
*   - Xiaozhong Xu       <xxz@video.mdc.tsinghua.edu.cn>
* \date   :
*   2006.1
************************************************************************
*/
distblk                                                //  ==> minimum motion cost after search
UMHEXBipredIntegerPelBlockMotionSearch (Macroblock *currMB,      // <--  current Macroblock
                                        int       list,          // <--  current reference list
                                        MotionVector *pred_mv1,  // <--  motion vector predictor (x|y) in sub-pel units
                                        MotionVector *pred_mv2,  // <--  motion vector predictor (x|y) in sub-pel units
                                        MotionVector  *mv1,      // <--> in: search center (x|y) / out: motion vector (x|y) - in sub-pel units
                                        MotionVector *mv2,       // <--> in: search center (x|y) 
                                        MEBlock *mv_block,       // <--  motion vector information
                                        int       search_range,  // <--  1-d search range in sub-pel units
                                        distblk       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                        int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                                        )
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;
  int   temp_Big_Hexagon_X[16];// = Big_Hexagon_X;
  int   temp_Big_Hexagon_Y[16];// = Big_Hexagon_Y;

  int   search_step;
  int   i,m,j;
  float betaFourth_1,betaFourth_2;
  int   pos;
  distblk mcost;
  short blocktype   = mv_block->blocktype;
  short blocksize_x = mv_block->blocksize_x;        // horizontal block size
  short blocksize_y = mv_block->blocksize_y;        // vertical block size

  short pic_pix_x     = mv_block->pos_x_padded;
  short pic_pix_y     = mv_block->pos_y_padded;

  short block_x       = mv_block->block_x;
  short block_y       = mv_block->block_y;
  distblk   ET_Thred      = p_UMHex->Median_Pred_Thd_MB[blocktype];
  short ref           = mv_block->ref_idx;

  StorablePicture *ref_picture1 = currSlice->listX[list + currMB->list_offset][ref];
  StorablePicture *ref_picture2 = currSlice->listX[list == 0 ? 1 + currMB->list_offset: currMB->list_offset][ 0 ];

  MotionVector iMinNow, best, cand;

  MotionVector pred1 = pad_MVs(*pred_mv1, mv_block);       // predicted position x (in sub-pel units)
  MotionVector pred2 = pad_MVs(*pred_mv2, mv_block);       // predicted position x (in sub-pel units)
  MotionVector center1 = pad_MVs(*mv1, mv_block);       // predicted position x (in sub-pel units)
  MotionVector center2 = pad_MVs(*mv2, mv_block);       // predicted position x (in sub-pel units)


  search_range >>= 2;
  //////////////////////////////////////////////////////////////////////////

  //////allocate memory for search state//////////////////////////
  memset(p_UMHex->McostState[0],0,(2*search_range+1)*(2*search_range+1));

  //check the center median predictor
  best = cand = center2;
  mcost  = mv_cost (p_Vid, lambda_factor, &center1, &pred1);
  mcost += mv_cost (p_Vid, lambda_factor, &cand, &pred2); 
  mcost += mv_block->computeBiPredFPel(ref_picture1, ref_picture2, mv_block, DISTBLK_MAX-mcost, &center1, &cand);

  p_UMHex->McostState[search_range][search_range] = 1;

  if (mcost < min_mcost)
  {
    min_mcost = mcost;
    best = cand;
  }

  iMinNow = best;
  for (m = 0; m < 4; m++)
  {
    cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
    cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
    SEARCH_ONE_PIXEL_BIPRED;
  }

  if(center2.mv_x != pic_pix_x || center2.mv_y != pic_pix_y)
  {
    cand.mv_x = pic_pix_x ;
    cand.mv_y = pic_pix_y ;
    SEARCH_ONE_PIXEL_BIPRED;

    iMinNow = best;

    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }
  }
  /***********************************init process*************************/

  if( min_mcost < ET_Thred)
  {
    goto terminate_step;
  }
  else
  {
    int  N_Bframe=0;
    int  n_Bframe=0;
    MotionVector *****bipred_mv = currSlice->bipred_mv[list];
    N_Bframe = p_Inp->NumberBFrames;
    n_Bframe = p_Vid->p_Stats->frame_ctr[B_SLICE]%(N_Bframe+1);


    /**************************** MV prediction **********************/
    //MV uplayer prediction
    // non for bipred mode

    //MV ref-frame prediction

    if(list==0)
    {
      if (p_Vid->field_picture)
      {
        p_UMHex->pred_MV_ref[0] =(int) (bipred_mv[1][0][blocktype][block_y][block_x].mv_x * (-n_Bframe)/(N_Bframe-n_Bframe+1.0f));
        p_UMHex->pred_MV_ref[1] =(int) (bipred_mv[1][0][blocktype][block_y][block_x].mv_y * (-n_Bframe)/(N_Bframe-n_Bframe+1.0f));
      }
      else //frame case
      {
        p_UMHex->pred_MV_ref[0] =(int) (bipred_mv[1][0][blocktype][block_y][block_x].mv_x * (-n_Bframe)/(N_Bframe-n_Bframe+1.0f));
        p_UMHex->pred_MV_ref[1] =(int) (bipred_mv[1][0][blocktype][block_y][block_x].mv_y * (-n_Bframe)/(N_Bframe-n_Bframe+1.0f));
      }
    }
    /******************************SAD prediction**********************************/
    p_UMHex->pred_SAD =distblkmin(distblkmin(p_UMHex->SAD_a,p_UMHex->SAD_b),p_UMHex->SAD_c);  // pred_SAD_space
    ET_Thred = p_UMHex->Big_Hexagon_Thd_MB[blocktype];

    ///////Threshold defined for early termination///////////////////
    if (p_UMHex->pred_SAD == 0)
    {
      betaFourth_1=0;
      betaFourth_2=0;
    }
    else
    {
      betaFourth_1 = p_UMHex->Bsize[blocktype]/(p_UMHex->pred_SAD * p_UMHex->pred_SAD)-p_UMHex->AlphaFourth_1[blocktype];
      betaFourth_2 = p_UMHex->Bsize[blocktype]/(p_UMHex->pred_SAD * p_UMHex->pred_SAD)-p_UMHex->AlphaFourth_2[blocktype];
    }
  }

  /***********************************end of init *************************/



  // first_step: initial start point prediction
  //prediction using mV of last ref moiton vector
  if(list == 0)
  {
    cand.mv_x = (short) (pic_pix_x + (p_UMHex->pred_MV_ref[0] / 4) * 4);
    cand.mv_y = (short) (pic_pix_y + (p_UMHex->pred_MV_ref[1] / 4) * 4);
    SEARCH_ONE_PIXEL_BIPRED;
  }


  //small local search
  iMinNow = best;
  for (m = 0; m < 4; m++)
  {
    cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
    cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
    SEARCH_ONE_PIXEL_BIPRED;
  }

  //early termination alogrithm, refer to JVT-G016
  EARLY_TERMINATION;


  //sec_step: //Unsymmetrical-cross search
  iMinNow = best;

  for(i = 1; i < search_range; i+=2)
  {
    search_step = i;
    cand.mv_x = (short) (iMinNow.mv_x + search_step);
    cand.mv_y = iMinNow.mv_y ;
    SEARCH_ONE_PIXEL_BIPRED;
    cand.mv_x = (short) (iMinNow.mv_x - search_step);
    SEARCH_ONE_PIXEL_BIPRED;
  }

  for(i = 1; i < (search_range >> 1);i+=2)
  {
    search_step = i;
    cand.mv_x = iMinNow.mv_x ;
    cand.mv_y = (short) (iMinNow.mv_y + search_step);
    SEARCH_ONE_PIXEL_BIPRED;
    cand.mv_y = (short) (iMinNow.mv_y - search_step);
    SEARCH_ONE_PIXEL_BIPRED;
  }
  //early termination alogrithm, refer to JVT-G016
  EARLY_TERMINATION;

  //third_step:     // Uneven Multi-Hexagon-grid Search
  iMinNow = best;
  //sub step1: 5x5 square search
  for(pos=1;pos<25;pos++)
  {
    cand.mv_x = iMinNow.mv_x + p_Vid->spiral_qpel_search[pos].mv_x;
    cand.mv_y = iMinNow.mv_y + p_Vid->spiral_qpel_search[pos].mv_y;
    SEARCH_ONE_PIXEL_BIPRED;
  }

  //early termination alogrithm, refer to JVT-G016
  EARLY_TERMINATION;      //added back by xxz

  //sub step2: multi-grid-hexagon-search
  memcpy(temp_Big_Hexagon_X,Big_Hexagon_X,64);
  memcpy(temp_Big_Hexagon_Y,Big_Hexagon_Y,64);
  for(i=1;i<=(p_Inp->search_range[p_Vid->view_id] >> 2); i++)
  {

    for (m = 0; m < 16; m++)
    {
      cand.mv_x = (short) (iMinNow.mv_x + temp_Big_Hexagon_X[m]);
      cand.mv_y = (short) (iMinNow.mv_y + temp_Big_Hexagon_Y[m]);
      temp_Big_Hexagon_X[m] += Big_Hexagon_X[m];
      temp_Big_Hexagon_Y[m] += Big_Hexagon_Y[m];

      SEARCH_ONE_PIXEL_BIPRED;
    }
    if(min_mcost < ET_Thred)
    {
      goto terminate_step;

    }
  }
  //fourth step: Local Refinement: Extended Hexagon-based Search
fourth_1_step:

  for(i=0; i < search_range; i++)
  {
    iMinNow = best;
    for (m = 0; m < 6; m++)
    {
      cand.mv_x = iMinNow.mv_x + Hexagon[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Hexagon[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }
    if(best.mv_x == iMinNow.mv_x && best.mv_y == iMinNow.mv_y)
      break;
  }
fourth_2_step:

  for(i = 0; i < search_range; i++)
  {
    iMinNow = best;
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }
    if(best.mv_x == iMinNow.mv_x && best.mv_y == iMinNow.mv_y)
      break;
  }

terminate_step:
  for (i=0; i < (blocksize_x>>2); i++)
  {
    for (j=0; j < (blocksize_y>>2); j++)
    {
      if(list == 0)
      {
        p_UMHex->fastme_l0_cost_bipred[blocktype][(currMB->block_y)+block_y+j][(currMB->block_x)+block_x+i] = min_mcost;
      }
      else
      {
        p_UMHex->fastme_l1_cost_bipred[blocktype][(currMB->block_y)+block_y+j][(currMB->block_x)+block_x+i] = min_mcost;
      }
    }
  }

  mv1->mv_x = (short) (best.mv_x - pic_pix_x);
  mv1->mv_y = (short) (best.mv_y - pic_pix_y);

  return min_mcost;
}

/*!
************************************************************************
* \brief
*    Set motion vector predictor
************************************************************************
*/
void UMHEXSetMotionVectorPredictor (Macroblock *currMB, 
                                    MotionVector *pmv,
                                    struct pic_motion_params **mv_info,
                                    short       ref_frame,
                                    int         list,
                                    int         mb_x,
                                    int         mb_y,
                                    int         blockshape_x,
                                    int         blockshape_y,
                                    MEBlock    *mv_block)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  UMHexStruct *p_UMHex = p_Vid->p_UMHex;
  int mv_a, mv_b, mv_c;
  short pred_vec=0;
  int mvPredType, rFrameL, rFrameU, rFrameUR;
  int hv;

  PixelPos block_a, block_b, block_c, block_d;

  // added for bipred mode
  distblk *** fastme_l0_cost_flag = (p_UMHex->bipred_flag ? p_UMHex->fastme_l0_cost_bipred : p_UMHex->fastme_l0_cost);
  distblk *** fastme_l1_cost_flag = (p_UMHex->bipred_flag ? p_UMHex->fastme_l1_cost_bipred : p_UMHex->fastme_l1_cost);
  //Dynamic Search Range

  int dsr_temp_search_range[2];
  int dsr_mv_avail, dsr_mv_max, dsr_mv_sum, dsr_small_search_range;
  int *mb_size = p_Vid->mb_size[IS_LUMA];

  // neighborhood SAD init
  p_UMHex->SAD_a = 0;
  p_UMHex->SAD_b = 0;
  p_UMHex->SAD_c = 0;
  p_UMHex->SAD_d = 0;

  get4x4Neighbour(currMB, mb_x - 1           , mb_y    , mb_size, &block_a);
  get4x4Neighbour(currMB, mb_x               , mb_y - 1, mb_size, &block_b);
  get4x4Neighbour(currMB, mb_x + blockshape_x, mb_y - 1, mb_size, &block_c);
  get4x4Neighbour(currMB, mb_x - 1           , mb_y - 1, mb_size, &block_d);

  if (mb_y > 0)
  {
    if (mb_x < 8)  // first column of 8x8 blocks
    {
      if (mb_y==8)
      {
        if (blockshape_x == 16)      block_c.available  = 0;
      }
      else
      {
        if (mb_x+blockshape_x == 8)  block_c.available = 0;
      }
    }
    else
    {
      if (mb_x+blockshape_x == 16)   block_c.available = 0;
    }
  }

  if (!block_c.available)
  {
    block_c=block_d;
  }

  mvPredType = MVPRED_MEDIAN;

  if (!p_Vid->mb_aff_frame_flag)
  {
    rFrameL    = block_a.available    ? mv_info[block_a.pos_y][block_a.pos_x].ref_idx[list] : -1;
    rFrameU    = block_b.available    ? mv_info[block_b.pos_y][block_b.pos_x].ref_idx[list] : -1;
    rFrameUR   = block_c.available    ? mv_info[block_c.pos_y][block_c.pos_x].ref_idx[list] : -1;
  }
  else
  {
    if (p_Vid->mb_data[currMB->mbAddrX].mb_field)
    {
      rFrameL  = block_a.available
        ? (p_Vid->mb_data[block_a.mb_addr].mb_field
        ? mv_info[block_a.pos_y][block_a.pos_x].ref_idx[list]
      : mv_info[block_a.pos_y][block_a.pos_x].ref_idx[list] * 2) : -1;
      rFrameU  = block_b.available
        ? (p_Vid->mb_data[block_b.mb_addr].mb_field
        ? mv_info[block_b.pos_y][block_b.pos_x].ref_idx[list]
      : mv_info[block_b.pos_y][block_b.pos_x].ref_idx[list] * 2) : -1;
      rFrameUR = block_c.available
        ? (p_Vid->mb_data[block_c.mb_addr].mb_field
        ? mv_info[block_c.pos_y][block_c.pos_x].ref_idx[list]
      : mv_info[block_c.pos_y][block_c.pos_x].ref_idx[list] * 2) : -1;
    }
    else
    {
      rFrameL = block_a.available
        ? (p_Vid->mb_data[block_a.mb_addr].mb_field
        ? mv_info[block_a.pos_y][block_a.pos_x].ref_idx[list] >>1
        : mv_info[block_a.pos_y][block_a.pos_x].ref_idx[list]) : -1;
      rFrameU    = block_b.available    ?
        p_Vid->mb_data[block_b.mb_addr].mb_field ?
        mv_info[block_b.pos_y][block_b.pos_x].ref_idx[list] >>1:
      mv_info[block_b.pos_y][block_b.pos_x].ref_idx[list] :
      -1;
      rFrameUR    = block_c.available    ?
        p_Vid->mb_data[block_c.mb_addr].mb_field ?
        mv_info[block_c.pos_y][block_c.pos_x].ref_idx[list] >>1:
      mv_info[block_c.pos_y][block_c.pos_x].ref_idx[list] :
      -1;
    }
  }

  /* Prediction if only one of the neighbors uses the reference frame
  * we are checking
  */
  if(rFrameL == ref_frame && rFrameU != ref_frame && rFrameUR != ref_frame)       mvPredType = MVPRED_L;
  else if(rFrameL != ref_frame && rFrameU == ref_frame && rFrameUR != ref_frame)  mvPredType = MVPRED_U;
  else if(rFrameL != ref_frame && rFrameU != ref_frame && rFrameUR == ref_frame)  mvPredType = MVPRED_UR;
  // Directional predictions
  if(blockshape_x == 8 && blockshape_y == 16)
  {
    if(mb_x == 0)
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
    else
    {
      if( rFrameUR == ref_frame)
        mvPredType = MVPRED_UR;
    }
  }
  else if(blockshape_x == 16 && blockshape_y == 8)
  {
    if(mb_y == 0)
    {
      if(rFrameU == ref_frame)
        mvPredType = MVPRED_U;
    }
    else
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
  }

  // neighborhood SAD prediction
  if((p_Inp->UMHexDSR == 1 || p_Inp->BiPredMotionEstimation == 1))
  {
    p_UMHex->SAD_a = block_a.available ? ((list==1) ? (fastme_l1_cost_flag[p_UMHex->UMHEX_blocktype][block_a.pos_y][block_a.pos_x]) : (fastme_l0_cost_flag[p_UMHex->UMHEX_blocktype][block_a.pos_y][block_a.pos_x])) : 0;
    p_UMHex->SAD_b = block_b.available ? ((list==1) ? (fastme_l1_cost_flag[p_UMHex->UMHEX_blocktype][block_b.pos_y][block_b.pos_x]) : (fastme_l0_cost_flag[p_UMHex->UMHEX_blocktype][block_b.pos_y][block_b.pos_x])) : 0;
    p_UMHex->SAD_d = block_d.available ? ((list==1) ? (fastme_l1_cost_flag[p_UMHex->UMHEX_blocktype][block_d.pos_y][block_d.pos_x]) : (fastme_l0_cost_flag[p_UMHex->UMHEX_blocktype][block_d.pos_y][block_d.pos_x])) : 0;
    p_UMHex->SAD_c = block_c.available ? ((list==1) ? (fastme_l1_cost_flag[p_UMHex->UMHEX_blocktype][block_c.pos_y][block_c.pos_x]) : (fastme_l0_cost_flag[p_UMHex->UMHEX_blocktype][block_c.pos_y][block_c.pos_x])) : p_UMHex->SAD_d;
  }
  for (hv=0; hv < 2; hv++)
  {
    if (!p_Vid->mb_aff_frame_flag || hv==0)
    {
      mv_a = block_a.available  ? mv_info[block_a.pos_y][block_a.pos_x].mv[list].mv_x : 0;
      mv_b = block_b.available  ? mv_info[block_b.pos_y][block_b.pos_x].mv[list].mv_x : 0;
      mv_c = block_c.available  ? mv_info[block_c.pos_y][block_c.pos_x].mv[list].mv_x : 0;
    }
    else
    {
      if (p_Vid->mb_data[currMB->mbAddrX].mb_field)
      {
        mv_a = block_a.available  ? p_Vid->mb_data[block_a.mb_addr].mb_field
          ? mv_info[block_a.pos_y][block_a.pos_x].mv[list].mv_y
        : mv_info[block_a.pos_y][block_a.pos_x].mv[list].mv_y / 2
          : 0;
        mv_b = block_b.available  ? p_Vid->mb_data[block_b.mb_addr].mb_field
          ? mv_info[block_b.pos_y][block_b.pos_x].mv[list].mv_y
        : mv_info[block_b.pos_y][block_b.pos_x].mv[list].mv_y / 2
          : 0;
        mv_c = block_c.available  ? p_Vid->mb_data[block_c.mb_addr].mb_field
          ? mv_info[block_c.pos_y][block_c.pos_x].mv[list].mv_y
        : mv_info[block_c.pos_y][block_c.pos_x].mv[list].mv_y / 2
          : 0;
      }
      else
      {
        mv_a = block_a.available  ? p_Vid->mb_data[block_a.mb_addr].mb_field
          ? mv_info[block_a.pos_y][block_a.pos_x].mv[list].mv_y * 2
          : mv_info[block_a.pos_y][block_a.pos_x].mv[list].mv_y
        : 0;
        mv_b = block_b.available  ? p_Vid->mb_data[block_b.mb_addr].mb_field
          ? mv_info[block_b.pos_y][block_b.pos_x].mv[list].mv_y * 2
          : mv_info[block_b.pos_y][block_b.pos_x].mv[list].mv_y
        : 0;
        mv_c = block_c.available  ? p_Vid->mb_data[block_c.mb_addr].mb_field
          ? mv_info[block_c.pos_y][block_c.pos_x].mv[list].mv_y * 2
          : mv_info[block_c.pos_y][block_c.pos_x].mv[list].mv_y
        : 0;
      }
    }

    switch (mvPredType)
    {
    case MVPRED_MEDIAN:
      if(!(block_b.available || block_c.available))
      {
        pred_vec = (short) mv_a;
      }
      else
      {
        pred_vec = (short) (mv_a+mv_b+mv_c-imin(mv_a,imin(mv_b,mv_c))-imax(mv_a,imax(mv_b,mv_c)));
      }
      break;
    case MVPRED_L:
      pred_vec = (short) mv_a;
      break;
    case MVPRED_U:
      pred_vec = (short) mv_b;
      break;
    case MVPRED_UR:
      pred_vec = (short) mv_c;
      break;
    default:
      break;
    }

    if (hv == 0)
      pmv->mv_x = pred_vec;
    else
      pmv->mv_y = pred_vec;

    //Dynamic Search Range
    if (p_Inp->UMHexDSR)
    {
      dsr_mv_avail=block_a.available+block_b.available+block_c.available;
      if(dsr_mv_avail < 2)
      {
        dsr_temp_search_range[hv] = p_Inp->search_range[p_Vid->view_id];
      }
      else
      {
        dsr_mv_max = imax(iabs(mv_a),imax(iabs(mv_b),iabs(mv_c)));
        dsr_mv_sum = (iabs(mv_a)+iabs(mv_b)+iabs(mv_c));
        if(dsr_mv_sum == 0) dsr_small_search_range = (p_Inp->search_range[p_Vid->view_id] + 4) >> 3;
        else if(dsr_mv_sum > 3 ) dsr_small_search_range = (p_Inp->search_range[p_Vid->view_id] + 2) >>2;
        else dsr_small_search_range = (3*p_Inp->search_range[p_Vid->view_id] + 8) >> 4;
        dsr_temp_search_range[hv]=imin(p_Inp->search_range[p_Vid->view_id],imax(dsr_small_search_range,dsr_mv_max<<1));
        if(distblkmax(p_UMHex->SAD_a, distblkmax(p_UMHex->SAD_b,p_UMHex->SAD_c)) > p_UMHex->Threshold_DSR_MB[p_UMHex->UMHEX_blocktype])
          dsr_temp_search_range[hv] = p_Inp->search_range[p_Vid->view_id];
      }
    }
  }

  //Dynamic Search Range
  if (p_Inp->UMHexDSR) 
  {
    int search_range = imax(dsr_temp_search_range[0],dsr_temp_search_range[1]);
    search_range <<= 2;

    if      (p_Inp->full_search == 2) 
    {      
      mv_block->searchRange.min_x = -search_range;
      mv_block->searchRange.max_x =  search_range;
      mv_block->searchRange.min_y = -search_range;
      mv_block->searchRange.max_y =  search_range;
    }
    else if (p_Inp->full_search == 1) 
    {
      int scale = (imin(ref_frame,1)+1);
      mv_block->searchRange.min_x = -search_range / scale;
      mv_block->searchRange.max_x =  search_range / scale;
      mv_block->searchRange.min_y = -search_range / scale;
      mv_block->searchRange.max_y =  search_range / scale;
    }
    else                              
    {      
      int scale = ((imin(ref_frame,1)+1) * imin(2,p_UMHex->BlockType_LUT[(blockshape_y >> 2) - 1][(blockshape_x >> 2) - 1]));
      mv_block->searchRange.min_x = -search_range / scale;
      mv_block->searchRange.max_x =  search_range / scale;
      mv_block->searchRange.min_y = -search_range / scale;
      mv_block->searchRange.max_y =  search_range / scale;
    }
  }
}

#undef SEARCH_ONE_PIXEL

