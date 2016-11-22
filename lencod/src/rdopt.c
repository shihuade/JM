/*!
 ***************************************************************************
 * \file rdopt.c
 *
 * \brief
 *    Rate-Distortion optimized mode decision
 *
 * \author
 *    - Heiko Schwarz
 *    - Valeri George
 *    - Lowell Winger              <lwinger@lsil.com>
 *    - Alexis Michael Tourapis    <alexismt@ieee.org>
 * \date
 *    12. April 2001
 **************************************************************************
 */

#include <math.h>
#include <limits.h>

#include "global.h"

#include "rdopt.h"
#include "q_around.h"
#include "rdopt_coding_state.h"
#include "memalloc.h"
#include "mb_access.h"
#include "elements.h"
#include "intrarefresh.h"
#include "image.h"
#include "transform8x8.h"
#include "cabac.h"
#include "biariencode.h"
#include "slice.h"
#include "vlc.h"
#include "ratectl.h"            // head file for rate control
#include "mode_decision.h"
#include "rd_intra_jm.h"
#include "rd_intra_jm444.h"
#include "macroblock.h"
#include "q_offsets.h"
#include "conformance.h"
#include "errdo.h"
#include "mv_search.h"
#include "md_common.h"
#include "md_distortion.h"
#include "intra16x16.h"

#define FASTMODE 1

static void set_stored_macroblock_parameters      (Macroblock *currMB);
static void set_stored_macroblock_parameters_sp   (Macroblock *currMB);
static void set_stored_macroblock_parameters_mpass(Macroblock *currMB);
static void set_ref_and_motion_vectors_P_slice    (Macroblock *currMB, PicMotionParams **motion, Info8x8 *pred, int);
static void set_ref_and_motion_vectors_B_slice    (Macroblock *currMB, PicMotionParams **motion, Info8x8 *pred, int);

static distblk compute_sad4x4_cost (VideoParameters *p_Vid, imgpel **cur_img, imgpel **prd_img, int pic_opix_x, distblk min_cost);
static distblk compute_sse4x4_cost (VideoParameters *p_Vid, imgpel **cur_img, imgpel **prd_img, int pic_opix_x, distblk min_cost);
static distblk compute_satd4x4_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **prd_img, int pic_opix_x, distblk min_cost);
static distblk compute_comp4x4_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **prd_img, int pic_opix_x, distblk min_cost);

static distblk rdcost_for_4x4_intra_blocks     (Macroblock *currMB, int* nonzero, int b8, int b4, int ipmode, int lambda, int mostProbableMode, distblk min_rdcost);
static distblk rdcost_for_4x4_intra_blocks_444 (Macroblock *currMB, int* nonzero, int b8, int b4, int ipmode, int lambda, int mostProbableMode, distblk min_rdcost);

static inline void copy_motion_vectors_MB (Slice *currSlice, RD_DATA *rdopt)
{
  memcpy(&currSlice->all_mv [LIST_0][0][0][0][0], &rdopt->all_mv [LIST_0][0][0][0][0], 288 * currSlice->max_num_references * sizeof(MotionVector));  
}

Info8x8 init_info_8x8_struct(void)
{
  Info8x8 i8x8;
  i8x8.mode = 0;
  i8x8.pdir = 0;
  i8x8.ref[LIST_0] = 0;
  i8x8.ref[LIST_1] = -1;
  i8x8.bipred = 0;
  
  return i8x8;
}

void alloc_rd8x8data (RD_8x8DATA *rd_data)
{  
  get_mem2Dint(&rd_data->lrec, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
  get_mem2Dpel(&rd_data->mpr8x8, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
  get_mem3Dpel(&rd_data->mpr8x8CbCr, 2, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
  get_mem2Dpel(&rd_data->rec_mbY8x8, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
  get_mem3Dpel(&rd_data->rec_mb8x8_cr, 2, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
}

void free_rd8x8data (RD_8x8DATA *rd_data)
{
  free_mem3Dpel(rd_data->rec_mb8x8_cr);
  free_mem2Dpel(rd_data->rec_mbY8x8);
  free_mem3Dpel(rd_data->mpr8x8CbCr);
  free_mem2Dpel(rd_data->mpr8x8);  
  free_mem2Dint(rd_data->lrec);
}

/*!
 ************************************************************************
 * \brief
 *    delete structure for RD-optimized mode decision
 ************************************************************************
 */
void clear_rdopt (Slice *currSlice)
{  
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  RDOPTStructure *p_RDO  = currSlice->p_RDO;

  free_mem_DCcoeff (p_RDO->cofDC);
  free_mem_ACcoeff (p_RDO->cofAC);
  free_mem_ACcoeff (p_RDO->cofAC4x4intern);
  free_mem_ACcoeff_new(p_RDO->coefAC8x8);
  free_mem_ACcoeff_new(p_RDO->coefAC8x8intra);

  if (p_Inp->Transform8x8Mode)
  {
    free_mem_ACcoeff_new(p_RDO->cofAC8x8ts);
  }
  
  if (p_Vid->P444_joined)
  {
    free_mem_ACcoeff_new(p_RDO->cofAC4x4CbCrintern);
  }

  // Should create new functions for free/alloc of RD_8x8DATA
  free_rd8x8data(p_RDO->tr4x4);
  free(p_RDO->tr4x4);
  free_rd8x8data(p_RDO->tr8x8);
  free(p_RDO->tr8x8);

  free_mem4Dmv(p_RDO->all_mv8x8);

  free_mem3Dpel(p_RDO->rec4x4);
  free_mem3Dpel(p_RDO->rec8x8);
  free_mem2Dpel(p_RDO->pred);
  free_mem3Dpel(p_RDO->rec_mb);

  if (p_Inp->ProfileIDC == EXTENDED)
  {
    free_mem2Dint(p_RDO->lrec_rec);
    free_mem3Dint(p_RDO->lrec_rec_uv);
  }

  free_mem2D((byte **) p_RDO->l0_refframe);  
  free_mem2D((byte **) p_RDO->l1_refframe);  

  // structure for saving the coding state
  delete_coding_state (p_RDO->cs_mb);
  delete_coding_state (p_RDO->cs_b8);
  delete_coding_state (p_RDO->cs_cm);
  delete_coding_state (p_RDO->cs_tmp);
}

void setupDistCost(Slice *currSlice, InputParameters *p_Inp)
{
  switch(p_Inp->ModeDecisionMetric)
  {
  case ERROR_SAD:
    currSlice->compute_cost4x4 = compute_sad4x4_cost; 
    currSlice->compute_cost8x8 = compute_sad8x8_cost;
    currSlice->distI16x16      = distI16x16_sad;
  break;
  case ERROR_SSE:
    currSlice->compute_cost4x4 = compute_sse4x4_cost; 
    currSlice->compute_cost8x8 = compute_sse8x8_cost;
    currSlice->distI16x16      = distI16x16_sse;
  break;
  case ERROR_SATD:
    currSlice->compute_cost4x4 = compute_satd4x4_cost;
    currSlice->compute_cost8x8 = compute_satd8x8_cost;
    currSlice->distI16x16      = distI16x16_satd;
  break;
  default:
    currSlice->compute_cost4x4 = compute_comp4x4_cost;  
    currSlice->compute_cost8x8 = compute_comp8x8_cost;
    currSlice->distI16x16      = distI16x16_satd;
    break;
  }
}


/*!
 ************************************************************************
 * \brief
 *    create structure for RD-optimized mode decision
 ************************************************************************
 */
void init_rdopt (Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp; 
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  get_mem_DCcoeff (&p_RDO->cofDC);
  get_mem_ACcoeff (p_Vid, &p_RDO->cofAC);
  get_mem_ACcoeff (p_Vid, &p_RDO->cofAC4x4intern);
  p_RDO->cofAC4x4 = p_RDO->cofAC4x4intern[0][0];
  get_mem_ACcoeff_new(&p_RDO->coefAC8x8, 3);
  get_mem_ACcoeff_new(&p_RDO->coefAC8x8intra, 3);

  if (p_Inp->Transform8x8Mode)
  {
    get_mem_ACcoeff_new(&p_RDO->cofAC8x8ts, 3);
  }

  if (((p_RDO->tr4x4)  = (RD_8x8DATA *) calloc(1, sizeof(RD_8x8DATA)))==NULL) 
    no_mem_exit("init_rdopt: p_RDO->tr4x4");
  alloc_rd8x8data(p_RDO->tr4x4);
  if (((p_RDO->tr8x8)  = (RD_8x8DATA *) calloc(1, sizeof(RD_8x8DATA)))==NULL) 
    no_mem_exit("init_rdopt: p_RDO->tr4x4");
  alloc_rd8x8data(p_RDO->tr8x8);

  get_mem2Dpel(&p_RDO->pred, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
  get_mem3Dpel(&p_RDO->rec8x8, 3, BLOCK_SIZE_8x8, BLOCK_SIZE_8x8);
  get_mem3Dpel(&p_RDO->rec4x4, 3, BLOCK_SIZE, BLOCK_SIZE);
  get_mem3Dpel(&p_RDO->rec_mb, 3, MB_BLOCK_SIZE, MB_BLOCK_SIZE);

  if (p_Inp->ProfileIDC == EXTENDED)
  {
    get_mem2Dint(&p_RDO->lrec_rec, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
    get_mem3Dint(&p_RDO->lrec_rec_uv, 2, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
  }

  get_mem2D((byte ***) &p_RDO->l0_refframe, 4, 4);
  get_mem2D((byte ***) &p_RDO->l1_refframe, 4, 4);

  get_mem4Dmv(&p_RDO->all_mv8x8, 2, 2, 4, 4);

  if (p_Vid->P444_joined)
  {
    get_mem_ACcoeff_new(&p_RDO->cofAC4x4CbCrintern, 2);

    p_RDO->cofAC4x4CbCr[0] = p_RDO->cofAC4x4CbCrintern[0][0][0];
    p_RDO->cofAC4x4CbCr[1] = p_RDO->cofAC4x4CbCrintern[0][1][0];    
  }

  currSlice->set_lagrangian_multipliers = p_Inp->rdopt == 0 ? SetLagrangianMultipliersOff : SetLagrangianMultipliersOn;

  switch (p_Inp->rdopt)
  {
    case 0:
      currSlice->encode_one_macroblock = encode_one_macroblock_low;
      break;
    case 1:
    default:
      currSlice->encode_one_macroblock = encode_one_macroblock_high;
      break;
    case 2:
      currSlice->encode_one_macroblock = encode_one_macroblock_highfast;
      break;
    case 3:
      currSlice->encode_one_macroblock = encode_one_macroblock_highloss;
      break;
    case 4:
      currSlice->encode_one_macroblock = encode_one_macroblock_high_updated;
      break;
  }

  if (currSlice->mb_aff_frame_flag || (currSlice->UseRDOQuant && currSlice->RDOQ_QP_Num > 1))
    currSlice->set_stored_mb_parameters = set_stored_macroblock_parameters_mpass;
  else
  {
    if (currSlice->slice_type == SP_SLICE || currSlice->slice_type == SI_SLICE)
      currSlice->set_stored_mb_parameters = set_stored_macroblock_parameters_sp;
    else
      currSlice->set_stored_mb_parameters = set_stored_macroblock_parameters;
  }
  
  // structure for saving the coding state
  p_RDO->cs_mb  = create_coding_state (p_Inp);
  p_RDO->cs_b8  = create_coding_state (p_Inp);
  p_RDO->cs_cm  = create_coding_state (p_Inp);
  p_RDO->cs_tmp = create_coding_state (p_Inp);
  if (p_Inp->CtxAdptLagrangeMult == 1)
  {
    p_Vid->mb16x16_cost = CALM_MF_FACTOR_THRESHOLD;
    p_RDO->lambda_mf_factor = 1.0;
  }

  currSlice->rdcost_for_4x4_intra_blocks = (p_Vid->yuv_format == YUV444) ? rdcost_for_4x4_intra_blocks_444 : rdcost_for_4x4_intra_blocks;
  currSlice->rdcost_for_8x8_intra_blocks = (p_Vid->yuv_format == YUV444) ? rdcost_for_8x8_intra_blocks_444 : rdcost_for_8x8_intra_blocks;

  if (currSlice->mb_aff_frame_flag)
    currSlice->intra_chroma_RD_decision = intra_chroma_RD_decision_mbaff;
  else
    currSlice->intra_chroma_RD_decision = intra_chroma_RD_decision;
  if (p_Inp->rdopt == 0)
  {
    currSlice->mode_decision_for_I8x8_blocks = (p_Vid->yuv_format == YUV444) ? mode_decision_for_I8x8_blocks_JM_Low444 : mode_decision_for_I8x8_blocks_JM_Low;
    currSlice->mode_decision_for_I4x4_blocks = (p_Vid->yuv_format == YUV444) ? mode_decision_for_I4x4_blocks_JM_Low444 : mode_decision_for_I4x4_blocks_JM_Low;
  }
  else
  {
    currSlice->mode_decision_for_I8x8_blocks = (p_Vid->yuv_format == YUV444) ? mode_decision_for_I8x8_blocks_JM_High444 : mode_decision_for_I8x8_blocks_JM_High;
    currSlice->mode_decision_for_I4x4_blocks = (p_Vid->yuv_format == YUV444) ? mode_decision_for_I4x4_blocks_JM_High444 : mode_decision_for_I4x4_blocks_JM_High;
  }

  currSlice->find_sad_16x16 = find_sad_16x16_JM;

  if (currSlice->slice_type == B_SLICE)
    currSlice->set_ref_and_motion_vectors = set_ref_and_motion_vectors_B_slice;
  else
    currSlice->set_ref_and_motion_vectors = set_ref_and_motion_vectors_P_slice;

  setupDistortion(currSlice);
  setupDistCost  (currSlice, p_Inp);
}

/*!
 *************************************************************************************
 * \brief
 *    Updates the pixel map that shows, which reference frames are reliable for
 *    each MB-area of the picture.
 *
 * \note
 *    The new values of the p_Vid->pixel_map are taken from the temporary buffer p_Vid->refresh_map
 *
 *************************************************************************************
 */
void UpdatePixelMap(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int mx,my,y,x,i,j;
  if (p_Vid->type==I_SLICE)
  {
    memset(p_Vid->pixel_map, 1, p_Vid->height * p_Vid->width * sizeof(byte));
  }
  else
  {
    for (my=0; my<p_Vid->height >> 3; my++)
    {
      for (mx=0; mx<p_Vid->width >> 3;  mx++)
      {
        j = my*8 + 8;
        i = mx*8 + 8;
        if (p_Vid->refresh_map[my][mx])
        {
          for (y=my*8; y<j; y++)
            memset(&p_Vid->pixel_map[y][mx*8], 1, 8 * sizeof(byte));
        }
        else
        {
          for (y=my*8; y<j; y++)
          {
            for (x=mx*8; x<i; x++)
            {
              p_Vid->pixel_map[y][x] = (byte) imin(p_Vid->pixel_map[y][x] + 1, p_Inp->num_ref_frames+1);
            }
          }
        }
      }
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Checks if a given reference frame is reliable for the current
 *    macroblock, given the motion vectors that the motion search has
 *    returned.
 *
 * \return
 *    If the return value is 1, the reference frame is reliable. If it
 *    is 0, then it is not reliable.
 *
 * \note
 *    A specific area in each reference frame is assumed to be unreliable
 *    if the same area has been intra-refreshed in a subsequent frame.
 *    The information about intra-refreshed areas is kept in the p_Vid->pixel_map.
 *
 *************************************************************************************
 */
int CheckReliabilityOfRef (Macroblock *currMB, int block, int list_idx, int ref, int mode)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;

  int y,x, block_y, block_x, dy, dx, y_pos, x_pos, yy, xx, pres_x, pres_y;
  int maxold_x  = p_Vid->width  - 1;
  int maxold_y  = p_Vid->height - 1;
  int ref_frame = ref + 1;

  int by0 = (mode>=4?2*(block >> 1):mode==2?2*block:0);
  int by1 = by0 + (mode>=4||mode==2?2:4);
  int bx0 = (mode>=4?2*(block & 0x01):mode==3?2*block:0);
  int bx1 = bx0 + (mode>=4||mode==3?2:4);

  for (block_y=by0; block_y<by1; block_y++)
  {
    for (block_x=bx0; block_x<bx1; block_x++)
    {
      y_pos  = currSlice->all_mv[list_idx][ref][mode][block_y][block_x].mv_y;
      y_pos += (currMB->block_y + block_y) * BLOCK_SIZE * 4;
      x_pos  = currSlice->all_mv[list_idx][ref][mode][block_y][block_x].mv_x;
      x_pos += (currMB->block_x + block_x) * BLOCK_SIZE * 4;

      /* Here we specify which pixels of the reference frame influence
      the reference values and check their reliability. This is
      based on the function Get_Reference_Pixel */

      dy = y_pos & 3;
      dx = x_pos & 3;

      y_pos = (y_pos - dy) >> 2;
      x_pos = (x_pos - dx) >> 2;

      if (dy==0 && dx==0) //full-pel
      {
        for (y=y_pos ; y < y_pos + BLOCK_SIZE ; y++)
          for (x=x_pos ; x < x_pos + BLOCK_SIZE ; x++)
            if (p_Vid->pixel_map[iClip3(0,maxold_y,y)][iClip3(0,maxold_x,x)] < ref_frame)
              return 0;
      }
      else  /* other positions */
      {
        if (dy == 0)
        {
          for (y = y_pos ; y < y_pos + BLOCK_SIZE ; y++)
          {
            pres_y = iClip3(0, maxold_y, y);
            for (x = x_pos ; x < x_pos + BLOCK_SIZE ; x++)
            {
              for(xx = -2 ; xx < 4 ; xx++) 
              {
                pres_x = iClip3(0, maxold_x, x + xx);
                if (p_Vid->pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
          }
        }
        else if (dx == 0)
        {
          for (y = y_pos ; y < y_pos + BLOCK_SIZE ; y++)
            for (x = x_pos ; x < x_pos + BLOCK_SIZE ; x++)
            {
              pres_x = iClip3(0,maxold_x,x);
              for(yy=-2;yy<4;yy++) 
              {
                pres_y = iClip3(0,maxold_y, yy + y);
                if (p_Vid->pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
        }
        else if (dx == 2)
        {
          for (y = y_pos ; y < y_pos + BLOCK_SIZE ; y++)
            for (x = x_pos ; x < x_pos + BLOCK_SIZE ; x++)
            {
              for(yy=-2;yy<4;yy++) 
              {
                pres_y = iClip3(0,maxold_y, yy + y);
                for(xx=-2;xx<4;xx++) 
                {
                  pres_x = iClip3(0,maxold_x, xx + x);
                  if (p_Vid->pixel_map[pres_y][pres_x] < ref_frame)
                    return 0;
                }
              }
            }
        }
        else if (dy == 2)
        {
          for (y = y_pos ; y < y_pos + BLOCK_SIZE ; y++)
            for (x = x_pos ; x < x_pos + BLOCK_SIZE ; x++)
            {
              for(xx=-2;xx<4;xx++) 
              {
                pres_x = iClip3(0,maxold_x, xx + x);
                for(yy=-2;yy<4;yy++) 
                {
                  pres_y = iClip3(0,maxold_y, yy + y);
                  if (p_Vid->pixel_map[pres_y][pres_x] < ref_frame)
                    return 0;
                }
              }
            }
        }
        else
        {
          for (y = y_pos ; y < y_pos + BLOCK_SIZE ; y++)
          {
            for (x = x_pos ; x < x_pos + BLOCK_SIZE ; x++)
            {
              pres_y = dy == 1 ? y : y + 1;
              pres_y = iClip3(0,maxold_y,pres_y);

              for(xx = -2; xx < 4; xx++)
              {
                pres_x = iClip3(0,maxold_x,xx + x);
                if (p_Vid->pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }

              pres_x = dx == 1 ? x : x + 1;
              pres_x = iClip3(0,maxold_x,pres_x);

              for(yy=-2;yy<4;yy++)
              {
                pres_y = iClip3(0,maxold_y, yy + y);
                if (p_Vid->pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
          }
        }
      }
    }
  }
  return 1;
}

/*!
 *************************************************************************************
 * \brief
 *    R-D Cost for an 4x4 Intra block
 *************************************************************************************
 */
static distblk rdcost_for_4x4_intra_blocks (Macroblock *currMB,
                                       int*        nonzero,
                                       int         b8,
                                       int         b4,
                                       int         ipmode,
                                       int         lambda,
                                       int         mostProbableMode,
                                       distblk       min_rdcost)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  distblk  rdcost;
  int     dummy = 0, rate;
  distblk   distortion  = 0;
  int     block_x     = ((b8 & 0x01) << 3) + ((b4 & 0x01) << 2);
  int     block_y     = ((b8 >> 1) << 3) + ((b4 >> 1) << 2);
  int     pic_pix_x   = currMB->pix_x  + block_x;
  int     pic_pix_y   = currMB->pix_y  + block_y;
  int     pic_opix_y  = currMB->opix_y + block_y;

  SyntaxElement  se;
  const int      *partMap   = assignSE2partition[currSlice->partition_mode];
  //--- choose data partition ---
  DataPartition  *dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);

  //===== perform forward transform, Q, IQ, inverse transform, Reconstruction =====
  //select_transform(currMB);

  currMB->ipmode_DPCM = (short) ipmode;
  *nonzero = currMB->residual_transform_quant_luma_4x4 (currMB, PLANE_Y, block_x, block_y, &dummy, 1);

  //===== get distortion (SSD) of 4x4 block =====
  distortion += compute_SSE4x4(&p_Vid->pCurImg[pic_opix_y], &p_Vid->enc_picture->imgY[pic_pix_y], pic_pix_x, pic_pix_x);
#if INTRA_RDCOSTCALC_ET
  // check if already distortion larger than min_rdcost
  if (distortion >= min_rdcost)
  {
    return (distortion);
  }
#endif
  currMB->ipmode_DPCM = NO_INTRA_PMODE;

  //===== RATE for INTRA PREDICTION MODE  (SYMBOL MODE MUST BE SET TO CAVLC) =====
  se.value1 = (mostProbableMode == ipmode) ? -1 : ipmode < mostProbableMode ? ipmode : ipmode - 1;

  //--- set position and type ---
  se.context = (b8 << 2) + b4;
  se.type    = SE_INTRAPREDMODE;

  //--- encode and update rate ---
  currSlice->writeIntraPredMode (&se, dataPart);
  rate = se.len;

  //===== RATE for LUMINANCE COEFFICIENTS =====
  if (currSlice->symbol_mode == CAVLC)
  {
    rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, b4, 0);
  }
  else
  {
    rate  += writeCoeff4x4_CABAC (currMB, PLANE_Y, b8, b4, 1);
  }

  rdcost = distortion + weighted_cost(lambda, rate); //((distblk)lambda) * rate;

  currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);

  return rdcost;
}


/*!
 *************************************************************************************
 * \brief
 *    R-D Cost for an 4x4 Intra block (for 4:4:4 formats)
 *************************************************************************************
 */
static distblk rdcost_for_4x4_intra_blocks_444 (Macroblock *currMB,
                                                int*        nonzero,
                                                int         b8,
                                                int         b4,
                                                int         ipmode,
                                                int         lambda,
                                                int         mostProbableMode,
                                                distblk     min_rdcost)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  distblk  rdcost;
  int     dummy = 0, rate;
  distblk   distortion  = 0;
  int     block_x     = ((b8 & 0x01) << 3) + ((b4 & 0x01) << 2);
  int     block_y     = ((b8 >> 1) << 3) + ((b4 >> 1) << 2);
  int     pic_pix_x   = currMB->pix_x  + block_x;
  int     pic_pix_y   = currMB->pix_y  + block_y;
  int     pic_opix_y  = currMB->opix_y + block_y;

  SyntaxElement  se;
  const int      *partMap   = assignSE2partition[currSlice->partition_mode];
  DataPartition  *dataPart;
  ColorPlane k;

  //===== perform forward transform, Q, IQ, inverse transform, Reconstruction =====
  //select_transform(currMB);
  *nonzero = currMB->residual_transform_quant_luma_4x4 (currMB, PLANE_Y, block_x, block_y, &dummy, 1);

  //===== get distortion (SSD) of 4x4 block =====
  distortion += compute_SSE4x4(&p_Vid->pCurImg[pic_opix_y], &p_Vid->enc_picture->imgY[pic_pix_y], pic_pix_x, pic_pix_x);

  if(p_Vid->P444_joined)
  {
    for (k = PLANE_U; k <= PLANE_V; k++)
    {
      select_plane(p_Vid, k);
      currMB->c_nzCbCr[k] = currMB->residual_transform_quant_luma_4x4(currMB, k, block_x, block_y, &dummy, 1);
      distortion += compute_SSE4x4(&p_Vid->pCurImg[pic_opix_y], &p_Vid->enc_picture->p_curr_img[pic_pix_y], pic_pix_x, pic_pix_x);
    }
    select_plane(p_Vid, PLANE_Y);
  }
  currMB->ipmode_DPCM = NO_INTRA_PMODE;

  //===== RATE for INTRA PREDICTION MODE  (SYMBOL MODE MUST BE SET TO CAVLC) =====
  se.value1 = (mostProbableMode == ipmode) ? -1 : ipmode < mostProbableMode ? ipmode : ipmode - 1;

  //--- set position and type ---
  se.context = (b8 << 2) + b4;
  se.type    = SE_INTRAPREDMODE;

  //--- choose data partition ---
  dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);
  //--- encode and update rate ---
  currSlice->writeIntraPredMode (&se, dataPart);
  rate = se.len;

  //===== RATE for LUMINANCE COEFFICIENTS =====
  if (currSlice->symbol_mode == CAVLC)
  {
    rate  += currSlice->writeCoeff4x4_CAVLC (currMB, LUMA, b8, b4, 0);
    if(p_Vid->P444_joined) 
    {
      rate  += currSlice->writeCoeff4x4_CAVLC (currMB, CB, b8, b4, 0);
      rate  += currSlice->writeCoeff4x4_CAVLC (currMB, CR, b8, b4, 0);
    }
  }
  else
  {
    rate  += writeCoeff4x4_CABAC (currMB, PLANE_Y, b8, b4, 1);
    if(p_Vid->P444_joined) 
    {
      rate  += writeCoeff4x4_CABAC (currMB, PLANE_U, b8, b4, 1);
      rate  += writeCoeff4x4_CABAC (currMB, PLANE_V, b8, b4, 1);
    }
  }
  rdcost = distortion + weight_cost(lambda,rate);
  currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);

  return rdcost;
}

/*!
*************************************************************************************
* \brief
*    R-D Cost for an 8x8 Partition
*************************************************************************************
*/
distblk rdcost_for_8x8blocks (Macroblock *currMB, // --> Current macroblock to code
                             RD_8x8DATA *dataTr,                                                  
                             int*    cnt_nonz,   // --> number of nonzero coefficients
                             int64*  cbp_blk,    // --> cbp blk
                             int  lambda,     // <-- lagrange multiplier
                             int     block,      // <-- 8x8 block number
                             short   mode,       // <-- partitioning mode
                             Info8x8 *part,      // <-- partition information
                             distblk  min_rdcost)     
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  pic_parameter_set_rbsp_t *active_pps = p_Vid->active_pps;
  int  rate=0;
  distblk distortion=0;

  int  dummy = 0, mrate;
  int  list_mode[2];
  int  cbp     = 0;
  int  pax     = 8*(block & 0x01);
  int  pay     = 8*(block >> 1);
  int  i0      = pax >> 2;
  int  j0      = pay >> 2;
  int  direct  = (currSlice->slice_type == B_SLICE && mode==0);
  int  b8value = B8Mode2Value (currSlice, (short) mode, part->pdir);

  SyntaxElement se;  
  DataPartition *dataPart;
  const int     *partMap   = assignSE2partition[currSlice->partition_mode];

  EncodingEnvironmentPtr eep_dp;

  short pdir = part->pdir;
  int l0_ref = part->ref[LIST_0];
  int l1_ref = part->ref[LIST_1];

  //=====
  //=====  GET COEFFICIENTS, RECONSTRUCTIONS, CBP
  //=====
  currMB->b8x8[block].bipred = part->bipred;
  currMB->ar_mode = (short) ((mode != 0)? mode: P8x8);
 
  
  if (direct)
  {
    int p_dir = currSlice->direct_pdir[currMB->block_y + j0][currMB->block_x + i0];
    if (p_dir < 0) // mode not allowed
      return (DISTBLK_MAX);
    else
    {
      int list_mode[2] = {0, 0};
      *cnt_nonz = currSlice->luma_residual_coding_8x8 (currMB, &cbp, cbp_blk, block, (short) p_dir, list_mode,
        currSlice->direct_ref_idx[currMB->block_y+j0][currMB->block_x+i0]);
    }
  }
  else
  {
    if (pdir == 2 && active_pps->weighted_bipred_idc == 1)
    {

      int weight_sum = (active_pps->weighted_bipred_idc == 1)? currSlice->wbp_weight[0][l0_ref][l1_ref][0] + currSlice->wbp_weight[1][l0_ref][l1_ref][0] : 0;
      if (weight_sum < -128 ||  weight_sum > 127)
      {
        return (DISTBLK_MAX);
      }
    }

    list_mode[0]  = (pdir==0||pdir==2 ? mode : 0);
    list_mode[1]   = (pdir==1||pdir==2 ? mode : 0);
    *cnt_nonz = currSlice->luma_residual_coding_8x8 (currMB, &cbp, cbp_blk, block, pdir, list_mode, part->ref);
  }

  if(p_Vid->P444_joined) 
  {
    *cnt_nonz += ( currSlice->coeff_cost_cr[1] + currSlice->coeff_cost_cr[2] );
  }

  // RDOPT with losses
  if (p_Inp->rdopt==3)
  {
    //Zhifeng 090611
    distortion = p_Vid->estimate_distortion(currMB, block, 8, mode, pdir, min_rdcost);    //===== get residue =====
  }
  else
  {    
    distortion += compute_SSE8x8(&p_Vid->pCurImg[currMB->opix_y + pay], &p_Vid->enc_picture->imgY[currMB->pix_y + pay], currMB->pix_x + pax, currMB->pix_x + pax);
  }
    
  if (p_Vid->P444_joined)
  {
    distortion += compute_SSE8x8(&p_Vid->pImgOrg[1][currMB->opix_y + pay], &p_Vid->enc_picture->imgUV[0][currMB->pix_y + pay], currMB->pix_x + pax, currMB->pix_x + pax);
    distortion += compute_SSE8x8(&p_Vid->pImgOrg[2][currMB->opix_y + pay], &p_Vid->enc_picture->imgUV[1][currMB->pix_y + pay], currMB->pix_x + pax, currMB->pix_x + pax);
  }    
  
  // Early termination
  if (distortion >= min_rdcost)
    return (distortion);
  //printf("distortion %.2f %.2f\n", (double) distortion, min_rdcost);

  if(p_Vid->P444_joined) 
  {   
    cbp |= currSlice->cmp_cbp[1];
    cbp |= currSlice->cmp_cbp[2];

    currSlice->cmp_cbp[1] = cbp;
    currSlice->cmp_cbp[2] = cbp;
  }

  //=====
  //=====   GET RATE
  //=====
  //----- block 8x8 mode -----
  if (currSlice->symbol_mode == CAVLC)
  {
    ue_linfo (b8value, dummy, &mrate, &dummy);
    rate += mrate;
  }
  else
  {
    se.value1 = b8value;
    se.type   = SE_MBTYPE;
    dataPart  = &(currSlice->partArr[partMap[se.type]]);
    currSlice->writeB8_typeInfo(&se, dataPart);
    rate += se.len;
  }

  //----- motion information -----
  if (!direct)
  {
    if ((currSlice->num_ref_idx_active[LIST_0] > 1 ) && (pdir==0 || pdir==2))
      rate  += writeReferenceFrame (currMB, i0, j0, LIST_0, l0_ref);

    if ((currSlice->num_ref_idx_active[LIST_1] > 1 && currSlice->slice_type == B_SLICE ) && (pdir==1 || pdir==2))
    {
      rate  += writeReferenceFrame (currMB, i0, j0, LIST_1, l1_ref);
    }

    if (pdir==0 || pdir==2)
    {
      rate  += writeMotionVector8x8 (currMB, i0, j0, i0 + 2, j0 + 2, l0_ref, LIST_0, mode, currMB->b8x8[block].bipred);
    }
    if (pdir==1 || pdir==2)
    {
      rate  += writeMotionVector8x8 (currMB, i0, j0, i0 + 2, j0 + 2, l1_ref, LIST_1, mode, currMB->b8x8[block].bipred);
    }
  }

  //----- coded block pattern (for CABAC only) -----
  if (!currSlice->symbol_mode == CAVLC)
  {
    dataPart = &(currSlice->partArr[partMap[SE_CBP]]);
    eep_dp   = &(dataPart->ee_cabac);
    mrate    = arienco_bits_written (eep_dp);
    writeCBP_BIT_CABAC (currMB, block, ((*cnt_nonz>0)?1:0), dataTr->cbp8x8, eep_dp, currSlice->tex_ctx);
    mrate    = arienco_bits_written (eep_dp) - mrate;
    rate    += mrate;
  }

  //----- luminance coefficients -----
  if (*cnt_nonz)
  {
    rate += writeCoeff8x8 ( currMB, PLANE_Y, block, mode, currMB->luma_transform_size_8x8_flag);
  }

  if(p_Vid->P444_joined)
  {
    rate += writeCoeff8x8( currMB, PLANE_U, block, mode, currMB->luma_transform_size_8x8_flag );
    rate += writeCoeff8x8( currMB, PLANE_V, block, mode, currMB->luma_transform_size_8x8_flag );
  }
  return (distortion + weight_cost(lambda, rate));
}


/*
 *************************************************************************************
 * \brief
 *    Gets mode offset for intra16x16 mode
 *************************************************************************************
 */
short I16Offset (int cbp, short i16mode)
{
  return (short) ((cbp & 15 ? 13 : 1) + i16mode + ((cbp & 0x30) >> 2));
}


/*!
 *************************************************************************************
 * \brief
 *    Sets modes and reference frames for a macroblock
 *************************************************************************************
 */
void set_modes_and_refs_for_blocks_i_slice(Macroblock *currMB, short mode)
{
  VideoParameters *p_Vid = currMB->p_Vid;

  int i,j; 
  Block8x8Info *b8x8info = p_Vid->b8x8info;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  //--- macroblock type ---
  currMB->mb_type = mode;
  currMB->ar_mode = mode;

  for( i = 0; i < 4; i++) 
  {
    currMB->b8x8[i] = b8x8info->best[mode][i];
  }

  //--- block 8x8 mode and prediction direction ---
  switch (mode)
  {
  case I4MB:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = IBLOCK;
      currMB->b8x8[i].pdir = -1;
    }
    break;
  case I16MB:
    //memset(currMB->b8x8, 0, 4 * sizeof(short));
    currMB->b8x8[0].mode = 0;
    currMB->b8x8[1].mode = 0;
    currMB->b8x8[2].mode = 0;
    currMB->b8x8[3].mode = 0;
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].pdir = -1;
    }
    break;
  case I8MB:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = I8MB;
      currMB->b8x8[i].pdir = -1;
    }
    //switch to 8x8 transform
    currMB->luma_transform_size_8x8_flag = TRUE;
    break;
  case IPCM:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = IPCM;
      currMB->b8x8[i].pdir = -1;
    }
    currMB->luma_transform_size_8x8_flag = FALSE;
    break;
  default:
    printf ("Unsupported mode in set_modes_and_refs_for_blocks!\n");
    exit (1);
  }

  //--- reference frame arrays ---
  for (j = currMB->block_y; j < currMB->block_y + 4; j++)
  {
    for (i = currMB->block_x; i < currMB->block_x + 4;i++)
    {
      motion[j][i].ref_pic [LIST_0] = NULL;
      motion[j][i].ref_idx [LIST_0] = -1;
    }
  }
}


/*!
 *************************************************************************************
 * \brief
 *    Sets modes and reference frames for a macroblock
 *************************************************************************************
 */
void set_modes_and_refs_for_blocks_p_slice(Macroblock *currMB, short mode)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;

  int i,j,k; 

  Block8x8Info *b8x8info = p_Vid->b8x8info;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  //--- macroblock type ---
  currMB->mb_type = mode;

  for( i = 0; i < 4; i++) 
  {
    currMB->b8x8[i] = b8x8info->best[mode][i];
  }

  currMB->ar_mode = mode;

  //--- block 8x8 mode and prediction direction ---
  switch (mode)
  {
  case 0:
    currMB->b8x8[0].mode = 0;
    currMB->b8x8[1].mode = 0;
    currMB->b8x8[2].mode = 0;
    currMB->b8x8[3].mode = 0;
    currMB->b8x8[0].pdir = 0;
    currMB->b8x8[1].pdir = 0;
    currMB->b8x8[2].pdir = 0;
    currMB->b8x8[3].pdir = 0;
    break;
  case 1:
  case 2:
  case 3:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = (char) mode;
    }
    break;
  case P8x8:
    break;
  case I4MB:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = IBLOCK;
      currMB->b8x8[i].pdir = -1;
    }
    break;
  case I16MB:
    //memset(currMB->b8x8, 0, 4 * sizeof(short));
    currMB->b8x8[0].mode = 0;
    currMB->b8x8[1].mode = 0;
    currMB->b8x8[2].mode = 0;
    currMB->b8x8[3].mode = 0;
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].pdir = -1;
    }
    break;
  case I8MB:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = I8MB;
      currMB->b8x8[i].pdir = -1;
    }
    //switch to 8x8 transform
    currMB->luma_transform_size_8x8_flag = TRUE;
    break;
  case IPCM:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = IPCM;
      currMB->b8x8[i].pdir = -1;
    }
    currMB->luma_transform_size_8x8_flag = FALSE;
    break;
  default:
    printf ("Unsupported mode in set_modes_and_refs_for_blocks!\n");
    exit (1);
  }

  //--- reference frame arrays ---
  if (mode==0 || mode==I4MB || mode==I16MB || mode==I8MB || mode == IPCM || mode == SI4MB)
  {
    if (!mode) // Skip
    {
      for (j = currMB->block_y; j < currMB->block_y + 4; j++)
        for (i = currMB->block_x; i < currMB->block_x + 4; i++)
          motion[j][i].ref_idx[LIST_0] = 0;
    }
    else // Intra
    {
      for (j = currMB->block_y; j < currMB->block_y + 4; j++)
        for (i = currMB->block_x; i < currMB->block_x + 4; i++)
          motion[j][i].ref_idx[LIST_0] = -1;
    }
  }
  else
  {
    if (mode == 1)
    {
      short curref = b8x8info->best[mode][0].pdir == 0 ? b8x8info->best[mode][0].ref[LIST_0] : -1;
      for (j = currMB->block_y; j < currMB->block_y + 4; j++)
      {
        for (i = currMB->block_x; i < currMB->block_x + 4;i++)
        {
          motion[j][i].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
          motion[j][i].ref_idx [LIST_0] = (char) curref;
        }
      }
    }
    else if (mode == 2)
    {
      short curref = b8x8info->best[mode][0].pdir == 0 ? b8x8info->best[mode][0].ref[LIST_0] : -1;      
      for (j = currMB->block_y; j < currMB->block_y + 2; j++)
      {
        for (i = currMB->block_x; i < currMB->block_x + 4;i++)
        {
          motion[j][i].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
          motion[j][i].ref_idx [LIST_0] = (char) curref;
        }
      }
      curref = b8x8info->best[mode][2].pdir == 0 ? b8x8info->best[mode][2].ref[LIST_0] : -1;
      for (j = currMB->block_y + 2; j < currMB->block_y + 4; j++)
      {
        for (i = currMB->block_x; i < currMB->block_x + 4;i++)
        {
          motion[j][i].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
          motion[j][i].ref_idx [LIST_0] = (char) curref;
        }
      }
    }      
    else if (mode == 3)
    {
      short curref = (b8x8info->best[mode][0].pdir == 0) ? b8x8info->best[mode][0].ref[LIST_0] : -1;
      for (j = currMB->block_y; j < currMB->block_y + 4; j++)
      {
        for (i = currMB->block_x; i < currMB->block_x + 2;i++)
        {
          motion[j][i].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
          motion[j][i].ref_idx [LIST_0] = (char) curref;
        }
      }
      curref = (b8x8info->best[mode][1].pdir == 0) ? b8x8info->best[mode][1].ref[LIST_0] : -1;
      for (j = currMB->block_y; j < currMB->block_y + 4; j++)
      {
        for (i = currMB->block_x + 2; i < currMB->block_x + 4;i++)
        {
          motion[j][i].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
          motion[j][i].ref_idx [LIST_0] = (char) curref;
        }
      }
    }      
    else
    {
      int  block_x, block_y;
      int curref;
      for (j = 0; j < 4; j++)
      {
        block_x = currMB->block_x;
        block_y = currMB->block_y + j;
        k = 2*(j >> 1);
        curref = b8x8info->best[mode][k++].ref[LIST_0];
        motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
        motion[block_y][block_x++].ref_idx[LIST_0] = (char) curref;
        motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
        motion[block_y][block_x++].ref_idx[LIST_0] = (char) curref;
        curref = b8x8info->best[mode][k].ref[LIST_0];
        motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
        motion[block_y][block_x++].ref_idx[LIST_0] = (char) curref;
        motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][curref];
        motion[block_y][block_x].ref_idx[LIST_0] = (char) curref;
      }
    }
  }
}


/*!
 *************************************************************************************
 * \brief
 *    Sets modes and reference frames for a macroblock
 *************************************************************************************
 */
void set_modes_and_refs_for_blocks_b_slice(Macroblock *currMB, short mode)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;

  int i,j,k; 
  int  block_x, block_y, block8x8, block4x4;
  int  cur_ref;
  int  clist;
  char curref, bestref;
  Block8x8Info *b8x8info = p_Vid->b8x8info;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  //--- macroblock type ---
  currMB->mb_type = mode;
  currMB->ar_mode = mode;

  for( i = 0; i < 4; i++) 
  {
    currMB->b8x8[i] = b8x8info->best[mode][i];
  }  

  //--- block 8x8 mode and prediction direction ---
  switch (mode)
  {
  case 0:
    currMB->b8x8[0].mode = 0;
    currMB->b8x8[1].mode = 0;
    currMB->b8x8[2].mode = 0;
    currMB->b8x8[3].mode = 0;

    currMB->b8x8[0].pdir = currSlice->direct_pdir[currMB->block_y    ][currMB->block_x    ];
    currMB->b8x8[1].pdir = currSlice->direct_pdir[currMB->block_y    ][currMB->block_x + 2];
    currMB->b8x8[2].pdir = currSlice->direct_pdir[currMB->block_y + 2][currMB->block_x    ];
    currMB->b8x8[3].pdir = currSlice->direct_pdir[currMB->block_y + 2][currMB->block_x + 2];
    break;
  case 1:
  case 2:
  case 3:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = (char) mode;
    }
    break;
  case P8x8:
    break;
  case I4MB:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = IBLOCK;
      currMB->b8x8[i].pdir = -1;
    }
    break;
  case I16MB:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = 0;
      currMB->b8x8[i].pdir = -1;
    }
    break;
  case I8MB:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = I8MB;
      currMB->b8x8[i].pdir = -1;
    }
    //switch to 8x8 transform
    currMB->luma_transform_size_8x8_flag = TRUE;
    break;
  case IPCM:
    for(i=0;i<4;i++)
    {
      currMB->b8x8[i].mode = IPCM;
      currMB->b8x8[i].pdir = -1;
    }
    currMB->luma_transform_size_8x8_flag = FALSE;
    break;
  default:
    printf ("Unsupported mode in set_modes_and_refs_for_blocks!\n");
    exit (1);
  }

#define IS_FW ((b8x8info->best[mode][k].pdir==0 || b8x8info->best[mode][k].pdir==2) && (mode!=P8x8 || b8x8info->best[mode][k].mode!=0))
#define IS_BW ((b8x8info->best[mode][k].pdir==1 || b8x8info->best[mode][k].pdir==2) && (mode!=P8x8 || b8x8info->best[mode][k].mode!=0))

  //--- reference frame arrays ---
  if (mode == 0) // Direct
  {
    for (clist = LIST_0; clist <= LIST_1; clist++)
    {
      for (j = currMB->block_y; j < currMB->block_y + 4; j++)
      {
        for (i = currMB->block_x; i < currMB->block_x + 4; i++)
          motion[j][i].ref_idx[clist] = currSlice->direct_ref_idx[j][i][clist];
      }
    }
  }
  else if (mode==I4MB || mode==I16MB || mode==I8MB || mode == IPCM)
  {
    for (clist = LIST_0; clist <= LIST_1; clist++)
    {
      for (j = currMB->block_y; j < currMB->block_y + 4; j++)
      {
        for (i = currMB->block_x; i < currMB->block_x + 4; i++)
          motion[j][i].ref_idx[clist] = -1;
      }
    }
  }
  else if (mode == 1 || mode == 2 || mode == 3) 
  {
    for (block8x8 = 0; block8x8 < 4; block8x8++)
    {
      for (clist = LIST_0; clist <= LIST_1; clist++)
      {
        bestref = b8x8info->best[mode][block8x8].ref[clist];

        if ( b8x8info->best[mode][block8x8].pdir == 2)
        {
          curref = (b8x8info->best[mode][block8x8].bipred) ? 0 : bestref;
        }
        else
        {
          curref = (clist == b8x8info->best[mode][block8x8].pdir) ? bestref : -1;
        }

        for (block4x4 = 0; block4x4 < 4; block4x4++)
        {
          block_x = currMB->block_x + 2 * (block8x8 & 0x01) + (block4x4 & 0x01);
          block_y = currMB->block_y + 2 * (block8x8 >> 1) + (block4x4 >> 1);
          motion[block_y][block_x].ref_idx[clist] = curref;
        }
      }
    }
  }
  else // mode == P8x8
  {
    for (j = 0; j < 4; j++)
    {
      block_y = currMB->block_y + j;
      for (i = 0; i < 4; i++)
      {
        block_x = currMB->block_x + i;
        k = 2*(j >> 1) + (i >> 1);

        if(b8x8info->best[mode][k].mode==0)
        {
          motion[block_y][block_x].ref_idx[LIST_0] = currSlice->direct_ref_idx[block_y][block_x][LIST_0];
          motion[block_y][block_x].ref_idx[LIST_1] = currSlice->direct_ref_idx[block_y][block_x][LIST_1];
        }
        else
        {
          motion[block_y][block_x].ref_idx[LIST_0] = (IS_FW ? b8x8info->best[mode][k].ref[LIST_0] : -1);
          motion[block_y][block_x].ref_idx[LIST_1] = (IS_BW ? b8x8info->best[mode][k].ref[LIST_1] : -1);
        }
      }        
    }        
  }

  for (clist = LIST_0; clist <= LIST_1; clist++)
  {
    StorablePicture **ref_pic = currSlice->listX[clist + currMB->list_offset];
    for (j = currMB->block_y; j < currMB->block_y + 4; j++)
    {
      for (i = currMB->block_x; i < currMB->block_x + 4;i++)
      {
        cur_ref = (int) motion[j][i].ref_idx[clist];
        motion[j][i].ref_pic [clist] = (cur_ref < 0 ? NULL : ref_pic[cur_ref]);
      }
    }
  }

#undef IS_FW
#undef IS_BW
}


/*!
 *************************************************************************************
 * \brief
 *    Sets Coefficients and reconstruction for an 8x8 block
 *************************************************************************************
 */
void set_coeff_and_recon_8x8_p_slice (Macroblock* currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  int block, k, j, i, uv;
  int cur_ref;

  //============= MIXED TRANSFORM SIZES FOR 8x8 PARTITION ==============
  //--------------------------------------------------------------------  
  if (currMB->luma_transform_size_8x8_flag && (currMB->valid_8x8))
  {    
    //============= set mode and ref. frames ==============
    memcpy(currMB->b8x8, p_RDO->tr8x8->part, 4 * sizeof(Info8x8));

    for (j = 0; j < 4; j++)
    {
      for (i = 0; i < 4; i++)
      {
        k = 2*(j >> 1)+(i >> 1);
        motion[currMB->block_y+j][currMB->block_x+i].ref_idx[LIST_0] = p_RDO->tr8x8->part[k].ref[LIST_0];
      }
    }

    for (j = currMB->block_y;j<currMB->block_y + BLOCK_MULTIPLE;j++)
    {
      for (i = currMB->block_x;i<currMB->block_x + BLOCK_MULTIPLE;i++)
      {
        cur_ref = (int) motion[j][i].ref_idx[LIST_0];
        motion[j][i].ref_pic[LIST_0] =(cur_ref >= 0 ? currSlice->listX[LIST_0 + currMB->list_offset][cur_ref] : NULL);
      }
    }    


    //====== set the mv's for 8x8 partition with transform size 8x8 ======
    //save the mv data for 4x4 transform
    StoreMV8x8(currSlice, 1);
    //set new mv data for 8x8 transform
    RestoreMV8x8(currSlice, 0);

    //============= get pre-calculated data ==============
    //restore coefficients from 8x8 transform

    for (block = 0; block<4; block++)
    {
      memcpy (currSlice->cofAC[block][0][0],p_RDO->cofAC8x8ts[block][0][0][0], 4 * 2 * 65 * sizeof(int));
    }

    if (p_Vid->P444_joined)
    {
      for (uv=0; uv<2; uv++)
      {
        for (block = 0; block<4; block++)
        {
          memcpy (currSlice->cofAC[4+block+uv*4][0][0],p_RDO->cofAC8x8ts[block][uv + 1][0][0], 4 * 2 * 65 * sizeof(int));
        }
      }
    }

    //restore reconstruction
    if (p_RDO->tr8x8->cnt_nonz_8x8 <= _LUMA_8x8_COEFF_COST_ &&
      ((currMB->qp_scaled[0])!=0 || p_Vid->active_sps->lossless_qpprime_flag==0) &&
      (currSlice->slice_type != SP_SLICE))// modif ES added last condition (we probably never go there so is the next modification useful ? check)
    {      
      currMB->cbp     = 0;
      currMB->cbp_blk = 0;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], p_RDO->tr8x8->mpr8x8, currMB->pix_x, 0);

      if(p_Inp->rdopt == 3)
      {
        p_Vid->p_decs->rec_type = 0;
      }
      
      if(currSlice->slice_type == SP_SLICE && !p_Vid->sp2_frame_indicator)
      {
        for (j = 0; j < MB_BLOCK_SIZE; j++)
        {
          memcpy(&p_Vid->lrec[currMB->pix_y+j][currMB->pix_x],p_RDO->tr8x8->lrec[j], MB_BLOCK_SIZE * sizeof(int));
        }
      }

      //memset( currSlice->cofAC[0][0][0], 0, 2080 * sizeof(int)); // 4 * 4 * 2 * 65

      if(p_Vid->P444_joined)
      {
        currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], p_RDO->tr8x8->mpr8x8CbCr[0], currMB->pix_x, 0);
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], p_RDO->tr8x8->mpr8x8CbCr[1], currMB->pix_x, 0);

        for (uv=0; uv<2; uv++)
        {
          for (block = 0; block<4; block++)
          {
            memset( currSlice->cofAC[4+block+uv*4][0][0], 0, 4 * 2 * 65 * sizeof(int));
          }
        }
      }
    }
    else
    {      
      currMB->cbp     = p_RDO->tr8x8->cbp8x8;
      currMB->cbp_blk = p_RDO->tr8x8->cbp_blk8x8;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], p_RDO->tr8x8->rec_mbY8x8, currMB->pix_x, 0);

      if (p_Inp->rdopt == 3)
      {
        p_Vid->p_decs->rec_type = 1;
      }

      if(currSlice->slice_type == SP_SLICE && !p_Vid->sp2_frame_indicator)
      {
        for (j = 0; j < MB_BLOCK_SIZE; j++)
        {
          memcpy (&p_Vid->lrec[currMB->pix_y+j][currMB->pix_x],p_RDO->tr8x8->lrec[j], MB_BLOCK_SIZE * sizeof(int));
        }
      }

      if (p_Vid->P444_joined) 
      {
        currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = p_RDO->tr8x8->cbp8x8;
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], p_RDO->tr8x8->rec_mb8x8_cr[0], currMB->pix_x, 0);
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], p_RDO->tr8x8->rec_mb8x8_cr[1], currMB->pix_x, 0);
      }
    }
    if (p_Inp->rdopt == 3)
    {
      errdo_get_best_P8x8(currMB, 1);
    }
  }
  else
  {
    //============= get pre-calculated data ==============
    //---------------------------------------------------
    //--- restore coefficients ---
    for (block = 0; block < 4; block ++)
    {
      memcpy (currSlice->cofAC[block][0][0],p_RDO->coefAC8x8[block][0][0][0], 4 * 2 * 65 * sizeof(int));     
    }

    if (currSlice->P444_joined) 
    {
      for (block = 0; block<4; block++)
      {
        memcpy (currSlice->cofAC[block+4][0][0],p_RDO->coefAC8x8[block][1][0][0], 4 * 2 * 65 * sizeof(int));     
        memcpy (currSlice->cofAC[block+8][0][0],p_RDO->coefAC8x8[block][2][0][0], 4 * 2 * 65 * sizeof(int));   
      }
    }

    if (((p_Inp->disthres && p_RDO->tr4x4->cnt_nonz_8x8 <= 0) || (p_RDO->tr4x4->cnt_nonz_8x8 <= 5)) && currSlice->slice_type != SP_SLICE &&
      ((currMB->qp_scaled[0])!=0 || p_Vid->active_sps->lossless_qpprime_flag==0))
    {
      currMB->cbp     = 0;
      currMB->cbp_blk = 0;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], p_RDO->tr4x4->mpr8x8, currMB->pix_x, 0);

      if(p_Inp->rdopt == 3)
      {
        p_Vid->p_decs->rec_type = 0;
      }

      if(currSlice->slice_type == SP_SLICE && !p_Vid->sp2_frame_indicator)
      {
        for (j = 0; j < MB_BLOCK_SIZE; j++)
        {
          memcpy (&p_Vid->lrec[currMB->pix_y+j][currMB->pix_x],p_RDO->tr4x4->lrec[j], MB_BLOCK_SIZE * sizeof(int)); // restore coeff. SP frame
        }
      }

      //memset( currSlice->cofAC[0][0][0], 0, 2080 * sizeof(int)); // 4 * 4 * 2 * 65

      if (currSlice->P444_joined)
      {
        currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;

        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], p_RDO->tr4x4->mpr8x8CbCr[0], currMB->pix_x, 0);
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], p_RDO->tr4x4->mpr8x8CbCr[1], currMB->pix_x, 0);

        for (uv=0; uv<2; uv++)
        {
          for (block = 0; block<4; block++)
          {
            memset( currSlice->cofAC[4+block+uv*4][0][0], 0, 4 * 2 * 65 * sizeof(int));
          }
        }
      }
    }
    else
    {
      currMB->cbp     = p_RDO->tr4x4->cbp8x8;
      currMB->cbp_blk = p_RDO->tr4x4->cbp_blk8x8;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], p_RDO->tr4x4->rec_mbY8x8, currMB->pix_x, 0);

      if(p_Inp->rdopt == 3)
      {
        p_Vid->p_decs->rec_type = 1;
      }

      if(currSlice->slice_type == SP_SLICE && !p_Vid->sp2_frame_indicator)
      {
        for (j = 0; j < MB_BLOCK_SIZE; j++)
        {
          memcpy (&p_Vid->lrec[currMB->pix_y+j][currMB->pix_x],p_RDO->tr4x4->lrec[j], MB_BLOCK_SIZE * sizeof(int));
        }
      }
      if (currSlice->P444_joined)
      {
        currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = p_RDO->tr4x4->cbp8x8;
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], p_RDO->tr4x4->rec_mb8x8_cr[0], currMB->pix_x, 0);
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], p_RDO->tr4x4->rec_mb8x8_cr[1], currMB->pix_x, 0);
      }
    }
    if (p_Inp->rdopt == 3)
    {
      errdo_get_best_P8x8(currMB, 0);
    }
  }
}


/*!
 *************************************************************************************
 * \brief
 *    Sets Coefficients and reconstruction for an 8x8 block
 *************************************************************************************
 */
void set_coeff_and_recon_8x8_b_slice (Macroblock* currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  int block, k, j, i, uv;
  int cur_ref;

  //============= MIXED TRANSFORM SIZES FOR 8x8 PARTITION ==============
  //--------------------------------------------------------------------  
  if (currMB->luma_transform_size_8x8_flag && (currMB->valid_8x8))
  {
    //============= set mode and ref. frames ==============
    memcpy(currMB->b8x8, p_RDO->tr8x8->part, 4 * sizeof(Info8x8));

    for (j = 0;j<4;j++)
    {
      for (i = 0;i<4;i++)
      {
        k = 2*(j >> 1)+(i >> 1);
        motion[currMB->block_y+j][currMB->block_x+i].ref_idx[LIST_0] = ((currMB->b8x8[k].pdir & 0x01) == 0) ? p_RDO->tr8x8->part[k].ref[LIST_0] : - 1;
        motion[currMB->block_y+j][currMB->block_x+i].ref_idx[LIST_1] =  (currMB->b8x8[k].pdir > 0)          ? p_RDO->tr8x8->part[k].ref[LIST_1] : - 1;
      }
    }    

    for (j = currMB->block_y;j<currMB->block_y + BLOCK_MULTIPLE;j++)
    {
      for (i = currMB->block_x;i<currMB->block_x + BLOCK_MULTIPLE;i++)
      {
        cur_ref = (int) motion[j][i].ref_idx[LIST_0];
        motion[j][i].ref_pic[LIST_0] =(cur_ref >= 0 ? currSlice->listX[LIST_0 + currMB->list_offset][cur_ref] : NULL);
      }
    }    

    for (j = currMB->block_y;j<currMB->block_y + BLOCK_MULTIPLE;j++)
    {
      for (i = currMB->block_x;i<currMB->block_x + BLOCK_MULTIPLE;i++)
      {
        cur_ref = (int) motion[j][i].ref_idx[LIST_1];
        motion[j][i].ref_pic[LIST_1] =(cur_ref >= 0 ? currSlice->listX[LIST_1 + currMB->list_offset][cur_ref] : NULL);
      }
    }    

    //====== set the mv's for 8x8 partition with transform size 8x8 ======
    //save the mv data for 4x4 transform
    StoreMV8x8(currSlice, 1);
    //set new mv data for 8x8 transform
    RestoreMV8x8(currSlice, 0);

    //============= get pre-calculated data ==============
    //restore coefficients from 8x8 transform

    for (block = 0; block<4; block++)
    {
      memcpy (currSlice->cofAC[block][0][0],p_RDO->cofAC8x8ts[block][0][0][0], 4 * 2 * 65 * sizeof(int));
    }

    if (p_Vid->P444_joined)
    {
      for (uv=0; uv<2; uv++)
      {
        for (block = 0; block<4; block++)
        {
          memcpy (currSlice->cofAC[4+block+uv*4][0][0],p_RDO->cofAC8x8ts[block][uv + 1][0][0], 4 * 2 * 65 * sizeof(int));
        }
      }
    }

    //restore reconstruction
    if (p_RDO->tr8x8->cnt_nonz_8x8 <= _LUMA_8x8_COEFF_COST_ &&
      ((currMB->qp_scaled[0])!=0 || p_Vid->active_sps->lossless_qpprime_flag==0))// modif ES added last condition (we probably never go there so is the next modification useful ? check)
    {      
      currMB->cbp     = 0;
      currMB->cbp_blk = 0;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], p_RDO->tr8x8->mpr8x8, currMB->pix_x, 0);

      if(p_Inp->rdopt == 3)
      {
        p_Vid->p_decs->rec_type = 0;
      }


      //memset( currSlice->cofAC[0][0][0], 0, 2080 * sizeof(int)); // 4 * 4 * 2 * 65

      if(p_Vid->P444_joined)
      {
        currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], p_RDO->tr8x8->mpr8x8CbCr[0], currMB->pix_x, 0);
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], p_RDO->tr8x8->mpr8x8CbCr[1], currMB->pix_x, 0);

        for (uv=0; uv<2; uv++)
        {
          for (block = 0; block<4; block++)
          {
            memset( currSlice->cofAC[4+block+uv*4][0][0], 0, 4 * 2 * 65 * sizeof(int));
          }
        }
      }
    }
    else
    {      
      currMB->cbp     = p_RDO->tr8x8->cbp8x8;
      currMB->cbp_blk = p_RDO->tr8x8->cbp_blk8x8;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], p_RDO->tr8x8->rec_mbY8x8, currMB->pix_x, 0);

      if(p_Inp->rdopt == 3)
      {
        p_Vid->p_decs->rec_type = 1;
      }


      if (p_Vid->P444_joined) 
      {
        currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = p_RDO->tr8x8->cbp8x8;
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], p_RDO->tr8x8->rec_mb8x8_cr[0], currMB->pix_x, 0);
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], p_RDO->tr8x8->rec_mb8x8_cr[1], currMB->pix_x, 0);
      }
    }
    if (p_Inp->rdopt == 3)
    {
      errdo_get_best_P8x8(currMB, 1);
    }
  }
  else
  {
    if (currMB->valid_8x8)
    {
      StoreMV8x8(currSlice, 1);
    }

    //============= get pre-calculated data ==============
    //---------------------------------------------------
    //--- restore coefficients ---
    for (block = 0; block < 4; block ++)
    {
      memcpy (currSlice->cofAC[block][0][0],p_RDO->coefAC8x8[block][0][0][0], 4 * 2 * 65 * sizeof(int));     
    }

    if (currSlice->P444_joined) 
    {
      for (block = 0; block<4; block++)
      {
        memcpy (currSlice->cofAC[block+4][0][0],p_RDO->coefAC8x8[block][1][0][0], 4 * 2 * 65 * sizeof(int));     
        memcpy (currSlice->cofAC[block+8][0][0],p_RDO->coefAC8x8[block][2][0][0], 4 * 2 * 65 * sizeof(int));   
      }
    }

    if (((p_Inp->disthres && p_RDO->tr4x4->cnt_nonz_8x8 <= 0) || (p_RDO->tr4x4->cnt_nonz_8x8 <= 5)) &&
      ((currMB->qp_scaled[0])!=0 || p_Vid->active_sps->lossless_qpprime_flag==0))
    {
      currMB->cbp     = 0;
      currMB->cbp_blk = 0;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], p_RDO->tr4x4->mpr8x8, currMB->pix_x, 0);

      if(p_Inp->rdopt == 3)
      {
        p_Vid->p_decs->rec_type = 0;
      }

      if (currSlice->P444_joined)
      {
        currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;

        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], p_RDO->tr4x4->mpr8x8CbCr[0], currMB->pix_x, 0);
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], p_RDO->tr4x4->mpr8x8CbCr[1], currMB->pix_x, 0);

        for (uv=0; uv<2; uv++)
        {
          for (block = 0; block<4; block++)
          {
            memset( currSlice->cofAC[4+block+uv*4][0][0], 0, 4 * 2 * 65 * sizeof(int));
          }
        }
      }
    }
    else
    {
      currMB->cbp     = p_RDO->tr4x4->cbp8x8;
      currMB->cbp_blk = p_RDO->tr4x4->cbp_blk8x8;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], p_RDO->tr4x4->rec_mbY8x8, currMB->pix_x, 0);

      if(p_Inp->rdopt == 3)
      {
        p_Vid->p_decs->rec_type = 1;
      }

      if (currSlice->P444_joined)
      {
        currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = p_RDO->tr4x4->cbp8x8;
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[0][currMB->pix_y], p_RDO->tr4x4->rec_mb8x8_cr[0], currMB->pix_x, 0);
        copy_image_data_16x16(&p_Vid->enc_picture->imgUV[1][currMB->pix_y], p_RDO->tr4x4->rec_mb8x8_cr[1], currMB->pix_x, 0);
      }
    }
    if (p_Inp->rdopt == 3)
    {
      errdo_get_best_P8x8(currMB, 0);
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Restore Non zero coefficients
 *************************************************************************************
 */
void restore_nz_coeff(Macroblock *currMB)
{
#ifdef BEST_NZ_COEFF
  VideoParameters *p_Vid = currMB->p_Vid;
  int i, j;
  int **nz_coeff = p_Vid->nz_coeff[currMB->mbAddrX];
  for (j=0;j<4;j++)
    for (i=0; i<(4+p_Vid->num_blk8x8_uv); i++)
      nz_coeff[j][i] = p_Vid->gaaiMBAFF_NZCoeff[j][i]; 
#endif
}


void prepare_ipcm_mode(Macroblock *currMB)
{
  int i, j;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  // LUMA
  copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], &p_Vid->pCurImg[currMB->opix_y], currMB->pix_x, currMB->pix_x);

  // CHROMA
  if ((p_Vid->yuv_format != YUV400) && (p_Inp->separate_colour_plane_flag == 0))
  {      
    copy_image_data(&p_Vid->enc_picture->imgUV[0][currMB->pix_c_y], &p_Vid->pImgOrg[1][currMB->opix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    copy_image_data(&p_Vid->enc_picture->imgUV[1][currMB->pix_c_y], &p_Vid->pImgOrg[2][currMB->opix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
  }

  for (j=0;j<4;j++)
    for (i=0; i<(4+p_Vid->num_blk8x8_uv); i++)
      p_Vid->nz_coeff[currMB->mbAddrX][j][i] = 16;
}

/*!
 *************************************************************************************
 * \brief
 *    R-D Cost for a macroblock
 *************************************************************************************
 */
int RDCost_for_macroblocks (Macroblock  *currMB,   // <-- Current Macroblock to code
                            int   lambda,       // <-- lagrange multiplier
                            short    mode)         // <-- mode (0-COPY/DIRECT, 1-16x16, 2-16x8, 3-8x16, 4-8x8(+), 5-Intra4x4, 6-Intra16x16)                            
{
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure *p_RDO = currSlice->p_RDO;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  seq_parameter_set_rbsp_t *active_sps = p_Vid->active_sps;

  int         rate = 0, coeff_rate = 0;
  distblk     distortion = 0;
  Macroblock  *prevMB = currMB->PrevMB; 
  int         tmp_cc;

  int         use_of_cc = (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE && currSlice->symbol_mode == CAVLC);
  int         cc_rate, dummy;

  distblk      rdcost;
  distblk      dummy_d;
  imgpel     **mb_pred = currSlice->mb_pred[0];
  imgpel     ***curr_mpr_16x16 = currSlice->mpr_16x16[0];

  // Test MV limits for Skip Mode. This could be necessary for MBAFF case Frame MBs.
  if ((currSlice->mb_aff_frame_flag) && (!currMB->mb_field) && (currSlice->slice_type == P_SLICE) && (mode==0) )
  {
    if (out_of_bounds_mvs(p_Vid, &currSlice->all_mv[LIST_0][0][0][0][0]))
      return 0;
  }

  if (mode == P8x8)
  {
     if((currMB->luma_transform_size_8x8_flag && !currMB->valid_8x8) || (!currMB->luma_transform_size_8x8_flag && !currMB->valid_4x4))
      return 0;
  }

  //=====
  //=====  Set Reference Pictures and Block modes
  //=====
  currSlice->set_modes_and_refs_for_blocks (currMB, mode);
  //=====
  //=====  Set Motion Vectors
  //=====
  currSlice->set_motion_vectors_mb (currMB);

  //=====
  //=====  Get coefficients, reconstruction values, CBP etc
  //=====
  if (mode < P8x8)
  {
    currSlice->luma_residual_coding(currMB);
  }
  else if (mode == P8x8)
  {
    currSlice->set_coeff_and_recon_8x8 (currMB);
  }
  else if (mode==I4MB)
  {
    currMB->cbp = mode_decision_for_I4x4_MB (currMB, lambda, &dummy_d);
  }
  else if (mode==I16MB)
  {
    currMB->cbp = currSlice->mode_decision_for_I16x16_MB (currMB, lambda);
  }
  else if(mode==I8MB)
  {
    currMB->cbp = mode_decision_for_I8x8_MB(currMB, lambda, &dummy_d);
  }
  else if(mode==IPCM)
  {
    prepare_ipcm_mode(currMB);
  }

  //if (p_Inp->rdopt == 3)
  //{
  //  // We need the reconstructed prediction residue for the simulated decoders.
  //  compute_residue_block (currMB, &p_Vid->enc_picture->p_curr_img[currMB->pix_y], p_Vid->p_decs->res_img[0], mode == I16MB ? currSlice->mpr_16x16[0][ (short) currMB->i16mode] : currSlice->mb_pred[0], 0, 16);
  //}

  //Rate control
  if (p_Inp->RCEnable)
  {
    if (mode == I16MB)
      memcpy(p_RDO->pred[0], curr_mpr_16x16[ (short) currMB->i16mode][0], MB_PIXELS * sizeof(imgpel));
    else
      memcpy(p_RDO->pred[0], mb_pred[0], MB_PIXELS * sizeof(imgpel));
  }

  currMB->i16offset = 0;
  dummy = 0;

  if (((p_Vid->yuv_format != YUV400) && (active_sps->chroma_format_idc != YUV444)) && (mode != IPCM))
    currSlice->chroma_residual_coding (currMB);

  if (mode==I16MB)
    currMB->i16offset = I16Offset  (currMB->cbp, currMB->i16mode);

  //=====
  //=====   GET DISTORTION
  //=====
  // LUMA
  if (p_Inp->rdopt == 3)
  {
    distortion = p_Vid->estimate_distortion(currMB, 0, 16, mode, 0, currMB->min_rdcost);
    //if (errdo_distortion (currMB, mode, &distortion) == 0)
    //  return 0;
  }
  else
  {

    distortion = currSlice->getDistortion(currMB);
  }
  if (distortion > currMB->min_rdcost)
    return 0;
  //printf("passed distortion %.2f %.2f\n", (double)distortion, currMB->min_rdcost);

  if (currMB->qp_scaled[0] == 0 && p_Vid->active_sps->lossless_qpprime_flag == 1 && distortion != 0)
    return 0;

  //=====   S T O R E   C O D I N G   S T A T E   =====
  //---------------------------------------------------
  currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_cm);

  //=====
  //=====   GET RATE
  //=====
  //----- macroblock header -----
  if (use_of_cc)
  {
    if (currMB->mb_type!=0 || (currSlice->slice_type == B_SLICE && currMB->cbp!=0))
    {
      // cod counter and macroblock mode are written ==> do not consider code counter
      tmp_cc = p_Vid->cod_counter;

      rate = currSlice->write_MB_layer (currMB, 1, &coeff_rate);

      ue_linfo (tmp_cc, dummy, &cc_rate, &dummy);
      rate  -= cc_rate;
      p_Vid->cod_counter = tmp_cc;
    }
    else
    {
      // cod counter is just increased  ==> get additional rate
      ue_linfo (p_Vid->cod_counter + 1, dummy, &rate,    &dummy);
      ue_linfo (p_Vid->cod_counter    , dummy, &cc_rate, &dummy);
      rate -= cc_rate;
    }
  }
  else
  {
    rate = currSlice->write_MB_layer (currMB, 1, &coeff_rate);
  }

  //=====   R E S T O R E   C O D I N G   S T A T E   =====
  //-------------------------------------------------------
  currSlice->reset_coding_state (currMB, currSlice->p_RDO->cs_cm);

  if (p_Inp->ForceTrueRateRDO == 1)
    rdcost = distortion + weight_cost(lambda, rate);
  else if (p_Inp->ForceTrueRateRDO == 2)
    rdcost = distortion + weight_cost(lambda, (rate +  is_intra(currMB)));
  else
    rdcost = distortion + (rate>0? (weight_cost(lambda, rate)): (weight_cost(lambda, 1)/2));

#if JCOST_OVERFLOWCHECK //overflow checking;
  if(rdcost < distortion)
  {
    printf("lambda: %d, rate: %d, error: %s: %d\n", lambda, rate, __FILE__, __LINE__);
    exit(-1);
  }
#endif //end;

  // 
  if ((currSlice->slice_type != I_SLICE) && (p_Inp->BiasSkipRDO == 1) && (mode == 1) && (currMB->best_mode == 0) && (currMB->min_dcost > 4 * distortion) && (currMB->min_dcost > ((64 * (256 + 2 * p_Vid->mb_cr_size_y * p_Vid->mb_cr_size_x)) << LAMBDA_ACCURACY_BITS)))
  {
    currMB->min_rdcost = rdcost + 1;
  }

  if ((rdcost >= currMB->min_rdcost) ||
    ((currMB->qp_scaled[0]) == 0 && p_Vid->active_sps->lossless_qpprime_flag == 1 && distortion != 0))
  {
#if FASTMODE
    // Reordering RDCost comparison order of mode 0 and mode 1 in P_SLICE
    // if RDcost of mode 0 and mode 1 is same, we choose best_mode is 0
    // This might not always be good since mode 0 is more biased towards rate than quality.
    if((currSlice->slice_type != P_SLICE || mode != 0 || rdcost != currMB->min_rdcost) || is_FREXT_profile(p_Inp->ProfileIDC))
#endif
      return 0;
  }


  if ((currSlice->mb_aff_frame_flag) && (mode ? 0: ((currSlice->slice_type == B_SLICE) ? !currMB->cbp:1)))  // AFF and current is skip
  {
    if (currMB->mbAddrX & 0x01) //bottom
    {
      if (prevMB->mb_type ? 0:((currSlice->slice_type == B_SLICE) ? !prevMB->cbp:1)) //top is skip
      {
        if (!(field_flag_inference(currMB) == currMB->mb_field)) //skip only allowed when correct inference
          return 0;
      }
    }
  }

  //=====   U P D A T E   M I N I M U M   C O S T   =====
  //-----------------------------------------------------
  currMB->min_rdcost = rdcost;
  currMB->min_dcost = distortion;
  currMB->min_rate = weight_cost(lambda, coeff_rate);
  currMB->min_bits = rate;

#ifdef BEST_NZ_COEFF  
  for (j=0;j<4;j++)
    memcpy(&p_Vid->gaaiMBAFF_NZCoeff[j][0], &p_Vid->nz_coeff[currMB->mbAddrX][j][0], (4 + p_Vid->num_blk8x8_uv) * sizeof(int));
#endif

  return 1;
}

/*!
 *************************************************************************************
 * \brief
 *    Initialize best mode information
 *************************************************************************************
 */
void init_md_best(BestMode  *best)
{
  best->cbp       = 0;
  best->cost      = DISTBLK_MAX; //(1i64<<(20+LAMBDA_ACCURACY_BITS));
  best->rdcost    = 1e20;
  best->dcost     = 1e20;
  best->rate      = 1e20;
  best->c_imode   = 0;
  best->i16offset = 0;
  best->mode      = 10;
}

/*!
 *************************************************************************************
 * \brief
 *    Store macroblock parameters
 *************************************************************************************
 */
void store_macroblock_parameters (Macroblock *currMB, int mode)
{
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  int  i, j, ****i4p, ***i3p;

  //--- store best mode ---
  currMB->best_mode      = (short) mode;
  currMB->best_c_imode   = currMB->c_ipred_mode;
  currMB->best_i16offset = currMB->i16offset;
  currMB->best_i16mode   = currMB->i16mode;
  currMB->best_cbp       = currMB->cbp;

  memcpy(currSlice->p_RDO->best8x8, currMB->b8x8, BLOCK_MULTIPLE * sizeof(Info8x8));

  memcpy(currSlice->b4_intra_pred_modes,   currMB->intra_pred_modes, MB_BLOCK_PARTITIONS * sizeof(char));
  memcpy(currSlice->b8_intra_pred_modes8x8,currMB->intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));

  for (j = 0 ; j < BLOCK_MULTIPLE; j++)
  {
    memcpy(&currSlice->b4_ipredmode[j * BLOCK_MULTIPLE],&p_Vid->ipredmode   [currMB->block_y + j][currMB->block_x],BLOCK_MULTIPLE * sizeof(char));
    memcpy(&currSlice->b8_ipredmode8x8[j][0],           &p_Vid->ipredmode8x8[currMB->block_y + j][currMB->block_x],BLOCK_MULTIPLE * sizeof(char));
  }
  //--- reconstructed blocks ----
  copy_image_data_16x16(p_RDO->rec_mb[0], &p_Vid->enc_picture->imgY[currMB->pix_y], 0, currMB->pix_x);

  if((currSlice->slice_type == SP_SLICE) && p_Vid->sp2_frame_indicator==0)
  {
    for (j = 0; j < MB_BLOCK_SIZE; j++)
    {
      memcpy(p_RDO->lrec_rec[j], &p_Vid->lrec[currMB->pix_y+j][currMB->pix_x], MB_BLOCK_SIZE * sizeof(int));//store coefficients SP frame
    }
  }

  //modes 0, and 1 of a B frame 
  if (p_Vid->AdaptiveRounding && (currSlice->slice_type == B_SLICE) && mode <= 1)
  {
    if (currMB->luma_transform_size_8x8_flag)
      store_adaptive_rounding_16x16( p_Vid, p_Vid->ARCofAdj8x8, mode);
    else
      store_adaptive_rounding_16x16( p_Vid, p_Vid->ARCofAdj4x4, mode);
  }

  if (p_Vid->yuv_format != YUV400)
  {
    int k;
    for (k = 0; k < 2; k++)
    {
      copy_image_data(p_RDO->rec_mb[k + 1], &p_Vid->enc_picture->imgUV[k][currMB->pix_c_y], 0, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    }

    if((currSlice->slice_type == SP_SLICE) && p_Vid->sp2_frame_indicator==0)
    {
      //store uv coefficients SP frame
      for (k = 0; k < 2; k++)
      {
        for (j = 0; j<p_Vid->mb_cr_size_y; j++)
        {
          memcpy(p_RDO->lrec_rec_uv[k][j],&p_Vid->lrec_uv[k][currMB->pix_c_y + j][currMB->pix_c_x], p_Vid->mb_cr_size_x * sizeof(int));
        }
      }
    }
  }

  //--- store results of decoders ---
  if (p_Inp->rdopt == 3)
  {    
    errdo_store_best_MB(currMB);
  }

  //--- coeff, cbp, kac ---
  if (mode || currSlice->slice_type == B_SLICE)
  {
    i4p = p_RDO->cofAC; 
    p_RDO->cofAC = currSlice->cofAC; 
    currSlice->cofAC = i4p;
    i3p = p_RDO->cofDC; 
    p_RDO->cofDC = currSlice->cofDC; 
    currSlice->cofDC = i3p;
    p_RDO->cbp     = currMB->cbp;
    currSlice->curr_cbp[0] = currSlice->cmp_cbp[1];  
    currSlice->curr_cbp[1] = currSlice->cmp_cbp[2]; 

    currSlice->cur_cbp_blk[0] = currMB->cbp_blk;
  }
  else
  {
    currSlice->cur_cbp_blk[0] = p_RDO->cbp = 0;
    currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0; 
  }

  //--- store transform size ---
  currMB->temp_transform_size_8x8_flag = currMB->luma_transform_size_8x8_flag;

  if (currSlice->slice_type != B_SLICE)
  {
    for (j = 0; j < 4; j++)
    {
      for (i = 0; i < 4; i++)
      {
        p_RDO->l0_refframe[j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
      }
    }
  }
  else
  {
    for (j = 0; j < 4; j++)
    {
      for (i = 0; i < 4; i++)
      {
        p_RDO->l0_refframe[j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
        p_RDO->l1_refframe[j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1];
      }
    }
  }
}

/*!
*************************************************************************************
* \brief
*    Set stored macroblock parameters
*************************************************************************************
*/
static void set_stored_macroblock_parameters_mpass (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  imgpel     **imgY  = p_Vid->enc_picture->imgY;
  imgpel    ***imgUV = p_Vid->enc_picture->imgUV;

  int         mode   = currMB->best_mode;
  int         i, j, k, ****i4p, ***i3p;
  int         block_x, block_y;
  char    **ipredmodes = p_Vid->ipredmode;

  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  //===== reconstruction values =====

  // Luma
  copy_image_data_16x16(&imgY[currMB->pix_y], p_RDO->rec_mb[0], currMB->pix_x, 0);

  // Copy coding info into rddata structure for MBAFF or RDOQ
  copy_image_data_16x16(currSlice->rddata->rec_mb[0], p_RDO->rec_mb[0], 0, 0);

  if((currSlice->slice_type == SP_SLICE) && p_Vid->sp2_frame_indicator==0)
  {
    for (j = 0; j < MB_BLOCK_SIZE; j++)
      memcpy(&p_Vid->lrec[currMB->pix_y+j][currMB->pix_x], p_RDO->lrec_rec[j], MB_BLOCK_SIZE * sizeof(int)); //restore coeff SP frame
  }

  if (p_Vid->AdaptiveRounding)
  {
    update_offset_params(currMB, mode, currMB->temp_transform_size_8x8_flag);
  }

  if (p_Vid->yuv_format != YUV400)
  {
    for (k = 0; k < 2; k++)
    {
      copy_image_data(&imgUV[k][currMB->pix_c_y], p_RDO->rec_mb[k + 1], currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    }

    // Copy coding info into rddata structure for MBAFF or RDOQ
    copy_image_data(currSlice->rddata->rec_mb[1], p_RDO->rec_mb[1], 0, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    copy_image_data(currSlice->rddata->rec_mb[2], p_RDO->rec_mb[2], 0, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);

    if((currSlice->slice_type == SP_SLICE) && !p_Vid->sp2_frame_indicator)
    {
      for (k = 0; k < 2; k++)
      {
        for (j = 0; j<p_Vid->mb_cr_size_y; j++)
        {
          memcpy(&p_Vid->lrec_uv[k][currMB->pix_c_y + j][currMB->pix_c_x], p_RDO->lrec_rec_uv[k][j], p_Vid->mb_cr_size_x * sizeof(int));
        }
      }
    }
  }

  //===== coefficients and cbp =====
  i4p = p_RDO->cofAC; 
  p_RDO->cofAC = currSlice->cofAC; 
  currSlice->cofAC = i4p;

  i3p = p_RDO->cofDC; 
  p_RDO->cofDC = currSlice->cofDC; 
  currSlice->cofDC = i3p;
  currMB->cbp      = p_RDO->cbp;
  currMB->cbp_blk = currSlice->cur_cbp_blk[0];
  currMB->cbp |= currSlice->curr_cbp[0];
  currMB->cbp |= currSlice->curr_cbp[1];
  currSlice->cmp_cbp[1] = currMB->cbp; 
  currSlice->cmp_cbp[2] = currMB->cbp;

  //==== macroblock type ====
  currMB->mb_type = (short) mode;

  memcpy(currMB->b8x8, currSlice->p_RDO->best8x8, BLOCK_MULTIPLE * sizeof(Info8x8));

  // Copy coding info into rddata structure for MBAFF or RDOQ
  currSlice->rddata->mode = mode;
  currSlice->rddata->i16offset = currMB->i16offset;
  currSlice->rddata->i16mode   = currMB->i16mode;
  currSlice->rddata->cbp = p_RDO->cbp;
  currSlice->rddata->cbp_blk = currSlice->cur_cbp_blk[0];
  currSlice->rddata->mb_type  = (short) mode;

  currSlice->rddata->prev_qp  = currMB->prev_qp;
  currSlice->rddata->prev_dqp = currMB->prev_dqp;
  currSlice->rddata->qp       = currMB->qp;
  currSlice->rddata->prev_cbp = currMB->prev_cbp;

  memcpy(currSlice->rddata->cofAC[0][0][0], currSlice->cofAC[0][0][0], (4+p_Vid->num_blk8x8_uv) * 4 * 2 * 65 * sizeof(int));
  memcpy(currSlice->rddata->cofDC[0][0], currSlice->cofDC[0][0], 3 * 2 * 18 * sizeof(int));

  memcpy(currSlice->rddata->b8x8, currSlice->p_RDO->best8x8, BLOCK_MULTIPLE * sizeof(Info8x8));

  //if P8x8 mode and transform size 4x4 choosen, restore motion vector data for this transform size
  if (mode == P8x8 && !currMB->temp_transform_size_8x8_flag && p_Inp->Transform8x8Mode && (currMB->valid_8x8 == TRUE))
  {
    RestoreMV8x8(currSlice, 1);
  }

  //==== transform size flag ====
  if (p_Vid->P444_joined)
  {
    if (((currMB->cbp == 0) && currSlice->cmp_cbp[1] == 0 && currSlice->cmp_cbp[2] == 0) && (currMB->mb_type != I4MB && currMB->mb_type != I8MB))
      currMB->luma_transform_size_8x8_flag = FALSE;
    else
      currMB->luma_transform_size_8x8_flag = currMB->temp_transform_size_8x8_flag;
  }
  else
  {

    if (((currMB->cbp & 15) == 0) && (currMB->mb_type != I4MB && currMB->mb_type != I8MB))
      currMB->luma_transform_size_8x8_flag = FALSE;
    else
      currMB->luma_transform_size_8x8_flag = currMB->temp_transform_size_8x8_flag;
  }

  currSlice->rddata->luma_transform_size_8x8_flag  = currMB->luma_transform_size_8x8_flag;

  if (p_Inp->rdopt == 3)  
  {
    errdo_get_best_MB(currMB);
  }

  //==== reference frames =====
  for (j = 0; j < 4; j++)
  {
    block_y = currMB->block_y + j;
    for (i = 0; i < 4; i++)
    {
      block_x = currMB->block_x + i;
      k = 2*(j >> 1)+(i >> 1);

      // backward prediction or intra
      if ((currMB->b8x8[k].pdir == 1) || is_intra(currMB))
      {
        motion[block_y][block_x].ref_idx [LIST_0] = -1;
        motion[block_y][block_x].mv      [LIST_0] = zero_mv;
        motion[block_y][block_x].ref_pic [LIST_0] = NULL;        

        // MBAFF or RDOQ
        currSlice->rddata->refar[LIST_0][j][i] = -1;
      }
      else
      {
        if (currMB->b8x8[k].bipred && (currMB->b8x8[k].pdir == 2) && is_bipred_enabled(p_Vid, currMB->mb_type))
        {
          motion[block_y][block_x].ref_idx [LIST_0] = 0;
          motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][0];
          motion[block_y][block_x].mv      [LIST_0] = currSlice->bipred_mv[currMB->b8x8[k].bipred - 1][LIST_0][0][(short) currMB->b8x8[k].mode][j][i]; 

          // MBAFF or RDOQ
          currSlice->rddata->refar[LIST_0][j][i] = 0;
        }
        else
        {
          char cur_ref = p_RDO->l0_refframe[j][i];
          motion[block_y][block_x].ref_idx [LIST_0] = cur_ref;
          if(cur_ref >=0)
          {
          motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][(short)cur_ref];
          motion[block_y][block_x].mv      [LIST_0] = currSlice->all_mv[LIST_0][(short)cur_ref][(short) currMB->b8x8[k].mode][j][i];
          }
          // MBAFF or RDOQ
          currSlice->rddata->refar[LIST_0][j][i] = cur_ref;
        }
      }

      // forward prediction or intra
      if ((currMB->b8x8[k].pdir == 0) || is_intra(currMB))
      {
        motion[block_y][block_x].ref_idx [LIST_1] = -1;
        motion[block_y][block_x].ref_pic [LIST_1] = NULL;
        motion[block_y][block_x].mv      [LIST_1] = zero_mv;

        // MBAFF or RDOQ
        currSlice->rddata->refar[LIST_1][j][i] = -1;
      }
    }
  }

  if (currSlice->slice_type == B_SLICE)
  {
    for (j=0; j<4; j++)
    {
      block_y = currMB->block_y + j;
      for (i=0; i<4; i++)
      {
        block_x = currMB->block_x + i;
        k = 2*(j >> 1)+(i >> 1);

        // forward
        if (is_intra(currMB)||(currMB->b8x8[k].pdir == 0))
        {
          motion[block_y][block_x].ref_idx [LIST_1] = -1;
          motion[block_y][block_x].ref_pic [LIST_1] = NULL;
          motion[block_y][block_x].mv      [LIST_1] = zero_mv;

          // MBAFF or RDOQ
          currSlice->rddata->refar[LIST_1][j][i] = -1;
        }
        else
        {
          if (currMB->b8x8[k].bipred && (currMB->b8x8[k].pdir == 2) && is_bipred_enabled(p_Vid, currMB->mb_type))
          {
            motion[block_y][block_x].ref_idx [LIST_1] = 0;
            motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1 + currMB->list_offset][0];
            motion[block_y][block_x].mv      [LIST_1] = currSlice->bipred_mv[currMB->b8x8[k].bipred - 1][LIST_1][0][(short) currMB->b8x8[k].mode][j][i]; 

            // MBAFF or RDOQ
            currSlice->rddata->refar[LIST_1][j][i] = 0;
          }
          else
          {
            motion[block_y][block_x].ref_idx [LIST_1] = p_RDO->l1_refframe[j][i];
            motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1 + currMB->list_offset][(short)p_RDO->l1_refframe[j][i]];
            motion[block_y][block_x].mv      [LIST_1] = currSlice->all_mv[LIST_1][(short)p_RDO->l1_refframe[j][i]][(short) currMB->b8x8[k].mode][j][i];

            // MBAFF or RDOQ
            currSlice->rddata->refar[LIST_1][j][i] = p_RDO->l1_refframe[j][i];
          }
        }
      }
    }
  }

  //==== intra prediction modes ====
  currMB->c_ipred_mode = currMB->best_c_imode;
  currMB->i16offset    = currMB->best_i16offset;
  currMB->i16mode      = currMB->best_i16mode;
  currMB->cbp          = currMB->best_cbp;

  if(currMB->mb_type == I8MB)
  {
    memcpy(currMB->intra_pred_modes8x8,currSlice->b8_intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
    memcpy(currMB->intra_pred_modes   ,currSlice->b8_intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = 0; j < BLOCK_MULTIPLE; j++)
    {
      memcpy(&p_Vid->ipredmode   [currMB->block_y+j][currMB->block_x], currSlice->b8_ipredmode8x8[j], BLOCK_MULTIPLE * sizeof(char));
      memcpy(&p_Vid->ipredmode8x8[currMB->block_y+j][currMB->block_x], currSlice->b8_ipredmode8x8[j], BLOCK_MULTIPLE * sizeof(char));
    }
  }
  else if (mode!=I4MB && mode!=I8MB)
  {
    memset(currMB->intra_pred_modes,DC_PRED, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
      memset(&p_Vid->ipredmode[j][currMB->block_x], DC_PRED, BLOCK_MULTIPLE * sizeof(char));
  }
  // Residue Color Transform
  else if (mode == I4MB)
  {
    memcpy(currMB->intra_pred_modes,currSlice->b4_intra_pred_modes, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = 0; j < BLOCK_MULTIPLE; j++)
      memcpy(&p_Vid->ipredmode[currMB->block_y + j][currMB->block_x],&currSlice->b4_ipredmode[BLOCK_MULTIPLE * j], BLOCK_MULTIPLE * sizeof(char));
  }

  // Copy coding info into rddata structure for MBAFF or RDOQ
  currSlice->rddata->c_ipred_mode = currMB->c_ipred_mode;
  currSlice->rddata->i16offset    = currMB->i16offset;
  currSlice->rddata->i16mode      = currMB->i16mode;
  currSlice->rddata->cbp          = currMB->cbp;
  currSlice->rddata->cbp_blk     = currMB->cbp_blk;
  memcpy(currSlice->rddata->intra_pred_modes,currMB->intra_pred_modes, MB_BLOCK_PARTITIONS * sizeof(char));
  memcpy(currSlice->rddata->intra_pred_modes8x8,currMB->intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
  for(j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
    memcpy(&currSlice->rddata->ipredmode[j][currMB->block_x],&ipredmodes[j][currMB->block_x], BLOCK_MULTIPLE * sizeof(char));

  //==== motion vectors =====
  currSlice->set_motion_vectors_mb (currMB);
}

/*!
*************************************************************************************
* \brief
*    Set stored macroblock parameters (non RDOQ or MBAFF)
*************************************************************************************
*/
static void set_stored_macroblock_parameters (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  imgpel     **imgY  = p_Vid->enc_picture->imgY;
  imgpel    ***imgUV = p_Vid->enc_picture->imgUV;

  int         mode   = currMB->best_mode;
  int         i, j, k, ****i4p, ***i3p;
  int         block_x, block_y;

  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  //===== reconstruction values =====

  // Luma
  copy_image_data_16x16(&imgY[currMB->pix_y], p_RDO->rec_mb[0], currMB->pix_x, 0);

  if (p_Vid->AdaptiveRounding)
  {
    update_offset_params(currMB, mode, currMB->temp_transform_size_8x8_flag);
  }

  if (p_Vid->yuv_format != YUV400)
  {
    for (k = 0; k < 2; k++)
    {
      copy_image_data(&imgUV[k][currMB->pix_c_y], p_RDO->rec_mb[k + 1], currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    }
  }

  //===== coefficients and cbp =====
  i4p = p_RDO->cofAC; 
  p_RDO->cofAC = currSlice->cofAC; 
  currSlice->cofAC = i4p;

  i3p = p_RDO->cofDC; 
  p_RDO->cofDC = currSlice->cofDC; 
  currSlice->cofDC = i3p;
  currMB->cbp      = p_RDO->cbp;
  currMB->cbp_blk = currSlice->cur_cbp_blk[0];
  currMB->cbp |= currSlice->curr_cbp[0];
  currMB->cbp |= currSlice->curr_cbp[1];
  currSlice->cmp_cbp[1] = currMB->cbp; 
  currSlice->cmp_cbp[2] = currMB->cbp;

  //==== macroblock type ====
  currMB->mb_type = (short) mode;

  memcpy(currMB->b8x8, currSlice->p_RDO->best8x8, BLOCK_MULTIPLE * sizeof(Info8x8));


  //if P8x8 mode and transform size 4x4 choosen, restore motion vector data for this transform size
  if (mode == P8x8 && !currMB->temp_transform_size_8x8_flag && p_Inp->Transform8x8Mode && (currMB->valid_8x8 == TRUE))
  {
    RestoreMV8x8(currSlice, 1);
  }

  //==== transform size flag ====
  if (p_Vid->P444_joined)
  {
    if (((currMB->cbp == 0) && currSlice->cmp_cbp[1] == 0 && currSlice->cmp_cbp[2] == 0) && (currMB->mb_type != I4MB && currMB->mb_type != I8MB))
      currMB->luma_transform_size_8x8_flag = FALSE;
    else
      currMB->luma_transform_size_8x8_flag = currMB->temp_transform_size_8x8_flag;
  }
  else
  {

    if (((currMB->cbp & 15) == 0) && (currMB->mb_type != I4MB && currMB->mb_type != I8MB))
      currMB->luma_transform_size_8x8_flag = FALSE;
    else
      currMB->luma_transform_size_8x8_flag = currMB->temp_transform_size_8x8_flag;
  }

  currSlice->rddata->luma_transform_size_8x8_flag  = currMB->luma_transform_size_8x8_flag;

  if (p_Inp->rdopt == 3)  
  {
    errdo_get_best_MB(currMB);
  }

  //==== reference frames =====
  for (j = 0; j < 4; j++)
  {
    block_y = currMB->block_y + j;
    for (i = 0; i < 4; i++)
    {
      block_x = currMB->block_x + i;
      k = 2*(j >> 1)+(i >> 1);

      // backward prediction or intra
      if ((currMB->b8x8[k].pdir == 1) || is_intra(currMB))
      {
        motion[block_y][block_x].ref_idx [LIST_0] = -1;
        motion[block_y][block_x].ref_pic [LIST_0] = NULL;
        motion[block_y][block_x].mv      [LIST_0] = zero_mv;
      }
      else
      {
        if (currMB->b8x8[k].bipred && (currMB->b8x8[k].pdir == 2) && is_bipred_enabled(p_Vid, currMB->mb_type))
        {
          motion[block_y][block_x].ref_idx [LIST_0] = 0;
          motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][0];
          motion[block_y][block_x].mv      [LIST_0] = currSlice->bipred_mv[currMB->b8x8[k].bipred - 1][LIST_0][0][(short) currMB->b8x8[k].mode][j][i]; 
        }
        else
        {
          char cur_ref = p_RDO->l0_refframe[j][i];
          motion[block_y][block_x].ref_idx [LIST_0] = cur_ref;
          motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][(short)cur_ref];
          motion[block_y][block_x].mv      [LIST_0] = currSlice->all_mv[LIST_0][(short)cur_ref][(short) currMB->b8x8[k].mode][j][i];
        }
      }

      // forward prediction or intra
      if ((currMB->b8x8[k].pdir == 0) || is_intra(currMB))
      {
        motion[block_y][block_x].ref_idx [LIST_1] = -1;
        motion[block_y][block_x].ref_pic [LIST_1] = NULL;
        motion[block_y][block_x].mv      [LIST_1] = zero_mv;
      }
    }
  }

  if (currSlice->slice_type == B_SLICE)
  {
    for (j=0; j<4; j++)
    {
      block_y = currMB->block_y + j;
      for (i=0; i<4; i++)
      {
        block_x = currMB->block_x + i;
        k = 2*(j >> 1)+(i >> 1);

        // forward
        if (is_intra(currMB)||(currMB->b8x8[k].pdir == 0))
        {
          motion[block_y][block_x].ref_idx [LIST_1] = -1;
          motion[block_y][block_x].ref_pic [LIST_1] = NULL;
          motion[block_y][block_x].mv      [LIST_1] = zero_mv;
        }
        else
        {
          if (currMB->b8x8[k].bipred && (currMB->b8x8[k].pdir == 2) && is_bipred_enabled(p_Vid, currMB->mb_type))
          {
            motion[block_y][block_x].ref_idx [LIST_1] = 0;
            motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1 + currMB->list_offset][0];
            motion[block_y][block_x].mv      [LIST_1] = currSlice->bipred_mv[currMB->b8x8[k].bipred - 1][LIST_1][0][(short) currMB->b8x8[k].mode][j][i]; 
          }
          else
          {
            motion[block_y][block_x].ref_idx [LIST_1] = p_RDO->l1_refframe[j][i];
            motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1 + currMB->list_offset][(short)p_RDO->l1_refframe[j][i]];
            motion[block_y][block_x].mv      [LIST_1] = currSlice->all_mv[LIST_1][(short)p_RDO->l1_refframe[j][i]][(short) currMB->b8x8[k].mode][j][i];
          }
        }
      }
    }
  }

  //==== intra prediction modes ====
  currMB->c_ipred_mode = currMB->best_c_imode;
  currMB->i16offset    = currMB->best_i16offset;
  currMB->i16mode      = currMB->best_i16mode;
  currMB->cbp          = currMB->best_cbp;

  if(currMB->mb_type == I8MB)
  {
    memcpy(currMB->intra_pred_modes8x8,currSlice->b8_intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
    memcpy(currMB->intra_pred_modes   ,currSlice->b8_intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = 0; j < BLOCK_MULTIPLE; j++)
    {
      memcpy(&p_Vid->ipredmode   [currMB->block_y+j][currMB->block_x], currSlice->b8_ipredmode8x8[j], BLOCK_MULTIPLE * sizeof(char));
      memcpy(&p_Vid->ipredmode8x8[currMB->block_y+j][currMB->block_x], currSlice->b8_ipredmode8x8[j], BLOCK_MULTIPLE * sizeof(char));
    }
  }
  else if (mode!=I4MB && mode!=I8MB)
  {
    memset(currMB->intra_pred_modes,DC_PRED, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
      memset(&p_Vid->ipredmode[j][currMB->block_x], DC_PRED, BLOCK_MULTIPLE * sizeof(char));
  }
  // Residue Color Transform
  else if (mode == I4MB)
  {
    memcpy(currMB->intra_pred_modes,currSlice->b4_intra_pred_modes, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = 0; j < BLOCK_MULTIPLE; j++)
      memcpy(&p_Vid->ipredmode[currMB->block_y + j][currMB->block_x],&currSlice->b4_ipredmode[BLOCK_MULTIPLE * j], BLOCK_MULTIPLE * sizeof(char));
  }

  //==== motion vectors =====
  currSlice->set_motion_vectors_mb (currMB);
}


/*!
*************************************************************************************
* \brief
*    Set stored macroblock parameters (non RDOQ or MBAFF)
*************************************************************************************
*/
static void set_stored_macroblock_parameters_sp (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  imgpel     **imgY  = p_Vid->enc_picture->imgY;
  imgpel    ***imgUV = p_Vid->enc_picture->imgUV;

  int         mode   = currMB->best_mode;
  int         i, j, k, ****i4p, ***i3p;
  int         block_x, block_y;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  //===== reconstruction values =====

  // Luma
  copy_image_data_16x16(&imgY[currMB->pix_y], p_RDO->rec_mb[0], currMB->pix_x, 0);

  if((currSlice->slice_type == SP_SLICE) && p_Vid->sp2_frame_indicator==0)
  {
    for (j = 0; j < MB_BLOCK_SIZE; j++)
      memcpy(&p_Vid->lrec[currMB->pix_y+j][currMB->pix_x], p_RDO->lrec_rec[j], MB_BLOCK_SIZE * sizeof(int)); //restore coeff SP frame
  }

  if (p_Vid->AdaptiveRounding)
  {
    update_offset_params(currMB, mode, currMB->temp_transform_size_8x8_flag);
  }

  if (p_Vid->yuv_format != YUV400)
  {
    for (k = 0; k < 2; k++)
    {
      copy_image_data(&imgUV[k][currMB->pix_c_y], p_RDO->rec_mb[k + 1], currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    }

    if((currSlice->slice_type == SP_SLICE) && !p_Vid->sp2_frame_indicator)
    {
      for (k = 0; k < 2; k++)
      {
        for (j = 0; j<p_Vid->mb_cr_size_y; j++)
        {
          memcpy(&p_Vid->lrec_uv[k][currMB->pix_c_y + j][currMB->pix_c_x], p_RDO->lrec_rec_uv[k][j], p_Vid->mb_cr_size_x * sizeof(int));
        }
      }
    }
  }

  //===== coefficients and cbp =====
  i4p = p_RDO->cofAC; 
  p_RDO->cofAC = currSlice->cofAC; 
  currSlice->cofAC = i4p;

  i3p = p_RDO->cofDC; 
  p_RDO->cofDC = currSlice->cofDC; 
  currSlice->cofDC = i3p;
  currMB->cbp      = p_RDO->cbp;
  currMB->cbp_blk = currSlice->cur_cbp_blk[0];
  currMB->cbp |= currSlice->curr_cbp[0];
  currMB->cbp |= currSlice->curr_cbp[1];
  currSlice->cmp_cbp[1] = currMB->cbp; 
  currSlice->cmp_cbp[2] = currMB->cbp;

  //==== macroblock type ====
  currMB->mb_type = (short) mode;

  memcpy(currMB->b8x8, currSlice->p_RDO->best8x8, BLOCK_MULTIPLE * sizeof(Info8x8));

  //if P8x8 mode and transform size 4x4 choosen, restore motion vector data for this transform size
  if (mode == P8x8 && !currMB->temp_transform_size_8x8_flag && p_Inp->Transform8x8Mode && (currMB->valid_8x8 == TRUE))
  {
    RestoreMV8x8(currSlice, 1);
  }

  //==== transform size flag ====
  if (p_Vid->P444_joined)
  {
    if (((currMB->cbp == 0) && currSlice->cmp_cbp[1] == 0 && currSlice->cmp_cbp[2] == 0) && (currMB->mb_type != I4MB && currMB->mb_type != I8MB))
      currMB->luma_transform_size_8x8_flag = FALSE;
    else
      currMB->luma_transform_size_8x8_flag = currMB->temp_transform_size_8x8_flag;
  }
  else
  {

    if (((currMB->cbp & 15) == 0) && (currMB->mb_type != I4MB && currMB->mb_type != I8MB))
      currMB->luma_transform_size_8x8_flag = FALSE;
    else
      currMB->luma_transform_size_8x8_flag = currMB->temp_transform_size_8x8_flag;
  }

  currSlice->rddata->luma_transform_size_8x8_flag  = currMB->luma_transform_size_8x8_flag;

  if (p_Inp->rdopt == 3)  
  {
    errdo_get_best_MB(currMB);
  }

  //==== reference frames =====
  for (j = 0; j < 4; j++)
  {
    block_y = currMB->block_y + j;
    for (i = 0; i < 4; i++)
    {
      block_x = currMB->block_x + i;
      k = 2*(j >> 1)+(i >> 1);

      // backward prediction or intra
      if ((currMB->b8x8[k].pdir == 1) || is_intra(currMB))
      {
        motion[block_y][block_x].ref_idx [LIST_0] = -1;
        motion[block_y][block_x].ref_pic [LIST_0] = NULL;
        motion[block_y][block_x].mv      [LIST_0] = zero_mv;
      }
      else
      {
        if (currMB->b8x8[k].bipred && (currMB->b8x8[k].pdir == 2) && is_bipred_enabled(p_Vid, currMB->mb_type))
        {
          motion[block_y][block_x].ref_idx [LIST_0] = 0;
          motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][0];
          motion[block_y][block_x].mv      [LIST_0] = currSlice->bipred_mv[currMB->b8x8[k].bipred - 1][LIST_0][0][(short) currMB->b8x8[k].mode][j][i]; 
        }
        else
        {
          char cur_ref = p_RDO->l0_refframe[j][i];
          motion[block_y][block_x].ref_idx [LIST_0] = cur_ref;
          motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][(short)cur_ref];
          motion[block_y][block_x].mv      [LIST_0] = currSlice->all_mv[LIST_0][(short)cur_ref][(short) currMB->b8x8[k].mode][j][i];
        }
      }

      // forward prediction or intra
      if ((currMB->b8x8[k].pdir == 0) || is_intra(currMB))
      {
        motion[block_y][block_x].ref_idx [LIST_1] = -1;
        motion[block_y][block_x].ref_pic [LIST_1] = NULL;
        motion[block_y][block_x].mv      [LIST_1] = zero_mv;
      }
    }
  }

  if (currSlice->slice_type == B_SLICE)
  {
    for (j=0; j<4; j++)
    {
      block_y = currMB->block_y + j;
      for (i=0; i<4; i++)
      {
        block_x = currMB->block_x + i;
        k = 2*(j >> 1)+(i >> 1);

        // forward
        if (is_intra(currMB)||(currMB->b8x8[k].pdir == 0))
        {
          motion[block_y][block_x].ref_idx [LIST_1] = -1;
          motion[block_y][block_x].ref_pic [LIST_1] = NULL;
          motion[block_y][block_x].mv      [LIST_1] = zero_mv;
        }
        else
        {
          if (currMB->b8x8[k].bipred && (currMB->b8x8[k].pdir == 2) && is_bipred_enabled(p_Vid, currMB->mb_type))
          {
            motion[block_y][block_x].ref_idx [LIST_1] = 0;
            motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1 + currMB->list_offset][0];
            motion[block_y][block_x].mv      [LIST_1] = currSlice->bipred_mv[currMB->b8x8[k].bipred - 1][LIST_1][0][(short) currMB->b8x8[k].mode][j][i]; 
          }
          else
          {
            motion[block_y][block_x].ref_idx [LIST_1] = p_RDO->l1_refframe[j][i];
            motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1 + currMB->list_offset][(short)p_RDO->l1_refframe[j][i]];
            motion[block_y][block_x].mv      [LIST_1] = currSlice->all_mv[LIST_1][(short)p_RDO->l1_refframe[j][i]][(short) currMB->b8x8[k].mode][j][i];
          }
        }
      }
    }
  }

  //==== intra prediction modes ====
  currMB->c_ipred_mode = currMB->best_c_imode;
  currMB->i16offset    = currMB->best_i16offset;
  currMB->i16mode      = currMB->best_i16mode;
  currMB->cbp          = currMB->best_cbp;

  if(currMB->mb_type == I8MB)
  {
    memcpy(currMB->intra_pred_modes8x8,currSlice->b8_intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
    memcpy(currMB->intra_pred_modes   ,currSlice->b8_intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = 0; j < BLOCK_MULTIPLE; j++)
    {
      memcpy(&p_Vid->ipredmode   [currMB->block_y+j][currMB->block_x], currSlice->b8_ipredmode8x8[j], BLOCK_MULTIPLE * sizeof(char));
      memcpy(&p_Vid->ipredmode8x8[currMB->block_y+j][currMB->block_x], currSlice->b8_ipredmode8x8[j], BLOCK_MULTIPLE * sizeof(char));
    }
  }
  else if (mode!=I4MB && mode!=I8MB)
  {
    memset(currMB->intra_pred_modes,DC_PRED, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
      memset(&p_Vid->ipredmode[j][currMB->block_x], DC_PRED, BLOCK_MULTIPLE * sizeof(char));
  }
  // Residue Color Transform
  else if (mode == I4MB)
  {
    memcpy(currMB->intra_pred_modes,currSlice->b4_intra_pred_modes, MB_BLOCK_PARTITIONS * sizeof(char));
    for(j = 0; j < BLOCK_MULTIPLE; j++)
      memcpy(&p_Vid->ipredmode[currMB->block_y + j][currMB->block_x],&currSlice->b4_ipredmode[BLOCK_MULTIPLE * j], BLOCK_MULTIPLE * sizeof(char));
  }

  //==== motion vectors =====
  currSlice->set_motion_vectors_mb (currMB);
}


/*!
 *************************************************************************************
 * \brief
 *    Set reference frames and motion vectors
 *************************************************************************************
 */
void set_ref_and_motion_vectors_P_slice (Macroblock *currMB, PicMotionParams **motion, Info8x8 *part, int block)
{
  int mode = part->mode;
  int j = 0, i;
  int pmode   = (mode==1||mode==2||mode==3 ? mode : 4);
  int j0      = ((block >> 1)<<1);
  int i0      = ((block & 0x01)<<1);
  int i1      = i0 + part_size[pmode][0];
  int j1      = j0 + part_size[pmode][1];  
  PicMotionParams *mv = NULL;

  if (part->pdir < 0)
  {
    for (j = currMB->block_y + j0; j < currMB->block_y + j1; j++)
    {
      for (i = currMB->block_x + i0; i < currMB->block_x + i1; i++)
      {
        mv = &motion[j][i];
        mv->ref_pic[LIST_0] = NULL;
        mv->ref_pic[LIST_1] = NULL;
        mv->mv     [LIST_0] = zero_mv;
        mv->mv     [LIST_1] = zero_mv;
        mv->ref_idx[LIST_0] = -1;
        mv->ref_idx[LIST_1] = -1;
      }
    }
    return;
  }
  else
  {
    Slice *currSlice = currMB->p_Slice;
    int block_y;
    int fwref = part->ref[LIST_0];
    for (j = j0; j < j1; j++)
    {
      block_y = currMB->block_y + j;
      for (i = i0; i < i1; i++)
      {
        mv = &motion[block_y][currMB->block_x + i];
        mv->ref_pic[LIST_0] = currSlice->listX[LIST_0+currMB->list_offset][fwref];;
        mv->mv     [LIST_0] = currSlice->all_mv[LIST_0][fwref][mode][j][i];
        mv->ref_idx[LIST_0] = (char) fwref;
      }
    }
    return;
  }
}


/*!
 *************************************************************************************
 * \brief
 *    Set reference frames and motion vectors
 *************************************************************************************
 */
static void set_ref_and_motion_vectors_B_slice (Macroblock *currMB, PicMotionParams **motion, Info8x8 *part, int block)
{
  int mode = part->mode;
  int i, j=0;
  int pmode   = (mode==1||mode==2||mode==3 ? mode : 4);
  int j0      = ((block >> 1)<<1);
  int i0      = ((block & 0x01)<<1);
  int i1      = i0 + part_size[pmode][0];
  int j1      = j0 + part_size[pmode][1];
  PicMotionParams *mv = NULL;

  if (part->pdir < 0)
  {
    for (j = currMB->block_y + j0; j < currMB->block_y + j1; j++)
    {
      for (i = currMB->block_x + i0; i < currMB->block_x + i1; i++)
      {
        mv = &motion[j][i];
        mv->ref_pic[LIST_0] = NULL;
        mv->ref_pic[LIST_1] = NULL;
        mv->mv     [LIST_0] = zero_mv;
        mv->mv     [LIST_1] = zero_mv;
        mv->ref_idx[LIST_0] = -1;
        mv->ref_idx[LIST_1] = -1;
      }
    }
    return;
  }
  else
  {
    Slice *currSlice = currMB->p_Slice;
    VideoParameters *p_Vid = currMB->p_Vid;

    int block_x, block_y;

    if ((part->pdir == 0 || part->pdir == 2))
    {
      if (part->bipred && (part->pdir == 2) && is_bipred_enabled(p_Vid, mode))
      {
        for (j=j0; j<j1; j++)
        {
          block_y = currMB->block_y + j;
          for (i=i0; i<i1; i++)
          {
            block_x = currMB->block_x + i;

            motion[block_y][block_x].mv      [LIST_0] = currSlice->bipred_mv[part->bipred - 1][LIST_0][0][mode][j][i]; 
            motion[block_y][block_x].ref_idx [LIST_0] = 0;
            motion[block_y][block_x].ref_pic [LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][0];            
          }
        }
      }
      else
      {
        if (mode==0)
        {
          int fwref;
          for (j=j0; j<j1; j++)
          {
            block_y = currMB->block_y + j;
            for (i=i0; i<i1; i++)
            {
              block_x = currMB->block_x + i;
              fwref = currSlice->direct_ref_idx[block_y][block_x][LIST_0];              
              motion[block_y][block_x].ref_pic[LIST_0] = currSlice->listX[LIST_0 + currMB->list_offset][fwref];
              motion[block_y][block_x].mv     [LIST_0] = currSlice->all_mv[LIST_0][fwref][mode][j][i];
              motion[block_y][block_x].ref_idx[LIST_0] = (char) fwref;             
            }            
          }
        }
        else
        {
          int fwref = part->ref[LIST_0];
          for (j=j0; j<j1; j++)
          {
            block_y = currMB->block_y + j;
            for (i = i0; i < i1; i++)
            {                            
              block_x = currMB->block_x + i;
              motion[block_y][block_x].ref_pic[LIST_0] = currSlice->listX[LIST_0+currMB->list_offset][fwref];
              motion[block_y][block_x].mv     [LIST_0] = currSlice->all_mv[LIST_0][fwref][mode][j][i];
              motion[block_y][block_x].ref_idx[LIST_0] = (char) fwref;
            }
          }
        }
      }  
    }
    else
    {
      for (j=currMB->block_y + j0; j < currMB->block_y + j1; j++)
      {
        for (i = currMB->block_x + i0; i < currMB->block_x + i1; i++)
        {
          motion[j][i].ref_pic [LIST_0] = NULL;
          motion[j][i].mv      [LIST_0] = zero_mv;
          motion[j][i].ref_idx [LIST_0] = -1;
        }
      }
    }


    if ((part->pdir==1 || part->pdir==2))
    {
      if (part->bipred && (part->pdir == 2) && is_bipred_enabled(p_Vid, mode))
      {
        for (j=j0; j<j1; j++)
        {
          block_y = currMB->block_y + j;

          for (i=i0; i<i1; i++)
          {
            block_x = currMB->block_x + i;

            motion[block_y][block_x].mv      [LIST_1] = currSlice->bipred_mv[part->bipred - 1][LIST_1][0][mode][j][i]; 
            motion[block_y][block_x].ref_idx [LIST_1] = 0;
            motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1+currMB->list_offset][0];
          }
        }        
      }
      else
      {
        if (mode==0)
        {
          int bwref = part->ref[LIST_1];
          for (j=j0; j<j1; j++)
          {
            block_y = currMB->block_y + j;

            for (i = i0; i < i1; i++)
            {
              block_x = currMB->block_x + i;
              if (bwref != currSlice->direct_ref_idx[block_y][block_x][LIST_1])
                printf("error\n");
              motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1+currMB->list_offset][bwref]; 
              motion[block_y][block_x].mv      [LIST_1] = currSlice->all_mv[LIST_1][bwref][mode][j][i];
              motion[block_y][block_x].ref_idx [LIST_1] = (char) bwref; // currSlice->direct_ref_idx[block_y][block_x][LIST_1];              
              //motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1+currMB->list_offset][(short)motion[block_y][block_x].ref_idx [LIST_1]]; 
            }            
          }
        }
        else
        {
          int bwref = part->ref[LIST_1];
          for (j=j0; j<j1; j++)
          {
            block_y = currMB->block_y + j;
            for (i = i0; i < i1; i++)
            {
              block_x = currMB->block_x + i;

              motion[block_y][block_x].ref_pic [LIST_1] = currSlice->listX[LIST_1+currMB->list_offset][bwref];
              motion[block_y][block_x].mv      [LIST_1] = currSlice->all_mv[LIST_1][bwref][mode][j][i];
              motion[block_y][block_x].ref_idx [LIST_1] = (char) bwref;
            }
          }
        }
      }  
    }
    else
    {
      for (block_y = currMB->block_y + j0; block_y < currMB->block_y + j1; block_y++)
      {
        for (block_x = currMB->block_x + i0; block_x < currMB->block_x + i1; block_x++)
        {
          motion[block_y][block_x].ref_pic[LIST_1] = NULL;
          motion[block_y][block_x].mv     [LIST_1] = zero_mv;
          motion[block_y][block_x].ref_idx[LIST_1] = -1;
        }
      }
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    skip macroblock field inference
 * \return
 *    inferred field flag
 *************************************************************************************
 */
byte field_flag_inference(Macroblock *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  byte mb_field;

  if (currMB->mbAvailA)
  {
    mb_field = p_Vid->mb_data[currMB->mbAddrA].mb_field;
  }
  else
  {
    // check top macroblock pair
    if (currMB->mbAvailB)
      mb_field = p_Vid->mb_data[currMB->mbAddrB].mb_field;
    else
      mb_field = 0;
  }

  return mb_field;
}

/*!
 *************************************************************************************
 * \brief
 *    Store motion vectors for 8x8 partition
 *************************************************************************************
 */

void StoreMVBlock8x8(Slice *currSlice, int dir, int block8x8, int mode, Info8x8 *B8x8Info)
{
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  int i0 = (block8x8 & 0x01) << 1;
  int j0 = (block8x8 >> 1) << 1;
  int j1 = j0 + 1;

  MotionVector *****all_mv  = currSlice->all_mv;
  MotionVector **lc_l0_mv8x8 = p_RDO->all_mv8x8[dir][LIST_0];
  MotionVector **lc_l1_mv8x8 = p_RDO->all_mv8x8[dir][LIST_1];

  if (currSlice->slice_type != B_SLICE )
  {
    if (B8x8Info->pdir >= 0) //(mode8!=IBLOCK)&&(mode8!=I16MB))  // && ref != -1)
    {
      int l0_ref = B8x8Info->ref[LIST_0];
      memcpy(&lc_l0_mv8x8[j0][i0], &all_mv[LIST_0][l0_ref][mode][j0][i0],  2 * sizeof(MotionVector));
      memcpy(&lc_l0_mv8x8[j1][i0], &all_mv[LIST_0][l0_ref][mode][j1][i0],  2 * sizeof(MotionVector));
    }
  }
  else
  {
    int bipred_me = B8x8Info->bipred;
    int pdir8 = B8x8Info->pdir;

    if (pdir8 == 0) // list0
    {
      int l0_ref = B8x8Info->ref[LIST_0];
      memcpy(&lc_l0_mv8x8[j0][i0], &all_mv[LIST_0][l0_ref][mode][j0][i0],  2 * sizeof(MotionVector));
      memcpy(&lc_l0_mv8x8[j1][i0], &all_mv[LIST_0][l0_ref][mode][j1][i0],  2 * sizeof(MotionVector));
    }
    else if (pdir8 == 1) // list1
    {
      int l1_ref = B8x8Info->ref[LIST_1];
      memcpy(&lc_l1_mv8x8[j0][i0], &all_mv[LIST_1][l1_ref][mode][j0][i0],  2 * sizeof(MotionVector));
      memcpy(&lc_l1_mv8x8[j1][i0], &all_mv[LIST_1][l1_ref][mode][j1][i0],  2 * sizeof(MotionVector));
    }
    else if (pdir8==2) // bipred
    {
      int l0_ref = B8x8Info->ref[LIST_0];
      int l1_ref = B8x8Info->ref[LIST_1];
      if (bipred_me)
      {
        all_mv = currSlice->bipred_mv[bipred_me - 1];
      }

      memcpy(&lc_l0_mv8x8[j0][i0], &all_mv[LIST_0][l0_ref][mode][j0][i0],  2 * sizeof(MotionVector));
      memcpy(&lc_l0_mv8x8[j1][i0], &all_mv[LIST_0][l0_ref][mode][j1][i0],  2 * sizeof(MotionVector));
      memcpy(&lc_l1_mv8x8[j0][i0], &all_mv[LIST_1][l1_ref][mode][j0][i0],  2 * sizeof(MotionVector));
      memcpy(&lc_l1_mv8x8[j1][i0], &all_mv[LIST_1][l1_ref][mode][j1][i0],  2 * sizeof(MotionVector));
    }
    else
    {
      error("invalid direction mode", 255);
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Store motion vectors of 8x8 partitions of one macroblock
 *************************************************************************************
 */
void StoreMV8x8(Slice *currSlice, int dir)
{
  RDOPTStructure *p_RDO = currSlice->p_RDO;
  int block8x8;

  for (block8x8=0; block8x8<4; block8x8++)
    StoreMVBlock8x8(currSlice, dir, block8x8, p_RDO->tr8x8->part[block8x8].mode, &p_RDO->tr8x8->part[block8x8]);
}

/*!
*************************************************************************************
* \brief
*    Restore motion vectors for 8x8 partition
*************************************************************************************
*/
void RestoreMVBlock8x8(Slice *currSlice, int dir, int block8x8, RD_8x8DATA *tr)
{
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  MotionVector *****all_mv  = currSlice->all_mv;
  MotionVector **lc_l0_mv8x8 = p_RDO->all_mv8x8[dir][LIST_0];
  MotionVector **lc_l1_mv8x8 = p_RDO->all_mv8x8[dir][LIST_1];

  short pdir8     = tr->part[block8x8].pdir;
  short mode      = tr->part[block8x8].mode;
  short l0_ref    = tr->part[block8x8].ref[LIST_0];
  short l1_ref    = tr->part[block8x8].ref[LIST_1];
  short bipred_me = tr->part[block8x8].bipred;

  int i0 = (block8x8 & 0x01) << 1;
  int j0 = (block8x8 >> 1) << 1;
  int j1 = j0 + 1;

  if (currSlice->slice_type != B_SLICE)
  {
    if (pdir8>=0) //(mode8!=IBLOCK)&&(mode8!=I16MB))  // && ref != -1)
    {
      memcpy(&all_mv[LIST_0][l0_ref][4][j0][i0],  &lc_l0_mv8x8[j0][i0], 2 * sizeof(MotionVector));
      memcpy(&all_mv[LIST_0][l0_ref][4][j1][i0],  &lc_l0_mv8x8[j1][i0], 2 * sizeof(MotionVector));
    }
  }
  else
  {
    if (pdir8==0)  // list0
    {
      memcpy(&all_mv[LIST_0][l0_ref][mode][j0][i0],  &lc_l0_mv8x8[j0][i0], 2 * sizeof(MotionVector));
      memcpy(&all_mv[LIST_0][l0_ref][mode][j1][i0],  &lc_l0_mv8x8[j1][i0], 2 * sizeof(MotionVector));
    }
    else if (pdir8==1) // list1
    {
      memcpy(&all_mv[LIST_1][l1_ref][mode][j0][i0],  &lc_l1_mv8x8[j0][i0], 2 * sizeof(MotionVector));
      memcpy(&all_mv[LIST_1][l1_ref][mode][j1][i0],  &lc_l1_mv8x8[j1][i0], 2 * sizeof(MotionVector));
    }
    else if (pdir8==2) // bipred
    {
      if(bipred_me)
      {
        all_mv = currSlice->bipred_mv[bipred_me - 1];
      }

      memcpy(&all_mv[LIST_0][l0_ref][mode][j0][i0],  &lc_l0_mv8x8[j0][i0], 2 * sizeof(MotionVector));
      memcpy(&all_mv[LIST_0][l0_ref][mode][j1][i0],  &lc_l0_mv8x8[j1][i0], 2 * sizeof(MotionVector));
      memcpy(&all_mv[LIST_1][l1_ref][mode][j0][i0],  &lc_l1_mv8x8[j0][i0], 2 * sizeof(MotionVector));
      memcpy(&all_mv[LIST_1][l1_ref][mode][j1][i0],  &lc_l1_mv8x8[j1][i0], 2 * sizeof(MotionVector));
    }
    else
    {
      error("invalid direction mode", 255);
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Restore motion vectors of 8x8 partitions of one macroblock
 *************************************************************************************
 */
void RestoreMV8x8(Slice *currSlice, int dir)
{
  RDOPTStructure *p_RDO = currSlice->p_RDO;
  int block8x8;

  for (block8x8=0; block8x8<4; block8x8++)
    RestoreMVBlock8x8(currSlice, dir, block8x8, p_RDO->tr8x8);
}


/*!
 *************************************************************************************
 * \brief
 *    Store predictors for 8x8 partition
 *************************************************************************************
 */
void StoreNewMotionVectorsBlock8x8(Slice *currSlice, int dir, int block8x8, Info8x8 *B8x8Info)
{
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  int mode = B8x8Info->mode;
  int i0 = (block8x8 & 0x01) << 1;
  int j0 = (block8x8 >> 1) << 1;
  int j1 = j0 + 1;

  MotionVector ****all_mv_l0  = currSlice->all_mv[LIST_0];
  MotionVector ****all_mv_l1  = currSlice->all_mv[LIST_1];
  MotionVector **lc_l0_mv8x8 = &p_RDO->all_mv8x8[dir][LIST_0][j0];
  MotionVector **lc_l1_mv8x8 = &p_RDO->all_mv8x8[dir][LIST_1][j0];

 
  if (B8x8Info->pdir < 0)
  {
    memset(&lc_l0_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l0_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    return;
  }

  if (currSlice->slice_type != B_SLICE)
  {
    int l0_ref = B8x8Info->ref[LIST_0];
    memcpy(&lc_l0_mv8x8[0][i0], &all_mv_l0[l0_ref][mode][j0][i0], 2 * sizeof(MotionVector));
    memcpy(&lc_l0_mv8x8[1][i0], &all_mv_l0[l0_ref][mode][j1][i0], 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    return;
  }
  else
  {
    int bipred_me = B8x8Info->bipred;
    int pdir8     = B8x8Info->pdir;
    if (bipred_me)
    {
      all_mv_l0  = currSlice->bipred_mv[bipred_me - 1][LIST_0];
      all_mv_l1  = currSlice->bipred_mv[bipred_me - 1][LIST_1];
    }

    if ((pdir8==0 || pdir8==2))
    {
      int l0_ref = B8x8Info->ref[LIST_0];
      memcpy(&lc_l0_mv8x8[0][i0], &all_mv_l0[l0_ref][mode][j0][i0], 2 * sizeof(MotionVector));
      memcpy(&lc_l0_mv8x8[1][i0], &all_mv_l0[l0_ref][mode][j1][i0], 2 * sizeof(MotionVector));
    }
    else
    {
      memset(&lc_l0_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
      memset(&lc_l0_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    }

    if ((pdir8==1 || pdir8==2))
    {
      int l1_ref = B8x8Info->ref[LIST_1];
      memcpy(&lc_l1_mv8x8[0][i0], &all_mv_l1[l1_ref][mode][j0][i0], 2 * sizeof(MotionVector));
      memcpy(&lc_l1_mv8x8[1][i0], &all_mv_l1[l1_ref][mode][j1][i0], 2 * sizeof(MotionVector));
    }
    else
    {
      memset(&lc_l1_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
      memset(&lc_l1_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    }
  }
}


/*!
 *************************************************************************************
 * \brief
 *    Store predictors for 8x8 partition
 *************************************************************************************
 */
void store_8x8_motion_vectors_p_slice(Slice *currSlice, int dir, int block8x8, Info8x8 *B8x8Info)
{
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  int i0 = (block8x8 & 0x01) << 1;
  int j0 = (block8x8 >> 1) << 1;

  MotionVector **lc_l0_mv8x8 = &p_RDO->all_mv8x8[dir][LIST_0][j0];
  MotionVector **lc_l1_mv8x8 = &p_RDO->all_mv8x8[dir][LIST_1][j0];

  if (B8x8Info->pdir < 0)
  {
    memset(&lc_l0_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l0_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
  }
  else
  {
    int j1 = j0 + 1;
    MotionVector **all_mv_l0  = currSlice->all_mv[LIST_0][(short) B8x8Info->ref[LIST_0]][(short)B8x8Info->mode];
    memcpy(&lc_l0_mv8x8[0][i0], &all_mv_l0[j0][i0], 2 * sizeof(MotionVector));
    memcpy(&lc_l0_mv8x8[1][i0], &all_mv_l0[j1][i0], 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
  } 
}


/*!
 *************************************************************************************
 * \brief
 *    Store predictors for 8x8 partition
 *************************************************************************************
 */
void store_8x8_motion_vectors_b_slice(Slice *currSlice, int dir, int block8x8, Info8x8 *B8x8Info)
{
  RDOPTStructure  *p_RDO = currSlice->p_RDO;
  int i0 = (block8x8 & 0x01) << 1;
  int j0 = (block8x8 >> 1) << 1;

  MotionVector **lc_l0_mv8x8 = &p_RDO->all_mv8x8[dir][LIST_0][j0];
  MotionVector **lc_l1_mv8x8 = &p_RDO->all_mv8x8[dir][LIST_1][j0];
 
  if (B8x8Info->pdir < 0)
  {
    memset(&lc_l0_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l0_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
    memset(&lc_l1_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
  }
  else
  {
    int   mode = B8x8Info->mode;
    int   j1 = j0 + 1;
    MotionVector ****all_mv_l0  = currSlice->all_mv[LIST_0];
    MotionVector ****all_mv_l1  = currSlice->all_mv[LIST_1];

    int bipred_me = B8x8Info->bipred;
    int pdir8     = B8x8Info->pdir;
    if (bipred_me)
    {
      all_mv_l0  = currSlice->bipred_mv[bipred_me - 1][LIST_0];
      all_mv_l1  = currSlice->bipred_mv[bipred_me - 1][LIST_1];
    }

    if ((pdir8==0 || pdir8==2))
    {
      int l0_ref = B8x8Info->ref[LIST_0];
      memcpy(&lc_l0_mv8x8[0][i0], &all_mv_l0[l0_ref][mode][j0][i0], 2 * sizeof(MotionVector));
      memcpy(&lc_l0_mv8x8[1][i0], &all_mv_l0[l0_ref][mode][j1][i0], 2 * sizeof(MotionVector));
    }
    else
    {
      memset(&lc_l0_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
      memset(&lc_l0_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    }

    if ((pdir8==1 || pdir8==2))
    {
      int l1_ref = B8x8Info->ref[LIST_1];
      memcpy(&lc_l1_mv8x8[0][i0], &all_mv_l1[l1_ref][mode][j0][i0], 2 * sizeof(MotionVector));
      memcpy(&lc_l1_mv8x8[1][i0], &all_mv_l1[l1_ref][mode][j1][i0], 2 * sizeof(MotionVector));
    }
    else
    {
      memset(&lc_l1_mv8x8[0][i0], 0, 2 * sizeof(MotionVector));
      memset(&lc_l1_mv8x8[1][i0], 0, 2 * sizeof(MotionVector));
    }
  }
}


/*!
************************************************************************
* \brief
*    Sets MBAFF RD parameters
************************************************************************
*/
void set_mbaff_parameters(Macroblock  *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;

  int  j, i;
  int  mode         = currMB->best_mode;
  char **ipredmodes = p_Vid->ipredmode;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  RD_DATA *rdopt = currSlice->rddata;


  //===== reconstruction values =====
  copy_image_data_16x16(rdopt->rec_mb[0], &p_Vid->enc_picture->imgY[currMB->pix_y], 0, currMB->pix_x);

  if (p_Vid->yuv_format != YUV400)
  {
    copy_image_data(rdopt->rec_mb[1], &p_Vid->enc_picture->imgUV[0][currMB->pix_c_y], 0, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    copy_image_data(rdopt->rec_mb[2], &p_Vid->enc_picture->imgUV[1][currMB->pix_c_y], 0, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
  }

  //===== coefficients and cbp =====
  rdopt->mode      = mode;
  rdopt->i16offset = currMB->i16offset;
  rdopt->i16mode   = currMB->i16mode;
  rdopt->cbp       = currMB->cbp;
  rdopt->cbp_blk   = currMB->cbp_blk;
  rdopt->mb_type   = currMB->mb_type;

  rdopt->luma_transform_size_8x8_flag = currMB->luma_transform_size_8x8_flag;

  if(rdopt->mb_type == 0 && mode != 0)
  {
    mode=0;
    rdopt->mode=0;
  }

  memcpy(rdopt->cofAC[0][0][0], currSlice->cofAC[0][0][0], (4+p_Vid->num_blk8x8_uv) * 4 * 2 * 65 * sizeof(int));
  memcpy(rdopt->cofDC[0][0], currSlice->cofDC[0][0], 3 * 2 * 18 * sizeof(int));

  memcpy(rdopt->b8x8, currMB->b8x8, BLOCK_MULTIPLE * sizeof(Info8x8));

  //==== reference frames =====
  if (currSlice->slice_type == B_SLICE)
  {
    if (p_Inp->BiPredMERefinements == 1)
    {      
      int k;
      for (j = 0; j < BLOCK_MULTIPLE; j++)
      {
        for (i = 0; i < BLOCK_MULTIPLE; i++)
        {
          k = 2*(j >> 1)+(i >> 1);
          if (currMB->b8x8[k].bipred == 0)
          {
            rdopt->refar[LIST_0][j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
            rdopt->refar[LIST_1][j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1];
          }
          else
          {
            rdopt->refar[LIST_0][j][i] = 0;
            rdopt->refar[LIST_1][j][i] = 0;
          }
        }
      }
    }
    else
    {
      for (j = 0; j < BLOCK_MULTIPLE; j++)
      {
        for (i = 0; i < BLOCK_MULTIPLE; i++)
        {
          rdopt->refar[LIST_0][j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
          rdopt->refar[LIST_1][j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1];
        }
      }
    }
  }
  else
  {
    for (j = 0; j < BLOCK_MULTIPLE; j++)
    {
      for (i = 0; i < BLOCK_MULTIPLE; i++)
      {
        rdopt->refar[LIST_0][j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
        //rdopt->refar[LIST_1][j][i] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1];
      }
    }
  }

  memcpy(rdopt->intra_pred_modes,   currMB->intra_pred_modes, MB_BLOCK_PARTITIONS * sizeof(char));
  memcpy(rdopt->intra_pred_modes8x8,currMB->intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
  for (j = currMB->block_y; j < currMB->block_y + 4; j++)
  {
    memcpy(&rdopt->ipredmode[j][currMB->block_x],&ipredmodes[j][currMB->block_x], BLOCK_MULTIPLE * sizeof(char));
  }
}


void assign_enc_picture_params (Macroblock *currMB, int mode, Info8x8 *best, int block)
{
  int i,j;
  int block_y, block_x;
  int list;
  MotionVector **curr_mv = NULL;
  int list_offset = currMB->list_offset;
  VideoParameters *p_Vid  = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  int maxlist  = (currSlice->slice_type == B_SLICE) ? 1: 0;
  StorablePicture *listX = NULL;
 
  int start_x = 0, start_y = 0, end_x = BLOCK_MULTIPLE, end_y = BLOCK_MULTIPLE; 

  switch (mode)
  {
    case 1:
      start_x = 0;
      start_y = 0;
      end_x   = BLOCK_MULTIPLE;
      end_y   = BLOCK_MULTIPLE;
      break;
    case 2:
      start_x = 0;
      start_y = block;
      end_x   = BLOCK_MULTIPLE;
      end_y   = block + 2;
      break;
    case 3:
      start_x = block;
      start_y = 0;
      end_x   = block + 2;
      end_y   = BLOCK_MULTIPLE;
      break;
    case 4: //P8x8 
    default:
      start_x = ((block>>1) & 0x01)* 2;
      start_y = ((block>>1) & 0x02);
      end_x   = start_x + 2;
      end_y   = start_y + 2;
      break;
  }

  for (list = 0; list <= maxlist; list++)
  {    
    if ((best->pdir != 2) && (best->pdir != list))
    {
      for (j = currMB->block_y + start_y; j < currMB->block_y + end_y; j++)
      {        
        for (i = currMB->block_x + start_x; i < currMB->block_x + end_x; i++)
        {
          motion[j][i].ref_pic[list] = NULL;
          motion[j][i].mv[list]      = zero_mv;
          motion[j][i].ref_idx[list] = - 1;
        }
      }
    }
    else
    {
      switch (best->bipred)
      {
      case 0:
        {
          int bestref = best->ref[list];
          curr_mv = currSlice->all_mv[list][bestref][mode];
          listX = currSlice->listX[list + list_offset][bestref];
        }
        break;
      case 1:
        curr_mv = currSlice->bipred_mv[0][list][0][mode] ; //best->ref[LIST_0] has to be zero in this case
        listX = currSlice->listX[list + list_offset][0];
        break;
      case 2:
        curr_mv = currSlice->bipred_mv[1][list][0][mode] ; //best->ref[LIST_0] has to be zero in this case
        listX = currSlice->listX[list + list_offset][0];
        break;
      default:
        break;
      }

      for (j = start_y; j < end_y; j++)
      {
        block_y = currMB->block_y + j;
        for (i = start_x; i < end_x; i++)
        {
          block_x = currMB->block_x + i;
          motion[block_y][block_x].ref_pic [list]    = listX;
          motion[block_y][block_x].mv      [list]    = curr_mv[j][i];
        }
      }
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Set block 8x8 mode information
 *************************************************************************************
 */
void set_block8x8_info(Block8x8Info *b8x8info, int mode, int block,  Info8x8 *best)
{
  //----- set reference frame and direction parameters -----
  if (mode==3)
  {
    b8x8info->best[3][block  ] = *best;
    b8x8info->best[3][block+2] = *best;
  }
  else if (mode==2)
  {
    b8x8info->best[2][2*block    ] = *best;
    b8x8info->best[2][2*block + 1] = *best;
  }
  else if (mode==1)
  {
    b8x8info->best[1][0] = *best;
    b8x8info->best[1][1] = *best;
    b8x8info->best[1][2] = *best;
    b8x8info->best[1][3] = *best;

  }
  else //P8x8 
  {
    b8x8info->best[mode][block] = *best;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Set block 8x8 mode information for P8x8 mode
 *************************************************************************************
 */
void set_subblock8x8_info(Block8x8Info *b8x8info,int mode, int block, RD_8x8DATA *tr)
{            
  b8x8info->best [mode][block] = tr->part[block];
}



void update_refresh_map(Macroblock *currMB, int intra, int intra1)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  if (p_Inp->RestrictRef==1)
  {
    // Modified for Fast Mode Decision. Inchoon Choi, SungKyunKwan Univ.
    if (p_Inp->rdopt<2)
    {
      p_Vid->refresh_map[2*currMB->mb_y    ][2*currMB->mb_x  ] = (byte) (intra ? 1 : 0);
      p_Vid->refresh_map[2*currMB->mb_y    ][2*currMB->mb_x+1] = (byte) (intra ? 1 : 0);
      p_Vid->refresh_map[2*currMB->mb_y + 1][2*currMB->mb_x  ] = (byte) (intra ? 1 : 0);
      p_Vid->refresh_map[2*currMB->mb_y + 1][2*currMB->mb_x+1] = (byte) (intra ? 1 : 0);
    }
    else if (p_Inp->rdopt == 3)
    {
      p_Vid->refresh_map[2*currMB->mb_y    ][2*currMB->mb_x  ] = (byte) (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      p_Vid->refresh_map[2*currMB->mb_y    ][2*currMB->mb_x+1] = (byte) (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      p_Vid->refresh_map[2*currMB->mb_y + 1][2*currMB->mb_x  ] = (byte) (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      p_Vid->refresh_map[2*currMB->mb_y + 1][2*currMB->mb_x+1] = (byte) (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
    }
  }
  else if (p_Inp->RestrictRef==2)
  {
    p_Vid->refresh_map[2*currMB->mb_y    ][2*currMB->mb_x  ] = (byte) (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    p_Vid->refresh_map[2*currMB->mb_y    ][2*currMB->mb_x+1] = (byte) (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    p_Vid->refresh_map[2*currMB->mb_y + 1][2*currMB->mb_x  ] = (byte) (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    p_Vid->refresh_map[2*currMB->mb_y + 1][2*currMB->mb_x+1] = (byte) (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
  }
}

int valid_intra_mode(Slice *currSlice, int ipmode)
{
  InputParameters *p_Inp = currSlice->p_Inp;

  if (p_Inp->IntraDisableInterOnly==0 || (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE))
  {
    if (p_Inp->Intra4x4ParDisable && (ipmode==VERT_PRED||ipmode==HOR_PRED))
      return 0;

    if (p_Inp->Intra4x4DiagDisable && (ipmode==DIAG_DOWN_LEFT_PRED||ipmode==DIAG_DOWN_RIGHT_PRED))
      return 0;

    if (p_Inp->Intra4x4DirDisable && ipmode>=VERT_RIGHT_PRED)
      return 0;
  }
  return 1;
}

distblk compute_sad4x4_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **prd_img, int pic_opix_x, distblk min_cost)
{
  imgpel *cur_line, *prd_line;
  int i32Cost = 0;  
  int imin_cost = dist_down(min_cost);
  
  int j;
  for (j = 0; j < BLOCK_SIZE; j++)
  {
    cur_line = &cur_img[j][pic_opix_x];
    prd_line = prd_img[j];

    i32Cost += iabs(cur_line[0] - prd_line[0]);
    i32Cost += iabs(cur_line[1] - prd_line[1]);
    i32Cost += iabs(cur_line[2] - prd_line[2]);
    i32Cost += iabs(cur_line[3] - prd_line[3]);

    if (i32Cost > imin_cost)
    {
      return(min_cost);
    }
  }
  return dist_scale(i32Cost);
}

distblk compute_sse4x4_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **prd_img, int pic_opix_x, distblk min_cost)
{
  int j, i;
  imgpel *cur_line, *prd_line;
  int i32Cost = 0;
  int imin_cost = dist_down(min_cost);
  for (j = 0; j < BLOCK_SIZE; j++)
  {
    cur_line = &cur_img[j][pic_opix_x];
    prd_line = prd_img[j];
    for (i = 0; i < BLOCK_SIZE; i++)
    {
       i32Cost += iabs2(*cur_line++ - *prd_line++);
    }

    if (i32Cost > imin_cost)
    {
      return(min_cost);
    }
  }
  return dist_scale(i32Cost);
}

distblk compute_satd4x4_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **prd_img, int pic_opix_x, distblk min_cost)
{
  int j, i;
  imgpel *cur_line, *prd_line;
  short diff[16];

  short *d = &diff[0];

  for (j = 0; j < BLOCK_SIZE; j++)
  {
    cur_line = &cur_img[j][pic_opix_x];
    prd_line = prd_img[j];

    for (i = 0; i < BLOCK_SIZE; i++)
    {
      *d++ = *cur_line++ - *prd_line++;
    }
  }

  return dist_scale(HadamardSAD4x4 (diff));
}

static distblk compute_comp4x4_cost(VideoParameters *p_Vid, imgpel **cur_img, imgpel **prd_img, int pic_opix_x, distblk min_cost)
{
  int j, i;
  imgpel *cur_line, *prd_line;
  short diff[16];

  short *d = &diff[0];

  for (j = 0; j < BLOCK_SIZE; j++)
  {
    cur_line = &cur_img[j][pic_opix_x];
    prd_line = prd_img[j];

    for (i = 0; i < BLOCK_SIZE; i++)
    {
      *d++ = *cur_line++ - *prd_line++;
    }
  }      
  return(p_Vid->distortion4x4 (diff, min_cost));
}



void update_qp_cbp_tmp(Macroblock *currMB, int cbp)
{
  if (((cbp!=0 || currMB->best_mode==I16MB) && (currMB->best_mode!=IPCM) ))
    currMB->prev_cbp = 1;
  else if ((cbp==0) || (currMB->best_mode==IPCM))
  {
    currMB->prev_cbp  = 0;
    currMB->qp        = currMB->prev_qp;
    currMB->p_Vid->qp = currMB->qp;
    update_qp(currMB);        
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Update QP Parameters (in case of SKIP MBs or MBAFF)
 *************************************************************************************
 */

void update_qp_cbp(Macroblock *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  // delta_qp is present only for non-skipped macroblocks
  if ((currMB->cbp!=0 || currMB->best_mode == I16MB) && (currMB->best_mode != IPCM))
    currMB->prev_cbp = 1;
  else
  {
    currMB->prev_cbp = 0;
    currMB->qp       = currMB->prev_qp;
    p_Vid->qp        = currMB->qp;
    update_qp(currMB); 
  }

  if (p_Inp->MbInterlace)
  {
    Slice *currSlice = currMB->p_Slice;
    // update rdopt buffered qps...
    currSlice->rddata->qp       = currMB->qp;
    currSlice->rddata->prev_cbp = currMB->prev_cbp;
  }  
}

/*!
***************************************************************************
// For MB level field/frame coding
***************************************************************************
*/
void copy_rdopt_data (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  int i, j;
  RD_DATA *rdopt = currSlice->rddata;

  int mode;
  short b8mode, b8pdir;
  int block_y;

  int list_offset = currMB->list_offset;

  mode                = rdopt->mode;
  currMB->mb_type     = rdopt->mb_type;    // copy mb_type
  currMB->cbp         = rdopt->cbp;        // copy cbp
  currMB->cbp_blk     = rdopt->cbp_blk;    // copy cbp_blk
  currMB->i16offset   = rdopt->i16offset;
  currMB->i16mode     = rdopt->i16mode;

  currMB->prev_qp  = rdopt->prev_qp;
  currMB->prev_dqp = rdopt->prev_dqp;
  currMB->prev_cbp = rdopt->prev_cbp;
  currMB->qp       = rdopt->qp;
  update_qp (currMB);

  currMB->c_ipred_mode = rdopt->c_ipred_mode;

  memcpy(currSlice->cofAC[0][0][0],rdopt->cofAC[0][0][0], (4 + p_Vid->num_blk8x8_uv) * 4 * 2 * 65 * sizeof(int));
  memcpy(currSlice->cofDC[0][0],rdopt->cofDC[0][0], 3 * 2 * 18 * sizeof(int));

  for (j = 0; j < BLOCK_MULTIPLE; j++)
  {
    block_y = currMB->block_y + j;    
    for (i = 0; i < BLOCK_MULTIPLE; i++)
    {
      motion[block_y][currMB->block_x + i].ref_idx [LIST_0] = rdopt->refar[LIST_0][j][i];
      motion[block_y][currMB->block_x + i].ref_pic [LIST_0] = rdopt->refar[LIST_0][j][i] < 0 ? NULL :
      currSlice->listX[LIST_0 + list_offset][(short)rdopt->refar[LIST_0][j][i]];
    }
  }

  if (currSlice->slice_type == B_SLICE)
  {
    for (j = 0; j < BLOCK_MULTIPLE; j++)
    {
      block_y = currMB->block_y + j;      
      for (i = 0; i < BLOCK_MULTIPLE; i++)
      {
        motion[block_y][currMB->block_x + i].ref_idx [LIST_1] = rdopt->refar[LIST_1][j][i];
        motion[block_y][currMB->block_x + i].ref_pic [LIST_1] = rdopt->refar[LIST_1][j][i] < 0 ? NULL :
        currSlice->listX[LIST_1 + list_offset][(short) rdopt->refar[LIST_1][j][i]];
      }
    }
  }


  //===== reconstruction values =====
  copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], rdopt->rec_mb[0], currMB->pix_x, 0);

  if (p_Vid->yuv_format != YUV400)
  {
    copy_image_data(&p_Vid->enc_picture->imgUV[0][currMB->pix_c_y], rdopt->rec_mb[1], currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    copy_image_data(&p_Vid->enc_picture->imgUV[1][currMB->pix_c_y], rdopt->rec_mb[2], currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
  }


  memcpy(currMB->b8x8, rdopt->b8x8, BLOCK_MULTIPLE * sizeof(Info8x8));

  currMB->luma_transform_size_8x8_flag = rdopt->luma_transform_size_8x8_flag;

  //==== intra prediction modes ====
  if (mode == P8x8)
  {
    memcpy(currMB->intra_pred_modes,rdopt->intra_pred_modes, MB_BLOCK_PARTITIONS * sizeof(char));
    for (j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
      memcpy(&p_Vid->ipredmode[j][currMB->block_x],&rdopt->ipredmode[j][currMB->block_x], BLOCK_MULTIPLE * sizeof(char));
  }
  else if (mode != I4MB && mode != I8MB)
  {
    memset(currMB->intra_pred_modes,DC_PRED, MB_BLOCK_PARTITIONS * sizeof(char));
    for (j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++)
      memset(&p_Vid->ipredmode[j][currMB->block_x],DC_PRED, BLOCK_MULTIPLE * sizeof(char));
  }
  else if (mode == I4MB || mode == I8MB)
  {
    memcpy(currMB->intra_pred_modes,rdopt->intra_pred_modes, MB_BLOCK_PARTITIONS * sizeof(char));
    memcpy(currMB->intra_pred_modes8x8,rdopt->intra_pred_modes8x8, MB_BLOCK_PARTITIONS * sizeof(char));
    for (j = currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; j++) 
    {
      memcpy(&p_Vid->ipredmode[j][currMB->block_x],&rdopt->ipredmode[j][currMB->block_x], BLOCK_MULTIPLE * sizeof(char));
    }
  }

  if (currSlice->mb_aff_frame_flag || (currSlice->UseRDOQuant && currSlice->RDOQ_QP_Num > 1))
  {
    // motion vectors
    if (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE)
      copy_motion_vectors_MB (currSlice, rdopt);

    if (!is_intra(currMB))
    {
      currMB->b8x8[0].bipred = 0;
      currMB->b8x8[1].bipred = 0;
      currMB->b8x8[2].bipred = 0;
      currMB->b8x8[3].bipred = 0;

      for (j = 0; j < 4; j++)
      {
        for (i = 0; i < 4; i++)
        {
          b8mode = currMB->b8x8[(i >> 1) + 2 * (j >> 1)].mode;
          b8pdir = currMB->b8x8[(i >> 1) + 2 * (j >> 1)].pdir;

          if (b8pdir!=1)
          {
            motion[j+currMB->block_y][i+currMB->block_x].mv[LIST_0] = rdopt->all_mv[LIST_0][(short)rdopt->refar[LIST_0][j][i]][b8mode][j][i];
          }
          else
          {
            motion[j+currMB->block_y][i+currMB->block_x].mv[LIST_0] = zero_mv;
          }
          if (currSlice->slice_type == B_SLICE)
          {
            if (b8pdir!=0)
            {
              motion[j+currMB->block_y][i+currMB->block_x].mv[LIST_1] = rdopt->all_mv[LIST_1][(short)rdopt->refar[LIST_1][j][i]][b8mode][j][i];
            }
            else
            {
              motion[j+currMB->block_y][i+currMB->block_x].mv[LIST_1] = zero_mv;
            }
          }
        }
      }
    }
    else
    {

      if (currSlice->slice_type == B_SLICE)
      {
        for (j = currMB->block_y; j < currMB->block_y + 4; j++)
        {
          for (i = currMB->block_x; i < currMB->block_x + 4; i++)
          {
            motion[j][i].mv[LIST_0] = zero_mv;
            motion[j][i].mv[LIST_1] = zero_mv;
          }
        }
      }
      else
      {
        for (j = currMB->block_y; j < currMB->block_y + 4; j++)
        {
          for (i = currMB->block_x; i < currMB->block_x + 4; i++)
          {
            motion[j][i].mv[LIST_0] = zero_mv;
          }
        }
      }
    }
  }
} // end of copy_rdopt_data



