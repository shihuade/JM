
/*!
 *************************************************************************************
 *
 * \file me_umhexsmp.c
 *
 * \brief
 *   Fast integer pixel and sub pixel motion estimation
 *   Improved and simplified from the original UMHexagonS algorithms
 *   See JVT-P021 for details
 *
 * \author
 *    Main contributors: (see contributors.h for copyright, address and affiliation details)
 *    - Zhibo Chen                      <chenzhibo@tsinghua.org.cn>
 *    - JianFeng Xu                     <fenax@video.mdc.tsinghua.edu.cn>
 *    - Wenfang Fu                      <fwf@video.mdc.tsinghua.edu.cn>
 *
 *    - Xiaoquan Yi                     <xyi@engr.scu.edu>
 *    - Jun Zhang                       <jzhang2@engr.scu.edu>
 *
 * \date
 *    6. Nov. 2006
 *************************************************************************************
 */

#include <limits.h>

#include "global.h"
#include "memalloc.h"
#include "me_umhexsmp.h"
#include "refbuf.h"
#include "macroblock.h"
#include "me_distortion.h"
#include "mv_search.h"


static const MotionVector Diamond[4] = {{-4, 0}, {4, 0}, {0, -4}, {0, 4}};
static const MotionVector Diamond_SubPelSearch[4] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
static const MotionVector Hexagon[6] = {{-8, 0}, {8, 0},{-4, -8}, {4, 8}, {-4, 8}, {4 , -8}};
static const short Big_Hexagon_X[16] = {-16, 16, 0, 0,-16, 16,-16, 16,-16, 16,-16, 16,-8, 8,-8, 8};
static const short Big_Hexagon_Y[16] = { 0, 0,-16, 16,-4, 4, 4,-4,-8, 8, 8,-8,-12, 12, 12,-12};

static const short block_type_shift_factor[8] = {0, 0, 1, 1, 2, 3, 3, 1}; // last one relaxed to 1 instead 4

// Macro for motion estimation cost computation per match
#undef SEARCH_ONE_PIXEL_BIPRED
#undef SEARCH_ONE_PIXEL
#define SEARCH_ONE_PIXEL                                                                        \
if((iabs(cand.mv_x - center.mv_x)>>2) <= search_range &&                                           \
   (iabs(cand.mv_y - center.mv_y)>>2) <= search_range)                                             \
{                                                                                             \
    mcost  = mv_cost (p_Vid, lambda_factor, &cand, &pred);                                      \
    mcost += mv_block->computePredFPel( ref_picture, mv_block, min_mcost - mcost, &cand);       \
    if (mcost < min_mcost)                                                                      \
    {                                                                                           \
      best = cand;                                                                              \
      min_mcost = mcost;                                                                        \
    }                                                                                           \
}

#define SEARCH_ONE_PIXEL_BIPRED                                                                 \
if ((iabs(cand.mv_x - center2.mv_x)>>2) <= search_range                                              \
&& (iabs(cand.mv_y - center2.mv_y)>>2) <= search_range)                                              \
{                                                                                               \
  mcost  = mv_cost (p_Vid, lambda_factor, &center1, &pred1);                                    \
  mcost += mv_cost (p_Vid, lambda_factor, &cand, &pred2);                                       \
  if (mcost < min_mcost)                                                                        \
  {                                                                                             \
    mcost  += mv_block->computeBiPredFPel(ref_picture1, ref_picture2,                           \
                           mv_block, min_mcost - mcost, &center1,  &cand);                      \
    if (mcost < min_mcost)                                                                      \
    {                                                                                           \
      best = cand;                                                                              \
      min_mcost = mcost;                                                                        \
    }                                                                                           \
  }                                                                                             \
}

/*!
 ************************************************************************
 * \brief
 *    Set thresholds for fast motion estimation
 *    Those thresholds may be adjusted to trade off rate-distortion
 *    performance and simplified UMHEX speed
 ************************************************************************
 */
void smpUMHEX_init(VideoParameters *p_Vid)
{
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;

  p_UMHexSMP->SymmetricalCrossSearchThreshold1 = dist_scale(800);
  p_UMHexSMP->SymmetricalCrossSearchThreshold2 = dist_scale(7000);
  p_UMHexSMP->ConvergeThreshold                = dist_scale(1000);
  p_UMHexSMP->SubPelThreshold1                 = dist_scale(1000);
  p_UMHexSMP->SubPelThreshold3                 = dist_scale(400);
}

/*!
 ************************************************************************
 * \brief
 *    Allocation of space for fast motion estimation
 ************************************************************************
 */
int smpUMHEX_get_mem(VideoParameters *p_Vid)
{
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;
  int memory_size = 0;
  if (NULL==(p_UMHexSMP->flag_intra = calloc((p_Vid->width>>4)+1, sizeof(byte))))
    no_mem_exit("smpUMHEX_get_mem: flag_intra");

  memory_size += get_mem3Ddistblk(&p_UMHexSMP->l0_cost, 9, p_Vid->height >> 2, p_Vid->width >> 2);
  memory_size += get_mem3Ddistblk(&p_UMHexSMP->l1_cost, 9, p_Vid->height >> 2, p_Vid->width >> 2);
  memory_size += get_mem2D(&p_UMHexSMP->SearchState, 7, 7);

  return memory_size;
}

/*!
 ************************************************************************
 * \brief
 *    Free space for fast motion estimation
 ************************************************************************
 */
void smpUMHEX_free_mem(VideoParameters *p_Vid)
{
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;
  free_mem3Ddistblk(p_UMHexSMP->l0_cost );
  free_mem3Ddistblk(p_UMHexSMP->l1_cost );
  free_mem2D(p_UMHexSMP->SearchState);

  free (p_UMHexSMP->flag_intra);
  free (p_UMHexSMP);
}


/*!
************************************************************************
* \brief
*    Fast integer pixel block motion estimation
************************************************************************
*/
distblk                                     //  ==> minimum motion cost after search
smpUMHEXIntegerPelBlockMotionSearch (Macroblock *currMB,      // <--  current Macroblock
                                     MotionVector *pred_mv,    // <--  motion vector predictor (x|y) in sub-pel units
                                     MEBlock *mv_block,
                                     distblk     min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                     int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                                     )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice           *currSlice = currMB->p_Slice;
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;
  int   blocktype     = mv_block->blocktype;  
  short blocksize_x   = mv_block->blocksize_x;            // horizontal block size  
  short blocksize_y   = mv_block->blocksize_y;            // vertical block size
  short pic_pix_x     = mv_block->pos_x_padded;
  short pic_pix_y     = mv_block->pos_y_padded;
  
  int   list = mv_block->list;
  int   cur_list = list + currMB->list_offset;
  short ref = mv_block->ref_idx;
  StorablePicture *ref_picture = currSlice->listX[cur_list][ref];
  int  search_range = mv_block->searchRange.max_x >> 2;

  MotionVector *mv    = &mv_block->mv[list];
  MotionVector iMinNow, cand, center, pred, best = {0, 0};

  int   search_step;
  distblk   mcost;
  uint16  i, j, m;

  pic_pix_x   = mv_block->pos_x_padded;
  pic_pix_y   = mv_block->pos_y_padded;
  pred.mv_x   = pic_pix_x + pred_mv->mv_x;       // predicted position x (in sub-pel units)
  pred.mv_y   = pic_pix_y + pred_mv->mv_y;       // predicted position y (in sub-pel units)
  center.mv_x = pic_pix_x + mv->mv_x;            // center position x (in sub-pel units)
  center.mv_y = pic_pix_y + mv->mv_y;            // center position y (in sub-pel units)


  //check the center median predictor

  cand = center;
  mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred);

  mcost += mv_block->computePredFPel(ref_picture, mv_block, min_mcost - mcost, &cand);

  if (mcost < min_mcost)
  {
    min_mcost = mcost;
    best = cand;
  }

  iMinNow = best;

  // check 0
  if ((0 != pred_mv->mv_x) || (0 != pred_mv->mv_y))
  {
    cand.mv_x = pic_pix_x;
    cand.mv_y = pic_pix_y;
    SEARCH_ONE_PIXEL;
    iMinNow = best;
  }

  // If the min_mcost is small enough, do a local search then terminate
  // Ihis is good for stationary or quasi-stationary areas
  if (min_mcost < (p_UMHexSMP->ConvergeThreshold >> block_type_shift_factor[blocktype]))
  {
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL;
    }
    mv->mv_x = (short) (best.mv_x - pic_pix_x);
    mv->mv_y = (short) (best.mv_y - pic_pix_y);
    return min_mcost;
  }

  // Small local search
  iMinNow = best;
  for (m = 0; m < 4; m++)
  {
    cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
    cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
    SEARCH_ONE_PIXEL;
  }

  // First_step: Symmetrical-cross search
  // If distortion is large, use large shapes. Otherwise, compact shapes are faster
  if ( (blocktype == 1 &&
    min_mcost > (p_UMHexSMP->SymmetricalCrossSearchThreshold1 >> block_type_shift_factor[blocktype])) ||
    (min_mcost > (p_UMHexSMP->SymmetricalCrossSearchThreshold2>>block_type_shift_factor[blocktype])) )
  {
    iMinNow = best;

    for(i = 4; i <= 4 * search_range - 4; i+= 8)
    {
      search_step = i;
      cand.mv_x = (short) (iMinNow.mv_x + search_step);
      cand.mv_y = iMinNow.mv_y;
      SEARCH_ONE_PIXEL;

      cand.mv_x = (short) (iMinNow.mv_x - search_step);
      SEARCH_ONE_PIXEL;

      cand.mv_x = iMinNow.mv_x;
      cand.mv_y = (short) (iMinNow.mv_y + search_step);
      SEARCH_ONE_PIXEL;

      cand.mv_y = (short) (iMinNow.mv_y - search_step);
      SEARCH_ONE_PIXEL;
    }

    // Hexagon Search
    iMinNow = best;

    for (m = 0; m < 6; m++)
    {
      cand.mv_x = iMinNow.mv_x + Hexagon[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Hexagon[m].mv_y;
      SEARCH_ONE_PIXEL;
    }
    // Multi Big Hexagon Search
    iMinNow = best;

    for(i = 1; i <= search_range >> 2; i++)
    {
      for (m = 0; m < 16; m++)
      {
        cand.mv_x = iMinNow.mv_x + Big_Hexagon_X[m]*i;
        cand.mv_y = iMinNow.mv_y + Big_Hexagon_Y[m]*i;
        SEARCH_ONE_PIXEL;
      }
    }
  }

  // Search up_layer predictor for non 16x16 blocks
  if (blocktype > 1)
  {
    cand.mv_x = pic_pix_x + (p_UMHexSMP->pred_MV_uplayer_X / 4) * 4;
    cand.mv_y = pic_pix_y + (p_UMHexSMP->pred_MV_uplayer_Y / 4) * 4;
    SEARCH_ONE_PIXEL;
  }

  if(center.mv_x != pic_pix_x || center.mv_y != pic_pix_y)
  {
    cand.mv_x = pic_pix_x;
    cand.mv_y = pic_pix_y;
    SEARCH_ONE_PIXEL;

    iMinNow = best;
    // Local diamond search
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL;
    }
  }

  // If the minimum cost is small enough, do a local search
  // and finish the search here
  if (min_mcost < (p_UMHexSMP->ConvergeThreshold >> block_type_shift_factor[blocktype]))
  {
    iMinNow = best;

    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL;
    }
    mv->mv_x = (short) (best.mv_x - pic_pix_x);
    mv->mv_y = (short) (best.mv_y - pic_pix_y);
    return min_mcost;
  }

  //second_step:  Extended Hexagon-based Search
  for(i = 0; i < search_range; i++)
  {
    iMinNow = best;
    for (m = 0; m < 6; m++)
    {
      cand.mv_x = iMinNow.mv_x + Hexagon[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Hexagon[m].mv_y;
      SEARCH_ONE_PIXEL;
    }
    // The minimum cost point happens in the center
    if (best.mv_x == iMinNow.mv_x && best.mv_y == iMinNow.mv_y)
    {
      break;
    }
  }

  //third_step: Small diamond search
  for(i = 0; i < search_range; i++)
  {
    iMinNow = best;
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL;
    }

    // The minimum cost point happens in the center
    if (best.mv_x == iMinNow.mv_x && best.mv_y == iMinNow.mv_y)
    {
      break;
    }
  }

  mv->mv_x = (short) (best.mv_x - pic_pix_x);
  mv->mv_y = (short) (best.mv_y - pic_pix_y);

  for (j=(mv_block->pos_y2); j < (mv_block->pos_y2) + (blocksize_y>>2); j++)
  {
    for (i=(mv_block->pos_x2); i < (mv_block->pos_x2) + (blocksize_x>>2); i++)    
    {
      if(mv_block->list == 0)
      {
        p_UMHexSMP->l0_cost[blocktype][j][i] = min_mcost;
      }
      else
      {
        p_UMHexSMP->l1_cost[blocktype][j][i] = min_mcost;
      }
    }
  }

  return min_mcost;
}

/*!
 ***********************************************************************
 * \brief
 *    Sub pixel block motion search enhanced
 ***********************************************************************
 */
distblk                                               //  ==> minimum motion cost after search
smpUMHEXFullSubPelBlockMotionSearch (Macroblock *currMB,      // <--  current Macroblock
                                     MotionVector *pred_mv,    // <--  motion vector predictor (x|y) in sub-pel units
                                     MEBlock *mv_block,
                                     distblk     min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                     int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                                     )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;
  int   pos, best_pos;
  distblk mcost;

  MotionVector cand;

  int   blocktype = mv_block->blocktype;
  int   list = mv_block->list;
  int   list_offset = currMB->list_offset;
  short ref = mv_block->ref_idx;
  MotionVector *mv  = &mv_block->mv[list];
 
  int   check_position0 = (!p_Inp->rdopt && currSlice->slice_type!=B_SLICE && ref==0 && blocktype==1 && mv->mv_x==0 && mv->mv_y==0);

  int   max_pos2        = ( !p_Vid->start_me_refinement_hp ? imax(1, mv_block->search_pos2) : mv_block->search_pos2);
  MotionVector cmv;

  StorablePicture *ref_picture = currSlice->listX[list + list_offset][ref];


  /*********************************
   *****                       *****
   *****  HALF-PEL REFINEMENT  *****
   *****                       *****
   *********************************/

  //===== loop over search positions =====
  for (best_pos = 0, pos = p_Vid->start_me_refinement_hp; pos < max_pos2; pos++)
  {
    cand.mv_x = mv->mv_x + p_Vid->spiral_hpel_search[pos].mv_x;    // quarter-pel units
    cand.mv_y = mv->mv_y + p_Vid->spiral_hpel_search[pos].mv_y;    // quarter-pel units

    //----- set motion vector cost -----
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred_mv);

    if (mcost >= min_mcost) continue;

    cmv = pad_MVs(cand, mv_block);

    mcost += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cmv);

    if (pos==0 && check_position0)
    {
      distblk dbTmp = weighted_cost (lambda_factor, 16);
      mcost = (mcost > dbTmp)? (mcost - dbTmp):0;
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
    if (min_mcost < (p_UMHexSMP->SubPelThreshold3 >> block_type_shift_factor[blocktype]))
    {
      break;
    }
  }

  if (best_pos)
  {
    add_mvs(mv, &p_Vid->spiral_hpel_search[best_pos]);
  }

  if ((mv->mv_x == 0) && (mv->mv_y == 0) && (pred_mv->mv_x == 0 && pred_mv->mv_y == 0) &&
    (min_mcost < (p_UMHexSMP->SubPelThreshold1 >> block_type_shift_factor[blocktype])) )
  {
    best_pos = 0;
    return min_mcost;
  }
  if ( !p_Vid->start_me_refinement_qp )
    min_mcost = DISTBLK_MAX;
  /************************************
   *****                          *****
   *****  QUARTER-PEL REFINEMENT  *****
   *****                          *****
   ************************************/

  //===== loop over search positions =====
  for (best_pos = 0, pos = p_Vid->start_me_refinement_qp; pos < mv_block->search_pos4; pos++)
  {
    cand.mv_x = mv->mv_x + p_Vid->spiral_search[pos].mv_x;    // quarter-pel units
    cand.mv_y = mv->mv_y + p_Vid->spiral_search[pos].mv_y;    // quarter-pel units

    //----- set motion vector cost -----
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred_mv);

    if (mcost >= min_mcost) continue;

    cmv = pad_MVs(cand, mv_block);

    mcost += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cmv);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
    if (min_mcost < (p_UMHexSMP->SubPelThreshold3 >> block_type_shift_factor[blocktype]))
    {
      break;
    }
  }

  if (best_pos)
  {
    add_mvs(mv, &p_Vid->spiral_search[best_pos]);
  }

  //===== return minimum motion cost =====
  return min_mcost;
}


/*!
 ************************************************************************
 * \brief
 *    Fast sub pixel block motion estimation
 ************************************************************************
 */
distblk                                     //  ==> minimum motion cost after search
smpUMHEXSubPelBlockMotionSearch  (
                                  Macroblock *currMB,      // <--  current Macroblock
                                  MotionVector *pred_mv,    // <--  motion vector predictor (x|y) in sub-pel units
                                  MEBlock   *mv_block,           
                                  distblk     min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                  int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                                  )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;
  distblk   mcost;
  MotionVector cand, iMinNow, currmv = {0, 0}, cand_pad;

  int   blocktype = mv_block->blocktype;
  int   list = mv_block->list;
  int   list_offset = currMB->list_offset;
  short ref = mv_block->ref_idx;
  MotionVector *mv  = &mv_block->mv[list];

  StorablePicture *ref_picture = currSlice->listX[list+list_offset][ref];

  short dynamic_search_range = 3, i, m;
  int   pred_frac_mv_x,pred_frac_mv_y,abort_search;
  int   pred_frac_up_mv_x, pred_frac_up_mv_y;

  pred_frac_mv_x = (pred_mv->mv_x - mv->mv_x) %4; //& 0x03;
  pred_frac_mv_y = (pred_mv->mv_y - mv->mv_y) %4; //& 0x03;

  pred_frac_up_mv_x = (p_UMHexSMP->pred_MV_uplayer_X - mv->mv_x) & 0x03;
  pred_frac_up_mv_y = (p_UMHexSMP->pred_MV_uplayer_Y - mv->mv_y) & 0x03;

  memset(p_UMHexSMP->SearchState[0], 0, (2 * dynamic_search_range + 1)*(2 * dynamic_search_range + 1));

  p_UMHexSMP->SearchState[dynamic_search_range][dynamic_search_range] = 1;
  if( !p_Vid->start_me_refinement_hp )
  {
    cand.mv_x = mv->mv_x;
    cand.mv_y = mv->mv_y;
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred_mv);
    cand_pad = pad_MVs (cand, mv_block); //cand = pad_MVs (cand, mv_block);

    mcost   += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cand_pad); //&cand);
    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      currmv = cand;
    }
  }
  else
  {
    currmv = *mv;
  }

  // If the min_mcost is small enough and other statistics are positive,
  // better to stop the search now
  if ( ((mv->mv_x) == 0) && ((mv->mv_y) == 0) &&
    (pred_frac_mv_x == 0 && pred_frac_up_mv_x == 0) &&
    (pred_frac_mv_y == 0 && pred_frac_up_mv_y == 0) &&
    (min_mcost < (p_UMHexSMP->SubPelThreshold1 >> block_type_shift_factor[blocktype])) )
  {
    *mv = currmv;
    return min_mcost;
  }

  if(pred_frac_mv_x || pred_frac_mv_y)
  {
    cand.mv_x = (short) (mv->mv_x + pred_frac_mv_x);
    cand.mv_y = (short) (mv->mv_y + pred_frac_mv_y);
    p_UMHexSMP->SearchState[cand.mv_y -mv->mv_y + dynamic_search_range][cand.mv_x - mv->mv_x + dynamic_search_range] = 1;
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred_mv);

    cand_pad = pad_MVs (cand, mv_block); //cand = pad_MVs (cand, mv_block);
    mcost   += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cand_pad); //&cand);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      currmv = cand;
    }
  }

  // Multiple small diamond search
  for(i = 0; i < dynamic_search_range; i++)
  {
    abort_search = 1;

    iMinNow = currmv;

    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond_SubPelSearch[m].mv_x; //Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond_SubPelSearch[m].mv_y; //Diamond[m].mv_y;

      if(iabs(cand.mv_x - mv->mv_x) <= dynamic_search_range && iabs(cand.mv_y - mv->mv_y)  <= dynamic_search_range)
      {
        if(!p_UMHexSMP->SearchState[cand.mv_y - mv->mv_y + dynamic_search_range][cand.mv_x - mv->mv_x + dynamic_search_range])
        {
          p_UMHexSMP->SearchState[cand.mv_y - mv->mv_y + dynamic_search_range][cand.mv_x - mv->mv_x + dynamic_search_range] = 1;
          mcost = mv_cost (p_Vid, lambda_factor, &cand, pred_mv);
          cand_pad = pad_MVs (cand, mv_block); //cand = pad_MVs (cand, mv_block);
          mcost += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cand_pad); //&cand);
          if (mcost < min_mcost)
          {
            min_mcost    = mcost;
            currmv = cand;
            abort_search = 0;
          }
          if (min_mcost < (p_UMHexSMP->SubPelThreshold3 >> block_type_shift_factor[blocktype]))
          {
            *mv = currmv;
            return min_mcost;
          }
        }
      }
    }
    // If the minimum cost point is in the center, break out the loop
    if (abort_search)
    {
      break;
    }
  }

  *mv = currmv;

  //===== return minimum motion cost =====
  return min_mcost;
}

distblk                                                   //  ==> minimum motion cost after search
smpUMHEXSubPelBlockME (Macroblock *currMB,       // <--  current Macroblock
                       MotionVector *pred_mv,    // <--  motion vector predictor (x|y) in sub-pel units
                       MEBlock *mv_block, 
                       distblk       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                       int*      lambda_factor                       )
{  
  if(mv_block->blocktype > 1)
  {
    min_mcost =  smpUMHEXSubPelBlockMotionSearch (currMB, pred_mv, mv_block, min_mcost, lambda_factor[Q_PEL]);
  }
  else
  {
    min_mcost =  smpUMHEXFullSubPelBlockMotionSearch (currMB, pred_mv, mv_block, min_mcost, lambda_factor[Q_PEL]);
  }

  return min_mcost;
}

/*!
 ************************************************************************
 * \brief
 *    smpUMHEXBipredIntegerPelBlockMotionSearch: fast pixel block motion search for bipred mode
 *
 ************************************************************************
 */
distblk                                                                 //  ==> minimum motion cost after search
smpUMHEXBipredIntegerPelBlockMotionSearch (Macroblock *currMB,      // <--  current Macroblock
                                           int       list,          // <--  Current reference list
                                           MotionVector *pred_mv1,  // <--  motion vector predictor (x|y) in sub-pel units
                                           MotionVector *pred_mv2,  // <--  motion vector predictor (x|y) in sub-pel units
                                           MotionVector *mv1,       // <--> in: search center (x|y) / out: motion vector (x|y) - in sub-pel units
                                           MotionVector *mv2,       // <--> in: search center (x|y) 
                                           MEBlock *mv_block,       // <--  motion vector information
                                           int       search_range,  // <--  1-d search range in sub-pel units
                                           distblk     min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                           int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                                           )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;

  int   search_step;
  int   i, m;
  distblk mcost;
  short blocktype     = mv_block->blocktype;

  short pic_pix_x     = mv_block->pos_x_padded;
  short pic_pix_y     = mv_block->pos_y_padded;
  short ref = mv_block->ref_idx;

  StorablePicture *ref_picture1 = currSlice->listX[list + currMB->list_offset][ref];
  StorablePicture *ref_picture2 = currSlice->listX[list == 0 ? 1 + currMB->list_offset: currMB->list_offset][ 0 ];  

  MotionVector iMinNow, best, cand, pred1, pred2, center1, center2;
  search_range >>= 2;

  pred1.mv_x       = mv_block->pos_x_padded + pred_mv1->mv_x;  // predicted position x (in sub-pel units)
  pred1.mv_y       = mv_block->pos_y_padded + pred_mv1->mv_y;  // predicted position y (in sub-pel units)
  pred2.mv_x       = mv_block->pos_x_padded + pred_mv2->mv_x;  // predicted position x (in sub-pel units)
  pred2.mv_y       = mv_block->pos_y_padded + pred_mv2->mv_y;  // predicted position y (in sub-pel units)
  
  center2.mv_x     = mv_block->pos_x_padded + mv1->mv_x;            // center position x (in sub-pel units)
  center2.mv_y     = mv_block->pos_y_padded + mv1->mv_y;                   // center position y (in sub-pel units)
  center1.mv_x     = mv_block->pos_x_padded + mv2->mv_x;            // mvx of second pred (in sub-pel units)
  center1.mv_y     = mv_block->pos_y_padded + mv2->mv_y;            // mvy of second pred (in sub-pel units)

  //check the center median predictor
  best = cand = center2;
  mcost  = mv_cost (p_Vid, lambda_factor, &center1, &pred1);
  mcost += mv_cost (p_Vid, lambda_factor, &cand, &pred2);
  mcost += mv_block->computeBiPredFPel(ref_picture1, ref_picture2, mv_block,
                        DISTBLK_MAX, &center1, &cand);
  if (mcost < min_mcost)
  {
    min_mcost = mcost;
    best = cand;
  }

  iMinNow = best;
  if (0 != pred_mv1->mv_x || 0 != pred_mv1->mv_y || 0 != pred_mv2->mv_x || 0 != pred_mv2->mv_y)
  {
    cand.mv_x = pic_pix_x;
    cand.mv_y = pic_pix_y;
    SEARCH_ONE_PIXEL_BIPRED;
  }

  // If the min_mcost is small enough, do a local search then terminate
  // This is good for stationary or quasi-stationary areas
  if ((min_mcost<<3) < (p_UMHexSMP->ConvergeThreshold >> (block_type_shift_factor[blocktype])))
  {
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }

    mv1->mv_x = (short) (best.mv_x - pic_pix_x);
    mv1->mv_y = (short) (best.mv_y - pic_pix_y);
    return min_mcost;
  }

  // Small local search
  for (m = 0; m < 4; m++)
  {
    cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
    cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
    SEARCH_ONE_PIXEL_BIPRED;
  }

  // First_step: Symmetrical-cross search
  // If distortion is large, use large shapes. Otherwise, compact shapes are faster
  if ((blocktype == 1 &&
    (min_mcost<<2) > (p_UMHexSMP->SymmetricalCrossSearchThreshold1 >> block_type_shift_factor[blocktype])) ||
    ((min_mcost<<2) > (p_UMHexSMP->SymmetricalCrossSearchThreshold2>>block_type_shift_factor[blocktype])))
  {
    iMinNow.mv_x = best.mv_x;
    iMinNow.mv_y = best.mv_y;

    for (i = 8; i <= 4 * search_range; i+= 8)
    {
      search_step = i - 4;
      cand.mv_x = (short) (iMinNow.mv_x + search_step);
      cand.mv_y = iMinNow.mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
      cand.mv_x = (short) (iMinNow.mv_x - search_step);
      SEARCH_ONE_PIXEL_BIPRED;

      cand.mv_x = iMinNow.mv_x;
      cand.mv_y = (short) (iMinNow.mv_y + search_step);
      SEARCH_ONE_PIXEL_BIPRED;
      cand.mv_y = (short) (iMinNow.mv_y - search_step);
      SEARCH_ONE_PIXEL_BIPRED;
    }

    // Hexagon Search
    iMinNow.mv_x = best.mv_x;
    iMinNow.mv_y = best.mv_y;
    for (m = 0; m < 6; m++)
    {
      cand.mv_x = iMinNow.mv_x + Hexagon[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Hexagon[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }
    // Multi Big Hexagon Search
    iMinNow = best;
    for(i = 1; i <= search_range / 4; i++)
    {
      for (m = 0; m < 16; m++)
      {
        cand.mv_x = (short) (iMinNow.mv_x + Big_Hexagon_X[m] * i);
        cand.mv_y = (short) (iMinNow.mv_y + Big_Hexagon_Y[m] * i);
        SEARCH_ONE_PIXEL_BIPRED;
      }
    }
  }

  // Search up_layer predictor for non 16x16 blocks
  if (blocktype > 1)
  {
    cand.mv_x = pic_pix_x + (p_UMHexSMP->pred_MV_uplayer_X / 4) * 4;
    cand.mv_y = pic_pix_y + (p_UMHexSMP->pred_MV_uplayer_Y / 4) * 4;
    SEARCH_ONE_PIXEL_BIPRED;
  }

  if(center2.mv_x != pic_pix_x || center2.mv_x != pic_pix_y)
  {
    cand.mv_x = pic_pix_x;
    cand.mv_y = pic_pix_y;
    SEARCH_ONE_PIXEL_BIPRED;

    iMinNow = best;
    // Local diamond search
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }
  }

  // If the minimum cost is small enough, do a local search
  // and finish the search here
  if ((min_mcost<<2) < (p_UMHexSMP->ConvergeThreshold >> block_type_shift_factor[blocktype]))
  {
    iMinNow = best;
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }
    mv1->mv_x = (short) (best.mv_x - pic_pix_x);
    mv1->mv_y = (short) (best.mv_y - pic_pix_y);
    return min_mcost;
  }

  // Second_step:  Extended Hexagon-based Search
  for (i = 0; i < search_range; i++)
  {
    iMinNow = best;
    for (m = 0; m < 6; m++)
    {
      cand.mv_x = iMinNow.mv_x + Hexagon[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Hexagon[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }
    // The minimum cost point happens in the center
    if (best.mv_x == iMinNow.mv_x && best.mv_y == iMinNow.mv_y)
    {
      break;
    }
  }

  // Third_step: Small diamond search
  for (i = 0; i < search_range; i++)
  {
    iMinNow = best;
    for (m = 0; m < 4; m++)
    {
      cand.mv_x = iMinNow.mv_x + Diamond[m].mv_x;
      cand.mv_y = iMinNow.mv_y + Diamond[m].mv_y;
      SEARCH_ONE_PIXEL_BIPRED;
    }

    // The minimum cost point happens in the center
    if (best.mv_x == iMinNow.mv_x && best.mv_y == iMinNow.mv_y)
    {
      break;
    }
  }

  mv1->mv_x = (short) (best.mv_x - pic_pix_x);
  mv1->mv_y = (short) (best.mv_y - pic_pix_y);
  return min_mcost;
}

/*!
 ************************************************************************
 * \brief
 *    Set neighbouring block mode (intra/inter)
 *    used for fast motion estimation
 ************************************************************************
 */
void smpUMHEX_decide_intrabk_SAD(Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;

  if (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE)
  {
    if (currMB->pix_x == 0 && currMB->pix_y == 0)
    {
      p_UMHexSMP->flag_intra_SAD = 0;
    }
    else if (currMB->pix_x == 0)
    {
      p_UMHexSMP->flag_intra_SAD = p_UMHexSMP->flag_intra[(currMB->pix_x)>>4];
    }
    else if (currMB->pix_y == 0)
    {
      p_UMHexSMP->flag_intra_SAD = p_UMHexSMP->flag_intra[((currMB->pix_x)>>4)-1];
    }
    else
    {
      p_UMHexSMP->flag_intra_SAD = ((p_UMHexSMP->flag_intra[(currMB->pix_x)>>4])||
        (p_UMHexSMP->flag_intra[((currMB->pix_x)>>4)-1])||
        (p_UMHexSMP->flag_intra[((currMB->pix_x)>>4)+1])) ;
    }
  }
  return;
}

/*!
 ************************************************************************
 * \brief
 *    Set cost to zero if neighbouring block is intra
 *    used for fast motion estimation
 ************************************************************************
 */
void smpUMHEX_skip_intrabk_SAD(Macroblock *currMB)
{
  short i, j, k;
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;

  if (p_Vid->number > 0)
  {
    p_UMHexSMP->flag_intra[(currMB->pix_x)>>4] = (currMB->best_mode == 9 || currMB->best_mode == 10) ? 1 : 0;
  }

  if (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE && (currMB->best_mode == 9 || currMB->best_mode == 10))
  {
    for (k=0; k < 9;k++)
    {
      for (j=0; j < 4; j++)
      {
        for (i=0; i < 4; i++)
        {
          p_UMHexSMP->l0_cost[k][j][i] = 0;
          p_UMHexSMP->l1_cost[k][j][i] = 0;
        }
      }
    }
  }
  return;
}

/*!
 ************************************************************************
 * \brief
 *    Set up prediction MV and prediction up layer cost
 *    used for fast motion estimation
 ************************************************************************
 */
void smpUMHEX_setup(Macroblock *currMB,
                    short ref,
                    int list,
                    int block_y,
                    int block_x,
                    int blocktype,
                    MotionVector *****all_mv)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  UMHexSMPStruct *p_UMHexSMP = p_Vid->p_UMHexSMP;

  if (blocktype > 6)
  {
    p_UMHexSMP->pred_MV_uplayer_X = all_mv[list][ref][5][block_y][block_x].mv_x;
    p_UMHexSMP->pred_MV_uplayer_Y = all_mv[list][ref][5][block_y][block_x].mv_y;
  }
  else if (blocktype > 4)
  {
    p_UMHexSMP->pred_MV_uplayer_X = all_mv[list][ref][4][block_y][block_x].mv_x;
    p_UMHexSMP->pred_MV_uplayer_Y = all_mv[list][ref][4][block_y][block_x].mv_y;
  }
  else if (blocktype == 4)
  {
    p_UMHexSMP->pred_MV_uplayer_X = all_mv[list][ref][2][block_y][block_x].mv_x;
    p_UMHexSMP->pred_MV_uplayer_Y = all_mv[list][ref][2][block_y][block_x].mv_y;
  }
  else if (blocktype > 1)
  {
    p_UMHexSMP->pred_MV_uplayer_X = all_mv[list][ref][1][block_y][block_x].mv_x;
    p_UMHexSMP->pred_MV_uplayer_Y = all_mv[list][ref][1][block_y][block_x].mv_y;
  }

  if (blocktype > 1)
  {
    if (blocktype > 6)
    {
      p_UMHexSMP->pred_SAD_uplayer = (list==1) ?
        (p_UMHexSMP->l1_cost[5][(currMB->block_y)+block_y][(currMB->block_x)+block_x])
        : (p_UMHexSMP->l0_cost[5][(currMB->block_y)+block_y][(currMB->block_x)+block_x]);
      p_UMHexSMP->pred_SAD_uplayer /= 2;
    }
    else if (blocktype > 4)
    {
      p_UMHexSMP->pred_SAD_uplayer = (list==1) ?
        (p_UMHexSMP->l1_cost[4][(currMB->block_y)+block_y][(currMB->block_x)+block_x])
        : (p_UMHexSMP->l0_cost[4][(currMB->block_y)+block_y][(currMB->block_x)+block_x]);
      p_UMHexSMP->pred_SAD_uplayer /= 2;
    }
    else if (blocktype == 4)
    {
      p_UMHexSMP->pred_SAD_uplayer = (list==1) ?
        (p_UMHexSMP->l1_cost[2][(currMB->block_y)+block_y][(currMB->block_x)+block_x])
        : (p_UMHexSMP->l0_cost[2][(currMB->block_y)+block_y][(currMB->block_x)+block_x]);
      p_UMHexSMP->pred_SAD_uplayer /= 2;
    }
    else
    {
      p_UMHexSMP->pred_SAD_uplayer = (list==1) ?
        (p_UMHexSMP->l1_cost[1][(currMB->block_y)+block_y][(currMB->block_x)+block_x])
        : (p_UMHexSMP->l0_cost[1][(currMB->block_y)+block_y][(currMB->block_x)+block_x]);
      p_UMHexSMP->pred_SAD_uplayer /= 2;
    }

    p_UMHexSMP->pred_SAD_uplayer = p_UMHexSMP->flag_intra_SAD ? 0 : p_UMHexSMP->pred_SAD_uplayer;
  }
}

