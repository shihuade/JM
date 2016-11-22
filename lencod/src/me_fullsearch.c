
/*!
*************************************************************************************
* \file me_fullsearch.c
*
* \brief
*    Motion Estimation using Fullsearch
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*      - Alexis Michael Tourapis <alexismt@ieee.org>
*      - Athanasios Leontaris    <aleon@dolby.com>
*
*************************************************************************************
*/

// Includes
#include "contributors.h"
#include <limits.h>

#include "global.h"
#include "image.h"
#include "memalloc.h"
#include "mb_access.h"
#include "refbuf.h"
#include "macroblock.h"
#include "me_distortion.h"
#include "me_fullsearch.h"
#include "mv_search.h"

// Functions
/*!
 ***********************************************************************
 * \brief
 *    Full pixel block motion search
 ***********************************************************************
 */
distblk                                                //  ==> minimum motion cost after search
full_search_motion_estimation (Macroblock   *currMB ,       // <--  current Macroblock
                               MotionVector *pred_mv,       // <--  motion vector predictor in sub-pel units
                               MEBlock      *mv_block,      // <--  motion estimation structure
                               distblk       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                               int           lambda_factor  // <--  lagrangian parameter for determining motion cost
                              )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  int  search_range = imin(mv_block->searchRange.max_x, mv_block->searchRange.max_y)>> 2;
  distblk mcost;
  int   pos;
  MotionVector cand, center, pred;
  short ref = mv_block->ref_idx;

  StorablePicture *ref_picture = currSlice->listX[mv_block->list+currMB->list_offset][ref];

  int   best_pos      = 0;                                        // position with minimum motion cost
  int   max_pos       = (2*search_range+1)*(2*search_range+1);    // number of search positions

  MotionVector *mv    = &mv_block->mv[(short) mv_block->list];
  int   check_for_00  = (mv_block->blocktype==1 && !p_Inp->rdopt && currSlice->slice_type!=B_SLICE && ref==0);
  center.mv_x      = mv_block->pos_x_padded + mv->mv_x;                        // center position x (in sub-pel units)
  center.mv_y      = mv_block->pos_y_padded + mv->mv_y;                        // center position y (in sub-pel units)
  pred.mv_x        = mv_block->pos_x_padded + pred_mv->mv_x;       // predicted position x (in sub-pel units)
  pred.mv_y        = mv_block->pos_y_padded + pred_mv->mv_y;       // predicted position y (in sub-pel units)
  

  //===== loop over all search positions =====
  for (pos=0; pos<max_pos; pos++)
  {
    //--- set candidate position (absolute position in sub-pel units) ---
    cand.mv_x = center.mv_x + p_Vid->spiral_qpel_search[pos].mv_x;
    cand.mv_y = center.mv_y + p_Vid->spiral_qpel_search[pos].mv_y;

    //--- initialize motion cost (cost for motion vector) and check ---
    mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred);

    if (check_for_00 && cand.mv_x == mv_block->pos_x_padded && cand.mv_y == mv_block->pos_y_padded)
    {
      distblk tmp = weighted_cost (lambda_factor, 16);
      mcost = mcost > tmp? (mcost-tmp): 0;
    }
    if (mcost >= min_mcost)   continue;

    //--- add residual cost to motion cost ---
    mcost += mv_block->computePredFPel(ref_picture, mv_block, min_mcost - mcost, &cand);

    //--- check if motion cost is less than minimum cost ---
    if (mcost < min_mcost)
    {
      best_pos  = pos;
      min_mcost = mcost;
    }
  }


  //===== set best motion vector and return minimum motion cost =====
  if (best_pos)
  {
    add_mvs(mv, &p_Vid->spiral_qpel_search[best_pos]);
  }
  return min_mcost;
}

/*!
 ***********************************************************************
 * \brief
 *    Full pixel block motion search
 ***********************************************************************
 */
distblk                                            //  ==> minimum motion cost after search
full_search_bipred_motion_estimation (Macroblock *currMB,      // <--  current Macroblock
                          int       list,          // <--  reference list
                          MotionVector *pred_mv1,  // <--  motion vector predictor from first list (x|y) in sub-pel units
                          MotionVector *pred_mv2,  // <--  motion vector predictor from second list (x|y) in sub-pel units
                          MotionVector *mv1,       // <--> in: search center (x|y) / out: motion vector (x|y) - in sub-pel units
                          MotionVector *mv2,       // <--> in: search center (x|y) 
                          MEBlock *mv_block,       // <--  motion vector information
                          int       search_range,  // <--  1-d search range in sub-pel units
                          distblk     min_mcost,   // <--  minimum motion cost (cost for center or huge value)
                          int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                          ) 
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  short ref = mv_block->ref_idx;
  StorablePicture *ref_picture1 = currSlice->listX[list       + currMB->list_offset][ref];
  StorablePicture *ref_picture2 = currSlice->listX[(list ^ 1) + currMB->list_offset][0];
  distblk mcost;
  int   pos;

  int   best_pos      = 0;                                     // position with minimum motion cost
  int   max_pos       = (2 * (search_range >> 2) +1)*(2 * (search_range >> 2) + 1); // number of search positions

  MotionVector center1, center2, cand, pred1, pred2;
  pred1.mv_x   = mv_block->pos_x_padded + pred_mv1->mv_x; // predicted position x (in sub-pel units)
  pred1.mv_y   = mv_block->pos_y_padded + pred_mv1->mv_y; // predicted position y (in sub-pel units)
  pred2.mv_x   = mv_block->pos_x_padded + pred_mv2->mv_x; // predicted position x (in sub-pel units)
  pred2.mv_y   = mv_block->pos_y_padded + pred_mv2->mv_y; // predicted position y (in sub-pel units)
  center1.mv_x = mv_block->pos_x_padded + mv1->mv_x;              // center position x (in sub-pel units)
  center1.mv_y = mv_block->pos_y_padded + mv1->mv_y;              // center position y (in sub-pel units)
  center2.mv_x = mv_block->pos_x_padded + mv2->mv_x;       // center position x of static mv (in sub-pel units)
  center2.mv_y = mv_block->pos_y_padded + mv2->mv_y;       // center position y of static mv (in sub-pel units)


  //===== loop over all search positions =====
  for (pos=0; pos<max_pos; pos++)
  {
    //--- set candidate position (absolute position in sub-pel units) ---
    cand.mv_x = center1.mv_x + p_Vid->spiral_qpel_search[pos].mv_x;
    cand.mv_y = center1.mv_y + p_Vid->spiral_qpel_search[pos].mv_y;

    //--- initialize motion cost (cost for motion vector) and check ---
    mcost  = mv_cost (p_Vid, lambda_factor, &cand, &pred1);
    mcost += mv_cost (p_Vid, lambda_factor, &center2, &pred2); 

    if (mcost >= min_mcost)   continue;

    //--- add residual cost to motion cost ---
    mcost += mv_block->computeBiPredFPel(ref_picture1, ref_picture2, mv_block, min_mcost - mcost,
      &cand, &center2);

    //--- check if motion cost is less than minimum cost ---
    if (mcost < min_mcost)
    {
      best_pos  = pos;
      min_mcost = mcost;
    }
  }

  //===== set best motion vector and return minimum motion cost =====
  if (best_pos)
  {
    add_mvs(mv1, &p_Vid->spiral_qpel_search[best_pos]);
  }
  return min_mcost;
}

/*!
 ***********************************************************************
 * \brief
 *    Sub pixel block motion search
 ***********************************************************************
 */
distblk                                               //  ==> minimum motion cost after search
sub_pel_motion_estimation (Macroblock   *currMB,      // <--  Current Macroblock
                           MotionVector *pred,        // <--  motion vector predictor in sub-pel units
                           MEBlock      *mv_block,    // <--  motion estimation structure
                           distblk       min_mcost,   // <--  minimum motion cost (cost for center or huge value)
                           int*          lambda       // <--  lagrangian parameter for determining motion cost                         
                           )
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  distblk mcost;
  int   pos, best_pos;

  MotionVector cand;
  int list = mv_block->list;
  int cur_list = list + currMB->list_offset;
  short ref = mv_block->ref_idx;
  StorablePicture *ref_picture = currSlice->listX[cur_list][ref];

  MotionVector *mv  = &mv_block->mv[list];

  int   check_position0 = (!p_Inp->rdopt && currSlice->slice_type != B_SLICE && ref==0 && mv_block->blocktype==1 && mv->mv_x == 0 && mv->mv_y ==0);

  int   max_pos2        = ( !p_Vid->start_me_refinement_hp ? imax(1, mv_block->search_pos2) : mv_block->search_pos2);  
  MotionVector cmv;

  int lambda_factor = lambda[H_PEL];

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
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred);


    if (mcost >= min_mcost) continue;

    cmv = pad_MVs(cand, mv_block);

    mcost += mv_block->computePredHPel( ref_picture, mv_block, min_mcost - mcost, &cmv);

    if (pos==0 && check_position0)
    {
      mcost -= weighted_cost (lambda_factor, 16);
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    mv->mv_x = mv->mv_x + p_Vid->spiral_hpel_search[best_pos].mv_x;
    mv->mv_y = mv->mv_y + p_Vid->spiral_hpel_search[best_pos].mv_y;
  }
  if ( !p_Vid->start_me_refinement_qp )
    min_mcost = DISTBLK_MAX;
  /************************************
  *****                          *****
  *****  QUARTER-PEL REFINEMENT  *****
  *****                          *****
  ************************************/

  lambda_factor = lambda[Q_PEL];

  //===== loop over search positions =====
  for (best_pos = 0, pos = p_Vid->start_me_refinement_qp; pos < mv_block->search_pos4; pos++)
  {
    cand.mv_x = mv->mv_x + p_Vid->spiral_search[pos].mv_x;    // quarter-pel units
    cand.mv_y = mv->mv_y + p_Vid->spiral_search[pos].mv_y;    // quarter-pel units

    //----- set motion vector cost -----
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred);

    if (mcost >= min_mcost) continue;

    cmv = pad_MVs(cand, mv_block);

    mcost += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cmv);
    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
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
***********************************************************************
* \brief
*    Bipred Sub pixel block motion search
***********************************************************************
*/
distblk                                           //  ==> minimum motion cost after search
sub_pel_bipred_motion_estimation (Macroblock   *currMB,    // <--  current Macroblock
                         MEBlock      *mv_block,  // <--  motion vector information
                         int           list,      // <--  reference picture list
                         MotionVector *pred_mv1,  // <--  motion vector predictor (x) in sub-pel units
                         MotionVector *pred_mv2,  // <--  motion vector predictor (x) in sub-pel units
                         MotionVector *mv1,       // <--> in: search center (x) / out: motion vector (x) - in sub-pel units
                         MotionVector *mv2,       // <--> in: search center (x) / out: motion vector (x) - in sub-pel units
                         distblk       min_mcost, // <--  minimum motion cost (cost for center or huge value)
                         int*          lambda     // <--  lagrangian parameter for determining motion cost
                         )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice; 
  int   list_offset   = p_Vid->mb_data[currMB->mbAddrX].list_offset;
  distblk mcost;
  int   pos, best_pos;
  MotionVector cand;
  int   start_hp    = (min_mcost == DISTBLK_MAX) ? 0 : p_Vid->start_me_refinement_hp;
  int   max_pos2        = ( !p_Vid->start_me_refinement_hp ? imax(1, mv_block->search_pos2) : mv_block->search_pos2);

  MotionVector cmv, smv = pad_MVs(*mv2, mv_block);
  short ref = mv_block->ref_idx;

  StorablePicture *ref_picture1 = currSlice->listX[list       + list_offset][ref];
  StorablePicture *ref_picture2 = currSlice->listX[(list ^ 1) + list_offset][0];

  int lambda_factor = lambda[H_PEL];

  /*********************************
   *****                       *****
   *****  HALF-PEL REFINEMENT  *****
   *****                       *****
   *********************************/

  //===== loop over search positions =====
  for (best_pos = 0, pos = start_hp; pos < max_pos2; pos++)
  {
    cand.mv_x = mv1->mv_x + p_Vid->spiral_hpel_search[pos].mv_x;    // quarter-pel units
    cand.mv_y = mv1->mv_y + p_Vid->spiral_hpel_search[pos].mv_y;    // quarter-pel units

    //----- set motion vector cost -----
    mcost  = mv_cost (p_Vid, lambda_factor, &cand, pred_mv1);
    mcost += mv_cost (p_Vid, lambda_factor,   mv2, pred_mv2);

    if (mcost >= min_mcost) continue;

    cmv = pad_MVs(cand, mv_block);
    mcost += mv_block->computeBiPredHPel(ref_picture1, ref_picture2, mv_block, min_mcost - mcost, &cmv, &smv);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }

  if (best_pos)
  {
    add_mvs(mv1, &p_Vid->spiral_hpel_search[best_pos]);
  }

  /************************************
  *****                          *****
  *****  QUARTER-PEL REFINEMENT  *****
  *****                          *****
  ************************************/
  if ( !p_Vid->start_me_refinement_qp )
    min_mcost = DISTBLK_MAX;

  lambda_factor = lambda[Q_PEL];

  //===== loop over search positions =====
  for (best_pos = 0, pos = p_Vid->start_me_refinement_qp; pos < mv_block->search_pos4; pos++)
  {
    cand.mv_x = mv1->mv_x + p_Vid->spiral_search[pos].mv_x;    // quarter-pel units
    cand.mv_y = mv1->mv_y + p_Vid->spiral_search[pos].mv_y;    // quarter-pel units

    //----- set motion vector cost -----
    mcost  = mv_cost (p_Vid, lambda_factor, &cand, pred_mv1);
    mcost += mv_cost (p_Vid, lambda_factor,   mv2, pred_mv2);

    if (mcost >= min_mcost) continue;
    
    cmv = pad_MVs(cand, mv_block);
    mcost += mv_block->computeBiPredQPel(ref_picture1, ref_picture2, mv_block, min_mcost - mcost, &cmv, &smv);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }

  }

  if (best_pos)
  {
    add_mvs(mv1, &p_Vid->spiral_search[best_pos]);
  }

  //===== return minimum motion cost =====
  return min_mcost;
}

/*!
 ***********************************************************************
 * \brief
 *    Brute Force Sub pixel block motion search
 ***********************************************************************
 */
distblk                                                  //  ==> minimum motion cost after search
full_sub_pel_motion_estimation (Macroblock   *currMB,      // <--  Current Macroblock
                              MotionVector *pred,        // <--  motion vector predictor in sub-pel units
                              MEBlock      *mv_block,    // <--  motion estimation structure
                              distblk       min_mcost,   // <--  minimum motion cost (cost for center or huge value)
                              int*          lambda       // <--  lagrangian parameter for determining motion cost                         
                              )
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  distblk mcost;
  int   pos, best_pos;

  MotionVector cand;
  int list = mv_block->list;
  int cur_list = list + currMB->list_offset;
  short ref = mv_block->ref_idx;
  StorablePicture *ref_picture = currSlice->listX[cur_list][ref];

  MotionVector *mv  = &mv_block->mv[list];

  int   check_position0 = (!p_Inp->rdopt && currSlice->slice_type != B_SLICE && ref==0 && mv_block->blocktype==1 && mv->mv_x == 0 && mv->mv_y ==0);

  MotionVector cmv;

  int lambda_factor = lambda[Q_PEL];
  // reset distortion. We are always checking all positions to ensure
  // distortion computation is done correctly.
  min_mcost = DISTBLK_MAX;

  //===== loop over all search positions =====
  for (best_pos = 0, pos = 0; pos < 49; pos++)
  {
    cand.mv_x = mv->mv_x + p_Vid->spiral_search[pos].mv_x;   
    cand.mv_y = mv->mv_y + p_Vid->spiral_search[pos].mv_y;

    //----- set motion vector cost -----
    mcost = mv_cost (p_Vid, lambda_factor, &cand, pred);

    if (mcost >= min_mcost) continue;

    cmv = pad_MVs(cand, mv_block);
    mcost += mv_block->computePredQPel( ref_picture, mv_block, min_mcost - mcost, &cmv);

    if (pos == 0 && check_position0)
    {
      mcost -= weighted_cost (lambda_factor, 16);
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
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
***********************************************************************
* \brief
*    Brute Force Bipred Sub pixel block motion search
***********************************************************************
*/
distblk                                           //  ==> minimum motion cost after search
full_sub_pel_bipred_motion_estimation (Macroblock   *currMB,    // <--  current Macroblock
                         MEBlock      *mv_block,  // <--  motion vector information
                         int           list,      // <--  reference picture list
                         MotionVector *pred_mv1,  // <--  motion vector predictor (x) in sub-pel units
                         MotionVector *pred_mv2,  // <--  motion vector predictor (x) in sub-pel units
                         MotionVector *mv1,       // <--> in: search center (x) / out: motion vector (x) - in pel units
                         MotionVector *mv2,       // <--> in: search center (x) / out: motion vector (x) - in pel units
                         distblk       min_mcost, // <--  minimum motion cost (cost for center or huge value)
                         int*          lambda     // <--  lagrangian parameter for determining motion cost
                         )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice; 
  int   list_offset   = p_Vid->mb_data[currMB->mbAddrX].list_offset;
  distblk mcost;
  int   pos, best_pos;
  MotionVector cand;

  MotionVector cmv, smv = pad_MVs(*mv2, mv_block);
  short ref = mv_block->ref_idx;

  StorablePicture *ref_picture1 = currSlice->listX[list       + list_offset][ref];
  StorablePicture *ref_picture2 = currSlice->listX[(list ^ 1) + list_offset][0];

  int lambda_factor = lambda[Q_PEL];
  
  // reset distortion. We are always checking all positions to ensure
  // distortion computation is done correctly.

  //===== loop over search positions =====
  for (best_pos = 0, pos = 0; pos < 49; pos++)
  {
    cand.mv_x = mv1->mv_x + p_Vid->spiral_search[pos].mv_x;    // quarter-pel units
    cand.mv_y = mv1->mv_y + p_Vid->spiral_search[pos].mv_y;    // quarter-pel units

    //----- set motion vector cost -----
    mcost  = mv_cost (p_Vid, lambda_factor, &cand, pred_mv1);
    mcost += mv_cost (p_Vid, lambda_factor,   mv2, pred_mv2);

    if (mcost >= min_mcost) continue;

    cmv = pad_MVs(cand, mv_block);
    mcost += mv_block->computeBiPredQPel(ref_picture1, ref_picture2, mv_block, min_mcost - mcost, &cmv, &smv);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }

  if (best_pos)
  {
    add_mvs(mv1, &p_Vid->spiral_search[best_pos]);
  }

  //===== return minimum motion cost =====
  return min_mcost;
}

