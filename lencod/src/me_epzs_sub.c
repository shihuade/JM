
/*!
*************************************************************************************
* \file me_epzs_sub.c
*
* \brief
*    SubPel Motion Estimation refinement using EPZS
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*      - Alexis Michael Tourapis <alexismt@ieee.org>
*
*************************************************************************************
*/ 

#include "contributors.h"

#include "global.h"
#include "refbuf.h"
#include "me_epzs.h"
#include "mv_search.h"

/*!
***********************************************************************
* \brief
*    Fast sub pixel block motion search to support EPZS
***********************************************************************
*/
distblk                                                   //  ==> minimum motion cost after search
EPZS_sub_pel_motion_estimation ( Macroblock *currMB,      // <--  current Macroblock
                                 MotionVector *pred,      // <--  motion vector predictor in sub-pel units
                                 MEBlock  *mv_block,      // <--  motion vector information
                                 distblk   min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                 int*      lambda         // <--  lagrangian parameter for determining motion cost
                                )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  EPZSParameters *p_EPZS = currSlice->p_EPZS;
  int   pos, best_pos = 0, second_pos = 0;
  distblk mcost;
  distblk second_mcost = DISTBLK_MAX;
  int   max_pos2     = ( (!p_Vid->start_me_refinement_hp || !p_Vid->start_me_refinement_qp) ? imax(1, mv_block->search_pos2) : mv_block->search_pos2);

  int list = mv_block->list;
  int cur_list = list + currMB->list_offset;
  short ref = mv_block->ref_idx;
  StorablePicture *ref_picture = currSlice->listX[cur_list][ref];
  MotionVector *mv  = &mv_block->mv[list];
  MotionVector cand;
  MotionVector padded_mv = pad_MVs (*mv,   mv_block);
  MotionVector pred_mv   = pad_MVs (*pred, mv_block);

  int start_pos = 5, end_pos = max_pos2;
  int lambda_factor = lambda[H_PEL];
  distblk lambda_dist   = weighted_cost(lambda_factor, 2);
  distblk sub_threshold = p_EPZS->subthres[mv_block->blocktype] + lambda_dist;

  /*********************************
  *****                       *****
  *****  HALF-PEL REFINEMENT  *****
  *****                       *****
  *********************************/

  //===== loop over search positions =====
  for (best_pos = 0, pos = p_Vid->start_me_refinement_hp; pos < imin(5, max_pos2); ++pos)
  {
    cand = add_MVs(search_point_hp[pos], &padded_mv);

    //----- set motion vector cost -----
    mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred_mv);        
    if (mcost < second_mcost)
    {
      mcost += mv_block->computePredHPel(ref_picture, mv_block, second_mcost - mcost, &cand);

      if (mcost < min_mcost)
      {
        second_mcost = min_mcost;
        second_pos  = best_pos;
        min_mcost = mcost;
        best_pos  = pos;
      }
      else if (mcost < second_mcost)
      {
        second_mcost = mcost;
        second_pos  = pos;
      }
    }
  }

  if (best_pos ==0 && (pred->mv_x == mv->mv_x) && (pred->mv_y == mv->mv_y) && min_mcost < sub_threshold)
  {
    return min_mcost;
  }

  if(mv_block->search_pos2 >= 9)
  {
    if (best_pos !=0 || (iabs(pred->mv_x - mv->mv_x) + iabs(pred->mv_y - mv->mv_y)))
    {
      start_pos = next_start_pos[best_pos][second_pos];
      end_pos   = next_end_pos[best_pos][second_pos];      
      
      for (pos = start_pos; pos < end_pos; ++pos)
      {
        cand = add_MVs(search_point_hp[pos], &padded_mv);

        //----- set motion vector cost -----
        mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred_mv);

        if (mcost < min_mcost)
        {
          mcost += mv_block->computePredHPel( ref_picture, mv_block, min_mcost - mcost, &cand);

          if (mcost < min_mcost)
          {
            min_mcost = mcost;
            best_pos  = pos;
          }
        }
      }
    }
  }

  if (best_pos)
  {
    add_mvs(mv, &search_point_hp[best_pos]);
    padded_mv = pad_MVs (*mv, mv_block);
  }

  /************************************
  *****                          *****
  *****  QUARTER-PEL REFINEMENT  *****
  *****                          *****
  ************************************/
  end_pos = (min_mcost < sub_threshold) ? 1 : 5;
  second_mcost = DISTBLK_MAX;

  if ( !p_Vid->start_me_refinement_qp )
  {
    best_pos = -1;
    min_mcost = DISTBLK_MAX;   
  }
  else
  {
    best_pos = 0;
  }
  lambda_factor = lambda[Q_PEL];      
  
  //===== loop over search positions =====
  for (pos = p_Vid->start_me_refinement_qp; pos < end_pos; ++pos)
  {    
    cand = add_MVs(search_point_qp[pos], &padded_mv);

    //----- set motion vector cost -----
    mcost  = mv_cost (p_Vid, lambda_factor, &cand, &pred_mv);

    if (mcost < second_mcost)
    {
      mcost += mv_block->computePredQPel(ref_picture, mv_block, second_mcost - mcost, &cand);

      if (mcost < min_mcost)
      {
        second_mcost = min_mcost;
        second_pos  = best_pos;
        min_mcost = mcost;
        best_pos  = pos;
      }
      else if (mcost < second_mcost)
      {
        second_mcost = mcost;
        second_pos  = pos;
      }
    }
  }

  //if (best_pos ==0 && (pred->mv_x == mv->mv_x) && (pred->mv_y == mv->mv_y) && min_mcost < sub_threshold)
  if ( (min_mcost > sub_threshold))
  {

    if (best_pos !=0 || (iabs(pred->mv_x - mv->mv_x) + iabs(pred->mv_y - mv->mv_y)))
    {
      start_pos = next_start_pos[best_pos][second_pos];
      end_pos   = next_end_pos[best_pos][second_pos];      

      for (pos = start_pos; pos < end_pos; ++pos)
      {
        cand = add_MVs(search_point_qp[pos], &padded_mv);

        //----- set motion vector cost -----
        mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred_mv);

        if (mcost < min_mcost) 
        {
          mcost += mv_block->computePredQPel(ref_picture, mv_block, min_mcost - mcost, &cand);

          if (mcost < min_mcost)
          {
            min_mcost = mcost;
            best_pos  = pos;
          }
        }
      }
    }
  }

  if (best_pos)
  {
    add_mvs(mv, &search_point_qp[best_pos]);
  }

  //===== return minimum motion cost =====
  return min_mcost;
}


/*!
***********************************************************************
* \brief
*    Fast bipred sub pixel block motion search to support EPZS
***********************************************************************
*/
distblk                                               //  ==> minimum motion cost after search
EPZS_sub_pel_bipred_motion_estimation (Macroblock *currMB,      // <--  current Macroblock
                             MEBlock *mv_block,       // <--  motion vector information
                             int       list,          // <--  reference picture list
                             MotionVector *pred1,     // <--  motion vector predictor in sub-pel units
                             MotionVector *pred2,     // <--  motion vector predictor in sub-pel units
                             MotionVector *mv1,       // <--> in: search center  / out: motion vector - in sub-pel units
                             MotionVector *mv2,       // <--> in: search center / out: motion vector - in sun-pel units
                             distblk     min_mcost,   // <--  minimum motion cost (cost for center or huge value)
                             int*      lambda         // <--  lagrangian parameter for determining motion cost
                             )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  EPZSParameters *p_EPZS = currSlice->p_EPZS;

  int   list_offset   = p_Vid->mb_data[currMB->mbAddrX].list_offset;

  int   pos, best_pos = 0, second_pos = 0;
  distblk mcost;
  distblk second_mcost = DISTBLK_MAX;


  int   start_hp    = (min_mcost == DISTBLK_MAX) ? 0 : p_Vid->start_me_refinement_hp;
  int   max_pos2    = ( (!p_Vid->start_me_refinement_hp || !p_Vid->start_me_refinement_qp) ? imax(1, mv_block->search_pos2) : mv_block->search_pos2);  

  short ref = mv_block->ref_idx;
  StorablePicture *ref_picture1 = currSlice->listX[list       + list_offset][ref];
  StorablePicture *ref_picture2 = currSlice->listX[(list ^ 1) + list_offset][0];
  MotionVector cand;
  MotionVector padded_mv1 = pad_MVs (*mv1,   mv_block);
  MotionVector padded_mv2 = pad_MVs (*mv2,   mv_block);
  MotionVector pred_mv1   = pad_MVs (*pred1, mv_block);
  MotionVector pred_mv2   = pad_MVs (*pred2, mv_block);

  int start_pos = 5, end_pos = max_pos2;
  int lambda_factor = lambda[H_PEL];
  distblk lambda_dist   = weighted_cost(lambda_factor, 2);
  distblk sub_threshold = p_EPZS->subthres[mv_block->blocktype] + lambda_dist;

  distblk mcost2 = mv_cost (p_Vid, lambda_factor, &padded_mv2, &pred_mv2);  
  min_mcost -= mcost2;

  /*********************************
  *****                       *****
  *****  HALF-PEL REFINEMENT  *****
  *****                       *****
  *********************************/

  //===== loop over search positions =====
  for (best_pos = 0, pos = start_hp; pos < 5; ++pos)
  {
    cand = add_MVs(search_point_hp[pos], &padded_mv1);

    //----- set motion vector cost -----
    mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred_mv1);

    if (mcost < second_mcost)
    {
      mcost += mv_block->computeBiPredHPel(ref_picture1, ref_picture2, mv_block, second_mcost - mcost, &cand, &padded_mv2);

      if (mcost < min_mcost)
      {
        second_mcost = min_mcost;
        second_pos  = best_pos;
        min_mcost = mcost;
        best_pos  = pos;
      }
      else if (mcost < second_mcost)
      {
        second_mcost = mcost;
        second_pos  = pos;
      }
    }
  }

  if (best_pos ==0 && (pred1->mv_x == mv1->mv_x) && (pred1->mv_y == mv1->mv_y) && min_mcost < sub_threshold)
  {
    return min_mcost;
  }

  if(mv_block->search_pos2 >= 9)
  {
  if (best_pos !=0 || (iabs(pred1->mv_x - mv1->mv_x) + iabs(pred1->mv_y - mv1->mv_y)))
  {
    start_pos = next_start_pos[best_pos][second_pos];
    end_pos   = next_end_pos[best_pos][second_pos];          

    for (pos = start_pos; pos < end_pos; ++pos)
    {
      cand = add_MVs(search_point_hp[pos], &padded_mv1);

      //----- set motion vector cost -----
      mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred_mv1);

      if (mcost < min_mcost)
      {
        mcost += mv_block->computeBiPredHPel(ref_picture1, ref_picture2, mv_block, min_mcost - mcost, &cand, &padded_mv2);

        if (mcost < min_mcost)
        {
          min_mcost = mcost;
          best_pos  = pos;
        }
      }
    }
  }
  }

  if (best_pos)
  {
    add_mvs (mv1, &search_point_hp[best_pos]);
    padded_mv1 = pad_MVs (*mv1, mv_block);
  }

  /************************************
  *****                          *****
  *****  QUARTER-PEL REFINEMENT  *****
  *****                          *****
  ************************************/
  end_pos = (min_mcost < sub_threshold) ? 1 : 5;
  second_mcost = DISTBLK_MAX;

  if ( !p_Vid->start_me_refinement_qp )
  {
    best_pos = -1;
    min_mcost = DISTBLK_MAX;
  }
  else 
  {
    best_pos = 0;
    min_mcost += mcost2;
  }
  lambda_factor = lambda[Q_PEL];      
  mcost2 = mv_cost (p_Vid, lambda_factor, &padded_mv2, &pred_mv2);
  min_mcost -= mcost2;

  //===== loop over search positions =====
  for (pos = p_Vid->start_me_refinement_qp; pos < 5; ++pos)
  {
    cand = add_MVs(search_point_qp[pos], &padded_mv1);

    //----- set motion vector cost -----
    mcost  = mv_cost (p_Vid, lambda_factor, &cand, &pred_mv1);

    if (mcost < second_mcost)
    {
      mcost += mv_block->computeBiPredQPel(ref_picture1, ref_picture2, mv_block, second_mcost - mcost, &cand, &padded_mv2);

      if (mcost < min_mcost)
      {
        second_mcost = min_mcost;
        second_pos  = best_pos;
        min_mcost = mcost;
        best_pos  = pos;
      }
      else if (mcost < second_mcost)
      {
        second_mcost = mcost;
        second_pos  = pos;
      }
    }
  }

  if ( (min_mcost > sub_threshold))
  {

    if (best_pos !=0 || (iabs(pred1->mv_x - mv1->mv_x) + iabs(pred1->mv_y - mv1->mv_y)))
    {
      start_pos = next_start_pos[best_pos][second_pos];
      end_pos   = next_end_pos[best_pos][second_pos];      
      
      for (pos = start_pos; pos < end_pos; ++pos)
      {
        cand = add_MVs(search_point_qp[pos], &padded_mv1);

        //----- set motion vector cost -----
        mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred_mv1);

        if (mcost < min_mcost)
        {
          mcost += mv_block->computeBiPredQPel(ref_picture1, ref_picture2, mv_block, min_mcost - mcost, &cand, &padded_mv2);

          if (mcost < min_mcost)
          {
            min_mcost = mcost;
            best_pos  = pos;
          }
        }
      }
    }
  }

  if (best_pos)
  {
    add_mvs(mv1, &search_point_qp[best_pos]);
  }

  //===== return minimum motion cost =====
  return min_mcost + mcost2;
}

