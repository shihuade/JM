
/*!
 ************************************************************************
 *
 * \file me_umhex.h
 *
 * \brief
 *   Macro definitions and global variables for UMHEX fast
 *   integer pel motion estimation and fractional pel motion estimation
 *
 * \author
 *   Main contributors: (see contributors.h for copyright, address and affiliation details)
 *    - Zhibo Chen         <chenzhibo@tsinghua.org.cn>
 *    - JianFeng Xu        <fenax@video.mdc.tsinghua.edu.cn>
 *    - Wenfang Fu         <fwf@video.mdc.tsinghua.edu.cn>
 *    - Xiaozhong Xu       <xxz@video.mdc.tsinghua.edu.cn>
 *
 * \date
 *   2006.1
 ************************************************************************
 */

#ifndef _ME_UMHEX_H_
#define _ME_UMHEX_H_

#include "mbuffer.h"

struct umhex_struct {
  float AlphaFourth_1[8];
  float AlphaFourth_2[8];
  // for bipred mode
  int pred_MV_ref_flag;
  int BlockType_LUT[4][4];
  distblk Median_Pred_Thd_MB[8];
  distblk Big_Hexagon_Thd_MB[8];
  distblk Multi_Ref_Thd_MB[8];
  distblk Threshold_DSR_MB[8];                    //!<  Threshold for usage of DSR. DSR refer to JVT-Q088
  byte **McostState;                          //!< state for integer pel search
  byte **SearchState;                         //!< state for fractional pel search
  distblk ****fastme_ref_cost;                    //!< store SAD information needed for forward ref-frame prediction
  distblk ***fastme_l0_cost;                      //!< store SAD information needed for forward median and uplayer prediction
  distblk ***fastme_l1_cost;                      //!< store SAD information needed for backward median and uplayer prediction
  distblk **fastme_best_cost;                     //!< for multi ref early termination threshold
  distblk ***fastme_l0_cost_bipred;               //!< store SAD information for bipred mode
  distblk ***fastme_l1_cost_bipred;               //!< store SAD information for bipred mode
  distblk pred_SAD;                               //!<  SAD prediction in use.
  distblk SAD_a,SAD_b,SAD_c,SAD_d;
  int bipred_flag;                            //!< flag for bipred
  int pred_MV_ref[2], pred_MV_uplayer[2];     //!< pred motion vector by space or temporal correlation,Median is provided

  int UMHEX_blocktype;                        //!< blocktype for UMHEX SetMotionVectorPredictor
  int predict_point[5][2];
  //for early termination
  float  Bsize[8];

  byte *flag_intra;
  int  flag_intra_SAD;
};

typedef struct umhex_struct UMHexStruct;

#define EARLY_TERMINATION                                                             \
  if ((min_mcost - p_UMHex->pred_SAD)<p_UMHex->pred_SAD * betaFourth_2)                                     \
  goto fourth_2_step;                                                                 \
  else if((min_mcost - p_UMHex->pred_SAD) < p_UMHex->pred_SAD * betaFourth_1)                                 \
  goto fourth_1_step;

#define SEARCH_ONE_PIXEL                                                              \
  if((iabs(cand.mv_x - center.mv_x)>>2) < search_range && (iabs(cand.mv_y - center.mv_y)>>2)< search_range)\
  {                                                                                   \
    if(!p_UMHex->McostState[((cand.mv_y - center.mv_y) >> 2)+ search_range][((cand.mv_x-center.mv_x)>>2)+search_range])       \
    {                                                                                 \
      mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred);                           \
      if(mcost<min_mcost)                                                             \
      {                                                                               \
        mcost += mv_block->computePredFPel(ref_picture, mv_block,                     \
        min_mcost - mcost, &cand);                                     \
        p_UMHex->McostState[((cand.mv_y - center.mv_y) >> 2) + search_range][((cand.mv_x - center.mv_x) >> 2) + search_range] = 1;   \
        if (mcost < min_mcost)                                                        \
        {                                                                             \
          best = cand;                                                                \
          min_mcost = mcost;                                                          \
        }                                                                             \
      }                                                                               \
    }                                                                                 \
   }

#define SEARCH_ONE_PIXEL_BIPRED                                                                         \
if((iabs(cand.mv_x - center2.mv_x) >> 2) < search_range && (iabs(cand.mv_y - center2.mv_y) >> 2) < search_range)     \
{                                                                                                       \
  if(!p_UMHex->McostState[((cand.mv_y - center2.mv_y) >> 2) + search_range][((cand.mv_x-center2.mv_x) >> 2)+search_range])         \
  {                                                                                                     \
    mcost  = mv_cost (p_Vid, lambda_factor, &center1, &pred1);                                          \
    mcost += mv_cost (p_Vid, lambda_factor, &cand, &pred2);                                             \
    if(mcost<min_mcost)                                                                                   \
    {                                                                                                     \
      mcost  += mv_block->computeBiPredFPel(ref_picture1, ref_picture2,                                 \
      mv_block, min_mcost - mcost, &center1, &cand);                                                \
      p_UMHex->McostState[((cand.mv_y - center2.mv_y) >> 2) + search_range][((cand.mv_x - center2.mv_x) >> 2) + search_range] = 1; \
      if (mcost < min_mcost)                                                                            \
      {                                                                                                 \
        best = cand;                                                                                    \
        min_mcost = mcost;                                                                              \
      }                                                                                                 \
    }                                                                                                   \
  }                                                                                                     \
}                                                                                                       \


extern void UMHEX_DefineThreshold  (VideoParameters *p_Vid);
extern void UMHEX_DefineThresholdMB(VideoParameters *p_Vid, InputParameters *p_Inp);
extern int  UMHEX_get_mem          (VideoParameters *p_Vid, InputParameters *p_Inp);
extern void UMHEX_free_mem         (VideoParameters *p_Vid, InputParameters *p_Inp);

extern void UMHEX_decide_intrabk_SAD(Macroblock *currMB);
extern void UMHEX_skip_intrabk_SAD  (Macroblock *currMB, int ref_max);
extern void UMHEX_setup             (Macroblock *currMB, short ref, int list, int block_y, int block_x, int blocktype, MotionVector  *****all_mv);

extern distblk                                     //  ==> minimum motion cost after search
UMHEXIntegerPelBlockMotionSearch  (Macroblock *currMB,     // <--  current Macroblock
                                  MotionVector *pred,      // <--  motion vector predictor (x) in sub-pel units
                                  MEBlock   *mv_block,
                                  distblk       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                  int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                                  );
extern distblk UMHEXSubPelBlockME ( //  ==> minimum motion cost after search
                             Macroblock *currMB,      // <--  Current Macroblock
                             MotionVector *pred,      // <--  motion vector predictor (x) in sub-pel units
                             MEBlock *mv_block, 
                             distblk     min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                             int*      lambda         // <--  lagrangian parameter for determining motion cost
                             );
extern distblk UMHEXSubPelBlockMotionSearch (                         //  ==> minimum motion cost after search
                                         Macroblock *currMB,     // <--  current Macroblock
                                         MotionVector *pred_mv,    // < <--  motion vector predictor (x|y) in sub-pel units
                                         MEBlock *mv_block,
                                         distblk     min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                         int       lambda_factor  // <--  lagrangian parameter for determining motion cost
                                         );
extern distblk UMHEXBipredIntegerPelBlockMotionSearch (Macroblock *, int, 
                                       MotionVector *, MotionVector *, MotionVector *, MotionVector *, 
                                       MEBlock *, int, distblk, int);

extern void UMHEXSetMotionVectorPredictor (Macroblock *currMB, MotionVector *pmv, struct pic_motion_params **mv_info,
                                    short  ref_frame, int list, int mb_x, int mb_y, int bl_x, int bl_y, MEBlock *mv_block);

#endif
