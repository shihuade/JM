
/*!
************************************************************************
* \file
*     me_epzs.h
*
* \author
*    Alexis Michael Tourapis        <alexis.tourapis@dolby.com>
*
* \date
*    11. August 2006
*
* \brief
*    Headerfile for EPZS motion estimation
**************************************************************************
*/


#ifndef _ME_EPZS_H_
#define _ME_EPZS_H_
#include "me_epzs_common.h"

static const int next_start_pos[5][5] =
{
  {0,  8, 5,  6, 7},
  {8,  0, 5,  8, 8}, 
  {5,  5, 0,  6, 5},
  {6,  6, 6,  0, 7},
  {7,  8, 7,  7, 0}
};

static const int next_end_pos[5][5] =
{
  {0, 10,  7,  8, 9},
  {10, 0,  6, 10, 9},
  {7,  6,  0,  7, 7},
  {8,  8,  7,  0, 8},
  {9,  9,  9,  8, 0}
};

static const MotionVector search_point_hp[10] = {{0,0},{-2,0}, {0,2}, {2,0},  {0,-2}, {-2,2},  {2,2},  {2,-2}, {-2,-2}, {-2,2}};
static const MotionVector search_point_qp[10] = {{0,0},{-1,0}, {0,1}, {1,0},  {0,-1}, {-1,1},  {1,1},  {1,-1}, {-1,-1}, {-1,1}};

// Functions
extern distblk EPZS_motion_estimation                (Macroblock *, MotionVector *, MEBlock *, distblk, int);
extern distblk EPZS_subMB_motion_estimation          (Macroblock *, MotionVector *, MEBlock *, distblk, int);
extern distblk EPZS_sub_pel_motion_estimation        (Macroblock *, MotionVector *, MEBlock *, distblk, int*);
extern distblk EPZS_sub_pel_bipred_motion_estimation (Macroblock *, MEBlock *, int list, MotionVector *, MotionVector *, MotionVector *, MotionVector *, distblk, int *);
extern distblk EPZS_bipred_motion_estimation         (Macroblock *, int, MotionVector *, MotionVector *, MotionVector *, MotionVector *, MEBlock *,  int, distblk, int);


#endif

