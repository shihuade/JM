
/*!
 ************************************************************************
 * \file
 *     me_epzs_int.h
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


#ifndef _ME_EPZS_INT_H_
#define _ME_EPZS_INT_H_

#include "me_epzs.h"

// Functions
extern distblk  EPZS_integer_motion_estimation        (Macroblock *, MotionVector *, MEBlock *, distblk, int);
extern distblk  EPZS_integer_subMB_motion_estimation  (Macroblock *, MotionVector *, MEBlock *, distblk, int);
extern distblk  EPZS_integer_bipred_motion_estimation (Macroblock *, int, MotionVector *, MotionVector *, MotionVector *, MotionVector *, MEBlock *, int, distblk, int);
#endif

