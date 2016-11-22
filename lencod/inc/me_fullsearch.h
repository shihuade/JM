
/*!
 ************************************************************************
 * \file
 *     me_fullsearch.h
 *
 * \author
 *    Alexis Michael Tourapis        <alexis.tourapis@dolby.com>
 *
 * \date
 *    9 September 2006
 *
 * \brief
 *    Headerfile for Full Search motion estimation
 **************************************************************************
 */


#ifndef _ME_FULLSEARCH_H_
#define _ME_FULLSEARCH_H_
extern distblk full_search_motion_estimation         (Macroblock *, MotionVector *, MEBlock *, distblk, int );
extern distblk full_search_bipred_motion_estimation  (Macroblock *, int, MotionVector *, MotionVector *, MotionVector *, MotionVector *, MEBlock *, int, distblk, int );
extern distblk sub_pel_motion_estimation             (Macroblock *, MotionVector *, MEBlock *, distblk, int *);
extern distblk sub_pel_bipred_motion_estimation      (Macroblock *, MEBlock *, int, MotionVector *, MotionVector *, MotionVector *, MotionVector *, distblk, int *);
extern distblk full_sub_pel_motion_estimation        (Macroblock *, MotionVector *, MEBlock *, distblk, int *);
extern distblk full_sub_pel_bipred_motion_estimation (Macroblock *, MEBlock *, int, MotionVector *, MotionVector *, MotionVector *, MotionVector *, distblk, int *);
#endif

