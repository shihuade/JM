/*!
 ***************************************************************************
 * \file
 *    wp.h
 *
 * \author
 *    Alexis Michael Tourapis
 *
 * \date
 *    22. February 2008
 *
 * \brief
 *    Headerfile for weighted prediction support
 **************************************************************************
 */

#ifndef _WP_H_
#define _WP_H_

#include "wp_lms.h"
#include "wp_mcprec.h"
#include "wp_mciter.h"
#include "wp_random.h"
#include "wp_periodic.h"

#define DEBUG_WP  0

void InitWP              (VideoParameters *p_Vid, InputParameters *p_Inp, int force_wp_method);
void ResetWP             (VideoParameters *p_Vid, InputParameters *p_Inp);

extern void   EstimateWPBSliceAlg0   (Slice *currSlice);
extern void   EstimateWPPSliceAlg0   (Slice *currSlice, int offset);
extern int    TestWPPSliceAlg0       (Slice *currSlice, int offset);
extern int    TestWPBSliceAlg0       (Slice *currSlice, int method);
extern double ComputeImgSum          (imgpel **CurrentImage, int height, int width);
extern void   ComputeImgSumBlockBased(imgpel **CurrentImage, int height_in_blk, int width_in_blk, int blk_size_y, int blk_size_x, int start_blk, int end_blk, double *dc);
extern int64  ComputeSumBlockBased   (imgpel **CurrentImage, int height_in_blk, int width_in_blk, int blk_size_y, int blk_size_x, int start_blk, int end_blk);

extern void   ComputeImplicitWeights    (Slice *currSlice,
                                         short default_weight[3],
                                         short im_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3]);

#endif

