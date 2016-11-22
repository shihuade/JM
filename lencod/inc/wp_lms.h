/*!
 ***************************************************************************
 * \file
 *    wp_lms.h
 *
 * \author
 *    Alexis Michael Tourapis
 *
 * \date
 *    22. February 2008
 *
 * \brief
 *    Headerfile for weighted prediction support using LMS
 **************************************************************************
 */

#ifndef _WP_LMS_H_
#define _WP_LMS_H_

extern void EstimateWPPSliceAlg1(Slice *currSlice, int offset);
extern void EstimateWPBSliceAlg1(Slice *currSlice);
extern int  TestWPPSliceAlg1    (Slice *currSlice, int offset);
extern int  TestWPBSliceAlg1    (Slice *currSlice, int method);
extern void   ComputeExplicitWPParamsLMS(Slice *currSlice,
                                         int select_offset,
                                         int start_mb,
                                         int end_mb, 
                                         short default_weight[3],
                                         short weight[6][MAX_REFERENCE_PICTURES][3],
                                         short offset[6][MAX_REFERENCE_PICTURES][3]);
extern void   ComputeExplicitWPParamsJNT(Slice *currSlice,
                                         int start_mb,
                                         int end_mb, 
                                         short default_weight[3],
                                         short weight[6][MAX_REFERENCE_PICTURES][3],
                                         short offset[6][MAX_REFERENCE_PICTURES][3]);



#endif

