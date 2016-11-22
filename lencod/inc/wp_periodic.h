/*!
 ***************************************************************************
 * \file
 *    wp_periodic.h
 *
 * \author
 *    Alexis Michael Tourapis
 *
 * \date
 *    19. December 2011
 *
 * \brief
 *    Headerfile for weighted prediction using a random number generator
 **************************************************************************
 */

#ifndef _WP_PERIODIC_H_
#define _WP_PERIODIC_H_

extern void EstimateWPPSlicePeriodic(Slice *currSlice, int offset);
extern void EstimateWPBSlicePeriodic(Slice *currSlice);
extern int  TestWPPSlicePeriodic    (Slice *currSlice, int offset);
extern int  TestWPBSlicePeriodic    (Slice *currSlice, int method);

#endif

