/*!
 ***************************************************************************
 * \file
 *    wp_random.h
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

#ifndef _WP_RANDOM_H_
#define _WP_RANDOM_H_

extern void EstimateWPPSliceRandom(Slice *currSlice, int offset);
extern void EstimateWPBSliceRandom(Slice *currSlice);
extern int  TestWPPSliceRandom    (Slice *currSlice, int offset);
extern int  TestWPBSliceRandom    (Slice *currSlice, int method);

#endif

