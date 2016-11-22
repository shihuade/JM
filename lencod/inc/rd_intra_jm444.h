/*!
 ***************************************************************************
 * \file
 *    rd_intra_jm444.h
 *
 * \author
 *    Alexis Michael Tourapis
 *
 * \date
 *    2 January 2008
 *
 * \brief
 *    Headerfile for JM rd based intra mode decision (High444)
 **************************************************************************
 */

#ifndef _RD_INTRA_JM444_H_
#define _RD_INTRA_JM444_H_

extern int mode_decision_for_I16x16_MB_444          (Macroblock *currMB, int lambda);
extern int mode_decision_for_I4x4_blocks_JM_High444 (Macroblock *currMB, int  b8,  int  b4,  int  lambda,  distblk*  min_cost);
extern int mode_decision_for_I4x4_blocks_JM_Low444  (Macroblock *currMB, int  b8,  int  b4,  int  lambda,  distblk*  min_cost);
#endif

