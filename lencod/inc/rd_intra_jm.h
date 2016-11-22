/*!
 ***************************************************************************
 * \file
 *    rd_intra_jm.h
 *
 * \author
 *    Alexis Michael Tourapis
 *
 * \date
 *    2 January 2008
 *
 * \brief
 *    Headerfile for JM rd based intra mode decision
 **************************************************************************
 */

#ifndef _RD_INTRA_JM_H_
#define _RD_INTRA_JM_H_

extern int mode_decision_for_I16x16_MB_RDO          (Macroblock *currMB, int lambda);
extern int mode_decision_for_I16x16_MB              (Macroblock *currMB, int lambda);
extern int mode_decision_for_I4x4_blocks_JM_High    (Macroblock *currMB, int  b8,  int  b4,  int  lambda,  distblk*  min_cost);
extern int mode_decision_for_I4x4_blocks_JM_Low     (Macroblock *currMB, int  b8,  int  b4,  int  lambda,  distblk*  min_cost);
extern int find_best_mode_I16x16_MB                 (Macroblock *currMB, int lambda, distblk min_cost);

#endif

