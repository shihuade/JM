/*!
 ***************************************************************************
 * \file
 *    mode_decision_p8x8.h
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Alexis Michael Tourapis         <alexismt@ieee.org>
 *
 * \date
 *    1 September 2009
 *
 * \brief
 *    Headerfile for 8x8 mode decision
 **************************************************************************
 */

#ifndef _MODE_DECISION_P8x8_H_
#define _MODE_DECISION_P8x8_H_

//==== MODULE PARAMETERS ====
extern void submacroblock_mode_decision_p_slice (Macroblock *currMB, RD_PARAMS *, RD_8x8DATA *, int ****, int, distblk *);
extern void submacroblock_mode_decision_b_slice (Macroblock *currMB, RD_PARAMS *, RD_8x8DATA *, int ****, int, distblk *);
extern void submacroblock_mode_decision_low(Macroblock *currMB, RD_PARAMS *, RD_8x8DATA *, int ****, int *, int, distblk *, distblk *, distblk *, int);
extern void copy_part_info(Info8x8 *b8x8, Info8x8 *part);

#endif

