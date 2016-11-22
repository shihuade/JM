
/*!
 ************************************************************************
 * \file
 *    macroblock_p444.h
 *
 * \brief
 *    Arrays for macroblock processing (P444 mode)
 *
 * \author
 *
 ************************************************************************/

#ifndef _MACROBLOCK_P444_H_
#define _MACROBLOCK_P444_H_

#include "block.h"

extern int luma_residual_coding_p444_16x16 (Macroblock* currMB, int, short, int[2], char *);
extern int luma_residual_coding_p444_8x8   (Macroblock* currMB, int*, int64*, int, short, int[2], char *);
extern void luma_residual_coding_p444 (Macroblock *currMB);

#endif

