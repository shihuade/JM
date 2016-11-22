
/*!
************************************************************************
* \file quant8x8.h
*
* \brief
*    Quantization process header file
*
* \author
*    Alexis Michael Tourapis         <alexismt@ieee.org>                
*
************************************************************************
*/

#ifndef _QUANT8x8_H_
#define _QUANT8x8_H_

extern void init_quant_8x8 (Slice *currSlice);

extern int quant_8x8_normal (Macroblock *currMB, int **tblock, struct quant_methods *q_method);
extern int quant_8x8_around (Macroblock *currMB, int **tblock, struct quant_methods *q_method);
extern int quant_8x8_trellis(Macroblock *currMB, int **tblock, struct quant_methods *q_method);

extern int quant_8x8cavlc_around (Macroblock *currMB, int **tblock, struct quant_methods *q_method, int***  cofAC); 
extern int quant_8x8cavlc_normal (Macroblock *currMB, int **tblock, struct quant_methods *q_method, int***  cofAC); 
extern int quant_8x8cavlc_trellis(Macroblock *currMB, int **tblock, struct quant_methods *q_method, int***  cofAC); 

#endif

