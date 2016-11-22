
/*!
************************************************************************
* \file quantChroma.h
*
* \brief
*    Quantization process for chroma header file
*
* \author
*    Limin Liu                       <lliu@dolby.com>
*    Alexis Michael Tourapis         <alexismt@ieee.org>                
*
************************************************************************
*/

#ifndef _QUANT_CR_H_
#define _QUANT_CR_H_

extern void init_quant_Chroma(Slice *currSlice);

extern int quant_dc2x2_normal (Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4, int **fadjust2x2, const byte (*pos_scan)[2]);

extern int quant_dc2x2_around (Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4, int **fadjust2x2, const byte (*pos_scan)[2]);

extern int quant_dc2x2_trellis(Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4, int **fadjust, const byte (*pos_scan)[2]);

extern int quant_dc4x2_normal (Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4, int **fadjust, const byte (*pos_scan)[2]);

extern int quant_dc4x2_around (Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4,  int **fadjust, const byte (*pos_scan)[2]);

extern int quant_dc4x2_trellis(Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4, int **fadjust, const byte (*pos_scan)[2]);

extern void rdoq_dc_cr_CAVLC  (Macroblock *currMB, int **tblock, int qp_per, int qp_rem, 
                               LevelQuantParams *q_params_4x4, 
                               const byte (*pos_scan)[2], int levelTrellis[16], int type);

extern void rdoq_dc_cr_CABAC  (Macroblock *currMB, int **tblock, int qp_per, int qp_rem, 
                               LevelQuantParams *q_params_4x4, const byte (*pos_scan)[2], int levelTrellis[16], int type);

#endif

