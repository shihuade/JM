
/*!
************************************************************************
* \file quant4x4.h
*
* \brief
*    Quantization process header file
*
* \author
*    Limin Liu                       <lliu@dolby.com>
*    Alexis Michael Tourapis         <alexismt@ieee.org>                
*
************************************************************************
*/

#ifndef _QUANT4x4_H_
#define _QUANT4x4_H_

extern void init_quant_4x4  (Slice *currSlice);

extern int quant_4x4_normal (Macroblock *currMB, int **tblock, struct quant_methods *q_method);
extern int quant_4x4_around (Macroblock *currMB, int **tblock, struct quant_methods *q_method);
extern int quant_4x4_trellis(Macroblock *currMB, int **tblock, struct quant_methods *q_method);
extern int quant_dc4x4_normal (Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4, const byte (*pos_scan)[2]);

extern int quant_dc4x4_around (Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4, const byte (*pos_scan)[2]);

extern int quant_dc4x4_trellis(Macroblock *currMB, int **tblock, int qp, int* DCLevel, int* DCRun, 
                               LevelQuantParams *q_params_4x4, const byte (*pos_scan)[2]);

extern int quant_ac4x4_normal (Macroblock *currMB, int **tblock, struct quant_methods *q_method);
extern int quant_ac4x4_around (Macroblock *currMB, int **tblock, struct quant_methods *q_method);
extern int quant_ac4x4_trellis(Macroblock *currMB, int **tblock, struct quant_methods *q_method);

extern void rdoq_4x4_CAVLC    (Macroblock *currMB, int **tblock, struct quant_methods *q_method, int levelTrellis[16]);

extern void rdoq_4x4_CABAC    (Macroblock *currMB, int **tblock, struct quant_methods *q_method, int levelTrellis[16]);

extern void rdoq_dc_CAVLC     (Macroblock *currMB, int **tblock, int qp_per, int qp_rem, LevelQuantParams *q_params_4x4, 
                               const byte (*pos_scan)[2], int levelTrellis[16], int type);

extern void rdoq_dc_CABAC     (Macroblock *currMB, int **tblock, int qp_per, int qp_rem, LevelQuantParams *q_params_4x4, 
                               const byte (*pos_scan)[2], int levelTrellis[16], int type);

extern void rdoq_ac4x4_CAVLC  (Macroblock *currMB, int **tblock, struct quant_methods *q_method, int levelTrellis[16]);

extern void rdoq_ac4x4_CABAC  (Macroblock *currMB, int **tblock , struct quant_methods *q_method, int levelTrellis[16]);


#endif

