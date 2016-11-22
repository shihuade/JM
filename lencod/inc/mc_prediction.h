
/*!
 ************************************************************************
 * \file
 *    mc_prediction.h
 *
 * \brief
 *    motion compensation header
 *
 * \author
 *      Main contributors (see contributors.h for copyright, 
 *                         address and affiliation details)
 *      - Alexis Michael Tourapis  <alexismt@ieee.org>
 *
 *************************************************************************************
 */

#ifndef _MC_PREDICTION_H_
#define _MC_PREDICTION_H_
#include "mbuffer.h"

extern void luma_prediction       ( Macroblock* currMB, int, int, int, int, int, int[2], char *, short );
extern void luma_prediction_bi    ( Macroblock* currMB, int, int, int, int, int, int, short, short, int );
extern void chroma_prediction     ( Macroblock* currMB, int, int, int, int, int, int, int, int, short, short, short );
extern void chroma_prediction_4x4 ( Macroblock* currMB, int, int, int, int, int, int, short, short, short);   

extern void rdo_low_intra_chroma_decision              (Macroblock *currMB, int mb_available_up, int mb_available_left[2], int mb_available_up_left);
extern void rdo_low_intra_chroma_decision_mbaff        (Macroblock *currMB, int mb_available_up, int mb_available_left[2], int mb_available_up_left);
extern void OneComponentChromaPrediction4x4_regenerate (Macroblock *currMB, imgpel* , int , int , MotionVector ** , StorablePicture *listX, int );
extern void OneComponentChromaPrediction4x4_retrieve   (Macroblock *currMB, imgpel* , int , int , MotionVector ** , StorablePicture *listX, int );

extern void intra_chroma_prediction_mbaff(Macroblock *currMB, int*, int*, int*);
extern void intra_chroma_prediction      (Macroblock *currMB, int*, int*, int*);
extern void IntraChromaPrediction4x4     (Macroblock* currMB, int uv, int block_x, int  block_y);
extern void get_difference_4x4(imgpel **src, imgpel **prd, short *diff, int pos_x, int block_x);

#endif

