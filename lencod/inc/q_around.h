/*!
 ***************************************************************************
 * \file
 *    q_around.h
 *
 * \author
 *    Alexis Michael Tourapis
 *
 * \brief
 *    Headerfile for Quantization Adaptive Rounding
 **************************************************************************
 */

#ifndef _Q_AROUND_H_
#define _Q_AROUND_H_

extern void update_adaptive_rounding_8x8  (VideoParameters *p_Vid, InputParameters *p_Inp, RD_8x8DATA* dataTr, int**** ARCofAdj);
extern void store_adaptive_rounding_4x4   (VideoParameters *p_Vid, int****ARCofAdj, int mode, int block_y, int block_x);
extern void update_adaptive_rounding_4x4  (VideoParameters *p_Vid, int****ARCofAdj , int mode, int block_y, int block_x);
extern void update_adaptive_rounding_16x16(VideoParameters *p_Vid, int****ARCofAdj , int mode);
extern void store_adaptive_rounding_16x16 (VideoParameters *p_Vid, int****ARCofAdj, int mode);
extern void reset_adaptive_rounding_direct(VideoParameters *p_Vid);
extern void update_offset_params          (Macroblock *currMB, int mode, byte luma_transform_size_8x8_flag);

#endif

