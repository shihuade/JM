
/*!
 ************************************************************************
 * \file intra4x4.h
 *
 * \brief
 *    intra 4x4 functions
 *
 * \author
 *    - Alexis Michael Tourapis         <alexismt@ieee.org>
 *
 ************************************************************************
 */

#ifndef _INTRA4x4_H_
#define _INTRA4x4_H_

// intra 4x4 generation function
extern void set_intrapred_4x4_mbaff(Macroblock *currMB, ColorPlane pl, int img_x, int img_y, int *left_available, int *up_available, int *all_available);
extern void set_intrapred_4x4      (Macroblock *currMB, ColorPlane pl, int img_x, int img_y, int *left_available, int *up_available, int *all_available);
extern void get_intrapred_4x4      (Macroblock *currMB, ColorPlane pl, int i4x4_mode, int img_x, int img_y, int left_available, int up_available);

// 4x4 residual generation
extern void generate_pred_error_4x4(imgpel **cur_img, imgpel **prd_img, imgpel **cur_prd, int **mb_rres, int pic_opix_x, int block_x);
#endif

