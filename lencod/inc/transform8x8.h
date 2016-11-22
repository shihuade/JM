
/*!
 ***************************************************************************
 *
 * \file transform8x8.h
 *
 * \brief
 *    prototypes of 8x8 transform functions
 *
 * \date
 *    9. October 2003
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    Yuri Vatis
 **************************************************************************/

#ifndef _TRANSFORM8X8_H_
#define _TRANSFORM8X8_H_
extern int     mode_decision_for_I8x8_MB               (Macroblock *currMB, int lambda, distblk *min_cost);
extern int     mode_decision_for_I8x8_blocks_JM_Low    (Macroblock *currMB, int b8, int lambda, distblk *min_cost);
extern int     mode_decision_for_I8x8_blocks_JM_High   (Macroblock *currMB, int b8, int lambda, distblk *min_cost);
extern int     mode_decision_for_I8x8_blocks_JM_Low444 (Macroblock *currMB, int b8, int lambda, distblk *min_cost);
extern int     mode_decision_for_I8x8_blocks_JM_High444(Macroblock *currMB, int b8, int lambda, distblk *min_cost);

extern distblk rdcost_for_8x8_intra_blocks             (Macroblock *currMB, int *c_nz, int b8, int ipmode, int lambda, distblk min_rdcost, int mostProbableMode);
extern distblk rdcost_for_8x8_intra_blocks_444         (Macroblock *currMB, int *c_nz, int b8, int ipmode, int lambda, distblk min_rdcost, int mostProbableMode);
extern distblk compute_satd8x8_cost                    (VideoParameters *p_Vid, imgpel **cur_img, imgpel **mpr8x8, int pic_opix_x, distblk min_cost);
extern distblk compute_sse8x8_cost                     (VideoParameters *p_Vid, imgpel **cur_img, imgpel **mpr8x8, int pic_opix_x, distblk min_cost);
extern distblk compute_sad8x8_cost                     (VideoParameters *p_Vid, imgpel **cur_img, imgpel **mpr8x8, int pic_opix_x, distblk min_cost);
extern distblk compute_comp8x8_cost                    (VideoParameters *p_Vid, imgpel **cur_img, imgpel **mpr8x8, int pic_opix_x, distblk min_cost);

extern int     residual_transform_quant_luma_8x8       (Macroblock *currMB, ColorPlane pl, int b8, int *coeff_cost, int intra);
extern int     residual_transform_quant_luma_8x8_cavlc (Macroblock *currMB, ColorPlane pl, int b8, int *coeff_cost, int intra);
extern int     residual_transform_quant_luma_8x8_ls    (Macroblock *currMB, ColorPlane pl, int b8, int *coeff_cost, int intra);

#endif //_TRANSFORM8X8_H_
