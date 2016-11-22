/*!
 ***************************************************************************
 *
 * \file intra16x16.h
 *
 * \brief
 *    prototypes of 16x16 intra prediction 
 *
 * \date
 *    26 September 2009
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    Alexis Michael Tourapis
 **************************************************************************/

#ifndef _INTRA16X16_H_
#define _INTRA16X16_H_

extern distblk distI16x16_sad  (Macroblock *currMB, imgpel **img_org, imgpel **pred_img, distblk min_cost);
extern distblk distI16x16_sse  (Macroblock *currMB, imgpel **img_org, imgpel **pred_img, distblk min_cost);
extern distblk distI16x16_satd (Macroblock *currMB, imgpel **img_org, imgpel **pred_img, distblk min_cost);

extern distblk find_sad_16x16_JM      (Macroblock *currMB);

extern void set_intrapred_16x16_mbaff (Macroblock *currMB, ColorPlane pl, int *left_available, int *up_available, int *all_available);
extern void set_intrapred_16x16       (Macroblock *currMB, ColorPlane pl, int *left_available, int *up_available, int *all_available);
extern void get_intrapred_16x16       (Macroblock *currMB, ColorPlane pl, int i16x16_mode, int left_available, int up_available);

#endif //_INTRA16X16_H_
