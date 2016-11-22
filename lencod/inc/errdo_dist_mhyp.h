/*!
 **************************************************************************
 *  \file errdo_dist_mhyp.h
 *  \brief  
 *    Header file for distortion estimation with multiple decoder based method
 *
 *  \author 
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Zhifeng Chen                     <zzchen@dolby.com>
 *
 **************************************************************************
 */

#ifndef _ERRDO_DIST_MHYP_H_
#define _ERRDO_DIST_MHYP_H_

extern void errdo_store_best_block_multihyp  (InputParameters *p_Inp, imgpel*** mbY, imgpel*** dec_img, int block, int img_i, int img_j, int block_size);
extern void errdo_get_best_block_multihyp    (Macroblock *currMB, imgpel*** dec_img, imgpel*** mbY, int block, int block_size);
extern distblk errdo_distortion_estimation_multihyp(Macroblock *currMB, int block, int block_size, short mode, short pdir, distblk min_rdcost);
extern void copy_conceal_picture    (VideoParameters *p_Vid, StorablePicture *enc_pic, int decoder);
#endif
