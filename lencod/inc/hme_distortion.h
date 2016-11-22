/*!
 ***************************************************************************
 * \file
 *    hme_distortion.h
 *
 * \author
 *    Yuwen He                       
 *    Alexis Michael Tourapis       
 *
 * \date
 *    11. August 2006 / Updated 18. December 2013
 *
 * \brief
 *    Headerfile for motion estimation distortion
 **************************************************************************
 */

#ifndef _HME_DISTORTION_H_
#define _HME_DISTORTION_H_


//only use integer pel reference frame memory;
static inline imgpel *UMVLine4X_int_hme(StorablePicture *ref, MEBlock *mv_block, int y, int x)
{
    return &(ref->pHmeImage[mv_block->hme_level][iClip3(-IMG_PAD_SIZE_Y, mv_block->hme_ref_size_y_max, y >> 2)][iClip3(-IMG_PAD_SIZE_X, mv_block->hme_ref_size_x_max, x >> 2)]);
}


// SAD functions

extern distblk computeSAD_hme(StorablePicture *ref1, MEBlock*, distblk, MotionVector *);


#endif
