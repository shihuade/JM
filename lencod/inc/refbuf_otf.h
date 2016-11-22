
/*!
 ************************************************************************
 * \file refbuf_otf.h
 *
 * \brief
 *    Declarations of the reference frame buffer types and functions for on-the-fly interpolations
 ************************************************************************
 */
#ifndef _REBUF_OTF_H_
#define _REBUF_OTF_H_

#include "mbuffer.h"

static inline imgpel *UMVLine4X_otf (StorablePicture *ref, int y, int x )
{
  return &(ref->p_curr_img_sub[(y & 0x03)>>1][(x & 0x03)>>1][iClip3( -IMG_PAD_SIZE_Y, ref->size_y_pad, y >> 2)][iClip3(-IMG_PAD_SIZE_X, ref->size_x_pad, x >> 2)]);
}

static inline imgpel *UMVLine4Xcr_otf (StorablePicture *ref, int cmp, int y, int x)
{
  return &(ref->p_img_sub[cmp][(y & 0x03)>>1][(x & 0x03)>>1][iClip3(-ref->pad_size_uv_y, ref->size_y_cr_pad, y >> 2)][iClip3(-ref->pad_size_uv_x, ref->size_x_cr_pad, x >> 2)]);
}

static inline imgpel *UMVLine8X_chroma_otf (StorablePicture *ref, int cmp, int y, int x)
{
  return &(ref->p_img_sub[cmp][(y & ref->chroma_mask_mv_y)>>1][(x & ref->chroma_mask_mv_x)>>1][iClip3 (-ref->pad_size_uv_y, ref->size_y_cr_pad, y >> ref->chroma_shift_y)][iClip3 (-ref->pad_size_uv_x, ref->size_x_cr_pad, x >> ref->chroma_shift_x)]);
}


#endif

