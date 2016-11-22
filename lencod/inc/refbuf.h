
/*!
 ************************************************************************
 * \file refbuf.h
 *
 * \brief
 *    Declarations of the reference frame buffer types and functions
 ************************************************************************
 */
#ifndef _REBUF_H_
#define _REBUF_H_

#include "mbuffer.h"

/*!
 ************************************************************************
 * \brief
 *    Yields a pel line _pointer_ from one of the 16 sub-images
 *    Input does not require subpixel image indices
 ************************************************************************
 */
static inline imgpel *UMVLine4X (StorablePicture *ref, int y, int x)
{
  //return &(ref->p_curr_img_sub[(y & 0x03)][(x & 0x03)][iClip3( 0, ref->size_y_pad, y >> 2)][iClip3( 0, ref->size_x_pad, x >> 2)]);
  return &(ref->p_curr_img_sub[(y & 0x03)][(x & 0x03)][iClip3( -IMG_PAD_SIZE_Y, ref->size_y_pad, y >> 2)][iClip3(-IMG_PAD_SIZE_X, ref->size_x_pad, x >> 2)]);
}

/*!
 ************************************************************************
 * \brief
 *    Yields a pel line _pointer_ from one of the 16 sub-images
 *    Input does not require subpixel image indices
 ************************************************************************
 */
static inline imgpel *UMVLine4Xcr (StorablePicture *ref, int cmp, int y, int x)
{
  //return &(ref->p_img_sub[cmp][(y & 0x03)][(x & 0x03)][iClip3( 0, ref->size_y_cr_pad, y >> 2)][iClip3( 0, ref->size_x_cr_pad, x >> 2)]);
  return &(ref->p_img_sub[cmp][(y & 0x03)][(x & 0x03)][iClip3(-ref->pad_size_uv_y, ref->size_y_cr_pad, y >> 2)][iClip3(-ref->pad_size_uv_x, ref->size_x_cr_pad, x >> 2)]);
}

/*!
 ************************************************************************
 * \brief
 *    Yields a pel line _pointer_ from one of the 16 sub-images
 *    Input does not require subpixel image indices
 ************************************************************************
 */
static inline imgpel *FastLine4X (StorablePicture *ref, int y, int x)
{
  return &(ref->p_curr_img_sub[(y & 0x03)][(x & 0x03)][y >> 2][x >> 2]);
}

/*!
 ************************************************************************
 * \brief
 *    Yields a pel line _pointer_ from one of the 16 (4:4:4), 32 (4:2:2),
 *    or 64 (4:2:0) sub-images
 *    Input does not require subpixel image indices
 ************************************************************************
 */
static inline imgpel *UMVLine8X_chroma (StorablePicture *ref, int cmp, int y, int x)
{
  //return &(ref->p_img_sub[cmp][y & ref->chroma_mask_mv_y][x & ref->chroma_mask_mv_x][iClip3 (0, ref->size_y_cr_pad, y >> ref->chroma_shift_y)][iClip3 (0, ref->size_x_cr_pad, x >> ref->chroma_shift_x)]);
  return &(ref->p_img_sub[cmp][y & ref->chroma_mask_mv_y][x & ref->chroma_mask_mv_x][iClip3 (-ref->pad_size_uv_y, ref->size_y_cr_pad, y >> ref->chroma_shift_y)][iClip3 (-ref->pad_size_uv_x, ref->size_x_cr_pad, x >> ref->chroma_shift_x)]);
}

/*!
 ************************************************************************
 * \brief
 *    Yields a pel line _pointer_ from one of the 16 (4:4:4), 32 (4:2:2),
 *    or 64 (4:2:0) sub-images
 *    Input does not require subpixel image indices
 ************************************************************************
 */
static inline imgpel *FastLine8X_chroma (StorablePicture *ref, int cmp, int y, int x)
{
  return &(ref->p_img_sub[cmp][y & ref->chroma_mask_mv_y][x & ref->chroma_mask_mv_x][y >> ref->chroma_shift_y][x >> ref->chroma_shift_x]);
}


#endif

