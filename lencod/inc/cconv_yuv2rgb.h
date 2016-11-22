/*!
 ***************************************************************************
 * \file
 *    cconv_yuv2rgb.h
 *
 * \author
 *    Woo-Shik Kim
 *
 * \date
 *    29 May 2008
 *
 * \brief
 *    Headerfile for YUV to RGB color conversion
 **************************************************************************
 */

#ifndef _CCONV_YUV2RGB_H_
#define _CCONV_YUV2RGB_H_

#include "img_distortion.h"

extern void init_YUVtoRGB    (VideoParameters *p_Vid, InputParameters *p_Inp);
extern void YUVtoRGB         (VideoParameters *p_Vid, ImageStructure *YUV, ImageStructure *RGB);
extern int  create_RGB_memory(VideoParameters *p_Vid);
extern void delete_RGB_memory(VideoParameters *p_Vid);

#endif

