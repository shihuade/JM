/*!
 ***************************************************************************
 * \file
 *    img_luma.h
 *
 * \author
 *    Athanasios Leontaris           <aleon@dolby.com>
 *    Alexis Michael Tourapis        <alexis.tourapis@dolby.com>
 *
 * \date
 *    4. October 2006
 *
 * \brief
 *    Headerfile for luma interpolation functions
 **************************************************************************
 */

#ifndef _IMG_LUMA_H_
#define _IMG_LUMA_H_

static const int ONE_FOURTH_TAP[2][3] =
{
  {20, -5, 1},  // AVC Interpolation taps
  {20,-4, 0},   // Experimental - not valid
};

extern void getSubImagesLuma       ( VideoParameters *p_Vid, StorablePicture *s );
/*
extern void getSubImageInteger     ( StorablePicture *s, imgpel **dstImg, imgpel **srcImg);
extern void getSubImageInteger_s   ( StorablePicture *s, imgpel **dstImg, imgpel **srcImg);
extern void getHorSubImageSixTap   ( VideoParameters *p_Vid, StorablePicture *s, imgpel **dst_imgY, imgpel **ref_imgY);
extern void getVerSubImageSixTap   ( VideoParameters *p_Vid, StorablePicture *s, imgpel **dst_imgY, imgpel **ref_imgY);
extern void getVerSubImageSixTapTmp( VideoParameters *p_Vid, StorablePicture *s, imgpel **dst_imgY);
extern void getSubImageBiLinear    ( StorablePicture *s, imgpel **dstImg, imgpel **srcImgL, imgpel **srcImgR);
extern void getHorSubImageBiLinear ( StorablePicture *s, imgpel **dstImg, imgpel **srcImgL, imgpel **srcImgR);
extern void getVerSubImageBiLinear ( StorablePicture *s, imgpel **dstImg, imgpel **srcImgT, imgpel **srcImgB);
extern void getDiagSubImageBiLinear( StorablePicture *s, imgpel **dstImg, imgpel **srcImgT, imgpel **srcImgB);
*/
#endif // _IMG_LUMA_H_
