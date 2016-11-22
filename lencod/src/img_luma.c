/*!
*************************************************************************************
* \file img_luma_uint8.c
*
* \brief
*    Luma interpolation functions
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*      - Alexis Michael Tourapis <alexis.tourapis@dolby.com>
*      - Athanasios Leontaris    <aleon@dolby.com>
*      - Yuwen He                <yhe@dolby.com>
*
*************************************************************************************
*/

#include "contributors.h"

#include <limits.h>

#include "global.h"
#include "image.h"
#include "img_luma.h"
#include "memalloc.h"


/*!
 ************************************************************************
 * \brief
 *    Copy Integer Samples to image [0][0]
 *
 * \param s
 *    pointer to StorablePicture structure
 * \param dstImg
 *    destination image
 * \param srcImg
 *    source image
 ************************************************************************
 */
static void getSubImageInteger( StorablePicture *s, imgpel **dstImg, imgpel **srcImg)
{
  int i, j;
  int size_x_minus1 = s->size_x - 1;

  imgpel *wBufSrc, *wBufDst;

  // Copy top line
  wBufDst = &( dstImg[-IMG_PAD_SIZE_Y][-IMG_PAD_SIZE_X] ); 
  wBufSrc = srcImg[0];
  // left IMG_PAD_SIZE
  for (i = 0; i < IMG_PAD_SIZE_X; i++)
    *(wBufDst++) = wBufSrc[0];
  // center 0-(s->size_x)
  memcpy(wBufDst, wBufSrc, s->size_x * sizeof(imgpel));
  wBufDst += s->size_x;
  // right IMG_PAD_SIZE
  for (i = 0; i < IMG_PAD_SIZE_X; i++)
    *(wBufDst++) = wBufSrc[size_x_minus1];

  // Now copy remaining pad lines
  for (j = -IMG_PAD_SIZE_Y+1; j < 1; ++j)
  {
    memcpy(dstImg[j]-IMG_PAD_SIZE_X, dstImg[j - 1]-IMG_PAD_SIZE_X, s->size_x_padded * sizeof(imgpel));
  }

  for (j = 1; j < s->size_y; j++)
  {    
    wBufDst = &( dstImg[j][-IMG_PAD_SIZE_X] ); 
    wBufSrc = srcImg[j];
    // left IMG_PAD_SIZE
    for (i = 0; i < IMG_PAD_SIZE_X; i++)
      *(wBufDst++) = wBufSrc[0];
    // center 0-(s->size_x)
    memcpy(wBufDst, wBufSrc, s->size_x * sizeof(imgpel));
    wBufDst += s->size_x;
    // right IMG_PAD_SIZE
    for (i = 0; i < IMG_PAD_SIZE_X; i++)
      *(wBufDst++) = wBufSrc[size_x_minus1];
  }

  // Replicate bottom pad lines
  for (j = s->size_y; j < s->size_y+IMG_PAD_SIZE_Y; j++)
  {    
    memcpy(dstImg[j]-IMG_PAD_SIZE_X, dstImg[j - 1]-IMG_PAD_SIZE_X, s->size_x_padded * sizeof(imgpel));
  }
}

static void getSubImageInteger_s( StorablePicture *s, imgpel **dstImg, imgpel **srcImg)
{
  int i, j;
  int size_x_minus1 = s->size_x - 1;

  imgpel *wBufSrc, *wBufDst;

  // Copy top line
  wBufDst = &( dstImg[-IMG_PAD_SIZE_Y][-IMG_PAD_SIZE_X] ); 
  wBufSrc = srcImg[0];
  // left IMG_PAD_SIZE
  for (i = 0; i < IMG_PAD_SIZE_X; ++i)
    *(wBufDst++) = wBufSrc[0];
  // center 0-(s->size_x)
  memcpy(wBufDst, wBufSrc, s->size_x * sizeof(imgpel));
  wBufDst += s->size_x;
  // right IMG_PAD_SIZE
  for (i = 0; i < IMG_PAD_SIZE_X; ++i)
    *(wBufDst++) = wBufSrc[size_x_minus1];

  // Now copy remaining pad lines
  for (j = -IMG_PAD_SIZE_Y+1; j < 1; ++j)
  {
    memcpy(dstImg[j]-IMG_PAD_SIZE_X, dstImg[j - 1]-IMG_PAD_SIZE_X, s->size_x_padded * sizeof(imgpel));
  }

  for (j = 1; j < s->size_y; ++j)
  {    
    wBufDst = &( dstImg[j][-IMG_PAD_SIZE_X] ); 
    wBufSrc = srcImg[j];
    // left IMG_PAD_SIZE
    for (i = 0; i < IMG_PAD_SIZE_X; ++i)
      *(wBufDst++) = wBufSrc[0];
    // center 0-(s->size_x)
    //memcpy(wBufDst, wBufSrc, s->size_x * sizeof(imgpel));
    wBufDst += s->size_x;
    // right IMG_PAD_SIZE
    for (i = 0; i < IMG_PAD_SIZE_X; ++i)
      *(wBufDst++) = wBufSrc[size_x_minus1];
  }

  // Replicate bottom pad lines
  for (j = s->size_y; j < s->size_y+IMG_PAD_SIZE_Y; ++j)
  {    
    memcpy(dstImg[j]-IMG_PAD_SIZE_X, dstImg[j - 1]-IMG_PAD_SIZE_X, s->size_x_padded * sizeof(imgpel));
  }
}

/*!
 ************************************************************************
 * \brief
 *    Does _horizontal_ interpolation using the SIX TAP filters
 *
 * \param p_Vid
 *    pointer to VideoParameters structure
 * \param s
 *    pointer to StorablePicture structure
 * \param dstImg
 *    destination image
 * \param srcImg
 *    source image
 ************************************************************************
 */
static void getHorSubImageSixTap( VideoParameters *p_Vid, StorablePicture *s, imgpel **dstImg, imgpel **srcImg)
{
  int is, jpad, ipad;
  int ypadded_size = s->size_y_padded;
  int xpadded_size = s->size_x_padded;

  imgpel *wBufSrc, *wBufDst;
  imgpel *srcImgA, *srcImgB, *srcImgC, *srcImgD, *srcImgE, *srcImgF;
  int *iBufDst;
  const int tap0 = ONE_FOURTH_TAP[0][0];
  const int tap1 = ONE_FOURTH_TAP[0][1];
  const int tap2 = ONE_FOURTH_TAP[0][2];

  for (jpad = -IMG_PAD_SIZE_Y; jpad < ypadded_size-IMG_PAD_SIZE_Y; jpad++)
  {
    wBufSrc = srcImg[jpad]-IMG_PAD_SIZE_X; 
    wBufDst = dstImg[jpad]-IMG_PAD_SIZE_X; 
    iBufDst = p_Vid->imgY_sub_tmp[jpad]-IMG_PAD_SIZE_X;

    srcImgA = &wBufSrc[0];
    srcImgB = &wBufSrc[0];      
    srcImgC = &wBufSrc[0];
    srcImgD = &wBufSrc[1];
    srcImgE = &wBufSrc[2];
    srcImgF = &wBufSrc[3];

    // left padded area
    is =
      (tap0 * (*srcImgA++ + *srcImgD++) +
      tap1 *  (*srcImgB   + *srcImgE++) +
      tap2 *  (*srcImgC   + *srcImgF++));

    *iBufDst++ =  is;
    *wBufDst++ = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );      

    is =
      (tap0 * (*srcImgA++ + *srcImgD++) +
      tap1 *  (*srcImgB++ + *srcImgE++) +
      tap2 *  (*srcImgC   + *srcImgF++));

    *iBufDst++ =  is;
    *wBufDst++ = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );      

    // center
    for (ipad = 2; ipad < xpadded_size - 4; ipad++)
    {
      is =
        (tap0 * (*srcImgA++ + *srcImgD++) +
        tap1 *  (*srcImgB++ + *srcImgE++) +
        tap2 *  (*srcImgC++ + *srcImgF++));

      *iBufDst++ =  is;
      *wBufDst++ = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );      
    }

    is = (
      tap0 * (*srcImgA++ + *srcImgD++) +
      tap1 * (*srcImgB++ + *srcImgE++) +
      tap2 * (*srcImgC++ + *srcImgF  ));

    *iBufDst++ =  is;
    *wBufDst++ = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );      

    // right padded area
    is = (
      tap0 * (*srcImgA++ + *srcImgD++) +
      tap1 * (*srcImgB++ + *srcImgE) +
      tap2 * (*srcImgC++ + *srcImgF));

    *iBufDst++ =  is;
    *wBufDst++ = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );      

    is = (
      tap0 * (*srcImgA++ + *srcImgD) +
      tap1 * (*srcImgB++ + *srcImgE) +
      tap2 * (*srcImgC++ + *srcImgF));

    *iBufDst++ =  is;
    *wBufDst++ = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );      

    is = (
      tap0 * (*srcImgA + *srcImgD) +
      tap1 * (*srcImgB + *srcImgE) +
      tap2 * (*srcImgC + *srcImgF));

    *iBufDst =  is;
    *wBufDst = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );      

  }
}

/*!
 ************************************************************************
 * \brief
 *    Does _vertical_ interpolation using the SIX TAP filters
 *
 * \param p_Vid
 *    pointer to VideoParameters structure
 * \param s
 *    pointer to StorablePicture structure
 * \param dstImg
 *    pointer to target image
 * \param srcImg
 *    pointer to source image
 ************************************************************************
 */
static void getVerSubImageSixTap( VideoParameters *p_Vid, StorablePicture *s, imgpel **dstImg, imgpel **srcImg)
{
  int is, jpad, ipad;
  int ypadded_size = s->size_y_padded;
  int xpadded_size = s->size_x_padded;
  int maxy = ypadded_size - 1-IMG_PAD_SIZE_Y;

  imgpel *wxLineDst;
  imgpel *srcImgA, *srcImgB, *srcImgC, *srcImgD, *srcImgE, *srcImgF;
  const int tap0 = ONE_FOURTH_TAP[0][0];
  const int tap1 = ONE_FOURTH_TAP[0][1];
  const int tap2 = ONE_FOURTH_TAP[0][2];

  // branches within the j loop
  // top
  for (jpad = -IMG_PAD_SIZE_Y; jpad < 2-IMG_PAD_SIZE_Y; jpad++)
  {
    wxLineDst = dstImg[jpad]-IMG_PAD_SIZE_X;
    srcImgA = srcImg[jpad ]-IMG_PAD_SIZE_X;
    srcImgB = srcImg[0-IMG_PAD_SIZE_Y]-IMG_PAD_SIZE_X;      
    srcImgC = srcImg[0-IMG_PAD_SIZE_Y]-IMG_PAD_SIZE_X;
    srcImgD = srcImg[jpad + 1]-IMG_PAD_SIZE_X;
    srcImgE = srcImg[jpad + 2]-IMG_PAD_SIZE_X;
    srcImgF = srcImg[jpad + 3]-IMG_PAD_SIZE_X;
    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      is =
        (tap0 * (*srcImgA++ + *srcImgD++) +
        tap1 *  (*srcImgB++ + *srcImgE++) +
        tap2 *  (*srcImgC++ + *srcImgF++));

      wxLineDst[ipad] = (imgpel) iClip1 (p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );
    }
  }
  // center
  for (jpad = 2-IMG_PAD_SIZE_Y; jpad < ypadded_size - 3-IMG_PAD_SIZE_Y; jpad++)
  {
    wxLineDst = dstImg[jpad]-IMG_PAD_SIZE_X;
    srcImgA = srcImg[jpad ]-IMG_PAD_SIZE_X;
    srcImgB = srcImg[jpad - 1]-IMG_PAD_SIZE_X;      
    srcImgC = srcImg[jpad - 2]-IMG_PAD_SIZE_X;
    srcImgD = srcImg[jpad + 1]-IMG_PAD_SIZE_X;
    srcImgE = srcImg[jpad + 2]-IMG_PAD_SIZE_X;
    srcImgF = srcImg[jpad + 3]-IMG_PAD_SIZE_X;
    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      is =
        (tap0 * (*srcImgA++ + *srcImgD++) +
        tap1 *  (*srcImgB++ + *srcImgE++) +
        tap2 *  (*srcImgC++ + *srcImgF++));

      wxLineDst[ipad] = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );
    }
  }

  // bottom
  for (jpad = ypadded_size - 3-IMG_PAD_SIZE_Y; jpad < ypadded_size-IMG_PAD_SIZE_Y; jpad++)
  {
    wxLineDst = dstImg[jpad]-IMG_PAD_SIZE_X;
    srcImgA = srcImg[jpad ]-IMG_PAD_SIZE_X;
    srcImgB = srcImg[jpad - 1]-IMG_PAD_SIZE_X;      
    srcImgC = srcImg[jpad - 2]-IMG_PAD_SIZE_X;
    srcImgD = srcImg[imin (maxy, jpad + 1)]-IMG_PAD_SIZE_X;
    srcImgE = srcImg[maxy]-IMG_PAD_SIZE_X;
    srcImgF = srcImg[maxy]-IMG_PAD_SIZE_X;
    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      is =
        (tap0 * (*srcImgA++ + *srcImgD++) +
        tap1 *  (*srcImgB++ + *srcImgE++) +
        tap2 *  (*srcImgC++ + *srcImgF++));

      wxLineDst[ipad] = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 5 ) );
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Does _vertical_ interpolation using the SIX TAP filters
 *
 * \param p_Vid
 *    pointer to VideoParameters structure
 * \param s
 *    pointer to StorablePicture structure
 * \param dstImg
 *    pointer to source image
 ************************************************************************
 */
static void getVerSubImageSixTapTmp( VideoParameters *p_Vid, StorablePicture *s, imgpel **dstImg)
{
  int is, jpad, ipad;
  int ypadded_size = s->size_y_padded;
  int xpadded_size = s->size_x_padded;
  int maxy = ypadded_size - 1-IMG_PAD_SIZE_Y;

  imgpel *wxLineDst;
  int *srcImgA, *srcImgB, *srcImgC, *srcImgD, *srcImgE, *srcImgF;
  const int tap0 = ONE_FOURTH_TAP[0][0];
  const int tap1 = ONE_FOURTH_TAP[0][1];
  const int tap2 = ONE_FOURTH_TAP[0][2];

  // top
  for (jpad = -IMG_PAD_SIZE_Y; jpad < 2-IMG_PAD_SIZE_Y; jpad++)
  {
    wxLineDst = dstImg[jpad]-IMG_PAD_SIZE_X;
    srcImgA = p_Vid->imgY_sub_tmp[jpad ]-IMG_PAD_SIZE_X;
    srcImgB = p_Vid->imgY_sub_tmp[0-IMG_PAD_SIZE_Y]-IMG_PAD_SIZE_X;     
    srcImgC = p_Vid->imgY_sub_tmp[0-IMG_PAD_SIZE_Y]-IMG_PAD_SIZE_X;
    srcImgD = p_Vid->imgY_sub_tmp[jpad + 1]-IMG_PAD_SIZE_X;
    srcImgE = p_Vid->imgY_sub_tmp[jpad + 2]-IMG_PAD_SIZE_X;
    srcImgF = p_Vid->imgY_sub_tmp[jpad + 3]-IMG_PAD_SIZE_X;

    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      is =
        (tap0 * (*srcImgA++ + *srcImgD++) +
        tap1 *  (*srcImgB++ + *srcImgE++) +
        tap2 *  (*srcImgC++ + *srcImgF++));

      wxLineDst[ipad] = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 10 ) );
    }
  }

  // center
  for (jpad = 2-IMG_PAD_SIZE_Y; jpad < ypadded_size - 3-IMG_PAD_SIZE_Y; jpad++)
  {
    wxLineDst = dstImg[jpad]-IMG_PAD_SIZE_X;
    srcImgA = p_Vid->imgY_sub_tmp[jpad ]-IMG_PAD_SIZE_X;
    srcImgB = p_Vid->imgY_sub_tmp[jpad - 1]-IMG_PAD_SIZE_X;     
    srcImgC = p_Vid->imgY_sub_tmp[jpad - 2]-IMG_PAD_SIZE_X;
    srcImgD = p_Vid->imgY_sub_tmp[jpad + 1]-IMG_PAD_SIZE_X;
    srcImgE = p_Vid->imgY_sub_tmp[jpad + 2]-IMG_PAD_SIZE_X;
    srcImgF = p_Vid->imgY_sub_tmp[jpad + 3]-IMG_PAD_SIZE_X;
    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      is =
        (tap0 * (*srcImgA++ + *srcImgD++) +
        tap1 *  (*srcImgB++ + *srcImgE++) +
        tap2 *  (*srcImgC++ + *srcImgF++));

      wxLineDst[ipad] = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 10 ) );
    }
  }

  // bottom
  for (jpad = ypadded_size - 3-IMG_PAD_SIZE_Y; jpad < ypadded_size-IMG_PAD_SIZE_Y; jpad++)
  {
    wxLineDst = dstImg[jpad]-IMG_PAD_SIZE_X;
    srcImgA = p_Vid->imgY_sub_tmp[jpad ]-IMG_PAD_SIZE_X;
    srcImgB = p_Vid->imgY_sub_tmp[jpad - 1]-IMG_PAD_SIZE_X;      
    srcImgC = p_Vid->imgY_sub_tmp[jpad - 2]-IMG_PAD_SIZE_X;
    srcImgD = p_Vid->imgY_sub_tmp[imin (maxy, jpad + 1)]-IMG_PAD_SIZE_X;
    srcImgE = p_Vid->imgY_sub_tmp[maxy]-IMG_PAD_SIZE_X;
    srcImgF = p_Vid->imgY_sub_tmp[maxy]-IMG_PAD_SIZE_X;
    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      is =
        (tap0 * (*srcImgA++ + *srcImgD++) +
        tap1 *  (*srcImgB++ + *srcImgE++) +
        tap2 *  (*srcImgC++ + *srcImgF++));

      wxLineDst[ipad] = (imgpel) iClip1 ( p_Vid->max_imgpel_value, rshift_rnd_sf( is, 10 ) );
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Does _horizontal_ interpolation using the BiLinear filter
 *
 * \param s
 *    pointer to StorablePicture structure
 * \param dstImg
 *    destination Image
 * \param srcImgL
 *    source left image
 * \param srcImgR
 *    source right image 
 ************************************************************************
 */
static void getSubImageBiLinear( StorablePicture *s, imgpel **dstImg, imgpel **srcImgL, imgpel **srcImgR)
{
  int jpad, ipad;
  int ypadded_size = s->size_y_padded;
  int xpadded_size = s->size_x_padded;

  imgpel *wBufSrcL, *wBufSrcR, *wBufDst;

  for (jpad = -IMG_PAD_SIZE_Y; jpad < ypadded_size-IMG_PAD_SIZE_Y; jpad++)
  {
    wBufSrcL = srcImgL[jpad]-IMG_PAD_SIZE_X; 
    wBufSrcR = srcImgR[jpad]-IMG_PAD_SIZE_X; 
    wBufDst  = dstImg[jpad]-IMG_PAD_SIZE_X;  

    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      *wBufDst++ = (imgpel) rshift_rnd_sf( *wBufSrcL++ + *wBufSrcR++, 1 );
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Does _horizontal_ interpolation using the BiLinear filter
 *
 * \param s
 *    pointer to StorablePicture structure
 * \param dstImg
 *    destination Image
 * \param srcImgL
 *    source left image
 * \param srcImgR
 *    source right image 
 ************************************************************************
 */
static void getHorSubImageBiLinear( StorablePicture *s, imgpel **dstImg, imgpel **srcImgL, imgpel **srcImgR)
{
  int jpad, ipad;
  int ypadded_size = s->size_y_padded;
  int xpadded_size = s->size_x_padded - 1;

  imgpel *wBufSrcL, *wBufSrcR, *wBufDst;

  for (jpad = -IMG_PAD_SIZE_Y; jpad < ypadded_size-IMG_PAD_SIZE_Y; jpad++)
  {
    wBufSrcL = srcImgL[jpad]-IMG_PAD_SIZE_X; 
    wBufSrcR = &srcImgR[jpad][1-IMG_PAD_SIZE_X]; 
    wBufDst  = dstImg[jpad]-IMG_PAD_SIZE_X;     

    // left padded area + center
    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      *wBufDst++ = (imgpel) rshift_rnd_sf( *wBufSrcL++ + *wBufSrcR++, 1 );
    }
    // right padded area
      *wBufDst++ = (imgpel) rshift_rnd_sf( *wBufSrcL++ + wBufSrcR[-1], 1 );
  }
}


/*!
 ************************************************************************
 * \brief
 *    Does _vertical_ interpolation using the BiLinear filter
 *
 * \param s
 *    pointer to StorablePicture structure
 * \param dstImg
 *    destination Image
 * \param srcImgT
 *    source top image
 * \param srcImgB
 *    source bottom image 
 ************************************************************************
 */
static void getVerSubImageBiLinear( StorablePicture *s, imgpel **dstImg, imgpel **srcImgT, imgpel **srcImgB)
{
  int jpad, ipad;
  int ypadded_size = s->size_y_padded - 1;
  int xpadded_size = s->size_x_padded;  

  imgpel *wBufSrcT, *wBufSrcB, *wBufDst;

  // top
  for (jpad = -IMG_PAD_SIZE_Y; jpad < ypadded_size-IMG_PAD_SIZE_Y; jpad++)
  {
    wBufSrcT = srcImgT[jpad]-IMG_PAD_SIZE_X;           
    wBufDst  = dstImg[jpad]-IMG_PAD_SIZE_X;            
    wBufSrcB = srcImgB[jpad + 1]-IMG_PAD_SIZE_X;  

    for (ipad = 0; ipad < xpadded_size; ipad++)
    {
      *wBufDst++ = (imgpel) rshift_rnd_sf(*wBufSrcT++ + *wBufSrcB++, 1);
    }
  }
  // bottom
  wBufSrcT = srcImgT[ypadded_size-IMG_PAD_SIZE_Y]-IMG_PAD_SIZE_X;           
  wBufDst  = dstImg[ypadded_size-IMG_PAD_SIZE_Y]-IMG_PAD_SIZE_X;            
  wBufSrcB = srcImgB[ypadded_size-IMG_PAD_SIZE_Y]-IMG_PAD_SIZE_X;           

  for (ipad = 0; ipad < xpadded_size; ipad++)
  {
    *wBufDst++ = (imgpel) rshift_rnd_sf(*wBufSrcT++ + *wBufSrcB++, 1);
  }
}


/*!
 ************************************************************************
 * \brief
 *    Does _diagonal_ interpolation using the BiLinear filter
 *
 * \param s
 *    pointer to StorablePicture structure
 * \param dstImg
 *    destination Image
 * \param srcImgT
 *    source top/left image
 * \param srcImgB
 *    source bottom/right image 
 ************************************************************************
 */
static void getDiagSubImageBiLinear( StorablePicture *s, imgpel **dstImg, imgpel **srcImgT, imgpel **srcImgB )
{
  int jpad, ipad;
  int maxx = s->size_x_padded - 1-IMG_PAD_SIZE_X;
  int maxy = s->size_y_padded - 1-IMG_PAD_SIZE_Y;

  imgpel *wBufSrcL, *wBufSrcR, *wBufDst;

  for (jpad = -IMG_PAD_SIZE_Y; jpad < maxy; jpad++)
  {
    wBufSrcL = srcImgT[jpad + 1]-IMG_PAD_SIZE_X; 
    wBufSrcR = &srcImgB[jpad][1-IMG_PAD_SIZE_X]; 
    wBufDst  = dstImg[jpad]-IMG_PAD_SIZE_X;      

    for (ipad = -IMG_PAD_SIZE_X; ipad < maxx; ipad++)
    {
      *wBufDst++ = (imgpel) rshift_rnd_sf(*wBufSrcL++ + *wBufSrcR++, 1);
    }

    *wBufDst++ = (imgpel) rshift_rnd_sf(*wBufSrcL++ +  wBufSrcR[-1], 1);
  }

  wBufSrcL = srcImgT[maxy]-IMG_PAD_SIZE_X;     
  wBufSrcR = &srcImgB[maxy][1-IMG_PAD_SIZE_X]; 
  wBufDst = dstImg[maxy]-IMG_PAD_SIZE_X;       

  for (ipad = -IMG_PAD_SIZE_X; ipad < maxx; ipad++)
  {
    *wBufDst++ = (imgpel) rshift_rnd_sf(*wBufSrcL++ + *wBufSrcR++, 1);
  }

    *wBufDst++ = (imgpel) rshift_rnd_sf(*wBufSrcL++ + wBufSrcR[-1], 1);
}

/*!
 ************************************************************************
 * \brief
 *    Creates the 4x4 = 16 images that contain quarter-pel samples
 *    sub-sampled at different spatial orientations;
 *    enables more efficient implementation
 *
 * \param p_Vid
 *    pointer to VideoParameters structure
 * \param s
 *    pointer to StorablePicture structure
 s************************************************************************
 */
void getSubImagesLuma( VideoParameters *p_Vid, StorablePicture *s )
{
  imgpel ****cImgSub   = s->p_curr_img_sub;
  int        otf_shift = ( p_Vid->p_Inp->OnTheFlyFractMCP == OTF_L1 ) ? (1) : (0) ;

  //  0  1  2  3
  //  4  5  6  7
  //  8  9 10 11
  // 12 13 14 15

  //// INTEGER PEL POSITIONS ////

  // sub-image 0 [0][0]
  // simply copy the integer pels
  if (cImgSub[0][0][0] != s->p_curr_img[0])
  {
    getSubImageInteger( s, cImgSub[0][0], s->p_curr_img);
  }
  else
  {
    getSubImageInteger_s( s, cImgSub[0][0], s->p_curr_img);
  }

  //// HALF-PEL POSITIONS: SIX-TAP FILTER ////

  // sub-image 2 [0][2]
  // HOR interpolate (six-tap) sub-image [0][0]
  getHorSubImageSixTap( p_Vid, s, cImgSub[0][2>>otf_shift], cImgSub[0][0] );

  // sub-image 8 [2][0]
  // VER interpolate (six-tap) sub-image [0][0]
  getVerSubImageSixTap( p_Vid, s, cImgSub[2>>otf_shift][0], cImgSub[0][0]);

  // sub-image 10 [2][2]
  // VER interpolate (six-tap) sub-image [0][2]
  getVerSubImageSixTapTmp( p_Vid, s, cImgSub[2>>otf_shift][2>>otf_shift]);

  if( !p_Vid->p_Inp->OnTheFlyFractMCP )
  {
    //// QUARTER-PEL POSITIONS: BI-LINEAR INTERPOLATION ////

    // sub-image 1 [0][1]
    getSubImageBiLinear    ( s, cImgSub[0][1], cImgSub[0][0], cImgSub[0][2]);
    // sub-image 4 [1][0]
    getSubImageBiLinear    ( s, cImgSub[1][0], cImgSub[0][0], cImgSub[2][0]);
    // sub-image 5 [1][1]
    getSubImageBiLinear    ( s, cImgSub[1][1], cImgSub[0][2], cImgSub[2][0]);
    // sub-image 6 [1][2]
    getSubImageBiLinear    ( s, cImgSub[1][2], cImgSub[0][2], cImgSub[2][2]);
    // sub-image 9 [2][1]
    getSubImageBiLinear    ( s, cImgSub[2][1], cImgSub[2][0], cImgSub[2][2]);

    // sub-image 3  [0][3]
    getHorSubImageBiLinear ( s, cImgSub[0][3], cImgSub[0][2], cImgSub[0][0]);
    // sub-image 7  [1][3]
    getHorSubImageBiLinear ( s, cImgSub[1][3], cImgSub[0][2], cImgSub[2][0]);
    // sub-image 11 [2][3]
    getHorSubImageBiLinear ( s, cImgSub[2][3], cImgSub[2][2], cImgSub[2][0]);

    // sub-image 12 [3][0]
    getVerSubImageBiLinear ( s, cImgSub[3][0], cImgSub[2][0], cImgSub[0][0]);
    // sub-image 13 [3][1]
    getVerSubImageBiLinear ( s, cImgSub[3][1], cImgSub[2][0], cImgSub[0][2]);
    // sub-image 14 [3][2]
    getVerSubImageBiLinear ( s, cImgSub[3][2], cImgSub[2][2], cImgSub[0][2]);

    // sub-image 15 [3][3]
    getDiagSubImageBiLinear( s, cImgSub[3][3], cImgSub[0][2], cImgSub[2][0]);
  }
}
