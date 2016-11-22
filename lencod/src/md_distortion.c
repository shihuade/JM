
/*!
 ***************************************************************************
 * \file md_distortion.c
 *
 * \brief
 *    Main macroblock mode decision functions and helpers
 *
 **************************************************************************
 */

#include <math.h>
#include <limits.h>
#include <float.h>

#include "global.h"
#include "rdopt_coding_state.h"
#include "mb_access.h"
#include "intrarefresh.h"
#include "image.h"
#include "transform8x8.h"
#include "ratectl.h"
#include "mode_decision.h"
#include "fmo.h"
#include "me_umhex.h"
#include "me_umhexsmp.h"
#include "macroblock.h"
#include "mv_search.h"
#include "md_distortion.h"

void setupDistortion(Slice *currSlice)
{
  currSlice->getDistortion = distortionSSE;
}

/*!
 ***********************************************************************
 * \brief
 *    compute generic SSE
 ***********************************************************************
 */
int64 compute_SSE(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc, int ySize, int xSize)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  int64 distortion = 0;

  for (j = 0; j < ySize; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < xSize; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }
  return distortion;
}

distblk compute_SSE_cr(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc, int ySize, int xSize)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  distblk distortion = 0;

  for (j = 0; j < ySize; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < xSize; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }

  return dist_scale(distortion);
}

/*!
 ***********************************************************************
 * \brief
 *    compute 16x16 SSE
 ***********************************************************************
 */
distblk compute_SSE16x16(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  distblk distortion = 0;

  for (j = 0; j < MB_BLOCK_SIZE; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < MB_BLOCK_SIZE; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }

  return dist_scale(distortion);
}

distblk compute_SSE16x16_thres(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc, distblk min_cost)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  distblk distortion = 0;
  int imin_cost = dist_down(min_cost);

  for (j = 0; j < MB_BLOCK_SIZE; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < MB_BLOCK_SIZE; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
    if (distortion > imin_cost)
      return (min_cost);
  }

  return dist_scale(distortion);
}
/*!
 ***********************************************************************
 * \brief
 *    compute 8x8 SSE
 ***********************************************************************
 */
distblk compute_SSE8x8(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  distblk distortion = 0;

  for (j = 0; j < BLOCK_SIZE_8x8; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < BLOCK_SIZE_8x8; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }

  return dist_scale(distortion);
}


/*!
 ***********************************************************************
 * \brief
 *    compute 4x4 SSE
 ***********************************************************************
 */
distblk compute_SSE4x4(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  distblk distortion = 0;

  for (j = 0; j < BLOCK_SIZE; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < BLOCK_SIZE; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }

  return dist_scale(distortion);
}

/*!
*************************************************************************************
* \brief
*    SSE distortion calculation for a macroblock
*************************************************************************************
*/
distblk distortionSSE(Macroblock *currMB) 
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  distblk distortionY = 0;
  distblk distortionCr[2] = {0, 0};

  // LUMA
  distortionY = compute_SSE16x16(&p_Vid->pCurImg[currMB->opix_y], &p_Vid->enc_picture->p_curr_img[currMB->pix_y], currMB->pix_x, currMB->pix_x);

  // CHROMA
  if ((p_Vid->yuv_format != YUV400) && (p_Inp->separate_colour_plane_flag == 0))
  {
    distortionCr[0] = compute_SSE_cr(&p_Vid->pImgOrg[1][currMB->opix_c_y], &p_Vid->enc_picture->imgUV[0][currMB->pix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x);
    distortionCr[1] = compute_SSE_cr(&p_Vid->pImgOrg[2][currMB->opix_c_y], &p_Vid->enc_picture->imgUV[1][currMB->pix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Vid->mb_cr_size_y, p_Vid->mb_cr_size_x);
  }
#if JCOST_OVERFLOWCHECK //overflow checking;
  if(distortionY * p_Inp->WeightY + distortionCr[0] * p_Inp->WeightCb + distortionCr[1] * p_Inp->WeightCr > DISTBLK_MAX)
  {
    printf("Overflow: %s : %d \n MB: %d, Value: %lf\n", __FILE__, __LINE__, currMB->mbAddrX, (distortionY * p_Inp->WeightY + distortionCr[0] * p_Inp->WeightCb + distortionCr[1] * p_Inp->WeightCr));
    exit(-1);
  }
#endif //end;
  return (distblk)( distortionY * p_Inp->WeightY + distortionCr[0] * p_Inp->WeightCb + distortionCr[1] * p_Inp->WeightCr );
}
