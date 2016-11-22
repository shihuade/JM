/*!
 ***************************************************************************
 * \file
 *    img_dist_ms_ssim.h
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Woo-Shik Kim                    <wooshik.kim@usc.edu>
 *     - Alexis Michael Tourapis         <alexismt@ieee.org>
 *
 * \date
 *    18 July 2008
 *
 * \brief
 *    Headerfile to calculate multi-scale structural similarity (SSIM) index
 **************************************************************************
 */

#ifndef _IMG_DIST_MS_SSIM_H_
#define _IMG_DIST_MS_SSIM_H_
#include "img_distortion.h"

extern void find_ms_ssim (VideoParameters *p_Vid, InputParameters *p_Inp, ImageStructure *imgREF, ImageStructure *imgSRC, DistMetric metricMS_SSIM[3]);

#endif
