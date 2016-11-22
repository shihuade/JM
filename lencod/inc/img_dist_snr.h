/*!
 ***************************************************************************
 * \file
 *    img_dist_snr.h
 *
 * \author
 *    Woo-Shik Kim
 *
 * \date
 *    29 May 2008
 *
 * \brief
 *    Headerfile to calculate signal to noise ratio (SNR)
 **************************************************************************
 */

#ifndef _IMG_DIST_SNR_H_
#define _IMG_DIST_SNR_H_
#include "img_distortion.h"

extern void find_snr(VideoParameters *p_Vid, ImageStructure *imgREF, ImageStructure *imgSRC, DistMetric metricSSE[3], DistMetric metricPSNR[3]);

#endif

