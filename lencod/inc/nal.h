
/*!
 **************************************************************************************
 * \file
 *    nal.h
 * \brief
 *    NAL related headers
 *
 * \date 4 Agust 2008
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Karsten Suehring
 *      - Alexis Michael Tourapis         <alexismt@ieee.org> 
 ***************************************************************************************
 */


#ifndef _NAL_H_
#define _NAL_H_

#include "nalucommon.h"
#include "enc_statistics.h"

extern int  addCabacZeroWords(VideoParameters *p_Vid, NALU_t *nalu, StatParameters *cur_stats);
extern void SODBtoRBSP (Bitstream *currStream);
extern int  RBSPtoEBSP(byte *NaluBuffer, unsigned char *rbsp, int rbsp_size);

#endif
