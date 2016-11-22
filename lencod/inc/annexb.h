/*!
 **************************************************************************************
 * \file
 *    annexb.h
 * \brief
 *    Byte stream operations support
 *
 *  \date 7 December 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */

#ifndef _ANNEXB_H_
#define _ANNEXB_H_

#include "nalucommon.h"

extern int WriteAnnexbNALU (VideoParameters *p_Vid, NALU_t *n, FILE **f_annexb);
extern void OpenAnnexbFile (char *fn, FILE **f_annexb);
extern void CloseAnnexbFile(FILE *f_annexb);


#endif //_ANNEXB_H_
