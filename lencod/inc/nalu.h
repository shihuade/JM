
/*!
 **************************************************************************************
 * \file
 *    nalu.h
 * \brief
 *    Common NALU support functions
 *
 * \date 25 November 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */


#ifndef _NALU_H_
#define _NALU_H_

#include "nalucommon.h"

extern int RBSPtoNALU (unsigned char *rbsp, NALU_t *nalu, int rbsp_size, int nal_unit_type, int nal_reference_idc, int UseAnnexbLongStartcode);
extern int Write_AUD_NALU( VideoParameters *p_Vid );
extern int Write_Filler_Data_NALU( VideoParameters *p_Vid, int num_bytes );

#endif
