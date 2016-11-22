
/*!
 ***************************************************************************
 * \file
 *    q_offsets.h
 *
 * \brief
 *    Headerfile for q_offsets array
 *
 * \date
 *    18. Nov 2004
 ***************************************************************************
 */

#ifndef _Q_OFFSETS_H_
#define _Q_OFFSETS_H_

static const int OffsetBits = 11;

extern void init_qoffset            (VideoParameters *p_Vid);
extern void CalculateOffset4x4Param (VideoParameters *p_Vid);
extern void CalculateOffset8x8Param (VideoParameters *p_Vid);
extern void free_QOffsets           (QuantParameters *p_Quant, InputParameters *p_Inp);
extern void InitOffsetParam (QuantParameters *p_Quant, InputParameters *p_Inp);
#endif
