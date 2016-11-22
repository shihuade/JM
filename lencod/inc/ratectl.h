
/*!
 ***************************************************************************
 * \file
 *    ratectl.h
 *
 * \author
 *    Zhengguo LI
 *    Athanasios Leontaris
 *
 * \date
 *    14 Jan 2003
 *
 * \brief
 *    Headerfile for rate control
 **************************************************************************
 */

#ifndef _RATE_CTL_H_
#define _RATE_CTL_H_

#include "global.h"
#include "rc_quadratic.h"


// generic functions
extern int    Qstep2QP          ( double Qstep, int qp_offset );
extern double QP2Qstep          ( int QP );
extern int    ComputeMBMAD      ( int diff[16][16] );
extern double ComputeFrameMAD   ( VideoParameters *p_Vid );
extern void   rc_store_mad      ( Macroblock *currMB );

// rate control functions
// init/copy
extern void  rc_alloc_generic           ( VideoParameters *p_Vid, RCGeneric **p_quad );
extern void  rc_free_generic            ( RCGeneric **p_quad );
extern void  rc_copy_generic            ( VideoParameters *p_Vid, RCGeneric *dst, RCGeneric *src );
extern void  rc_init_gop_params         ( VideoParameters *p_Vid, InputParameters *p_Inp );
extern void  rc_init_frame              ( VideoParameters *p_Vid, InputParameters *p_Inp);
extern void  rc_init_sequence           ( VideoParameters *p_Vid, InputParameters *p_Inp);
extern void  rc_store_slice_header_bits ( VideoParameters *p_Vid, InputParameters *p_Inp, int len);

#endif

