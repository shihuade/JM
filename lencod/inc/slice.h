/*!
 ***************************************************************************
 * \file
 *    slice.h
 *
 * \date
 *    16 July 2008
 *
 * \brief
 *    Headerfile for slice-related functions
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Athanasios Leontaris            <aleon@dolby.com>
 *     - Karsten Suehring
 *     - Alexis Michael Tourapis         <alexismt@ieee.org> 

 **************************************************************************
 */

#ifndef _SLICE_H_
#define _SLICE_H_

#include "global.h"
#include "mbuffer.h"
#include "rdopt_coding_state.h"

static const int QP2QUANT[40]=
{
   1, 1, 1, 1, 2, 2, 2, 2,
   3, 3, 3, 4, 4, 4, 5, 6,
   6, 7, 8, 9,10,11,13,14,
  16,18,20,23,25,29,32,36,
  40,45,51,57,64,72,81,91
};


extern int  encode_one_slice       ( VideoParameters *p_Vid, int SliceGroupId, int TotalCodedMBs );
extern int  encode_one_slice_MBAFF ( VideoParameters *p_Vid, int SliceGroupId, int TotalCodedMBs );
extern void init_slice             ( VideoParameters *p_Vid, Slice **currSlice, int start_mb_addr );
extern void init_slice_lite        ( VideoParameters *p_Vid, Slice **currSlice, int start_mb_addr );
extern void free_slice_list        ( Picture *currPic );

extern void SetLagrangianMultipliersOn (Slice *currSlice);
extern void SetLagrangianMultipliersOff(Slice *currSlice);
extern void  free_slice                (Slice *currSlice);


#endif
