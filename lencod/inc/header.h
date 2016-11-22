
/*!
 *************************************************************************************
 * \file header.h
 *
 * \brief
 *    Prototypes for header.c
 *************************************************************************************
 */

#ifndef _HEADER_H_
#define _HEADER_H_

extern int SliceHeader        (Slice* currSlice);
extern int Partition_BC_Header(Slice *currSlice, int PartNo);
extern int writeERPS          (SyntaxElement *sym, DataPartition *partition);
extern int dec_ref_pic_marking(Bitstream *bitstream, DecRefPicMarking_t *p_drpm, int idr_flag, int no_output_of_prior_pics_flag, int long_term_reference_flag );

#endif

