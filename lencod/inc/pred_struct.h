/*!
 ***************************************************************************
 * \file
 *    pred_struct.h
 *
 * \author
 *    Athanasios Leontaris           <aleon@dolby.com>   
 *
 * \date
 *    June 8, 2009
 *
 * \brief
 *    Header file for prediction structure function headers
 **************************************************************************
 */

#ifndef _PRED_STRUCT_H_
#define _PRED_STRUCT_H_

#include "global.h"
#include "pred_struct_types.h"
#include "explicit_seq.h"

extern void get_poc_type_zero( VideoParameters *p_Vid, InputParameters *p_Inp, FrameUnitStruct *p_frm_struct );
extern void get_poc_type_one( VideoParameters *p_Vid, InputParameters *p_Inp, FrameUnitStruct *p_frm_struct );
extern void init_poc(VideoParameters *p_Vid);
extern SeqStructure * init_seq_structure( VideoParameters *p_Vid, InputParameters *p_Inp, int *memory_size );
extern void free_seq_structure( SeqStructure *p_seq_struct );
extern void populate_frm_struct( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, int num_to_populate, int init_frames_to_code );
extern void populate_frame_explicit( ExpFrameInfo *info, InputParameters *p_Inp, FrameUnitStruct *p_frm_struct, int num_slices );
extern void populate_frame_slice_type( InputParameters *p_Inp, FrameUnitStruct *p_frm_struct, int slice_type, int num_slices );
extern void populate_reg_pic( InputParameters *p_Inp, PicStructure *p_pic, FrameUnitStruct *p_frm_struct, int num_slices, int is_bot_fld );
extern void populate_frm_struct_mvc( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, int start, int end );

#endif
