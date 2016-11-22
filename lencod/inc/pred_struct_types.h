/*!
 ***************************************************************************
 * \file
 *    pred_struct_types.h
 *
 * \author
 *    Athanasios Leontaris           <aleon@dolby.com>   
 *
 * \date
 *    June 8, 2009
 *
 * \brief
 *    Header file for prediction structure types
 **************************************************************************
 */

#ifndef _PRED_STRUCT_TYPES_H_
#define _PRED_STRUCT_TYPES_H_

// default coding structures
// these are utilized to compress the current sequence when they are not pre-defined
// sizes are 1 to 4 frames: if a length 4 structure does not *fit* then we try a length 3, 2, or 1 structure. 
// One can define also structures > length 4. If these structures do not fit then we attempt to replace them 4, 3, 2, or 1. 
// We do not define default structures > 4 since the coding gains are questionable and the increase in memory and 
// computational compleity is usually prohibitive (e.g. Blu-Ray can do up to length 4 in HD resolution).

// After a prediction structure atom has been coded and the coding order of next frame to code is "code_order" 
// then all frames with frame_no lesser or equal to "code_order" times (frame_skip + 1) have been coded.

// non idr atoms
// idr atoms (for intra_delay)

typedef enum
{
  POP_IDR = 1,
  POP_INTRA = 2,
  POP_SP = 3
} SliceTypeToPop;

// prediction frame structure
typedef struct pred_struct_frm
{
  int disp_offset;
  int slice_type;
  int nal_ref_idc;
  int layer; // the lower the higher the priority
  int slice_qp_off;
  int random_access;
  int temporal_layer; 
} PredStructFrm;

// prediction structure
typedef struct pred_struct_atom
{
  int length;
  int gop_levels;
  PredStructFrm *p_frm;
} PredStructAtom;

// slice structure
typedef struct slice_struct
{
  int type;
  int qp;
  int deblock;
  int weighted_pred;
  int num_refs;
} SliceStructure;

// picture structure (frame, top field, bottom field)
typedef struct pic_struct
{
  int idr_flag;
  int nal_ref_idc;
  int random_access; // to support open GOPs
  int num_slices;
  SliceStructure *p_Slice;
} PicStructure;

// frame structure
typedef struct frame_struct
{
  int type; // dominant
  int idr_flag;
  int nal_ref_idc;
  int frame_no;
  int field_pic_flag;
  int mod_qp;             // QP modifier with respect to the slice type default QP
  int qp;                 // QP used to code this frame unit
  int layer;
  int num_refs;
  int random_access;      // random access point (IDR or Intra-coded picture that precludes future pictures in display order to reference pictures decoded prior to this picture)
  int atom_idx;           // index in the prediction structure pointed to be *p_atom
  int temporal_layer;     
#if (MVC_EXTENSION_ENABLE)
  int view_id;
#endif
  PicStructure *p_frame_pic;
  PicStructure *p_top_fld_pic;
  PicStructure *p_bot_fld_pic;
  PredStructAtom *p_atom; // pointer to the prediction structure type to which this frame belongs
} FrameUnitStruct;        // here "frame" is as in the real thing: the displayed frame; not as "frame" in "frame picture" in H.264

// sequence structure
typedef struct seq_struct
{
  int num_frames;
  int num_prds;
  int num_gops;
  int num_intra_gops;

  // frame indices based on *coding* order
  int last_random_access_frame;
  int last_idr_frame;
  int last_intra_frame;
  int last_mmco_5_frame; // values during building the structure: have *not* happened yet!
  int last_sp_frame;
  // frame indices based on *display* order
  int last_rand_access_disp;
  int last_idr_disp;
  int last_intra_disp;
  int last_sp_disp;
  int last_bl_frm_disposable; // check whether the last frame with layer == 0 (highest priority) 
  
  int curr_num_to_populate; // number of future frames in coding order to populate with each iteration of function populate_frm_struct (note that there are cases that fewer than "num_to_populate" frames may be populated
  int pop_start_frame; // frame index (in coding order) from which frame structure population commences

  int max_num_slices;
  int pop_flag;

#if (MVC_EXTENSION_ENABLE)
  int num_frames_mvc;
  FrameUnitStruct *p_frm_mvc; // frame struct store for num_views > 1
#endif

  FrameUnitStruct *p_frm;  

  PredStructAtom *p_prd; // regular prediction structure
  PredStructAtom *p_gop; // IDR GOPs
  PredStructAtom *p_intra_gop; // Intra GOPs
} SeqStructure;

#endif
