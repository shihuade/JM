
/*!
 ***********************************************************************
 *  \file
 *      mbuffer.h
 *
 *  \brief
 *      Frame buffer functions
 *
 *  \author
 *      Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Karsten Suehring
 *      - Alexis Michael Tourapis  <alexismt@ieee.org>
 *      - Yuwen He                 <yhe@dolby.com>
 ***********************************************************************
 */
#ifndef _MBUFFERENC_H_
#define _MBUFFERENC_H_

#include "global.h"
#include "enc_statistics.h"

#define MAX_LIST_SIZE 33

typedef struct frame_store FrameStore;
typedef struct distortion_estimation Dist_Estm;
typedef struct picture_stats PictureStats;
typedef struct pic_motion_params_old PicMotionParamsOld;

struct picture_stats
{
  double dsum[3];
  double dvar[3];
};


//! definition of pic motion parameters
struct pic_motion_params_old
{
  byte *      mb_field;      //!< field macroblock indicator
};


//! definition of pic motion parameters
typedef struct pic_motion_params
{
  struct storable_picture *ref_pic[2];  //!< referrence picture pointer
  char                     ref_idx[2];  //!< reference picture   [list][subblock_y][subblock_x]
  MotionVector             mv[2];       //!< motion vector  
  byte                     field_frame; //!< indicates if co_located is field or frame. Will be removed at some point
} PicMotionParams;


//! definition a picture (field or frame)
typedef struct storable_picture
{
  PictureStructure structure;

  int         poc;
  int         top_poc;
  int         bottom_poc;
  int         frame_poc;
  int         order_num;
  unsigned    frame_num;
  int         pic_num;
  int         long_term_pic_num;
  int         long_term_frame_idx;
  int         temporal_layer;     

  byte        is_long_term;
  int         used_for_reference;
  int         is_output;
  int         non_existing;

  int         size_x, size_y, size_x_cr, size_y_cr;
  int         size_x_padded, size_y_padded;
  int         size_x_pad, size_y_pad;
  int         size_x_cr_pad, size_y_cr_pad;
  int         pad_size_uv_y, pad_size_uv_x; 
  int         chroma_vector_adjustment;
  int         coded_frame;
  int         mb_aff_frame_flag;

  imgpel **   imgY;          //!< Y picture component
  imgpel **** imgY_sub;      //!< Y picture component upsampled (Quarter pel)
  imgpel ***  imgUV;         //!< U and V picture components
  imgpel *****imgUV_sub;     //!< UV picture component upsampled (Quarter/One-Eighth pel)

  imgpel ***  p_dec_img[MAX_PLANE];      //!< pointer array for accessing decoded pictures in hypothetical decoders

  imgpel **   p_img[MAX_PLANE];          //!< pointer array for accessing imgY/imgUV[]
  imgpel **** p_img_sub[MAX_PLANE];      //!< pointer array for storing top address of imgY_sub/imgUV_sub[]
  imgpel **   p_curr_img;                //!< current int-pel ref. picture area to be used for motion estimation
  imgpel **** p_curr_img_sub;            //!< current sub-pel ref. picture area to be used for motion estimation
  
  // Hierarchical ME Image buffer
  imgpel ***  pHmeImage;     //!< Array allocated with dimensions [level][y][x];
  int    *    pHmeWidth;     //!< Width of hierarchical image at each level
  int    *    pHmeHeight;    //!< Height of hierarchical image at each level
  
  Dist_Estm * de_mem; 

  PicMotionParams **mv_info;                 //!< Motion info
  PicMotionParams **JVmv_info[MAX_PLANE];    //!< Motion info for 4:4:4 independent coding
  PicMotionParamsOld  motion;    //!< Motion info
  PicMotionParamsOld JVmotion[MAX_PLANE];    //!< Motion info for 4:4:4 independent coding

  int colour_plane_id;                     //!< colour_plane_id to be used for 4:4:4 independent mode encoding

  struct storable_picture *top_field;     // for mb aff, if frame for referencing the top field
  struct storable_picture *bottom_field;  // for mb aff, if frame for referencing the bottom field
  struct storable_picture *frame;         // for mb aff, if field for referencing the combined frame

  int         chroma_format_idc;
  int         chroma_mask_mv_x;
  int         chroma_mask_mv_y;
  int         chroma_shift_y;
  int         chroma_shift_x;
  int         frame_mbs_only_flag;
  int         frame_cropping_flag;
  int         frame_crop_left_offset;
  int         frame_crop_right_offset;
  int         frame_crop_top_offset;
  int         frame_crop_bottom_offset;

  PictureStats p_stats;
  StatParameters stats;

  int         type;

#if (MVC_EXTENSION_ENABLE)
  int         view_id;
  int         inter_view_flag[2];
  int         anchor_pic_flag[2];
#endif

  int  bInterpolated;
  int  ref_pic_na[6];
  int  otf_flag;
  //int  separate_colour_plane_flag;
} StorablePicture;

typedef StorablePicture *StorablePicturePtr;

//! Frame Stores for Decoded Picture Buffer
struct frame_store
{
  int       is_used;                //!< 0=empty; 1=top; 2=bottom; 3=both fields (or frame)
  int       is_reference;           //!< 0=not used for ref; 1=top used; 2=bottom used; 3=both fields (or frame) used
  int       is_long_term;           //!< 0=not used for ref; 1=top used; 2=bottom used; 3=both fields (or frame) used
  int       is_orig_reference;      //!< original marking by nal_ref_idc: 0=not used for ref; 1=top used; 2=bottom used; 3=both fields (or frame) used

  int       is_non_existent;

  unsigned  frame_num;
  int       frame_num_wrap;
  int       long_term_frame_idx;
  int       is_output;
  int       poc;

  StorablePicture *frame;
  StorablePicture *top_field;
  StorablePicture *bottom_field;

  Boolean   is_inter_layer;

#if (MVC_EXTENSION_ENABLE)
  int       view_id;
  int       inter_view_flag[2];
  int       anchor_pic_flag[2];
#endif
};



//! Decoded Picture Buffer
typedef struct decoded_picture_buffer
{
  VideoParameters *p_Vid;
  InputParameters *p_Inp;
  FrameStore  **fs;
  FrameStore  **fs_ref;
  FrameStore  **fs_ltref;
  FrameStore  **fs_ilref;
  int           num_ref_frames;
  unsigned      size;
  unsigned      used_size;
  unsigned      ref_frames_in_buffer;
  unsigned      ltref_frames_in_buffer;
  int           last_output_poc;
#if (MVC_EXTENSION_ENABLE)
  int           last_output_view_id;
#endif
  int           max_long_term_pic_idx;

  int           init_done;

  FrameStore   *last_picture;
  int           layer_id;
  unsigned      used_size_il;

  FrameFormat   storage_format;
  
  void (*pf_GetHMEIntImagesLuma)( VideoParameters *p_Vid, int size_x, int size_y, imgpel ***cImgInt);


  void (*pf_luma_prediction)       ( Macroblock* currMB, int, int, int, int, int, int[2], char *, short );
  void (*pf_luma_prediction_bi)    ( Macroblock* currMB, int, int, int, int, int, int, short, short, int );
  void (*pf_chroma_prediction)     ( Macroblock* currMB, int, int, int, int, int, int, int, int, short, short, short );
  void (*pf_get_block_luma)        ( struct video_par *, imgpel*, int*, int, int, int, int, struct storable_picture*, int );
  void (*pf_get_block_chroma[2])   ( struct video_par *, imgpel*, int*, int, int, int, int, struct storable_picture*, int );
  void (*pf_OneComponentChromaPrediction4x4_regenerate)(Macroblock *currMB, imgpel* , int , int , MotionVector ** , StorablePicture *listX, int );
  void (*pf_OneComponentChromaPrediction4x4_retrieve) (Macroblock *currMB, imgpel* , int , int , MotionVector ** , StorablePicture *listX, int );
  distblk (*pf_computeSAD)         (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
  distblk (*pf_computeSADWP)     (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
  distblk (*pf_computeSATD)      (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
  distblk (*pf_computeSATDWP)    (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
  distblk (*pf_computeBiPredSAD1)(StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
  distblk (*pf_computeBiPredSAD2)(StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
  distblk (*pf_computeBiPredSATD1)  (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
  distblk (*pf_computeBiPredSATD2)  (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
  distblk (*pf_computeSSE)       (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
  distblk (*pf_computeSSEWP)     (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
  distblk (*pf_computeBiPredSSE1)    (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
  distblk (*pf_computeBiPredSSE2)    (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
}DecodedPictureBuffer;

extern void             init_dpb                  (VideoParameters *p_Vid, DecodedPictureBuffer *dpb);
extern void             free_dpb                  (DecodedPictureBuffer *p_Dpb);
extern FrameStore*      alloc_frame_store(void);
extern void             free_frame_store          (VideoParameters *p_Vid, FrameStore* f);
extern StorablePicture* alloc_storable_picture    (VideoParameters *p_Vid, PictureStructure type, int size_x, int size_y, int size_x_cr, int size_y_cr);
extern void             free_storable_picture     (VideoParameters *p_Vid, StorablePicture* p);
extern void             store_picture_in_dpb      (DecodedPictureBuffer *p_Dpb, StorablePicture* p, FrameFormat *output);
extern void             replace_top_pic_with_frame(DecodedPictureBuffer *p_Dpb, StorablePicture* p, FrameFormat *output);
extern void             flush_dpb                 (DecodedPictureBuffer *p_Dpb, FrameFormat *output);
extern void             dpb_split_field           (VideoParameters *p_Vid, FrameStore *fs);
extern void             dpb_combine_field         (VideoParameters *p_Vid, FrameStore *fs);
extern void             dpb_combine_field_yuv     (VideoParameters *p_Vid, FrameStore *fs);
extern void             init_lists_p_slice        (Slice *currSlice);
extern void             init_lists_b_slice        (Slice *currSlice);
extern void             init_lists_i_slice        (Slice *currSlice);
extern void             update_pic_num            (Slice *currSlice);
extern void             reorder_ref_pic_list      (Slice *currSlice, int cur_list);
extern void             init_mbaff_lists          (Slice *currSlice);
extern void             alloc_ref_pic_list_reordering_buffer (Slice *currSlice);
extern void             free_ref_pic_list_reordering_buffer  (Slice *currSlice);
extern void             fill_frame_num_gap        (VideoParameters *p_Vid, FrameFormat *output);
extern void             compute_colocated         (Slice *currSlice, StorablePicture **listX[6]);
extern void             reorder_short_term(Slice *currSlice, DecodedPictureBuffer *p_Dpb, int cur_list, int picNumLX, int *refIdxLX);

extern void unmark_for_reference(FrameStore* fs);
extern void unmark_for_long_term_reference(FrameStore* fs);
extern void remove_frame_from_dpb(DecodedPictureBuffer *p_Dpb, int pos);

#if (MVC_EXTENSION_ENABLE)
void check_num_ref(DecodedPictureBuffer *p_Dpb);
extern void replace_top_proc_pic_with_frame(DecodedPictureBuffer *p_Dpb, StorablePicture* p);
extern void store_proc_picture_in_dpb(DecodedPictureBuffer *p_Dpb, StorablePicture* p, FrameFormat *output);
#endif

#endif

