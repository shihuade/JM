
/*!
 ***********************************************************************
 *  \file
 *      mbuffer.c
 *
 *  \brief
 *      Frame buffer functions
 *
 *  \author
 *      Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Karsten Suehring
 *      - Alexis Tourapis                 <alexismt@ieee.org>
 ***********************************************************************
 */

#include <limits.h>

#include "global.h"
#include "mbuffer.h"
#include "mbuffer_common.h"
#include "memalloc.h"
#include "output.h"
#include "image.h"
#include "nalucommon.h"
#include "img_luma.h"
#include "img_chroma.h"
#include "errdo.h"
#include "me_hme.h"

extern void SbSMuxBasic(ImageData *imgOut, ImageData *imgIn0, ImageData *imgIn1, int offset);
extern void init_stats                   (InputParameters *p_Inp, StatParameters *stats);
static void insert_picture_in_dpb        (VideoParameters *p_Vid, FrameStore* fs, StorablePicture* p);
static void output_one_frame_from_dpb    (DecodedPictureBuffer *p_Dpb, FrameFormat *output);
static void gen_field_ref_ids            (StorablePicture *p);
static int  flush_unused_frame_from_dpb  (DecodedPictureBuffer *p_Dpb);
static void process_picture_in_dpb_s(VideoParameters *p_Vid, StorablePicture *p_pic);
static StorablePicture * clone_storable_picture( VideoParameters *p_Vid, StorablePicture *p_pic );

#define MAX_LIST_SIZE 33

extern void free_mem2Duint16(uint16 **array2D);
extern void free_mem3Duint16(uint16 ***array3D);

/*!
 ************************************************************************
 * \brief
 *    Print out list of pictures in DPB. Used for debug purposes.
 ************************************************************************
 */
void dump_dpb(DecodedPictureBuffer *p_Dpb)
{
#if DUMP_DPB
  unsigned i;

  for (i=0; i<p_Dpb->used_size;i++)
  {
    printf("(");
    printf("fn=%d  ", p_Dpb->fs[i]->frame_num);
    if (p_Dpb->fs[i]->is_used & 1)
    {
      if (p_Dpb->fs[i]->top_field)
        printf("T: poc=%d  ", p_Dpb->fs[i]->top_field->poc);
      else
        printf("T: poc=%d  ", p_Dpb->fs[i]->frame->top_poc);
    }
    if (p_Dpb->fs[i]->is_used & 2)
    {
      if (p_Dpb->fs[i]->bottom_field)
        printf("B: poc=%d  ", p_Dpb->fs[i]->bottom_field->poc);
      else
        printf("B: poc=%d  ", p_Dpb->fs[i]->frame->bottom_poc);
    }
    if (p_Dpb->fs[i]->is_used == 3)
      printf("F: poc=%d  ", p_Dpb->fs[i]->frame->poc);
    printf("G: poc=%d)  ", p_Dpb->fs[i]->poc);
    if (p_Dpb->fs[i]->is_reference) printf ("ref (%d) ", p_Dpb->fs[i]->is_reference);
    if (p_Dpb->fs[i]->is_long_term) printf ("lt_ref (%d) ", p_Dpb->fs[i]->is_reference);
    if (p_Dpb->fs[i]->is_output) printf ("out  ");
    if (p_Dpb->fs[i]->is_used == 3)
    {
      if (p_Dpb->fs[i]->frame->non_existing) printf ("ne  ");
    }
#if (MVC_EXTENSION_ENABLE)
    if(p_Dpb->p_Inp->num_of_views==2)
      printf ("view_id=%d ", p_Dpb->fs[i]->view_id);
#endif
    printf ("\n");
  }
#endif
}

/*!
 ************************************************************************
 * \brief
 *    Returns the size of the dpb depending on level and picture size
 *
 *
 ************************************************************************
 */
int getDpbSize(VideoParameters *p_Vid, seq_parameter_set_rbsp_t *active_sps)
{
  int pic_size_mb = (active_sps->pic_width_in_mbs_minus1 + 1) * (active_sps->pic_height_in_map_units_minus1 + 1) * (active_sps->frame_mbs_only_flag?1:2);

  int size = 0;

  switch (active_sps->level_idc)
  {
  case 0:
    // if there is no level defined, we expect experimental usage and return a DPB size of 16
    return 16;
  case 9:
    size = 396;
    break;
  case 10:
    size = 396;
    break;
  case 11:
    if (!is_FREXT_profile(active_sps->profile_idc) && (active_sps->constrained_set3_flag == 1))
      size = 396;
    else
      size = 900;
    break;
  case 12:
    size = 2376;
    break;
  case 13:
    size = 2376;
    break;
  case 20:
    size = 2376;
    break;
  case 21:
    size = 4752;
    break;
  case 22:
    size = 8100;
    break;
  case 30:
    size = 8100;
    break;
  case 31:
    size = 18000;
    break;
  case 32:
    size = 20480;
    break;
  case 40:
    size = 32768;
    break;
  case 41:
    size = 32768;
    break;
  case 42:
    size = 34816;
    break;
  case 50:
    size = 110400;
    break;
  case 51:
    size = 184320;
    break;
  case 52:
    size = 184320;
    break;
  case 60:
  case 61:
  case 62:
    size = 696320;
    break;
  default:
    error ("undefined level", 500);
    break;
  }

  size /= pic_size_mb;
  if(is_MVC_profile(p_Vid->p_Inp->ProfileIDC))
  {
    int num_views = p_Vid->p_Inp->num_of_views;
    size = imin(2*size, imax(1, RoundLog2(num_views))*16)/num_views;
  }
  else
    size = imin( size, 16);

  if (active_sps->vui_parameters_present_flag && active_sps->vui_seq_parameters.bitstream_restriction_flag)
  {
    if ((int)active_sps->vui_seq_parameters.max_dec_frame_buffering > size)
    {
      error ("max_dec_frame_buffering larger than MaxDpbSize", 500);
    }
    size = imax (1, active_sps->vui_seq_parameters.max_dec_frame_buffering);
  }

  return size;
}

/*!
 ************************************************************************
 * \brief
 *    Check then number of frames marked "used for reference" and break
 *    if maximum is exceeded
 *
 ************************************************************************
 */
void check_num_ref(DecodedPictureBuffer *p_Dpb)
{
  if ((int)(p_Dpb->ltref_frames_in_buffer +  p_Dpb->ref_frames_in_buffer ) > imax(1, p_Dpb->num_ref_frames))
  {
    error ("Max. number of reference frames exceeded. Invalid stream.", 500);
  }
}


/*!
 ************************************************************************
 * \brief
 *    Allocate memory for decoded picture buffer and initialize with sane values.
 *
 ************************************************************************
 */
void init_dpb(VideoParameters *p_Vid, DecodedPictureBuffer *p_Dpb)
{
  unsigned i;

  p_Dpb->p_Vid = p_Vid;
  p_Dpb->p_Inp = p_Vid->p_Inp;
  p_Dpb->num_ref_frames = p_Vid->num_ref_frames;

  if (p_Dpb->init_done)
  {
    free_dpb(p_Dpb);
  }

  p_Dpb->size = getDpbSize(p_Vid, p_Vid->sps[p_Dpb->layer_id]);


  if (p_Dpb->size < (unsigned int) (p_Vid->p_Inp)->num_ref_frames)
  {
    error ("DPB size at specified level is smaller than the specified number of reference frames. This is not allowed.\n", 1000);
  }

  p_Dpb->used_size = 0;
  p_Dpb->last_picture = NULL;

  p_Dpb->ref_frames_in_buffer = 0;
  p_Dpb->ltref_frames_in_buffer = 0;

  p_Dpb->fs = calloc(p_Dpb->size, sizeof (FrameStore*));
  if (NULL==p_Dpb->fs)
    no_mem_exit("init_dpb: p_Dpb->fs");

  p_Dpb->fs_ref = calloc(p_Dpb->size, sizeof (FrameStore*));
  if (NULL==p_Dpb->fs_ref)
    no_mem_exit("init_dpb: p_Dpb->fs_ref");

  p_Dpb->fs_ltref = calloc(p_Dpb->size, sizeof (FrameStore*));
  if (NULL==p_Dpb->fs_ltref)
    no_mem_exit("init_dpb: p_Dpb->fs_ltref");

  for (i = 0; i < p_Dpb->size; i++)
  {
    p_Dpb->fs[i]       = alloc_frame_store();
    p_Dpb->fs_ref[i]   = NULL;
    p_Dpb->fs_ltref[i] = NULL;
  }
  p_Dpb->fs_ilref = calloc(1, sizeof (FrameStore*));
  if (NULL==p_Dpb->fs_ilref)
    no_mem_exit("init_dpb: p_Dpb->fs_ilref");

  if(p_Dpb->layer_id == 0)
  {
    p_Dpb->fs_ilref[0] = NULL;
  }
  else if(p_Dpb->layer_id == 1)
  {
    p_Dpb->fs_ilref[0] = alloc_frame_store();
  }

  p_Dpb->last_output_poc = INT_MIN;

#if (MVC_EXTENSION_ENABLE)
  p_Dpb->last_output_view_id = 0;
#endif

  p_Vid->last_has_mmco_5 = 0;

  p_Dpb->init_done = 1;

}


/*!
 ************************************************************************
 * \brief
 *    Free memory for decoded picture buffer.
 ************************************************************************
 */
void free_dpb(DecodedPictureBuffer *p_Dpb)
{
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  unsigned i;
  if (p_Dpb->fs)
  {
    for (i=0; i<p_Dpb->size; i++)
    {
      free_frame_store(p_Vid, p_Dpb->fs[i]);
    }
    free (p_Dpb->fs);
    p_Dpb->fs=NULL;
  }

  if (p_Dpb->fs_ref)
  {
    free (p_Dpb->fs_ref);
  }
  if (p_Dpb->fs_ltref)
  {
    free (p_Dpb->fs_ltref);
  }
  if(p_Dpb->fs_ilref)
  {
    if(p_Dpb->fs_ilref[0])
    {
      free_frame_store(p_Vid, p_Dpb->fs_ilref[0]);
      p_Dpb->fs_ilref[0] = NULL;
    }
    free(p_Dpb->fs_ilref);
  }
  p_Dpb->last_output_poc = INT_MIN;

#if (MVC_EXTENSION_ENABLE)
  p_Dpb->last_output_view_id = 0;
#endif

  p_Dpb->init_done = 0;
}


/*!
 ************************************************************************
 * \brief
 *    Allocate memory for decoded picture buffer frame stores and initialize with sane values.
 *
 * \return
 *    the allocated FrameStore structure
 ************************************************************************
 */
FrameStore* alloc_frame_store(void)
{
  FrameStore *f;

  f = calloc (1, sizeof(FrameStore));
  if (NULL==f)
    no_mem_exit("alloc_frame_store: f");

  f->is_used      = 0;
  f->is_reference = 0;
  f->is_long_term = 0;
  f->is_orig_reference = 0;

  f->is_output = 0;

  f->frame        = NULL;;
  f->top_field    = NULL;
  f->bottom_field = NULL;

#if (MVC_EXTENSION_ENABLE)
  f->view_id            = -1;
  f->inter_view_flag[0] = 0;
  f->inter_view_flag[1] = 0;
  f->anchor_pic_flag[0] = 0;
  f->anchor_pic_flag[1] = 0;
#endif

  return f;
}

void alloc_pic_motion(PicMotionParamsOld *motion, int size_y, int size_x)
{
  motion->mb_field = calloc (size_y * size_x, sizeof(byte));
  if (motion->mb_field == NULL)
    no_mem_exit("alloc_storable_picture: motion->mb_field");
}

/*!
 ************************************************************************
 * \brief
 *    Allocate memory for a stored picture.
 *
 * \param p_Vid
 *    VideoParameters
 * \param structure
 *    picture structure
 * \param size_x
 *    horizontal luma size
 * \param size_y
 *    vertical luma size
 * \param size_x_cr
 *    horizontal chroma size
 * \param size_y_cr
 *    vertical chroma size
 *
 * \return
 *    the allocated StorablePicture structure
 ************************************************************************
 */
StorablePicture* alloc_storable_picture(VideoParameters *p_Vid, PictureStructure structure, int size_x, int size_y, int size_x_cr, int size_y_cr)
{
  StorablePicture *s;
  int   nplane;
  InputParameters *p_Inp = p_Vid->p_Inp;

  //printf ("Allocating (%s) picture (x=%d, y=%d, x_cr=%d, y_cr=%d)\n", (type == FRAME)?"FRAME":(type == TOP_FIELD)?"TOP_FIELD":"BOTTOM_FIELD", size_x, size_y, size_x_cr, size_y_cr);

  s = calloc (1, sizeof(StorablePicture));
  if (NULL==s)
    no_mem_exit("alloc_storable_picture: s");

  s->imgY       = NULL;
  s->imgUV      = NULL;
  s->imgY_sub   = NULL;
  s->imgUV_sub  = NULL;
  
  s->p_img_sub[0] = NULL;
  s->p_img_sub[1] = NULL;
  s->p_img_sub[2] = NULL;

  s->de_mem = NULL;
  
#if (MVC_EXTENSION_ENABLE)  
  if ((p_Vid->nal_reference_idc != NALU_PRIORITY_DISPOSABLE) || ((p_Inp->num_of_views == 2) && p_Vid->view_id == 0)) //p_Vid->inter_view_flag[structure?structure-1: structure]))
#else
  if (p_Vid->nal_reference_idc != NALU_PRIORITY_DISPOSABLE)
#endif
  {
    if (!p_Inp->OnTheFlyFractMCP) // JLT : on-the-fly flag
    {
      get_mem4Dpel_pad(&(s->imgY_sub), 4, 4, size_y, size_x, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
      s->imgY = s->imgY_sub[0][0];

      if ( p_Inp->ChromaMCBuffer || p_Vid->P444_joined || (p_Inp->yuv_format==YUV444 && !p_Vid->P444_joined))
      {
        // UV components
        if ( p_Vid->yuv_format != YUV400 )
        {
          if ( p_Vid->yuv_format == YUV420 )
          {
            get_mem5Dpel_pad(&(s->imgUV_sub), 2, 8, 8, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
          }
          else if ( p_Vid->yuv_format == YUV422 )
          {
            get_mem5Dpel_pad(&(s->imgUV_sub), 2, 4, 8, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
          }
          else
          { // YUV444
            get_mem5Dpel_pad(&(s->imgUV_sub), 2, 4, 4, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
          }
          s->p_img_sub[1] = s->imgUV_sub[0];
          s->p_img_sub[2] = s->imgUV_sub[1];
          s->imgUV = (imgpel ***)malloc(2*sizeof(imgpel**));
          s->imgUV[0] = s->imgUV_sub[0][0][0];
          s->imgUV[1] = s->imgUV_sub[1][0][0];
        }
      }
      else
      {
        get_mem3Dpel_pad(&(s->imgUV), 2, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
      }
    }
    else if ( p_Inp->OnTheFlyFractMCP == OTF_L1 ) // OTF L1
    {
      get_mem4Dpel_pad(&(s->imgY_sub), 2, 2, size_y, size_x, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
      s->imgY = s->imgY_sub[0][0];

      if ( p_Inp->ChromaMCBuffer || p_Vid->P444_joined || (p_Inp->yuv_format==YUV444 && !p_Vid->P444_joined))
      {
        // UV components
        if ( p_Vid->yuv_format != YUV400 )
        {
          if ( p_Vid->yuv_format == YUV420 )
          {
            get_mem5Dpel_pad(&(s->imgUV_sub), 2, 4, 4, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
          }
          else if ( p_Vid->yuv_format == YUV422 )
          {
            get_mem5Dpel_pad(&(s->imgUV_sub), 2, 2, 4, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
          }
          else
          { // YUV444
            get_mem5Dpel_pad(&(s->imgUV_sub), 2, 2, 2, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
          }
          s->p_img_sub[1] = s->imgUV_sub[0];
          s->p_img_sub[2] = s->imgUV_sub[1];
          s->imgUV = (imgpel ***)malloc(2*sizeof(imgpel**));
          s->imgUV[0] = s->imgUV_sub[0][0][0];
          s->imgUV[1] = s->imgUV_sub[1][0][0];
        }
      }
      else
      {
        get_mem3Dpel_pad(&(s->imgUV), 2, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
      }
    }
    else // OTF_L2
    {
      get_mem2Dpel_pad(&(s->imgY), size_y, size_x, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
      get_mem3Dpel_pad(&(s->imgUV), 2, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
    }
  }
  else
  {
    get_mem2Dpel_pad(&(s->imgY), size_y, size_x, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
    get_mem3Dpel_pad(&(s->imgUV), 2, size_y_cr, size_x_cr, p_Vid->pad_size_uv_y, p_Vid->pad_size_uv_x);
  }  
    
  s->p_img[0] = s->imgY;
  s->p_curr_img = s->p_img[0];    
  s->p_curr_img_sub = s->p_img_sub[0];

  if (p_Vid->yuv_format != YUV400)
  {
    //get_mem3Dpel (&(s->imgUV), 2, size_y_cr, size_x_cr);
    s->p_img[1] = s->imgUV[0];
    s->p_img[2] = s->imgUV[1];
  }

  get_mem2Dmp (&s->mv_info, (size_y >> BLOCK_SHIFT), (size_x >> BLOCK_SHIFT));
  alloc_pic_motion(&s->motion, (size_y >> BLOCK_SHIFT), (size_x >> BLOCK_SHIFT));

  if( (p_Inp->separate_colour_plane_flag != 0) )
  {
    for( nplane=0; nplane<MAX_PLANE; nplane++ )
    {
      get_mem2Dmp (&s->JVmv_info[nplane], (size_y >> BLOCK_SHIFT), (size_x >> BLOCK_SHIFT));
      alloc_pic_motion(&s->JVmotion[nplane], (size_y >> BLOCK_SHIFT), (size_x >> BLOCK_SHIFT));
    }
  }

  if (p_Inp->rdopt == 3) 
  {
    errdo_alloc_storable_picture(s, p_Vid, p_Inp, size_x, size_y, size_x_cr, size_y_cr);
  }

  // Allocate memory for HME
  if(p_Inp->HMEEnable && p_Vid->nal_reference_idc != NALU_PRIORITY_DISPOSABLE)
  {
      AllocHMEMemory(&s->pHmeImage, p_Vid, size_y, size_x, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X, 1);
  }
  
  s->pic_num=0;
  s->frame_num=0;
  s->long_term_frame_idx=0;
  s->long_term_pic_num=0;
  s->used_for_reference=0;
  s->is_long_term=0;
  s->non_existing=0;
  s->is_output = 0;

  s->structure=structure;
  s->size_x = size_x;
  s->size_y = size_y;
  s->size_x_padded = size_x + 2 * IMG_PAD_SIZE_X;
  s->size_y_padded = size_y + 2 * IMG_PAD_SIZE_Y;
  s->size_x_pad = size_x + 2 * IMG_PAD_SIZE_X - 1 - MB_BLOCK_SIZE -IMG_PAD_SIZE_X;
  s->size_y_pad = size_y + 2 * IMG_PAD_SIZE_Y - 1 - MB_BLOCK_SIZE -IMG_PAD_SIZE_Y;
  s->size_x_cr = size_x_cr;
  s->size_y_cr = size_y_cr;
  s->size_x_cr_pad = (int) (size_x_cr - 1) + (p_Vid->pad_size_uv_x << 1) - (p_Vid->mb_cr_size_x) - p_Vid->pad_size_uv_x;
  s->size_y_cr_pad = (int) (size_y_cr - 1) + (p_Vid->pad_size_uv_y << 1) - (p_Vid->mb_cr_size_y) - p_Vid->pad_size_uv_y;
  s->pad_size_uv_x = p_Vid->pad_size_uv_x;
  s->pad_size_uv_y = p_Vid->pad_size_uv_y;

  s->top_field    = NULL;
  s->bottom_field = NULL;
  s->frame        = NULL;

  s->coded_frame    = 0;
  s->mb_aff_frame_flag = 0;

  memset(s->ref_pic_na, 0xff, sizeof(int)*6);

  init_stats(p_Inp, &s->stats);
  return s;
}


/*!
 ************************************************************************
 * \brief
 *    Free frame store memory.
 *
 * \param p_Vid
 *    VideoParameters
 * \param f
 *    FrameStore to be freed
 *
 ************************************************************************
 */
void free_frame_store(VideoParameters *p_Vid, FrameStore* f)
{
  if (f)
  {
    if (f->frame)
    {
      free_storable_picture(p_Vid, f->frame);
      f->frame=NULL;
    }
    if (f->top_field)
    {
      free_storable_picture(p_Vid, f->top_field);
      f->top_field=NULL;
    }
    if (f->bottom_field)
    {
      free_storable_picture(p_Vid, f->bottom_field);
      f->bottom_field=NULL;
    }
    free(f);
  }
}

void free_pic_motion(PicMotionParamsOld *motion)
{
  if (motion->mb_field)
  {
    free(motion->mb_field);
    motion->mb_field = NULL;
  }
}

static void free_frame_data_memory(StorablePicture *picture, int bFreeImage)
{
  if(picture)
  {
    if (picture->imgY_sub)
    {
      if(bFreeImage)
      {
        picture->imgY = NULL;
        if( picture->otf_flag == OTF_L1 ) // JLT : on-the-fly mode
        {
          free_mem4Dpel_pad(picture->imgY_sub, 4, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
        }
        else
        {
          free_mem4Dpel_pad(picture->imgY_sub, 16, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
        }
        picture->imgY_sub=NULL;
      }
      else
      {
        int k;
        int K = (picture->otf_flag==OTF_L1) ? (4):(16) ;
        for(k = 1; k < K; k++)
        {
          if(picture->imgY_sub[k>>2][k&3])
          {
            free_mem2Dpel_pad(picture->imgY_sub[k>>2][k&3], IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
            picture->imgY_sub[k>>2][k&3] = NULL;
          }
        }
      }
    }

    if (picture->imgUV_sub)
    {
      int iUVResX = 4*(picture->size_x/picture->size_x_cr);
      int iUVResY = 4*(picture->size_y/picture->size_y_cr);

      if ( picture->otf_flag == OTF_L1 ) // JLT : on-the-fly mode
      {
        iUVResX >>= 1 ;
        iUVResY >>= 1 ;
      }

      if(bFreeImage)
      {
        if(picture->imgUV)
        {
          free(picture->imgUV);
          picture->imgUV = NULL;
        }
        free_mem5Dpel_pad(picture->imgUV_sub, 2*iUVResY*iUVResX, picture->pad_size_uv_y, picture->pad_size_uv_x);
        picture->imgUV_sub = NULL;
      }
      else
      {
        int i, j, k;
        for(k=1; k<iUVResY*iUVResX; k++)
        {
          j = k/iUVResX;
          i = k%iUVResX;
          if(picture->imgUV_sub[0][j][i])
          {
           free_mem2Dpel_pad(picture->imgUV_sub[0][j][i], picture->pad_size_uv_y, picture->pad_size_uv_x);
           picture->imgUV_sub[0][j][i] = NULL;
          }
          if(picture->imgUV_sub[1][j][i])
          {
           free_mem2Dpel_pad(picture->imgUV_sub[1][j][i], picture->pad_size_uv_y, picture->pad_size_uv_x);
           picture->imgUV_sub[1][j][i] = NULL;
          }
        }
      }
    }

    if (picture->mv_info)
    {
      free_mem2Dmp(picture->mv_info);
      picture->mv_info = NULL;
    }

    free_pic_motion(&picture->motion);
  }
}

/*!
 ************************************************************************
 * \brief
 *    Free picture memory.
 *
 * \param p_Vid
 *    VideoParameters
 * \param p
 *    Picture to be freed
 *
 ************************************************************************
 */
void free_storable_picture(VideoParameters *p_Vid, StorablePicture* p)
{
  if (p)
  {
    InputParameters *p_Inp = p_Vid->p_Inp;
    
    if(p->imgY && !p->imgY_sub)
    {
      free_mem2Dpel_pad(p->imgY, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
      p->imgY=NULL;
    }
    
    if (p->imgUV && !p->imgUV_sub)
    {
      free_mem3Dpel_pad(p->imgUV, 2, p->pad_size_uv_y, p->pad_size_uv_x);
      p->imgUV=NULL;
    }
    
    free_frame_data_memory(p, 1);
    
    if( (p_Inp->separate_colour_plane_flag != 0) )
    {
      int nplane;
      for( nplane=0; nplane<MAX_PLANE; nplane++ )
      {
        if (p->JVmv_info[nplane])
        {
          free_mem2Dmp(p->JVmv_info[nplane]);
          p->JVmv_info[nplane] = NULL;
        }
        free_pic_motion(&p->JVmotion[nplane]);
      }
      p->p_curr_img     = NULL;
      p->p_curr_img_sub = NULL;
      p->p_img[0]       = NULL;
      p->p_img[1]       = NULL;
      p->p_img[2]       = NULL;
      p->p_img_sub[0]   = NULL;
      p->p_img_sub[1]   = NULL;
      p->p_img_sub[2]   = NULL;
    }
    
    if (p_Inp->rdopt == 3)
    {
      errdo_free_storable_picture(p);
    }
    
    //free memory for HME;
    if(p_Inp->HMEEnable)
    {
      FreeHMEMemory(&(p->pHmeImage), p_Vid, 1, IMG_PAD_SIZE_Y, IMG_PAD_SIZE_X);
    }
    
    free(p);
    p = NULL;
  }
}

/*!
 ************************************************************************
 * \brief
 *    mark FrameStore unused for reference
 *
 ************************************************************************
 */
void unmark_for_reference(FrameStore* fs)
{
  if (fs->is_used & 1)
  {
    if (fs->top_field)
    {
      fs->top_field->used_for_reference = 0;
    }
  }
  if (fs->is_used & 2)
  {
    if (fs->bottom_field)
    {
      fs->bottom_field->used_for_reference = 0;
    }
  }
  if (fs->is_used == 3)
  {
    if (fs->top_field && fs->bottom_field)
    {
      fs->top_field->used_for_reference = 0;
      fs->bottom_field->used_for_reference = 0;
    }
    fs->frame->used_for_reference = 0;
  }

  fs->is_reference = 0;

  free_frame_data_memory(fs->frame, 0);
  free_frame_data_memory(fs->top_field, 0);
  free_frame_data_memory(fs->bottom_field, 0);
}


/*!
 ************************************************************************
 * \brief
 *    mark FrameStore unused for reference and reset long term flags
 *
 ************************************************************************
 */
void unmark_for_long_term_reference(FrameStore* fs)
{
  if (fs->is_used & 1)
  {
    if (fs->top_field)
    {
      fs->top_field->used_for_reference = 0;
      fs->top_field->is_long_term = 0;
    }
  }
  if (fs->is_used & 2)
  {
    if (fs->bottom_field)
    {
      fs->bottom_field->used_for_reference = 0;
      fs->bottom_field->is_long_term = 0;
    }
  }
  if (fs->is_used == 3)
  {
    if (fs->top_field && fs->bottom_field)
    {
      fs->top_field->used_for_reference = 0;
      fs->top_field->is_long_term = 0;
      fs->bottom_field->used_for_reference = 0;
      fs->bottom_field->is_long_term = 0;
    }
    fs->frame->used_for_reference = 0;
    fs->frame->is_long_term = 0;
  }

  free_frame_data_memory(fs->frame, 0);
  free_frame_data_memory(fs->top_field, 0);
  free_frame_data_memory(fs->bottom_field, 0);

  fs->is_reference = 0;
  fs->is_long_term = 0;
}



void update_pic_num(Slice *currSlice)
{
  unsigned int i;
  //VideoParameters *p_Vid = currSlice->p_Vid;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;

  int add_top = 0, add_bottom = 0;

  int max_frame_num = currSlice->max_frame_num;


  if (currSlice->structure == FRAME)
  {
    for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
    {
      if ( p_Dpb->fs_ref[i]->is_used==3 )
      {
        if ((p_Dpb->fs_ref[i]->frame->used_for_reference)&&(!p_Dpb->fs_ref[i]->frame->is_long_term))
        {
          if( p_Dpb->fs_ref[i]->frame_num > currSlice->frame_num )
          {
            p_Dpb->fs_ref[i]->frame_num_wrap = p_Dpb->fs_ref[i]->frame_num - max_frame_num;
          }
          else
          {
            p_Dpb->fs_ref[i]->frame_num_wrap = p_Dpb->fs_ref[i]->frame_num;
          }
          p_Dpb->fs_ref[i]->frame->pic_num = p_Dpb->fs_ref[i]->frame_num_wrap;
        }
      }
    }
    // update long_term_pic_num
    for (i = 0; i < p_Dpb->ltref_frames_in_buffer; i++)
    {
      if (p_Dpb->fs_ltref[i]->is_used==3)
      {
        if (p_Dpb->fs_ltref[i]->frame->is_long_term)
        {
          p_Dpb->fs_ltref[i]->frame->long_term_pic_num = p_Dpb->fs_ltref[i]->frame->long_term_frame_idx;
        }
      }
    }
  }
  else
  {
    if (currSlice->structure == TOP_FIELD)
    {
      add_top    = 1;
      add_bottom = 0;
    }
    else
    {
      add_top    = 0;
      add_bottom = 1;
    }

    for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
    {
      if (p_Dpb->fs_ref[i]->is_reference)
      {
        if( p_Dpb->fs_ref[i]->frame_num > currSlice->frame_num )
        {
          p_Dpb->fs_ref[i]->frame_num_wrap = p_Dpb->fs_ref[i]->frame_num - max_frame_num;
        }
        else
        {
          p_Dpb->fs_ref[i]->frame_num_wrap = p_Dpb->fs_ref[i]->frame_num;
        }
        if (p_Dpb->fs_ref[i]->is_reference & 1)
        {
          p_Dpb->fs_ref[i]->top_field->pic_num = (2 * p_Dpb->fs_ref[i]->frame_num_wrap) + add_top;
        }
        if (p_Dpb->fs_ref[i]->is_reference & 2)
        {
          p_Dpb->fs_ref[i]->bottom_field->pic_num = (2 * p_Dpb->fs_ref[i]->frame_num_wrap) + add_bottom;
        }
      }
    }
    // update long_term_pic_num
    for (i=0; i<p_Dpb->ltref_frames_in_buffer; i++)
    {
      if (p_Dpb->fs_ltref[i]->is_long_term & 1)
      {
        p_Dpb->fs_ltref[i]->top_field->long_term_pic_num = 2 * p_Dpb->fs_ltref[i]->top_field->long_term_frame_idx + add_top;
      }
      if (p_Dpb->fs_ltref[i]->is_long_term & 2)
      {
        p_Dpb->fs_ltref[i]->bottom_field->long_term_pic_num = 2 * p_Dpb->fs_ltref[i]->bottom_field->long_term_frame_idx + add_bottom;
      }
    }
  }
}
/*!
 ************************************************************************
 * \brief
 *    Initialize reference lists depending on current slice type
 *
 ************************************************************************
 */
void init_lists_i_slice(Slice *currSlice)
{
  currSlice->listXsize[0] = 0;
  currSlice->listXsize[1] = 0;
}



/*!
 ************************************************************************
 * \brief
 *    Initialize reference lists for a P Slice
 *
 ************************************************************************
 */
void init_lists_p_slice(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;

  unsigned int i, j;

  int list0idx = 0;
  int listltidx = 0;

#if (MVC_EXTENSION_ENABLE)
  InputParameters *p_Inp = currSlice->p_Inp;
  int interview_pos = 0;
#endif

  FrameStore **fs_list0;
  FrameStore **fs_listlt;

  for (i = 0; i < 6; i++)
  {
    currSlice->listX[i] = calloc(MAX_LIST_SIZE, sizeof (StorablePicture*)); // +1 for reordering
    if (NULL == currSlice->listX[i])
      no_mem_exit("init_dpb: currSlice->listX[i]");
  }

  for (j = 0; j < 6; j++)
  {
    for (i = 0; i < MAX_LIST_SIZE; i++)
    {
      currSlice->listX[j][i] = NULL;
    }
    currSlice->listXsize[j]=0;
  }

  // Calculate FrameNumWrap and PicNum
  if (currSlice->structure == FRAME)
  {
    for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
    {
      if (p_Dpb->fs_ref[i]->is_used==3)
      {
        if ((p_Dpb->fs_ref[i]->frame->used_for_reference)&&(!p_Dpb->fs_ref[i]->frame->is_long_term))
        {
          currSlice->listX[0][list0idx++] = p_Dpb->fs_ref[i]->frame;
        }
      }
    }
    // order list 0 by PicNum
    qsort((void *)currSlice->listX[0], list0idx, sizeof(StorablePicture*), compare_pic_by_pic_num_desc);
    currSlice->listXsize[0] = (char) list0idx;
    //printf("listX[0] (PicNum): "); for (i=0; i<list0idx; i++){printf ("%d  ", currSlice->listX[0][i]->pic_num);} printf("\n");
    // long term handling
    for (i=0; i<p_Dpb->ltref_frames_in_buffer; i++)
    {
      if (p_Dpb->fs_ltref[i]->is_used==3)
      {
        if (p_Dpb->fs_ltref[i]->frame->is_long_term)
        {
          currSlice->listX[0][list0idx++] = p_Dpb->fs_ltref[i]->frame;
        }
      }
    }
    qsort((void *)&currSlice->listX[0][(short) currSlice->listXsize[0]], list0idx - currSlice->listXsize[0], sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);
    currSlice->listXsize[0] = (char) list0idx;
  }
  else
  {
    fs_list0 = calloc(p_Dpb->size, sizeof (FrameStore*));
    if (NULL==fs_list0)
      no_mem_exit("init_lists: fs_list0");
    fs_listlt = calloc(p_Dpb->size, sizeof (FrameStore*));
    if (NULL==fs_listlt)
      no_mem_exit("init_lists: fs_listlt");

    for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
    {
      if (p_Dpb->fs_ref[i]->is_reference)
      {
        fs_list0[list0idx++] = p_Dpb->fs_ref[i];
      }
    }

    qsort((void *)fs_list0, list0idx, sizeof(FrameStore*), compare_fs_by_frame_num_desc);

    //printf("fs_list0 (FrameNum): "); for (i=0; i<list0idx; i++){printf ("%d  ", fs_list0[i]->frame_num_wrap);} printf("\n");

    currSlice->listXsize[0] = 0;
    gen_pic_list_from_frame_list(currSlice->structure, fs_list0, list0idx, currSlice->listX[0], &currSlice->listXsize[0], 0);

    //printf("listX[0] (PicNum): "); for (i=0; i < currSlice->listXsize[0]; i++){printf ("%d  ", currSlice->listX[0][i]->pic_num);} printf("\n");

    // long term handling
    for (i=0; i<p_Dpb->ltref_frames_in_buffer; i++)
    {
      fs_listlt[listltidx++]=p_Dpb->fs_ltref[i];
    }

    qsort((void *)fs_listlt, listltidx, sizeof(FrameStore*), compare_fs_by_lt_pic_idx_asc);

    gen_pic_list_from_frame_list(currSlice->structure, fs_listlt, listltidx, currSlice->listX[0], &currSlice->listXsize[0], 1);

    free(fs_list0);
    free(fs_listlt);
  }
  currSlice->listXsize[1] = 0;

#if (MVC_EXTENSION_ENABLE && !SIMULCAST_ENABLE)
  // MVC interview pictures list
  if(p_Inp->num_of_views == 2 && p_Vid->view_id == 1)
  {
    StorablePicture *base_ref;
    if(currSlice->structure == FRAME)
    {
      currSlice->listXsize[0] = (char) imin (currSlice->listXsize[0], currSlice->num_ref_idx_active[LIST_0]);
      currSlice->listXsize[1] = (char) imin (currSlice->listXsize[1], currSlice->num_ref_idx_active[LIST_1]);
      list0idx = currSlice->listXsize[0];
      /*
      interview_pos = p_Dpb->used_size - 1;
      base_ref      = p_Dpb->fs[interview_pos]->frame;
      */
      interview_pos = 0;
      base_ref      = p_Dpb->fs_ilref[interview_pos]->frame;
      //assert(base_ref->proc_flag);
      if (base_ref->used_for_reference || base_ref->inter_view_flag[0])
      {
        //p_Dpb->fs[interview_pos]->frame_num_wrap = -99;
        p_Dpb->fs_ilref[interview_pos]->frame_num_wrap = -99;
        currSlice->listX[0][list0idx++] = base_ref;
        currSlice->listXsize[0] = (char) list0idx;
      }
    }
    else  //FIELD
    {
      int cur_inter_view_flag = currSlice->structure != BOTTOM_FIELD ? 0 : 1;
      currSlice->listXsize[0] = (char) imin (currSlice->listXsize[0], currSlice->num_ref_idx_active[LIST_0]);
      currSlice->listXsize[1] = (char) imin (currSlice->listXsize[1], currSlice->num_ref_idx_active[LIST_1]);
      list0idx = currSlice->listXsize[0];
      /*
      interview_pos = (currSlice->structure != BOTTOM_FIELD) ? p_Dpb->used_size - 1: p_Dpb->used_size - 2; // top field is at the last position
      base_ref = currSlice->structure!=BOTTOM_FIELD?p_Dpb->fs[interview_pos]->top_field:p_Dpb->fs[interview_pos]->bottom_field;
      */
      interview_pos = 0;
      base_ref = currSlice->structure!=BOTTOM_FIELD? p_Dpb->fs_ilref[interview_pos]->top_field:p_Dpb->fs_ilref[interview_pos]->bottom_field;
      if (base_ref->used_for_reference || base_ref->inter_view_flag[cur_inter_view_flag])
      {
        //p_Dpb->fs[interview_pos]->frame_num_wrap = -99;
        p_Dpb->fs_ilref[interview_pos]->frame_num_wrap = -99;
        currSlice->listX[0][list0idx++] = base_ref;
        currSlice->listXsize[0] = (char) list0idx;
      }
    }
  }
#endif

  // set max size
if (p_Vid->is_hme == 0)
{
#if (MVC_EXTENSION_ENABLE)
  if(p_Vid->num_of_layers == 2 && p_Inp->MVCInterViewReorder && p_Vid->view_id == 1)
  {
    currSlice->listXsize[0] = (char) imin (currSlice->listXsize[0], currSlice->num_ref_idx_active[LIST_0] + 1);
    currSlice->listXsize[1] = (char) imin (currSlice->listXsize[1], currSlice->num_ref_idx_active[LIST_1]);
  }
  else
#endif
  {
    currSlice->listXsize[0] = (char) imin (currSlice->listXsize[0], currSlice->num_ref_idx_active[LIST_0]);
    currSlice->listXsize[1] = (char) imin (currSlice->listXsize[1], currSlice->num_ref_idx_active[LIST_1]);
  }
}

  // set the unused list entries to NULL
  for (i=currSlice->listXsize[0]; i< (MAX_LIST_SIZE) ; i++)
  {
    currSlice->listX[0][i] = NULL;
  }
  for (i=currSlice->listXsize[1]; i< (MAX_LIST_SIZE) ; i++)
  {
    currSlice->listX[1][i] = NULL;
  }

#if PRINTREFLIST
#if (MVC_EXTENSION_ENABLE)
  // print out for debug purpose
  if(p_Inp->num_of_views==2 && p_Vid->current_slice_nr==0)
  {
    if(currSlice->listXsize[0]>0)
    {
      printf("\n");
      printf(" ** (CurViewID:%d) %s Ref Pic List 0 ****\n", p_Vid->view_id, currSlice->structure==FRAME ? "FRM":(currSlice->structure==TOP_FIELD ? "TOP":"BOT"));
      for(i=0; i<(unsigned int)(currSlice->listXsize[0]); i++)  //ref list 0
      {
        printf("   %2d -> POC: %4d PicNum: %4d ViewID: %d\n", i, currSlice->listX[0][i]->poc, currSlice->listX[0][i]->pic_num, currSlice->listX[0][i]->view_id);
      }
    }
  }
#endif
#endif
}


/*!
 ************************************************************************
 * \brief
 *    Initialize reference lists for a B Slice
 *
 ************************************************************************
 */
void init_lists_b_slice(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;

  unsigned int i;
  int j;

  int list0idx = 0;
  int list0idx_1 = 0;
  int listltidx = 0;

#if (MVC_EXTENSION_ENABLE)
  InputParameters *p_Inp = currSlice->p_Inp;
  int list1idx = 0;
  int interview_pos = 0;
#endif

  FrameStore **fs_list0;
  FrameStore **fs_list1;
  FrameStore **fs_listlt;


  for (i = 0; i < 6; i++)
  {
    currSlice->listX[i] = calloc(MAX_LIST_SIZE, sizeof (StorablePicture*)); // +1 for reordering
    if (NULL == currSlice->listX[i])
      no_mem_exit("init_dpb: currSlice->listX[i]");
  }

  for (j = 0; j < 6; j++)
  {
    for (i = 0; i < MAX_LIST_SIZE; i++)
    {
      currSlice->listX[j][i] = NULL;
    }
    currSlice->listXsize[j]=0;
  }

  {
    // B-Slice
    if (currSlice->structure == FRAME)
    {
      for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++)
      {
        if (p_Dpb->fs_ref[i]->is_used==3)
        {
          if ((p_Dpb->fs_ref[i]->frame->used_for_reference)&&(!p_Dpb->fs_ref[i]->frame->is_long_term))
          {
            if (currSlice->framepoc > p_Dpb->fs_ref[i]->frame->poc)
            {
              currSlice->listX[0][list0idx++] = p_Dpb->fs_ref[i]->frame;
            }
          }
        }
      }
      qsort((void *)currSlice->listX[0], list0idx, sizeof(StorablePicture*), compare_pic_by_poc_desc);

      //get the backward reference picture (POC>current POC) in list0;
      list0idx_1 = list0idx;
      for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
      {
        if (p_Dpb->fs_ref[i]->is_used==3)
        {
          if ((p_Dpb->fs_ref[i]->frame->used_for_reference)&&(!p_Dpb->fs_ref[i]->frame->is_long_term))
          {
            if (currSlice->framepoc < p_Dpb->fs_ref[i]->frame->poc)
            {
              currSlice->listX[0][list0idx++] = p_Dpb->fs_ref[i]->frame;
            }
          }
        }
      }
      qsort((void *)&currSlice->listX[0][list0idx_1], list0idx-list0idx_1, sizeof(StorablePicture*), compare_pic_by_poc_asc);

      for (j=0; j<list0idx_1; j++)
      {
        currSlice->listX[1][list0idx-list0idx_1+j]=currSlice->listX[0][j];
      }
      for (j=list0idx_1; j<list0idx; j++)
      {
        currSlice->listX[1][j-list0idx_1]=currSlice->listX[0][j];
      }

      currSlice->listXsize[0] = currSlice->listXsize[1] = (char) list0idx;

      //      printf("currSlice->listX[0] currPoc=%d (Poc): ", currSlice->framepoc); for (i=0; i<currSlice->listXsize[0]; i++){printf ("%d  ", currSlice->listX[0][i]->poc);} printf("\n");
      //      printf("currSlice->listX[1] currPoc=%d (Poc): ", currSlice->framepoc); for (i=0; i<currSlice->listXsize[1]; i++){printf ("%d  ", currSlice->listX[1][i]->poc);} printf("\n");

      // long term handling
      for (i=0; i<p_Dpb->ltref_frames_in_buffer; i++)
      {
        if (p_Dpb->fs_ltref[i]->is_used==3)
        {
          if (p_Dpb->fs_ltref[i]->frame->is_long_term)
          {
            currSlice->listX[0][list0idx]   = p_Dpb->fs_ltref[i]->frame;
            currSlice->listX[1][list0idx++] = p_Dpb->fs_ltref[i]->frame;
          }
        }
      }
      qsort((void *)&currSlice->listX[0][(short) currSlice->listXsize[0]], list0idx - currSlice->listXsize[0], sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);
      qsort((void *)&currSlice->listX[1][(short) currSlice->listXsize[0]], list0idx - currSlice->listXsize[0], sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);
      currSlice->listXsize[0] = currSlice->listXsize[1] = (char) list0idx;
    }
    else
    {
      fs_list0 = calloc(p_Dpb->size, sizeof (FrameStore*));
      if (NULL==fs_list0)
        no_mem_exit("init_lists: fs_list0");
      fs_list1 = calloc(p_Dpb->size, sizeof (FrameStore*));
      if (NULL==fs_list1)
        no_mem_exit("init_lists: fs_list1");
      fs_listlt = calloc(p_Dpb->size, sizeof (FrameStore*));
      if (NULL==fs_listlt)
        no_mem_exit("init_lists: fs_listlt");

      currSlice->listXsize[0] = 0;
      currSlice->listXsize[1] = 1;

      for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
      {
        if (p_Dpb->fs_ref[i]->is_used)
        {
          if (currSlice->ThisPOC >= p_Dpb->fs_ref[i]->poc)
          {
            fs_list0[list0idx++] = p_Dpb->fs_ref[i];
          }
        }
      }
      qsort((void *)fs_list0, list0idx, sizeof(FrameStore*), compare_fs_by_poc_desc);
      list0idx_1 = list0idx;
      for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
      {
        if (p_Dpb->fs_ref[i]->is_used)
        {
          if (currSlice->ThisPOC < p_Dpb->fs_ref[i]->poc)
          {
            fs_list0[list0idx++] = p_Dpb->fs_ref[i];
          }
        }
      }
      qsort((void *)&fs_list0[list0idx_1], list0idx-list0idx_1, sizeof(FrameStore*), compare_fs_by_poc_asc);

      for (j=0; j<list0idx_1; j++)
      {
        fs_list1[list0idx-list0idx_1+j]=fs_list0[j];
      }
      for (j=list0idx_1; j<list0idx; j++)
      {
        fs_list1[j-list0idx_1]=fs_list0[j];
      }

      //      printf("fs_list0 currPoc=%d (Poc): ", currSlice->ThisPOC); for (i=0; i<list0idx; i++){printf ("%d  ", fs_list0[i]->poc);} printf("\n");
      //      printf("fs_list1 currPoc=%d (Poc): ", currSlice->ThisPOC); for (i=0; i<list0idx; i++){printf ("%d  ", fs_list1[i]->poc);} printf("\n");

      currSlice->listXsize[0] = 0;
      currSlice->listXsize[1] = 0;
      gen_pic_list_from_frame_list(currSlice->structure, fs_list0, list0idx, currSlice->listX[0], &currSlice->listXsize[0], 0);
      gen_pic_list_from_frame_list(currSlice->structure, fs_list1, list0idx, currSlice->listX[1], &currSlice->listXsize[1], 0);

      //      printf("currSlice->listX[0] currPoc=%d (Poc): ", currSlice->framepoc); for (i=0; i<currSlice->listXsize[0]; i++){printf ("%d  ", currSlice->listX[0][i]->poc);} printf("\n");
      //      printf("currSlice->listX[1] currPoc=%d (Poc): ", currSlice->framepoc); for (i=0; i<currSlice->listXsize[1]; i++){printf ("%d  ", currSlice->listX[1][i]->poc);} printf("\n");

      // long term handling
      for (i=0; i<p_Dpb->ltref_frames_in_buffer; i++)
      {
        fs_listlt[listltidx++]=p_Dpb->fs_ltref[i];
      }

      qsort((void *)fs_listlt, listltidx, sizeof(FrameStore*), compare_fs_by_lt_pic_idx_asc);

      gen_pic_list_from_frame_list(currSlice->structure, fs_listlt, listltidx, currSlice->listX[0], &currSlice->listXsize[0], 1);
      gen_pic_list_from_frame_list(currSlice->structure, fs_listlt, listltidx, currSlice->listX[1], &currSlice->listXsize[1], 1);

      free(fs_list0);
      free(fs_list1);
      free(fs_listlt);
    }
  }

  if ((currSlice->listXsize[0] == currSlice->listXsize[1]) && (currSlice->listXsize[0] > 1))
  {
    // check if lists are identical, if yes swap first two elements of currSlice->listX[1]
    int diff=0;
    for (j = 0; j< currSlice->listXsize[0]; j++)
    {
      if (currSlice->listX[0][j] != currSlice->listX[1][j])
      {
        diff = 1;
        break;
      }
    }
    if (!diff)
    {
      StorablePicture *tmp_s = currSlice->listX[1][0];
      currSlice->listX[1][0]=currSlice->listX[1][1];
      currSlice->listX[1][1]=tmp_s;
    }
  }

#if (MVC_EXTENSION_ENABLE && !SIMULCAST_ENABLE)
  // MVC interview pictures list
  if(p_Inp->num_of_views == 2 && p_Vid->view_id == 1)
  { 
    StorablePicture *base_ref;
    if(currSlice->structure == FRAME)
    {
      currSlice->listXsize[0] = (char) imin (currSlice->listXsize[0], currSlice->num_ref_idx_active[LIST_0]);
      currSlice->listXsize[1] = (char) imin (currSlice->listXsize[1], currSlice->num_ref_idx_active[LIST_1]);
      list0idx = currSlice->listXsize[0];
      list1idx = currSlice->listXsize[1];
      /*
      interview_pos = p_Dpb->used_size-1;
      base_ref = p_Dpb->fs[interview_pos]->frame;
      */
      interview_pos = 0;
      base_ref = p_Dpb->fs_ilref[interview_pos]->frame;

      if (base_ref->used_for_reference || base_ref->inter_view_flag[0])
      {
        //p_Dpb->fs[interview_pos]->frame_num_wrap = -99;
        p_Dpb->fs_ilref[interview_pos]->frame_num_wrap = -99;
        currSlice->listX[0][list0idx++] = base_ref;
        currSlice->listX[1][list1idx++] = base_ref;
        currSlice->listXsize[0] = (char) list0idx;
        currSlice->listXsize[1] = (char) list1idx;
      }
    }
    else  //FIELD
    {
      int cur_inter_view_flag = currSlice->structure != BOTTOM_FIELD ? 0 : 1;
      currSlice->listXsize[0] = (char) imin (currSlice->listXsize[0], currSlice->num_ref_idx_active[LIST_0]);
      currSlice->listXsize[1] = (char) imin (currSlice->listXsize[1], currSlice->num_ref_idx_active[LIST_1]);
      list0idx = currSlice->listXsize[0];
      list1idx = currSlice->listXsize[1];
      /*
      interview_pos = currSlice->structure!=BOTTOM_FIELD ? p_Dpb->used_size - 1: p_Dpb->used_size - 2; // top field is at the last position
      base_ref = currSlice->structure != BOTTOM_FIELD ? p_Dpb->fs[interview_pos]->top_field:p_Dpb->fs[interview_pos]->bottom_field;
      */
      interview_pos = 0;
      base_ref = currSlice->structure != BOTTOM_FIELD ? p_Dpb->fs_ilref[interview_pos]->top_field:p_Dpb->fs_ilref[interview_pos]->bottom_field;
      if (base_ref->used_for_reference || base_ref->inter_view_flag[cur_inter_view_flag])
      {
        //p_Dpb->fs[interview_pos]->frame_num_wrap = -99;
        p_Dpb->fs_ilref[interview_pos]->frame_num_wrap = -99;
        currSlice->listX[0][list0idx++] = base_ref;
        currSlice->listX[1][list1idx++] = base_ref;
        currSlice->listXsize[0] = (char) list0idx;
        currSlice->listXsize[1] = (char) list1idx;
      }
    }
  }
#endif

  // set max size
#if (MVC_EXTENSION_ENABLE)
  if(p_Vid->num_of_layers == 2 && p_Inp->MVCInterViewReorder && p_Vid->view_id == 1)
  {
    currSlice->listXsize[0] = (char) imin (currSlice->listXsize[0], currSlice->num_ref_idx_active[LIST_0] + 1);
    currSlice->listXsize[1] = (char) imin (currSlice->listXsize[1], currSlice->num_ref_idx_active[LIST_1]);
  }
  else
#endif
  {
    currSlice->listXsize[0] = (char) imin (currSlice->listXsize[0], currSlice->num_ref_idx_active[LIST_0]);
    currSlice->listXsize[1] = (char) imin (currSlice->listXsize[1], currSlice->num_ref_idx_active[LIST_1]);
  }

  // set the unused list entries to NULL
  for (i=currSlice->listXsize[0]; i< (MAX_LIST_SIZE) ; i++)
  {
    currSlice->listX[0][i] = NULL;
  }
  for (i=currSlice->listXsize[1]; i< (MAX_LIST_SIZE) ; i++)
  {
    currSlice->listX[1][i] = NULL;
  }

#if PRINTREFLIST
#if (MVC_EXTENSION_ENABLE)
  // print out for debug purpose
  if(p_Inp->num_of_views==2 && p_Vid->current_slice_nr==0)
  {
    if((currSlice->listXsize[0]>0) || (currSlice->listXsize[1]>0))
      printf("\n");
    if(currSlice->listXsize[0]>0)
    {
      printf(" ** (CurViewID:%d) %s Ref Pic List 0 ****\n", p_Vid->view_id, currSlice->structure==FRAME ? "FRM":(currSlice->structure==TOP_FIELD ? "TOP":"BOT"));
      for(i=0; i<(unsigned int)(currSlice->listXsize[0]); i++)  //ref list 0
      {
        printf("   %2d -> POC: %4d PicNum: %4d ViewID: %d\n", i, currSlice->listX[0][i]->poc, currSlice->listX[0][i]->pic_num, currSlice->listX[0][i]->view_id);
      }
    }
    if(currSlice->listXsize[1]>0)
    {
      printf(" ** (CurViewID:%d) %s Ref Pic List 1 ****\n", p_Vid->view_id, currSlice->structure==FRAME ? "FRM":(currSlice->structure==TOP_FIELD ? "TOP":"BOT"));
      for(i=0; i<(unsigned int)(currSlice->listXsize[1]); i++)  //ref list 1
      {
        printf("   %2d -> POC: %4d PicNum: %4d ViewID: %d\n", i, currSlice->listX[1][i]->poc, currSlice->listX[1][i]->pic_num, currSlice->listX[1][i]->view_id);
      }
    }
  }
#endif
#endif
}

/*!
 ************************************************************************
 * \brief
 *    Initialize listX[2..5] from lists 0 and 1
 *    listX[2]: list0 for current_field==top
 *    listX[3]: list1 for current_field==top
 *    listX[4]: list0 for current_field==bottom
 *    listX[5]: list1 for current_field==bottom
 *
 ************************************************************************
 */
void init_mbaff_lists(Slice *currSlice)
{
  unsigned j;
  int i;

  for (i=2;i<6;i++)
  {
    if(currSlice->listX[i])
    {
      for (j=0; j<MAX_LIST_SIZE; j++)
      {
        currSlice->listX[i][j] = NULL;
      }
    }
    currSlice->listXsize[i]=0;
  }

  for (i = 0; i < currSlice->listXsize[0]; i++)
  {
    currSlice->listX[2][2*i  ] = currSlice->listX[0][i]->top_field;
    currSlice->listX[2][2*i+1] = currSlice->listX[0][i]->bottom_field;
    currSlice->listX[4][2*i  ] = currSlice->listX[0][i]->bottom_field;
    currSlice->listX[4][2*i+1] = currSlice->listX[0][i]->top_field;
  }
  currSlice->listXsize[2] = currSlice->listXsize[4] = currSlice->listXsize[0] * 2;

  for (i = 0; i < currSlice->listXsize[1]; i++)
  {
    currSlice->listX[3][2*i  ] = currSlice->listX[1][i]->top_field;
    currSlice->listX[3][2*i+1] = currSlice->listX[1][i]->bottom_field;
    currSlice->listX[5][2*i  ] = currSlice->listX[1][i]->bottom_field;
    currSlice->listX[5][2*i+1] = currSlice->listX[1][i]->top_field;
  }
  currSlice->listXsize[3] = currSlice->listXsize[5] = currSlice->listXsize[1] * 2;
}

 /*!
 ************************************************************************
 * \brief
 *    Returns short term pic with given picNum
 *
 ************************************************************************
 */
StorablePicture *get_short_term_pic(Slice *currSlice, DecodedPictureBuffer *p_Dpb, int picNum)
{
  unsigned i;

  for (i = 0; i < p_Dpb->ref_frames_in_buffer; i++)
  {
    if (currSlice->structure == FRAME)
    {
      if (p_Dpb->fs_ref[i]->is_reference == 3)
        if ((!p_Dpb->fs_ref[i]->frame->is_long_term)&&(p_Dpb->fs_ref[i]->frame->pic_num == picNum))
          return p_Dpb->fs_ref[i]->frame;
    }
    else
    {
      if (p_Dpb->fs_ref[i]->is_reference & 1)
        if ((!p_Dpb->fs_ref[i]->top_field->is_long_term)&&(p_Dpb->fs_ref[i]->top_field->pic_num == picNum))
          return p_Dpb->fs_ref[i]->top_field;
      if (p_Dpb->fs_ref[i]->is_reference & 2)
        if ((!p_Dpb->fs_ref[i]->bottom_field->is_long_term)&&(p_Dpb->fs_ref[i]->bottom_field->pic_num == picNum))
          return p_Dpb->fs_ref[i]->bottom_field;
    }
  }
  return NULL;
}


/*!
 ************************************************************************
 * \brief
 *    Reordering process for short-term reference pictures
 *
 ************************************************************************
 */
void reorder_short_term(Slice *currSlice, DecodedPictureBuffer *p_Dpb, int cur_list, int picNumLX, int *refIdxLX)
{
  StorablePicture **RefPicListX = currSlice->listX[cur_list];
  int cIdx, nIdx;

  StorablePicture *picLX;

  picLX = get_short_term_pic(currSlice, p_Dpb, picNumLX);

  for( cIdx = currSlice->num_ref_idx_active[cur_list]; cIdx > *refIdxLX; cIdx-- )
    RefPicListX[ cIdx ] = RefPicListX[ cIdx - 1];

  RefPicListX[ (*refIdxLX)++ ] = picLX;

  nIdx = *refIdxLX;

  for( cIdx = *refIdxLX; cIdx <= currSlice->num_ref_idx_active[cur_list]; cIdx++ )
  {
    if (RefPicListX[ cIdx ])
      if( (RefPicListX[ cIdx ]->is_long_term ) ||  (RefPicListX[ cIdx ]->pic_num != picNumLX ))
        RefPicListX[ nIdx++ ] = RefPicListX[ cIdx ];
  }
}


/*!
 ************************************************************************
 * \brief
 *    Reordering process for long-term reference pictures
 *
 ************************************************************************
 */
static void reorder_long_term(Slice *currSlice, DecodedPictureBuffer *p_Dpb, StorablePicture **RefPicListX, int cur_list, int frame_no, int *refIdxLX)
{
  int cIdx, nIdx;
  int LongTermPicNum = currSlice->long_term_pic_idx[cur_list][frame_no];

  StorablePicture *picLX;

  picLX = get_long_term_pic(currSlice, p_Dpb, LongTermPicNum);

  for( cIdx = currSlice->num_ref_idx_active[cur_list]; cIdx > *refIdxLX; cIdx-- )
    RefPicListX[ cIdx ] = RefPicListX[ cIdx - 1];

  RefPicListX[ (*refIdxLX)++ ] = picLX;

  nIdx = *refIdxLX;

  for( cIdx = *refIdxLX; cIdx <= currSlice->num_ref_idx_active[cur_list]; cIdx++ )
    if( (!RefPicListX[ cIdx ]->is_long_term ) ||  (RefPicListX[ cIdx ]->long_term_pic_num != LongTermPicNum ))
      RefPicListX[ nIdx++ ] = RefPicListX[ cIdx ];
}



/*!
 ************************************************************************
 * \brief
 *    Reordering process for reference picture lists
 *
 ************************************************************************
 */
void reorder_ref_pic_list(Slice *currSlice, int cur_list)
{
  int *modification_of_pic_nums_idc  = currSlice->modification_of_pic_nums_idc[cur_list];
  int *abs_diff_pic_num_minus1 = currSlice->abs_diff_pic_num_minus1[cur_list];
  DecodedPictureBuffer *p_Dpb = currSlice->p_Dpb;
  int i;

  int maxPicNum, currPicNum, picNumLXNoWrap, picNumLXPred, picNumLX;
  int refIdxLX = 0;

  if (currSlice->structure==FRAME)
  {
    maxPicNum  = currSlice->max_frame_num;
    currPicNum = currSlice->frame_num;
  }
  else
  {
    maxPicNum  = 2 * currSlice->max_frame_num;
    currPicNum = 2 * currSlice->frame_num + 1;
  }

  picNumLXPred = currPicNum;

  for (i=0; modification_of_pic_nums_idc[i]!=3; i++)
  {
    if (modification_of_pic_nums_idc[i] > 3)
      error ("Invalid modification_of_pic_nums_idc command", 500);

    if (modification_of_pic_nums_idc[i] < 2)
    {
      if (modification_of_pic_nums_idc[i] == 0)
      {
        if( picNumLXPred - ( abs_diff_pic_num_minus1[i] + 1 ) < 0 )
          picNumLXNoWrap = picNumLXPred - ( abs_diff_pic_num_minus1[i] + 1 ) + maxPicNum;
        else
          picNumLXNoWrap = picNumLXPred - ( abs_diff_pic_num_minus1[i] + 1 );
      }
      else // (modification_of_pic_nums_idc[i] == 1)
      {
        if( picNumLXPred + ( abs_diff_pic_num_minus1[i] + 1 )  >=  maxPicNum )
          picNumLXNoWrap = picNumLXPred + ( abs_diff_pic_num_minus1[i] + 1 ) - maxPicNum;
        else
          picNumLXNoWrap = picNumLXPred + ( abs_diff_pic_num_minus1[i] + 1 );
      }
      picNumLXPred = picNumLXNoWrap;

      if( picNumLXNoWrap > currPicNum )
        picNumLX = picNumLXNoWrap - maxPicNum;
      else
        picNumLX = picNumLXNoWrap;

      reorder_short_term(currSlice, p_Dpb, cur_list, picNumLX, &refIdxLX);
    }
    else //(modification_of_pic_nums_idc[i] == 2)
    {
      reorder_long_term (currSlice, p_Dpb, currSlice->listX[cur_list], cur_list, i, &refIdxLX);
    }
  }

  // that's a definition
  currSlice->listXsize[cur_list] = currSlice->num_ref_idx_active[cur_list];
}




/*!
 ************************************************************************
 * \brief
 *    Perform Memory management for idr pictures
 *
 ************************************************************************
 */
static void idr_memory_management(DecodedPictureBuffer *p_Dpb, StorablePicture* p, FrameFormat *output)
{
  unsigned i;
  VideoParameters *p_Vid = p_Dpb->p_Vid;

  assert (p_Vid->currentPicture->idr_flag);

  if (p_Vid->no_output_of_prior_pics_flag)
  {
    // free all stored pictures
    for (i=0; i<p_Dpb->used_size; i++)
    {
      // reset all reference settings
      free_frame_store(p_Vid, p_Dpb->fs[i]);
      p_Dpb->fs[i] = alloc_frame_store();
    }
    for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
    {
      p_Dpb->fs_ref[i]=NULL;
    }
    for (i=0; i<p_Dpb->ltref_frames_in_buffer; i++)
    {
      p_Dpb->fs_ltref[i]=NULL;
    }
    p_Dpb->used_size=0;
  }
  else
  {
    flush_dpb(p_Dpb, output);
  }
  p_Dpb->last_picture = NULL;

  update_ref_list(p_Dpb);
  update_ltref_list(p_Dpb);
  p_Dpb->last_output_poc = INT_MIN;
#if (MVC_EXTENSION_ENABLE)
  p_Dpb->last_output_view_id = 0;
#endif

  if (p_Vid->long_term_reference_flag)
  {
    p_Dpb->max_long_term_pic_idx = 0;
    p->is_long_term           = 1;
    p->long_term_frame_idx    = 0;
  }
  else
  {
    p_Dpb->max_long_term_pic_idx = -1;
    p->is_long_term           = 0;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Perform Sliding window decoded reference picture marking process
 *
 ************************************************************************
 */
static void sliding_window_memory_management(DecodedPictureBuffer *p_Dpb, StorablePicture* p)
{
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  unsigned i;

  // if this is a reference pic with sliding sliding window, unmark first ref frame
  if (p_Dpb->ref_frames_in_buffer == imax(1, p_Vid->active_sps->num_ref_frames) - p_Dpb->ltref_frames_in_buffer)
  {
    for (i = 0; i < p_Dpb->used_size; i++)
    {
      if (p_Dpb->fs[i]->is_reference && (!(p_Dpb->fs[i]->is_long_term)))
      {
        unmark_for_reference(p_Dpb->fs[i]);
        update_ref_list(p_Dpb);
        break;
      }
    }
  }

  p->is_long_term = 0;
}






/*!
 ************************************************************************
 * \brief
 *    Perform Adaptive memory control decoded reference picture marking process
 ************************************************************************
 */
static void adaptive_memory_management(DecodedPictureBuffer *p_Dpb, StorablePicture* p, FrameFormat *output)
{
  DecRefPicMarking_t *tmp_drpm;
  VideoParameters *p_Vid = p_Dpb->p_Vid;

  p_Vid->last_has_mmco_5 = 0;

  assert (!p_Vid->currentPicture->idr_flag);
  assert (p_Vid->adaptive_ref_pic_buffering_flag);

  while (p_Vid->dec_ref_pic_marking_buffer)
  {
    tmp_drpm = p_Vid->dec_ref_pic_marking_buffer;
    switch (tmp_drpm->memory_management_control_operation)
    {
      case 0:
        if (tmp_drpm->Next != NULL)
        {
          error ("memory_management_control_operation = 0 not last operation in buffer", 500);
        }
        break;
      case 1:
        mm_unmark_short_term_for_reference(p_Dpb, p, tmp_drpm->difference_of_pic_nums_minus1);
        update_ref_list(p_Dpb);
        break;
      case 2:
        mm_unmark_long_term_for_reference(p_Dpb, p, tmp_drpm->long_term_pic_num);
        update_ltref_list(p_Dpb);
        break;
      case 3:
        mm_assign_long_term_frame_idx(p_Dpb, p, tmp_drpm->difference_of_pic_nums_minus1, tmp_drpm->long_term_frame_idx);
        update_ref_list(p_Dpb);
        update_ltref_list(p_Dpb);
        break;
      case 4:
        mm_update_max_long_term_frame_idx (p_Dpb, tmp_drpm->max_long_term_frame_idx_plus1);
        update_ltref_list(p_Dpb);
        break;
      case 5:
        mm_unmark_all_short_term_for_reference(p_Dpb);
        mm_unmark_all_long_term_for_reference(p_Dpb);
        p_Vid->last_has_mmco_5 = 1;
        break;
      case 6:
        mm_mark_current_picture_long_term(p_Dpb, p, tmp_drpm->long_term_frame_idx);
        check_num_ref(p_Dpb);
        break;
      default:
        error ("invalid memory_management_control_operation in buffer", 500);
    }
    p_Vid->dec_ref_pic_marking_buffer = tmp_drpm->Next;
    free (tmp_drpm);
  }
  if ( p_Vid->last_has_mmco_5 )
  {
    p->pic_num = p->frame_num = 0;

    switch (p->structure)
    {
    case TOP_FIELD:
      {
        p->poc = p->top_poc = p_Vid->toppoc =0;
        break;
      }
    case BOTTOM_FIELD:
      {
        p->poc = p->bottom_poc = p_Vid->bottompoc = 0;
        break;
      }
    case FRAME:
      {
        p->top_poc    -= p->poc;
        p->bottom_poc -= p->poc;

        p_Vid->toppoc = p->top_poc;
        p_Vid->bottompoc = p->bottom_poc;

        p->poc = imin (p->top_poc, p->bottom_poc);
        p_Vid->framepoc = p->poc;
        break;
      }
    }
    p_Vid->ThisPOC = p->poc;
    flush_dpb(p_Vid->p_Dpb_layer[0], output);
    if(p_Vid->num_of_layers >1)
     flush_dpb(p_Vid->p_Dpb_layer[1], output);
  }
}


/*!
 ************************************************************************
 * \brief
 *    Store a picture in DPB. This includes cheking for space in DPB and
 *    flushing frames.
 *    If we received a frame, we need to check for a new store, if we
 *    got a field, check if it's the second field of an already allocated
 *    store.
 *
 * \param p_Vid
 *    VideoParameters
 * \param p
 *    Picture to be stored
 * \param output
 *    FrameFormat for output
 *
 ************************************************************************
 */
void store_picture_in_dpb(DecodedPictureBuffer *p_Dpb, StorablePicture* p, FrameFormat *output)
{
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  unsigned i;
  int poc, pos;
   
  // diagnostics
  //printf ("Storing (%s) non-ref pic with frame_num #%d\n", (p->type == FRAME)?"FRAME":(p->type == TOP_FIELD)?"TOP_FIELD":"BOTTOM_FIELD", p->pic_num);
  // if frame, check for new store,
  assert (p!=NULL);

  p->otf_flag = p_Vid->p_Inp->OnTheFlyFractMCP ; // JLT : on-the-fly

  p->used_for_reference = (p_Vid->nal_reference_idc != NALU_PRIORITY_DISPOSABLE);

  p->type = p_Vid->type;

#if (MVC_EXTENSION_ENABLE)
  if(p_Vid->p_Inp->num_of_views==2)
  {
    p->view_id = p_Vid->view_id;

    if(p->structure==FRAME)
    {
      p->inter_view_flag[0] = p->inter_view_flag[1] = p_Vid->inter_view_flag[0];
      p->anchor_pic_flag[0] = p->anchor_pic_flag[1] = p_Vid->anchor_pic_flag[0];
    }
    else
    {
      if(p->structure==TOP_FIELD)
      {
        p->inter_view_flag[0] = p_Vid->inter_view_flag[0];
        p->anchor_pic_flag[0] = p_Vid->anchor_pic_flag[0];
      }
      else  // BOTTOM_FIELD
      {
        p->inter_view_flag[1] = p_Vid->inter_view_flag[1];
        p->anchor_pic_flag[1] = p_Vid->anchor_pic_flag[1];
      }
    }
    
    if(p_Vid->view_id == 0)
    {
      process_picture_in_dpb_s(p_Vid, p);
      p_Vid->proc_picture = clone_storable_picture(p_Vid, p);
    }
  }
#endif

  p_Vid->last_has_mmco_5 = 0;
  p_Vid->last_pic_bottom_field = (p_Vid->structure == BOTTOM_FIELD);

  if (p_Vid->currentPicture->idr_flag)
  {
    idr_memory_management(p_Dpb, p, output);
    if(p_Vid->num_of_layers > 1)
      idr_memory_management(p_Vid->p_Dpb_layer[1], p, output);
  }
  else
  {
    // adaptive memory management
    if (p->used_for_reference && (p_Vid->adaptive_ref_pic_buffering_flag))
      adaptive_memory_management(p_Dpb, p, output);
  }

  if ((p->structure==TOP_FIELD)||(p->structure==BOTTOM_FIELD))
  {
    // check for frame store with same pic_number
    if (p_Dpb->last_picture)
    {
      if ((int)p_Dpb->last_picture->frame_num == p->pic_num)
      {
        if (((p->structure==TOP_FIELD)&&(p_Dpb->last_picture->is_used==2))||((p->structure==BOTTOM_FIELD)&&(p_Dpb->last_picture->is_used==1)))
        {
          if ((p->used_for_reference && (p_Dpb->last_picture->is_orig_reference!=0))||
              (!p->used_for_reference && (p_Dpb->last_picture->is_orig_reference==0)))
          {
            insert_picture_in_dpb(p_Vid, p_Dpb->last_picture, p);
            update_ref_list(p_Dpb);
            update_ltref_list(p_Dpb);
#if (DUMP_DPB)
            printf("\nstore_picture_in_dpb inserting complementary field\n");
#endif
            dump_dpb(p_Dpb);
            p_Dpb->last_picture = NULL;
            return;
          }
        }
      }
    }
  }

  // this is a frame or a field which has no stored complementary field

  // sliding window, if necessary
  if ((!p_Vid->currentPicture->idr_flag) && (p->used_for_reference && (!p_Vid->adaptive_ref_pic_buffering_flag)))
  {
    sliding_window_memory_management(p_Dpb, p);
  }

  // first try to remove unused frames
  if (p_Dpb->used_size == p_Dpb->size)
  {
    remove_unused_frame_from_dpb(p_Dpb);
  }

  // then output frames until one can be removed
  while (p_Dpb->used_size == p_Dpb->size)
  {
    // non-reference frames may be output directly
    if (!p->used_for_reference)
    {
      get_smallest_poc(p_Dpb, &poc, &pos);

#if (MVC_EXTENSION_ENABLE)
      if(p_Vid->p_Inp->num_of_views==2)
      {
        if (((-1==pos) || (p->poc < poc)))
        {
          direct_output(p_Vid, p, output, (p_Dpb->layer_id ? p_Vid->p_dec2: p_Vid->p_dec));
          return;
        }
      }
      else
#endif
      {
        if ((-1==pos) || (p->poc < poc))
        {
          direct_output(p_Vid, p, output, p_Vid->p_dec);
          return;
        }
      }
    }
    // flush a frame
    output_one_frame_from_dpb(p_Dpb, output);
  }

  // check for duplicate frame number in short term reference buffer
  if ((p->used_for_reference)&&(!p->is_long_term))
  {
    for (i=0; i<p_Dpb->ref_frames_in_buffer; i++)
    {
      if (p_Dpb->fs_ref[i]->frame_num == p->frame_num)
      {
        error("duplicate frame_num in short-term reference picture buffer", 500);
      }
    }
  }
  // store at end of buffer
  insert_picture_in_dpb(p_Vid, p_Dpb->fs[p_Dpb->used_size],p);

  if (p->structure != FRAME)
  {
    p_Dpb->last_picture = p_Dpb->fs[p_Dpb->used_size];
  }
  else
  {
    p_Dpb->last_picture = NULL;
  }

  p_Dpb->used_size++;

  update_ref_list(p_Dpb);
  update_ltref_list(p_Dpb);

  check_num_ref(p_Dpb);
#if (DUMP_DPB)
  printf("\nEnd of store_picture_in_dpb\n");
#endif
  dump_dpb(p_Dpb);
}

#if (MVC_EXTENSION_ENABLE)
/*!
 ************************************************************************
 * \brief
 *    Reference process the frame picture and insert into the buffer 
 *    if the reference processed top field has already
 *    been stored 
 *
 * \param p_Vid
 *    VideoParameters
 * \param p
 *    StorablePicture to be inserted
 * \param output
 *    FrameFormat for output
 *
 ************************************************************************
 */
void replace_top_proc_pic_with_frame(DecodedPictureBuffer *p_Dpb, StorablePicture* p)
{
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  StorablePicture *proc_picture = p_Vid->proc_picture;
  FrameStore* fs = NULL;
  unsigned found;//i, 

  assert (p!=NULL);
  assert (p->structure==FRAME);
  assert (proc_picture==NULL);
  assert (p_Vid->p_Inp->ProfileIDC <=STEREO_HIGH);
  process_picture_in_dpb_s(p_Vid, p);
  proc_picture = clone_storable_picture(p_Vid, p);

  proc_picture->used_for_reference = (p_Vid->nal_reference_idc != NALU_PRIORITY_DISPOSABLE);
  proc_picture->type = p_Vid->type;
  p_Vid->last_has_mmco_5=0;
  p_Vid->last_pic_bottom_field = (p_Vid->structure == BOTTOM_FIELD);

  // upsample a reference picture
  if( (p_Vid->p_Inp->separate_colour_plane_flag != 0) )
  {
    UnifiedOneForthPix_JV(p_Vid, 0, proc_picture);
    UnifiedOneForthPix_JV(p_Vid, 1, proc_picture);
    UnifiedOneForthPix_JV(p_Vid, 2, proc_picture);
  }
  else
  {
    UnifiedOneForthPix(p_Vid, proc_picture);
    if (p_Vid->p_Inp->HMEEnable)
      GenerateHMELayers(p_Vid, proc_picture);
  }

  found=0;

  if(p_Dpb->fs_ilref[0] && (p_Dpb->fs_ilref[0]->frame_num == p_Vid->frame_num) && (p_Dpb->fs_ilref[0]->is_used == 1))
  {
    found = 1;
    fs = p_Dpb->fs_ilref[0];
  }

  if (!found)
  {
    printf("replace_top_proc_pic_with_frame: Warning! Reference processed field not found. Processed frame not inserted in DPB\n");
    free_storable_picture(p_Vid, proc_picture);
  }
  else
  {
    free_storable_picture(p_Vid, fs->top_field);
    fs->top_field=NULL;
    fs->frame=proc_picture;
    fs->is_used = 3;
    if (p->used_for_reference)
    {
      fs->is_reference = 3;
      if (p->is_long_term)
      {
        fs->is_long_term = 3;
      }
    }
    // generate field views
    dpb_split_field(p_Vid, fs);
    fs->top_field->view_id = fs->bottom_field->view_id = fs->frame->view_id;
    update_ref_list(p_Dpb);
    update_ltref_list(p_Dpb);
  }

  p_Vid->proc_picture = NULL;
}
#endif

/*!
 ************************************************************************
 * \brief
 *    Insert the frame picture into the buffer if the top field has already
 *    been stored for the coding decision
 *
 * \param p_Vid
 *    VideoParameters
 * \param p
 *    StorablePicture to be inserted
 * \param output
 *    FrameFormat for output
 *
 ************************************************************************
 */
void replace_top_pic_with_frame(DecodedPictureBuffer *p_Dpb, StorablePicture* p, FrameFormat *output)
{
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  FrameStore* fs = NULL;
  unsigned i, found;

  assert (p!=NULL);
  assert (p->structure==FRAME);

  p->used_for_reference = (p_Vid->nal_reference_idc != NALU_PRIORITY_DISPOSABLE);
  p->type = p_Vid->type;
  // upsample a reference picture

#if (MVC_EXTENSION_ENABLE)
  if (p->used_for_reference || (p_Vid->p_Inp->num_of_views==2 && p_Vid->view_id == 0))
#else
  if (p->used_for_reference)
#endif
  {
    if( (p_Vid->p_Inp->separate_colour_plane_flag != 0) )
    {
      UnifiedOneForthPix_JV(p_Vid, 0, p);
      UnifiedOneForthPix_JV(p_Vid, 1, p);
      UnifiedOneForthPix_JV(p_Vid, 2, p);
    }
    else
    {
      UnifiedOneForthPix(p_Vid, p);
      if (p_Vid->p_Inp->HMEEnable)
        GenerateHMELayers(p_Vid, p);
    }
  }

  found=0;

  for (i = 0; i < p_Dpb->used_size; i++)
  {
    if((p_Dpb->fs[i]->frame_num == p_Vid->frame_num) && (p_Dpb->fs[i]->is_used == 1))
    {
      found=1;
      fs = p_Dpb->fs[i];
      break;
    }
  }

#if (MVC_EXTENSION_ENABLE)
  if(p_Vid->p_Inp->num_of_views == 2)
  {
    if (p_Dpb->layer_id == 0)
    {
      replace_top_proc_pic_with_frame(p_Vid->p_Dpb_layer[1], p);
    }
  }
#endif

  if (!found)
  {
    // this should only happen for non-reference pictures when the dpb is full of reference pics
#if (MVC_EXTENSION_ENABLE)
    if(p_Vid->p_Inp->num_of_views == 2)
    {
      if (p_Dpb->layer_id == 0)    // view 0
        direct_output_paff(p_Vid, p, output, p_Vid->p_dec);
      else                        // view 1
        direct_output_paff(p_Vid, p, output, p_Vid->p_dec2);
    }
    else
#endif
    direct_output_paff(p_Vid, p, output, p_Vid->p_dec);
  }
  else
  {
    free_storable_picture(p_Vid, fs->top_field);
    fs->top_field = NULL;
    fs->frame = p;
    fs->is_used = 3;
    if (p->used_for_reference)
    {
      fs->is_reference = 3;
      if (p->is_long_term)
      {
        fs->is_long_term = 3;
      }
    }
    // generate field views
    dpb_split_field(p_Vid, fs);
    update_ref_list(p_Dpb);
    update_ltref_list(p_Dpb);
  }
#if (DUMP_DPB)
  printf("\nReplace Top pic with Frame\n");
  dump_dpb(p_Dpb);
#endif
}


/*!
 ************************************************************************
 * \brief
 *    Insert the picture into the DPB. A free DPB position is necessary
 *    for frames, .
 *
 * \param p_Vid
 *    VideoParameters
 * \param fs
 *    FrameStore into which the picture will be inserted
 * \param p
 *    StorablePicture to be inserted
 *
 ************************************************************************
 */
static void insert_picture_in_dpb(VideoParameters *p_Vid, FrameStore* fs, StorablePicture* p)
{
  //  printf ("insert (%s) pic with frame_num #%d, poc %d\n", (p->structure == FRAME)?"FRAME":(p->structure == TOP_FIELD)?"TOP_FIELD":"BOTTOM_FIELD", p->pic_num, p->poc);
  assert (p!=NULL);
  assert (fs!=NULL);

  // upsample a reference picture
  if (p->used_for_reference)
  {    
    if( (p_Vid->p_Inp->separate_colour_plane_flag != 0) )
    {
      UnifiedOneForthPix_JV(p_Vid, 0, p);
      UnifiedOneForthPix_JV(p_Vid, 1, p);
      UnifiedOneForthPix_JV(p_Vid, 2, p);
    }
    else
    {
      UnifiedOneForthPix(p_Vid, p);
      if (p_Vid->p_Inp->HMEEnable)
      {
        GenerateHMELayers(p_Vid, p);
      }
    }
  }
#if (MVC_EXTENSION_ENABLE)
  else if(p_Vid->p_Inp->num_of_views==2 && p->inter_view_flag[p->structure>>1])
  {
    UnifiedOneForthPix(p_Vid, p);
    if (p_Vid->p_Inp->HMEEnable)
      GenerateHMELayers(p_Vid, p);
  }
#endif

  switch (p->structure)
  {
  case FRAME:
    fs->frame = p;
    fs->is_used = 3;
    if (p->used_for_reference)
    {
      fs->is_reference = 3;
      fs->is_orig_reference = 3;
      if (p->is_long_term)
      {
        fs->is_long_term = 3;
        fs->long_term_frame_idx = p->long_term_frame_idx;
      }
    }
    // generate field views
    dpb_split_field(p_Vid, fs);
#if (MVC_EXTENSION_ENABLE)
    if(p_Vid->p_Inp->num_of_views==2)
    {
      fs->anchor_pic_flag[0] = fs->anchor_pic_flag[1] = p->anchor_pic_flag[0];
      fs->inter_view_flag[0] = fs->inter_view_flag[1] = p->inter_view_flag[0];
    }
    if (fs->top_field)
    {
      fs->top_field->view_id = p->view_id;
    }
    if (fs->bottom_field)
    {
      fs->bottom_field->view_id = p->view_id;
    }
#endif
    break;
  case TOP_FIELD:
    fs->top_field = p;
    fs->is_used |= 1;
    if (p->used_for_reference)
    {
      fs->is_reference |= 1;
      fs->is_orig_reference |= 1;
      if (p->is_long_term)
      {
        fs->is_long_term |= 1;
        fs->long_term_frame_idx = p->long_term_frame_idx;
      }
    }
    if (fs->is_used == 3)
    {
      // generate frame view
      dpb_combine_field(p_Vid, fs);
    }
    else
    {
      fs->poc = p->poc;
      gen_field_ref_ids(p);
    }
#if (MVC_EXTENSION_ENABLE)
    if(p_Vid->p_Inp->num_of_views==2)
    {
      fs->anchor_pic_flag[0] = p->anchor_pic_flag[0];
      fs->inter_view_flag[0] = p->inter_view_flag[0];
    }
    fs->top_field->view_id = p->view_id;
#endif
    break;
  case BOTTOM_FIELD:
    fs->bottom_field = p;
    fs->is_used |= 2;
    if (p->used_for_reference)
    {
      fs->is_reference |= 2;
      fs->is_orig_reference |= 2;
      if (p->is_long_term)
      {
        fs->is_long_term |= 2;
        fs->long_term_frame_idx = p->long_term_frame_idx;
      }
    }
    if (fs->is_used == 3)
    {
      // generate frame view
      dpb_combine_field(p_Vid, fs);
#if (MVC_EXTENSION_ENABLE)
      fs->frame->view_id = p->view_id;
#endif
    }
    else
    {
      fs->poc = p->poc;
      gen_field_ref_ids(p);
    }
#if (MVC_EXTENSION_ENABLE)
    if(p_Vid->p_Inp->num_of_views==2)
    {
      fs->anchor_pic_flag[1] = p->anchor_pic_flag[1];
      fs->inter_view_flag[1] = p->inter_view_flag[1];
    }
    fs->bottom_field->view_id = p->view_id;
#endif
    break;
  }
  fs->frame_num = p->pic_num;
  fs->is_output = p->is_output;
  fs->is_inter_layer = FALSE;

#if (MVC_EXTENSION_ENABLE)
  fs->view_id = p->view_id;
#endif
}



/*!
 ************************************************************************
 * \brief
 *    remove one frame from DPB
 ************************************************************************
 */
void remove_frame_from_dpb(DecodedPictureBuffer *p_Dpb, int pos)
{  
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  FrameStore* fs = p_Dpb->fs[pos];
  FrameStore* tmp;
  unsigned i;

  //printf ("remove frame with frame_num #%d\n", fs->frame_num);
  switch (fs->is_used)
  {
  case 3:
    free_storable_picture(p_Vid, fs->frame);
    free_storable_picture(p_Vid, fs->top_field);
    free_storable_picture(p_Vid, fs->bottom_field);
    fs->frame=NULL;
    fs->top_field=NULL;
    fs->bottom_field=NULL;
    break;
  case 2:
    free_storable_picture(p_Vid, fs->bottom_field);
    fs->bottom_field=NULL;
    break;
  case 1:
    free_storable_picture(p_Vid, fs->top_field);
    fs->top_field=NULL;
    break;
  case 0:
    break;
  default:
    error("invalid frame store type",500);
  }
  fs->is_used = 0;
  fs->is_long_term = 0;
  fs->is_reference = 0;
  fs->is_orig_reference = 0;

  // move empty framestore to end of buffer
  tmp = p_Dpb->fs[pos];

  for (i=pos; i<p_Dpb->used_size-1;i++)
  {
    p_Dpb->fs[i] = p_Dpb->fs[i+1];
  }
  p_Dpb->fs[p_Dpb->used_size-1] = tmp;
  p_Dpb->used_size--;
}


/*!
 ************************************************************************
 * \brief
 *    Flush a picture from DPB which is no longer needed.
 ************************************************************************
 */
static int flush_unused_frame_from_dpb(DecodedPictureBuffer *p_Dpb)
{
  unsigned i;

  // check for frames that were already output and no longer used for reference
  for (i = 0; i < p_Dpb->used_size; i++)
  {
    if (p_Dpb->fs[i]->is_output && (!is_used_for_reference(p_Dpb->fs[i])))
    {
      remove_frame_from_dpb(p_Dpb, i);
      return 1;
    }
  }
  return 0;
}



/*!
 ************************************************************************
 * \brief
 *    Output one picture stored in the DPB.
 ************************************************************************
 */
static void output_one_frame_from_dpb(DecodedPictureBuffer *p_Dpb, FrameFormat *output)
{
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  InputParameters *p_Inp = p_Dpb->p_Inp;
  int poc, pos;
  //diagnostics
  if (p_Dpb->used_size < 1)
  {
    error("Cannot output frame, DPB empty.",150);
  }

  // find smallest POC
  get_smallest_poc(p_Dpb, &poc, &pos);

  if(pos==-1)
  {
    error("no frames for output available", 150);
  }

  // call the output function
  //  printf ("output frame with frame_num #%d, poc %d (p_Dpb-> p_Dpb->size=%d, p_Dpb->used_size=%d)\n", p_Dpb->fs[pos]->frame_num, p_Dpb->fs[pos]->frame->poc, p_Dpb->size, p_Dpb->used_size);
#if (MVC_EXTENSION_ENABLE)
  if(p_Inp->num_of_views==2)
  {
    if(p_Dpb->fs[pos]->view_id==1)
      write_stored_frame(p_Vid, p_Dpb->fs[pos], output, p_Vid->p_dec2);
    else
      write_stored_frame(p_Vid, p_Dpb->fs[pos], output, p_Vid->p_dec);
  }
  else
#endif
    write_stored_frame(p_Vid, p_Dpb->fs[pos], output, p_Vid->p_dec);

  // if redundant picture in use, output POC may be not in ascending order
  if(p_Inp->redundant_pic_flag == 0)
  {
    if (p_Dpb->last_output_poc >= poc)
    {
      error ("output POC must be in ascending order", 150);
    }
  }
  p_Dpb->last_output_poc = poc;

  // free frame store and move empty store to end of buffer
  if (!is_used_for_reference(p_Dpb->fs[pos]))
  {
    remove_frame_from_dpb(p_Dpb, pos);
  }
}



/*!
 ************************************************************************
 * \brief
 *    All stored picture are output. Should be called to empty the buffer
 ************************************************************************
 */
void flush_dpb(DecodedPictureBuffer *p_Dpb, FrameFormat *output)
{
  unsigned i;

  // diagnostics
  // printf("Flush remaining frames from the dpb. p_Dpb->size=%d, p_Dpb->used_size=%d\n",p_Dpb->size,p_Dpb->used_size);

  // mark all frames unused
  //VideoParameters *p_Vid = p_Dpb->p_Vid;
  for (i=0; i<p_Dpb->used_size; i++)
  {
    unmark_for_reference (p_Dpb->fs[i]);
  }

  while (flush_unused_frame_from_dpb(p_Dpb)) ;

  // output frames in POC order
  while (p_Dpb->used_size)
  {
    output_one_frame_from_dpb(p_Dpb, output);
  }

  p_Dpb->last_output_poc = INT_MIN;
#if (MVC_EXTENSION_ENABLE)
  p_Dpb->last_output_view_id = 0;
#endif
}


static void gen_field_ref_ids(StorablePicture *p)
{
  int i,j;
   //! Generate Frame parameters from field information.
  for (i = 0; i < (p->size_x >> 2); i++)
  {
    for (j = 0; j < (p->size_y >> 2); j++)
    {
        p->mv_info[j][i].field_frame=1;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Extract top field from a frame
 ************************************************************************
 */
void dpb_split_field(VideoParameters *p_Vid, FrameStore *fs)
{
  int i, j, k, ii, jj, jj4;
  int idiv,jdiv;
  int currentmb;

  int twosz16 = 2 * (fs->frame->size_x >> 4);
  StorablePicture *fs_top, *fs_btm; 
  StorablePicture *frame = fs->frame;

  fs->poc = frame->poc;

  if (!p_Vid->active_sps->frame_mbs_only_flag)
  {
    fs_top = fs->top_field    = alloc_storable_picture(p_Vid, TOP_FIELD,    frame->size_x, frame->size_y >> 1, frame->size_x_cr, frame->size_y_cr >> 1);
    fs_btm = fs->bottom_field = alloc_storable_picture(p_Vid, BOTTOM_FIELD, frame->size_x, frame->size_y >> 1, frame->size_x_cr, frame->size_y_cr >> 1);

    for (i = 0; i < (frame->size_y >> 1); i++)
    {
      memcpy(fs_top->imgY[i], frame->imgY[i*2], frame->size_x*sizeof(imgpel));
      memcpy(fs_btm->imgY[i], frame->imgY[i*2 + 1], frame->size_x*sizeof(imgpel));
    }

    for (k = 0; k < 2; k++)
    {
      for (i=0; i< (frame->size_y_cr >> 1); i++)
      {
        memcpy(fs_top->imgUV[k][i], frame->imgUV[k][i*2], frame->size_x_cr*sizeof(imgpel));
        memcpy(fs_btm->imgUV[k][i], frame->imgUV[k][i*2 + 1], frame->size_x_cr*sizeof(imgpel));
      }
    }
    if(frame->used_for_reference)
    {
      if( (p_Vid->p_Inp->separate_colour_plane_flag != 0) )
      {
        UnifiedOneForthPix_JV(p_Vid, 0, fs->top_field);
        UnifiedOneForthPix_JV(p_Vid, 1, fs->top_field);
        UnifiedOneForthPix_JV(p_Vid, 2, fs->top_field);
        UnifiedOneForthPix_JV(p_Vid, 0, fs->bottom_field);
        UnifiedOneForthPix_JV(p_Vid, 1, fs->bottom_field);
        UnifiedOneForthPix_JV(p_Vid, 2, fs->bottom_field);
      }
      else
      {
        UnifiedOneForthPix(p_Vid, fs->top_field);
        UnifiedOneForthPix(p_Vid, fs->bottom_field);
        if (p_Vid->p_Inp->HMEEnable)
        {
          GenerateHMELayers(p_Vid, fs->top_field);
          GenerateHMELayers(p_Vid, fs->bottom_field);
        }
      }
    }
    fs->top_field->poc = frame->top_poc;
    fs->bottom_field->poc =  frame->bottom_poc;

    fs->top_field->frame_poc =  frame->frame_poc;

    fs->top_field->bottom_poc =fs->bottom_field->bottom_poc =  frame->bottom_poc;
    fs->top_field->top_poc =fs->bottom_field->top_poc =  frame->top_poc;
    fs->bottom_field->frame_poc =  frame->frame_poc;

    fs->top_field->used_for_reference = fs->bottom_field->used_for_reference
      = frame->used_for_reference;
    fs->top_field->is_long_term = fs->bottom_field->is_long_term
      = frame->is_long_term;
    fs->long_term_frame_idx = fs->top_field->long_term_frame_idx
      = fs->bottom_field->long_term_frame_idx
      = frame->long_term_frame_idx;

    fs->top_field->coded_frame = fs->bottom_field->coded_frame = 1;
    fs->top_field->mb_aff_frame_flag = fs->bottom_field->mb_aff_frame_flag
      = frame->mb_aff_frame_flag;

    frame->top_field    = fs->top_field;
    frame->bottom_field = fs->bottom_field;
    frame->frame = frame;
    fs->top_field->bottom_field = fs->bottom_field;
    fs->top_field->frame        = fs->frame;
    fs->top_field->top_field = fs->top_field;
    fs->bottom_field->top_field = fs->top_field;
    fs->bottom_field->frame     = frame;
    fs->bottom_field->bottom_field = fs->bottom_field;

    fs->top_field->chroma_format_idc = fs->bottom_field->chroma_format_idc = frame->chroma_format_idc;
    fs->top_field->chroma_mask_mv_x  = fs->bottom_field->chroma_mask_mv_x  = frame->chroma_mask_mv_x;
    fs->top_field->chroma_mask_mv_y  = fs->bottom_field->chroma_mask_mv_y  = frame->chroma_mask_mv_y;
    fs->top_field->chroma_shift_x    = fs->bottom_field->chroma_shift_x    = frame->chroma_shift_x;
    fs->top_field->chroma_shift_y    = fs->bottom_field->chroma_shift_y    = frame->chroma_shift_y;
    
    // JLT : otf flag
    fs->top_field->otf_flag = fs->bottom_field->otf_flag = frame->otf_flag ;
  }
  else
  {
    fs->top_field=NULL;
    fs->bottom_field=NULL;
    frame->top_field=NULL;
    frame->bottom_field=NULL;
    frame->frame = frame;
  }


  if (!p_Vid->active_sps->frame_mbs_only_flag && fs->frame->mb_aff_frame_flag)
  {
    for (j=0 ; j< frame->size_y >> 3; j++)
    {
      jj = (j >> 2)*8 + (j & 0x03);
      jj4 = jj + 4;
      jdiv = (j >> 1);
      for (i=0 ; i < (frame->size_x >> 2); i++)
      {
        idiv=i >> 2;

        currentmb = twosz16*(jdiv >> 1)+ (idiv)*2 + (jdiv & 0x01);
        // Assign field mvs attached to MB-Frame buffer to the proper buffer
        if (fs->frame->motion.mb_field[currentmb])
        {
          fs->bottom_field->mv_info[j][i].field_frame = fs->top_field->mv_info[j][i].field_frame=1;
          fs->frame->mv_info[2*j][i].field_frame = fs->frame->mv_info[2*j+1][i].field_frame = 1;

          fs->bottom_field->mv_info[j][i].mv[LIST_0] = fs->frame->mv_info[jj4][i].mv[LIST_0];
          fs->bottom_field->mv_info[j][i].mv[LIST_1] = fs->frame->mv_info[jj4][i].mv[LIST_1];
          fs->bottom_field->mv_info[j][i].ref_idx[LIST_0] = fs->frame->mv_info[jj4][i].ref_idx[LIST_0];
          fs->bottom_field->mv_info[j][i].ref_idx[LIST_1] = fs->frame->mv_info[jj4][i].ref_idx[LIST_1];

          fs->top_field->mv_info[j][i].mv[LIST_0] = fs->frame->mv_info[jj][i].mv[LIST_0];
          fs->top_field->mv_info[j][i].mv[LIST_1] = fs->frame->mv_info[jj][i].mv[LIST_1];
          fs->top_field->mv_info[j][i].ref_idx[LIST_0] = fs->frame->mv_info[jj][i].ref_idx[LIST_0];
          fs->top_field->mv_info[j][i].ref_idx[LIST_1] = fs->frame->mv_info[jj][i].ref_idx[LIST_1];
        }
      }
    }
  }

  //! Generate field MVs from Frame MVs
  if (!p_Vid->active_sps->frame_mbs_only_flag)
  {
    for (j=0 ; j<fs->frame->size_y >> 3 ; j++)
    {
      jj = 2* RSD(j);
      jdiv = j >> 1;
      for (i=0 ; i<fs->frame->size_x >> 2 ; i++)
      {
        ii = RSD(i);
        idiv = i >> 2;

        currentmb = twosz16*(jdiv >> 1)+ (idiv)*2 + (jdiv & 0x01);

        if (!fs->frame->mb_aff_frame_flag  || !fs->frame->motion.mb_field[currentmb])
        {
          fs->frame->mv_info[2*j+1][i].field_frame = fs->frame->mv_info[2*j][i].field_frame = 0;

          fs->top_field->mv_info[j][i].field_frame = fs->bottom_field->mv_info[j][i].field_frame = 0;

          fs->top_field->mv_info[j][i].mv[LIST_0] = fs->bottom_field->mv_info[j][i].mv[LIST_0] = fs->frame->mv_info[jj][ii].mv[LIST_0];
          fs->top_field->mv_info[j][i].mv[LIST_1] = fs->bottom_field->mv_info[j][i].mv[LIST_1] = fs->frame->mv_info[jj][ii].mv[LIST_1];

          // Scaling of references is done here since it will not affect spatial direct (2*0 =0)
          if (fs->frame->mv_info[jj][ii].ref_idx[LIST_0] == -1)
            fs->top_field->mv_info[j][i].ref_idx[LIST_0] = fs->bottom_field->mv_info[j][i].ref_idx[LIST_0] = - 1;
          else
          {
            fs->top_field->mv_info[j][i].ref_idx[LIST_0] = fs->bottom_field->mv_info[j][i].ref_idx[LIST_0] = fs->frame->mv_info[jj][ii].ref_idx[LIST_0];
          }

          if (fs->frame->mv_info[jj][ii].ref_idx[LIST_1] == -1)
            fs->top_field->mv_info[j][i].ref_idx[LIST_1] = fs->bottom_field->mv_info[j][i].ref_idx[LIST_1] = - 1;
          else
          {
            fs->top_field->mv_info[j][i].ref_idx[LIST_1] = fs->bottom_field->mv_info[j][i].ref_idx[LIST_1] = fs->frame->mv_info[jj][ii].ref_idx[LIST_1];
          }
        }
        else
        {
          fs->frame->mv_info[2*j+1][i].field_frame = fs->frame->mv_info[2*j][i].field_frame= fs->frame->motion.mb_field[currentmb];
        }
      }
    }
  }
  else
  {
    for (j=0 ; j<fs->frame->size_y >> 2 ; j++)
    {
      for (i=0 ; i<fs->frame->size_x >> 2 ; i++)
      {
        frame->mv_info[j][i].field_frame = 0;
      }
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Generate a frame from top and bottom fields,
 *    YUV components and display information only
 ************************************************************************
 */
void dpb_combine_field_yuv(VideoParameters *p_Vid, FrameStore *fs)
{
  int i, j;

  if (!fs->frame)
  {
    fs->frame = alloc_storable_picture(p_Vid, FRAME, fs->top_field->size_x, fs->top_field->size_y*2, fs->top_field->size_x_cr, fs->top_field->size_y_cr*2);
  }

  for (i=0; i<fs->top_field->size_y; i++)
  {
    memcpy(fs->frame->imgY[i*2],     fs->top_field->imgY[i]   , fs->top_field->size_x * sizeof(imgpel));     // top field
    memcpy(fs->frame->imgY[i*2 + 1], fs->bottom_field->imgY[i], fs->bottom_field->size_x * sizeof(imgpel)); // bottom field
  }

  for (j = 0; j < 2; j++)
  {
    for (i=0; i<fs->top_field->size_y_cr; i++)
    {
      memcpy(fs->frame->imgUV[j][i*2],     fs->top_field->imgUV[j][i],    fs->top_field->size_x_cr*sizeof(imgpel));
      memcpy(fs->frame->imgUV[j][i*2 + 1], fs->bottom_field->imgUV[j][i], fs->bottom_field->size_x_cr*sizeof(imgpel));
    }
  }

  fs->poc=fs->frame->poc =fs->frame->frame_poc = imin (fs->top_field->poc, fs->bottom_field->poc);

  fs->bottom_field->frame_poc=fs->top_field->frame_poc=fs->frame->poc;

  fs->bottom_field->top_poc=fs->frame->top_poc=fs->top_field->poc;
  fs->top_field->bottom_poc=fs->frame->bottom_poc=fs->bottom_field->poc;

  fs->frame->used_for_reference = (fs->top_field->used_for_reference && fs->bottom_field->used_for_reference );
  fs->frame->is_long_term = (fs->top_field->is_long_term && fs->bottom_field->is_long_term );

  if (fs->frame->is_long_term)
    fs->frame->long_term_frame_idx = fs->long_term_frame_idx;

  fs->frame->top_field    = fs->top_field;
  fs->frame->bottom_field = fs->bottom_field;
  fs->frame->frame = fs->frame;

  fs->frame->coded_frame = 0;

  fs->frame->chroma_format_idc = fs->top_field->chroma_format_idc;
  fs->frame->chroma_mask_mv_x  = fs->top_field->chroma_mask_mv_x;
  fs->frame->chroma_mask_mv_y  = fs->top_field->chroma_mask_mv_y;
  fs->frame->chroma_shift_x    = fs->top_field->chroma_shift_x;
  fs->frame->chroma_shift_y    = fs->top_field->chroma_shift_y;

  fs->frame->frame_cropping_flag = fs->top_field->frame_cropping_flag;
  if (fs->frame->frame_cropping_flag)
  {
    fs->frame->frame_crop_top_offset = fs->top_field->frame_crop_top_offset;
    fs->frame->frame_crop_bottom_offset = fs->top_field->frame_crop_bottom_offset;
    fs->frame->frame_crop_left_offset = fs->top_field->frame_crop_left_offset;
    fs->frame->frame_crop_right_offset = fs->top_field->frame_crop_right_offset;
  }

  fs->top_field->frame = fs->bottom_field->frame = fs->frame;
  fs->top_field->top_field = fs->top_field;
  fs->top_field->bottom_field = fs->bottom_field;
  fs->bottom_field->top_field = fs->top_field;
  fs->bottom_field->bottom_field = fs->bottom_field;
  
  // otf flag
  fs->frame->otf_flag = fs->top_field->otf_flag;
}


/*!
 ************************************************************************
 * \brief
 *    Generate a frame from top and bottom fields
 ************************************************************************
 */
void dpb_combine_field(VideoParameters *p_Vid, FrameStore *fs)
{
  int i,j, jj, jj4;

  dpb_combine_field_yuv(p_Vid, fs);
  if (fs->frame->used_for_reference)
  {
    if( (p_Vid->p_Inp->separate_colour_plane_flag != 0) )
    {
      UnifiedOneForthPix_JV(p_Vid, 0, fs->frame);
      UnifiedOneForthPix_JV(p_Vid, 1, fs->frame);
      UnifiedOneForthPix_JV(p_Vid, 2, fs->frame);
    }
    else
    {
      UnifiedOneForthPix(p_Vid, fs->frame);
      if (p_Vid->p_Inp->HMEEnable)
        GenerateHMELayers(p_Vid, fs->frame);
    }
  }

  //! Generate Frame parameters from field information.
  for (j=0 ; j<fs->top_field->size_y >> 2 ; j++)
  {
    jj = 8*(j >> 2) + (j & 0x03);
    jj4 = jj + 4;
    for (i=0 ; i<fs->top_field->size_x >> 2 ; i++)
    {
      fs->frame->mv_info[jj][i].field_frame= fs->frame->mv_info[jj4][i].field_frame = 1;

      fs->frame->mv_info[jj][i].mv[LIST_0] = fs->top_field->mv_info[j][i].mv[LIST_0];
      fs->frame->mv_info[jj][i].mv[LIST_1] = fs->top_field->mv_info[j][i].mv[LIST_1];

      fs->frame->mv_info[jj4][i].mv[LIST_0] = fs->bottom_field->mv_info[j][i].mv[LIST_0];
      fs->frame->mv_info[jj4][i].mv[LIST_1] = fs->bottom_field->mv_info[j][i].mv[LIST_1];

      fs->top_field->mv_info[j][i].field_frame = 1;
      fs->bottom_field->mv_info[j][i].field_frame = 1;
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Allocate memory for buffering of reference picture reordering commands
 ************************************************************************
 */
void alloc_ref_pic_list_reordering_buffer(Slice *currSlice)
{  
  free_ref_pic_list_reordering_buffer(currSlice);

  if (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE)
  {
    int size = currSlice->num_ref_idx_active[LIST_0] + 1;
    if ((currSlice->modification_of_pic_nums_idc[LIST_0] = calloc(size, sizeof(int)))==NULL) 
      no_mem_exit("alloc_ref_pic_list_reordering_buffer: modification_of_pic_nums_idc_l0");
    if ((currSlice->abs_diff_pic_num_minus1[LIST_0] = calloc(size, sizeof(int)))==NULL) 
      no_mem_exit("alloc_ref_pic_list_reordering_buffer: abs_diff_pic_num_minus1_l0");
    if ((currSlice->long_term_pic_idx[LIST_0] = calloc(size, sizeof(int)))==NULL)
      no_mem_exit("alloc_ref_pic_list_reordering_buffer: long_term_pic_idx_l0");
#if (MVC_EXTENSION_ENABLE)
    if ((currSlice->abs_diff_view_idx_minus1[LIST_0] = calloc(size, sizeof(int)))==NULL) 
      no_mem_exit("alloc_ref_pic_list_reordering_buffer: abs_diff_view_idx_minus1_l0");
#endif
  }
  else
  {
    currSlice->modification_of_pic_nums_idc[LIST_0] = NULL;
    currSlice->abs_diff_pic_num_minus1[LIST_0] = NULL;
    currSlice->long_term_pic_idx[LIST_0] = NULL;
#if (MVC_EXTENSION_ENABLE)
    currSlice->abs_diff_view_idx_minus1[LIST_0] = NULL;
#endif
  }

  if (currSlice->slice_type == B_SLICE)
  {
    int size = currSlice->num_ref_idx_active[LIST_1] + 1;
    if ((currSlice->modification_of_pic_nums_idc[LIST_1] = calloc(size, sizeof(int)))==NULL) 
      no_mem_exit("alloc_ref_pic_list_reordering_buffer: modification_of_pic_nums_idc_l1");
    if ((currSlice->abs_diff_pic_num_minus1[LIST_1] = calloc(size, sizeof(int)))==NULL) 
      no_mem_exit("alloc_ref_pic_list_reordering_buffer: abs_diff_pic_num_minus1_l1");
    if ((currSlice->long_term_pic_idx[LIST_1] = calloc(size, sizeof(int)))==NULL) 
      no_mem_exit("alloc_ref_pic_list_reordering_buffer: long_term_pic_idx_l1");
#if (MVC_EXTENSION_ENABLE)
    if ((currSlice->abs_diff_view_idx_minus1[LIST_1] = calloc(size, sizeof(int)))==NULL) 
      no_mem_exit("alloc_ref_pic_list_reordering_buffer: abs_diff_view_idx_minus1_l1");
#endif
  }
  else
  {
    currSlice->modification_of_pic_nums_idc[LIST_1] = NULL;
    currSlice->abs_diff_pic_num_minus1[LIST_1] = NULL;
    currSlice->long_term_pic_idx[LIST_1] = NULL;
#if (MVC_EXTENSION_ENABLE)
    currSlice->abs_diff_view_idx_minus1[LIST_1] = NULL;
#endif
  }
}


/*!
 ************************************************************************
 * \brief
 *    Free memory for buffering of reference picture reordering commands
 ************************************************************************
 */
void free_ref_pic_list_reordering_buffer(Slice *currSlice)
{
  if (currSlice->modification_of_pic_nums_idc[LIST_0])
    free(currSlice->modification_of_pic_nums_idc[LIST_0]);
  if (currSlice->abs_diff_pic_num_minus1[LIST_0])
    free(currSlice->abs_diff_pic_num_minus1[LIST_0]);
  if (currSlice->long_term_pic_idx[LIST_0])
    free(currSlice->long_term_pic_idx[LIST_0]);

  currSlice->modification_of_pic_nums_idc[LIST_0] = NULL;
  currSlice->abs_diff_pic_num_minus1[LIST_0] = NULL;
  currSlice->long_term_pic_idx[LIST_0] = NULL;

  if (currSlice->modification_of_pic_nums_idc[LIST_1])
    free(currSlice->modification_of_pic_nums_idc[LIST_1]);
  if (currSlice->abs_diff_pic_num_minus1[LIST_1])
    free(currSlice->abs_diff_pic_num_minus1[LIST_1]);
  if (currSlice->long_term_pic_idx[LIST_1])
    free(currSlice->long_term_pic_idx[LIST_1]);

  currSlice->modification_of_pic_nums_idc[LIST_1] = NULL;
  currSlice->abs_diff_pic_num_minus1[LIST_1] = NULL;
  currSlice->long_term_pic_idx[LIST_1] = NULL;

#if (MVC_EXTENSION_ENABLE)
  if (currSlice->abs_diff_view_idx_minus1[LIST_0])
  {
    free(currSlice->abs_diff_view_idx_minus1[LIST_0]);
    currSlice->abs_diff_view_idx_minus1[LIST_0] = NULL;
  }
  if (currSlice->abs_diff_view_idx_minus1[LIST_1])
  {
    free(currSlice->abs_diff_view_idx_minus1[LIST_1]);
    currSlice->abs_diff_view_idx_minus1[LIST_1] = NULL;
  }
#endif
}

/*!
 ************************************************************************
 * \brief
 *      Tian Dong
 *          June 13, 2002, Modified on July 30, 2003
 *
 *      If a gap in frame_num is found, try to fill the gap
 * \param p_Vid
 *    VideoParameters structure
 * \param output
 *    FrameFormat for ouput
 *
 ************************************************************************
 */
void fill_frame_num_gap(VideoParameters *p_Vid, FrameFormat *output)
{
  int CurrFrameNum;
  int UnusedShortTermFrameNum;
  StorablePicture *picture = NULL;
  int nal_ref_idc_bak = p_Vid->nal_reference_idc;

//  printf("A gap in frame number is found, try to fill it.\n");
  
  
  p_Vid->nal_reference_idc = NALU_PRIORITY_LOW;

  UnusedShortTermFrameNum = (p_Vid->pre_frame_num + 1) % p_Vid->max_frame_num;
  CurrFrameNum = p_Vid->frame_num;

  while (CurrFrameNum != UnusedShortTermFrameNum)
  {
    picture = alloc_storable_picture (p_Vid, FRAME, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr);
    picture->coded_frame = 1;
    picture->pic_num = UnusedShortTermFrameNum;
    picture->non_existing = 1;
    picture->is_output = 1;

    p_Vid->adaptive_ref_pic_buffering_flag = 0;
    store_picture_in_dpb(p_Vid->p_Dpb_layer[0], picture, output);
    picture=NULL;
    UnusedShortTermFrameNum = (UnusedShortTermFrameNum + 1) % p_Vid->max_frame_num;
  }

  p_Vid->nal_reference_idc = nal_ref_idc_bak;
}

/*!
 ************************************************************************
 * \brief
 *    Compute co-located motion info
 *
 ************************************************************************
 */
void compute_colocated(Slice *currSlice, StorablePicture **listX[6])
{
  int i, j;

  VideoParameters *p_Vid = currSlice->p_Vid;

  if (currSlice->direct_spatial_mv_pred_flag == 0)
  {
    for (j = 0; j < 2 + (currSlice->mb_aff_frame_flag * 4); j += 2)
    {
      for (i = 0; i < currSlice->listXsize[j];i++)
      {
        int prescale, iTRb, iTRp;

        if (j==0)
        {
          iTRb = iClip3( -128, 127, p_Vid->enc_picture->poc - listX[LIST_0 + j][i]->poc );
        }
        else if (j == 2)
        {
          iTRb = iClip3( -128, 127, p_Vid->enc_picture->top_poc - listX[LIST_0 + j][i]->poc );
        }
        else
        {
          iTRb = iClip3( -128, 127, p_Vid->enc_picture->bottom_poc - listX[LIST_0 + j][i]->poc );
        }

        iTRp = iClip3( -128, 127,  listX[LIST_1 + j][0]->poc - listX[LIST_0 + j][i]->poc);

        if (iTRp!=0)
        {
          prescale = ( 16384 + iabs( iTRp / 2 ) ) / iTRp;
          currSlice->mvscale[j][i] = iClip3( -1024, 1023, ( iTRb * prescale + 32 ) >> 6 ) ;
        }
        else
        {
          currSlice->mvscale[j][i] = 9999;
        }
      }
    }
  }
}


static void process_picture_in_dpb_s(VideoParameters *p_Vid, StorablePicture *p_pic)
{
  //InputParameters *p_Inp = p_Vid->p_Inp;
 
  ImageData *p_img_out = &p_Vid->tempData3;
  ImageData img_in;
  imgpel***  d_img;
  int i;

  img_in.frm_stride[0] = p_pic->size_x_padded;
  img_in.frm_stride[1] = img_in.frm_stride[2] = p_pic->size_x_cr + (p_pic->pad_size_uv_x << 1);

  if (p_pic->structure == FRAME)
  {
    img_in.frm_data[0] = p_pic->imgY;
    img_in.frm_data[1] = p_pic->imgUV[0];
    img_in.frm_data[2] = p_pic->imgUV[1];
    img_in.top_stride[0] = img_in.bot_stride[0] = img_in.frm_stride[0] << 1;
    img_in.top_stride[1] = img_in.bot_stride[1] = img_in.top_stride[2] = img_in.bot_stride[2] = img_in.frm_stride[1] << 1;
    d_img = p_img_out->frm_data;
  }
  else //If reference picture is a field, then frm_data will actually contain field data and therefore top/bottom stride is set accordingly.
  {
    img_in.top_stride[0] = img_in.bot_stride[0] = img_in.frm_stride[0];
    img_in.top_stride[1] = img_in.bot_stride[1] = img_in.top_stride[2] = img_in.bot_stride[2] =img_in.frm_stride[1];
    if (p_pic->structure == TOP_FIELD)
    {
      img_in.top_data[0] = p_pic->imgY;
      img_in.top_data[1] = p_pic->imgUV[0];
      img_in.top_data[2] = p_pic->imgUV[1];
      d_img = p_img_out->top_data;
    }
    else
    {
      img_in.bot_data[0] = p_pic->imgY;
      img_in.bot_data[1] = p_pic->imgUV[0];
      img_in.bot_data[2] = p_pic->imgUV[1];
      d_img = p_img_out->bot_data;
    }
  }

  for(i=0; i<p_pic->size_y; i++)
    memcpy(d_img[0][i], p_pic->imgY[i], p_pic->size_x*sizeof(imgpel));
  if (p_Vid->yuv_format != YUV400)
  {
    for(i=0; i<p_pic->size_y_cr; i++)
      memcpy(d_img[1][i], p_pic->imgUV[0][i], p_pic->size_x_cr * sizeof(imgpel));
    for(i=0; i<p_pic->size_y_cr; i++)
      memcpy(d_img[2][i], p_pic->imgUV[1][i], p_pic->size_x_cr * sizeof(imgpel));
  }
}

/*!
 ************************************************************************
 * \brief
 *    Copy a stored picture
 ************************************************************************
 */
static void copy_storable_picture(VideoParameters *p_Vid, StorablePicture *s, StorablePicture *d)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int   nplane;
  int   i, j;
  int   size_x = s->size_x;
  int   size_y = s->size_y;
  int   size_x_cr = s->size_x_cr;
  int   size_y_cr = s->size_y_cr;
  imgpel***  s_img;
  extern void check_value(imgpel **pBufIn, int width, int height, int max);

  // store BL reconstruction
  if (s->structure == FRAME)
  {
    s_img = p_Vid->tempData3.frm_data;
  }
  else if (s->structure == TOP_FIELD)
  {
    s_img = p_Vid->tempData3.top_data;
  }
  else
  {
    s_img = p_Vid->tempData3.bot_data;
  }

  for(i=0; i<size_y; i++)
    memcpy((void *)d->imgY[i], (void *)s_img[0][i], size_x * sizeof (imgpel));

  d->p_img[0] = d->imgY;
  d->p_curr_img = d->p_img[0];

  if (p_Vid->yuv_format != YUV400)
  {
    // store BL reconstruction
    for(i=0; i<size_y_cr; i++)
      memcpy((void *)d->imgUV[0][i], (void *)s_img[1][i], size_x_cr * sizeof (imgpel));
    for(i=0; i<size_y_cr; i++)
      memcpy((void *)d->imgUV[1][i], (void *)s_img[2][i], size_x_cr * sizeof (imgpel));

    d->p_img[1] = d->imgUV[0];
    d->p_img[2] = d->imgUV[1];
  }

  d->p_curr_img_sub = d->p_img_sub[0];
  for (j = 0; j < (size_y >> BLOCK_SHIFT); j++)
  {
    for (i = 0; i < (size_x >> BLOCK_SHIFT); i++)
    {
      d->mv_info[j][i].ref_idx[LIST_0] = -1;
      d->mv_info[j][i].ref_idx[LIST_1] = -1;
      d->mv_info[j][i].ref_pic[LIST_0] = NULL;
      d->mv_info[j][i].ref_pic[LIST_1] = NULL;
    }
  }

  if( (p_Inp->separate_colour_plane_flag != 0) )
  {
    for( nplane=0; nplane<MAX_PLANE; nplane++ )
    {
      for (j = 0; j < (size_y >> BLOCK_SHIFT); j++)
      {
        for (i = 0; i < (size_x >> BLOCK_SHIFT); i++)
        {
          d->JVmv_info[nplane][j][i].ref_idx[LIST_0] = -1;
          d->JVmv_info[nplane][j][i].ref_idx[LIST_1] = -1;
        }
      }
    }
  }

  d->pic_num = s->pic_num;
  d->frame_num = s->frame_num;
  d->long_term_frame_idx = s->long_term_frame_idx;
  d->long_term_pic_num = s->long_term_pic_num;
  d->used_for_reference = s->used_for_reference;
  d->is_long_term = s->is_long_term;
  d->non_existing = s->non_existing;
  d->is_output = 1;
  d->structure = s->structure;
  
  // these are set in insert_picture_in_dpb
  d->top_field    = NULL;
  d->bottom_field = NULL;
  d->frame        = NULL;

  d->coded_frame = s->coded_frame;
  d->mb_aff_frame_flag = s->mb_aff_frame_flag;

  memcpy( (void *)&d->stats, (void *)&s->stats, sizeof( StatParameters ) );
  d->stats.NumberBFrames = s->stats.NumberBFrames;
}

/*!
 ************************************************************************
 * \brief
 *    Clone an encoded frame picture structure
 ************************************************************************
 */
static StorablePicture * clone_storable_picture( VideoParameters *p_Vid, StorablePicture *p_pic )
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  StorablePicture *p_stored_pic;
  int nal_reference_idc= p_Vid->nal_reference_idc;
  p_Vid->nal_reference_idc = NALU_PRIORITY_HIGH; //NALU_PRIORITY_DISPOSABLE;
  p_stored_pic = alloc_storable_picture (p_Vid, (PictureStructure) p_Vid->structure, p_pic->size_x, p_pic->size_y, p_pic->size_x_cr, p_pic->size_y_cr);
  p_Vid->nal_reference_idc = nal_reference_idc;
  p_stored_pic->poc         = p_Vid->framepoc;
  p_stored_pic->top_poc     = p_Vid->toppoc;
  p_stored_pic->bottom_poc  = p_Vid->bottompoc;
  p_stored_pic->frame_poc   = p_Vid->framepoc;
  p_stored_pic->pic_num     = p_pic->pic_num;
  p_stored_pic->frame_num   = p_pic->frame_num;
  p_stored_pic->coded_frame = 1;
  p_stored_pic->mb_aff_frame_flag = p_Vid->mb_aff_frame_flag = (Boolean) (p_Inp->MbInterlace != FRAME_CODING);

  copy_params(p_Vid, p_stored_pic, p_Vid->active_sps);


  copy_storable_picture(p_Vid, p_pic, p_stored_pic);
#if (MVC_EXTENSION_ENABLE)
  // MVC-related parameters
  p_stored_pic->inter_view_flag[0] = p_pic->inter_view_flag[0];
  p_stored_pic->inter_view_flag[1] = p_pic->inter_view_flag[1];
  p_stored_pic->anchor_pic_flag[0] = p_stored_pic->anchor_pic_flag[1] = 0;
  p_stored_pic->view_id = 0;
  //p_stored_pic->proc_flag = 1;
#endif
  return p_stored_pic;
}

/*!
 ************************************************************************
 * \brief
 *    Store a processed picture in DPB.
 * \param p_Vid
 *    VideoParameters
 * \param p
 *    Picture to be stored
 * \param output
 *    FrameFormat for output
 ************************************************************************
 */
#if (MVC_EXTENSION_ENABLE)
void store_proc_picture_in_dpb(DecodedPictureBuffer *p_Dpb, StorablePicture* p, FrameFormat *output)
{
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  FrameStore *fs = p_Dpb->fs_ilref[0];
  if(p_Dpb->used_size_il>0 && fs->is_used==3)
  {
    //checking;
#ifdef _DEBUG
    if(p->structure==FRAME)
      assert(fs->frame->frame_poc != p->poc);
    else if(p->structure==TOP_FIELD)
      assert(fs->top_field->top_poc != p->poc);
    else if(p->structure==BOTTOM_FIELD)
      assert(fs->bottom_field->bottom_poc != p->poc);
#endif
    if(fs->frame)
    {
      free_storable_picture(p_Vid, fs->frame);
      fs->frame = NULL;
    }
    if(fs->top_field)
    {
      free_storable_picture(p_Vid, fs->top_field);
      fs->top_field = NULL;
    }
    if(fs->bottom_field)
    {
      free_storable_picture(p_Vid, fs->bottom_field);
      fs->bottom_field = NULL;
    }
    fs->is_used = 0;
    fs->is_reference = 0;
    fs->is_inter_layer = TRUE;
    p_Dpb->used_size_il--;   
  }
#ifdef _DEBUG  
  if(fs->is_used>0)
  {
    //checking;
    if(p->structure==FRAME)
      assert(fs->frame == NULL);
    else if(p->structure==TOP_FIELD)
      assert(fs->top_field == NULL);
    else if(p->structure==BOTTOM_FIELD)
      assert(fs->bottom_field == NULL);
  }
#endif

  insert_picture_in_dpb(p_Vid, fs, p);
  if((p->structure==FRAME && fs->is_used == 3) || (p->structure!=FRAME && fs->is_used && fs->is_used <3))
    p_Dpb->used_size_il++;  
}
#endif


