/*!
 ***************************************************************************
 * \file
 *    pred_struct.c
 *
 * \author
 *    Athanasios Leontaris           <aleon@dolby.com>   
 *
 * \date
 *    June 8, 2009
 *
 * \brief
 *    Source file for prediction structure function
 **************************************************************************
 */

#include "pred_struct.h"
#include "explicit_seq.h"

#define DEBUG_PRED_STRUCT 0

static inline void pop_pred_struct_frm( PredStructFrm *p_dst, int slice_type, int nal_ref_idc, int disp_offset, int layer, int slice_qp_off,
                                       int random_access, int temporal_layer );
static void init_gop_struct( InputParameters *p_Inp, SeqStructure *p_seq_struct, int is_idr, int *memory_size );
static void free_gop_struct( SeqStructure *p_seq_struct );
static void init_pred_struct( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, int *memory_size );
static void free_pred_struct( SeqStructure *p_seq_struct );
static PicStructure * alloc_pic_struct( int num_slices, int *memory_size );
static void free_pic_struct( PicStructure *p_pic );
static FrameUnitStruct * alloc_frame_buffer( InputParameters *p_Inp, int num_frames, int num_slices, int *memory_size );
static void free_frame_buffer( FrameUnitStruct *p_frm_struct, int num_frames );

static int rand_acc_test_range( int curr_frame, int offset, int idr_period );
static inline int check_power_of_two( int number );
static int  establish_random_access( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames, int sim );
static int  establish_intra( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames, int sim );
static int  establish_sp( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames, int sim );
static int  get_fixed_frame( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames );
static int  get_prd_index( InputParameters *p_Inp, SeqStructure *p_seq_struct, int num_frames );
static int  get_idr_index( InputParameters *p_Inp, SeqStructure *p_seq_struct, int num_frames );
static int  get_intra_index( InputParameters *p_Inp, SeqStructure *p_seq_struct, int num_frames );
static void populate_frame( InputParameters *p_Inp, SeqStructure *p_seq_struct, FrameUnitStruct *p_frm_struct, PredStructAtom *p_cur_prd, 
                        int curr_frame, int pred_frame, int idx, int num_slices, int no_rnd_acc );
static void update_frame_indices( SeqStructure *p_seq_struct, FrameUnitStruct *p_frm_struct, int curr_frame, int pred_frame );
static int  populate_reg_pred_struct_atom( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, 
                                          int curr_frame, int proc_frames, int *terminate_pop );
static int  populate_rnd_acc_pred_struct_atom( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct,
                                              int curr_frame, int proc_frames, int *terminate_pop, int is_idr, int no_rnd_acc );
#if (MVC_EXTENSION_ENABLE)
static void copy_frame_mvc( InputParameters *p_Inp, FrameUnitStruct *p_src, FrameUnitStruct *p_dst, int num_slices, int view_id );
#endif

/*!
 ***********************************************************************
 * \brief
 *    Derivation of Picture Order Count Type 0
 * \param p_Vid
 *    pointer to the VideoParameters structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_frm_struct
 *    pointer to the cyclical frame unit buffer
 ***********************************************************************
 */

void get_poc_type_zero( VideoParameters *p_Vid, InputParameters *p_Inp, FrameUnitStruct *p_frm_struct )
{
  FrameUnitStruct *p_cur = p_frm_struct; // + (curr_frm_idx % p_Vid->frm_struct_buffer);
  int last_disp_order;

  if ( p_Vid->last_idr_code_order > p_Vid->last_mmco_5_code_order )
  {
    last_disp_order = p_Vid->last_idr_disp_order;
  }
  else
  {
    last_disp_order = p_Vid->last_mmco_5_disp_order;
  }

  if ( p_cur->idr_flag )
  {
    p_Vid->toppoc = 0;
  }
  else
  {
    p_Vid->toppoc = (p_cur->frame_no - last_disp_order) << 1;
  }
  // frame_skip consideration
  p_Vid->toppoc = (1 + p_Inp->frame_skip) * p_Vid->toppoc;

  if ( p_Inp->PicInterlace == FRAME_CODING && p_Inp->MbInterlace == FRAME_CODING )
  {
    p_Vid->bottompoc = p_Vid->toppoc; // progressive
  }
  else
  {
    p_Vid->bottompoc = p_Vid->toppoc + 1; // hard coded
  }

  p_Vid->framepoc = imin( p_Vid->toppoc, p_Vid->bottompoc );
}

/*!
 ***********************************************************************
 * \brief
 *    Derivation of Picture Order Count Type 1
 * \param p_Vid
 *    pointer to the VideoParameters structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_frm_struct
 *    pointer to the cyclical frame unit buffer
 ***********************************************************************
 */

void get_poc_type_one( VideoParameters *p_Vid, InputParameters *p_Inp, FrameUnitStruct *p_frm_struct )
{
  FrameUnitStruct *p_cur = p_frm_struct; // + curr_frm_idx;
  int i;
  int AbsFrameNum;
  int ExpectedDeltaPerPicOrderCntCycle;
  int PicOrderCntCycleCnt;
  int FrameNumInPicOrderCntCycle;
  int ExpectedPicOrderCnt;
  CodingParameters *p_EncodePar = p_Vid->p_EncodePar[p_Vid->dpb_layer_id];

  // we need to get type 0 values first and then estimate delta_pic_order_cnt with respect to those
  get_poc_type_zero( p_Vid, p_Inp, p_frm_struct );

  // next we use the decoder POC Type 1 code to determine the delta_pic_order_cnt

  // derive p_Vid->FrameNumOffset
  if ( p_cur->idr_flag )
  {
    p_Vid->FrameNumOffset = 0;
    p_Vid->delta_pic_order_cnt[0] = 0;
  }
  else
  {
    if ( p_Vid->last_has_mmco_5 )
    {
      p_EncodePar->prevFrameNumOffset = 0;
      p_EncodePar->prevFrameNum = 0;
    }
    if ( p_Vid->frame_num < p_EncodePar->prevFrameNum )
    {
      p_Vid->FrameNumOffset = p_EncodePar->prevFrameNumOffset + p_Vid->max_frame_num;
    }
    else
    {
      p_Vid->FrameNumOffset = p_EncodePar->prevFrameNumOffset;
    }
  }

  // derive AbsFrameNum
  if ( p_Vid->num_ref_frames_in_pic_order_cnt_cycle )
  {
    AbsFrameNum = p_Vid->FrameNumOffset + p_Vid->frame_num;
  }
  else
  {
    AbsFrameNum=0;
  }
  if ( !(p_cur->nal_ref_idc) && AbsFrameNum > 0 )
  {
    AbsFrameNum--;
  }

  // derive ExpectedPicOrderCnt
  ExpectedDeltaPerPicOrderCntCycle=0;
  if ( p_Vid->num_ref_frames_in_pic_order_cnt_cycle )
  {
    for( i = 0; i < (int) p_Vid->num_ref_frames_in_pic_order_cnt_cycle; i++ )
    {
      ExpectedDeltaPerPicOrderCntCycle += p_Vid->offset_for_ref_frame[i];
    }
  }

  if ( AbsFrameNum )
  {
    PicOrderCntCycleCnt = (AbsFrameNum - 1) / p_Vid->num_ref_frames_in_pic_order_cnt_cycle;
    FrameNumInPicOrderCntCycle = (AbsFrameNum - 1) % p_Vid->num_ref_frames_in_pic_order_cnt_cycle;

    ExpectedPicOrderCnt = PicOrderCntCycleCnt * ExpectedDeltaPerPicOrderCntCycle;
    for ( i = 0; i <= (int)FrameNumInPicOrderCntCycle; i++ )
    {
      ExpectedPicOrderCnt += p_Vid->offset_for_ref_frame[i];
    }
  }
  else
  {
    ExpectedPicOrderCnt = 0;
  }

  if ( !(p_cur->nal_ref_idc) )
  {
    ExpectedPicOrderCnt += p_Vid->offset_for_non_ref_pic;
  }

  p_Vid->delta_pic_order_cnt[0] = p_Vid->toppoc - ExpectedPicOrderCnt;
  p_Vid->delta_pic_order_cnt[1] = 0;

  // update parameters
  p_Vid->prevFrameNum = p_EncodePar->prevFrameNum = p_Vid->frame_num;
  p_Vid->prevFrameNumOffset = p_EncodePar->prevFrameNumOffset = p_Vid->FrameNumOffset;
}

/*!
 ***********************************************************************
 * \brief
 *    Initializes POC type 0 and 1: POC type 1 simulates the decoder's behavior and sets values \n
 *    given the POCs calculated for POC type 0; needs to be tested
 * \param p_Vid
 *    pointer to the VideoParameters structure
 ***********************************************************************
 */

void init_poc(VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  // general parameters
  p_Vid->pic_order_cnt_type = p_Inp->pic_order_cnt_type;
  if ((p_Inp->PicInterlace == FRAME_CODING) && (p_Inp->MbInterlace == FRAME_CODING))
  {
    p_Vid->bottom_field_pic_order_in_frame_present_flag = FALSE;
  }
  else
  {
    p_Vid->bottom_field_pic_order_in_frame_present_flag = TRUE;
  }

  // POC type 0
  if ((p_Inp->PicInterlace==FRAME_CODING) && (p_Inp->MbInterlace==FRAME_CODING))
  {
    p_Vid->delta_pic_order_cnt_bottom = 0;
  }
  else
  {
    p_Vid->delta_pic_order_cnt_bottom = 1;
  }

  // POC type 1 (parameters need to be *optimized* for the prediction structure)
  // should be good enough for now
  p_Vid->delta_pic_order_always_zero_flag = FALSE;
  p_Vid->num_ref_frames_in_pic_order_cnt_cycle = 1;

  // these initialization have to depend more on the structure
  if (p_Inp->BRefPictures == 1)
  {
    p_Vid->offset_for_non_ref_pic  = 0;
    p_Vid->offset_for_ref_frame[0] = 2;
  }
  else
  {
    p_Vid->offset_for_non_ref_pic  = -2*(p_Inp->NumberBFrames);
    p_Vid->offset_for_ref_frame[0] =  2*(p_Inp->NumberBFrames + 1);
  }

  if ((p_Inp->PicInterlace==FRAME_CODING) && (p_Inp->MbInterlace==FRAME_CODING))
  {
    p_Vid->offset_for_top_to_bottom_field = 0;
  }
  else
  {
    p_Vid->offset_for_top_to_bottom_field = 1;
  }  
}

/*!
 ***********************************************************************
 * \brief
 *    Allocate picture structure
 * \param num_slices
 *    max number of slices used to code the current picture
 * \param memory_size
 *    pointer to the memory_size variable to count the number of bytes allocated
 * \return
 *    pointer to the allocated PicStructure
 ***********************************************************************
 */

static PicStructure * alloc_pic_struct( int num_slices, int *memory_size )
{
  PicStructure *p_pic = (PicStructure *)malloc( sizeof( PicStructure ) );
  if ( p_pic == NULL )
  {
    fprintf(stderr, "\nNot enough memory or error during memory allocation: alloc_pic_struct()");
    exit(1);
  }
  *memory_size += sizeof( PicStructure );

  p_pic->num_slices = num_slices; // this could also be set on a frame basis
  p_pic->p_Slice = (SliceStructure *)malloc( num_slices * sizeof( SliceStructure ) );
  if ( p_pic->p_Slice == NULL )
  {
    fprintf(stderr, "\nNot enough memory or error during memory allocation: alloc_pic_struct()");
    exit(1);
  }
  *memory_size += (num_slices * sizeof( SliceStructure ));

  return p_pic;
}

/*!
 ***********************************************************************
 * \brief
 *    Free picture structure
 * \param p_pic
 *    pointer to the picture structure that we wish to deallocate
 ***********************************************************************
 */

static void free_pic_struct( PicStructure *p_pic )
{
  if ( p_pic != NULL )
  {
    if ( p_pic->p_Slice != NULL )
    {
      free( p_pic->p_Slice );
      p_pic->p_Slice = NULL;
    }
    free( p_pic );
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Allocate frame buffer
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param num_frames
 *    number of frames/length of the cyclical frame buffer
 * \param num_slices
 *    max number of slices used to code each frame unit in the cyclical frame buffer
 * \param memory_size
 *    pointer to the memory_size variable to count the number of bytes allocated
 * \return
 *    pointer to the allocated FrameUnitStruct
 ***********************************************************************
 */

static FrameUnitStruct * alloc_frame_buffer( InputParameters *p_Inp, int num_frames, int num_slices, int *memory_size )
{
  int idx;

  // FrameUnitStruct
  FrameUnitStruct *p_frm_struct = (FrameUnitStruct *)malloc( num_frames * sizeof( FrameUnitStruct ) );
  FrameUnitStruct *p_frm_tmp;

  if ( p_frm_struct == NULL )
  {
    fprintf(stderr, "\nNot enough memory or error during memory allocation: alloc_frame_buffer()");
    exit(1);
  }
  *memory_size += num_frames * sizeof( FrameUnitStruct );

  for ( idx = 0; idx < num_frames; idx++ )
  {
    p_frm_tmp = p_frm_struct + idx;

    switch( p_Inp->PicInterlace )
    {
    default:
    case FRAME_CODING:
      p_frm_tmp->field_pic_flag = 0;
      p_frm_tmp->p_top_fld_pic = NULL;
      p_frm_tmp->p_bot_fld_pic = NULL;
      p_frm_tmp->p_frame_pic = alloc_pic_struct( num_slices, memory_size );
      break;
    case FIELD_CODING:
      p_frm_tmp->field_pic_flag = 1;
      p_frm_tmp->p_top_fld_pic = alloc_pic_struct( num_slices, memory_size );
      p_frm_tmp->p_bot_fld_pic = alloc_pic_struct( num_slices, memory_size );
      p_frm_tmp->p_frame_pic = NULL;
      break;
    case ADAPTIVE_CODING:
      p_frm_tmp->p_top_fld_pic = alloc_pic_struct( num_slices, memory_size );
      p_frm_tmp->p_bot_fld_pic = alloc_pic_struct( num_slices, memory_size );
      p_frm_tmp->p_frame_pic   = alloc_pic_struct( num_slices, memory_size );
      break;
    }
  }

  return p_frm_struct;
}

/*!
 ***********************************************************************
 * \brief
 *    Free frame buffer
 * \param p_frm_struct
 *    pointer to the cyclical frame buffer
 * \param num_frames
 *    number of frames in the cyclical frame buffer
 ***********************************************************************
 */

static void free_frame_buffer( FrameUnitStruct *p_frm_struct, int num_frames )
{
  int idx;
  FrameUnitStruct *p_frm_tmp;

  if ( p_frm_struct != NULL )
  {
    for ( idx = 0; idx < num_frames; idx++ )
    {
      p_frm_tmp = p_frm_struct + idx;

      free_pic_struct( p_frm_tmp->p_top_fld_pic );
      p_frm_tmp->p_top_fld_pic = NULL;

      free_pic_struct( p_frm_tmp->p_bot_fld_pic );
      p_frm_tmp->p_bot_fld_pic = NULL;

      free_pic_struct( p_frm_tmp->p_frame_pic );
      p_frm_tmp->p_frame_pic = NULL;
    }

    free( p_frm_struct );
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Initialize sequence structure
 * \param p_Vid
 *    pointer to the VideoParameters structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param memory_size
 *    pointer to the memory_size variable that stores the number of bytes allocated so far
 * \return
 *    pointer to SeqStructure
 ***********************************************************************
 */

SeqStructure * init_seq_structure( VideoParameters *p_Vid, InputParameters *p_Inp, int *memory_size )
{
#if (MVC_EXTENSION_ENABLE)
  int start, end;
#endif
  SeqStructure *p_seq_struct = (SeqStructure *)malloc( sizeof( SeqStructure ) );
  int frames_to_pop;

  if ( p_seq_struct == NULL )
  {
    fprintf(stderr, "\nNot enough memory or error during memory allocation: init_seq_structure()");
    exit(1);
  }

  // set length of frame buffer
  p_Inp->FrmStructBufferLength = imax( p_Inp->FrmStructBufferLength, imax( p_Inp->idr_period, p_Inp->intra_period ) + p_Inp->NumberBFrames + p_Inp->intra_delay + 2 );
  p_Vid->frm_struct_buffer = p_Inp->FrmStructBufferLength;
  p_Vid->frm_struct_buffer = imin( p_Vid->frm_struct_buffer, p_Inp->no_frames );
  p_Inp->FrmStructBufferLength = imin( p_Inp->FrmStructBufferLength, p_Inp->no_frames ); 

  p_Inp->FrmStructBufferLength = p_Inp->no_frames;
  p_Vid->frm_struct_buffer = p_Inp->no_frames;

  *memory_size += sizeof( SeqStructure );

  p_seq_struct->max_num_slices           = (p_Inp->slice_mode == 1) ? imax(1, p_Vid->FrameSizeInMbs / p_Inp->slice_argument) : 1;
  p_seq_struct->num_frames               = p_Vid->frm_struct_buffer;
  p_seq_struct->p_frm                    = alloc_frame_buffer( p_Inp, p_seq_struct->num_frames, p_seq_struct->max_num_slices, memory_size );
  p_seq_struct->last_random_access_frame = 0;
  p_seq_struct->last_idr_frame           = 0;
  p_seq_struct->last_intra_frame         = 0;
  p_seq_struct->last_mmco_5_frame        = -1;
  p_seq_struct->curr_num_to_populate     = p_Vid->frm_struct_buffer;
  p_seq_struct->pop_start_frame          = 0;
  p_seq_struct->last_rand_access_disp    = 0;
  p_seq_struct->last_idr_disp            = 0;
  p_seq_struct->last_intra_disp          = 0;
  p_seq_struct->last_bl_frm_disposable   = 0;
  p_seq_struct->last_sp_frame            = 0;
  p_seq_struct->last_sp_disp             = 0;
  p_seq_struct->pop_flag                 = 0;

#if (MVC_EXTENSION_ENABLE)
  p_seq_struct->num_frames_mvc           = p_seq_struct->num_frames * p_Inp->num_of_views; // two views hence twice the buffer size
  p_seq_struct->p_frm_mvc                = alloc_frame_buffer( p_Inp, p_seq_struct->num_frames_mvc, p_seq_struct->max_num_slices, memory_size );
#endif

  // initialize prediction structures
  init_pred_struct( p_Vid, p_Inp, p_seq_struct, memory_size );
  init_gop_struct ( p_Inp, p_seq_struct, 1, memory_size ); // IDR GOPs
  init_gop_struct ( p_Inp, p_seq_struct, 0, memory_size ); // Intra GOPs

  frames_to_pop = p_Vid->frm_struct_buffer;

  // populate frames
#if (MVC_EXTENSION_ENABLE)
  start = p_seq_struct->pop_start_frame;
  populate_frm_struct( p_Vid, p_Inp, p_seq_struct, frames_to_pop, p_Inp->no_frames );

  if ( p_Inp->num_of_views > 1 )
  {
    assert( p_Inp->num_of_views < 4 );
    end = p_seq_struct->pop_start_frame;
    populate_frm_struct_mvc( p_Vid, p_Inp, p_seq_struct, start, end );
  }
#else
  populate_frm_struct( p_Vid, p_Inp, p_seq_struct, frames_to_pop, p_Inp->no_frames );
#endif

  return p_seq_struct;
}

/*!
 ***********************************************************************
 * \brief
 *    Free sequence structure
 * \param p_seq_struct
 *    pointer to the sequence structure we wish to deallocate
 ***********************************************************************
 */

void free_seq_structure( SeqStructure *p_seq_struct )
{
  free_frame_buffer( p_seq_struct->p_frm, p_seq_struct->num_frames );
  p_seq_struct->p_frm = NULL;
#if (MVC_EXTENSION_ENABLE)
  free_frame_buffer( p_seq_struct->p_frm_mvc, p_seq_struct->num_frames_mvc );
  p_seq_struct->p_frm_mvc = NULL;
#endif
  free_gop_struct( p_seq_struct );
  p_seq_struct->p_gop = NULL;
  free_pred_struct( p_seq_struct );
  p_seq_struct->p_prd = NULL;

  if ( p_seq_struct != NULL )
  {
    free( p_seq_struct );
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Establish whether the frame with coding order "curr_frame" will be coded as an IDR picture
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param curr_frame
 *    coding order of the current frame
 * \param avail_frames
 *    frames available for population
 * \param sim
 *    simulate the insertion: this disables checks about available frames ;)
 * \return
 *    returns 1 when current frame will be a random access frame \n
 *    if PreferDispOrder == 1 then it returns 1 + disp_offset (for the random access frame)
 ***********************************************************************
 */

static int establish_random_access( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames, int sim )
{
  int delta;
  int is_random_access = 0;

  if ( !curr_frame ) // first frame in coding order
  {
    if ( !(p_Inp->PreferDispOrder) ) // coding order
    {
      is_random_access = 1;
    }
    else
    {
      int idx;
      PredStructAtom *p_cur_gop;

      // test each random access prediction structure, starting from the longest one
      // if random access is established then break from this loop and prepare output parameter
      for ( idx = (p_seq_struct->num_gops - 1); idx >= 0; idx-- )
      {
        p_cur_gop = p_seq_struct->p_gop + idx;
        // check if length of structure overflows the available frame number
        if ( sim )
        {
          if ( (curr_frame + p_cur_gop->length) <= p_Inp->no_frames )
          {
            is_random_access = 1 + idx;
            break;
          }
        }
        else
        {
          if ( p_cur_gop->length <= avail_frames )
          {
            is_random_access = 1 + idx;
            break;
          }
        }
      }
    }
  }
  else if ( p_Inp->idr_period )
  {
    if ( !(p_Inp->PreferDispOrder) ) // coding order
    {
      switch ( p_Inp->adaptive_idr_period )
      {
      default:
      case 0:
        if ( curr_frame % p_Inp->idr_period == 0 )
        {
          is_random_access = 1;
        }
        break;
      case 1:
        delta = curr_frame - p_seq_struct->last_random_access_frame;
        if ( delta >= p_Inp->idr_period && delta >= p_Inp->MinIDRDistance )
        {
          is_random_access = 1;
        }
        break;
      case 2:
        delta = curr_frame - imax( p_seq_struct->last_random_access_frame, p_seq_struct->last_intra_frame);
        if ( delta >= p_Inp->idr_period && delta >= p_Inp->MinIDRDistance )
        {
          is_random_access = 1;
        }
        break;
      }
    }
    else // display order (insertion)
    {
      int idx;
      int offset;
      PredStructAtom *p_cur_gop;

      // test each random access prediction structure, starting from the longest one
      // if random access is established then break from this loop and prepare output parameter
      for ( idx = (p_seq_struct->num_gops - 1); idx >= 0; idx-- )
      {
        p_cur_gop = p_seq_struct->p_gop + idx;
        // check if length of structure overflows the available frame number
        if ( sim )
        {
          if ( (curr_frame + p_cur_gop->length) > p_Inp->no_frames )
          {
            continue;
          }
        }
        else
        {
          if ( p_cur_gop->length > avail_frames )
          {
            continue;
          }
        }
        // recall that frame "0" is the random access frame in those prediction structures
        offset = p_cur_gop->p_frm[0].disp_offset;

        // now test if random access conditions are satisfied
        switch ( p_Inp->adaptive_idr_period )
        {
        default:
        case 0:
          if ( !curr_frame || (curr_frame + offset - p_Inp->intra_delay) % p_Inp->idr_period == 0 )
          {
            is_random_access = 1 + idx;
          }
          break;
        case 1:
          delta = (curr_frame + offset) - p_seq_struct->last_rand_access_disp;
          if ( !curr_frame || (delta >= p_Inp->idr_period && delta >= p_Inp->MinIDRDistance) )
          {
            is_random_access = 1 + idx;
          }
          break;
        case 2:
          delta = (curr_frame + offset) - imax( p_seq_struct->last_rand_access_disp, p_seq_struct->last_intra_disp);
          if ( !curr_frame || (delta >= p_Inp->idr_period && delta >= p_Inp->MinIDRDistance) )
          {
            is_random_access = 1 + idx;
          }
          break;
        }
        if ( is_random_access )
        {
          break;
        }
      }
    }
  }
  // additional case can be added to insert IDR based on *pre-analysis* and other "pre-scient" tools

  return is_random_access;
}

/*!
 ***********************************************************************
 * \brief
 *    Test if IDR is placed between curr_frame and curr_frame + offset inclusive
 ***********************************************************************
 */

static int rand_acc_test_range( int curr_frame, int offset, int idr_period )
{
  int idx;
  int no_rnd_acc = 1;

  if ( idr_period > 0 )
  {
    for ( idx = curr_frame; idx <= (curr_frame + offset); idx++ )
    {
      if ( idx % idr_period == 0 )
      {
        no_rnd_acc = 0;
        break;
      }
    }
  }

  return no_rnd_acc;
}

/*!
 ***********************************************************************
 * \brief
 *    Establish whether the frame with coding order "curr_frame" will be coded as an INTRA picture
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param curr_frame
 *    coding order of the current frame
 * \param avail_frames
 *    frames available for population
 * \return
 *    returns 1 when current frame will be an intra frame \n
 *    if PreferDispOrder == 1 then it returns 1 + disp_offset (for the intra frame)
 ***********************************************************************
 */

static int establish_intra( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames, int sim )
{
  int is_intra = 0;

  if ( !curr_frame ) // first frame in coding order
  {
    is_intra = 1;
  }
  else if ( p_Inp->intra_period )
  {
    if ( !(p_Inp->PreferDispOrder) ) // coding order
    {
      switch ( p_Inp->adaptive_intra_period )
      {
      default:
      case 0:
        if ( curr_frame % p_Inp->intra_period == 0 )
        {
          is_intra = 1;
        }
        break;
      case 1:
        if ( (curr_frame - imax( p_seq_struct->last_idr_frame, p_seq_struct->last_intra_frame)) % p_Inp->intra_period == 0 )
        {
          is_intra = 1;
        }
        break;
      }
    }
    else // display order (insertion)
    {
      int idx;
      int offset;
      PredStructAtom *p_cur_gop;

      // test each random access prediction structure, staring from the longest one
      // if random access is established then break from this loop and prepare output parameter
      for ( idx = (p_seq_struct->num_intra_gops - 1); idx >= 0; idx-- )
      {
        p_cur_gop = p_seq_struct->p_intra_gop + idx;
        // check if length of structure overflows the available frame number
        if ( sim )
        {
          if ( (curr_frame + p_cur_gop->length) > p_Inp->no_frames )
          {
            continue;
          }
        }
        else
        {
          if ( p_cur_gop->length > avail_frames )
          {
            continue;
          }
        }
        // recall that frame "0" is the random access frame in those prediction structures
        offset = p_cur_gop->p_frm[0].disp_offset;

        // now test if random access conditions are satisfied
        switch ( p_Inp->adaptive_intra_period )
        {
        default:
        case 0:
          // intra_period operates as if EnableIDRGOP=0 which means that the criteria for its insertion, when inserting based on display order
          // may be satisfied before (in terms of coding order) an IDR_period with EnableIDRGOP=1 (for 0 there is no issue)
          // this code ensures that IDRPeriod always takes precedence over IntraPeriod
          if ( (curr_frame + offset) % p_Inp->intra_period == 0 && rand_acc_test_range( curr_frame, offset, p_Inp->idr_period ) )
          {
            is_intra = 1 + idx;
          }
          break;
        case 2:
          // see comment above
          if ( ((curr_frame + offset) - imax( p_seq_struct->last_idr_disp, p_seq_struct->last_intra_disp)) % p_Inp->intra_period == 0
            && rand_acc_test_range( curr_frame - imax( p_seq_struct->last_idr_disp, p_seq_struct->last_intra_disp), offset, p_Inp->idr_period ) )
          {
            is_intra = 1 + idx;
          }
          break;
        }
        if ( is_intra )
        {
          break;
        }
      }
    }
  }
  // additional case can be added to insert IDR based on *pre-analysis* and other "pre-scient" tools

  return is_intra;
}

/*!
 ***********************************************************************
 * \brief
 *    Establish whether the frame with coding order "curr_frame" will be coded as an SP picture
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param curr_frame
 *    coding order of the current frame
 * \param avail_frames
 *    frames available for population
 * \return
 *    returns 1 when current frame will be an SP frame \n
 *    if PreferDispOrder == 1 then it returns 1 + disp_offset (for the SP frame)
 ***********************************************************************
 */

static int establish_sp( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames, int sim )
{
  int is_sp = 0;

  if ( p_Inp->sp_periodicity )
  {
    if ( !(p_Inp->PreferDispOrder) ) // coding order
    {
      if ( (curr_frame - p_seq_struct->last_sp_frame) % p_Inp->sp_periodicity == 0 )
      {
        is_sp = 1;
      }
    }
    else // display order (insertion)
    {
      int idx;
      int offset;
      PredStructAtom *p_cur_gop;

      // test each random access prediction structure, staring from the longest one
      // if random access is established then break from this loop and prepare output parameter
      for ( idx = (p_seq_struct->num_gops - 1); idx >= 0; idx-- )
      {
        p_cur_gop = p_seq_struct->p_gop + idx;
        // check if length of structure overflows the available frame number
        if ( sim )
        {
          if ( (curr_frame + p_cur_gop->length) > p_Inp->no_frames )
          {
            continue;
          }
        }
        else
        {
          if ( p_cur_gop->length > avail_frames )
          {
            continue;
          }
        }

        // recall that frame "0" is the random access frame in those prediction structures
        offset = p_cur_gop->p_frm[0].disp_offset;

        // now test if random access conditions are satisfied
        if ( (curr_frame + offset - p_seq_struct->last_sp_disp) % p_Inp->sp_periodicity == 0 )
        {
          is_sp = 1 + idx;
          break;
        }
      }
    }
  }

  return is_sp;
}

/*!
 ***********************************************************************
 * \brief
 *    Find closest fixed frame (IDR, intra, or slice set through some pre-analysis tool)
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param curr_frame
 *    coding order of the current frame
 * \param avail_frames
 *    frames available for population
 * \return
 *    returns the coding order of the first "fixed" frame, if none then -1
 ***********************************************************************
 */

static int get_fixed_frame( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames )
{
  int idx         = 0;
  int is_rnd_acc  = 0;
  int is_intra    = 0;
  int is_sp       = 0;  

  while ( idx < avail_frames )
  {
    is_rnd_acc = establish_random_access( p_Inp, p_seq_struct, curr_frame + idx, avail_frames - idx, 1 );
    is_intra   = establish_intra( p_Inp, p_seq_struct, curr_frame + idx, avail_frames - idx, 1 );
    is_sp      = establish_sp( p_Inp, p_seq_struct, curr_frame + idx, avail_frames - idx, 1 );

    if ( is_rnd_acc || is_intra || is_sp )
    {
      return idx;
    }
    idx++;
  }
  // signal that there is no fixed frame in the processed frames buffer (functions have to interpret the negative output here)
  return -1;
}

/*!
 ***********************************************************************
 * \brief
 *    Find closest fixed frame (IDR, intra, or slice set through some pre-analysis tool)
 *    for regular prediction structures and then try place the longest possible structure...
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param curr_frame
 *    coding order of the current frame
 * \param avail_frames
 *    frames available for population
 * \return
 *    returns the coding order of the first "fixed" frame, if none then -1
 ***********************************************************************
 */

static int get_fixed_frame_for_prd( InputParameters *p_Inp, SeqStructure *p_seq_struct, int curr_frame, int avail_frames )
{
  int idx         = 0;
  int is_rnd_acc  = 0;
  int is_intra    = 0;
  int is_sp       = 0;
  PredStructAtom *p_longest_prd = p_seq_struct->p_prd + p_seq_struct->num_prds - 1;

  while ( idx < avail_frames )
  {
    // IDRs
    is_rnd_acc = establish_random_access( p_Inp, p_seq_struct, curr_frame + idx, avail_frames - idx, 1 );
    if ( is_rnd_acc )
    {
      if ( p_Inp->PreferDispOrder && !(p_Inp->EnableIDRGOP) )
      {
        int gop_length;
        PredStructAtom *p_curr_gop;

        // decode is_intra
        if ( is_rnd_acc > 1 )
        {
          p_curr_gop = p_seq_struct->p_gop + is_rnd_acc - 1;
          gop_length = p_curr_gop->length;
        }
        else
        {
          gop_length = 1;
        }
        // consider the case where even though there were enough available frames to a large structure, these slots 
        // instead were allocated to a very long intra structure, which reduces compression efficiency
        if ( (idx % p_longest_prd->length) < p_longest_prd->length )
        {
          p_seq_struct->pop_flag = POP_IDR;
          return ( imin( p_longest_prd->length, idx + gop_length - 1 ) );
        }
        else
        {
          return idx;
        }
      }
      else
      {
        return idx;
      }
    }
    // Intras
    is_intra = establish_intra( p_Inp, p_seq_struct, curr_frame + idx, avail_frames - idx, 1 );
    if ( is_intra )
    {
      if ( p_Inp->PreferDispOrder )
      {
        int gop_length;
        PredStructAtom *p_curr_gop;

        // decode is_intra
        if ( is_intra > 1 )
        {
          p_curr_gop = p_seq_struct->p_intra_gop + is_intra - 1;
          gop_length = p_curr_gop->length;
        }
        else
        {
          gop_length = 1;
        }
        // consider the case where even though there were enough available frames to a large structure, these slots 
        // instead were allocated to a very long intra structure, which reduces compression efficiency
        if ( (idx % p_longest_prd->length) < p_longest_prd->length )
        {
          p_seq_struct->pop_flag = POP_INTRA;
          return ( imin( p_longest_prd->length, idx + gop_length - 1 ) );
        }
        else
        {
          return idx;
        }
      }
      else
      {
        return idx;
      }
    }
    // SP/SIs
    is_sp = establish_sp( p_Inp, p_seq_struct, curr_frame + idx, avail_frames - idx, 1 );
    if ( is_sp )
    {
      if ( p_Inp->PreferDispOrder && !(p_Inp->EnableIDRGOP) )
      {
        int gop_length;
        PredStructAtom *p_curr_gop;

        // decode is_intra
        if ( is_sp > 1 )
        {
          p_curr_gop = p_seq_struct->p_gop + is_sp - 1;
          gop_length = p_curr_gop->length;
        }
        else
        {
          gop_length = 1;
        }
        // consider the case where even though there were enough available frames to a large structure, these slots 
        // instead were allocated to a very long intra structure, which reduces compression efficiency
        if ( (idx % p_longest_prd->length) < p_longest_prd->length )
        {
          p_seq_struct->pop_flag = POP_SP;
          return ( imin( p_longest_prd->length, idx + gop_length - 1 ) );
        }
        else
        {
          return idx;
        }
      }
      else
      {
        return idx;
      }
    }
    idx++;
  }
  // signal that there is no fixed frame in the processed frames buffer (functions have to interpret the negative output here)
  return -1;
}

/*!
 ***********************************************************************
 * \brief
 *    Check if "number" is a power of two
 * \param number
 *    input value
 * \return
 *    1 if power of two, 0 otherwise
 ***********************************************************************
 */

static inline int check_power_of_two( int number )
{
  return (((~((~number) + 1)) & number ) == 0) ? 1 : 0;
}

/*!
 ***********************************************************************
 * \brief
 *    Select index to pred structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param num_frames
 *    frames available for population
 * \return
 *    returns the index to the selected prediction structure
 ***********************************************************************
 */

static int get_prd_index( InputParameters *p_Inp, SeqStructure *p_seq_struct, int num_frames )
{
  int curr_prd;
  PredStructAtom *p_cur_prd;

  // allocate atoms
  for ( curr_prd = (p_seq_struct->num_prds - 1); curr_prd >= 0; curr_prd-- )
  {
    p_cur_prd = p_seq_struct->p_prd + curr_prd;
    if ( p_Inp->PreferPowerOfTwo && curr_prd != (p_seq_struct->num_prds - 1) && !check_power_of_two( p_cur_prd->length ) )
    {
      continue;
    }
    if ( p_cur_prd->length <= num_frames )
    {
      return curr_prd;
    }
  }
  return 0;
}

/*!
 ***********************************************************************
 * \brief
 *    Select index to random access structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param num_frames
 *    frames available for population
 * \return
 *    returns the index to the selected prediction structure
 ***********************************************************************
 */

static int get_idr_index( InputParameters *p_Inp, SeqStructure *p_seq_struct, int num_frames )
{
  int curr_gop;
  PredStructAtom *p_cur_gop;

  // allocate atoms
  for ( curr_gop = (p_seq_struct->num_gops - 1); curr_gop >= 0; curr_gop-- )
  {
    p_cur_gop = p_seq_struct->p_gop + curr_gop;
    if ( p_Inp->PreferPowerOfTwo && curr_gop != (p_seq_struct->num_gops - 1) && !check_power_of_two( p_cur_gop->length ) )
    {
      continue;
    }
    if ( p_cur_gop->length <= num_frames )
    {
      return curr_gop;
    }
  }
  return 0;
}

/*!
 ***********************************************************************
 * \brief
 *    Select index to intra structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param num_frames
 *    frames available for population
 * \return
 *    returns the index to the selected prediction structure
 ***********************************************************************
 */

static int get_intra_index( InputParameters *p_Inp, SeqStructure *p_seq_struct, int num_frames )
{
  int curr_gop;
  PredStructAtom *p_cur_gop;

  // allocate atoms
  for ( curr_gop = (p_seq_struct->num_intra_gops - 1); curr_gop >= 0; curr_gop-- )
  {
    p_cur_gop = p_seq_struct->p_intra_gop + curr_gop;
    if ( p_Inp->PreferPowerOfTwo && curr_gop != (p_seq_struct->num_intra_gops - 1) && !check_power_of_two( p_cur_gop->length ) )
    {
      continue;
    }
    if ( p_cur_gop->length <= num_frames )
    {
      return curr_gop;
    }
  }
  return 0;
}

/*!
 ***********************************************************************
 * \brief
 *    Populate a PredStructFrm object
 * \param p_dst
 *    Pointer to the object to be populated with the args of this func.
 ***********************************************************************
 */

static inline void pop_pred_struct_frm( PredStructFrm *p_dst, int slice_type, int nal_ref_idc, int disp_offset, int layer, int slice_qp_off,
                                       int random_access, int temporal_layer )
{
  p_dst->slice_type     = slice_type;
  p_dst->nal_ref_idc    = nal_ref_idc;
  p_dst->disp_offset    = disp_offset;
  p_dst->layer          = layer;
  p_dst->slice_qp_off   = slice_qp_off;
  p_dst->random_access  = random_access;
  p_dst->temporal_layer = temporal_layer;
}

/*!
 ***********************************************************************
 * \brief
 *    Create random access prediction structures
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param memory_size
 *    frames available for population
 * \param is_idr
 *    initialize an IDR GOP list as opposed to an Intra GOP list
 * \return
 *    returns the index to the selected prediction structure
 ***********************************************************************
 */

static void init_gop_struct( InputParameters *p_Inp, SeqStructure *p_seq_struct, int is_idr, int *memory_size )
{
  int idx;
  int length;
  PredStructAtom *p_cur_gop, *p_cur_prd, *p_cur_prd_tail = NULL, *p_gop;
  PredStructFrm *p_dst, *p_src;

  if ( is_idr )
  {
    // derive number of gop atoms
    if ( p_Inp->idr_period == 1 || p_Inp->intra_period == 1 )
    {
      p_seq_struct->num_gops = 1;
    }
    else if ( p_Inp->intra_delay && !p_Inp->EnableIDRGOP )
    {
      p_seq_struct->num_gops = 3;
    }
    else if ( p_Inp->intra_delay || !p_Inp->EnableIDRGOP )
    {
      p_seq_struct->num_gops = !p_Inp->EnableIDRGOP ? p_seq_struct->num_prds : 2;
    }
    else
    {
      p_seq_struct->num_gops = !p_Inp->EnableIDRGOP ? p_seq_struct->num_prds : 1;
    }

    // we assume that intra_delay is ALWAYS lesser than the idr_period
    if ( p_Inp->idr_period )
    {
      assert( p_Inp->intra_delay < p_Inp->idr_period );
    }

    p_seq_struct->p_gop = (PredStructAtom *)malloc( p_seq_struct->num_gops * sizeof( PredStructAtom ) );
    *memory_size += ( p_seq_struct->num_gops * sizeof( PredStructAtom ) );

    p_gop = p_seq_struct->p_gop;
  }
  else // intra GOP
  {
    // derive number of gop atoms
    if ( p_Inp->intra_period == 1 )
    {
      p_seq_struct->num_intra_gops = 1;
    }
    else if ( p_Inp->intra_delay )
    {
      p_seq_struct->num_intra_gops = 3;
    }
    else
    {
      p_seq_struct->num_intra_gops = p_seq_struct->num_prds;
    }

    // we assume that intra_delay is ALWAYS lesser than the intra_period
    if ( p_Inp->intra_period )
    {
      assert( p_Inp->intra_delay < p_Inp->intra_period );
    }

    p_seq_struct->p_intra_gop = (PredStructAtom *)malloc( p_seq_struct->num_intra_gops * sizeof( PredStructAtom ) );
    *memory_size += ( p_seq_struct->num_intra_gops * sizeof( PredStructAtom ) );

    p_gop = p_seq_struct->p_intra_gop;
  }

  // default IDR GOP
  length = 1;
  p_cur_gop = p_gop;
  p_cur_gop->length = length;
  p_cur_gop->p_frm = (PredStructFrm *)malloc( length * sizeof( PredStructFrm ) );
  *memory_size += length * sizeof( PredStructFrm );

  // initialize atoms
  p_cur_gop = p_gop;
  p_dst = p_cur_gop->p_frm + 0;
  pop_pred_struct_frm( p_dst, I_SLICE, NALU_PRIORITY_HIGHEST, 0, 0, 0, 1, 0 );

  // intra-delayed IDR GOP
  if ( p_Inp->intra_delay && p_Inp->intra_period != 1 && p_Inp->idr_period != 1 )
  {
    int proc_frames = 0;
    int start_frame = p_Inp->intra_delay - 1;
    int idr_idx = 0;
    int curr_prd;

    // allocate atoms
    length = p_Inp->intra_delay + 1;
    p_cur_gop = p_gop + 1;
    p_cur_gop->length = length;
    p_cur_gop->gop_levels = 1;
    p_cur_gop->p_frm = (PredStructFrm *)malloc( length * sizeof( PredStructFrm ) );
    *memory_size += length * sizeof( PredStructFrm );

    // IDR picture
    p_cur_gop = p_gop + 1;
    p_dst = p_cur_gop->p_frm + idr_idx;
    pop_pred_struct_frm( p_dst, I_SLICE, NALU_PRIORITY_HIGHEST, p_Inp->intra_delay, 0, 0, 1, 0 );

    idr_idx++;

    while ( proc_frames < p_Inp->intra_delay )
    {
      // given the available slots select the index to the pred structure to use for current segment
      curr_prd = get_prd_index( p_Inp, p_seq_struct, p_Inp->intra_delay - proc_frames );
      p_cur_prd = p_seq_struct->p_prd + curr_prd;
      // set gop_levels
      p_cur_gop->gop_levels = p_cur_prd->gop_levels;
      // populate gop structure from selected structure
      for ( idx = 0; idx < p_cur_prd->length; idx++, idr_idx++, proc_frames++ )
      {
        p_dst = p_cur_gop->p_frm + idr_idx;
        p_src = p_cur_prd->p_frm + idx;

        pop_pred_struct_frm( p_dst, p_src->slice_type, p_src->nal_ref_idc, start_frame - p_src->disp_offset, p_src->layer,
          p_src->slice_qp_off, 0, p_src->temporal_layer );
      }      
      // update start_frame
      start_frame -= p_cur_prd->length;      
    }

    // a separate case
    if ( !p_Inp->EnableIDRGOP || !is_idr ) // an additional atom: the longest one
    {
      proc_frames = 0;
      start_frame = p_Inp->intra_delay - 1;
      idr_idx = 0;

      // longest structure
      p_cur_prd_tail = p_seq_struct->p_prd + p_seq_struct->num_prds - 1;

      // allocate atoms
      length = p_Inp->intra_delay + 1 + (p_cur_prd_tail->length - 1);
      p_cur_gop = p_gop + 2;
      p_cur_gop->length = length;
      p_cur_gop->gop_levels = 1;
      p_cur_gop->p_frm = (PredStructFrm *)malloc( length * sizeof( PredStructFrm ) );
      *memory_size += length * sizeof( PredStructFrm );
      idr_idx = 0;

      // IDR picture
      p_cur_gop = p_gop + 2;
      p_dst = p_cur_gop->p_frm + idr_idx;
      pop_pred_struct_frm( p_dst, I_SLICE, NALU_PRIORITY_HIGHEST, p_Inp->intra_delay, 0, 0, 1, 0 );

      idr_idx++;

      while ( proc_frames < p_Inp->intra_delay )
      {
        // given the available slots select the index to the pred structure to use for current segment
        curr_prd = get_prd_index( p_Inp, p_seq_struct, p_Inp->intra_delay - proc_frames );
        p_cur_prd = p_seq_struct->p_prd + curr_prd;
        // set gop_levels
        p_cur_gop->gop_levels = p_cur_prd->gop_levels;
        // populate gop structure from selected structure
        for ( idx = 0; idx < p_cur_prd->length; idx++, idr_idx++, proc_frames++ )
        {
          p_dst = p_cur_gop->p_frm + idr_idx;
          p_src = p_cur_prd->p_frm + idx;

          pop_pred_struct_frm( p_dst, p_src->slice_type, p_src->nal_ref_idc, start_frame - p_src->disp_offset, 
            p_src->layer, p_src->slice_qp_off, 0, p_src->temporal_layer );
        }      
        // update start_frame
        start_frame -= p_cur_prd->length;      
      }

      // add an offset to the display offsets
      for ( idr_idx = 0; idr_idx < (p_Inp->intra_delay + 1); idr_idx++ )
      {
        p_cur_gop->p_frm[idr_idx].disp_offset += (p_cur_prd_tail->length - 1);
      }
      p_cur_gop->gop_levels = imax( p_cur_gop->gop_levels, p_cur_prd_tail->gop_levels );
      // and now place the prd struct *before* them in display order (set idx to 1 to avoid copying the I/P frame, we only seek the hierarchy if it exists)
      for ( idx = 1; idx < p_cur_prd_tail->length; idx++, idr_idx++ )
      {
        p_dst = p_cur_gop->p_frm + idr_idx;
        p_src = p_cur_prd_tail->p_frm + idx;

        pop_pred_struct_frm( p_dst, p_src->slice_type, p_src->nal_ref_idc, p_src->disp_offset, 
          p_src->layer, p_src->slice_qp_off, 0, p_src->temporal_layer );
      }
    }
  }
  else if ( !(p_Inp->intra_delay) && ((!p_Inp->EnableIDRGOP || !is_idr) && p_Inp->intra_period != 1 && p_Inp->idr_period != 1) )
  {
    int gop_idx;
    int idr_idx = 0;

    for ( gop_idx = 1; gop_idx < p_seq_struct->num_prds; gop_idx++ )
    {
      // set structure
      p_cur_prd = p_seq_struct->p_prd + gop_idx;

      // allocate atoms
      length = p_cur_prd->length;
      p_cur_gop = p_gop + gop_idx;
      p_cur_gop->length = length;
      p_cur_gop->p_frm = (PredStructFrm *)malloc( length * sizeof( PredStructFrm ) );
      *memory_size += length * sizeof( PredStructFrm );
      idr_idx = 0;

      // IDR picture
      p_cur_gop->gop_levels = p_cur_prd->gop_levels;
      p_dst = p_cur_gop->p_frm + idr_idx;
      pop_pred_struct_frm( p_dst, I_SLICE, NALU_PRIORITY_HIGHEST, p_cur_prd->p_frm[0].disp_offset, 0, 0, 1, 0 );
      idr_idx++;

      // populate gop structure from selected structure
      for ( idx = 1; idx < p_cur_prd->length; idx++, idr_idx++ )
      {
        p_dst = p_cur_gop->p_frm + idr_idx;
        p_src = p_cur_prd->p_frm + idx;

        pop_pred_struct_frm( p_dst, p_src->slice_type, p_src->nal_ref_idc, p_src->disp_offset, 
          p_src->layer, p_src->slice_qp_off, 0, p_src->temporal_layer );
      }
    } // gop_idx
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Create non-random access prediction structures
 * \param p_Vid
 *    pointer to the VideoParameters structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param memory_size
 *    frames available for population
 ***********************************************************************
 */

static void init_pred_struct( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, int *memory_size )
{
  int curr_prd, idx;
  int frm, sum, layer, order;
  int length, length2, pred_mode;
  PredStructAtom *p_cur_prd;
  PredStructFrm *p_dst;

  p_seq_struct->num_prds = (p_Inp->idr_period == 1 || p_Inp->intra_period == 1) ? 1 : (p_Inp->NumberBFrames + 1);

  p_seq_struct->p_prd = (PredStructAtom *)malloc( p_seq_struct->num_prds * sizeof( PredStructAtom ) );
  *memory_size += p_seq_struct->num_prds * sizeof( PredStructAtom );

  // allocate atoms
  for ( curr_prd = (p_seq_struct->num_prds - 1); curr_prd >= 0; curr_prd-- )
  {
    // length of prediction structure
    length = curr_prd + 1;

    p_cur_prd = p_seq_struct->p_prd + curr_prd;
    p_cur_prd->length = length;
    p_cur_prd->p_frm = (PredStructFrm *)malloc( length * sizeof( PredStructFrm ) );
    *memory_size += length * sizeof( PredStructFrm );
  }

  // initialize atoms
  for ( curr_prd = (p_seq_struct->num_prds - 1); curr_prd >= 0; curr_prd-- )
  {
    // length of prediction structure
    length = curr_prd + 1;

    switch ( p_Inp->HierarchicalCoding )
    {
    default:
    case 0:
      pred_mode = 0;
      break;
    case 1:
      pred_mode = 1;
      break;
    case 2:
      // check if power of two
      pred_mode = check_power_of_two( length ) ? 2 : ( length > 4 ? 1 : 0 );
      break;
    case 3:
      // check again if power of two
      if ( p_Inp->LowDelay )
      {
        pred_mode = 3;
      }
      else
      {
        pred_mode = (length == (p_Inp->NumberBFrames + 1)) ? 3 : (check_power_of_two( length ) ? 2 : ( length > 4 ? 1 : 0 ));
      }
      break;
    }

    p_cur_prd = p_seq_struct->p_prd + curr_prd;
    p_cur_prd->gop_levels = 0;
    switch ( pred_mode )
    {
    default:
    case 0: // disposable B-pics of equal priority
      p_dst = p_cur_prd->p_frm + 0;

      p_dst->slice_type    = ( p_Inp->BRefPictures == 2 ) ? B_SLICE : P_SLICE;
      p_dst->nal_ref_idc   = NALU_PRIORITY_HIGH;
      p_dst->disp_offset   = length - 1;
      p_dst->layer         = 0;
      p_dst->slice_qp_off  = ( p_Inp->BRefPictures == 2 ) ? (p_Inp->qp[P_SLICE] - p_Inp->qp[B_SLICE]) : 0; // ensure all types of frames have identical QP (modifiers)
      p_dst->random_access = 0;
      p_dst->temporal_layer = 0;

      for ( idx = 1; idx < length; idx++ )
      {
        p_dst = p_cur_prd->p_frm + idx;

        p_dst->slice_type    = ( p_Inp->PReplaceBSlice ) ? P_SLICE : B_SLICE;
        p_dst->nal_ref_idc   = ( p_Inp->BRefPictures == 1 && p_dst->slice_type == B_SLICE ) ? NALU_PRIORITY_LOW : NALU_PRIORITY_DISPOSABLE;
        p_dst->disp_offset   = idx - 1;
        p_dst->layer         = 2;
        p_dst->slice_qp_off  = ( p_Inp->PReplaceBSlice ) ? (p_Inp->qp[B_SLICE] - p_Inp->qp[P_SLICE]) : (p_dst->nal_ref_idc ? p_Inp->qpBRSOffset : 0); //2; 2 when depends on QPPSlice (highest layer QP)
        p_dst->random_access = 0;
        p_dst->temporal_layer = 0;
      }
      if ( length > 1 )
      {
        p_cur_prd->gop_levels = 1;
      }
      break;
    case 1: // even/odd hierarchy
      length2 = ((length - 1) >> 1) + 1;

      p_dst = p_cur_prd->p_frm + 0;

      p_dst->slice_type    = ( p_Inp->BRefPictures == 2 ) ? B_SLICE : P_SLICE;
      p_dst->nal_ref_idc   = NALU_PRIORITY_HIGH;
      p_dst->disp_offset   = length - 1;
      p_dst->layer         = 0;
      p_dst->slice_qp_off  = ( p_Inp->BRefPictures == 2 ) ? (p_Inp->qp[B_SLICE] - p_Inp->qp[P_SLICE]) : 0; // ensure all types of frames have identical QP (modifiers)
      p_dst->random_access = 0;
      p_dst->temporal_layer = 0;

      for ( idx = 1; idx < length2; idx++ )
      {
        p_dst = p_cur_prd->p_frm + idx;

        p_dst->slice_type    = ( p_Inp->PReplaceBSlice ) ? P_SLICE : B_SLICE;
        p_dst->nal_ref_idc   = NALU_PRIORITY_LOW;
        p_dst->disp_offset   = 2*(idx - 1) + 1;
        p_dst->layer         = 1;
        p_dst->slice_qp_off  = p_dst->slice_type == B_SLICE ? 
          (p_Inp->HierarchyLevelQPEnable ? -1: p_Inp->qpBRSOffset) : (p_Inp->HierarchyLevelQPEnable ? 1 : 0); // 1
        p_dst->random_access = 0;
        p_dst->temporal_layer = 0;
      }
      for ( idx = length2; idx < length; idx++ )
      {
        p_dst = p_cur_prd->p_frm + idx;

        p_dst->slice_type    = ( p_Inp->PReplaceBSlice ) ? P_SLICE : B_SLICE;
        p_dst->nal_ref_idc   = NALU_PRIORITY_DISPOSABLE;
        p_dst->disp_offset   = (idx - length2) * 2;
        p_dst->layer         = 2;
        p_dst->slice_qp_off  = p_dst->slice_type == B_SLICE ? 0 : 2;//2;
        p_dst->random_access = 0;
        p_dst->temporal_layer = 0;
      }
      if ( length > 1 )
      {
        p_cur_prd->gop_levels = 2;
      }
      break;
    case 2: // full binary decomposition
      p_dst = p_cur_prd->p_frm + 0;

      p_dst->slice_type    = ( p_Inp->BRefPictures == 2 ) ? B_SLICE : P_SLICE;
      p_dst->nal_ref_idc   = NALU_PRIORITY_HIGH;
      p_dst->disp_offset   = length - 1;
      p_dst->layer         = 0;
      p_dst->slice_qp_off  = ( p_Inp->BRefPictures == 2 ) ? (p_Inp->qp[B_SLICE] - p_Inp->qp[P_SLICE]) : 0; // ensure all types of frames have identical QP (modifiers)
      p_dst->random_access = 0;
      p_dst->temporal_layer = 0;

      if ( length > 3 )
      {
        int GOPlevels = 1;

        // determine number of levels
        while ((length >> GOPlevels) > 1)
        {
          GOPlevels ++;
        }
        p_cur_prd->gop_levels = GOPlevels;

        // hierarchical layer (from highest priority to lowest priority)
        for ( layer = 1, order = 1, sum = 0, frm = 1; sum < (length - 1); frm = (frm << 1), sum += frm, layer++ )
        {
          // layer frames (in *coding* order)
          for ( idx = 0; idx < frm; idx++, order++ )
          {
            p_dst = p_cur_prd->p_frm + order;

            p_dst->slice_type    = ( p_Inp->PReplaceBSlice ) ? P_SLICE : B_SLICE;
            p_dst->nal_ref_idc   = (frm == (length >> 1)) ? NALU_PRIORITY_DISPOSABLE : NALU_PRIORITY_LOW;
            p_dst->disp_offset   = ((length - 1) >> layer) + idx * (length >> (layer - 1));
            p_dst->layer         = layer;
            p_dst->slice_qp_off  = p_dst->slice_type == B_SLICE ? 
              (p_Inp->HierarchyLevelQPEnable ? (-GOPlevels + layer): (p_dst->nal_ref_idc != NALU_PRIORITY_DISPOSABLE ? p_Inp->qpBRSOffset : 0)) : (p_Inp->HierarchyLevelQPEnable ? layer : 0); // layer
            p_dst->random_access = 0;
            p_dst->temporal_layer = 0;
          }
        }
      }
      else
      {
        for ( idx = 1; idx < length; idx++ )
        {
          p_dst = p_cur_prd->p_frm + idx;

          p_dst->slice_type    = ( p_Inp->PReplaceBSlice ) ? P_SLICE : B_SLICE;
          p_dst->nal_ref_idc   = NALU_PRIORITY_DISPOSABLE;
          p_dst->disp_offset   = idx - 1;
          p_dst->layer         = 2;
          p_dst->slice_qp_off  = 0; //2;
          p_dst->random_access = 0;
          p_dst->temporal_layer = 0;
        }
        if ( length > 1 )
        {
          p_cur_prd->gop_levels = 1;
        }
      }
      break;
    case 3: // explicit structure
      {
        int max_layer = 0;
        // encode without delay
        if ( p_Inp->LowDelay ) 
        {
          for ( idx = 0; idx < (length - 1); idx++ )
          {
            p_dst = p_cur_prd->p_frm + idx;

            p_dst->slice_type    = p_Vid->gop_structure[idx].slice_type;
            p_dst->nal_ref_idc   = p_Vid->gop_structure[idx].reference_idc;
            p_dst->disp_offset   = p_Vid->gop_structure[idx].display_no;
            p_dst->layer         = ((p_Vid->gop_structure[idx].hierarchy_layer == 1) ? 0 : 1) + 1; // invert in new notation
            p_dst->slice_qp_off  = p_Vid->gop_structure[idx].slice_qp_off;
            p_dst->random_access = 0;

            p_dst->temporal_layer = p_Vid->gop_structure[idx].temporal_layer; 
            if ( p_Vid->gop_structure[idx].hierarchy_layer > max_layer )
            {
              max_layer = p_Vid->gop_structure[idx].hierarchy_layer;
            }
          }
          p_dst = p_cur_prd->p_frm + idx;
          p_dst->slice_type    = ( p_Inp->BRefPictures == 2 ) ? B_SLICE : P_SLICE;
          p_dst->nal_ref_idc   = NALU_PRIORITY_HIGH;
          p_dst->disp_offset   = length - 1;
          p_dst->layer         = 0;
          p_dst->slice_qp_off  = ( p_Inp->BRefPictures == 2 ) ? (p_Inp->qp[B_SLICE] - p_Inp->qp[P_SLICE]) : 0; // ensure all types of frames have identical QP (modifiers)
          p_dst->random_access = 0;

          p_dst->temporal_layer = 0;
        } 
        else 
        {
          p_dst = p_cur_prd->p_frm + 0;

          p_dst->slice_type    = ( p_Inp->BRefPictures == 2 ) ? B_SLICE : P_SLICE;
          p_dst->nal_ref_idc   = NALU_PRIORITY_HIGH;
          p_dst->disp_offset   = length - 1;
          p_dst->layer         = 0;
          p_dst->slice_qp_off  = ( p_Inp->BRefPictures == 2 ) ? (p_Inp->qp[B_SLICE] - p_Inp->qp[P_SLICE]) : 0; // ensure all types of frames have identical QP (modifiers)
          p_dst->random_access = 0;
          p_dst->temporal_layer = 0;
          
          for ( idx = 1; idx < length; idx++ )
          {
            p_dst = p_cur_prd->p_frm + idx;

            p_dst->slice_type    = p_Vid->gop_structure[idx - 1].slice_type;
            p_dst->nal_ref_idc   = p_Vid->gop_structure[idx - 1].reference_idc;
            p_dst->disp_offset   = p_Vid->gop_structure[idx - 1].display_no;
            p_dst->layer         = ((p_Vid->gop_structure[idx - 1].hierarchy_layer == 1) ? 0 : 1) + 1; // invert in new notation
            p_dst->slice_qp_off  = p_Vid->gop_structure[idx - 1].slice_qp_off;
            p_dst->random_access = 0;
            p_dst->temporal_layer = 0;

            if ( p_Vid->gop_structure[idx - 1].hierarchy_layer > max_layer )
            {
              max_layer = p_Vid->gop_structure[idx - 1].hierarchy_layer;
            }
          }
        }
        p_cur_prd->gop_levels = max_layer + 1;
      }
      break;
    }
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Populate frame structure
 * \param p_Vid
 *    pointer to the VideoParameters structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param num_to_populate
 *    number of frame unit structures to populate
 ***********************************************************************
 */

void populate_frm_struct( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, int num_to_populate, int init_frames_to_code )
{
  // case 5: read explicit seq info and store it in the frm_struct
  int curr_frame; // in coding order
  int proc_frames;
  int is_idr;
  int is_intra;
  int is_sp;
  int terminate_pop = 0;

  p_seq_struct->curr_num_to_populate = imin( num_to_populate, init_frames_to_code - p_seq_struct->pop_start_frame );
  curr_frame  = p_seq_struct->pop_start_frame;
  proc_frames = 0;

  while ( proc_frames < p_seq_struct->curr_num_to_populate )
  {
    is_idr   = 0;
    is_intra = 0;
    is_sp    = 0;

    // we have to check here for inserted frames (e.g. due to pre-analysis) and assign them to the structure (future work)

    // establish if it is an IDR frame
    is_idr = establish_random_access( p_Inp, p_seq_struct, curr_frame, p_seq_struct->curr_num_to_populate - proc_frames, 0 );
    if ( is_idr && p_seq_struct->pop_flag != POP_INTRA && p_seq_struct->pop_flag != POP_SP )
    {
      // random access structure
      int gop_frame = populate_rnd_acc_pred_struct_atom( p_Vid, p_Inp, p_seq_struct, curr_frame, proc_frames, &terminate_pop, is_idr, 0 );
      // update counters
      curr_frame += gop_frame;
      proc_frames += gop_frame;
      if ( terminate_pop )
      {
        terminate_pop = 0;
        break; // the proc_frames main loop
      }
    }
    else // INTRA frames check (non-IDR)
    {
      is_intra = establish_intra( p_Inp, p_seq_struct, curr_frame, p_seq_struct->curr_num_to_populate - proc_frames, 0 );
      if ( is_intra && p_seq_struct->pop_flag != POP_SP )
      {
        // intra structure (not random access)
        int gop_frame = populate_rnd_acc_pred_struct_atom( p_Vid, p_Inp, p_seq_struct, curr_frame, proc_frames, &terminate_pop, is_intra, 1 );
        // update counters
        proc_frames += gop_frame;
        curr_frame  += gop_frame;
        if ( terminate_pop )
        {
          terminate_pop = 0;
          break; // the proc_frames main loop
        }
      }
      else // SP frames check
      {
        is_sp = establish_sp( p_Inp, p_seq_struct, curr_frame, p_seq_struct->curr_num_to_populate - proc_frames, 0 );
        if ( is_sp )
        {
          // intra structure (not random access)
          int gop_frame = populate_rnd_acc_pred_struct_atom( p_Vid, p_Inp, p_seq_struct, curr_frame, proc_frames, &terminate_pop, is_sp, 2 );
          // update counters
          proc_frames += gop_frame;
          curr_frame  += gop_frame;
          if ( terminate_pop )
          {
            terminate_pop = 0;
            break; // the proc_frames main loop
          }
        }
      }
    }  

    if ( !is_idr && !is_intra && !is_sp )
    {
      int pred_frame = populate_reg_pred_struct_atom( p_Vid, p_Inp, p_seq_struct, curr_frame, proc_frames, &terminate_pop );
      // update counters
      curr_frame += pred_frame;
      proc_frames += pred_frame;
      if ( terminate_pop )
      {
        terminate_pop = 0;
        break; // the proc_frames main loop
      }
    } // is_idr is_intra is_sp
  } // proc_frames loop
  // update pop_start_frame for subsequent run of this function ;)
  p_seq_struct->pop_start_frame = curr_frame;
}

/*!
 ***********************************************************************
 * \brief
 *    Set parameters for a single coded picture (frame/field)
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_pic
 *    pointer to the current picture structure
 * \param p_frm_struct
 *    pointer to the current frame unit structure
 * \param num_slices
 *    number of slices for the current picture
 * \param is_bot_fld
 *    bottom field flag
 ***********************************************************************
 */

void populate_reg_pic( InputParameters *p_Inp, PicStructure *p_pic, FrameUnitStruct *p_frm_struct, int num_slices, int is_bot_fld )
{
  int slice;
  int slice_type    = (is_bot_fld && p_frm_struct->type == I_SLICE) ? ( p_Inp->IntraBottom ? I_SLICE : (p_Inp->BRefPictures == 2 ? B_SLICE : P_SLICE) ) : p_frm_struct->type;
  int deblock       = p_Inp->DFSendParameters ? 
    (!(p_Inp->DFDisableIdc[p_frm_struct->nal_ref_idc][p_frm_struct->type])) : 1;
  int weighted_pred = (slice_type == P_SLICE) ?
    p_Inp->WeightedPrediction : ((slice_type == B_SLICE) ? 
    p_Inp->WeightedBiprediction : 0);
  SliceStructure *p_Slice;

  // if bottom field picture idr_flag has to be 0 (even when an IDR top field pic precedes it)
  p_pic->idr_flag      = !is_bot_fld ? p_frm_struct->idr_flag : 0;
  p_pic->random_access = !is_bot_fld ? p_frm_struct->random_access : 0;
  p_pic->nal_ref_idc   = p_frm_struct->nal_ref_idc;
  p_pic->num_slices    = num_slices;

  for ( slice = 0; slice < p_pic->num_slices; slice++ )
  {
    // Slice Structure
    p_Slice = p_pic->p_Slice + slice;

    p_Slice->type          = slice_type;
    p_Slice->deblock       = deblock;
    p_Slice->weighted_pred = weighted_pred;
    p_Slice->num_refs      = p_frm_struct->num_refs;
    p_Slice->qp            = p_frm_struct->qp;
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Set parameters for a single frame unit
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param p_frm_struct
 *    pointer to the current frame unit structure
 * \param p_cur_prd
 *    pointer to the current prediction structure
 * \param curr_frame
 *    global (sequence-level) coding order of the first frame (in coding order) in the PredStructAtom
 * \param pred_frame
 *    coding order with respect to "curr_frame" (pred_frame + curr_frame + idx yields the absolute coding order of the current frame)
 * \param idx
 *    local (PredStructAtom-level) coding order with respect to the start of the current PredStructAtom
 * \param num_slices
 *    number of slices for the current picture
 * \param no_rnd_acc
 *    if 1 the first frame in the Atom is not a random access frame: Intra \n
 *    if 2 it is an SP frame
 ***********************************************************************
 */

static void populate_frame( InputParameters *p_Inp, SeqStructure *p_seq_struct, FrameUnitStruct *p_frm_struct, PredStructAtom *p_cur_prd, 
                           int curr_frame, int pred_frame, int idx, int num_slices, int no_rnd_acc )
{
  PredStructFrm *p_src = p_cur_prd->p_frm + idx;

  // Frame Unit Structure
  p_frm_struct->type           = p_src->slice_type;
  p_frm_struct->random_access  = p_src->random_access;
  p_frm_struct->num_refs       = p_Inp->num_ref_frames;
  p_frm_struct->layer          = p_src->layer;
  p_frm_struct->mod_qp         = p_src->slice_qp_off;  
  p_frm_struct->field_pic_flag = 0; // need to add better support for field coded pictures
  p_frm_struct->frame_no       = curr_frame + pred_frame + p_src->disp_offset;

  p_frm_struct->temporal_layer = p_src->temporal_layer;

  if ( !(curr_frame + pred_frame + idx) )
  {
    // first frame in the sequence
    p_frm_struct->type        = I_SLICE;
    p_frm_struct->idr_flag    = 1;
    p_frm_struct->nal_ref_idc = NALU_PRIORITY_HIGHEST;
  }
  // disposable P/B frames
  else if ( p_Inp->DisposableP && p_frm_struct->type != I_SLICE && !p_frm_struct->layer ) // this way it will also apply to B_SLICEs in layer 0
  {
    if ( !(p_seq_struct->last_bl_frm_disposable) )
    {
      p_frm_struct->nal_ref_idc = NALU_PRIORITY_DISPOSABLE;
      p_frm_struct->mod_qp     += p_Inp->DispPQPOffset;
    }
    else
    {
      p_frm_struct->nal_ref_idc = p_src->nal_ref_idc;
    }
    p_frm_struct->idr_flag = (p_src->nal_ref_idc == NALU_PRIORITY_HIGHEST) ? 1 : 0;    
  }
  else if ( no_rnd_acc == 1 && !idx ) // no random access frame and frame first in the struct (hence I_SLICE)
  {
    p_frm_struct->idr_flag          = 0;
    p_frm_struct->nal_ref_idc       = NALU_PRIORITY_HIGH;
    p_frm_struct->random_access     = p_Inp->EnableOpenGOP ? 1 : 0;
  }
  else if ( no_rnd_acc == 2 && !idx ) // no random access frame and frame first in the struct (hence SP_SLICE)
  {
    p_frm_struct->type          = p_Inp->si_frame_indicator ? SI_SLICE : SP_SLICE;
    p_frm_struct->idr_flag      = 0;
    p_frm_struct->nal_ref_idc   = NALU_PRIORITY_HIGH;
    p_frm_struct->random_access = 0;
  }
  else
  {
    p_frm_struct->idr_flag    = (p_src->nal_ref_idc == NALU_PRIORITY_HIGHEST) ? 1 : 0;
    p_frm_struct->nal_ref_idc = p_src->nal_ref_idc;
  }
  // final QP (w/o RC) set here to ensure "mod_qp" and "type" have been *finalized*
  p_frm_struct->qp            = p_frm_struct->mod_qp + p_Inp->qp[p_frm_struct->type];
#if (DEBUG_PRED_STRUCT)
  fprintf( stderr, "\n\n Frame Population Stats for Frame %5d in coding order", curr_frame + pred_frame + idx );
  switch ( p_frm_struct->type )
  {
  default:
  case I_SLICE:
    fprintf( stderr, "\n Slice Type:            I_SLICE " );
    break;
  case P_SLICE:
    fprintf( stderr, "\n Slice Type:            P_SLICE " );
    break;
  case B_SLICE:
    fprintf( stderr, "\n Slice Type:            B_SLICE " );
    break;
  case SP_SLICE:
    fprintf( stderr, "\n Slice Type:            SP_SLICE " );
    break;
  case SI_SLICE:
    fprintf( stderr, "\n Slice Type:            SI_SLICE " );
    break;
  }
  fprintf( stderr, "\n IDR picture:           %d ", p_frm_struct->idr_flag );
  switch( p_frm_struct->nal_ref_idc )
  {
  default:
  case NALU_PRIORITY_HIGHEST:
    fprintf( stderr, "\n NAL_reference_idc:     NALU_PRIORITY_HIGHEST ");
    break;
  case NALU_PRIORITY_HIGH:
    fprintf( stderr, "\n NAL_reference_idc:     NALU_PRIORITY_HIGH ");
    break;
  case NALU_PRIORITY_LOW:
    fprintf( stderr, "\n NAL_reference_idc:     NALU_PRIORITY_LOW ");
    break;
  case NALU_PRIORITY_DISPOSABLE:
    fprintf( stderr, "\n NAL_reference_idc:     NALU_PRIORITY_DISPOSABLE ");
    break;
  }
  fprintf( stderr, "\n Random Access:         %d ", p_frm_struct->random_access );
  fprintf( stderr, "\n Number of References: %2d ", p_frm_struct->num_refs );
  fprintf( stderr, "\n Hierarchy Layer:       %d ", p_frm_struct->layer );
  fprintf( stderr, "\n QP Modifier:          %2d ", p_frm_struct->qp );
  fprintf( stderr, "\n Field Coding:          %d ", p_frm_struct->field_pic_flag );
  fprintf( stderr, "\n Display Order:     %5d ", p_frm_struct->frame_no );
#endif

  // store a pointer to the prediction structure
  p_frm_struct->p_atom   = p_cur_prd;
  p_frm_struct->atom_idx = idx;
#if (MVC_EXTENSION_ENABLE)
  p_frm_struct->view_id  = 0;
#endif

  // Picture Structure
  switch( p_Inp->PicInterlace )
  {
  default:
  case FRAME_CODING:
    // frame
    populate_reg_pic( p_Inp, p_frm_struct->p_frame_pic, p_frm_struct, num_slices, 0 );
    break;
  case FIELD_CODING:
    // top field
    populate_reg_pic( p_Inp, p_frm_struct->p_top_fld_pic, p_frm_struct, num_slices, 0 );
    // bottom field
    populate_reg_pic( p_Inp, p_frm_struct->p_bot_fld_pic, p_frm_struct, num_slices, 1 );
    break;
  case ADAPTIVE_CODING:
    // frame
    populate_reg_pic( p_Inp, p_frm_struct->p_frame_pic, p_frm_struct, num_slices, 0 );
    // top field
    populate_reg_pic( p_Inp, p_frm_struct->p_top_fld_pic, p_frm_struct, num_slices, 0 );
    // bottom field
    populate_reg_pic( p_Inp, p_frm_struct->p_bot_fld_pic, p_frm_struct, num_slices, 1 );
    break;
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Update frame indices
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param p_frm_struct
 *    pointer to the current frame unit structure
 * \param curr_frame
 *    global (sequence-level) coding order of the first frame (in coding order) in the PredStructAtom
 * \param pred_frame
 *    coding order with respect to "curr_frame" (pred_frame + curr_frame yields the absolute coding order of the current frame)
 ***********************************************************************
 */

static void update_frame_indices( SeqStructure *p_seq_struct, FrameUnitStruct *p_frm_struct, int curr_frame, int pred_frame )
{
  if ( p_frm_struct->idr_flag )
  {
    // coding order
    p_seq_struct->last_intra_frame = curr_frame + pred_frame;
    p_seq_struct->last_idr_frame   = curr_frame + pred_frame;
    p_seq_struct->last_random_access_frame = curr_frame + pred_frame;
    // display order
    p_seq_struct->last_intra_disp  = p_frm_struct->frame_no;
    p_seq_struct->last_idr_frame   = p_frm_struct->frame_no;
    p_seq_struct->last_rand_access_disp = p_frm_struct->frame_no;
    // sp frame counter (no need to send SP frames if random access frames)
    // coding order
    p_seq_struct->last_sp_frame = curr_frame + pred_frame;
    // display order
    p_seq_struct->last_sp_disp  = p_frm_struct->frame_no;
  }
  else if ( p_frm_struct->type == I_SLICE )
  {
    // coding order
    p_seq_struct->last_intra_frame = curr_frame + pred_frame;
    // display order
    p_seq_struct->last_intra_disp  = p_frm_struct->frame_no;
    if ( p_frm_struct->random_access )
    {
      // coding order
      p_seq_struct->last_random_access_frame = curr_frame + pred_frame;
      // display order
      p_seq_struct->last_rand_access_disp  = p_frm_struct->frame_no;
    }
  }
  else if ( p_frm_struct->type == SP_SLICE || p_frm_struct->type == SI_SLICE )
  {
    // coding order
    p_seq_struct->last_sp_frame = curr_frame + pred_frame;
    // display order
    p_seq_struct->last_sp_disp  = p_frm_struct->frame_no;
  }
  if ( p_frm_struct->layer == 0 ) // support for p_Inp->DisposableP
  {
    if ( p_frm_struct->nal_ref_idc == NALU_PRIORITY_DISPOSABLE )
    {
      p_seq_struct->last_bl_frm_disposable = 1;
    }
    else
    {
      p_seq_struct->last_bl_frm_disposable = 0;
    }
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Populate regular prediction structure atom
 * \param p_Vid
 *    pointer to the VideoParameters structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param curr_frame
 *    global (sequence-level) coding order of the first frame (in coding order) in the PredStructAtom
 * \param proc_frames
 *    frames that have been populated so far out of the "num_to_populate" frames
 * \param terminate_pop
 *    pointer to the terminate_pop variable that signals termination of the frame population loop
 * \return
 *    number of frames that have been populated by this function run
 ***********************************************************************
 */

static int populate_reg_pred_struct_atom( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, 
                                  int curr_frame, int proc_frames, int *terminate_pop )
{
  int idx;
  int fixed_idx;
  int avail_frames;
  int pred_frame = 0;
  int pred_idx;

  FrameUnitStruct *p_frm_struct;
  PredStructAtom *p_cur_prd;

  // regular prediction structure (*in-between* idr and intra frames though)
  // first determine how many frames are available to be used to place our prediction structures
  // this finds the next place for a fixed (IDR/intra/SP/SI) frame *after* current_frame (hence counts from current_frame + 1 on)
  if ( !(p_Inp->intra_delay) )
  {
    fixed_idx = get_fixed_frame_for_prd( p_Inp, p_seq_struct, curr_frame, p_seq_struct->curr_num_to_populate - proc_frames );
  }
  else
  {
    // IntraDelay currently does not support the improved prediction structure allocation (get_fixed_frame_for_prd)
    fixed_idx = get_fixed_frame( p_Inp, p_seq_struct, curr_frame, p_seq_struct->curr_num_to_populate - proc_frames );
  }
  if ( fixed_idx == -1 )
  {
    avail_frames = p_seq_struct->curr_num_to_populate - proc_frames;
  }
  else
  {
    avail_frames = fixed_idx; // frames including "curr_frame"
  }
  // in the future we have to take care of the avail_frames = 0 case
  if ( !avail_frames )
  {
    *terminate_pop = 1;
  }

  // loop through these frames and apply the appropriate prediction structure (p_prd)
  while ( pred_frame < avail_frames )
  {
    pred_idx = get_prd_index( p_Inp, p_seq_struct, avail_frames - pred_frame );
    // check here whether the prediction structure does not fit even if there is no fixed frame detected;
    // if we proceed we will allocate an inefficient pred structure; better to terminate the frame population here
    if ( fixed_idx == -1 && (curr_frame + avail_frames) < p_Inp->no_frames )
    {
      if ( (p_seq_struct->num_prds - 1) != pred_idx )
      {
        *terminate_pop = 1;
        break; // the pred_frame loop
      }
    }        
    // prediction structure pointer
    p_cur_prd = p_seq_struct->p_prd + pred_idx;
    // populate gop structure from selected structure
    // "idx" is internal pred struct idx
    // "pred_frame" is general index for the current frame starting from "curr_frame" (it is zero for "curr_frame")
    for ( idx = 0; idx < p_cur_prd->length; idx++ )//, pred_frame++ )
    {
      // set pointer
      p_frm_struct = p_seq_struct->p_frm + ((curr_frame + pred_frame + idx) % p_Vid->frm_struct_buffer);

      // assign values
      populate_frame( p_Inp, p_seq_struct, p_frm_struct, p_cur_prd, curr_frame, pred_frame, idx, p_seq_struct->max_num_slices, 0 );

      // update IDR and intra frame counters
      update_frame_indices( p_seq_struct, p_frm_struct, curr_frame, pred_frame + idx );
    }
    pred_frame += idx;
  }
  return pred_frame; // number of frames that have been populated by this function run
}

/*!
 ***********************************************************************
 * \brief
 *    Populate intra-coded/random access prediction structure atom
 * \param p_Vid
 *    pointer to the VideoParameters structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param curr_frame
 *    global (sequence-level) coding order of the first frame (in coding order) in the PredStructAtom
 * \param proc_frames
 *    frames that have been populated so far out of the "num_to_populate" frames
 * \param terminate_pop
 *    pointer to the terminate_pop variable that signals termination of the frame population loop
 * \param is_idr
 *    first frame of the atom will be IDR if > 0. is_idr - 1 is equal to the index of the Atom used to populate the frames
 * \param no_rnd_acc
 *    if 1, then the first frame in the (given) Atom is not a random access frame: Intra \n
 *    if 2, then it is an SP frame
 * \return
 *    number of frames that have been populated by this function run
 ***********************************************************************
 */

static int populate_rnd_acc_pred_struct_atom( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, 
                                             int curr_frame, int proc_frames, int *terminate_pop, int is_idr, int no_rnd_acc )
{
  // insert IDR pred structure (can be *more* than one frame)
  int idx;
  int fixed_idx;
  int avail_frames;
  int gop_frame = 0;
  int gop_idx;
  int num_gops;

  FrameUnitStruct *p_frm_struct;
  PredStructAtom *p_cur_gop;
  PredStructAtom *p_gop;
  int (*get_rand_acc_index)( InputParameters *, SeqStructure *, int );

  if ( no_rnd_acc == 1 ) // intra
  {
    p_gop = p_seq_struct->p_intra_gop;
    num_gops = p_seq_struct->num_intra_gops;
    get_rand_acc_index = get_intra_index;
  }
  else
  {
    p_gop = p_seq_struct->p_gop;
    num_gops = p_seq_struct->num_gops;
    get_rand_acc_index = get_idr_index;
  }

  // regular prediction structure (*in-between* idr and intra frames though)
  // first determine how many frames are available to be used to place our prediction structures
  // this finds the next place for a fixed (IDR/intra/SP/SI) frame *after* current_frame (hence counts from current_frame + 1 on)
  fixed_idx = get_fixed_frame( p_Inp, p_seq_struct, curr_frame + 1, p_seq_struct->curr_num_to_populate - proc_frames - 1 );
  if ( fixed_idx == -1 )
  {
    avail_frames = p_seq_struct->curr_num_to_populate - proc_frames;
  }
  else
  {
    avail_frames = fixed_idx + 1; // frames including "curr_frame"
  }
  // in the future we have to take care of the avail_frames = 0 case

  // select prediction structure for random access
  if ( !curr_frame && !(p_Inp->EnableIDRGOP) && !(p_Inp->intra_delay) )
  {
    gop_idx = 0; // force shortest IDR pred struct for first frame
  }
  else if ( !curr_frame && !(p_Inp->EnableIDRGOP) && p_Inp->intra_delay )
  {
    gop_idx = 1; // force middle IDR pred struct (2 corresponds to the longest that is though reserved for within the sequence)
  }
  else
  {
    if ( !(p_Inp->PreferDispOrder) ) // coding order
    {
      // bias in favor of longest *available* and *possible* structure
      gop_idx = get_rand_acc_index( p_Inp, p_seq_struct, avail_frames );
    }
    else // display order
    {
      // selection here is *fixed* and is signaled within is_idr
      gop_idx = is_idr - 1;
    }
    // check here whether the prediction structure does not fit even if there is no fixed frame detected; if we proceed we will allocate an inefficient pred structure; better to terminate the frame population here
    if ( fixed_idx == -1 && (curr_frame + avail_frames) < p_Inp->no_frames )
    {
      if ( (num_gops - 1) != gop_idx && !p_seq_struct->pop_flag )
      {
        *terminate_pop = 1;
        return 0;
      }
    }
  }
  
  // reset flag
  if ( p_seq_struct->pop_flag )
  {
    p_seq_struct->pop_flag = 0;
  }

  // prediction structure pointer
  p_cur_gop = p_gop + gop_idx;
  // populate gop structure from selected structure
  for ( idx = 0; idx < p_cur_gop->length; idx++ )//, gop_frame++ )
  {
    // set pointer
    p_frm_struct = p_seq_struct->p_frm + ((curr_frame + gop_frame + idx) % p_Vid->frm_struct_buffer);
    
    // assign values
    populate_frame( p_Inp, p_seq_struct, p_frm_struct, p_cur_gop, curr_frame, gop_frame, idx, p_seq_struct->max_num_slices, no_rnd_acc );

    // update IDR and intra frame counters
    update_frame_indices( p_seq_struct, p_frm_struct, curr_frame, gop_frame + idx );
  }
  gop_frame += idx;

  return gop_frame; // populated frames
}

/*!
 ***********************************************************************
 * \brief
 *    Free random access prediction structures
 * \param p_seq_struct
 *    pointer to the sequence structure
 ***********************************************************************
 */

static void free_gop_struct( SeqStructure *p_seq_struct )
{
  int idx;
  PredStructAtom *p_cur_gop;

  // IDR GOPs
  for ( idx = 0; idx < p_seq_struct->num_gops; idx++ )
  {
    p_cur_gop = p_seq_struct->p_gop + idx;
    free( p_cur_gop->p_frm );
  }
  free( p_seq_struct->p_gop );
  // Intra GOPs
  for ( idx = 0; idx < p_seq_struct->num_intra_gops; idx++ )
  {
    p_cur_gop = p_seq_struct->p_intra_gop + idx;
    free( p_cur_gop->p_frm );
  }
  free( p_seq_struct->p_intra_gop );
}

/*!
 ***********************************************************************
 * \brief
 *    Free non-random access prediction structures
 * \param p_seq_struct
 *    pointer to the sequence structure
 ***********************************************************************
 */

static void free_pred_struct( SeqStructure *p_seq_struct )
{
  int idx;
  PredStructAtom *p_cur_prd;

  // free pred structures
  for ( idx = 0; idx < p_seq_struct->num_prds; idx++ )
  {
    p_cur_prd = p_seq_struct->p_prd + idx;
    free( p_cur_prd->p_frm );
  }
  free( p_seq_struct->p_prd );
}

/*!
 ***********************************************************************
 * \brief
 *    Set parameters for a single frame from the explicit structure
 * \param info
 *    pointer to the Explicit Structure information for the current frame
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_frm_struct
 *    pointer to the current frame structure
 * \param num_slices
 *    number of slices used to code this picture
 ***********************************************************************
 */

void populate_frame_explicit( ExpFrameInfo *info, InputParameters *p_Inp, FrameUnitStruct *p_frm_struct, int num_slices )
{
  // Frame Unit Structure
  p_frm_struct->idr_flag       = info->is_idr;
  p_frm_struct->nal_ref_idc    = info->reference_idc;
  p_frm_struct->type           = info->slice_type;
  p_frm_struct->random_access  = info->is_idr;
  p_frm_struct->num_refs       = p_Inp->num_ref_frames;
  p_frm_struct->layer          = info->reference_idc ? 0 : 1;
  p_frm_struct->mod_qp         = 0;
  p_frm_struct->field_pic_flag = 0;
  p_frm_struct->frame_no       = info->seq_number;
  p_frm_struct->qp             = p_frm_struct->mod_qp + p_Inp->qp[p_frm_struct->type];

  // Picture Structure
  switch( p_Inp->PicInterlace )
  {
  default:
  case FRAME_CODING:
    // frame
    populate_reg_pic( p_Inp, p_frm_struct->p_frame_pic, p_frm_struct, num_slices, 0 );
    break;
  case FIELD_CODING:
    // top field
    populate_reg_pic( p_Inp, p_frm_struct->p_top_fld_pic, p_frm_struct, num_slices, 0 );
    // bottom field
    populate_reg_pic( p_Inp, p_frm_struct->p_bot_fld_pic, p_frm_struct, num_slices, 1 );
    break;
  case ADAPTIVE_CODING:
    // frame
    populate_reg_pic( p_Inp, p_frm_struct->p_frame_pic, p_frm_struct, num_slices, 0 );
    // top field
    populate_reg_pic( p_Inp, p_frm_struct->p_top_fld_pic, p_frm_struct, num_slices, 0 );
    // bottom field
    populate_reg_pic( p_Inp, p_frm_struct->p_bot_fld_pic, p_frm_struct, num_slices, 1 );
    break;
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Set parameters for a single frame during RD-picture decision
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_frm_struct
 *    pointer to the current frame structure
 * \param slice_type
 *    NEW slice type that is set
 * \param num_slices
 *    number of slices used to code this picture
 ***********************************************************************
 */

void populate_frame_slice_type( InputParameters *p_Inp, FrameUnitStruct *p_frm_struct, int slice_type, int num_slices )
{
  p_frm_struct->type = slice_type;

  // frame (RDPictureDecision only called for frame pictures so no need to check here)
  populate_reg_pic( p_Inp, p_frm_struct->p_frame_pic, p_frm_struct, num_slices, 0 );
}

#if (MVC_EXTENSION_ENABLE)
/*!
 ***********************************************************************
 * \brief
 *    Populate frame structure
 * \param p_Vid
 *    pointer to the VideoParameters structure
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_seq_struct
 *    pointer to the sequence structure
 * \param start
 *    starting number of frame unit structures to populate
 * \param end
 *    starting number of frame unit structures to populate
 ***********************************************************************
 */

void populate_frm_struct_mvc( VideoParameters *p_Vid, InputParameters *p_Inp, SeqStructure *p_seq_struct, int start, int end )
{
  int view_id;
  int idx;
  FrameUnitStruct *p_curr_frm, *p_frm = p_seq_struct->p_frm;
  FrameUnitStruct *p_curr_frm_mvc, *p_frm_mvc = p_seq_struct->p_frm_mvc;

  if ( p_Inp->num_of_views == 2 )
  {
    for ( idx = start; idx < end; idx++ )
    {
      p_curr_frm = p_frm + (idx % p_Vid->frm_struct_buffer);

      // view 0
      p_curr_frm_mvc = p_frm_mvc + ((idx << 1) % p_seq_struct->num_frames_mvc);
      copy_frame_mvc( p_Inp, p_curr_frm, p_curr_frm_mvc, p_seq_struct->max_num_slices, 0 );
      // view 1
      p_curr_frm_mvc = p_frm_mvc + (((idx << 1) + 1) % p_seq_struct->num_frames_mvc);
      copy_frame_mvc( p_Inp, p_curr_frm, p_curr_frm_mvc, p_seq_struct->max_num_slices, 1 );
    }
  }
  else
  {
    assert( p_Inp->num_of_views > 2 );
    for ( idx = start; idx < end; idx++ )
    {
      p_curr_frm = p_frm + (idx % p_Vid->frm_struct_buffer);

      for ( view_id = 0; view_id < p_Inp->num_of_views; view_id++ )
      {
        // view "view_id"
        p_curr_frm_mvc = p_frm_mvc + (((idx * p_Inp->num_of_views) + view_id) % p_seq_struct->num_frames_mvc);
        copy_frame_mvc( p_Inp, p_curr_frm, p_curr_frm_mvc, p_seq_struct->max_num_slices, view_id );
      }
    }
  }
}


/*!
 ***********************************************************************
 * \brief
 *    Set parameters for a single frame unit for MVC compression
 * \param p_Inp
 *    pointer to the InputParameters structure
 * \param p_src
 *    pointer to the source frame unit structure
 * \param p_dst
 *    pointer to the destination frame unit structure
 * \param num_slices
 *    number of slices for the current picture
 * \param view_id
 *    view id
 ***********************************************************************
 */


static void copy_frame_mvc( InputParameters *p_Inp, FrameUnitStruct *p_src, FrameUnitStruct *p_dst, int num_slices, int view_id )
{
  // Frame Unit Structure
  p_dst->type           = view_id ? p_Inp->MVCInterViewForceB != 0? B_SLICE : (p_src->type == I_SLICE ? (p_Inp->BRefPictures == 2 ? B_SLICE : P_SLICE) : p_src->type) : p_src->type;
  p_dst->random_access  = p_src->random_access;
  p_dst->num_refs       = p_src->num_refs;
  p_dst->layer          = p_src->layer;
  p_dst->mod_qp         = p_src->mod_qp;  
  p_dst->field_pic_flag = p_src->field_pic_flag;
  p_dst->frame_no       = p_src->frame_no;
  p_dst->idr_flag       = view_id ? 0 : p_src->idr_flag;
  p_dst->nal_ref_idc    = view_id ? (p_src->idr_flag ? NALU_PRIORITY_HIGH : p_src->nal_ref_idc) : p_src->nal_ref_idc;
  p_dst->qp             = p_src->qp;
  p_dst->p_atom         = p_src->p_atom;
  p_dst->atom_idx       = p_src->atom_idx;
  p_dst->view_id        = view_id;

  // Picture Structure
  switch( p_Inp->PicInterlace )
  {
  default:
  case FRAME_CODING:
    // frame
    populate_reg_pic( p_Inp, p_dst->p_frame_pic, p_dst, num_slices, 0 );
    break;
  case FIELD_CODING:
    // top field
    populate_reg_pic( p_Inp, p_dst->p_top_fld_pic, p_dst, num_slices, 0 );
    // bottom field
    populate_reg_pic( p_Inp, p_dst->p_bot_fld_pic, p_dst, num_slices, 1 );
    break;
  case ADAPTIVE_CODING:
    // frame
    populate_reg_pic( p_Inp, p_dst->p_frame_pic, p_dst, num_slices, 0 );
    // top field
    populate_reg_pic( p_Inp, p_dst->p_top_fld_pic, p_dst, num_slices, 0 );
    // bottom field
    populate_reg_pic( p_Inp, p_dst->p_bot_fld_pic, p_dst, num_slices, 1 );
    break;
  }
}
#endif
