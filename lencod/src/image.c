
/*!
 *************************************************************************************
 * \file image.c
 *
 * \brief
 *    Code one image/slice
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Inge Lille-Langoy               <inge.lille-langoy@telenor.com>
 *     - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *     - Jani Lainema                    <jani.lainema@nokia.com>
 *     - Sebastian Purreiter             <sebastian.purreiter@mch.siemens.de>
 *     - Byeong-Moon Jeon                <jeonbm@lge.com>
 *     - Yoon-Seong Soh                  <yunsung@lge.com>
 *     - Thomas Stockhammer              <stockhammer@ei.tum.de>
 *     - Detlev Marpe
 *     - Guido Heising
 *     - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *     - Ragip Kurceren                  <ragip.kurceren@nokia.com>
 *     - Antti Hallapuro                 <antti.hallapuro@nokia.com>
 *     - Alexis Michael Tourapis         <alexismt@ieee.org>
 *     - Athanasios Leontaris            <aleon@dolby.com>
 *     - Yan Ye                          <yye@dolby.com>
 *************************************************************************************
 */
#include "contributors.h"

#include <math.h>
#include <time.h>

#include "global.h"

#include "filehandle.h"
#include "mbuffer.h"
#include "img_luma.h"
#include "img_chroma.h"
#include "img_distortion.h"
#include "intrarefresh.h"
#include "slice.h"
#include "fmo.h"
#include "sei.h"
#include "memalloc.h"
#include "fast_memory.h"
#include "nalu.h"
#include "ratectl.h"
#include "mb_access.h"
#include "context_ini.h"
#include "biariencode.h"
#include "enc_statistics.h"
#include "conformance.h"
#include "report.h"

#include "q_matrix.h"
#include "q_offsets.h"
#include "wp.h"
#include "input.h"
#include "image.h"
#include "errdo.h"
#include "img_process.h"
#include "rdopt.h"
#include "sei.h"
#include "configfile.h"
#include "mbuffer.h"

#include "resize.h"
#include "md_common.h"
#include "me_epzs_common.h"
#include "me_hme.h"

extern void UpdateDecoders            (VideoParameters *p_Vid, InputParameters *p_Inp, StorablePicture *enc_pic);

extern void DeblockFrame              (VideoParameters *p_Vid, imgpel **, imgpel ***);

static void code_a_picture            (VideoParameters *p_Vid, Picture *pic);
static void field_picture             (VideoParameters *p_Vid, Picture *top, Picture *bottom);
static void prepare_enc_frame_picture (VideoParameters *p_Vid, StorablePicture **stored_pic);
#if (MVC_EXTENSION_ENABLE)
static void writeout_picture          (VideoParameters *p_Vid, Picture *pic, int is_bottom);
#else
static void writeout_picture          (VideoParameters *p_Vid, Picture *pic);
#endif
static byte picture_structure_decision(VideoParameters *p_Vid, Picture *frame, Picture *top, Picture *bot);
static void distortion_fld            (VideoParameters *p_Vid, InputParameters *p_Inp, Picture *field_pic, ImageData *imgData);

static void field_mode_buffer (VideoParameters *p_Vid);
static void frame_mode_buffer (VideoParameters *p_Vid);
static void init_frame        (VideoParameters *p_Vid, InputParameters *p_Inp);
static void init_field        (VideoParameters *p_Vid, InputParameters *p_Inp);
static void put_buffer_frame  (VideoParameters *p_Vid);
static void put_buffer_top    (VideoParameters *p_Vid);
static void put_buffer_bot    (VideoParameters *p_Vid);

static void ReportFirstframe   (VideoParameters *p_Vid, int64 tmp_time);
static void ReportI            (VideoParameters *p_Vid, int64 tmp_time);
static void ReportP            (VideoParameters *p_Vid, int64 tmp_time);
static void ReportB            (VideoParameters *p_Vid, int64 tmp_time);
static void ReportNALNonVLCBits(VideoParameters *p_Vid, int64 tmp_time);

extern void rd_picture_coding(VideoParameters *p_Vid, InputParameters *p_Inp);

void MbAffPostProc(VideoParameters *p_Vid)
{
  imgpel temp[32][16];

  StorablePicture *p_Enc_Pic = p_Vid->enc_picture;
  imgpel ** imgY  = p_Enc_Pic->imgY;
  imgpel ***imgUV = p_Enc_Pic->imgUV;
  short i, y, x0, y0, uv;

  if (p_Vid->yuv_format != YUV400)
  {
    for (i=0; i<(int)p_Vid->PicSizeInMbs; i+=2)
    {
      if (p_Enc_Pic->motion.mb_field[i])
      {
        get_mb_pos(p_Vid, i, p_Vid->mb_size[IS_LUMA], &x0, &y0);
        for (y=0; y<(2*MB_BLOCK_SIZE);y++)
          memcpy(&temp[y],&imgY[y0+y][x0], MB_BLOCK_SIZE * sizeof(imgpel));

        for (y=0; y<MB_BLOCK_SIZE;y++)
        {
          memcpy(&imgY[y0+(2*y)][x0],temp[y], MB_BLOCK_SIZE * sizeof(imgpel));
          memcpy(&imgY[y0+(2*y + 1)][x0],temp[y+ MB_BLOCK_SIZE], MB_BLOCK_SIZE * sizeof(imgpel));
        }

        x0 = x0 / (16/p_Vid->mb_cr_size_x);
        y0 = y0 / (16/p_Vid->mb_cr_size_y);

        for (uv=0; uv<2; uv++)
        {
          for (y=0; y < (2 * p_Vid->mb_cr_size_y); y++)
            memcpy(&temp[y],&imgUV[uv][y0+y][x0], p_Vid->mb_cr_size_x * sizeof(imgpel));

          for (y=0; y<p_Vid->mb_cr_size_y;y++)
          {
            memcpy(&imgUV[uv][y0+(2*y)][x0],temp[y], p_Vid->mb_cr_size_x * sizeof(imgpel));
            memcpy(&imgUV[uv][y0+(2*y + 1)][x0],temp[y+ p_Vid->mb_cr_size_y], p_Vid->mb_cr_size_x * sizeof(imgpel));
          }
        }
      }
    }
  }
  else
  {
    for (i=0; i<(int)p_Vid->PicSizeInMbs; i+=2)
    {
      if (p_Enc_Pic->motion.mb_field[i])
      {
        get_mb_pos(p_Vid, i, p_Vid->mb_size[IS_LUMA], &x0, &y0);
        for (y=0; y<(2*MB_BLOCK_SIZE);y++)
          memcpy(&temp[y],&imgY[y0+y][x0], MB_BLOCK_SIZE * sizeof(imgpel));

        for (y=0; y<MB_BLOCK_SIZE;y++)
        {
          memcpy(&imgY[y0+(2*y)][x0],temp[y], MB_BLOCK_SIZE * sizeof(imgpel));
          memcpy(&imgY[y0+(2*y + 1)][x0],temp[y+ MB_BLOCK_SIZE], MB_BLOCK_SIZE * sizeof(imgpel));
        }
      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Sets slice type
 *
 ************************************************************************
 */
void set_slice_type(VideoParameters *p_Vid, InputParameters *p_Inp, int slice_type)
{
  p_Vid->type    = (short) slice_type;            // set slice type
  p_Vid->RCMinQP = p_Inp->RCMinQP[p_Vid->type];
  p_Vid->RCMaxQP = p_Inp->RCMaxQP[p_Vid->type];
}

static void code_a_plane(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  unsigned int NumberOfCodedMBs = 0;
  int SliceGroup = 0;
  DistMetric distortion; 

  // The slice_group_change_cycle can be changed here.
  // FmoInit() is called before coding each picture, frame or field
  p_Vid->slice_group_change_cycle=1;
  FmoInit(p_Vid, p_Vid->active_pps, p_Vid->active_sps);
  FmoStartPicture (p_Vid);           //! picture level initialization of FMO

  CalculateQuant4x4Param (p_Vid);
  CalculateOffset4x4Param(p_Vid);

  if(p_Inp->Transform8x8Mode)
  {
    CalculateQuant8x8Param (p_Vid);
    CalculateOffset8x8Param(p_Vid);
  }

  reset_pic_bin_count(p_Vid);
  p_Vid->bytes_in_picture = 0;

  while (NumberOfCodedMBs < p_Vid->PicSizeInMbs)       // loop over slices
  {
    // Encode one SLice Group
    while (!FmoSliceGroupCompletelyCoded (p_Vid, SliceGroup))
    {
      // Encode the current slice
      if (!p_Vid->mb_aff_frame_flag)
        NumberOfCodedMBs += encode_one_slice (p_Vid, SliceGroup, NumberOfCodedMBs);
      else
        NumberOfCodedMBs += encode_one_slice_MBAFF (p_Vid, SliceGroup, NumberOfCodedMBs);

      FmoSetLastMacroblockInSlice (p_Vid, p_Vid->current_mb_nr);
      // Proceed to next slice
      p_Vid->current_slice_nr++;
      p_Vid->p_Stats->bit_slice = 0;
    }
    // Proceed to next SliceGroup
    SliceGroup++;
  }
  FmoEndPicture ();

  if ((p_Inp->SkipDeBlockNonRef == 0) || (p_Vid->nal_reference_idc != 0))
  {
    if(p_Inp->RDPictureDeblocking && !p_Vid->TurnDBOff)
    {
      find_distortion(p_Vid, &p_Vid->imgData);
      distortion.value[0] = p_Vid->p_Dist->metric[SSE].value[0];
      distortion.value[1] = p_Vid->p_Dist->metric[SSE].value[1];
      distortion.value[2] = p_Vid->p_Dist->metric[SSE].value[2];
    }
    else
      distortion.value[0] = distortion.value[1] = distortion.value[2] = 0;

    DeblockFrame (p_Vid, p_Vid->enc_picture->imgY, p_Vid->enc_picture->imgUV); //comment out to disable deblocking filter

    if(p_Inp->RDPictureDeblocking && !p_Vid->TurnDBOff)
    {
      find_distortion(p_Vid, &p_Vid->imgData);
      if(distortion.value[0]+distortion.value[1]+distortion.value[2] < 
        p_Vid->p_Dist->metric[SSE].value[0]+
        p_Vid->p_Dist->metric[SSE].value[1]+
        p_Vid->p_Dist->metric[SSE].value[2])
      {
        p_Vid->EvaluateDBOff = 1; 
      }
    }
  }

}
/*!
 ************************************************************************
 * \brief
 *    Encodes a picture
 *
 *    This is the main picture coding loop.. It is called by all this
 *    frame and field coding stuff after the p_Vid-> elements have been
 *    set up.  Not sure whether it is useful for MB-adaptive frame/field
 *    coding
 ************************************************************************
 */
static void code_a_picture(VideoParameters *p_Vid, Picture *pic)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int pl;

  p_Vid->currentPicture = pic;
  p_Vid->currentPicture->idr_flag = get_idr_flag(p_Vid);

  pic->no_slices = 0;

  RandomIntraNewPicture (p_Vid);     //! Allocates forced INTRA MBs (even for fields!)
  if( (p_Inp->separate_colour_plane_flag != 0) )
  {
    for( pl=0; pl<MAX_PLANE; pl++ )
    {
      p_Vid->current_mb_nr = 0;
      p_Vid->current_slice_nr = 0;
      p_Vid->SumFrameQP = 0;
      p_Vid->num_ref_idx_l0_active = 0;
      p_Vid->num_ref_idx_l1_active = 0;

      p_Vid->colour_plane_id = (char) pl;
      
      code_a_plane(p_Vid, p_Inp);
    }
  }
  else
  {
    code_a_plane(p_Vid, p_Inp);
  }

  if (p_Vid->mb_aff_frame_flag)
    MbAffPostProc(p_Vid);

}

/*!
 ************************************************************************
 * \brief
 *    Determine whether picture is coded as IDR
 ************************************************************************
 */
byte get_idr_flag( VideoParameters *p_Vid )
{
  int idr_flag;
  FrameUnitStruct *p_cur_frm = p_Vid->p_curr_frm_struct;

  switch( p_Vid->structure )
  {
  default:
  case FRAME:
    idr_flag = p_cur_frm->p_frame_pic->idr_flag;
    break;
  case TOP_FIELD:
    idr_flag = p_cur_frm->p_top_fld_pic->idr_flag;
    break;
  case BOTTOM_FIELD:
    idr_flag = p_cur_frm->p_bot_fld_pic->idr_flag;
    break;
  }

  return (byte) idr_flag;
}

/*!
 ************************************************************************
 * \brief
 *    Determine whether picture is coded as IDR or Openg GOP Intra
 ************************************************************************
 */
byte get_random_access_flag( VideoParameters *p_Vid )
{
  int random_access;
  FrameUnitStruct *p_cur_frm = p_Vid->p_curr_frm_struct;

  switch( p_Vid->structure )
  {
  default:
  case FRAME:
    random_access = p_cur_frm->p_frame_pic->random_access;
    break;
  case TOP_FIELD:
    random_access = p_cur_frm->p_top_fld_pic->random_access;
    break;
  case BOTTOM_FIELD:
    random_access = p_cur_frm->p_bot_fld_pic->random_access;
    break;
  }

  return (byte) random_access;
}

/*!
 ************************************************************************
 * \brief
 *    Update global stats
 ************************************************************************
 */
void update_global_stats(InputParameters *p_Inp, StatParameters *gl_stats, StatParameters *cur_stats)
{  
  int i, j, k;
  
  if (p_Inp->skip_gl_stats == 0)
  {
    for (i = 0; i < 4; i++)
    {
      gl_stats->intra_chroma_mode[i]    += cur_stats->intra_chroma_mode[i];
    }

    for (i = 0; i < 5; i++)
    {
      gl_stats->quant[i]                 += cur_stats->quant[i];
      gl_stats->num_macroblocks[i]       += cur_stats->num_macroblocks[i];
      gl_stats->bit_use_mb_type [i]      += cur_stats->bit_use_mb_type[i];
      gl_stats->bit_use_header  [i]      += cur_stats->bit_use_header[i];
      gl_stats->tmp_bit_use_cbp [i]      += cur_stats->tmp_bit_use_cbp[i];
      gl_stats->bit_use_coeffC  [i]      += cur_stats->bit_use_coeffC[i];
      gl_stats->bit_use_coeff[0][i]      += cur_stats->bit_use_coeff[0][i];
      gl_stats->bit_use_coeff[1][i]      += cur_stats->bit_use_coeff[1][i]; 
      gl_stats->bit_use_coeff[2][i]      += cur_stats->bit_use_coeff[2][i]; 
      gl_stats->bit_use_delta_quant[i]   += cur_stats->bit_use_delta_quant[i];
      gl_stats->bit_use_stuffing_bits[i] += cur_stats->bit_use_stuffing_bits[i];

      for (k = 0; k < 2; k++)
        gl_stats->b8_mode_0_use[i][k] += cur_stats->b8_mode_0_use[i][k];

      for (j = 0; j < 15; j++)
      {
        gl_stats->mode_use[i][j]     += cur_stats->mode_use[i][j];
        gl_stats->bit_use_mode[i][j] += cur_stats->bit_use_mode[i][j];
        for (k = 0; k < 2; k++)
          gl_stats->mode_use_transform[i][j][k] += cur_stats->mode_use_transform[i][j][k];
      }
    }
  }
}

static void storeRedundantFrame(VideoParameters *p_Vid)
{
  int j, k;
  if(p_Vid->key_frame)
  {
    for(j=0; j<p_Vid->height; j++)
    {
      memcpy(p_Vid->imgY_tmp[j], p_Vid->enc_frame_picture[0]->imgY[j], p_Vid->width * sizeof(imgpel));
    }

    for (k = 0; k < 2; k++)
    {
      for(j=0; j<p_Vid->height_cr; j++)
      {
        memcpy(p_Vid->imgUV_tmp[k][j], p_Vid->enc_frame_picture[0]->imgUV[k][j], p_Vid->width_cr * sizeof(imgpel));
      }
    }
  }

  if(p_Vid->redundant_coding)
  {
    for(j=0; j<p_Vid->height; j++)
    {
      memcpy(p_Vid->enc_frame_picture[0]->imgY[j], p_Vid->imgY_tmp[j], p_Vid->width * sizeof(imgpel));
    }
    for (k = 0; k < 2; k++)
    {
      for(j=0; j<p_Vid->height_cr; j++)
      {
        memcpy(p_Vid->enc_frame_picture[0]->imgUV[k][j], p_Vid->imgUV_tmp[k][j], p_Vid->width_cr * sizeof(imgpel));
      }
    }
  }
}
/*!
 ************************************************************************
 * \brief
 *    Free storable pictures
 ************************************************************************
 */
void free_pictures(VideoParameters *p_Vid, int dummy)
{
  int i;
  for (i = 0; i < 6; i++)
  {
    if(p_Vid->enc_picture != p_Vid->enc_frame_picture[i])
      free_storable_picture(p_Vid, p_Vid->enc_frame_picture[i]);
  }
}

#if (MVC_EXTENSION_ENABLE)
/*!
 ************************************************************************
 * \brief
 *    Write EL NALUs for field coded pictures
 ************************************************************************
 */
void write_el_field_vcl_nalu(VideoParameters *p_Vid)
{
  int temp1_nal_reference_idc;
  int temp1_non_idr_flag[2];
  int temp1_priority_id;
  int temp1_view_id;
  int temp1_temporal_id;
  int temp1_anchor_pic_flag[2];
  int temp1_inter_view_flag[2];

  if(p_Vid->view_id == 1)
  {
    int temp_view_id = p_Vid->view_id;

    p_Vid->view_id = 0;
    p_Vid->structure = TOP_FIELD;
    write_non_vcl_nalu_mvc(p_Vid);

    //backup view_id 1
    temp1_nal_reference_idc   = p_Vid->nal_reference_idc;
    temp1_non_idr_flag[0]     = p_Vid->non_idr_flag[0];
    temp1_non_idr_flag[1]     = p_Vid->non_idr_flag[1];
    temp1_priority_id         = p_Vid->priority_id;
    temp1_view_id             = temp_view_id;
    temp1_temporal_id         = p_Vid->temporal_id;
    temp1_anchor_pic_flag[0]  = p_Vid->anchor_pic_flag[0];
    temp1_anchor_pic_flag[1]  = p_Vid->anchor_pic_flag[1];
    temp1_inter_view_flag[0]  = p_Vid->inter_view_flag[0];
    temp1_inter_view_flag[1]  = p_Vid->inter_view_flag[1];

    //restore view_id 0
    p_Vid->nal_reference_idc  = p_Vid->temp0_nal_reference_idc;
    p_Vid->non_idr_flag[0]    = p_Vid->temp0_non_idr_flag[0];
    p_Vid->non_idr_flag[1]    = p_Vid->temp0_non_idr_flag[1];
    p_Vid->priority_id        = p_Vid->temp0_priority_id;
    p_Vid->view_id            = p_Vid->temp0_view_id;
    p_Vid->temporal_id        = p_Vid->temp0_temporal_id;
    p_Vid->anchor_pic_flag[0] = p_Vid->temp0_anchor_pic_flag[0];
    p_Vid->anchor_pic_flag[1] = p_Vid->temp0_anchor_pic_flag[1];
    p_Vid->inter_view_flag[0] = p_Vid->temp0_inter_view_flag[0];
    p_Vid->inter_view_flag[1] = p_Vid->temp0_inter_view_flag[1];
    writeout_picture (p_Vid, p_Vid->field_pic1[0], 0);  //top

    //restore view_id 1
    p_Vid->nal_reference_idc  = temp1_nal_reference_idc;
    p_Vid->non_idr_flag[0]    = temp1_non_idr_flag[0];
    p_Vid->non_idr_flag[1]    = temp1_non_idr_flag[1];
    p_Vid->priority_id        = temp1_priority_id;
    p_Vid->view_id            = temp1_view_id;
    p_Vid->temporal_id        = temp1_temporal_id;
    p_Vid->anchor_pic_flag[0] = temp1_anchor_pic_flag[0];
    p_Vid->anchor_pic_flag[1] = temp1_anchor_pic_flag[1];
    p_Vid->inter_view_flag[0] = temp1_inter_view_flag[0];
    p_Vid->inter_view_flag[1] = temp1_inter_view_flag[1];
    write_non_vcl_nalu_mvc( p_Vid );
    writeout_picture (p_Vid, p_Vid->field_pic2[0], 0);  //top

    //restore view_id 0
    p_Vid->nal_reference_idc  = p_Vid->temp0_nal_reference_idc;
    p_Vid->non_idr_flag[0]    = p_Vid->temp0_non_idr_flag[0];
    p_Vid->non_idr_flag[1]    = p_Vid->temp0_non_idr_flag[1];
    p_Vid->priority_id        = p_Vid->temp0_priority_id;
    p_Vid->view_id            = p_Vid->temp0_view_id;
    p_Vid->temporal_id        = p_Vid->temp0_temporal_id;
    p_Vid->anchor_pic_flag[0] = p_Vid->temp0_anchor_pic_flag[0];
    p_Vid->anchor_pic_flag[1] = p_Vid->temp0_anchor_pic_flag[1];
    p_Vid->inter_view_flag[0] = p_Vid->temp0_inter_view_flag[0];
    p_Vid->inter_view_flag[1] = p_Vid->temp0_inter_view_flag[1];
    p_Vid->structure = BOTTOM_FIELD;
    write_non_vcl_nalu_mvc( p_Vid );
    writeout_picture (p_Vid, p_Vid->field_pic1[1], 1);  //bot

    //restore view_id 1
    p_Vid->nal_reference_idc  = temp1_nal_reference_idc;
    p_Vid->non_idr_flag[0]    = temp1_non_idr_flag[0];
    p_Vid->non_idr_flag[1]    = temp1_non_idr_flag[1];
    p_Vid->priority_id        = temp1_priority_id;
    p_Vid->view_id            = temp1_view_id;
    p_Vid->temporal_id        = temp1_temporal_id;
    p_Vid->anchor_pic_flag[0] = temp1_anchor_pic_flag[0];
    p_Vid->anchor_pic_flag[1] = temp1_anchor_pic_flag[1];
    p_Vid->inter_view_flag[0] = temp1_inter_view_flag[0];
    p_Vid->inter_view_flag[1] = temp1_inter_view_flag[1];
    write_non_vcl_nalu_mvc( p_Vid );
    writeout_picture (p_Vid, p_Vid->field_pic2[1], 1);  //bot
  }
  else  // store the first view nal unit
  {
    p_Vid->temp0_nal_reference_idc   = p_Vid->nal_reference_idc;
    p_Vid->temp0_non_idr_flag[0]     = p_Vid->non_idr_flag[0];
    p_Vid->temp0_non_idr_flag[1]     = p_Vid->non_idr_flag[1];
    p_Vid->temp0_priority_id         = p_Vid->priority_id;
    p_Vid->temp0_view_id             = p_Vid->view_id;
    p_Vid->temp0_temporal_id         = p_Vid->temporal_id;
    p_Vid->temp0_anchor_pic_flag[0]  = p_Vid->anchor_pic_flag[0];
    p_Vid->temp0_anchor_pic_flag[1]  = p_Vid->anchor_pic_flag[1];
    p_Vid->temp0_inter_view_flag[0]  = p_Vid->inter_view_flag[0];
    p_Vid->temp0_inter_view_flag[1]  = p_Vid->inter_view_flag[1];
  }
}

/*!
 ************************************************************************
 * \brief
 *    Write coded frame (one frame picture or two field pictures)
 ************************************************************************
 */
void write_frame_picture(VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;

  if (p_Vid->fld_flag)            // field mode (use field when fld_flag=1 only)
  {
    field_mode_buffer (p_Vid);

    if(p_Inp->num_of_views==1)
    {
      p_Vid->structure = TOP_FIELD;
      write_non_vcl_nalu(p_Vid);
      writeout_picture  (p_Vid, p_Vid->field_pic1[0], 0);
      p_Vid->structure = BOTTOM_FIELD;
      write_non_vcl_nalu_bot_fld(p_Vid);
      writeout_picture  (p_Vid, p_Vid->field_pic1[1], 1);
    }
    else
    {
      write_el_field_vcl_nalu(p_Vid);
    }
  }
  else                          //frame mode
  {
    frame_mode_buffer (p_Vid);
    p_Vid->structure = FRAME;

#if (MVC_EXTENSION_ENABLE)
    if ( p_Inp->num_of_views == 2 )
    {
      write_non_vcl_nalu_mvc(p_Vid);
    }
    else
#endif
    write_non_vcl_nalu(p_Vid);
    if (p_Vid->type==SI_SLICE)
    { 
      writeout_picture  (p_Vid, p_Vid->frame_pic_si, 0);
    }
    else
    {
      writeout_picture  (p_Vid, p_Vid->p_frame_pic, 0);
    }
  }
}
#else
void write_frame_picture(VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  if (p_Vid->fld_flag)            // field mode (use field when fld_flag=1 only)
  {
    p_Vid->structure = TOP_FIELD;
    field_mode_buffer (p_Vid);
    write_non_vcl_nalu(p_Vid);
    writeout_picture  (p_Vid, p_Vid->field_pic[0]);
    p_Vid->structure = BOTTOM_FIELD;
    write_non_vcl_nalu_bot_fld(p_Vid);
    writeout_picture  (p_Vid, p_Vid->field_pic[1]);
  }
  else                          //frame mode
  {
    frame_mode_buffer (p_Vid);

    write_non_vcl_nalu(p_Vid);
    if (p_Vid->type==SI_SLICE)
    { 
      writeout_picture  (p_Vid, p_Vid->frame_pic_si);
    }
    else
    {
      writeout_picture  (p_Vid, p_Vid->p_frame_pic);
    }
  }
}
#endif

/*!
 ************************************************************************
 * \brief
 *    Read input data while considering and performing a 3:2 pulldown
 *    process.
 ************************************************************************
 */
static int read_input_data_32pulldown(VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int file_read = 0;
  int frm_no_in_file_first = 0;
  int frm_no_in_file_second = 0;
  int pull_down_offset = p_Inp->enable_32_pulldown == 1 ? 1 : 2;

#if (MVC_EXTENSION_ENABLE)
    if(p_Inp->num_of_views==2 && p_Vid->view_id == 1)
    {
      frm_no_in_file_first = ((p_Vid->frm_no_in_file * 4 + pull_down_offset)/ 5);
      frm_no_in_file_second = ((p_Vid->frm_no_in_file * 4 + 3)/ 5);

      // Read frame data (first frame)
      file_read = read_one_frame (p_Vid, &p_Inp->input_file2, frm_no_in_file_first, p_Inp->infile_header, &p_Inp->source, &p_Inp->output, p_Vid->imgData0.frm_data);
      if ( !file_read )
      {
        // end of file or stream found: trigger error handling
        get_number_of_frames (p_Inp, &p_Inp->input_file2);
        fprintf(stdout, "\nIncorrect FramesToBeEncoded: actual number is %6d frames!\n", p_Inp->no_frames );
        return 0;
      }
      pad_borders (p_Inp->output, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr, p_Vid->imgData0.frm_data);

      // Read frame data (second frame)
      file_read = read_one_frame (p_Vid, &p_Inp->input_file2, frm_no_in_file_second, p_Inp->infile_header, &p_Inp->source, &p_Inp->output, p_Vid->imgData4.frm_data);
      if ( !file_read )
      {
        // end of file or stream found: trigger error handling
        get_number_of_frames (p_Inp, &p_Inp->input_file2);
        fprintf(stdout, "\nIncorrect FramesToBeEncoded: actual number is %6d frames!\n", p_Inp->no_frames );
        return 0;
      }
      pad_borders (p_Inp->output, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr, p_Vid->imgData4.frm_data);
    }
    else
#endif
    {
      frm_no_in_file_first = ((p_Vid->frm_no_in_file * 4 + pull_down_offset)/ 5);
      frm_no_in_file_second = ((p_Vid->frm_no_in_file * 4 + 3)/ 5);

      // Read frame data (first frame)
      file_read = read_one_frame (p_Vid, &p_Inp->input_file1, frm_no_in_file_first, p_Inp->infile_header, &p_Inp->source, &p_Inp->output, p_Vid->imgData0.frm_data);
      if ( !file_read )
      {
        // end of file or stream found: trigger error handling
        get_number_of_frames (p_Inp, &p_Inp->input_file1);
        fprintf(stdout, "\nIncorrect FramesToBeEncoded: actual number is %6d frames!\n", p_Inp->no_frames );
        return 0;
      }
      pad_borders (p_Inp->output, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr, p_Vid->imgData0.frm_data);

      // Read frame data (second frame)
      file_read = read_one_frame (p_Vid, &p_Inp->input_file1, frm_no_in_file_second, p_Inp->infile_header, &p_Inp->source, &p_Inp->output, p_Vid->imgData4.frm_data);
      if ( !file_read )
      {
        // end of file or stream found: trigger error handling
        get_number_of_frames (p_Inp, &p_Inp->input_file1);
        fprintf(stdout, "\nIncorrect FramesToBeEncoded: actual number is %6d frames!\n", p_Inp->no_frames );
        return 0;
      }
      pad_borders (p_Inp->output, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr, p_Vid->imgData4.frm_data);
    }


    return 1;
}

static int read_input_data(VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int file_read = 0;

#if (MVC_EXTENSION_ENABLE)
    if(p_Inp->num_of_views==2 && p_Vid->view_id == 1)
    {
      file_read = read_one_frame (p_Vid, &p_Inp->input_file2, p_Vid->frm_no_in_file, p_Inp->infile_header, &p_Inp->source, &p_Inp->output, p_Vid->imgData0.frm_data);
      if ( !file_read )
      {
        // end of file or stream found: trigger error handling
        get_number_of_frames (p_Inp, &p_Inp->input_file2);
        fprintf(stdout, "\nIncorrect FramesToBeEncoded: actual number is %6d frames!\n", p_Inp->no_frames );
        return 0;
      }
      pad_borders (p_Inp->output, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr, p_Vid->imgData0.frm_data);
    }
    else
#endif
    {
      file_read = read_one_frame (p_Vid, &p_Inp->input_file1, p_Vid->frm_no_in_file, p_Inp->infile_header, &p_Inp->source, &p_Inp->output, p_Vid->imgData0.frm_data);
      if ( !file_read )
      {
        // end of file or stream found: trigger error handling
        get_number_of_frames (p_Inp, &p_Inp->input_file1);
        fprintf(stdout, "\nIncorrect FramesToBeEncoded: actual number is %6d frames!\n", p_Inp->no_frames );
        return 0;
      }
      pad_borders (p_Inp->output, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr, p_Vid->imgData0.frm_data);
    }  

  return 1;
}

void perform_encode_field(VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;

  //Rate control
  if ( p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE )
    p_Vid->p_rc_gen->FieldControl = 1;

  p_Vid->field_picture = 1;  // we encode fields

#if (MVC_EXTENSION_ENABLE)
  field_picture (p_Vid, p_Vid->field_pic_ptr[0], p_Vid->field_pic_ptr[1]);
#else
  field_picture (p_Vid, p_Vid->field_pic[0], p_Vid->field_pic[1]);
#endif
  p_Vid->fld_flag = TRUE;
}

void perform_encode_frame(VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int tmpFrameQP = 0;
  int num_ref_idx_l0 = 0;
  int num_ref_idx_l1 = 0;

  int frame_type;

  //Rate control
  if ( p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE )
    p_Vid->p_rc_gen->FieldControl = 0;

  p_Vid->field_picture = 0; // we encode a frame

  //Rate control
  if(p_Inp->RCEnable)
    rc_init_frame(p_Vid, p_Inp);
  // Ensure that p_Vid->p_curr_frm_struct->qp is properly set
  p_Vid->p_curr_frm_struct->qp = p_Vid->qp;

  p_Vid->active_pps = p_Vid->PicParSet[0];

  // HME. Currently not supported for B slices
  if(p_Inp->HMEEnable)
  {
    if (p_Vid->type!=I_SLICE && p_Vid->type!=SI_SLICE)
      invoke_HME(p_Vid, 0);
  }
    
#if (MVC_EXTENSION_ENABLE)
  if(p_Vid->view_id!=1 || !p_Vid->sec_view_force_fld)
  {
#endif
    if ((p_Vid->type == B_SLICE || p_Vid->type == P_SLICE || p_Vid->type==I_SLICE) && p_Inp->RDPictureDecision)
    {
      frame_picture_mp (p_Vid, p_Inp); 
    }
    else
    {
      frame_picture (p_Vid, p_Vid->frame_pic[0], &p_Vid->imgData, 0);
      p_Vid->p_frame_pic = p_Vid->frame_pic[0]; 
    }

    tmpFrameQP = p_Vid->SumFrameQP; // call it here since rd_picture_coding buffers it and may modify it
    num_ref_idx_l0 = p_Vid->num_ref_idx_l0_active;
    num_ref_idx_l1 = p_Vid->num_ref_idx_l1_active;

    if (p_Vid->p_curr_frm_struct->type == SI_SLICE)
    {
      // once the picture has been encoded as a primary SP frame encode as an SI frame
      set_slice_type( p_Vid, p_Inp, SI_SLICE );
      frame_picture (p_Vid, p_Vid->frame_pic_si, &p_Vid->imgData, 0);
    }

    if ((p_Vid->type == SP_SLICE) && (p_Inp->sp_output_indicator))
    {
      // output the transformed and quantized coefficients (useful for switching SP frames)
      output_SP_coefficients(p_Vid, p_Inp);
    }
#if (MVC_EXTENSION_ENABLE)
  }
#endif

#if (MVC_EXTENSION_ENABLE)
  if ((p_Inp->PicInterlace == ADAPTIVE_CODING) && (!is_MVC_profile(p_Vid->active_sps->profile_idc) || 
     (/*(p_Vid->active_sps->profile_idc == SBSPLUS_HIGH) &&*/ ((p_Vid->view_id == 0) || (p_Vid->view_id == 1 && p_Vid->sec_view_force_fld)))))
#else
  if (p_Inp->PicInterlace == ADAPTIVE_CODING)
#endif
  {
    DecRefPicMarking_t *tmp_drpm;

    if ( p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE )
      p_Vid->p_rc_gen->FieldControl=1;
    p_Vid->write_macroblock = FALSE;
    p_Vid->bot_MB = FALSE;

    frame_type = p_Vid->type;

    p_Vid->field_picture = 1;  // we encode fields
   
    //Free frame based dec_ref_pic_marking_buffer
    while (p_Vid->dec_ref_pic_marking_buffer)
    {
      tmp_drpm = p_Vid->dec_ref_pic_marking_buffer;
      p_Vid->dec_ref_pic_marking_buffer = tmp_drpm->Next;
      free(tmp_drpm);
    }

#if (MVC_EXTENSION_ENABLE)
    field_picture (p_Vid, p_Vid->field_pic_ptr[0], p_Vid->field_pic_ptr[1]);

    if(p_Inp->num_of_views==1 || p_Vid->view_id==0)  // first view will make the decision
    {
      if(p_Vid->rd_pass == 0)
        p_Vid->fld_flag = picture_structure_decision (p_Vid, p_Vid->frame_pic[0], p_Vid->field_pic_ptr[0], p_Vid->field_pic_ptr[1]);
      else if(p_Vid->rd_pass == 1)
        p_Vid->fld_flag = picture_structure_decision (p_Vid, p_Vid->frame_pic[1], p_Vid->field_pic_ptr[0], p_Vid->field_pic_ptr[1]);
      else
        p_Vid->fld_flag = picture_structure_decision (p_Vid, p_Vid->frame_pic[2], p_Vid->field_pic_ptr[0], p_Vid->field_pic_ptr[1]);

      p_Vid->sec_view_force_fld = p_Vid->fld_flag;    // store 1st view decision, same coding structure to be used for second view
    }
    else
    {
      p_Vid->fld_flag = (byte) p_Vid->sec_view_force_fld;
    }
#else
    field_picture (p_Vid, p_Vid->field_pic[0], p_Vid->field_pic[1]);

    if(p_Vid->rd_pass == 0)
      p_Vid->fld_flag = picture_structure_decision (p_Vid, p_Vid->frame_pic[0], p_Vid->field_pic[0], p_Vid->field_pic[1]);
    else if(p_Vid->rd_pass == 1)
      p_Vid->fld_flag = picture_structure_decision (p_Vid, p_Vid->frame_pic[1], p_Vid->field_pic[0], p_Vid->field_pic[1]);
    else
      p_Vid->fld_flag = picture_structure_decision (p_Vid, p_Vid->frame_pic[2], p_Vid->field_pic[0], p_Vid->field_pic[1]);
#endif

    if ( p_Vid->fld_flag )
    {
      tmpFrameQP = p_Vid->SumFrameQP;
      num_ref_idx_l0 = p_Vid->num_ref_idx_l0_active;
      num_ref_idx_l1 = p_Vid->num_ref_idx_l1_active;
    }
    
    if ( p_Vid->fld_flag==0 )
    {
      p_Vid->type = (short) frame_type;
    }

    update_field_frame_contexts (p_Vid, p_Vid->fld_flag);

    //Rate control
    if ( p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE )
      p_Vid->p_rc_gen->FieldFrame = !(p_Vid->fld_flag) ? 1 : 0;
  }
  else
    p_Vid->fld_flag = FALSE;

  p_Vid->SumFrameQP = tmpFrameQP;
  p_Vid->num_ref_idx_l0_active = num_ref_idx_l0;
  p_Vid->num_ref_idx_l1_active = num_ref_idx_l1;
}

void free_slice_data(VideoParameters *p_Vid)
{
  int i;

  if (p_Vid->frame_pic_si)
  {
    free_slice_list(p_Vid->frame_pic_si);
  }

  for (i = 0; i < p_Vid->frm_iter; i++)
  {
    if (p_Vid->frame_pic[i])
    {
      free_slice_list(p_Vid->frame_pic[i]);
    }
  }

#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->field_pic_ptr && (p_Vid->p_Inp->num_of_views==1 || p_Vid->view_id==1) )
  {
    for (i = 0; i < 2; i++)
    {
      if (p_Vid->field_pic1[i])
        free_slice_list(p_Vid->field_pic1[i]);
      if (p_Vid->p_Inp->num_of_views==2 && p_Vid->field_pic2)
      {
        if (p_Vid->field_pic2[i])
          free_slice_list(p_Vid->field_pic2[i]);
      }
    }
  }
#else
  if (p_Vid->field_pic)
  {
    for (i = 0; i < 2; i++)
    {
      if (p_Vid->field_pic[i])
        free_slice_list(p_Vid->field_pic[i]);
    }
  }
#endif
}

void store_coded_picture(DecodedPictureBuffer *p_Dpb)
{
  VideoParameters *p_Vid = p_Dpb->p_Vid;
  InputParameters *p_Inp = p_Vid->p_Inp;
  unsigned int profile_idc = p_Vid->active_sps->profile_idc;

#if (MVC_EXTENSION_ENABLE)
  if ( (p_Inp->PicInterlace == ADAPTIVE_CODING) && ((p_Dpb->layer_id == 0 ) || !is_MVC_profile(profile_idc)))
#else
  if ( p_Inp->PicInterlace == ADAPTIVE_CODING )
#endif
  {
    if (p_Vid->fld_flag)
    {      
      update_global_stats(p_Inp, p_Vid->p_Stats, &p_Vid->enc_field_picture[0]->stats);
      update_global_stats(p_Inp, p_Vid->p_Stats, &p_Vid->enc_field_picture[1]->stats);
      // store bottom field
      store_picture_in_dpb (p_Dpb, p_Vid->enc_field_picture[1], &p_Inp->output);
#if (MVC_EXTENSION_ENABLE)
      if ( p_Vid->proc_picture && is_MVC_profile(profile_idc) )
      {
        store_proc_picture_in_dpb (p_Vid->p_Dpb_layer[1], p_Vid->proc_picture, &p_Inp->output);
        p_Vid->proc_picture = NULL;
      }
#endif
      free_storable_picture(p_Vid, p_Vid->enc_frame_picture[0]);
      free_storable_picture(p_Vid, p_Vid->enc_frame_picture[1]);
      free_storable_picture(p_Vid, p_Vid->enc_frame_picture[2]);
    }
    else
    {
      update_global_stats(p_Inp, p_Vid->p_Stats, &p_Vid->enc_frame_picture[p_Vid->rd_pass]->stats);
      // replace top with frame
      if (p_Vid->rd_pass==2)
      {
        replace_top_pic_with_frame(p_Dpb, p_Vid->enc_frame_picture[2], &p_Inp->output);
        free_storable_picture     (p_Vid, p_Vid->enc_frame_picture[0]);
        free_storable_picture     (p_Vid, p_Vid->enc_frame_picture[1]);
      }
      else if (p_Vid->rd_pass==1)
      {
        replace_top_pic_with_frame(p_Dpb, p_Vid->enc_frame_picture[1], &p_Inp->output);
        free_storable_picture     (p_Vid, p_Vid->enc_frame_picture[0]);
        free_storable_picture     (p_Vid, p_Vid->enc_frame_picture[2]);
      }
      else
      {
        if(p_Inp->redundant_pic_flag==0 || (p_Vid->key_frame==0))
        {
          replace_top_pic_with_frame(p_Dpb, p_Vid->enc_frame_picture[0], &p_Inp->output);
          free_storable_picture     (p_Vid, p_Vid->enc_frame_picture[1]);
          free_storable_picture     (p_Vid, p_Vid->enc_frame_picture[2]);
        }
      }
      free_storable_picture(p_Vid, p_Vid->enc_field_picture[1]);
    }
  }
  else
  {
    if (p_Vid->fld_flag)
    {
      update_global_stats(p_Inp, p_Vid->p_Stats, &p_Vid->enc_field_picture[0]->stats);
      update_global_stats(p_Inp, p_Vid->p_Stats, &p_Vid->enc_field_picture[1]->stats);
      store_picture_in_dpb(p_Dpb, p_Vid->enc_field_picture[1], &p_Inp->output);
#if (MVC_EXTENSION_ENABLE)
      if ( is_MVC_profile(profile_idc) && p_Vid->proc_picture)
      {
        store_proc_picture_in_dpb (p_Vid->p_Dpb_layer[1], p_Vid->proc_picture, &p_Inp->output);
        p_Vid->proc_picture = NULL;
      }
#endif
    }
    else
    {      
      if ((p_Inp->redundant_pic_flag != 1) || (p_Vid->key_frame == 0))
      {
        update_global_stats(p_Inp, p_Vid->p_Stats, &p_Vid->enc_picture->stats);
        store_picture_in_dpb (p_Dpb, p_Vid->enc_picture, &p_Inp->output);

#if (MVC_EXTENSION_ENABLE)
        if ( is_MVC_profile(profile_idc) && p_Vid->proc_picture)
        {
          store_proc_picture_in_dpb (p_Vid->p_Dpb_layer[1], p_Vid->proc_picture, &p_Inp->output);
          p_Vid->proc_picture = NULL;
        }
#endif
        free_pictures(p_Vid, -1);
      }
    }
  }
}

void update_bitcounter_stats(VideoParameters *p_Vid)
{
  p_Vid->p_Stats->bit_ctr_n = p_Vid->p_Stats->bit_ctr;
  p_Vid->p_Stats->bit_ctr_parametersets_n = 0;
  p_Vid->p_Stats->bit_ctr_filler_data_n = p_Vid->p_Stats->bit_ctr_filler_data;

#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->p_Inp->num_of_views == 2 )
  {
    if ( p_Vid->fld_flag )
    {
      if ( p_Vid->view_id == 1 )
      {
        p_Vid->p_Stats->bit_ctr_parametersets_v[0] += p_Vid->p_Stats->bit_ctr_parametersets_n_v[0];
        p_Vid->p_Stats->bit_ctr_parametersets_v[1] += p_Vid->p_Stats->bit_ctr_parametersets_n_v[1];
        p_Vid->p_Stats->bit_ctr_n_v[0] = p_Vid->p_Stats->bit_ctr_v[0];
        p_Vid->p_Stats->bit_ctr_n_v[1] = p_Vid->p_Stats->bit_ctr_v[1];
        p_Vid->p_Stats->bit_ctr_parametersets_n_v[0] = 0;
        p_Vid->p_Stats->bit_ctr_parametersets_n_v[1] = 0;
        p_Vid->p_Stats->bit_ctr_filler_data_n_v[0] = p_Vid->p_Stats->bit_ctr_filler_data_v[0];
        p_Vid->p_Stats->bit_ctr_filler_data_n_v[1] = p_Vid->p_Stats->bit_ctr_filler_data_v[1];
      }
    }
    else // this seems to be enough and the check above seems redundant...check later...
    {
      p_Vid->p_Stats->bit_ctr_parametersets_v[p_Vid->view_id] += p_Vid->p_Stats->bit_ctr_parametersets_n_v[p_Vid->view_id];
      p_Vid->p_Stats->bit_ctr_n_v[p_Vid->view_id] = p_Vid->p_Stats->bit_ctr_v[p_Vid->view_id];
      p_Vid->p_Stats->bit_ctr_parametersets_n_v[p_Vid->view_id] = 0;
      p_Vid->p_Stats->bit_ctr_filler_data_n_v[p_Vid->view_id] = p_Vid->p_Stats->bit_ctr_filler_data_v[p_Vid->view_id];
    }
  }
#endif
}

void update_idr_order_stats(VideoParameters *p_Vid)
{
  if ( p_Vid->fld_flag ) // need to check both fields here
  {
    // top field picture
    if ( p_Vid->p_curr_frm_struct->p_top_fld_pic->p_Slice[0].type == I_SLICE && p_Vid->p_curr_frm_struct->p_top_fld_pic->nal_ref_idc)
    {
      p_Vid->lastINTRA       = imax(p_Vid->lastINTRA, p_Vid->frame_no);
      p_Vid->lastIntraNumber = p_Vid->curr_frm_idx;
      if ( p_Vid->p_curr_frm_struct->p_top_fld_pic->idr_flag )
      {
        p_Vid->last_idr_disp_order = imax( p_Vid->frame_no, p_Vid->last_idr_disp_order );
        p_Vid->last_idr_code_order = p_Vid->curr_frm_idx;
      }
    }
    // bottom field picture
    if ( p_Vid->p_curr_frm_struct->p_bot_fld_pic->p_Slice[0].type == I_SLICE && p_Vid->p_curr_frm_struct->p_bot_fld_pic->nal_ref_idc)
    {
      p_Vid->lastINTRA       = imax(p_Vid->lastINTRA, p_Vid->frame_no);
      p_Vid->lastIntraNumber = p_Vid->curr_frm_idx;
      if ( p_Vid->p_curr_frm_struct->p_bot_fld_pic->idr_flag )
      {
        p_Vid->last_idr_disp_order = imax( p_Vid->frame_no, p_Vid->last_idr_disp_order );
        p_Vid->last_idr_code_order = p_Vid->curr_frm_idx;
      }
    }
  }
  else
  {
    // frame picture
    if ( p_Vid->p_curr_frm_struct->p_frame_pic->p_Slice[0].type == I_SLICE && p_Vid->p_curr_frm_struct->p_frame_pic->nal_ref_idc)
    {
      p_Vid->lastINTRA       = imax(p_Vid->lastINTRA, p_Vid->frame_no);
      p_Vid->lastIntraNumber = p_Vid->curr_frm_idx;
      if ( p_Vid->p_curr_frm_struct->p_frame_pic->idr_flag )
      {
        p_Vid->last_idr_disp_order = imax( p_Vid->frame_no, p_Vid->last_idr_disp_order );
        p_Vid->last_idr_code_order = p_Vid->curr_frm_idx;
      }
    }
  }
}

int update_video_stats(VideoParameters *p_Vid)
{
  int bits = 0;
  InputParameters *p_Inp = p_Vid->p_Inp;

  //Rate control
  if(p_Inp->RCEnable)
  {
    if ((!p_Inp->PicInterlace) && (!p_Inp->MbInterlace))
    {
      bits = (int) (p_Vid->p_Stats->bit_ctr - p_Vid->p_Stats->bit_ctr_n)
        + (int)( p_Vid->p_Stats->bit_ctr_filler_data - p_Vid->p_Stats->bit_ctr_filler_data_n );
    }
    else if ( p_Inp->RCUpdateMode <= MAX_RC_MODE )
    {
      bits = (int)(p_Vid->p_Stats->bit_ctr - (p_Vid->p_rc_quad->Pprev_bits))
        + (int)( p_Vid->p_Stats->bit_ctr_filler_data - p_Vid->p_Stats->bit_ctr_filler_data_n ); // used for rate control update
      p_Vid->p_rc_quad->Pprev_bits = p_Vid->p_Stats->bit_ctr + p_Vid->p_Stats->bit_ctr_filler_data;
    }
    else
    {
      bits = (int) (p_Vid->p_Stats->bit_ctr - p_Vid->p_Stats->bit_ctr_n)
        + (int)( p_Vid->p_Stats->bit_ctr_filler_data - p_Vid->p_Stats->bit_ctr_filler_data_n );
    }
  }

  p_Vid->p_Stats->bit_counter[p_Vid->type] += p_Vid->p_Stats->bit_ctr - p_Vid->p_Stats->bit_ctr_n;

#if (MVC_EXTENSION_ENABLE)
  if ( p_Inp->num_of_views == 2 )
  {
    if ( p_Vid->fld_flag )
    {
      if ( p_Vid->view_id == 1 )
      {
        int prev_type = (p_Vid->p_pred->p_frm_mvc + ( (p_Vid->curr_frm_idx << 1) % p_Vid->p_pred->num_frames_mvc ))->type;

        p_Vid->p_Stats->bit_counter_v[0][prev_type]   += p_Vid->p_Stats->bit_ctr_v[0] - p_Vid->p_Stats->bit_ctr_n_v[0];
        p_Vid->p_Stats->bit_counter_v[1][p_Vid->type] += p_Vid->p_Stats->bit_ctr_v[1] - p_Vid->p_Stats->bit_ctr_n_v[1];
      }
    }
    else
    {
      p_Vid->p_Stats->bit_counter_v[p_Vid->view_id][p_Vid->type] += p_Vid->p_Stats->bit_ctr_v[p_Vid->view_id] - p_Vid->p_Stats->bit_ctr_n_v[p_Vid->view_id];
    }
  }
#endif

  return bits;
}

/*!
 ************************************************************************
 * \brief
 *    Encodes one frame
 ************************************************************************
 */
int encode_one_frame (VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int i;
  int nplane;

  //Rate control
  int bits = 0;

  TIME_T start_time;
  TIME_T end_time;
  int64  tmp_time;

  p_Vid->me_time = 0;
  p_Vid->rd_pass = 0;

  if( (p_Inp->separate_colour_plane_flag != 0) )
  {
    for( nplane=0; nplane<MAX_PLANE; nplane++ ){
      p_Vid->enc_frame_picture_JV[nplane] = NULL;
    }
  }

  for (i = 0; i < 6; i++)
    p_Vid->enc_frame_picture[i]  = NULL;

  gettime(&start_time);          // start time in ms

  //Rate control
  p_Vid->write_macroblock = FALSE;
  /*
  //Shankar Regunathan (Oct 2002)
  //Prepare Panscanrect SEI payload
  UpdatePanScanRectInfo (p_SEI);
  //Prepare Arbitrarydata SEI Payload
  UpdateUser_data_unregistered (p_SEI);
  //Prepare Registered data SEI Payload
  UpdateUser_data_registered_itu_t_t35 (p_SEI);
  //Prepare RandomAccess SEI Payload
  UpdateRandomAccess (p_Vid);
  */

  put_buffer_frame (p_Vid);    // sets the pointers to the frame structures
                               // (and not to one of the field structures)
  init_frame (p_Vid, p_Inp);

  if (p_Inp->enable_32_pulldown)
  {
    if ( !read_input_data_32pulldown (p_Vid) )
    {
      return 0;
    }
  }
  else
  {
    if ( !read_input_data (p_Vid) )
    {
      return 0;
    }
  }

  process_image(p_Vid, p_Inp);
  pad_borders (p_Inp->output, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr, p_Vid->imgData.frm_data);

#if (MVC_EXTENSION_ENABLE)
  if(p_Inp->num_of_views==1 || p_Vid->view_id==0)
  {
    p_Vid->field_pic_ptr = p_Vid->field_pic1;
  }
  else
  {
    p_Vid->field_pic_ptr = p_Vid->field_pic2;
  }
#endif


  // Following code should consider optimal coding mode. Currently also does not support
  // multiple slices per frame.
  p_Vid->p_Dist->frame_ctr++;
#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->num_of_views == 2)
  {
    p_Vid->p_Dist->frame_ctr_v[p_Vid->view_id]++;
  }
#endif

  if(p_Vid->type == SP_SLICE)
  {
    if(p_Inp->sp2_frame_indicator)
    { // switching SP frame encoding
      p_Vid->sp2_frame_indicator = TRUE;
      read_SP_coefficients(p_Vid, p_Inp);
    }
  }
  else
  {
    p_Vid->sp2_frame_indicator = FALSE;
  }

  if ( p_Inp->WPMCPrecision )
  {
    wpxInitWPXPasses(p_Vid, p_Inp);
    p_Vid->pWPX->curr_wp_rd_pass = p_Vid->pWPX->wp_rd_passes;
    p_Vid->pWPX->curr_wp_rd_pass->algorithm = WP_REGULAR;
  }

  if (p_Inp->PicInterlace == FIELD_CODING)
    perform_encode_field(p_Vid);
  else
    perform_encode_frame(p_Vid);

  p_Vid->p_Stats->frame_counter++;
  p_Vid->p_Stats->frame_ctr[p_Vid->type]++;

  // Here, p_Vid->structure may be either FRAME or BOTTOM FIELD depending on whether AFF coding is used
  // The picture structure decision changes really only the fld_flag
  write_frame_picture(p_Vid);

#if (MVC_EXTENSION_ENABLE)
  if(p_Inp->num_of_views==2)
  {
    // view_id 1 will follow view_id 0 anchor_pic_flag value
    if((p_Vid->anchor_pic_flag[0]==1) && (p_Vid->view_id&1)==0)
    {
      p_Vid->prev_view_is_anchor = 1;
    }
    else
    {
      p_Vid->prev_view_is_anchor = 0;
    }
  }
#endif

  //Need slice data for deblocking in UpdateDecoders
  if (p_Inp->PicInterlace == FRAME_CODING)
  {
    if ((p_Inp->rdopt == 3) && (p_Inp->de == LLN) && (p_Vid->nal_reference_idc != 0))
    {
      UpdateDecoders (p_Vid, p_Inp, p_Vid->enc_picture);      // simulate packet losses and move decoded image to reference buffers
    }

    if (p_Inp->RestrictRef)
      UpdatePixelMap (p_Vid, p_Inp);
  }

#if OUTPUT_REF_LIST
  {
    int i;
    printf( " [L0]: " );
    for ( i = 0; i<p_Vid->currentSlice->listXsize[0]; i++ )
    {
      printf( "%d ", p_Vid->currentSlice->listX[0][i]->poc / 2 ); // may not work with field
    }
    printf( " [L1]: " );
    for ( i = 0; i<p_Vid->currentSlice->listXsize[1]; i++ )
    {
      printf( "%d ", p_Vid->currentSlice->listX[1][i]->poc / 2 ); // may not work with field
    }
    printf("\n");
  }  
#endif

  free_slice_data(p_Vid);

  //Rate control
  if ( p_Inp->RCEnable )
  {
    // we could add here a function pointer!
    bits = (int)( p_Vid->p_Stats->bit_ctr - p_Vid->p_Stats->bit_ctr_n )
      + (int)( p_Vid->p_Stats->bit_ctr_filler_data - p_Vid->p_Stats->bit_ctr_filler_data_n );

    if ( p_Inp->RCUpdateMode <= MAX_RC_MODE )
      p_Vid->rc_update_pict_frame_ptr(p_Vid, p_Inp, p_Vid->p_rc_quad, p_Vid->p_rc_gen, bits);

  }

  compute_distortion(p_Vid, &p_Vid->imgData);

  // redundant pictures: save reconstruction to calculate SNR and replace reference picture
  if(p_Inp->redundant_pic_flag)
  {
    storeRedundantFrame(p_Vid);
  }

  //printf("view_id:%d, frame_num:%d, poc:%d\n", p_Vid->view_id, (int)p_Vid->enc_picture->frame_num, p_Vid->enc_picture->poc);
  store_coded_picture(p_Vid->p_Dpb_layer[p_Vid->view_id]);


  p_Vid->AverageFrameQP = isign(p_Vid->SumFrameQP) * ((iabs(p_Vid->SumFrameQP) + (int) (p_Vid->FrameSizeInMbs >> 1))/ (int) p_Vid->FrameSizeInMbs);  

  if ( p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE && p_Vid->type != B_SLICE && p_Inp->basicunit < p_Vid->FrameSizeInMbs )
    p_Vid->p_rc_quad->CurrLastQP = p_Vid->AverageFrameQP + p_Vid->p_rc_quad->bitdepth_qp_scale;

#ifdef _LEAKYBUCKET_
  // Store bits used for this frame and increment counter of no. of coded frames
  if (!p_Vid->redundant_coding)
  {
    p_Vid->Bit_Buffer[p_Vid->total_frame_buffer++] = (long) (p_Vid->p_Stats->bit_ctr - p_Vid->p_Stats->bit_ctr_n)
      + (long)( p_Vid->p_Stats->bit_ctr_filler_data - p_Vid->p_Stats->bit_ctr_filler_data_n );
  }
#endif

  // POC200301: Verify that POC coding type 2 is not used if more than one consecutive
  // non-reference frame is requested or if decoding order is different from output order
  if (p_Vid->pic_order_cnt_type == 2)
  {
    if (!p_Vid->nal_reference_idc)
      p_Vid->consecutive_non_reference_pictures++;
    else
      p_Vid->consecutive_non_reference_pictures = 0;

    if (p_Vid->frame_no < p_Vid->prev_frame_no || p_Vid->consecutive_non_reference_pictures>1)
      error("POC type 2 cannot be applied for the coding pattern where the encoding /decoding order of pictures are different from the output order.\n", -1);
    p_Vid->prev_frame_no = p_Vid->frame_no;
  }

  gettime(&end_time);    // end time in ms
  tmp_time  = timediff(&start_time, &end_time);
  p_Vid->tot_time += tmp_time;
  tmp_time  = timenorm(tmp_time);
  p_Vid->me_time   = timenorm(p_Vid->me_time);
  if (p_Vid->p_Stats->bit_ctr_parametersets_n!=0 && p_Inp->Verbose != 3)
    ReportNALNonVLCBits(p_Vid, tmp_time);

#if (MVC_EXTENSION_ENABLE)
  if (p_Vid->curr_frm_idx == 0 && !p_Vid->view_id)
#else
  if (p_Vid->curr_frm_idx == 0)
#endif
    ReportFirstframe(p_Vid, tmp_time);
  else
  {
    bits = update_video_stats(p_Vid);

    switch (p_Vid->type)
    {
    case I_SLICE:
    case SI_SLICE:
      ReportI(p_Vid, tmp_time);
      break;
    case B_SLICE:
      ReportB(p_Vid, tmp_time);
      break;
    case SP_SLICE:
      ReportP(p_Vid, tmp_time);
      break;
    case P_SLICE:
    default:
      ReportP(p_Vid, tmp_time);
    }
  }

  if (p_Inp->Verbose == 0)
  {
    //for (i = 0; i <= (p_Vid->number & 0x0F); i++)
    //printf(".");
    //printf("                              \r");
    printf("Completed Encoding Frame %05d.\r", p_Vid->frame_no);
  }
  // Flush output statistics
  fflush(stdout);

  //Rate control
  if(p_Inp->RCEnable)
    p_Vid->rc_update_picture_ptr( p_Vid, p_Inp, bits );

  // update bit counters
  update_bitcounter_stats(p_Vid);

  update_idr_order_stats(p_Vid);

  return 1;
}


/*!
 ************************************************************************
 * \brief
 *    This function write out a picture
 * \return
 *    0 if OK,                                                         \n
 *    1 in case of error
 *
 ************************************************************************
 */
#if (MVC_EXTENSION_ENABLE)
static void writeout_picture(VideoParameters *p_Vid, Picture *pic, int is_bottom)
{
  int partition, slice, bits;
  Slice  *currSlice;
  NALU_t *nalu = NULL;

  p_Vid->currentPicture = pic;

  // loop over all slices of the picture
  for (slice=0; slice < pic->no_slices; slice++)
  {
    currSlice = pic->slices[slice];

    // loop over the partitions
    for (partition=0; partition < currSlice->max_part_nr; partition++)
    {
      // write only if the partition has content
      if ( currSlice->partArr[partition].bitstream->write_flag )
      {
        if(p_Vid->p_Inp->num_of_views==2 && p_Vid->view_id==0)
        {
          nalu = AllocNALU(MAXNALUSIZE);
          nalu->startcodeprefix_len = 4;
          nalu->nal_unit_type       = NALU_TYPE_PREFIX;
          nalu->nal_reference_idc   = NALU_PRIORITY_HIGHEST;
          nalu->svc_extension_flag  = 0;
          nalu->non_idr_flag        = p_Vid->non_idr_flag[is_bottom];
          nalu->priority_id         = p_Vid->priority_id;
          nalu->view_id             = p_Vid->view_id;
          nalu->temporal_id         = p_Vid->temporal_id;
          nalu->anchor_pic_flag     = p_Vid->anchor_pic_flag[is_bottom];
          nalu->inter_view_flag     = p_Vid->inter_view_flag[is_bottom];
          nalu->reserved_one_bit    = 1;

          bits = p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
          p_Vid->p_Stats->bit_ctr += bits;
          p_Vid->p_Stats->bit_ctr_v[0] += bits;
          FreeNALU(nalu);
        }

        bits = p_Vid->WriteNALU (p_Vid, currSlice->partArr[partition].nal_unit, p_Vid->f_out);
        p_Vid->p_Stats->bit_ctr += bits;
        if ( p_Vid->p_Inp->num_of_views == 2 )
        {
          p_Vid->p_Stats->bit_ctr_v[p_Vid->view_id] += bits;
        }
      }
    }
  }
}
#else
static void writeout_picture(VideoParameters *p_Vid, Picture *pic)
{
  int partition, slice, bits;
  Slice  *currSlice;

  p_Vid->currentPicture = pic;

  // loop over all slices of the picture
  for (slice=0; slice < pic->no_slices; slice++)
  {
    currSlice = pic->slices[slice];

    // loop over the partitions
    for (partition=0; partition < currSlice->max_part_nr; partition++)
    {
      // write only if the partition has content
      if (currSlice->partArr[partition].bitstream->write_flag )
      {
        bits = p_Vid->WriteNALU (p_Vid, currSlice->partArr[partition].nal_unit, p_Vid->f_out);
        p_Vid->p_Stats->bit_ctr += bits;

      }
    }
  }
}
#endif

void copy_params(VideoParameters *p_Vid, StorablePicture *enc_picture, seq_parameter_set_rbsp_t *active_sps)
{
  enc_picture->frame_mbs_only_flag = active_sps->frame_mbs_only_flag;
  enc_picture->frame_cropping_flag = active_sps->frame_cropping_flag;
  enc_picture->chroma_format_idc   = active_sps->chroma_format_idc;
  enc_picture->chroma_mask_mv_x    = p_Vid->chroma_mask_mv_x;
  enc_picture->chroma_mask_mv_y    = p_Vid->chroma_mask_mv_y;
  enc_picture->chroma_shift_y      = p_Vid->chroma_shift_y;
  enc_picture->chroma_shift_x      = p_Vid->chroma_shift_x;
  enc_picture->otf_flag            = p_Vid->p_Inp->OnTheFlyFractMCP;

  if (active_sps->frame_cropping_flag)
  {
    enc_picture->frame_crop_left_offset   = active_sps->frame_crop_left_offset;
    enc_picture->frame_crop_right_offset  = active_sps->frame_crop_right_offset;
    enc_picture->frame_crop_top_offset    = active_sps->frame_crop_top_offset;
    enc_picture->frame_crop_bottom_offset = active_sps->frame_crop_bottom_offset;
  }
  else
  {
    enc_picture->frame_crop_left_offset   = 0;
    enc_picture->frame_crop_right_offset  = 0;
    enc_picture->frame_crop_top_offset    = 0;
    enc_picture->frame_crop_bottom_offset = 0;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Prepare and allocate an encoded frame picture structure
 ************************************************************************
 */
static void prepare_enc_frame_picture (VideoParameters *p_Vid, StorablePicture **stored_pic)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  (*stored_pic)              = alloc_storable_picture (p_Vid, (PictureStructure) p_Vid->structure, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr);
  
  p_Vid->ThisPOC             = p_Vid->framepoc;
  (*stored_pic)->poc         = p_Vid->framepoc;
  (*stored_pic)->top_poc     = p_Vid->toppoc;
  (*stored_pic)->bottom_poc  = p_Vid->bottompoc;
  (*stored_pic)->frame_poc   = p_Vid->framepoc;
  (*stored_pic)->pic_num     = p_Vid->frame_num;
  (*stored_pic)->frame_num   = p_Vid->frame_num;
  (*stored_pic)->coded_frame = 1;
  (*stored_pic)->mb_aff_frame_flag = p_Vid->mb_aff_frame_flag = (Boolean) (p_Inp->MbInterlace != FRAME_CODING);
  
  p_Vid->get_mb_block_pos    = p_Vid->mb_aff_frame_flag ? get_mb_block_pos_mbaff : get_mb_block_pos_normal;
  p_Vid->getNeighbour        = p_Vid->mb_aff_frame_flag ? getAffNeighbour : getNonAffNeighbour;
  p_Vid->enc_picture         = *stored_pic;

  copy_params(p_Vid, p_Vid->enc_picture, p_Vid->active_sps);
}

static void calc_picture_bits(Picture *frame)
{
  int i, j;
  Slice *thisSlice = NULL;

  frame->bits_per_picture = 0;

  for ( i = 0; i < frame->no_slices; i++ )
  {
    thisSlice = frame->slices[i];

    for ( j = 0; j < thisSlice->max_part_nr; j++ )
      frame->bits_per_picture += 8 * ((thisSlice->partArr[j]).bitstream)->byte_pos;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Encodes a frame picture
 ************************************************************************
 */
void frame_picture (VideoParameters *p_Vid, Picture *frame, ImageData *imgData, int rd_pass)
{
  int nplane;
  InputParameters *p_Inp = p_Vid->p_Inp;
  p_Vid->rd_pass = rd_pass;
  p_Vid->SumFrameQP = 0;
  p_Vid->num_ref_idx_l0_active = 0;
  p_Vid->num_ref_idx_l1_active = 0;
  p_Vid->p_curr_pic = p_Vid->p_curr_frm_struct->p_frame_pic;
  p_Vid->structure = FRAME;
  p_Vid->PicSizeInMbs = p_Vid->FrameSizeInMbs;
  //set mv limits to frame type
  update_mv_limits(p_Vid, FALSE);

  InitWP(p_Vid, p_Inp, 0);
  if(rd_pass == 0 && p_Vid->wp_parameters_set == 0)
    ResetWP(p_Vid, p_Inp); 

  if( (p_Inp->separate_colour_plane_flag != 0) )
  {
    for( nplane=0; nplane<MAX_PLANE; nplane++ )
    {
      prepare_enc_frame_picture( p_Vid, &p_Vid->enc_frame_picture_JV[nplane] );      
    }
  }
  else
  {
    prepare_enc_frame_picture( p_Vid, &p_Vid->enc_frame_picture[rd_pass] );
  }

  p_Vid->fld_flag = FALSE;
  code_a_picture(p_Vid, frame);

  if( (p_Inp->separate_colour_plane_flag != 0) )
  {
    make_frame_picture_JV(p_Vid);
  }

  calc_picture_bits(frame);

  if (p_Vid->structure==FRAME)
  {
    find_distortion (p_Vid, imgData);
    frame->distortion = p_Vid->p_Dist->metric[SSE];
  }
}


/*!
 ************************************************************************
 * \brief
 *    Encodes a field picture, consisting of top and bottom field
 ************************************************************************
 */
static void field_picture (VideoParameters *p_Vid, Picture *top, Picture *bottom)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  //Rate control
  int TopFieldBits;

  p_Vid->SumFrameQP = 0;
  p_Vid->num_ref_idx_l0_active = 0;
  p_Vid->num_ref_idx_l1_active = 0;

  //set mv limits to field type
  update_mv_limits(p_Vid, TRUE);

  p_Vid->height    = (p_Inp->output.height[0] + p_Vid->auto_crop_bottom) >> 1;
  p_Vid->height_cr = p_Vid->height_cr_frame >> 1;
  p_Vid->fld_flag  = TRUE;
  p_Vid->PicSizeInMbs = p_Vid->FrameSizeInMbs >> 1;
  // Top field

  p_Vid->enc_field_picture[0]              = alloc_storable_picture (p_Vid, (PictureStructure) p_Vid->structure, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr);
  p_Vid->enc_field_picture[0]->poc         = p_Vid->toppoc;
  p_Vid->enc_field_picture[0]->frame_poc   = p_Vid->toppoc;
  /*Potentially buggy to not divide by num_of_layers */
  p_Vid->enc_field_picture[0]->pic_num     = p_Vid->frame_num;
  p_Vid->enc_field_picture[0]->frame_num   = p_Vid->frame_num;
  p_Vid->enc_field_picture[0]->coded_frame = 0;
  p_Vid->enc_field_picture[0]->mb_aff_frame_flag = p_Vid->mb_aff_frame_flag = FALSE;
  p_Vid->get_mb_block_pos = get_mb_block_pos_normal;
  p_Vid->getNeighbour = getNonAffNeighbour;
  p_Vid->ThisPOC = p_Vid->toppoc;

  p_Vid->p_curr_pic = p_Vid->p_curr_frm_struct->p_top_fld_pic;
  p_Vid->structure = TOP_FIELD;
  p_Vid->enc_picture = p_Vid->enc_field_picture[0];
  copy_params(p_Vid, p_Vid->enc_picture, p_Vid->active_sps);

  put_buffer_top (p_Vid);
  init_field (p_Vid, p_Inp);
  set_slice_type(p_Vid, p_Inp, p_Vid->p_curr_pic->p_Slice[0].type);

  p_Vid->fld_flag = TRUE;

  //Rate control
  if(p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE)
    rc_init_top_field(p_Vid, p_Inp);

  code_a_picture(p_Vid, top);
  p_Vid->enc_picture->structure = TOP_FIELD;
  store_picture_in_dpb(p_Vid->p_Dpb_layer[p_Vid->view_id], p_Vid->enc_field_picture[0], &p_Inp->output);

#if (MVC_EXTENSION_ENABLE)
  if ( is_MVC_profile(p_Vid->active_sps->profile_idc) && p_Vid->proc_picture)
  {
    store_proc_picture_in_dpb (p_Vid->p_Dpb_layer[1], p_Vid->proc_picture, &p_Inp->output);
    p_Vid->proc_picture = NULL;
  }
#endif
  calc_picture_bits(top);

  //Rate control
  TopFieldBits=top->bits_per_picture;

  //  Bottom field
  p_Vid->enc_field_picture[1]  = alloc_storable_picture (p_Vid, (PictureStructure) p_Vid->structure, p_Vid->width, p_Vid->height, p_Vid->width_cr, p_Vid->height_cr);
  p_Vid->enc_field_picture[1]->poc=p_Vid->bottompoc;
  p_Vid->enc_field_picture[1]->frame_poc = p_Vid->bottompoc;
  p_Vid->enc_field_picture[1]->pic_num = p_Vid->frame_num;
  p_Vid->enc_field_picture[1]->frame_num = p_Vid->frame_num;
  p_Vid->enc_field_picture[1]->coded_frame = 0;
  p_Vid->enc_field_picture[1]->mb_aff_frame_flag = p_Vid->mb_aff_frame_flag = FALSE;
  p_Vid->get_mb_block_pos = get_mb_block_pos_normal;
  p_Vid->getNeighbour = getNonAffNeighbour;

  p_Vid->ThisPOC = p_Vid->bottompoc;
  p_Vid->p_curr_pic = p_Vid->p_curr_frm_struct->p_bot_fld_pic;
  p_Vid->structure = BOTTOM_FIELD;
  p_Vid->enc_picture = p_Vid->enc_field_picture[1];
  copy_params(p_Vid, p_Vid->enc_picture, p_Vid->active_sps);
  put_buffer_bot (p_Vid);
  
  init_field (p_Vid, p_Inp);
  set_slice_type(p_Vid, p_Inp, p_Vid->p_curr_pic->p_Slice[0].type);

  p_Vid->fld_flag = TRUE;

  //Rate control
  if(p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE)
    rc_init_bottom_field( p_Vid, p_Inp, TopFieldBits );

  p_Vid->enc_picture->structure = BOTTOM_FIELD;
  code_a_picture(p_Vid, bottom);

  calc_picture_bits(bottom);

  // the distortion for a field coded frame (consisting of top and bottom field)
  // lives in the top->distortion variables, the bottom-> are dummies  
  distortion_fld (p_Vid, p_Inp, top, &p_Vid->imgData);
}

/*!
 ************************************************************************
 * \brief
 *    form frame picture from two field pictures
 ************************************************************************
 */
static void combine_field(VideoParameters *p_Vid)
{
  int i, k;

  for (i = 0; i < (p_Vid->height >> 1); i++)
  {
    memcpy(p_Vid->imgY_com[i*2],     p_Vid->enc_field_picture[0]->imgY[i], p_Vid->width*sizeof(imgpel));     // top field
    memcpy(p_Vid->imgY_com[i*2 + 1], p_Vid->enc_field_picture[1]->imgY[i], p_Vid->width*sizeof(imgpel)); // bottom field
  }

  if (p_Vid->yuv_format != YUV400)
  {
    for (k = 0; k < 2; k++)
    {
      for (i = 0; i < (p_Vid->height_cr >> 1); i++)
      {
        memcpy(p_Vid->imgUV_com[k][i * 2],     p_Vid->enc_field_picture[0]->imgUV[k][i], p_Vid->width_cr*sizeof(imgpel));
        memcpy(p_Vid->imgUV_com[k][i * 2 + 1], p_Vid->enc_field_picture[1]->imgUV[k][i], p_Vid->width_cr*sizeof(imgpel));
      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Distortion Field
 ************************************************************************
 */
static void distortion_fld (VideoParameters *p_Vid, InputParameters *p_Inp, Picture *field_pic, ImageData *imgData)
{
  p_Vid->height    = (p_Inp->output.height[0] + p_Vid->auto_crop_bottom);
  p_Vid->height_cr = p_Vid->height_cr_frame;

  combine_field (p_Vid);

  p_Vid->pCurImg   = imgData->frm_data[0];
  p_Vid->pImgOrg[0] = imgData->frm_data[0];

  if (p_Inp->output.yuv_format != YUV400)
  {
    p_Vid->pImgOrg[1] = imgData->frm_data[1];
    p_Vid->pImgOrg[2] = imgData->frm_data[2];
  }

  find_distortion (p_Vid, imgData);   // find snr from original frame picture
  field_pic->distortion = p_Vid->p_Dist->metric[SSE];
}


/*!
 ************************************************************************
 * \brief
 *    RD decision of frame and field coding
 ************************************************************************
 */
static byte decide_fld_frame(float snr_frame_Y, float snr_field_Y, int bit_field, int bit_frame, double lambda_picture)
{
  double cost_frame, cost_field;

  cost_frame = bit_frame * lambda_picture + snr_frame_Y;
  cost_field = bit_field * lambda_picture + snr_field_Y;

  if (cost_field > cost_frame)
    return FALSE;
  else
    return TRUE;
}

/*!
 ************************************************************************
 * \brief
 *    Picture Structure Decision
 ************************************************************************
 */
static byte picture_structure_decision (VideoParameters *p_Vid, Picture *frame, Picture *top, Picture *bot)
{
  double lambda_picture;
  float sse_frame, sse_field;
  int bit_frame, bit_field;

  lambda_picture = 0.68 * pow (2, p_Vid->bitdepth_lambda_scale + ((p_Vid->qp - SHIFT_QP) / 3.0)) * ((p_Vid->type == B_SLICE) ? 1 : 1);

  sse_frame = frame->distortion.value[0] + frame->distortion.value[1] + frame->distortion.value[2];
  //! all distrortions of a field picture are accumulated in the top field
  sse_field = top->distortion.value[0] + top->distortion.value[1] + top->distortion.value[2];

  bit_field = top->bits_per_picture + bot->bits_per_picture;
  bit_frame = frame->bits_per_picture;
  return decide_fld_frame (sse_frame, sse_field, bit_field, bit_frame, lambda_picture);
}


/*!
 ************************************************************************
 * \brief
 *    Field Mode Buffer
 ************************************************************************
 */
static void field_mode_buffer (VideoParameters *p_Vid)
{
  put_buffer_frame (p_Vid);
}


/*!
 ************************************************************************
 * \brief
 *    Frame Mode Buffer
 ************************************************************************
 */
static void frame_mode_buffer (VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  put_buffer_frame (p_Vid);

  if ((p_Inp->PicInterlace != FRAME_CODING)||(p_Inp->MbInterlace != FRAME_CODING))
  {
    p_Vid->height = p_Vid->height / 2;
    p_Vid->height_cr = p_Vid->height_cr / 2;

    put_buffer_top (p_Vid);

    put_buffer_bot (p_Vid);

    p_Vid->height = (p_Inp->output.height[0] + p_Vid->auto_crop_bottom);
    p_Vid->height_cr = p_Vid->height_cr_frame;
 
    put_buffer_frame (p_Vid);
  }
}


/*!
 ************************************************************************
 * \brief
 *    mmco initializations should go here
 ************************************************************************
 */
static void init_dec_ref_pic_marking_buffer(VideoParameters *p_Vid)
{
  p_Vid->dec_ref_pic_marking_buffer=NULL;
}

/*!
************************************************************************
* \brief
*    Init fixed QP I_SLICE
************************************************************************
*/

static inline void init_fixed_qp_i_slice(VideoParameters *p_Vid, InputParameters *p_Inp, FrameUnitStruct *p_cur_frm )
{
  //QP oscillation for secondary SP frames
  p_Vid->qp = p_cur_frm->qp;   // set quant. parameter for I-frame

  if (p_Vid->redundant_coding)
  {
    //!KS: hard code qp increment
    p_Vid->qp = imin(p_Vid->qp + 5, 51);
  }
}

/*!
************************************************************************
* \brief
*    Init fixed QP P_SLICE or B_SLICE
************************************************************************
*/

static inline void init_fixed_qp_pb_slice(VideoParameters *p_Vid, InputParameters *p_Inp, FrameUnitStruct *p_cur_frm, int type )
{
  //QP oscillation for secondary SP frames
  p_Vid->qp = p_cur_frm->qp;

}

/*!
************************************************************************
* \brief
*    Init fixed QP SP_SLICE
************************************************************************
*/

static inline void init_fixed_qp_sp_slice(VideoParameters *p_Vid, InputParameters *p_Inp, FrameUnitStruct *p_cur_frm )
{
  p_Vid->qp   = p_cur_frm->qp;
  p_Vid->qpsp = p_Inp->qpsp;
}

/*!
************************************************************************
* \brief
*    Initializes the quantization parameters for a new frame (no RC)
*    ***Much of what happens in this function have to be moved elsewhere...***
************************************************************************
*/

static void init_fixed_qp(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  FrameUnitStruct *p_cur_frm = p_Vid->p_curr_frm_struct;

  if (p_cur_frm->layer == 0)
  {
    switch( p_cur_frm->type )
    {
    default:
    case I_SLICE:
      init_fixed_qp_i_slice( p_Vid, p_Inp, p_cur_frm );
      break;
    case P_SLICE:      
    case B_SLICE:
      init_fixed_qp_pb_slice( p_Vid, p_Inp, p_cur_frm, p_cur_frm->type );
      break;
    case SP_SLICE:
    case SI_SLICE:
      init_fixed_qp_sp_slice( p_Vid, p_Inp, p_cur_frm );
      break;
    }

    p_Vid->mb_y_intra = p_Vid->mb_y_upd;  //  p_Vid->mb_y_intra indicates which GOB to intra code for this frame

    if (p_Inp->intra_upd > 0) // if error robustness, find next GOB to update
    {
      p_Vid->mb_y_upd = (p_Vid->curr_frm_idx / p_Inp->intra_upd) % (p_Vid->height / MB_BLOCK_SIZE);
    }
  }
  else
  {
    p_Vid->qp = p_cur_frm->qp;
  }

  // Change QP
  if ( p_Inp->qp2frame && p_Inp->qp2frame < p_Vid->frame_no )
  {
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp = iClip3( -p_Vid->bitdepth_luma_qp_scale, MAX_QP, p_Vid->qp + p_Inp->qp2off[p_cur_frm->type] );
  }
}

/*!
 ************************************************************************
 * \brief
 *    Initializes the quantization parameters for a new frame (no RC)
 ************************************************************************
 */

static void init_mb_line_intra_up(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  FrameUnitStruct *p_cur_frm = p_Vid->p_curr_frm_struct;

  if (p_cur_frm->layer == 0)
  {
    p_Vid->mb_y_intra = p_Vid->mb_y_upd;  //  p_Vid->mb_y_intra indicates which GOB to intra code for this frame

    if (p_Inp->intra_upd > 0) // if error robustness, find next GOB to update
    {
      p_Vid->mb_y_upd = (p_Vid->curr_frm_idx / p_Inp->intra_upd) % (p_Vid->width / MB_BLOCK_SIZE);
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Initializes the parameters for a new frame
 ************************************************************************
 */
static void init_frame (VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int i, j;

  p_Vid->current_mb_nr = 0;
  p_Vid->current_slice_nr = 0;
  p_Vid->p_Stats->bit_slice = 0;

  // The 'slice_nr' of each macroblock is set to -1 here, to guarantee the correct encoding
  // with FMO (if no FMO, encoding is correct without following assignment),
  // for which MBs may not be encoded with scan order
  if( (p_Inp->separate_colour_plane_flag != 0) )
  {
    for( j = 0; j < MAX_PLANE; j++ ){
      for(i = 0; i < ((int) (p_Vid->FrameSizeInMbs));i++)
        p_Vid->mb_data_JV[j][i].slice_nr = -1;
    }
  }
  else
  {
    for(i = 0; i < ((int) (p_Vid->FrameSizeInMbs)); i++)
      p_Vid->mb_data[i].slice_nr = -1;
  }

  if ( !(p_Inp->RCEnable) )
  {
    init_fixed_qp(p_Vid, p_Inp);
  }
  init_mb_line_intra_up(p_Vid, p_Inp);

  p_Vid->no_output_of_prior_pics_flag = 0;
  p_Vid->long_term_reference_flag = FALSE;

  init_dec_ref_pic_marking_buffer(p_Vid);

  if(p_Inp->WPIterMC)
    p_Vid->frameOffsetAvail = 0;

  // set parameters for direct mode and deblocking filter
  // currently selection is done at the frame level instead of slice level. This needs to be changed.  
  p_Vid->direct_spatial_mv_pred_flag = (char) p_Inp->direct_spatial_mv_pred_flag;
#if (MVC_EXTENSION_ENABLE)
  if ( (p_Inp->num_of_views == 1) || (p_Vid->view_id == 0) )
#endif
  {
    p_Vid->DFDisableIdc                = (char) p_Inp->DFDisableIdc[p_Vid->nal_reference_idc > 0][p_Vid->type];
    p_Vid->DFAlphaC0Offset             = (char) p_Inp->DFAlpha     [p_Vid->nal_reference_idc > 0][p_Vid->type];
    p_Vid->DFBetaOffset                = (char) p_Inp->DFBeta      [p_Vid->nal_reference_idc > 0][p_Vid->type];
  }
#if (MVC_EXTENSION_ENABLE)
  else
  {
    p_Vid->DFDisableIdc                = (char) p_Inp->EnhLayerDFDisableIdc[p_Vid->nal_reference_idc > 0][p_Vid->type];
    p_Vid->DFAlphaC0Offset             = (char) p_Inp->EnhLayerDFAlpha     [p_Vid->nal_reference_idc > 0][p_Vid->type];
    p_Vid->DFBetaOffset                = (char) p_Inp->EnhLayerDFBeta      [p_Vid->nal_reference_idc > 0][p_Vid->type];
  }
#endif
  p_Vid->AdaptiveRounding            = p_Inp->AdaptiveRounding; 
}

/*!
 ************************************************************************
 * \brief
 *    Initializes the parameters for a new field
 ************************************************************************
 */
static void init_field (VideoParameters *p_Vid, InputParameters *p_Inp)
{
  FrameUnitStruct *p_cur_frm = p_Vid->p_curr_frm_struct;

  p_Vid->current_mb_nr = 0;
  p_Vid->current_slice_nr = 0;
  p_Vid->p_Stats->bit_slice = 0;

  if (!p_cur_frm->layer)
  {
    if(!p_Inp->RCEnable)                  // without using rate control
    {
      p_Vid->qp = p_cur_frm->qp;
      if (p_Vid->type == SP_SLICE || p_Vid->type == SI_SLICE)
      {
        p_Vid->qpsp = p_Inp->qpsp;
      }
    }
    init_mb_line_intra_up( p_Vid, p_Inp );
  }
  else
  {
    if(!p_Inp->RCEnable)
    {
      p_Vid->qp = p_cur_frm->qp;
    }
  }
}

void GenerateImagePyramid(VideoParameters *p_Vid, int size_x, int size_y, imgpel ***p_hme_img, int offset_x, int offset_y)
{
  int i, iPrevWidth, iPrevHeight,iCurrWidth, iCurrHeight;
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;
  
  iPrevWidth = size_x;
  iPrevHeight = size_y;
 
  for(i=1; i < pHMEInfo->iPyramidLevels; i++)
  {
    iCurrWidth  = iPrevWidth  >>1;
    iCurrHeight = iPrevHeight >>1;

    PyrDownG5x5_U8CnR((const imgpel *) *(p_hme_img[i-1]), (int) ((p_hme_img[i-1][1]-p_hme_img[i-1][0])*sizeof(imgpel)), iPrevWidth, iPrevHeight, *(p_hme_img[i]), (int) ((p_hme_img[i][1]-p_hme_img[i][0])*sizeof(imgpel)), 1);

    // update image dimensions
    iPrevWidth = iCurrWidth;
    iPrevHeight = iCurrHeight;

  }
}

void GenerateHMELayers(VideoParameters *p_Vid, StorablePicture *s)
{
    //Generate Image pyramid;
    s->pHmeImage[0] = s->p_curr_img;
    GenerateImagePyramid(p_Vid, s->size_x, s->size_y, s->pHmeImage, IMG_PAD_SIZE_X, IMG_PAD_SIZE_Y);
    //Add Padding (we could remove this as well);
    GetHMEIntImagesLuma(p_Vid, s->size_x, s->size_y, s->pHmeImage);
 }
 
/*!
 ************************************************************************
 * \brief
 *    Upsample 4 times, store them in out4x.  Color is simply copied
 *
 * \par Input:
 *    srcy, srcu, srcv, out4y, out4u, out4v
 *
 * \par Side Effects_
 *    Uses (writes) img4Y_tmp.  This should be moved to a static variable
 *    in this module
 ************************************************************************/
void UnifiedOneForthPix ( VideoParameters *p_Vid, StorablePicture *s)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  if(s->bInterpolated)
    return;
  s->bInterpolated = 1;
  // Y component
  s->p_img_sub[0] = s->imgY_sub;
  s->p_curr_img_sub = s->imgY_sub;
  s->p_curr_img = s->imgY;

  // derive the subpixel images for first component
  // No need to interpolate if intra only encoding
  //if (p_Inp->intra_period != 1)
  if ( (!p_Inp->OnTheFlyFractMCP) || (p_Inp->OnTheFlyFractMCP==OTF_L1) ) // JLT : on-the-fly
  {
    getSubImagesLuma ( p_Vid, s );
    // and the sub-images for U and V
    if ( ((p_Vid->yuv_format != YUV400) && (p_Inp->ChromaMCBuffer)) || p_Vid->P444_joined )
    {
      if (p_Vid->P444_joined)
      {
        imgpel **** p_curr_img_sub = s->p_curr_img_sub;
        imgpel **   p_curr_img = s->p_curr_img;
        //U
        select_plane(p_Vid, PLANE_U);
        s->p_curr_img_sub = s->imgUV_sub[0];
        s->p_curr_img = s->imgUV[0];
        getSubImagesLuma (p_Vid, s);
        //V
        select_plane(p_Vid, PLANE_V);
        s->p_curr_img_sub = s->imgUV_sub[1];
        s->p_curr_img = s->imgUV[1];
        getSubImagesLuma (p_Vid, s);
        //Y
        select_plane(p_Vid, PLANE_Y);
        s->p_curr_img_sub = p_curr_img_sub;
        s->p_curr_img = p_curr_img;
      }
      else
      {
        getSubImagesChroma( p_Vid, s );
      }
    }
  }
  else
  {
    // perform  padding ( copying borders) that is implicitly done above if p_Inp->OnTheFlyFractMCP=0
    OtfCompatibility_copyWithPadding( s->imgY, s->imgY, s->size_x, s->size_y, IMG_PAD_SIZE_X, IMG_PAD_SIZE_Y ) ;
    OtfCompatibility_copyWithPadding( s->imgUV[0], s->imgUV[0], s->size_x_cr, s->size_y_cr, p_Vid->pad_size_uv_x,p_Vid->pad_size_uv_y ) ;
    OtfCompatibility_copyWithPadding( s->imgUV[1], s->imgUV[1], s->size_x_cr, s->size_y_cr, p_Vid->pad_size_uv_x, p_Vid->pad_size_uv_y ) ;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Upsample 4 times, store them in out4x.  Color is simply copied
 *    for 4:4:4 Independent mode
 *
 * \par Input:
 *    nplane
 *
 ************************************************************************/
void UnifiedOneForthPix_JV (VideoParameters *p_Vid, int nplane, StorablePicture *s)
{
  InputParameters *p_Inp = p_Vid->p_Inp;

  if( nplane == 0 )
  {
    if(s->bInterpolated)
      return;
    s->bInterpolated = 1;
    s->p_img[0] = s->imgY;
    s->p_img[1] = s->imgUV[0];
    s->p_img[2] = s->imgUV[1];

    if ( (!p_Inp->OnTheFlyFractMCP) || (p_Inp->OnTheFlyFractMCP==OTF_L1) ) // JLT : on-the-fly
    {
      s->p_img_sub[0] = s->imgY_sub;
      s->p_img_sub[1] = s->imgUV_sub[0];
      s->p_img_sub[2] = s->imgUV_sub[1];
    }
  }

  // derive the subpixel images for first component
  s->colour_plane_id = nplane;
  s->p_curr_img = s->p_img[nplane];
  s->p_curr_img_sub = s->p_img_sub[nplane];

  if( (!p_Inp->OnTheFlyFractMCP) || (p_Inp->OnTheFlyFractMCP==OTF_L1) )
  {
    getSubImagesLuma ( p_Vid, s );
  }
  else
  {
    // perform  padding ( copying borders) that is implicitly done above if p_Inp->OnTheFlyFractMCP=0
     OtfCompatibility_copyWithPadding( s->p_img[nplane], s->p_img[nplane], s->size_x, s->size_y, IMG_PAD_SIZE_X, IMG_PAD_SIZE_Y ) ;
  }
}

  /*!
 ************************************************************************
 * \brief
 *    Just a placebo
 ************************************************************************
 */
Boolean dummy_slice_too_big (int bits_slice)
{
  return FALSE;
}

static void ReportSimple(VideoParameters *p_Vid, char *pic_type, int cur_bits, DistMetric *metric, int tmp_time)
{
#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->p_Inp->num_of_views == 2 )
  {
    if ( p_Vid->fld_flag && p_Vid->view_id == 0 )
    {
      p_Vid->prev_cs.frm_no_in_file = p_Vid->frm_no_in_file;
      p_Vid->prev_cs.view_id = p_Vid->view_id;
      p_Vid->prev_cs.cur_bits = cur_bits;
      strcpy( p_Vid->prev_cs.pic_type, pic_type );
      p_Vid->prev_cs.AverageFrameQP = p_Vid->AverageFrameQP;
      p_Vid->prev_cs.lambda = 0;
      p_Vid->prev_cs.psnr_value[0] = metric->value[0];
      p_Vid->prev_cs.psnr_value[1] = metric->value[1];
      p_Vid->prev_cs.psnr_value[2] = metric->value[2];
      p_Vid->prev_cs.ssim_value[0] = 0;
      p_Vid->prev_cs.ssim_value[1] = 0;
      p_Vid->prev_cs.ssim_value[2] = 0;
      p_Vid->prev_cs.tmp_time = tmp_time;
      p_Vid->prev_cs.me_time = (int)p_Vid->me_time;
      p_Vid->prev_cs.fld_flag = p_Vid->fld_flag;
      p_Vid->prev_cs.intras = p_Vid->intras;
      p_Vid->prev_cs.direct_mode = 0;
      p_Vid->prev_cs.num_ref_idx_l0_active = p_Vid->num_ref_idx_l0_active;
      p_Vid->prev_cs.num_ref_idx_l1_active = p_Vid->num_ref_idx_l1_active;
      p_Vid->prev_cs.rd_pass = p_Vid->rd_pass;
      p_Vid->prev_cs.nal_reference_idc = p_Vid->nal_reference_idc;
    }
    else if ( p_Vid->fld_flag && p_Vid->view_id == 1 )
    {
      printf ("%05d(%3s)  %1d  %8d   %2d %7.3f %7.3f %7.3f %9d %7d    %3s    %d\n",
        p_Vid->prev_cs.frm_no_in_file, p_Vid->prev_cs.pic_type, p_Vid->prev_cs.view_id,
        (int)(p_Vid->p_Stats->bit_ctr_v[0] - p_Vid->p_Stats->bit_ctr_n_v[0]) + (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[0] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[0]),
        p_Vid->prev_cs.AverageFrameQP,
        p_Vid->prev_cs.psnr_value[0], p_Vid->prev_cs.psnr_value[1], p_Vid->prev_cs.psnr_value[2], 
        p_Vid->prev_cs.tmp_time, (int) p_Vid->prev_cs.me_time,
        p_Vid->prev_cs.fld_flag ? "FLD" : "FRM", 
        p_Vid->prev_cs.nal_reference_idc);
      printf ("%05d(%3s)  %1d  %8d   %2d %7.3f %7.3f %7.3f %9d %7d    %3s    %d\n",
        p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, 
        (int)(p_Vid->p_Stats->bit_ctr_v[1] - p_Vid->p_Stats->bit_ctr_n_v[1]) + (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[1] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[1]), 
        p_Vid->AverageFrameQP,
        metric->value[0], metric->value[1], metric->value[2], 
        tmp_time, (int) p_Vid->me_time,
        p_Vid->fld_flag ? "FLD" : "FRM", 
        p_Vid->nal_reference_idc);      
    }
    else
    {
      printf ("%05d(%3s)  %1d  %8d   %2d %7.3f %7.3f %7.3f %9d %7d    %3s    %d\n",
        p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, cur_bits, 
        p_Vid->AverageFrameQP,
        metric->value[0], metric->value[1], metric->value[2], 
        tmp_time, (int) p_Vid->me_time,
        p_Vid->fld_flag ? "FLD" : "FRM", 
        p_Vid->nal_reference_idc);
    }
  }
  else
#endif
  printf ("%05d(%3s)%8d   %2d %7.3f %7.3f %7.3f %9d %7d    %3s    %d\n",
    p_Vid->frm_no_in_file, pic_type, cur_bits, 
    p_Vid->AverageFrameQP,
    metric->value[0], metric->value[1], metric->value[2], 
    tmp_time, (int) p_Vid->me_time,
    p_Vid->fld_flag ? "FLD" : "FRM", 
    p_Vid->nal_reference_idc);
}

static void ReportVerbose(VideoParameters *p_Vid, char *pic_type, int cur_bits, int wp_method, int lambda, DistMetric *mPSNR, int tmp_time, int direct_mode)
{
#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->p_Inp->num_of_views == 2 )
  {
    if ( p_Vid->fld_flag && p_Vid->view_id == 0 )
    {
      p_Vid->prev_cs.frm_no_in_file = p_Vid->frm_no_in_file;
      p_Vid->prev_cs.view_id = p_Vid->view_id;
      p_Vid->prev_cs.cur_bits = cur_bits;
      strcpy( p_Vid->prev_cs.pic_type, pic_type );
      p_Vid->prev_cs.AverageFrameQP = p_Vid->AverageFrameQP;
      p_Vid->prev_cs.lambda = 0;
      p_Vid->prev_cs.psnr_value[0] = mPSNR->value[0];
      p_Vid->prev_cs.psnr_value[1] = mPSNR->value[1];
      p_Vid->prev_cs.psnr_value[2] = mPSNR->value[2];
      p_Vid->prev_cs.ssim_value[0] = 0;
      p_Vid->prev_cs.ssim_value[1] = 0;
      p_Vid->prev_cs.ssim_value[2] = 0;
      p_Vid->prev_cs.tmp_time = tmp_time;
      p_Vid->prev_cs.me_time = (int)p_Vid->me_time;
      p_Vid->prev_cs.fld_flag = p_Vid->fld_flag;
      p_Vid->prev_cs.intras = p_Vid->intras;
      p_Vid->prev_cs.direct_mode = direct_mode;
      p_Vid->prev_cs.num_ref_idx_l0_active = p_Vid->num_ref_idx_l0_active;
      p_Vid->prev_cs.num_ref_idx_l1_active = p_Vid->num_ref_idx_l1_active;
      p_Vid->prev_cs.rd_pass = p_Vid->rd_pass;
      p_Vid->prev_cs.nal_reference_idc = p_Vid->nal_reference_idc;
    }
    else if ( p_Vid->fld_flag && p_Vid->view_id == 1 )
    {
      printf ("%05d(%3s)  %1d  %8d %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
        p_Vid->prev_cs.frm_no_in_file, p_Vid->prev_cs.pic_type, p_Vid->prev_cs.view_id,
        (int)(p_Vid->p_Stats->bit_ctr_v[0] - p_Vid->p_Stats->bit_ctr_n_v[0]) + (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[0] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[0]),
        p_Vid->prev_cs.wp_method, p_Vid->prev_cs.AverageFrameQP, p_Vid->prev_cs.lambda, 
        p_Vid->prev_cs.psnr_value[0], p_Vid->prev_cs.psnr_value[1], p_Vid->prev_cs.psnr_value[2], 
        p_Vid->prev_cs.tmp_time, (int) p_Vid->prev_cs.me_time,
        p_Vid->prev_cs.fld_flag ? "FLD" : "FRM", p_Vid->prev_cs.intras, p_Vid->prev_cs.direct_mode,
        p_Vid->prev_cs.num_ref_idx_l0_active, p_Vid->prev_cs.num_ref_idx_l1_active, p_Vid->prev_cs.rd_pass, p_Vid->prev_cs.nal_reference_idc);
      printf ("%05d(%3s)  %1d  %8d %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
        p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, 
        (int)(p_Vid->p_Stats->bit_ctr_v[1] - p_Vid->p_Stats->bit_ctr_n_v[1]) + (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[1] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[1]), wp_method,
        p_Vid->AverageFrameQP, lambda, 
        mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
        tmp_time, (int) p_Vid->me_time,
        p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
        p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
    }
    else
    {
      printf ("%05d(%3s)  %1d  %8d %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
        p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, cur_bits, wp_method,
        p_Vid->AverageFrameQP, lambda, 
        mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
        tmp_time, (int) p_Vid->me_time,
        p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
        p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
    }
  }
  else
#endif
  {
  printf ("%05d(%3s)%8d %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
    p_Vid->frm_no_in_file, pic_type, cur_bits, wp_method,
    p_Vid->AverageFrameQP, lambda, 
    mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
    tmp_time, (int) p_Vid->me_time,
    p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
    p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
  }
}

static void ReportVerboseNVB(VideoParameters *p_Vid, char *pic_type, int cur_bits, int nvb_bits, int wp_method, int lambda, DistMetric *mPSNR, int tmp_time, int direct_mode)
{
#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->num_of_layers == 2 )
  {
    if ( p_Vid->fld_flag && p_Vid->view_id == 0 )
    {
      p_Vid->prev_cs.frm_no_in_file = p_Vid->frm_no_in_file;
      p_Vid->prev_cs.view_id = p_Vid->view_id;
      p_Vid->prev_cs.cur_bits = cur_bits;
      strcpy( p_Vid->prev_cs.pic_type, pic_type );
      p_Vid->prev_cs.AverageFrameQP = p_Vid->AverageFrameQP;
      p_Vid->prev_cs.lambda = 0;
      p_Vid->prev_cs.psnr_value[0] = mPSNR->value[0];
      p_Vid->prev_cs.psnr_value[1] = mPSNR->value[1];
      p_Vid->prev_cs.psnr_value[2] = mPSNR->value[2];
      p_Vid->prev_cs.ssim_value[0] = 0;
      p_Vid->prev_cs.ssim_value[1] = 0;
      p_Vid->prev_cs.ssim_value[2] = 0;
      p_Vid->prev_cs.tmp_time = tmp_time;
      p_Vid->prev_cs.me_time = (int)p_Vid->me_time;
      p_Vid->prev_cs.fld_flag = p_Vid->fld_flag;
      p_Vid->prev_cs.intras = p_Vid->intras;
      p_Vid->prev_cs.direct_mode = direct_mode;
      p_Vid->prev_cs.num_ref_idx_l0_active = p_Vid->num_ref_idx_l0_active;
      p_Vid->prev_cs.num_ref_idx_l1_active = p_Vid->num_ref_idx_l1_active;
      p_Vid->prev_cs.rd_pass = p_Vid->rd_pass;
      p_Vid->prev_cs.nal_reference_idc = p_Vid->nal_reference_idc;
    }
    else if ( p_Vid->fld_flag && p_Vid->view_id == 1 )
    {
      printf ("%05d(%3s)  %1d  %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
        p_Vid->prev_cs.frm_no_in_file, p_Vid->prev_cs.pic_type, p_Vid->prev_cs.view_id,
        (int)(p_Vid->p_Stats->bit_ctr_v[0] - p_Vid->p_Stats->bit_ctr_n_v[0]) 
        + (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[0] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[0]) + p_Vid->p_Stats->bit_ctr_parametersets_n_v[0],
        p_Vid->p_Stats->bit_ctr_parametersets_n_v[0], p_Vid->prev_cs.wp_method, p_Vid->prev_cs.AverageFrameQP, p_Vid->prev_cs.lambda, 
        p_Vid->prev_cs.psnr_value[0], p_Vid->prev_cs.psnr_value[1], p_Vid->prev_cs.psnr_value[2], 
        p_Vid->prev_cs.tmp_time, (int) p_Vid->prev_cs.me_time,
        p_Vid->prev_cs.fld_flag ? "FLD" : "FRM", p_Vid->prev_cs.intras, p_Vid->prev_cs.direct_mode,
        p_Vid->prev_cs.num_ref_idx_l0_active, p_Vid->prev_cs.num_ref_idx_l1_active, p_Vid->prev_cs.rd_pass, p_Vid->prev_cs.nal_reference_idc);
      printf ("%05d(%3s)  %1d  %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
        p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, 
        (int)(p_Vid->p_Stats->bit_ctr_v[1] - p_Vid->p_Stats->bit_ctr_n_v[1]) 
        + (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[1] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[1]) + p_Vid->p_Stats->bit_ctr_parametersets_n_v[1], 
        p_Vid->p_Stats->bit_ctr_parametersets_n_v[1], wp_method,
        p_Vid->AverageFrameQP, lambda, 
        mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
        tmp_time, (int) p_Vid->me_time,
        p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
        p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
    }
    else
    {
      printf ("%05d(%3s)  %1d  %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d  %d\n",
        p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, cur_bits, nvb_bits, wp_method,
        p_Vid->AverageFrameQP, lambda, 
        mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
        tmp_time, (int) p_Vid->me_time,
        p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
        p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc, p_Vid->iInterViewMBs);
    }
  }
  else
#endif
  {
    printf ("%05d(%3s)%8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
      p_Vid->frm_no_in_file, pic_type, cur_bits, nvb_bits, wp_method,
      p_Vid->AverageFrameQP, lambda, 
      mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
      tmp_time, (int) p_Vid->me_time,
      p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
      p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);

  }
}

static void ReportVerboseFDN(VideoParameters *p_Vid, char *pic_type, int cur_bits, int fdn_bits, int nvb_bits, int wp_method, int lambda, DistMetric *mPSNR, int tmp_time, int direct_mode)
{
#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->p_Inp->num_of_views == 2 )
  {
    if ( p_Vid->fld_flag && p_Vid->view_id == 0 )
    {
      p_Vid->prev_cs.frm_no_in_file = p_Vid->frm_no_in_file;
      p_Vid->prev_cs.view_id = p_Vid->view_id;
      p_Vid->prev_cs.cur_bits = cur_bits;
      strcpy( p_Vid->prev_cs.pic_type, pic_type );
      p_Vid->prev_cs.AverageFrameQP = p_Vid->AverageFrameQP;
      p_Vid->prev_cs.lambda = 0;
      p_Vid->prev_cs.psnr_value[0] = mPSNR->value[0];
      p_Vid->prev_cs.psnr_value[1] = mPSNR->value[1];
      p_Vid->prev_cs.psnr_value[2] = mPSNR->value[2];
      p_Vid->prev_cs.ssim_value[0] = 0;
      p_Vid->prev_cs.ssim_value[1] = 0;
      p_Vid->prev_cs.ssim_value[2] = 0;
      p_Vid->prev_cs.tmp_time = tmp_time;
      p_Vid->prev_cs.me_time = (int)p_Vid->me_time;
      p_Vid->prev_cs.fld_flag = p_Vid->fld_flag;
      p_Vid->prev_cs.intras = p_Vid->intras;
      p_Vid->prev_cs.direct_mode = direct_mode;
      p_Vid->prev_cs.num_ref_idx_l0_active = p_Vid->num_ref_idx_l0_active;
      p_Vid->prev_cs.num_ref_idx_l1_active = p_Vid->num_ref_idx_l1_active;
      p_Vid->prev_cs.rd_pass = p_Vid->rd_pass;
      p_Vid->prev_cs.nal_reference_idc = p_Vid->nal_reference_idc;
    }
    else if ( p_Vid->fld_flag && p_Vid->view_id == 1 )
    {
      printf ("%05d(%3s)  %1d  %8d %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
        p_Vid->prev_cs.frm_no_in_file, p_Vid->prev_cs.pic_type, p_Vid->prev_cs.view_id,
        (int)(p_Vid->p_Stats->bit_ctr_v[0] - p_Vid->p_Stats->bit_ctr_n_v[0]) 
        + (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[0] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[0]) + p_Vid->p_Stats->bit_ctr_parametersets_n_v[0], 
        (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[0] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[0]), 
        p_Vid->p_Stats->bit_ctr_parametersets_n_v[0],
        p_Vid->prev_cs.wp_method, p_Vid->prev_cs.AverageFrameQP, p_Vid->prev_cs.lambda, 
        p_Vid->prev_cs.psnr_value[0], p_Vid->prev_cs.psnr_value[1], p_Vid->prev_cs.psnr_value[2], 
        p_Vid->prev_cs.tmp_time, (int) p_Vid->prev_cs.me_time,
        p_Vid->prev_cs.fld_flag ? "FLD" : "FRM", p_Vid->prev_cs.intras, p_Vid->prev_cs.direct_mode,
        p_Vid->prev_cs.num_ref_idx_l0_active, p_Vid->prev_cs.num_ref_idx_l1_active, p_Vid->prev_cs.rd_pass, p_Vid->prev_cs.nal_reference_idc);
      printf ("%05d(%3s)  %1d  %8d %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
        p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, 
        (int)(p_Vid->p_Stats->bit_ctr_v[1] - p_Vid->p_Stats->bit_ctr_n_v[1]) 
        + (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[1] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[1]) + p_Vid->p_Stats->bit_ctr_parametersets_n_v[1], 
        (int)(p_Vid->p_Stats->bit_ctr_filler_data_v[1] - p_Vid->p_Stats->bit_ctr_filler_data_n_v[1]), 
        p_Vid->p_Stats->bit_ctr_parametersets_n_v[1], 
        wp_method,
        p_Vid->AverageFrameQP, lambda, 
        mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
        tmp_time, (int) p_Vid->me_time,
        p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
        p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
    }
    else
    {
      printf ("%05d(%3s)  %1d  %8d %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
        p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, cur_bits, fdn_bits, nvb_bits, wp_method,
        p_Vid->AverageFrameQP, lambda, 
        mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
        tmp_time, (int) p_Vid->me_time,
        p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
        p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
    }
  }
  else
#endif
  {
  printf ("%05d(%3s)%8d %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
    p_Vid->frm_no_in_file, pic_type, cur_bits, fdn_bits, nvb_bits, wp_method,
    p_Vid->AverageFrameQP, lambda, 
    mPSNR->value[0], mPSNR->value[1], mPSNR->value[2],     
    tmp_time, (int) p_Vid->me_time,
    p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
    p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
  }
}

static void ReportVerboseSSIM(VideoParameters *p_Vid, char *pic_type, int cur_bits, int wp_method, int lambda, DistMetric *mPSNR, DistMetric *mSSIM,int tmp_time, int direct_mode)
{
#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->p_Inp->num_of_views == 2 )
  {
    printf ("%05d(%3s)  %1d  %8d %1d %2d %4d %7.3f %7.3f %7.3f %7.4f %7.4f %7.4f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
      p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, cur_bits, wp_method,
      p_Vid->AverageFrameQP, lambda, 
      mPSNR->value[0], mPSNR->value[1], mPSNR->value[2], 
      mSSIM->value[0], mSSIM->value[1], mSSIM->value[2], 
      tmp_time, (int) p_Vid->me_time,
      p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
      p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active,p_Vid->rd_pass, p_Vid->nal_reference_idc);
  }
  else
#endif
  {
  printf ("%05d(%3s)%8d %1d %2d %4d %7.3f %7.3f %7.3f %7.4f %7.4f %7.4f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
    p_Vid->frm_no_in_file, pic_type, cur_bits, wp_method,
    p_Vid->AverageFrameQP, lambda, 
    mPSNR->value[0], mPSNR->value[1], mPSNR->value[2], 
    mSSIM->value[0], mSSIM->value[1], mSSIM->value[2], 
    tmp_time, (int) p_Vid->me_time,
    p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
    p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active,p_Vid->rd_pass, p_Vid->nal_reference_idc);
  }
}

static void ReportVerboseNVBSSIM(VideoParameters *p_Vid, char *pic_type, int cur_bits, int nvb_bits, int wp_method, int lambda, DistMetric *mPSNR, DistMetric *mSSIM,int tmp_time, int direct_mode)
{
#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->p_Inp->num_of_views == 2 )
  {
    printf ("%05d(%3s)  %1d  %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %7.4f %7.4f %7.4f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
      p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, cur_bits, nvb_bits, wp_method,
      p_Vid->AverageFrameQP, lambda, 
      mPSNR->value[0], mPSNR->value[1], mPSNR->value[2], 
      mSSIM->value[0], mSSIM->value[1], mSSIM->value[2], 
      tmp_time, (int) p_Vid->me_time,
      p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
      p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
  }
  else
#endif
  {
  printf ("%05d(%3s)%8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %7.4f %7.4f %7.4f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
    p_Vid->frm_no_in_file, pic_type, cur_bits, nvb_bits, wp_method,
    p_Vid->AverageFrameQP, lambda, 
    mPSNR->value[0], mPSNR->value[1], mPSNR->value[2], 
    mSSIM->value[0], mSSIM->value[1], mSSIM->value[2], 
    tmp_time, (int) p_Vid->me_time,
    p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
    p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
  }
}

static void ReportVerboseFDNSSIM(VideoParameters *p_Vid, char *pic_type, int cur_bits, int fdn_bits, int nvb_bits, int wp_method, int lambda, DistMetric *mPSNR, DistMetric *mSSIM,int tmp_time, int direct_mode)
{
#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->p_Inp->num_of_views == 2 )
  {
    printf ("%05d(%3s)  %1d  %8d %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %7.4f %7.4f %7.4f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
      p_Vid->frm_no_in_file, pic_type, p_Vid->view_id, cur_bits, fdn_bits, nvb_bits, wp_method,
      p_Vid->AverageFrameQP, lambda, 
      mPSNR->value[0], mPSNR->value[1], mPSNR->value[2], 
      mSSIM->value[0], mSSIM->value[1], mSSIM->value[2], 
      tmp_time, (int) p_Vid->me_time,
      p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
      p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
  }
  else
#endif
  {
  printf ("%05d(%3s)%8d %8d %3d  %1d %2d %4d %7.3f %7.3f %7.3f %7.4f %7.4f %7.4f %9d %7d    %3s %5d %1d %2d %2d  %d   %d\n",
    p_Vid->frm_no_in_file, pic_type, cur_bits, fdn_bits, nvb_bits, wp_method,
    p_Vid->AverageFrameQP, lambda, 
    mPSNR->value[0], mPSNR->value[1], mPSNR->value[2], 
    mSSIM->value[0], mSSIM->value[1], mSSIM->value[2], 
    tmp_time, (int) p_Vid->me_time,
    p_Vid->fld_flag ? "FLD" : "FRM", p_Vid->intras, direct_mode,
    p_Vid->num_ref_idx_l0_active, p_Vid->num_ref_idx_l1_active, p_Vid->rd_pass, p_Vid->nal_reference_idc);
  }
}


static void ReportNALNonVLCBits(VideoParameters *p_Vid, int64 tmp_time)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  StatParameters *p_Stats = p_Vid->p_Stats;

  //! Need to add type (i.e. SPS, PPS, SEI etc).
#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->num_of_views == 2)
  {
    if (p_Inp->Verbose != 0)
      printf ("%05d(NVB)     %8d \n", p_Vid->frame_no, p_Stats->bit_ctr_parametersets_n);
  }
  else
#endif
  {
    if (p_Inp->Verbose != 0)
      printf ("%05d(NVB)%8d \n", p_Vid->frame_no, p_Stats->bit_ctr_parametersets_n);
  }
}

static void ReportFirstframe(VideoParameters *p_Vid, int64 tmp_time)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  StatParameters *stats = p_Vid->p_Stats;

  int cur_bits = (int)(stats->bit_ctr - stats->bit_ctr_n)
    + (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n);

  if (p_Inp->Verbose == 1)
  {
    ReportSimple(p_Vid, "IDR", cur_bits, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time);
  }
  else if (p_Inp->Verbose == 2)
  {
    int lambda = (int) p_Vid->lambda_me[I_SLICE][p_Vid->masterQP][F_PEL];
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseSSIM(p_Vid, "IDR", cur_bits, 0, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    else
      ReportVerbose(p_Vid, "IDR", cur_bits, 0, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
  }
  else if (p_Inp->Verbose == 3)
  {
    int lambda = (int) p_Vid->lambda_me[I_SLICE][p_Vid->masterQP][F_PEL];
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseNVBSSIM(p_Vid, "IDR", cur_bits + stats->bit_ctr_parametersets_n, stats->bit_ctr_parametersets_n, 0, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    else
      ReportVerboseNVB(p_Vid, "IDR", cur_bits + stats->bit_ctr_parametersets_n, stats->bit_ctr_parametersets_n, 0, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
  }
  else if (p_Inp->Verbose == 4)
  {
    int lambda = (int) p_Vid->lambda_me[I_SLICE][p_Vid->masterQP][F_PEL];
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseFDNSSIM(p_Vid, "IDR", cur_bits + stats->bit_ctr_parametersets_n, (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n), stats->bit_ctr_parametersets_n, 0, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    else
      ReportVerboseFDN(p_Vid, "IDR", cur_bits + stats->bit_ctr_parametersets_n, (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n), stats->bit_ctr_parametersets_n, 0, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
  }

  stats->bit_counter[I_SLICE] = stats->bit_ctr;
#if (MVC_EXTENSION_ENABLE)
  if ( p_Inp->num_of_views == 2 )
  {
    if ( !p_Vid->fld_flag )
    {
      stats->bit_counter_v[p_Vid->view_id][I_SLICE] = stats->bit_ctr;
    }
    stats->bit_ctr_v[0] = 0;
    stats->bit_ctr_v[1] = 0;
  }
#endif
  stats->bit_ctr = 0;
}

static void ReportI(VideoParameters *p_Vid, int64 tmp_time)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  StatParameters *stats = p_Vid->p_Stats;
  char pic_type[4];
  int  cur_bits = (int)(stats->bit_ctr - stats->bit_ctr_n)
    + (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n);

  if ((p_Inp->redundant_pic_flag == 0) || !p_Vid->redundant_coding )
  {
    if (p_Vid->currentPicture->idr_flag == TRUE)
      strcpy(pic_type,"IDR");
    else if ( p_Vid->type == SI_SLICE )
      strcpy(pic_type,"SI ");
    else
      strcpy(pic_type," I ");
  }
  else
    strcpy(pic_type,"R");

  if (p_Inp->Verbose == 1)
  {
    ReportSimple(p_Vid, pic_type, cur_bits, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time);
  }
  else if (p_Inp->Verbose == 2)
  {
    int lambda = (int) p_Vid->lambda_me[I_SLICE][p_Vid->masterQP][F_PEL];
    if (p_Inp->Distortion[SSIM] == 1)
    {
      ReportVerboseSSIM(p_Vid, pic_type, cur_bits, 0, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    }
    else
    {
      ReportVerbose(p_Vid, pic_type, cur_bits, 0, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
    }
  }
  else if (p_Inp->Verbose == 3)
  {
    int lambda = (int) p_Vid->lambda_me[I_SLICE][p_Vid->masterQP][F_PEL];
    if (p_Inp->Distortion[SSIM] == 1)
    {
      ReportVerboseNVBSSIM(p_Vid, pic_type, cur_bits + stats->bit_ctr_parametersets_n, stats->bit_ctr_parametersets_n, 0, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    }
    else
    {
      ReportVerboseNVB(p_Vid, pic_type, cur_bits + stats->bit_ctr_parametersets_n, stats->bit_ctr_parametersets_n, 0, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
    }
  }
  else if (p_Inp->Verbose == 4)
  {
    int lambda = (int) p_Vid->lambda_me[I_SLICE][p_Vid->masterQP][F_PEL];
    if (p_Inp->Distortion[SSIM] == 1)
    {
      ReportVerboseFDNSSIM(p_Vid, pic_type, cur_bits + stats->bit_ctr_parametersets_n, (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n), stats->bit_ctr_parametersets_n, 0, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    }
    else
    {
      ReportVerboseFDN(p_Vid, pic_type, cur_bits + stats->bit_ctr_parametersets_n, (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n), stats->bit_ctr_parametersets_n, 0, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
    }
  }
}

static void ReportB(VideoParameters *p_Vid, int64 tmp_time)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  StatParameters *stats = p_Vid->p_Stats;

  int cur_bits = (int)(stats->bit_ctr - stats->bit_ctr_n)
    + (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n);

  if (p_Inp->Verbose == 1)
  {
    ReportSimple(p_Vid, " B ", cur_bits, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time);
  }
  else if (p_Inp->Verbose == 2)
  {
    int lambda = (int) p_Vid->lambda_me[B_SLICE][p_Vid->masterQP][F_PEL];    
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseSSIM(p_Vid, " B ", cur_bits, p_Vid->active_pps->weighted_bipred_idc, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, p_Vid->direct_spatial_mv_pred_flag);
    else
      ReportVerbose(p_Vid, " B ", cur_bits, p_Vid->active_pps->weighted_bipred_idc, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, p_Vid->direct_spatial_mv_pred_flag);
  }
  else if (p_Inp->Verbose == 3)
  {
    int lambda = (int) p_Vid->lambda_me[B_SLICE][p_Vid->masterQP][F_PEL];    
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseNVBSSIM(p_Vid, " B ", cur_bits + stats->bit_ctr_parametersets_n, stats->bit_ctr_parametersets_n, p_Vid->active_pps->weighted_bipred_idc, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, p_Vid->direct_spatial_mv_pred_flag);
    else
      ReportVerboseNVB(p_Vid, " B ", cur_bits + stats->bit_ctr_parametersets_n, stats->bit_ctr_parametersets_n, p_Vid->active_pps->weighted_bipred_idc, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, p_Vid->direct_spatial_mv_pred_flag);
  }
  else if (p_Inp->Verbose == 4)
  {
    int lambda = (int) p_Vid->lambda_me[B_SLICE][p_Vid->masterQP][F_PEL];    
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseFDNSSIM(p_Vid, " B ", cur_bits + stats->bit_ctr_parametersets_n, (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n), stats->bit_ctr_parametersets_n, p_Vid->active_pps->weighted_bipred_idc, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, p_Vid->direct_spatial_mv_pred_flag);
    else
      ReportVerboseFDN(p_Vid, " B ", cur_bits + stats->bit_ctr_parametersets_n, (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n), stats->bit_ctr_parametersets_n, p_Vid->active_pps->weighted_bipred_idc, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, p_Vid->direct_spatial_mv_pred_flag);
  }
}

static void ReportP(VideoParameters *p_Vid, int64 tmp_time)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  StatParameters *stats = p_Vid->p_Stats;

  char pic_type[4];
  int  cur_bits = (int)(stats->bit_ctr - stats->bit_ctr_n)
    + (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n);

  if (p_Vid->type == SP_SLICE)
    strcpy(pic_type,"SP ");
  else if ((p_Inp->redundant_pic_flag == 0) || !p_Vid->redundant_coding )
    strcpy(pic_type," P ");
  else
    strcpy(pic_type," R ");

  if (p_Inp->Verbose == 1)
  {
    ReportSimple(p_Vid, pic_type, cur_bits, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time);
  }
  else if (p_Inp->Verbose == 2)
  {
    int lambda = (int) p_Vid->lambda_me[P_SLICE][p_Vid->masterQP][F_PEL];    
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseSSIM(p_Vid, pic_type, cur_bits, p_Vid->active_pps->weighted_pred_flag, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    else
      ReportVerbose(p_Vid, pic_type, cur_bits, p_Vid->active_pps->weighted_pred_flag, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
  }
  else if (p_Inp->Verbose == 3)
  {
    int lambda = (int) p_Vid->lambda_me[P_SLICE][p_Vid->masterQP][F_PEL];        
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseNVBSSIM(p_Vid, pic_type, cur_bits + stats->bit_ctr_parametersets_n, stats->bit_ctr_parametersets_n, p_Vid->active_pps->weighted_pred_flag, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    else
      ReportVerboseNVB(p_Vid, pic_type, cur_bits + stats->bit_ctr_parametersets_n, stats->bit_ctr_parametersets_n, p_Vid->active_pps->weighted_pred_flag, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
  }
  else if (p_Inp->Verbose == 4)
  {
    int lambda = (int) p_Vid->lambda_me[P_SLICE][p_Vid->masterQP][F_PEL];    
    if (p_Inp->Distortion[SSIM] == 1)
      ReportVerboseFDNSSIM(p_Vid, pic_type, cur_bits + stats->bit_ctr_parametersets_n, (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n), stats->bit_ctr_parametersets_n, p_Vid->active_pps->weighted_pred_flag, lambda, &p_Vid->p_Dist->metric[PSNR], &p_Vid->p_Dist->metric[SSIM], (int) tmp_time, 0);
    else
      ReportVerboseFDN(p_Vid, pic_type, cur_bits + stats->bit_ctr_parametersets_n, (int)(stats->bit_ctr_filler_data - stats->bit_ctr_filler_data_n), stats->bit_ctr_parametersets_n, p_Vid->active_pps->weighted_pred_flag, lambda, &p_Vid->p_Dist->metric[PSNR], (int) tmp_time, 0);
  }
}

/*!
 ************************************************************************
 * \brief
 *    point to frame coding variables
 ************************************************************************
 */
static void put_buffer_frame(VideoParameters *p_Vid)
{
  p_Vid->pCurImg    = p_Vid->imgData.frm_data[0];
  p_Vid->pImgOrg[0] = p_Vid->imgData.frm_data[0];    
  
  if (p_Vid->yuv_format != YUV400)
  {
    p_Vid->pImgOrg[1] = p_Vid->imgData.frm_data[1];
    p_Vid->pImgOrg[2] = p_Vid->imgData.frm_data[2];
  }

  if (p_Vid->p_Inp->MDReference[0] || p_Vid->p_Inp->MDReference[1])
  {
    p_Vid->pCurImgRef    = p_Vid->imgRefData.frm_data[0];
    p_Vid->pImgOrgRef[0] = p_Vid->imgRefData.frm_data[0];    
    
    if (p_Vid->yuv_format != YUV400)
    {
      p_Vid->pImgOrgRef[1] = p_Vid->imgRefData.frm_data[1];
      p_Vid->pImgOrgRef[2] = p_Vid->imgRefData.frm_data[2];
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    point to top field coding variables
 ************************************************************************
 */
static void put_buffer_top(VideoParameters *p_Vid)
{
  p_Vid->fld_type = 0;

  p_Vid->pCurImg    = p_Vid->imgData.top_data[0];
  p_Vid->pImgOrg[0] = p_Vid->imgData.top_data[0];
  
  if (p_Vid->yuv_format != YUV400)
  {
    p_Vid->pImgOrg[1] = p_Vid->imgData.top_data[1];
    p_Vid->pImgOrg[2] = p_Vid->imgData.top_data[2];
  }
  
  if ( p_Vid->p_Inp->MDReference[0] || p_Vid->p_Inp->MDReference[1] )
  {
    p_Vid->pCurImgRef    = p_Vid->imgRefData.top_data[0];
    p_Vid->pImgOrgRef[0] = p_Vid->imgRefData.top_data[0];
    
    if (p_Vid->yuv_format != YUV400)
    {
      p_Vid->pImgOrgRef[1] = p_Vid->imgRefData.top_data[1];
      p_Vid->pImgOrgRef[2] = p_Vid->imgRefData.top_data[2];
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    point to bottom field coding variables
 ************************************************************************
 */
static void put_buffer_bot(VideoParameters *p_Vid)
{
  p_Vid->fld_type = 1;

  p_Vid->pCurImg    = p_Vid->imgData.bot_data[0];  
  p_Vid->pImgOrg[0] = p_Vid->imgData.bot_data[0];
  
  if (p_Vid->yuv_format != YUV400)
  {
    p_Vid->pImgOrg[1] = p_Vid->imgData.bot_data[1];
    p_Vid->pImgOrg[2] = p_Vid->imgData.bot_data[2];
  }

  if ( p_Vid->p_Inp->MDReference[0] || p_Vid->p_Inp->MDReference[1] )
  {
    p_Vid->pCurImgRef    = p_Vid->imgRefData.bot_data[0];
    p_Vid->pImgOrgRef[0] = p_Vid->imgRefData.bot_data[0];
    
    if (p_Vid->yuv_format != YUV400)
    {
      p_Vid->pImgOrgRef[1] = p_Vid->imgRefData.bot_data[1];
      p_Vid->pImgOrgRef[2] = p_Vid->imgRefData.bot_data[2];
    }
  }
}

/*!
*************************************************************************************
* Brief
*     Output SP frames coefficients
*************************************************************************************
*/
void output_SP_coefficients(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int i,k;
  FILE *SP_coeff_file;
  int ret;
  if(p_Vid->number_sp2_frames==0)
  {
    if ((SP_coeff_file = fopen(p_Inp->sp_output_filename,"wb")) == NULL)
    {
      printf ("Fatal: cannot open SP output file '%s', exit (-1)\n", p_Inp->sp_output_filename);
      exit (-1);
    }
    p_Vid->number_sp2_frames++;
  }
  else
  {
    if ((SP_coeff_file = fopen(p_Inp->sp_output_filename,"ab")) == NULL)
    {
      printf ("Fatal: cannot open SP output file '%s', exit (-1)\n", p_Inp->sp_output_filename);
      exit (-1);
    }
  }

  for(i=0;i<p_Vid->height;i++)
  {
    ret = (int) fwrite(p_Vid->lrec[i], sizeof(int), p_Vid->width, SP_coeff_file);
    if (ret != p_Vid->width)
    {
      error ("cannot write to SP output file", -1);
    }
  }

  for(k=0;k<2;k++)
  {
    for(i=0;i<p_Vid->height_cr;i++)
    {
      ret = (int) fwrite(p_Vid->lrec_uv[k][i], sizeof(int), p_Vid->width_cr, SP_coeff_file);
      if (ret != p_Vid->width_cr)
      {
        error ("cannot write to SP output file", -1);
      }
    }
  }
  fclose(SP_coeff_file);
}

/*!
*************************************************************************************
* Brief
*     Read SP frames coefficients
*************************************************************************************
*/
void read_SP_coefficients(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int i,k;
  FILE *SP_coeff_file;

  if ( p_Inp->sp_switch_period > 0 && ( p_Vid->number_sp2_frames % (p_Inp->sp_switch_period << 1) ) >= p_Inp->sp_switch_period )
  {
    if ((SP_coeff_file = fopen(p_Inp->sp2_input_filename1,"rb")) == NULL)
    {
      printf ("Fatal: cannot open SP input file '%s', exit (-1)\n", p_Inp->sp2_input_filename2);
      exit (-1);
    }
  }
  else
  {
    if ((SP_coeff_file = fopen(p_Inp->sp2_input_filename2,"rb")) == NULL)
    {
      printf ("Fatal: cannot open SP input file '%s', exit (-1)\n", p_Inp->sp2_input_filename1);
      exit (-1);
    }
  }

  if (0 != fseek (SP_coeff_file, p_Vid->size * 3/2*p_Vid->number_sp2_frames*sizeof(int), SEEK_SET))
  {
    printf ("Fatal: cannot seek in SP input file, exit (-1)\n");
    exit (-1);
  }
  p_Vid->number_sp2_frames++;

  for(i=0;i<p_Vid->height;i++)
  {
    if(p_Vid->width!=(int)fread(p_Vid->lrec[i],sizeof(int),p_Vid->width,SP_coeff_file))
    {
      printf ("Fatal: cannot read in SP input file, exit (-1)\n");
      exit (-1);
    }
  }

  for(k=0;k<2;k++)
  {
    for(i=0;i<p_Vid->height_cr;i++)
    {
      if(p_Vid->width_cr!=(int)fread(p_Vid->lrec_uv[k][i],sizeof(int),p_Vid->width_cr,SP_coeff_file))
      {
        printf ("Fatal: cannot read in SP input file, exit (-1)\n");
        exit (-1);
      }
    }
  }
  fclose(SP_coeff_file);
}


/*!
*************************************************************************************
* Brief
*     Select appropriate image plane (for 444 coding)
*************************************************************************************
*/
void select_plane(VideoParameters *p_Vid, ColorPlane color_plane)
{
  p_Vid->pCurImg              = p_Vid->pImgOrg[color_plane];
  p_Vid->enc_picture->p_curr_img     = p_Vid->enc_picture->p_img[color_plane];
  p_Vid->enc_picture->p_curr_img_sub = p_Vid->enc_picture->p_img_sub[color_plane];
  p_Vid->max_imgpel_value     = (short) p_Vid->max_pel_value_comp[color_plane];
  p_Vid->dc_pred_value        = p_Vid->dc_pred_value_comp[color_plane];
}

/*!
*************************************************************************************
* Brief
*     Is this picture the first access unit in this GOP?
*************************************************************************************
*/
static int is_gop_first_unit(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  return ( get_idr_flag(p_Vid) || ( p_Vid->type == I_SLICE && p_Inp->EnableOpenGOP && get_random_access_flag(p_Vid) ) );
}

/*!
*************************************************************************************
* Brief
*     AUD, SPS, PPS, and SEI messages
*************************************************************************************
*/
void write_non_vcl_nalu( VideoParameters *p_Vid )
{
  InputParameters *p_Inp = p_Vid->p_Inp;

  // SPS + PPS + AUD
  if (p_Inp->ResendSPS == 3 && is_gop_first_unit(p_Vid, p_Inp) && p_Vid->number)
  {
    p_Vid->p_Stats->bit_slice = rewrite_paramsets(p_Vid);
  }
  else if (p_Inp->ResendSPS == 2 && get_idr_flag(p_Vid) && p_Vid->number)
  {
    p_Vid->p_Stats->bit_slice = rewrite_paramsets(p_Vid);
  }
  else if (p_Inp->ResendSPS == 1 && p_Vid->type == I_SLICE && p_Vid->curr_frm_idx != 0)
  {
    p_Vid->p_Stats->bit_slice = rewrite_paramsets(p_Vid);
  }
  // PPS + AUD
  else if ( p_Inp->ResendPPS && p_Vid->curr_frm_idx != 0 )
  {
    int i, total_pps = p_Inp->GenerateMultiplePPS ? 3 : 1;

    p_Vid->p_Stats->bit_ctr_parametersets_n = 0;
    // Access Unit Delimiter NALU
    if ( p_Inp->SendAUD )
    {
      p_Vid->p_Stats->bit_ctr_parametersets_n = Write_AUD_NALU(p_Vid);
    }
    for ( i = 0; i < total_pps; i++ )
    {
      p_Vid->p_Stats->bit_ctr_parametersets_n += write_PPS(p_Vid, 0, i);
    }
  }
  // Access Unit Delimiter NALU
  else if ( p_Inp->SendAUD && p_Vid->curr_frm_idx != 0 )
  {
    p_Vid->p_Stats->bit_ctr_parametersets_n += Write_AUD_NALU(p_Vid);
  }

  UpdateSubseqInfo (p_Vid, p_Inp, p_Vid->layer);        // Tian Dong (Sept 2002)
  UpdateSceneInformation (p_Vid->p_SEI, FALSE, 0, 0, -1); // JVT-D099, scene information SEI, nothing included by default
  UpdateFramePackingArrangement (p_Vid, p_Inp);

  //! Commented out by StW, needs fixing in SEI.h to keep the trace file clean
  //  PrepareAggregationSEIMessage (p_Vid);

  // write tone mapping SEI message
  if (p_Inp->ToneMappingSEIPresentFlag)
  {
    UpdateToneMapping(p_Vid->p_SEI);
  }

  // pic timing
  ClearPicTiming(p_Vid->p_SEI);
  if ( p_Inp->SEIVUI32Pulldown )
  {
    UpdatePicTiming(p_Vid, p_Inp);
    PrepareAggregationSEIMessage(p_Vid);
    p_Vid->p_Stats->bit_ctr_parametersets_n += Write_SEI_NALU(p_Vid, 0);
    ClearPicTiming(p_Vid->p_SEI);
    // clear message
    clear_sei_message(p_Vid->p_SEI, AGGREGATION_SEI);
  }
  else
  {
    PrepareAggregationSEIMessage(p_Vid);
  }
  
  p_Vid->p_Stats->bit_ctr_parametersets_n += Write_SEI_NALU(p_Vid, 0);

  // update seq NVB counter
  p_Vid->p_Stats->bit_ctr_parametersets   += p_Vid->p_Stats->bit_ctr_parametersets_n;
}

/*!
*************************************************************************************
* Brief
*     AUD, SPS, PPS, and SEI messages for the bottom field picture
*************************************************************************************
*/
void write_non_vcl_nalu_bot_fld( VideoParameters *p_Vid )
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int init_param_set_bits = p_Vid->p_Stats->bit_ctr_parametersets_n;

  // SPS + PPS + AUD
  if (p_Inp->ResendSPS == 1 && p_Vid->type == I_SLICE)
  {
    p_Vid->p_Stats->bit_slice = rewrite_paramsets(p_Vid);
  }
  // PPS + AUD
  else if ( p_Inp->ResendPPS )
  {
    int i, total_pps = p_Inp->GenerateMultiplePPS ? 3 : 1;

    p_Vid->p_Stats->bit_ctr_parametersets_n = 0;
    // Access Unit Delimiter NALU
    if ( p_Inp->SendAUD )
    {
      p_Vid->p_Stats->bit_ctr_parametersets_n = Write_AUD_NALU(p_Vid);
    }
    for ( i = 0; i < total_pps; i++ )
    {
      p_Vid->p_Stats->bit_ctr_parametersets_n += write_PPS(p_Vid, 0, i);
    }
  }
  // Access Unit Delimiter NALU
  else if ( p_Inp->SendAUD )
  {
    p_Vid->p_Stats->bit_ctr_parametersets_n += Write_AUD_NALU(p_Vid);
  }

  UpdateSubseqInfo (p_Vid, p_Inp, p_Vid->layer);        // Tian Dong (Sept 2002)
  UpdateSceneInformation (p_Vid->p_SEI, FALSE, 0, 0, -1); // JVT-D099, scene information SEI, nothing included by default
  UpdateFramePackingArrangement (p_Vid, p_Inp);

  //! Commented out by StW, needs fixing in SEI.h to keep the trace file clean
  //  PrepareAggregationSEIMessage (p_Vid);

  // write tone mapping SEI message
  if (p_Inp->ToneMappingSEIPresentFlag)
  {
    UpdateToneMapping(p_Vid->p_SEI);
  }

  PrepareAggregationSEIMessage(p_Vid);
  
  p_Vid->p_Stats->bit_ctr_parametersets_n += Write_SEI_NALU(p_Vid, 0);

  // update seq NVB counter
  p_Vid->p_Stats->bit_ctr_parametersets   += (p_Vid->p_Stats->bit_ctr_parametersets_n - init_param_set_bits);
}

void OtfCompatibility_copyWithPadding ( imgpel **dstImg, imgpel **srcImg, int size_x, int size_y, int padding_x, int padding_y )
{
  int i, j;
  int size_x_minus1 = size_x - 1;

  imgpel *wBufSrc, *wBufDst;

  // Copy top line
  wBufDst = &( dstImg[-padding_y][-padding_x] ); 
  wBufSrc = srcImg[0];
  // left IMG_PAD_SIZE
  for (i = 0; i < padding_x; ++i)
    *(wBufDst++) = wBufSrc[0];
  // center 0-(size_x)
  memcpy(wBufDst, wBufSrc, size_x * sizeof(imgpel));
  wBufDst += size_x;
  // right IMG_PAD_SIZE
  for (i = 0; i < padding_x; ++i)
    *(wBufDst++) = wBufSrc[size_x_minus1];

  // Now copy remaining pad lines
  for (j = -padding_y+1; j < 0; ++j)
  {
    memcpy(dstImg[j]-padding_x, dstImg[j - 1]-padding_x, (size_x+2*padding_x) * sizeof(imgpel));
  }

  for (j = 0; j < size_y; ++j)
  {    
    wBufDst = &( dstImg[j][-padding_x] ); // 4:4:4 independent mode
    wBufSrc = srcImg[j];
    // left IMG_PAD_SIZE
    for (i = 0; i < padding_x; ++i)
      *(wBufDst++) = wBufSrc[0];
    // center 0-(size_x)
    //memcpy(wBufDst, wBufSrc, size_x * sizeof(imgpel));
    wBufDst += size_x;
    // right IMG_PAD_SIZE
    for (i = 0; i < padding_x; ++i)
      *(wBufDst++) = wBufSrc[size_x_minus1];
  }

  // Replicate bottom pad lines
  for (j = size_y; j < size_y+padding_y; ++j)
  {    
    memcpy(dstImg[j]-padding_x, dstImg[j - 1]-padding_x, (size_x+2*padding_x) * sizeof(imgpel));
  }

}

#if (MVC_EXTENSION_ENABLE)
/*!
*************************************************************************************
* Brief
*     MVC AUD, SPS, PPS, and SEI messages
*************************************************************************************
*/
void write_non_vcl_nalu_mvc( VideoParameters *p_Vid )
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int bits;

  if ( p_Vid->view_id )
  {
    if ( p_Inp->SendAUD == 2 )
    {
      p_Vid->p_Stats->bit_ctr_parametersets_n += bits = Write_AUD_NALU(p_Vid);
      p_Vid->p_Stats->bit_ctr_parametersets_n_v[p_Vid->view_id] += bits;
    }
  }
  else // view_id == 0
  {
    // SPS + PPS + AUD
    if (p_Inp->ResendSPS == 3 && is_gop_first_unit(p_Vid, p_Inp) && p_Vid->number && p_Vid->structure != BOTTOM_FIELD)
    {
      p_Vid->p_Stats->bit_slice = rewrite_paramsets(p_Vid);
    }
    else if (p_Inp->ResendSPS == 2 && get_idr_flag(p_Vid) && p_Vid->number && p_Vid->structure != BOTTOM_FIELD)
    {
      p_Vid->p_Stats->bit_slice = rewrite_paramsets(p_Vid);
    }
    else if (p_Inp->ResendSPS == 1 && p_Vid->type == I_SLICE && p_Vid->curr_frm_idx != 0 && p_Vid->structure != BOTTOM_FIELD)
    {
      p_Vid->p_Stats->bit_slice = rewrite_paramsets(p_Vid);
    }
    // PPS + AUD
    else if ( p_Inp->ResendPPS && p_Vid->curr_frm_idx != 0 )
    {
      int i, total_pps = p_Inp->GenerateMultiplePPS ? 3 : 1;

      p_Vid->p_Stats->bit_ctr_parametersets_n = 0;
      // Access Unit Delimiter NALU
      if ( p_Inp->SendAUD )
      {
        p_Vid->p_Stats->bit_ctr_parametersets_n = bits = Write_AUD_NALU(p_Vid);
        p_Vid->p_Stats->bit_ctr_parametersets_n_v[p_Vid->view_id] += bits;
        for ( i = 0; i < total_pps; i++ )
        {
          p_Vid->p_Stats->bit_ctr_parametersets_n += bits = write_PPS(p_Vid, 0, i);
          p_Vid->p_Stats->bit_ctr_parametersets_n_v[p_Vid->view_id] += bits;
        }
      }
      else
      {
        for ( i = 0; i < total_pps; i++ )
        {
          p_Vid->p_Stats->bit_ctr_parametersets_n += bits = write_PPS(p_Vid, 0, i);
          p_Vid->p_Stats->bit_ctr_parametersets_n_v[p_Vid->view_id] += bits;
        }
      }
    }
    // Access Unit Delimiter NALU
    else if ( p_Inp->SendAUD && (p_Vid->curr_frm_idx != 0 || p_Vid->structure == BOTTOM_FIELD) )
    {
      p_Vid->p_Stats->bit_ctr_parametersets_n += bits = Write_AUD_NALU(p_Vid);
      p_Vid->p_Stats->bit_ctr_parametersets_n_v[p_Vid->view_id] += bits;
    }
  }

  UpdateSubseqInfo (p_Vid, p_Inp, p_Vid->layer);        // Tian Dong (Sept 2002)
  UpdateSceneInformation (p_Vid->p_SEI, FALSE, 0, 0, -1); // JVT-D099, scene information SEI, nothing included by default

  //! Commented out by StW, needs fixing in SEI.h to keep the trace file clean
  //  PrepareAggregationSEIMessage (p_Vid);

  // write tone mapping SEI message
  if (p_Inp->ToneMappingSEIPresentFlag)
  {
    UpdateToneMapping(p_Vid->p_SEI);
  }

  PrepareAggregationSEIMessage(p_Vid);
  
  p_Vid->p_Stats->bit_ctr_parametersets_n += bits = Write_SEI_NALU(p_Vid, 0);
  p_Vid->p_Stats->bit_ctr_parametersets_n_v[p_Vid->view_id] += bits;

  // update seq NVB counter
  p_Vid->p_Stats->bit_ctr_parametersets += p_Vid->p_Stats->bit_ctr_parametersets_n;
}
#endif


