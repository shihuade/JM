/*!
 *************************************************************************************
 * \file macroblock.c
 *
 * \brief
 *    Process one macroblock
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Inge Lille-Langoy               <inge.lille-langoy@telenor.com>
 *    - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *    - Jani Lainema                    <jani.lainema@nokia.com>
 *    - Sebastian Purreiter             <sebastian.purreiter@mch.siemens.de>
 *    - Detlev Marpe
 *    - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *    - Ragip Kurceren                  <ragip.kurceren@nokia.com>
 *    - Alexis Michael Tourapis         <alexismt@ieee.org>
 *************************************************************************************
 */

#include "contributors.h"

#include <limits.h>
#include <math.h>

#include "global.h"
#include "enc_statistics.h"

#include "block.h"
#include "elements.h"
#include "macroblock.h"
#include "memalloc.h"
#include "blk_prediction.h"
#include "mc_prediction.h"
#include "fmo.h"
#include "vlc.h"
#include "image.h"
#include "mb_access.h"
#include "ratectl.h"              // header file for rate control
#include "cabac.h"
#include "biariencode.h"
#include "me_fullsearch.h"
#include "me_fullfast.h"
#include "symbol.h"
#include "conformance.h"
#include "md_common.h"
#include "mv_prediction.h"
#include "rdopt.h"
#include "transform.h"


#if TRACE
#define TRACE_SE(trace,str)  snprintf(trace,TRACESTRING_SIZE,str)
#else
#define TRACE_SE(trace,str)
#endif


static int  slice_too_big                (Slice *currSlice, int rlc_bits);
static int  write_chroma_intra_pred_mode (Macroblock* currMB);
static int  write_chroma_coeff           (Macroblock* currMB);
static int  write_CBP_and_Dquant         (Macroblock* currMB);

 /*!
 ************************************************************************
 * \brief
 *    updates the coordinates for the next macroblock to be processed
 *
 * \param currSlice
 *    current slice pointer
 * \param currMB
 *    macroblock structure to be set
 ************************************************************************
 */
void set_MB_parameters (Slice *currSlice, Macroblock *currMB)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;

  int mb_addr = currMB->mbAddrX;
  p_Vid->current_mb_nr = mb_addr;

  p_Vid->get_mb_block_pos(p_Vid->PicPos, currMB->mbAddrX, &currMB->mb_x, &currMB->mb_y);

  currMB->block_x = currMB->mb_x << 2; 
  currMB->block_y = currMB->mb_y << 2;

  currMB->pix_x   = currMB->block_x << 2;
  currMB->pix_y   = currMB->block_y << 2;
  
  if (currSlice->mb_aff_frame_flag)
  {
    if (currMB->mb_field)
    {
      p_Vid->pCurImg = (mb_addr & 0x01) ? p_Vid->imgData.bot_data[0]  : p_Vid->imgData.top_data[0];
      p_Vid->pImgOrg[0] = p_Vid->pCurImg;

      if ((p_Vid->yuv_format != YUV400) && (p_Inp->separate_colour_plane_flag == 0))
      {
        if (mb_addr & 0x01)
        {
          p_Vid->pImgOrg[1] = p_Vid->imgData.bot_data[1];
          p_Vid->pImgOrg[2] = p_Vid->imgData.bot_data[2];
        }
        else
        {
          p_Vid->pImgOrg[1] = p_Vid->imgData.top_data[1];
          p_Vid->pImgOrg[2] = p_Vid->imgData.top_data[2];
        }
      }

      currMB->opix_y   = (currMB->mb_y >> 1 ) << 4;
      currMB->list_offset = (mb_addr & 0x01) ? 4 : 2;
    }
    else
    {
      p_Vid->pCurImg    = p_Vid->imgData.frm_data[0];      
      p_Vid->pImgOrg[0] = p_Vid->imgData.frm_data[0];

      if ((p_Vid->yuv_format != YUV400) && (p_Inp->separate_colour_plane_flag == 0))
      {
        p_Vid->pImgOrg[1] = p_Vid->imgData.frm_data[1];
        p_Vid->pImgOrg[2] = p_Vid->imgData.frm_data[2];
      }

      currMB->opix_y   = currMB->block_y << 2;
      currMB->list_offset = 0;
    }
  }
  else
  {
    currMB->opix_y   = currMB->block_y << 2;
    currMB->list_offset = 0;
  }

  if (p_Vid->yuv_format != YUV400)
  {
    currMB->pix_c_x  = (p_Vid->mb_cr_size_x * currMB->pix_x) >> 4;
    currMB->pix_c_y  = (p_Vid->mb_cr_size_y * currMB->pix_y) >> 4;

    currMB->opix_c_y = (p_Vid->mb_cr_size_y * currMB->opix_y) >> 4;
  }

  //  printf ("set_MB_parameters: mb %d,  mb_x %d,  mb_y %d\n", mb_addr, currMB->mb_x, currMB->mb_y);
}


/*!
 ************************************************************************
 * \brief
 *    updates the coordinates and statistics parameter for the
 *    next macroblock
 ************************************************************************
 */
void next_macroblock(Macroblock *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  int slice_type = currSlice->slice_type;
  BitCounter *mbBits = &currMB->bits;
  int i;
  StatParameters *cur_stats = &p_Vid->enc_picture->stats;

  if (mbBits->mb_total > p_Vid->max_bitCount)
    printf("Warning!!! Number of bits (%d) of macroblock_layer() data seems to exceed defined limit (%d).\n", mbBits->mb_total,p_Vid->max_bitCount);

  // Update the statistics
  cur_stats->bit_use_mb_type[slice_type]       += mbBits->mb_mode;
  cur_stats->tmp_bit_use_cbp[slice_type]       += mbBits->mb_cbp;
  cur_stats->bit_use_coeffC[slice_type]        += mbBits->mb_uv_coeff;
  cur_stats->bit_use_coeff[0][slice_type]      += mbBits->mb_y_coeff;
  cur_stats->bit_use_coeff[1][slice_type]      += mbBits->mb_cb_coeff;
  cur_stats->bit_use_coeff[2][slice_type]      += mbBits->mb_cr_coeff;
  cur_stats->bit_use_delta_quant[slice_type]   += mbBits->mb_delta_quant;
  cur_stats->bit_use_stuffing_bits[slice_type] += mbBits->mb_stuffing;

  if (is_intra(currMB))
  {
    ++cur_stats->intra_chroma_mode[ (short) currMB->c_ipred_mode];


    if ((currMB->cbp&15) != 0)
    {
      ++cur_stats->mode_use_transform[slice_type][currMB->mb_type][currMB->luma_transform_size_8x8_flag];
    }
  }

  ++cur_stats->mode_use[slice_type][currMB->mb_type];
  cur_stats->bit_use_mode[slice_type][currMB->mb_type] += mbBits->mb_inter;

  if (slice_type != I_SLICE)
  {
    if (currMB->mb_type == P8x8)
    {
      for(i=0;i<4;++i)
      {
        if (currMB->b8x8[i].mode > 0)
          ++cur_stats->mode_use[slice_type][(short) currMB->b8x8[i].mode];
        else
          ++cur_stats->b8_mode_0_use[slice_type][currMB->luma_transform_size_8x8_flag];

        if (currMB->b8x8[i].mode ==4)
        {
          if ((currMB->luma_transform_size_8x8_flag && (currMB->cbp&15) != 0) || p_Inp->Transform8x8Mode == 2)
            ++cur_stats->mode_use_transform[slice_type][4][1];
          else
            ++cur_stats->mode_use_transform[slice_type][4][0];
        }
      }
    }
    else if (currMB->mb_type >= 0 && currMB->mb_type <=3 && ((currMB->cbp&15) != 0))
    {
      ++cur_stats->mode_use_transform[slice_type][currMB->mb_type][currMB->luma_transform_size_8x8_flag];
    }
  }

  // Statistics
  cur_stats->quant[slice_type] += currMB->qp;
  ++cur_stats->num_macroblocks[slice_type];
}

static void set_chroma_qp(Macroblock* currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  int i;
  for (i = 0; i <= 1; ++i)
  {
    currMB->qpc[i] = (short) iClip3 ( -currSlice->bitdepth_chroma_qp_scale, 51, currMB->qp + p_Vid->chroma_qp_offset[i] );
    currMB->qpc[i] = (short) (currMB->qpc[i] < 0 ? currMB->qpc[i] : QP_SCALE_CR[currMB->qpc[i]]);
    currMB->qp_scaled[i + 1] = (short) (currMB->qpc[i] + currSlice->bitdepth_chroma_qp_scale);
  }  
}

/*!
************************************************************************
* \brief
*    updates chroma QP according to luma QP and bit depth
************************************************************************
*/
void update_qp(Macroblock *currMB)
{
  currMB->qp_scaled[PLANE_Y] = (short) (currMB->qp + currMB->p_Slice->bitdepth_luma_qp_scale);
  set_chroma_qp(currMB);

  select_transform(currMB);
}

/*!
 ************************************************************************
 * \brief
 *    resets info for the current macroblock
 *
 * \param currMB
 *    current macroblock
 ************************************************************************
 */
void reset_macroblock(Macroblock *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  PicMotionParams *p_motion;
  int j, i;

  // Reset vectors and reference indices
  for (j=currMB->block_y; j < currMB->block_y + BLOCK_MULTIPLE; ++j)
  {
    for (i = currMB->block_x; i < currMB->block_x + BLOCK_MULTIPLE; ++i)
    {
      p_motion = &motion[j][i];
      p_motion->ref_pic[LIST_0] = NULL;
      p_motion->ref_pic[LIST_1] = NULL;
      p_motion->mv     [LIST_0] = zero_mv;
      p_motion->mv     [LIST_1] = zero_mv;
      p_motion->ref_idx[LIST_0] = -1;
      p_motion->ref_idx[LIST_1] = -1;
    }
  }


  // Reset syntax element entries in MB struct
  currMB->mb_type      = 0;
  currMB->cbp_blk      = 0;
  currMB->cbp          = 0;  
  currMB->c_ipred_mode = DC_PRED_8;

  currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = currSlice->curr_cbp[0] = currSlice->curr_cbp[1] = 0;

  memset(currMB->cbp_bits    , 0, 3 * sizeof(int64));
  memset(currMB->cbp_bits_8x8, 0, 3 * sizeof(int64));

  memset (currMB->mvd, 0, BLOCK_CONTEXT * sizeof(short));
  memset (currMB->intra_pred_modes, DC_PRED, MB_BLOCK_PARTITIONS * sizeof(char)); // changing this to char would allow us to use memset
  memset (currMB->intra_pred_modes8x8, DC_PRED, MB_BLOCK_PARTITIONS * sizeof(char));

  currMB->b8x8[0].bipred = 0;
  currMB->b8x8[1].bipred = 0;
  currMB->b8x8[2].bipred = 0;
  currMB->b8x8[3].bipred = 0;

  currMB->write_mb = FALSE;

  //initialize the whole MB as INTRA coded
  //Blocks are set to notINTRA in write_macroblock
  if (p_Inp->UseConstrainedIntraPred)
  {
    p_Vid->intra_block[currMB->mbAddrX] = 1;
  }

  // Initialize bitcounters for this macroblock
  memset(&currMB->bits, 0, sizeof(BitCounter));

  currMB->min_rdcost = DISTBLK_MAX; 
  currMB->min_dcost  = DISTBLK_MAX;
  currMB->min_rate   = DISTBLK_MAX; 

  currMB->best_mode      = 0;
  currMB->best_c_imode   = 0;
  currMB->best_i16offset = 0;
  currMB->best_cbp       = 0;
}


/*!
 ************************************************************************
 * \brief
 *    initializes the current macroblock
 *
 * \param currSlice
 *    current slice
 * \param currMB
 *    current macroblock
 * \param mb_addr
 *    current macroblock address
 * \param mb_field
 *    true for field macroblock coding
 ************************************************************************
 */
void start_macroblock(Slice *currSlice, Macroblock **currMB, int mb_addr, Boolean mb_field)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int i, mb_qp;
  int use_bitstream_backing = (p_Inp->slice_mode == FIXED_RATE || p_Inp->slice_mode == CALL_BACK);

  static Macroblock *last_coded_mb = NULL;    // used to find whether this is recoding an MB

  DataPartition *dataPart;
  Bitstream *currStream;
  int prev_mb;
  
  *currMB = &p_Vid->mb_data[mb_addr];
  (*currMB)->p_Slice = currSlice;
  (*currMB)->p_Vid = p_Vid;
  (*currMB)->p_Inp = p_Inp;

  (*currMB)->mbAddrX = mb_addr;  

  (*currMB)->is_intra_block = (currSlice->slice_type == I_SLICE || currSlice->slice_type == SI_SLICE) ? TRUE : FALSE;

  (*currMB)->mb_field = (byte) mb_field;
  p_Vid->enc_picture->motion.mb_field[mb_addr] = (byte) mb_field;

  (*currMB)->is_field_mode = (byte) (p_Vid->field_picture || ( currSlice->mb_aff_frame_flag && (*currMB)->mb_field));
  (*currMB)->prev_recode_mb = FALSE;
  (*currMB)->DeblockCall = 0;
  //set buffer;
  (*currMB)->intra4x4_pred = (*currMB)->intra4x4_pred_buf[p_Vid->dpb_layer_id];
  (*currMB)->intra8x8_pred = (*currMB)->intra8x8_pred_buf[p_Vid->dpb_layer_id];
  (*currMB)->intra16x16_pred = (*currMB)->intra16x16_pred_buf[p_Vid->dpb_layer_id];

  set_MB_parameters (currSlice, *currMB);

  prev_mb = FmoGetPreviousMBNr(p_Vid, mb_addr);

  if(use_bitstream_backing)
  {
    if ((!p_Inp->MbInterlace)||((mb_addr & 0x01)==0)) // KS: MB AFF -> store stream positions for 1st MB only
    {
      // Keep the current state of the bitstreams
      if(!p_Vid->cod_counter)
      {
        for (i=0; i<currSlice->max_part_nr; ++i)
        {
          dataPart = &(currSlice->partArr[i]);
          currStream = dataPart->bitstream;
          currStream->stored_bits_to_go = currStream->bits_to_go;
          currStream->stored_byte_pos   = currStream->byte_pos;
          currStream->stored_byte_buf   = currStream->byte_buf;
          p_Vid->p_Stats->stored_bit_slice       = p_Vid->p_Stats->bit_slice;

          if (currSlice->symbol_mode ==CABAC)
          {
            dataPart->ee_recode = dataPart->ee_cabac;
          }
        }
      }
    }
  }

  // Save the slice number of this macroblock. When the macroblock below
  // is coded it will use this to decide if prediction for above is possible
  (*currMB)->slice_nr = currSlice->slice_nr;

  // Initialize delta qp change from last macroblock. Feature may be used for future rate control
  // Rate control
  (*currMB)->qpsp = (short) p_Vid->qpsp;

  if (prev_mb > -1 && (p_Vid->mb_data[prev_mb].slice_nr == currSlice->slice_nr))
  {
    (*currMB)->PrevMB   = &p_Vid->mb_data[prev_mb];
    (*currMB)->prev_qp  = (*currMB)->PrevMB->qp;
    (*currMB)->prev_dqp = (*currMB)->prev_qp - (*currMB)->PrevMB->prev_qp;
  }
  else
  {
    prev_mb = - 1;
    (*currMB)->PrevMB   = NULL;
    (*currMB)->prev_qp  = (short) currSlice->qp;
    (*currMB)->prev_dqp = 0;
  }

  if(p_Inp->RCEnable && (last_coded_mb != *currMB))   //!< avoid decreasing the NumberofbasicUnit for the same MB twice
  {
    mb_qp = rc_handle_mb( *currMB, prev_mb);
  }
  else
  {
    mb_qp = p_Vid->qp;
  }

  last_coded_mb = *currMB;   // save the address of the last coded MB
  
  if ((*currMB)->mbAddrX == 0)
    p_Vid->BasicUnitQP = mb_qp;

  mb_qp = iClip3(-currSlice->bitdepth_luma_qp_scale, 51, mb_qp);
  (*currMB)->qp = (short) mb_qp;
  p_Vid->qp = mb_qp;
  
  update_qp (*currMB);


  // deblocking filter parameter
  if (p_Vid->active_pps->deblocking_filter_control_present_flag)
  {
    (*currMB)->DFDisableIdc    = currSlice->DFDisableIdc;
    (*currMB)->DFAlphaC0Offset = currSlice->DFAlphaC0Offset;
    (*currMB)->DFBetaOffset    = currSlice->DFBetaOffset;
  }
  else
  {
    (*currMB)->DFDisableIdc    = 0;
    (*currMB)->DFAlphaC0Offset = 0;
    (*currMB)->DFBetaOffset    = 0;
  }
  (*currMB)->min_rdcost = DISTBLK_MAX;
  (*currMB)->min_dcost  = DISTBLK_MAX;
  (*currMB)->min_rate   = DISTBLK_MAX;

  (*currMB)->i16mode    = 0;
  (*currMB)->best_mode  = 10;

  if ((p_Vid->yuv_format!=0) && (p_Vid->yuv_format!=3))
  {
    (*currMB)->cbp_linfo_intra = cbp_linfo_intra_normal;
    (*currMB)->cbp_linfo_inter = cbp_linfo_inter_normal;
  }
  else
  {
    (*currMB)->cbp_linfo_intra = cbp_linfo_intra_other;
    (*currMB)->cbp_linfo_inter = cbp_linfo_inter_other;
  }

  // Select appropriate MV predictor function
  if (currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE)
  {
    init_motion_vector_prediction(*currMB, currSlice->mb_aff_frame_flag);
    init_ME_engine(*currMB);
  }

  // If MB is next to a slice boundary, mark neighboring blocks unavailable for prediction
  CheckAvailabilityOfNeighbors(*currMB);

  if (currSlice->symbol_mode == CABAC)
    CheckAvailabilityOfNeighborsCABAC(*currMB);

  reset_macroblock(*currMB);

  if ((p_Inp->SearchMode[p_Vid->view_id] == FAST_FULL_SEARCH) && (!p_Inp->IntraProfile))
    reset_fast_full_search (p_Vid);

  // disable writing of trace file
#if TRACE
  for (i=0; i<currSlice->max_part_nr; ++i )
  {
    currSlice->partArr[i].bitstream->trace_enabled = FALSE;
  }  
#endif
}

/*!
 ************************************************************************
 * \brief
 *    terminates processing of the current macroblock depending
 *    on the chosen slice mode
 ************************************************************************
 */
void end_macroblock(Macroblock *currMB,         //!< Current Macroblock
                    Boolean *end_of_slice,      //!< returns true for last macroblock of a slice, otherwise false
                    Boolean *recode_macroblock  //!< returns true if max. slice size is exceeded an macroblock must be recoded in next slice
                    )
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  int i;
  SyntaxElement se;
  const int *partMap = assignSE2partition[currSlice->partition_mode];
  DataPartition *dataPart;
  Bitstream *currStream;
  int rlc_bits=0;
  int use_bitstream_backing = (p_Inp->slice_mode == FIXED_RATE || p_Inp->slice_mode == CALL_BACK);
  int skip = FALSE;
  int new_slice = 0;

  // if previous mb in the same slice group has different slice number as the current, it's the
  // the start of new slice
  if ((currMB->mbAddrX == 0) || currMB->PrevMB == NULL || ( currMB->PrevMB->slice_nr != currSlice->slice_nr ))
    new_slice=1;

  *recode_macroblock=FALSE;

  switch(p_Inp->slice_mode)
  {
  case NO_SLICES:
    ++currSlice->num_mb;
    *recode_macroblock = FALSE;
    if ((currSlice->num_mb) == (int)p_Vid->PicSizeInMbs) // maximum number of MBs reached
      *end_of_slice = TRUE;

    // if it's end of current slice group, slice ends too
    *end_of_slice = (Boolean) (*end_of_slice | (currMB->mbAddrX == FmoGetLastCodedMBOfSliceGroup (p_Vid, FmoMB2SliceGroup (p_Vid, currMB->mbAddrX))));

    break;
  case FIXED_MB:
    // For slice mode one, check if a new slice boundary follows
    ++currSlice->num_mb;
    *recode_macroblock = FALSE;
    //! Check end-of-slice group condition first
    *end_of_slice = (Boolean) (currMB->mbAddrX == FmoGetLastCodedMBOfSliceGroup (p_Vid, FmoMB2SliceGroup (p_Vid, currMB->mbAddrX)));
    //! Now check maximum # of MBs in slice
    *end_of_slice = (Boolean) (*end_of_slice | (currSlice->num_mb >= p_Inp->slice_argument));

    break;

    // For slice modes two and three, check if coding of this macroblock
    // resulted in too many bits for this slice. If so, indicate slice
    // boundary before this macroblock and code the macroblock again
  case FIXED_RATE:
    // in case of skip MBs check if there is a slice boundary
    // only for CAVLC (p_Vid->cod_counter is always 0 in case of CABAC)
    if(p_Vid->cod_counter)
    {
      // write out the skip MBs to know how many bits we need for the RLC
      se.value1 = p_Vid->cod_counter;
      se.value2 = 0;
      se.type = SE_MBTYPE;
      dataPart = &(currSlice->partArr[partMap[se.type]]);

      TRACE_SE (se.tracestring, "mb_skip_run");
      writeSE_UVLC(&se, dataPart);
      rlc_bits=se.len;

      for (i=0; i<currSlice->max_part_nr; ++i)
      {
        dataPart = &(currSlice->partArr[i]);
        currStream = dataPart->bitstream;
        // save the bitstream as it would be if we write the skip MBs
        currStream->bits_to_go_skip  = currStream->bits_to_go;
        currStream->byte_pos_skip    = currStream->byte_pos;
        currStream->byte_buf_skip    = currStream->byte_buf;
        // restore the bitstream
        currStream->bits_to_go = currStream->stored_bits_to_go;
        currStream->byte_pos   = currStream->stored_byte_pos;
        currStream->byte_buf   = currStream->stored_byte_buf;
      }
      skip = TRUE;
    }
    //! Check if the last coded macroblock fits into the size of the slice
    //! But only if this is not the first macroblock of this slice
    if (!new_slice)
    {
      if(slice_too_big(currSlice, rlc_bits))
      {
        *recode_macroblock = TRUE;
        *end_of_slice      = TRUE;
      }
      else if(!p_Vid->cod_counter)
        skip = FALSE;
    }
    // maximum number of MBs

    // check if current slice group is finished
    if ((*recode_macroblock == FALSE) && (currMB->mbAddrX == FmoGetLastCodedMBOfSliceGroup (p_Vid, FmoMB2SliceGroup (p_Vid, currMB->mbAddrX))))
    {
      *end_of_slice = TRUE;
      if(!p_Vid->cod_counter)
        skip = FALSE;
    }

    //! (first MB OR first MB in a slice) AND bigger that maximum size of slice
    if (new_slice && slice_too_big(currSlice, rlc_bits))
    {
      *end_of_slice = TRUE;
      if(!p_Vid->cod_counter)
        skip = FALSE;
    }
    if (!*recode_macroblock)
      ++currSlice->num_mb;
    break;

  case  CALL_BACK:
    if (currMB->mbAddrX > 0 && !new_slice)
    {
      if (currSlice->slice_too_big(rlc_bits))
      {
        *recode_macroblock = TRUE;
        *end_of_slice = TRUE;
      }
    }

    if ( (*recode_macroblock == FALSE) && (currMB->mbAddrX == FmoGetLastCodedMBOfSliceGroup (p_Vid, FmoMB2SliceGroup (p_Vid, currMB->mbAddrX))))
      *end_of_slice = TRUE;
    break;

  default:
    snprintf(errortext, ET_SIZE, "Slice Mode %d not supported", p_Inp->slice_mode);
    error(errortext, 600);
  }

  if (*recode_macroblock == TRUE)
  {
    // Restore everything
    for (i=0; i<currSlice->max_part_nr; ++i)
    {
      dataPart = &(currSlice->partArr[i]);
      currStream = dataPart->bitstream;
      currStream->bits_to_go = currStream->stored_bits_to_go;
      currStream->byte_pos   = currStream->stored_byte_pos;
      currStream->byte_buf   = currStream->stored_byte_buf;
      p_Vid->p_Stats->bit_slice       = p_Vid->p_Stats->stored_bit_slice;

      if (currSlice->symbol_mode == CABAC)
      {
        dataPart->ee_cabac = dataPart->ee_recode;
      }
    }
  }

  if (currSlice->symbol_mode == CAVLC)
  {
    // Skip MBs at the end of this slice
    dataPart = &(currSlice->partArr[partMap[SE_MBTYPE]]);
    if(*end_of_slice == TRUE  && skip == TRUE)
    {
      // only for Slice Mode 2 or 3
      // If we still have to write the skip, let's do it!
      if(p_Vid->cod_counter && *recode_macroblock == TRUE) // MB that did not fit in this slice
      {
        // If recoding is true and we have had skip,
        // we have to reduce the counter in case of recoding
        p_Vid->cod_counter--;
        if(p_Vid->cod_counter)
        {
          se.value1 = p_Vid->cod_counter;
          se.value2 = 0;
          se.type = SE_MBTYPE;
#if TRACE
          snprintf(se.tracestring, TRACESTRING_SIZE, "Final MB runlength = %3d",p_Vid->cod_counter);
#endif
          writeSE_UVLC(&se, dataPart);
          rlc_bits=se.len;
          currMB->bits.mb_mode = currMB->bits.mb_mode + (unsigned short) rlc_bits;
          p_Vid->cod_counter = 0;
        }
      }
      else //! MB that did not fit in this slice anymore is not a Skip MB
      {
        for (i=0; i<currSlice->max_part_nr; ++i)
        {
          dataPart = &(currSlice->partArr[i]);
          currStream = dataPart->bitstream;
          // update the bitstream
          currStream->bits_to_go = currStream->bits_to_go_skip;
          currStream->byte_pos   = currStream->byte_pos_skip;
          currStream->byte_buf   = currStream->byte_buf_skip;
        }
        // update the statistics
        p_Vid->cod_counter = 0;
        skip = FALSE;
      }
    }

    // Skip MBs at the end of this slice for Slice Mode 0 or 1
    if(*end_of_slice == TRUE && p_Vid->cod_counter && !use_bitstream_backing)
    {
      se.value1 = p_Vid->cod_counter;
      se.value2 = 0;
      se.type = SE_MBTYPE;

      TRACE_SE (se.tracestring, "mb_skip_run");
      writeSE_UVLC(&se, dataPart);

      rlc_bits=se.len;
      currMB->bits.mb_mode = currMB->bits.mb_mode + (unsigned short) rlc_bits;
      p_Vid->cod_counter = 0;
    }
  }
}

/*!
 *****************************************************************************
 *
 * \brief
 *    For Slice Mode 2: Checks if one partition of one slice exceeds the
 *    allowed size
 *
 * \return
 *    FALSE if all Partitions of this slice are smaller than the allowed size
 *    TRUE is at least one Partition exceeds the limit
 *
 * \par Side effects
 *    none
 *
 * \date
 *    4 November 2001
 *
 * \author
 *    Tobias Oelbaum      drehvial@gmx.net
 *****************************************************************************/

int slice_too_big(Slice *currSlice, int rlc_bits)
{
  InputParameters *p_Inp = currSlice->p_Inp;
  DataPartition *dataPart;
  Bitstream *currStream;
  EncodingEnvironmentPtr eep;
  int i;
  int size_in_bytes;

  //! CAVLC
  if (currSlice->symbol_mode == CAVLC)
  {    
    for (i = 0; i < currSlice->max_part_nr; ++i)
    {
      dataPart = &(currSlice->partArr[i]);
      currStream = dataPart->bitstream;
      size_in_bytes = currStream->byte_pos /*- currStream->tmp_byte_pos*/;

      if (currStream->bits_to_go < 8)
        ++size_in_bytes;
      if (currStream->bits_to_go < rlc_bits)
        ++size_in_bytes;
      if(size_in_bytes > p_Inp->slice_argument)
        return TRUE;
    }
  }
  else //! CABAC
  {
    for (i=0; i<currSlice->max_part_nr; ++i)
    {
      dataPart= &(currSlice->partArr[i]);
      eep = &(dataPart->ee_cabac);

      if( arienco_bits_written(eep) > (p_Inp->slice_argument*8))
        return TRUE;
    }
  }
  
  return FALSE;
}

/*
The purpose of the actions performed in this function is to prevent that single or 'expensive' coefficients are coded.
With 4x4 transform there is larger chance that a single coefficient in a 8x8 or 16x16 block may be nonzero.
A single small (level=1) coefficient in a 8x8 block will cost: 3 or more bits for the coefficient,
4 bits for EOBs for the 4x4 blocks,possibly also more bits for CBP.  Hence the total 'cost' of that single
coefficient will typically be 10-12 bits which in a RD consideration is too much to justify the distortion improvement.
The action below is to watch such 'single' coefficients and set the reconstructed block equal to the prediction according
to a given criterium.  The action is taken only for inter luma blocks.

Notice that this is a pure encoder issue and hence does not have any implication on the standard.
coeff_cost is a parameter set in residual_transform_quant_luma_4x4() and accumulated for each 8x8 block.  If level=1 for a coefficient,
coeff_cost is increased by a number depending on RUN for that coefficient.The numbers are (see also residual_transform_quant_luma_4x4()): 3,2,2,1,1,1,0,0,...
when RUN equals 0,1,2,3,4,5,6, etc.
If level >1 coeff_cost is increased by 9 (or any number above 3). The threshold is set to 3. This means for example:
1: If there is one coefficient with (RUN,level)=(0,1) in a 8x8 block this coefficient is discarded.
2: If there are two coefficients with (RUN,level)=(1,1) and (4,1) the coefficients are also discarded
sum_cnt_nonz is the accumulation of coeff_cost over a whole macro block.  If sum_cnt_nonz is 5 or less for the whole MB,
all nonzero coefficients are discarded for the MB and the reconstructed block is set equal to the prediction.
*/
int reset_block(Macroblock* currMB, int *cbp, int64 *cbp_blk, int block8x8)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  int i, j;
  int    mb_y       = (block8x8 >> 1) << 3;
  int    mb_x       = (block8x8 & 0x01) << 3;
  int    cbp_mask   = 1 << block8x8;

  (*cbp)     &=  (63 - cbp_mask);
  (*cbp_blk) &= ~(51 << (4 * block8x8 - 2 * (block8x8 & 0x01)));

  memset( currSlice->cofAC[block8x8][0][0], 0, 4 * 2 * 65 * sizeof(int));

  copy_image_data_8x8(&p_Vid->enc_picture->imgY[currMB->pix_y + mb_y], &currSlice->mb_pred[0][mb_y], currMB->pix_x + mb_x, mb_x);

  if (currSlice->slice_type == SP_SLICE || currSlice->slice_type == SI_SLICE)
  {
    for (i = mb_x; i < mb_x + BLOCK_SIZE_8x8; i += BLOCK_SIZE)
      for (j = mb_y; j < mb_y + BLOCK_SIZE_8x8; j += BLOCK_SIZE)
        copyblock_sp(currMB, PLANE_Y, i, j);
  }

  return 0;
}

/*!
 ************************************************************************
 * \brief
 *    Residual Coding of an 8x8 Luma block (not for intra)
 *
 * \return
 *    coefficient cost
 ************************************************************************
 */
int luma_residual_coding_16x16 (Macroblock* currMB,  //!< Current Macroblock to be coded
                                int   block8x8,      //!< block number of 8x8 block                             
                                int   list_mode[2]   //!< list prediction mode (1-7, 0=DIRECT)
)
{

  int   *cbp     = &(currMB->cbp);
  int64 *cbp_blk = &(currMB->cbp_blk);
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;  
  int    nonzero = 0, cbp_blk_mask;
  int    coeff_cost = 0;
  int    mb_y       = (block8x8 >> 1) << 3;
  int    mb_x       = (block8x8 & 0x01) << 3;
  int    cbp_mask   = 1 << block8x8;

  int    skipped    = (list_mode[0] == 0 && list_mode[1] == 0 && (currSlice->slice_type != B_SLICE));

  //memset( currSlice->cofAC[block8x8][0][0], 0, 4 * 2 * 65 * sizeof(int));

  //===== loop over 4x4 blocks =====
  if(!currMB->luma_transform_size_8x8_flag)
  {
    //===== forward transform, Quantization, inverse Quantization, inverse transform, Reconstruction =====
    if (!skipped && ( (currSlice->NoResidueDirect != 1) || (currMB->qp_scaled[PLANE_Y] == 0 && p_Vid->active_sps->lossless_qpprime_flag == 1) ))
    {
      int block_y, block_x;

      for (block_y=mb_y; block_y<mb_y + 8; block_y += 4)
      {
        for (block_x=mb_x; block_x<mb_x + 8; block_x += 4)
        {          
          //===== forward transform, Quantization, inverse Quantization, inverse transform, Reconstruction =====
          nonzero = currMB->residual_transform_quant_luma_4x4 (currMB, PLANE_Y, block_x, block_y, &coeff_cost, 0);

          if (nonzero)
          {
            cbp_blk_mask = (block_x >> 2) + block_y;      
            (*cbp_blk) |= (int64) 1 << cbp_blk_mask;  // one bit for every 4x4 block
            (*cbp)     |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
          }
        }
      }
    }
  }
  else
  {
    if (currSlice->NoResidueDirect != 1 && !skipped)
    {
      if (currSlice->slice_type != SP_SLICE && currSlice->slice_type != SI_SLICE)
        nonzero = currMB->residual_transform_quant_luma_8x8 (currMB, PLANE_Y, block8x8, &coeff_cost, 0);

      if (nonzero)
      {
        (*cbp_blk) |= 51 << (4*block8x8 - 2*(block8x8 & 0x01)); // corresponds to 110011, as if all four 4x4 blocks contain coeff, shifted to block position
        (*cbp)     |= cbp_mask;                               // one bit for the 4x4 blocks of an 8x8 block
      }
    }
  }

  if (currSlice->NoResidueDirect != 1 && !skipped && coeff_cost <= _LUMA_COEFF_COST_ &&
  ((currMB->qp_scaled[PLANE_Y])!=0 || p_Vid->active_sps->lossless_qpprime_flag==0))
  {
    coeff_cost  = reset_block(currMB, cbp, cbp_blk, block8x8);
  }
  
  return coeff_cost;
}

/*!
 ************************************************************************
 * \brief
 *    Residual Coding of an 8x8 Luma block (not for intra)
 *
 * \return
 *    coefficient cost
 ************************************************************************
 */
int luma_residual_coding_8x8 (Macroblock* currMB,  //!< Current Macroblock to be coded
                           int   *cbp,         //!< Output: cbp (updated according to processed 8x8 luminance block)
                           int64 *cbp_blk,     //!< Output: block cbp (updated according to processed 8x8 luminance block)
                           int   block8x8,     //!< block number of 8x8 block
                           short p_dir,        //!< prediction direction
                           int   list_mode[2],      //!< list0 prediction mode (1-7, 0=DIRECT)
                           char  *ref_idx)     //!< reference pictures for each list
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
  int    block_y, block_x, pic_pix_x, nonzero = 0, cbp_blk_mask;
  int    coeff_cost = 0;
  int    mb_y       = (block8x8 >> 1) << 3;
  int    mb_x       = (block8x8 & 0x01) << 3;
  int    cbp_mask   = 1 << block8x8;  
  int    skipped    = (list_mode[0] == 0 && list_mode[1] == 0 && (currSlice->slice_type != B_SLICE));
  Boolean direct_inference = (list_mode[0] == 0 && list_mode[1] == 0 && currSlice->slice_type == B_SLICE && p_Vid->active_sps->direct_8x8_inference_flag == 0);

  char   bipred_me = currMB->b8x8[block8x8].bipred;
  currSlice->coeff_cost_cr[1] = currSlice->coeff_cost_cr[2] = 0;

  //memset( currSlice->cofAC[block8x8][0][0], 0, 4 * 2 * 65 * sizeof(int));

  //===== loop over 4x4 blocks =====
  if(!currMB->luma_transform_size_8x8_flag)
  {
    int    bxx, byy;                   // indexing curr_blk
    if (!direct_inference && (((p_dir == 0 || p_dir == 2 )&& list_mode[0] < 5) || ((p_dir == 1 || p_dir == 2 ) && list_mode[1] < 5)))
    {
      p_Dpb->pf_luma_prediction (currMB, mb_x, mb_y, 8, 8, p_dir, list_mode, ref_idx, bipred_me); 

      //===== compute prediction residual ======            
      compute_residue (&p_Vid->pCurImg[currMB->opix_y + mb_y], &currSlice->mb_pred[0][mb_y], &currSlice->mb_ores[0][mb_y], mb_x, currMB->pix_x + mb_x, 8, 8);
    }

    for (byy=0, block_y=mb_y; block_y<mb_y+8; byy+=4, block_y+=4)
    {
      for (bxx=0, block_x=mb_x; block_x<mb_x+8; bxx+=4, block_x+=4)
      {
        pic_pix_x = currMB->pix_x + block_x;
        cbp_blk_mask = (block_x >> 2) + block_y;

        //===== prediction of 4x4 block =====
        if (direct_inference || !(((p_dir == 0 || p_dir == 2 )&& list_mode[0] < 5) || ((p_dir == 1 || p_dir == 2 ) && list_mode[1] < 5)))
        {
          p_Dpb->pf_luma_prediction (currMB, block_x, block_y, 4, 4, p_dir, list_mode, ref_idx, bipred_me);

          //===== compute prediction residual ======            
          compute_residue(&p_Vid->pCurImg[currMB->opix_y + block_y], &currSlice->mb_pred[0][block_y], &currSlice->mb_ores[0][block_y], block_x, pic_pix_x, 4, 4);
        }

        //===== forward transform, Quantization, inverse Quantization, inverse transform, Reconstruction =====
        if (!skipped && ( (currSlice->NoResidueDirect != 1) || (currMB->qp_scaled[PLANE_Y] == 0 && p_Vid->active_sps->lossless_qpprime_flag == 1) ))
        {
          //===== forward transform, Quantization, inverse Quantization, inverse transform, Reconstruction =====
          nonzero = currMB->residual_transform_quant_luma_4x4 (currMB, PLANE_Y, block_x, block_y, &coeff_cost, 0);

          if (nonzero)
          {
            (*cbp_blk) |= (int64)1 << cbp_blk_mask;  // one bit for every 4x4 block
            (*cbp)     |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
          }
        }
      }
    }
  }
  else
  {
    block_y = mb_y;
    block_x = mb_x;
    pic_pix_x = currMB->pix_x + block_x;

    cbp_blk_mask = (block_x>>2) + block_y;

    //===== prediction of 4x4 block =====
    p_Dpb->pf_luma_prediction (currMB, block_x, block_y, 8, 8, p_dir, list_mode, ref_idx, bipred_me);

    //===== compute prediction residual ======            
    compute_residue (&p_Vid->pCurImg[currMB->opix_y + block_y], &currSlice->mb_pred[0][block_y], &currSlice->mb_ores[0][block_y], block_x, pic_pix_x, 8, 8);

    if (currSlice->NoResidueDirect != 1 && !skipped)
    {
      if (currSlice->slice_type != SP_SLICE && currSlice->slice_type != SI_SLICE)
        nonzero = currMB->residual_transform_quant_luma_8x8 (currMB, PLANE_Y, block8x8, &coeff_cost, 0);

      if (nonzero)
      {
        (*cbp_blk) |= 51 << (4*block8x8 - 2*(block8x8 & 0x01)); // corresponds to 110011, as if all four 4x4 blocks contain coeff, shifted to block position
        (*cbp)     |= cbp_mask;                               // one bit for the 4x4 blocks of an 8x8 block
      }
    }
  }

  if (currSlice->NoResidueDirect != 1 && !skipped && coeff_cost <= _LUMA_COEFF_COST_ &&
  ((currMB->qp_scaled[PLANE_Y])!=0 || p_Vid->active_sps->lossless_qpprime_flag==0))
  {
    coeff_cost  = reset_block(currMB, cbp, cbp_blk, block8x8);
  }

  return coeff_cost;
}

/*!
 ************************************************************************
 * \brief
 *    Set mode parameters and reference frames for an 8x8 block
 ************************************************************************
 */
void set_modes_and_reframe (Macroblock* currMB, int b8, short* p_dir, int list_mode[2], char *list_ref_idx)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  int j = 2*(b8>>1);
  int i = 2*(b8 & 0x01);

  list_mode[0] = list_mode[1] = list_ref_idx[0] = list_ref_idx[1] = -1;

  *p_dir  = currMB->b8x8[b8].pdir;

  if (currSlice->slice_type != B_SLICE)
  {
    list_ref_idx[0] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
    list_ref_idx[1] = 0;
    list_mode[0] = currMB->b8x8[b8].mode;
    list_mode[1] = 0;
  }
  else
  {
    if (*p_dir == -1)
    {
      list_ref_idx[0] = -1;
      list_ref_idx[1] = -1;
      list_mode[0] =  0;
      list_mode[1] =  0;
    }
    else if (*p_dir == 0)
    {
      list_ref_idx[0] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
      list_ref_idx[1] = 0;
      list_mode[0] = currMB->b8x8[b8].mode;
      list_mode[1] = 0;
    }
    else if (*p_dir == 1)
    {
      list_ref_idx[0] = 0;
      list_ref_idx[1] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1];
      list_mode[0] = 0;
      list_mode[1] = currMB->b8x8[b8].mode;
    }
    else
    {
      list_ref_idx[0] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
      list_ref_idx[1] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1];
      list_mode[0] = currMB->b8x8[b8].mode;
      list_mode[1] = currMB->b8x8[b8].mode;
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Set mode parameters and reference frames for an 8x8 block
 ************************************************************************
 */
void set_modes_and_reframe_i_slice (Macroblock* currMB, int b8, short* p_dir, int list_mode[2], char *list_ref_idx)
{
  *p_dir  = currMB->b8x8[b8].pdir;

  list_ref_idx[0] = -1;
  list_ref_idx[1] =  0;
  list_mode[0] = currMB->b8x8[b8].mode;
  list_mode[1] = 0;
}

/*!
 ************************************************************************
 * \brief
 *    Set mode parameters and reference frames for an 8x8 block
 ************************************************************************
 */
void set_modes_and_reframe_p_slice (Macroblock* currMB, int b8, short* p_dir, int list_mode[2], char *list_ref_idx)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  int j = 2*(b8>>1);
  int i = 2*(b8 & 0x01);

  *p_dir  = currMB->b8x8[b8].pdir;

  list_ref_idx[0] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
  list_ref_idx[1] = 0;
  list_mode[0] = currMB->b8x8[b8].mode;
  list_mode[1] = 0; 
}



/*!
 ************************************************************************
 * \brief
 *    Set mode parameters and reference frames for an 8x8 block
 ************************************************************************
 */
void set_modes_and_reframe_b_slice (Macroblock* currMB, int b8, short* p_dir, int list_mode[2], char *list_ref_idx)
{
  *p_dir  = currMB->b8x8[b8].pdir;

  if (*p_dir == -1)
  {
    list_ref_idx[0] = -1;
    list_ref_idx[1] = -1;
    list_mode[0] =  0;
    list_mode[1] =  0;
  }
  else if (*p_dir == 0)
  {
    VideoParameters *p_Vid = currMB->p_Vid;
    PicMotionParams **motion = p_Vid->enc_picture->mv_info;
    int j = 2*(b8>>1);
    int i = 2*(b8 & 0x01);

    list_ref_idx[0] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
    list_ref_idx[1] = 0;
    list_mode[0] = currMB->b8x8[b8].mode;
    list_mode[1] = 0;
  }
  else if (*p_dir == 1)
  {
    VideoParameters *p_Vid = currMB->p_Vid;
    PicMotionParams **motion = p_Vid->enc_picture->mv_info;
    int j = 2*(b8>>1);
    int i = 2*(b8 & 0x01);

    list_ref_idx[0] = 0;
    list_ref_idx[1] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1];
    list_mode[0] = 0;
    list_mode[1] = currMB->b8x8[b8].mode;
  }
  else
  {
    VideoParameters *p_Vid = currMB->p_Vid;
    PicMotionParams **motion = p_Vid->enc_picture->mv_info;
    int j = 2*(b8>>1);
    int i = 2*(b8 & 0x01);

    list_ref_idx[0] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0];
    list_ref_idx[1] = motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1];
    list_mode[0] = currMB->b8x8[b8].mode;
    list_mode[1] = currMB->b8x8[b8].mode;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Residual Coding of a Luma macroblock (not for intra)
 ************************************************************************
 */
void luma_residual_coding (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];

  int block8x8;
  int list_mode[2];
  char list_ref_idx[2];
  short p_dir = 0;

  int is_skip = currSlice->slice_type == P_SLICE && currMB->mb_type == PSKIP;   

  currMB->cbp     = 0;
  currMB->cbp_blk = 0;
  // the following parameters are not useful and should be removed for profiles < P444
  currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;
  currSlice->cur_cbp_blk[1] = currSlice->cur_cbp_blk[2] = 0;

  if (is_skip)
  {
    for (block8x8=0; block8x8<4; ++block8x8)
    {    
      currSlice->set_modes_and_reframe (currMB, block8x8, &p_dir, list_mode, list_ref_idx);
    }

    //===== luma prediction ======
    p_Dpb->pf_luma_prediction (currMB, 0, 0, 16, 16, p_dir, list_mode, list_ref_idx, currMB->b8x8[0].bipred); 

    //===== compute prediction residual ======
    // We should not need to compute this for skip mode, but currently computed for "debugging" purposes
    compute_residue (&p_Vid->pCurImg[currMB->opix_y], *currSlice->mb_pred, *currSlice->mb_ores, 0, currMB->pix_x, MB_BLOCK_SIZE, MB_BLOCK_SIZE);

    copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], *currSlice->mb_pred, currMB->pix_x, 0);
  }
  else 
  {
    int sum_cnt_nonz = 0;
    if (currMB->mb_type == P16x16)
    {
      for (block8x8 = 0; block8x8 < 4; ++block8x8)
      {    
        currSlice->set_modes_and_reframe (currMB, block8x8, &p_dir, list_mode, list_ref_idx);
      }
      //===== luma prediction ======
      p_Dpb->pf_luma_prediction (currMB, 0, 0, MB_BLOCK_SIZE, MB_BLOCK_SIZE, p_dir, list_mode, list_ref_idx, currMB->b8x8[0].bipred); 

      //===== compute prediction residual ======      
      compute_residue (&p_Vid->pCurImg[currMB->opix_y], *currSlice->mb_pred, *currSlice->mb_ores, 0, currMB->pix_x, MB_BLOCK_SIZE, MB_BLOCK_SIZE);

      // Luma residual coding (16x16 mode)
      for (block8x8 = 0; block8x8 < 4; ++block8x8)
      {    
        sum_cnt_nonz += luma_residual_coding_16x16 (currMB, block8x8, list_mode);
      }
    }
    else
    {
      for (block8x8 = 0; block8x8 < 4; ++block8x8)
      {
        currSlice->set_modes_and_reframe (currMB, block8x8, &p_dir, list_mode, list_ref_idx);
        sum_cnt_nonz += currSlice->luma_residual_coding_8x8 (currMB, &(currMB->cbp), &(currMB->cbp_blk), block8x8, p_dir, list_mode, list_ref_idx);
      }
    }

    // reset information (coefficient thresholding)
    if (sum_cnt_nonz <= _LUMA_MB_COEFF_COST_ && ((currMB->qp_scaled[PLANE_Y])!=0 || p_Vid->active_sps->lossless_qpprime_flag == 0))
    {
      currMB->cbp     &= 0xfffff0 ;
      currMB->cbp_blk &= 0xff0000 ;

      copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], currSlice->mb_pred[0], currMB->pix_x, 0);
      //memset( currSlice->cofAC[0][0][0], 0, 2080 * sizeof(int)); // 4 * 4 * 2 * 65    
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Residual Coding of a Luma macroblock (not for intra)
 ************************************************************************
 */
void luma_residual_coding_sp (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];

  int i, j, block8x8, b8_x, b8_y;
  int list_mode[2];
  char list_ref_idx[2];
  short p_dir = 0;
  int sum_cnt_nonz = 0;

  int is_skip = currSlice->slice_type == P_SLICE && currMB->mb_type == PSKIP;   

  currMB->cbp     = 0;
  currMB->cbp_blk = 0;
  // the following parameters are not useful and should be removed for profiles < P444
  currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;
  currSlice->cur_cbp_blk[1] = currSlice->cur_cbp_blk[2] = 0;

  if (is_skip || currMB->mb_type == 1)
  {
    for (block8x8=0; block8x8<4; ++block8x8)
    {    
      currSlice->set_modes_and_reframe (currMB, block8x8, &p_dir, list_mode, list_ref_idx);
    }

    //===== luma prediction ======
    p_Dpb->pf_luma_prediction (currMB, 0, 0, 16, 16, p_dir, list_mode, list_ref_idx, currMB->b8x8[0].bipred); 

    //===== compute prediction residual ======
    // We should not need to compute this for skip mode, but currently computed for "debugging" purposes
    compute_residue (&p_Vid->pCurImg[currMB->opix_y], &currSlice->mb_pred[0][0], &currSlice->mb_ores[0][0], 0, currMB->pix_x, 16, 16);

    // Luma residual coding (16x16 mode)
    if (!is_skip)
    {
      for (block8x8=0; block8x8<4; ++block8x8)
      {    
        sum_cnt_nonz += luma_residual_coding_16x16 (currMB, block8x8, list_mode);
      }
    }
  }
  else
  {
    for (block8x8=0; block8x8<4; ++block8x8)
    {
      currSlice->set_modes_and_reframe (currMB, block8x8, &p_dir, list_mode, list_ref_idx);
      sum_cnt_nonz += currSlice->luma_residual_coding_8x8 (currMB, &(currMB->cbp), &(currMB->cbp_blk), block8x8, p_dir, list_mode, list_ref_idx);
    }
  }

  // reset information (coefficient thresholding)

  if ((is_skip || 
    (sum_cnt_nonz <= _LUMA_MB_COEFF_COST_ && ((currMB->qp_scaled[PLANE_Y])!=0 || p_Vid->active_sps->lossless_qpprime_flag == 0))))// modif ES added last set of conditions
  {
    currMB->cbp     &= 0xfffff0 ;
    currMB->cbp_blk &= 0xff0000 ;

    copy_image_data_16x16(&p_Vid->enc_picture->imgY[currMB->pix_y], currSlice->mb_pred[0], currMB->pix_x, 0);

    //memset( currSlice->cofAC[0][0][0], 0, 2080 * sizeof(int)); // 4 * 4 * 2 * 65

    for(block8x8 = 0;block8x8 < 4; ++block8x8)
    {
      b8_x=(block8x8&1)<<3;
      b8_y=(block8x8&2)<<2;
      for (i = b8_x; i < b8_x + BLOCK_SIZE_8x8; i += 4)
        for (j = b8_y; j < b8_y + BLOCK_SIZE_8x8;j += 4)
          copyblock_sp(currMB, PLANE_Y, i, j);
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    RDOff decision between 4x4 and 8x8 transform
 ************************************************************************
 */
byte transform_decision (Macroblock *currMB, int block_check, distblk *cost)
{
  if(currMB->p_Inp->Transform8x8Mode == 2) //always allow 8x8 transform
  {
    return 1;
  }
  else
  {
    VideoParameters *p_Vid = currMB->p_Vid;
    Slice *currSlice = currMB->p_Slice;
    DecodedPictureBuffer *p_Dpb = p_Vid->p_Dpb_layer[p_Vid->dpb_layer_id];
    int pic_pix_y, pic_pix_x, i, j;
    int    mb_y, mb_x, block8x8;  
    int    list_mode[2];
    char   list_ref_idx[2];
    short  p_dir;
    int    num_blks;
    distblk    cost8x8=0, cost4x4=0;
    char    bipred_me;
    imgpel **mb_pred = currSlice->mb_pred[0];
    short diff16[4][16];
    short diff64[64];
    int index;

    if(block_check == -1)
    {
      block8x8 = 0;
      num_blks = 4;
    }
    else
    {
      block8x8 = block_check;
      num_blks = block_check + 1;
    }

    for (; block8x8<num_blks; ++block8x8)
    {
      short *tmp64 = diff64;
      short *tmp16[4]; //{diff16[0], diff16[1], diff16[2], diff16[3]};

      tmp16[0] = diff16[0];
      tmp16[1] = diff16[1];
      tmp16[2] = diff16[2];
      tmp16[3] = diff16[3];

      currSlice->set_modes_and_reframe (currMB, block8x8, &p_dir, list_mode, list_ref_idx);
      bipred_me = currMB->b8x8[block8x8].bipred; 
      mb_y = (block8x8 >> 1) << 3;
      mb_x = (block8x8 & 0x01) << 3;

      //===== prediction of 8x8 block =====
      p_Dpb->pf_luma_prediction (currMB, mb_x, mb_y, 8, 8, p_dir, list_mode, list_ref_idx, bipred_me);

      //===== loop over 8x8 blocks =====
      pic_pix_y = currMB->opix_y;
      pic_pix_x = currMB->pix_x;
      //===== get displaced frame difference ======
      for (j = mb_y; j < 8 + mb_y; j++)
      {
        for (i = mb_x; i < 8 + mb_x; i++)
        {
          index = 2 * ((j - mb_y)> 3) + ((i - mb_x)> 3);
          *tmp64++ = *(tmp16[index])++ = (short) (p_Vid->pCurImg[pic_pix_y + j][pic_pix_x + i] - mb_pred[j][i]);
        }
      }

      cost4x4 += p_Vid->distortion4x4 (diff16[0], DISTBLK_MAX);
      cost4x4 += p_Vid->distortion4x4 (diff16[1], DISTBLK_MAX);
      cost4x4 += p_Vid->distortion4x4 (diff16[2], DISTBLK_MAX);
      cost4x4 += p_Vid->distortion4x4 (diff16[3], DISTBLK_MAX);
      cost8x8 += p_Vid->distortion8x8 (diff64   , DISTBLK_MAX);
    }


    if(cost8x8 < cost4x4)
    {
      return 1;
    }
    else
    {
      *cost += (cost4x4 - cost8x8);
      return 0;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Chroma residual coding for an macroblock
 ************************************************************************
 */
void chroma_residual_coding (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  int chroma_cbp = 0;
  int   uv, block_y, block_x;
  int   list_mode[2];
  char  list_ref_idx[2];
  short p_dir;
  int   skipped = (currMB->mb_type == 0 && (currSlice->slice_type == P_SLICE));
  int   yuv = p_Vid->yuv_format - 1; 

  for (uv = 0; uv < 2; ++uv)
  {
    //===== prediction of chrominance blocks ===d==
    // Intra modes
    if (currMB->mb_type > P8x8)
    {
      for (block_y=0; block_y < p_Vid->mb_cr_size_y; block_y += BLOCK_SIZE)
      {
        for (block_x=0; block_x < p_Vid->mb_cr_size_x; block_x += BLOCK_SIZE)
        {
          int block8 = block8x8_idx[yuv][block_y>>2][block_x>>2];
          currSlice->set_modes_and_reframe (currMB, block8, &p_dir, list_mode, list_ref_idx);
          IntraChromaPrediction4x4 (currMB, uv + 1, block_x, block_y);          
        }
      }
    }
    else
    {
      for (block_y = 0; block_y < p_Vid->mb_cr_size_y; block_y += BLOCK_SIZE)
      {
        for (block_x = 0; block_x < p_Vid->mb_cr_size_x; block_x += BLOCK_SIZE)
        {
          int block8 = block8x8_idx[yuv][block_y>>2][block_x>>2];
          char bipred_me = currMB->b8x8[block8].bipred;
          currSlice->set_modes_and_reframe (currMB, block8, &p_dir, list_mode, list_ref_idx);

          chroma_prediction_4x4 (currMB, uv, block_x, block_y, p_dir, list_mode[0], list_mode[1], list_ref_idx[0], list_ref_idx[1], bipred_me);
        }
      }
    }

    // ==== set chroma residue to zero for skip Mode in SP frames
    if (currSlice->NoResidueDirect)
    {
      copy_image_data(&p_Vid->enc_picture->imgUV[uv][currMB->pix_c_y], currSlice->mb_pred[ uv + 1], currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    }        
    else if (skipped)
    {
      copy_image_data(&p_Vid->enc_picture->imgUV[uv][currMB->pix_c_y], currSlice->mb_pred[ uv + 1], currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    }
    else
    {
      compute_residue(&p_Vid->pImgOrg[uv + 1][currMB->opix_c_y ], currSlice->mb_pred[ uv + 1], currSlice->mb_ores[uv + 1], 0, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);

      //===== forward transform, Quantization, inverse Quantization, inverse transform, and Reconstruction =====
      chroma_cbp = currMB->residual_transform_quant_chroma_4x4[uv] (currMB, uv, chroma_cbp);
    }
  }

  //===== update currMB->cbp =====
  currMB->cbp += ((chroma_cbp) << 4);
}

/*!
 ************************************************************************
 * \brief
 *    Chroma residual coding for an macroblock
 ************************************************************************
 */
void chroma_residual_coding_si (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  int chroma_cbp = 0;
  int   uv, block_y, block_x;
  int   list_mode[2];
  char  list_ref_idx[2];
  short p_dir;
  int   yuv = p_Vid->yuv_format - 1; 

  for (uv = 0; uv < 2; ++uv)
  {
    //===== prediction of chrominance blocks ===d==
    for (block_y=0; block_y < p_Vid->mb_cr_size_y; block_y+=4)
    {
      for (block_x=0; block_x < p_Vid->mb_cr_size_x; block_x+=4)
      {
        int block8 = block8x8_idx[yuv][block_y>>2][block_x>>2];
        currSlice->set_modes_and_reframe (currMB, block8, &p_dir, list_mode, list_ref_idx);
        IntraChromaPrediction4x4 (currMB, uv + 1, block_x, block_y);          
      }
    }

    compute_residue(&p_Vid->pImgOrg[uv + 1][currMB->opix_c_y ], currSlice->mb_pred[ uv + 1], currSlice->mb_ores[uv + 1], 0, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);

    //===== forward transform, Quantization, inverse Quantization, inverse transform, and Reconstruction =====
    if (currMB->mb_type==I16MB || currMB->mb_type == I4MB)
    {
      chroma_cbp = residual_transform_quant_chroma_4x4(currMB, uv, chroma_cbp);
    }
    else
    {
      chroma_cbp = residual_transform_quant_chroma_4x4_sp2(currMB, uv, chroma_cbp);
    }
  }

  //===== update currMB->cbp =====
  currMB->cbp += ((chroma_cbp) << 4);
}


/*!
 ************************************************************************
 * \brief
 *    Chroma residual coding for an macroblock
 ************************************************************************
 */
void chroma_residual_coding_sp (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  int chroma_cbp;
  int   uv, block_y, block_x, j;
  int   list_mode[2];
  char  list_ref_idx[2];
  short p_dir;
  int   skipped = (currMB->mb_type == 0);
  int   yuv = p_Vid->yuv_format - 1; 


  for (chroma_cbp = 0, uv=0; uv<2; ++uv)
  {
    //===== prediction of chrominance blocks ===d==
    // Intra modes
    if (currMB->mb_type > P8x8)
    {
      for (block_y=0; block_y < p_Vid->mb_cr_size_y; block_y += BLOCK_SIZE)
      {
        for (block_x=0; block_x < p_Vid->mb_cr_size_x; block_x += BLOCK_SIZE)
        {
          int block8 = block8x8_idx[yuv][block_y>>2][block_x>>2];
          currSlice->set_modes_and_reframe (currMB, block8, &p_dir, list_mode, list_ref_idx);
          IntraChromaPrediction4x4 (currMB, uv + 1, block_x, block_y);          
        }
      }
    }
    else
    {
      for (block_y=0; block_y < p_Vid->mb_cr_size_y; block_y += BLOCK_SIZE)
      {
        for (block_x=0; block_x < p_Vid->mb_cr_size_x; block_x += BLOCK_SIZE)
        {
          int block8 = block8x8_idx[yuv][block_y>>2][block_x>>2];
          char bipred_me = currMB->b8x8[block8].bipred;
          currSlice->set_modes_and_reframe (currMB, block8, &p_dir, list_mode, list_ref_idx);

          chroma_prediction_4x4 (currMB, uv, block_x, block_y, p_dir, list_mode[0], list_mode[1], list_ref_idx[0], list_ref_idx[1], bipred_me);
        }
      }
    }

    // ==== set chroma residue to zero for skip Mode in SP frames
    if (currSlice->NoResidueDirect)
    {
      copy_image_data(&p_Vid->enc_picture->imgUV[uv][currMB->pix_c_y], currSlice->mb_pred[ uv + 1], currMB->pix_c_x, 0, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    }
    else if (skipped)
    {
      for (j=0; j<p_Vid->mb_cr_size_y; ++j)
        memset(currSlice->mb_ores[uv + 1][j], 0 , p_Vid->mb_cr_size_x * sizeof(int));
      for (j=0; j<p_Vid->mb_cr_size_y; ++j)
        memset(currSlice->mb_rres[uv + 1][j], 0 , p_Vid->mb_cr_size_x * sizeof(int));
    }   
    else
    {
      compute_residue(&p_Vid->pImgOrg[uv + 1][currMB->opix_c_y ], &currSlice->mb_pred[ uv + 1][0], &currSlice->mb_ores[uv + 1][0], 0, currMB->pix_c_x, p_Vid->mb_cr_size_x, p_Vid->mb_cr_size_y);
    }

    //===== forward transform, Quantization, inverse Quantization, inverse transform, and Reconstruction =====
    //===== Call function for skip mode in SP frames to properly process frame ====

    if (skipped)
    {
      if(p_Vid->sp2_frame_indicator)
        chroma_cbp = residual_transform_quant_chroma_4x4_sp2(currMB, uv, chroma_cbp);
      else
        chroma_cbp = residual_transform_quant_chroma_4x4_sp(currMB, uv, chroma_cbp);
    }
    else
    {
      if (!currSlice->NoResidueDirect && !skipped)
      {
        if (currMB->mb_type==I16MB || currMB->mb_type == I4MB)
        {
          chroma_cbp = residual_transform_quant_chroma_4x4(currMB, uv,chroma_cbp);
        }
        else
        {
          if(p_Vid->sp2_frame_indicator)
            chroma_cbp = residual_transform_quant_chroma_4x4_sp2(currMB, uv, chroma_cbp);// switching SP frames
          else
            chroma_cbp=residual_transform_quant_chroma_4x4_sp(currMB, uv, chroma_cbp);
        }
      }
    }
  }

  //===== update currMB->cbp =====
  currMB->cbp += ((chroma_cbp)<<4);
}

/*!
 ************************************************************************
 * \brief
 *    Check if all reference frames for a macroblock are zero
 ************************************************************************
 */
int ZeroRef (Macroblock* currMB)
{
  PicMotionParams **motion = currMB->p_Vid->enc_picture->mv_info;
  int i,j;

  for (j=currMB->block_y; j<currMB->block_y + BLOCK_MULTIPLE; ++j)
  {
    for (i=currMB->block_x; i<currMB->block_x + BLOCK_MULTIPLE; ++i)
    {
      if (motion[j][i].ref_idx[LIST_0] != 0)
        return 0;
    }
  }
  return 1;
}


/*!
 ************************************************************************
 * \brief
 *    Converts macroblock type to coding value
 ************************************************************************
 */
int MBType2Value (Macroblock* currMB)
{
  Slice *currSlice = currMB->p_Slice;
  static const int dir1offset[3]    =  { 1,  2, 3};
  static const int dir2offset[3][3] = {{ 0,  4,  8},   // 1. block forward
                                       { 6,  2, 10},   // 1. block backward
                                       {12, 14, 16}};  // 1. block bi-directional

  int si_slice = (currSlice->slice_type == SI_SLICE) ? 1 : 0;

  if (currSlice->slice_type != B_SLICE)
  {
    switch( currMB->mb_type )
    {
    case I8MB:
    case I4MB:
      return ((currSlice->slice_type  == I_SLICE || si_slice) ? (0 + si_slice) : 6);
      break;
    case SI4MB:
      return 0;
      break;
    case I16MB:
      return ((currSlice->slice_type  == I_SLICE || si_slice) ? (0 + si_slice) : 6) + currMB->i16offset;
      break;
    case IPCM:
      return ((currSlice->slice_type == I_SLICE || si_slice ) ? (25 + si_slice) : 31);
      break;
    case P8x8:
      {
        if (currSlice->symbol_mode==CAVLC && ZeroRef (currMB))
          return 5;
        else
          return 4;
      }
      break;
    default:
      return currMB->mb_type;
      break;
    }
  }
  else
  {
    switch( currMB->mb_type )
    {
    case 0:
      return 0;
      break;
    case I4MB:
    case I8MB:
      return 23;
      break;
    case I16MB:
      return 23 + currMB->i16offset;
      break;
    case IPCM:
      return 48;
      break;
    case P8x8:
      return 22;
      break;
    case 1:
      {
        int pdir0  = currMB->b8x8[0].pdir;
        return dir1offset[pdir0];
      }
      break;
    case 2:
      {
        int pdir0  = currMB->b8x8[0].pdir;
        int pdir1  = currMB->b8x8[3].pdir;

        return 4 + dir2offset[pdir0][pdir1];
      }
      break;
    default:
      {
        int pdir0  = currMB->b8x8[0].pdir;
        int pdir1  = currMB->b8x8[3].pdir;
        return 5 + dir2offset[pdir0][pdir1];
      }
      break;
    }
  }
}

/*!
************************************************************************
* \brief
*    Writes 4x4 intra prediction modes for a macroblock
************************************************************************
*/
static int writeIntra4x4Modes(Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;
  int i;
  SyntaxElement se;
  BitCounter    *mbBits     = &currMB->bits;
  const int     *partMap    = assignSE2partition[currSlice->partition_mode];
  DataPartition *dataPart   = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);

  int rate = 0;

  currMB->IntraChromaPredModeFlag = 1;

  for(i = 0; i < 16; ++i)
  {
    se.context = i;
    se.value1  = currMB->intra_pred_modes[i];
    se.value2  = 0;

#if TRACE
    if (se.value1 < 0 )
      snprintf(se.tracestring, TRACESTRING_SIZE, "Intra 4x4 mode  = predicted (context: %d)",se.context);
    else
      snprintf(se.tracestring, TRACESTRING_SIZE, "Intra 4x4 mode  = %3d (context: %d)",se.value1,se.context);
#endif

    // set symbol type and function pointers
    se.type = SE_INTRAPREDMODE;

    // encode and update rate
    currSlice->writeIntraPredMode (&se, dataPart);

    rate += se.len;
  }

  mbBits->mb_mode += rate;

  return rate;
}

/*!
************************************************************************
* \brief
*    Writes 8x8 intra prediction modes for a macroblock
************************************************************************
*/
static int writeIntra8x8Modes(Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice;  
  int block8x8;
  SyntaxElement se;
  BitCounter    *mbBits     = &currMB->bits;
  const int     *partMap    = assignSE2partition[currSlice->partition_mode];
  DataPartition *dataPart   = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);

  int rate = 0;

  currMB->IntraChromaPredModeFlag = 1;

  for(block8x8 = 0; block8x8 < 16; block8x8 += 4)
  {

    se.context = block8x8;
    se.value1  = currMB->intra_pred_modes8x8[block8x8];
    se.value2  = 0;

#if TRACE
    if (se.value1 < 0 )
      snprintf(se.tracestring, TRACESTRING_SIZE, "Intra 8x8 mode  = predicted (context: %d)",se.context);
    else
      snprintf(se.tracestring, TRACESTRING_SIZE, "Intra 8x8 mode  = %3d (context: %d)",se.value1,se.context);
#endif

    // set symbol type and function pointers
    se.type = SE_INTRAPREDMODE;

    // encode and update rate
    currSlice->writeIntraPredMode (&se, dataPart);

    rate += se.len;
  }
  mbBits->mb_mode += rate;

  return rate;
}

static int writeIntraModes(Macroblock *currMB)
{
  switch (currMB->mb_type)
  {
  case I4MB:
    return writeIntra4x4Modes(currMB);
    break;
  case I8MB:
    return writeIntra8x8Modes(currMB);
    break;
  default:
    return 0;
    break;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Converts 8x8 block type to coding value
 ************************************************************************
 */
int B8Mode2Value (Slice *currSlice, short b8mode, short b8pdir)
{
  static const int b8start[8] = {0,0,0,0, 1, 4, 5, 10};
  static const int b8inc  [8] = {0,0,0,0, 1, 2, 2, 1};

  return (currSlice->slice_type!=B_SLICE) ? (b8mode - 4) : b8start[b8mode] + b8inc[b8mode] * b8pdir;
}

static int writeIPCMLuma (Macroblock *currMB, imgpel **imgY, Bitstream *currStream, SyntaxElement *se, int *bitCount)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int i, j;
  int bit_size = MB_BLOCK_SIZE * MB_BLOCK_SIZE * p_Vid->bitdepth_luma;

  *bitCount += bit_size;
  se->len   = p_Vid->bitdepth_luma;
  se->type  = SE_MBTYPE;   

  for (j = currMB->pix_y; j < currMB->pix_y + MB_BLOCK_SIZE; ++j)
  {      
    for (i = currMB->pix_x; i < currMB->pix_x + MB_BLOCK_SIZE; ++i)
    {            
      se->bitpattern = imax(p_Vid->min_IPCM_value, imgY[j][i]);
      se->value1 = se->bitpattern;      
#if TRACE
      snprintf(se->tracestring, TRACESTRING_SIZE, "pcm_sample_luma (%d %d) = %d", j, i, se->bitpattern);
#endif
      writeSE_Fix(se, currStream);
    }
  }
  return bit_size;
}

static int writeIPCMChroma (Macroblock *currMB, imgpel **imgUV, Bitstream *currStream, SyntaxElement *se, int *bitCount)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  int i, j;
  int bit_size = p_Vid->mb_cr_size_y * p_Vid->mb_cr_size_x * p_Vid->bitdepth_luma;
  *bitCount += bit_size;

  se->len   = p_Vid->bitdepth_chroma;
  se->type  = SE_MBTYPE;

  for (j = currMB->pix_c_y; j < currMB->pix_c_y + p_Vid->mb_cr_size_y; ++j)
  {
    for (i = currMB->pix_c_x; i < currMB->pix_c_x + p_Vid->mb_cr_size_x; ++i)
    {
      se->bitpattern = imax(p_Vid->min_IPCM_value, imgUV[j][i]);
      se->value1 = se->bitpattern;
#if TRACE
      //snprintf(se.tracestring, TRACESTRING_SIZE, "pcm_sample_chroma (%s) (%d %d) = %d", uv?"v":"u", j,i,se->bitpattern);
      snprintf(se->tracestring, TRACESTRING_SIZE, "pcm_sample_chroma (%d %d) = %d", j, i, se->bitpattern);
#endif
      writeSE_Fix(se, currStream);
    }
  }
  return bit_size;
}

static int writeIPCMByteAlign(Bitstream *currStream, SyntaxElement *se, int *bitCount)
{
  if (currStream->bits_to_go < 8)
  {
    // This will only happen in the CAVLC case, CABAC is already padded
    se->type  = SE_MBTYPE;
    se->len   = currStream->bits_to_go;      
    *bitCount += se->len;
    se->bitpattern = 0;
#if TRACE
    snprintf(se->tracestring, TRACESTRING_SIZE, "pcm_alignment_zero_bits = %d", se->len);
#endif
    writeSE_Fix(se, currStream);

    return se->len;
  }
  else 
    return 0;
}

/*!
************************************************************************
* \brief
*    Codes IPCM data
************************************************************************
*/
static int writeIPCMData(Macroblock *currMB, DataPartition*  dataPart)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  int no_bits = 0;
  SyntaxElement   se;
  BitCounter      *mbBits = &currMB->bits;

  if (currSlice->symbol_mode == CABAC)
  {
    int len;
    EncodingEnvironmentPtr eep = &dataPart->ee_cabac;
    len = arienco_bits_written(eep);
    arienco_done_encoding(currMB, eep); // This pads to byte
    len = arienco_bits_written(eep) - len;
    no_bits += len;
    // Now restart the encoder
    arienco_start_encoding(eep, dataPart->bitstream->streamBuffer, &(dataPart->bitstream->byte_pos));
  }

  writeIPCMByteAlign(dataPart->bitstream, &se, &(currMB->bits.mb_y_coeff));

  no_bits += writeIPCMLuma (currMB, p_Vid->enc_picture->imgY, dataPart->bitstream, &se, &mbBits->mb_y_coeff);

  if ((p_Vid->active_sps->chroma_format_idc != YUV400) && (p_Inp->separate_colour_plane_flag == 0))
  {
    int uv;
    for (uv = 0; uv < 2; ++uv )
    {
      no_bits += writeIPCMChroma (currMB, p_Vid->enc_picture->imgUV[uv], dataPart->bitstream, &se, &mbBits->mb_uv_coeff);
    }
  }
  return no_bits;
}

/*!
************************************************************************
* \brief
*    Codes macroblock header
* \param currMB
*    current macroblock
* \param rdopt
*    true for calls during RD-optimization
* \param coeff_rate
*    bitrate of Luma and Chroma coeff
************************************************************************
*/
int write_b_slice_MB_layer (Macroblock *currMB, int rdopt, int *coeff_rate)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;

  int             i;
  int             mb_nr      = currMB->mbAddrX;
  Macroblock     *prevMB     = currMB->PrevMB; 
  int             prev_mb_nr = (NULL == prevMB) ? -1 : prevMB->mbAddrX;
  
  SyntaxElement   se;
  BitCounter      *mbBits    = &currMB->bits;  
  const int*      partMap    = assignSE2partition[currSlice->partition_mode];
  // choose the appropriate data partition
  DataPartition*  dataPart   = &(currSlice->partArr[partMap[SE_MBTYPE]]);
  Bitstream      *currStream;

  int             no_bits    = 0;
  int             skip       = currMB->mb_type ? 0 : !currMB->cbp;  
  int             prevMbSkipped = 0;  
  int             WriteFrameFieldMBInHeader = 0;

  if (currSlice->mb_aff_frame_flag)
  {
    if ((mb_nr & 0x01) == 0)
    {
      WriteFrameFieldMBInHeader = 1; // top field

      prevMbSkipped = 0;
    }
    else
    {
      if (prevMB->mb_type ? 0: !prevMB->cbp)
      {
        WriteFrameFieldMBInHeader = 1; // bottom, if top was skipped
      }

      prevMbSkipped = p_Vid->mb_data[prev_mb_nr].skip_flag;
    }
  }
  currMB->IntraChromaPredModeFlag = (char) is_intra(currMB);

  if (currSlice->symbol_mode == CABAC)
  {
    int mb_type = MBType2Value (currMB);
    if (currSlice->mb_aff_frame_flag && ((currMB->mbAddrX & 0x01) == 0 || prevMbSkipped))
    {
      byte   mb_field_tmp = currMB->mb_field;
      currMB->mb_field = field_flag_inference(currMB);
      CheckAvailabilityOfNeighborsCABAC(currMB);
      currMB->mb_field = mb_field_tmp;
    }

    //========= write mb_skip_flag (CABAC) =========    
    se.value1  = mb_type;
    se.value2  = currMB->cbp;
    se.type    = SE_MBTYPE;

    TRACE_SE (se.tracestring, "mb_skip_flag");
    currSlice->writeMB_Skip(currMB, &se, dataPart);

    no_bits         += se.len;

    CheckAvailabilityOfNeighborsCABAC(currMB);

    //========= write mb_aff (CABAC) =========
    if(currSlice->mb_aff_frame_flag && !skip) // check for copy mode
    {
      if(WriteFrameFieldMBInHeader)
      {
        se.value1 = currMB->mb_field;
        se.value2 = 0;
        se.type   =  SE_MBTYPE;

        TRACE_SE(se.tracestring, "mb_field_decoding_flag");
        currSlice->writeFieldModeInfo(currMB, &se, dataPart);

        no_bits         += se.len;
      }
    }

    //========= write mb_type (CABAC) =========
    if (currMB->mb_type != 0 || ((currMB->cbp != 0 || currSlice->cmp_cbp[1] !=0 || currSlice->cmp_cbp[2]!=0 )))
    {
      se.value1  = mb_type;
      se.value2  = 0;
      se.type    = SE_MBTYPE;

#if TRACE
      snprintf(se.tracestring, TRACESTRING_SIZE, "mb_type (B_SLICE) (%2d,%2d) = %3d",currMB->mb_x, currMB->mb_y, currMB->mb_type);
#endif
      currSlice->writeMB_typeInfo(currMB, &se, dataPart);

      no_bits   += se.len;
    }
  }
  // VLC not intra
  else if (currMB->mb_type != 0 || ((currMB->cbp != 0 || currSlice->cmp_cbp[1] != 0 || currSlice->cmp_cbp[2]!=0 )))
  {
    //===== Run Length Coding: Non-Skipped macroblock =====
    se.value1 = p_Vid->cod_counter;
    se.value2 = 0;
    se.type   = SE_MBTYPE;

    TRACE_SE (se.tracestring, "mb_skip_run");
    writeSE_UVLC(&se, dataPart);

    no_bits  += se.len;

    // store position after skip_run when re-encoding can occur (slices with fixed number of bytes)
    if ( (p_Vid->cod_counter) &&  (p_Inp->slice_mode == FIXED_RATE) && (!rdopt))
    {
      for (i=0; i<currSlice->max_part_nr; ++i)
      {
        currStream = currSlice->partArr[i].bitstream;
        currStream->stored_bits_to_go = currStream->bits_to_go;
        currStream->stored_byte_pos   = currStream->byte_pos;
        currStream->stored_byte_buf   = currStream->byte_buf;
      }
      p_Vid->p_Stats->stored_bit_slice       = p_Vid->p_Stats->bit_slice;
    }

    // Reset cod counter
    p_Vid->cod_counter = 0;

    // write mb_aff
    if(currSlice->mb_aff_frame_flag && !skip) // check for copy mode
    {
      if(WriteFrameFieldMBInHeader)
      {
        se.value1 = currMB->mb_field;
        se.type   =  SE_MBTYPE;

        TRACE_SE(se.tracestring, "mb_field_decoding_flag");
        writeSE_Flag (&se, dataPart);

        no_bits  += se.len;
      }
    }
    // Put out mb mode
    se.value1  = MBType2Value (currMB);

    se.type   = SE_MBTYPE;
    se.value2 = 0;

#if TRACE
    snprintf(se.tracestring, TRACESTRING_SIZE, "mb_type (B_SLICE) (%2d,%2d) = %3d",currMB->mb_x, currMB->mb_y, currMB->mb_type);
#endif

    currSlice->writeMB_typeInfo(currMB, &se, dataPart);

    no_bits  += se.len;
  }
  else
  {
    //Run Length Coding: Skipped macroblock
    ++(p_Vid->cod_counter);

    currMB->skip_flag = 1;
    // CAVLC
    reset_mb_nz_coeff(p_Vid, currMB->mbAddrX);

    if(FmoGetNextMBNr(p_Vid, currMB->mbAddrX) == -1 && p_Vid->cod_counter > 0)
    {
      // Put out run
      se.value1  = p_Vid->cod_counter;
      se.value2  = 0;
      se.type    = SE_MBTYPE;

      TRACE_SE(se.tracestring, "mb_skip_run");
      writeSE_UVLC(&se, dataPart);

      no_bits   += se.len;

      // Reset cod counter
      p_Vid->cod_counter = 0;
    }
  }
  mbBits->mb_mode = mbBits->mb_mode + (unsigned short) no_bits;;

  //init NoMbPartLessThan8x8Flag
  currMB->NoMbPartLessThan8x8Flag = (currMB->mb_type == 0 && !(p_Vid->active_sps->direct_8x8_inference_flag))? FALSE : TRUE;

  if (currMB->mb_type == IPCM)
  {    
    no_bits += writeIPCMData(currMB, &(currSlice->partArr[partMap[SE_LUM_DC_INTRA]]));
    return no_bits;
  }

  dataPart = &(currSlice->partArr[partMap[SE_MBTYPE]]);

  //===== BITS FOR 8x8 SUB-PARTITION MODES =====
  if (currMB->mb_type == P8x8)
  {
    for (i=0; i<4; ++i)
    {
      se.value1  = B8Mode2Value (currSlice, currMB->b8x8[i].mode, currMB->b8x8[i].pdir);
      se.value2  = 0;
      se.type    = SE_MBTYPE;
#if TRACE
      snprintf(se.tracestring, TRACESTRING_SIZE, "8x8 mode/pdir(%2d) = %3d/%d", i, currMB->b8x8[i].mode, currMB->b8x8[i].pdir);
#endif
      currSlice->writeB8_typeInfo (&se, dataPart);
      mbBits->mb_mode = mbBits->mb_mode + (unsigned short) se.len;;
      no_bits  += se.len;

      //set NoMbPartLessThan8x8Flag for P8x8 mode
      currMB->NoMbPartLessThan8x8Flag &= (Boolean) ((currMB->b8x8[i].mode == 0 && p_Vid->active_sps->direct_8x8_inference_flag) 
                                                    || (currMB->b8x8[i].mode == 4));
    }

    no_bits += currSlice->writeMotionInfo2NAL (currMB);
  }

  //============= Transform size flag for INTRA MBs =============
  //-------------------------------------------------------------
  //transform size flag for INTRA_4x4 and INTRA_8x8 modes
  if ((currMB->mb_type == I8MB || currMB->mb_type == I4MB) && p_Inp->Transform8x8Mode)
  {
    se.value1 = currMB->luma_transform_size_8x8_flag;
    se.type   = SE_MBTYPE;

#if TRACE
    snprintf(se.tracestring, TRACESTRING_SIZE, "transform_size_8x8_flag = %3d", currMB->luma_transform_size_8x8_flag);
#endif
    currSlice->writeMB_transform_size(currMB, &se, dataPart);

    mbBits->mb_mode = mbBits->mb_mode + (unsigned short) se.len;;
    no_bits         += se.len;
  }

  //===== BITS FOR INTRA PREDICTION MODES ====
  no_bits += writeIntraModes(currMB);
  //===== BITS FOR CHROMA INTRA PREDICTION MODE ====
  if (currMB->IntraChromaPredModeFlag && ((p_Vid->active_sps->chroma_format_idc != YUV400) && (p_Vid->active_sps->chroma_format_idc != YUV444)))
    no_bits += write_chroma_intra_pred_mode(currMB);
  else if(!rdopt) //GB CHROMA !!!!!
  {
    currMB->c_ipred_mode = DC_PRED_8;     
  }
  //setting c_ipred_mode to default is not the right place here
  //resetting in rdopt.c (but where ??)
  //with cabac and bframes maybe it could crash without this default
  //since cabac needs the right neighborhood for the later MBs

  //----- motion information -----
  if (currMB->mb_type !=0 && currMB->mb_type != P8x8 && (!is_intra(currMB)))
  {
    no_bits  += currSlice->writeMotionInfo2NAL (currMB);
  }

  if ((currMB->mb_type!=0) || ((currMB->cbp!=0 || currSlice->cmp_cbp[1]!=0 || currSlice->cmp_cbp[2]!=0)))
  {
    *coeff_rate  = write_CBP_and_Dquant (currMB);
    *coeff_rate += currSlice->writeCoeff16x16   (currMB, PLANE_Y);

    if ((p_Vid->active_sps->chroma_format_idc != YUV400) && ((p_Inp->separate_colour_plane_flag == 0)))
    {
      if(p_Vid->active_sps->chroma_format_idc == YUV444)
      {
        *coeff_rate += currSlice->writeCoeff16x16(currMB, PLANE_U);
        *coeff_rate += currSlice->writeCoeff16x16(currMB, PLANE_V);
      }
      else
        *coeff_rate  += write_chroma_coeff (currMB);
    }
    no_bits  += *coeff_rate;
  }

  return no_bits;
}

/*!
************************************************************************
* \brief
*    Codes macroblock header
* \param currMB
*    current macroblock
* \param rdopt
*    true for calls during RD-optimization
* \param coeff_rate
*    bitrate of Luma and Chroma coeff
************************************************************************
*/
int write_p_slice_MB_layer (Macroblock *currMB, int rdopt, int *coeff_rate)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;

  int             i;
  int             mb_nr      = currMB->mbAddrX;
  Macroblock     *prevMB     = currMB->PrevMB; 
  int             prev_mb_nr = (NULL == prevMB) ? -1 : prevMB->mbAddrX;
  
  SyntaxElement   se;
  BitCounter      *mbBits    = &currMB->bits;  
  const int*      partMap    = assignSE2partition[currSlice->partition_mode];
  // choose the appropriate data partition
  DataPartition*  dataPart   = &(currSlice->partArr[partMap[SE_MBTYPE]]);
  Bitstream      *currStream;
  
  int             no_bits    = 0;
  int             skip       = currMB->mb_type ? 0 : 1;  
  int             prevMbSkipped = 0;  
  int             WriteFrameFieldMBInHeader = 0;

  if (currSlice->mb_aff_frame_flag)
  {
    if ((mb_nr & 0x01) == 0)
    {
      WriteFrameFieldMBInHeader = 1; // top field

      prevMbSkipped = 0;
    }
    else
    {
      if (prevMB->mb_type ? 0 : 1)
      {
        WriteFrameFieldMBInHeader = 1; // bottom, if top was skipped
      }

      prevMbSkipped = p_Vid->mb_data[prev_mb_nr].skip_flag;
    }
  }
  currMB->IntraChromaPredModeFlag = (char) is_intra(currMB);

  if (currSlice->symbol_mode == CABAC)
  {
    int mb_type = MBType2Value (currMB);
    if (currSlice->mb_aff_frame_flag && ((currMB->mbAddrX & 0x01) == 0 || prevMbSkipped))
    {
      byte   mb_field_tmp = currMB->mb_field;
      currMB->mb_field = field_flag_inference(currMB);
      CheckAvailabilityOfNeighborsCABAC(currMB);
      currMB->mb_field = mb_field_tmp;
    }

    //========= write mb_skip_flag (CABAC) =========    
    se.value1  = mb_type;
    se.value2  = currMB->cbp;
    se.type    = SE_MBTYPE;

    TRACE_SE (se.tracestring, "mb_skip_flag");
    currSlice->writeMB_Skip(currMB, &se, dataPart);

    no_bits         += se.len;

    CheckAvailabilityOfNeighborsCABAC(currMB);

    //========= write mb_aff (CABAC) =========
    if(currSlice->mb_aff_frame_flag && !skip) // check for copy mode
    {
      if(WriteFrameFieldMBInHeader)
      {
        se.value1 = currMB->mb_field;
        se.value2 = 0;
        se.type   =  SE_MBTYPE;

        TRACE_SE(se.tracestring, "mb_field_decoding_flag");
        currSlice->writeFieldModeInfo(currMB, &se, dataPart);

        no_bits         += se.len;
      }
    }

    //========= write mb_type (CABAC) =========
    if (currMB->mb_type != 0)
    {
      se.value1  = mb_type;
      se.value2  = 0;
      se.type    = SE_MBTYPE;

#if TRACE
        snprintf(se.tracestring, TRACESTRING_SIZE, "mb_type (P_SLICE) (%2d,%2d) = %3d",currMB->mb_x, currMB->mb_y, currMB->mb_type);
#endif
      currSlice->writeMB_typeInfo(currMB, &se, dataPart);

      no_bits         += se.len;
    }
  }
  // VLC not intra
  else if (currMB->mb_type != 0)
  {
    //===== Run Length Coding: Non-Skipped macroblock =====
    se.value1 = p_Vid->cod_counter;
    se.value2 = 0;
    se.type   = SE_MBTYPE;

    TRACE_SE (se.tracestring, "mb_skip_run");
    writeSE_UVLC(&se, dataPart);

    no_bits         += se.len;

    // store position after skip_run when re-encoding can occur (slices with fixed number of bytes)
    if ( (p_Vid->cod_counter) &&  (p_Inp->slice_mode == FIXED_RATE) && (!rdopt))
    {
      for (i=0; i<currSlice->max_part_nr; ++i)
      {
        currStream = currSlice->partArr[i].bitstream;
        currStream->stored_bits_to_go = currStream->bits_to_go;
        currStream->stored_byte_pos   = currStream->byte_pos;
        currStream->stored_byte_buf   = currStream->byte_buf;
        p_Vid->p_Stats->stored_bit_slice       = p_Vid->p_Stats->bit_slice;
      }
    }
    // Reset cod counter
    p_Vid->cod_counter = 0;

    // write mb_aff
    if(currSlice->mb_aff_frame_flag && !skip) // check for copy mode
    {
      if(WriteFrameFieldMBInHeader)
      {
        se.value1 = currMB->mb_field;
        se.type   =  SE_MBTYPE;

        TRACE_SE(se.tracestring, "mb_field_decoding_flag");
        writeSE_Flag (&se, dataPart);

        no_bits         += se.len;
      }
    }
    // Put out mb mode
    se.value1  = MBType2Value (currMB);
    se.value1--;

    se.type   = SE_MBTYPE;
    se.value2 = 0;

#if TRACE
      snprintf(se.tracestring, TRACESTRING_SIZE, "mb_type (P_SLICE) (%2d,%2d) = %3d",currMB->mb_x, currMB->mb_y, currMB->mb_type);
#endif

    currSlice->writeMB_typeInfo(currMB, &se, dataPart);

    no_bits         += se.len;
  }
  else
  {
    //Run Length Coding: Skipped macroblock
    ++(p_Vid->cod_counter);

    currMB->skip_flag = 1;
    // CAVLC
    reset_mb_nz_coeff(p_Vid, currMB->mbAddrX);

    if(FmoGetNextMBNr(p_Vid, currMB->mbAddrX) == -1 && p_Vid->cod_counter > 0)
    {
      // Put out run
      se.value1  = p_Vid->cod_counter;
      se.value2  = 0;
      se.type    = SE_MBTYPE;

      TRACE_SE(se.tracestring, "mb_skip_run");
      writeSE_UVLC(&se, dataPart);

      no_bits         += se.len;

      // Reset cod counter
      p_Vid->cod_counter = 0;
    }
  }
  mbBits->mb_mode = mbBits->mb_mode + (unsigned short) no_bits;;

  //init NoMbPartLessThan8x8Flag
  currMB->NoMbPartLessThan8x8Flag = TRUE;

  if (currMB->mb_type == IPCM)
  {    
    no_bits += writeIPCMData(currMB, &(currSlice->partArr[partMap[SE_LUM_DC_INTRA]]));
    return no_bits;
  }

  dataPart = &(currSlice->partArr[partMap[SE_MBTYPE]]);

  //===== BITS FOR 8x8 SUB-PARTITION MODES =====
  if (currMB->mb_type == P8x8)
  {
    for (i=0; i<4; ++i)
    {
      se.value1  = B8Mode2Value (currSlice, currMB->b8x8[i].mode, currMB->b8x8[i].pdir);
      se.value2  = 0;
      se.type    = SE_MBTYPE;
#if TRACE
      snprintf(se.tracestring, TRACESTRING_SIZE, "8x8 mode/pdir(%2d) = %3d/%d", i, currMB->b8x8[i].mode, currMB->b8x8[i].pdir);
#endif
      currSlice->writeB8_typeInfo (&se, dataPart);
      mbBits->mb_mode = mbBits->mb_mode + (unsigned short) se.len;;
      no_bits         += se.len;

      //set NoMbPartLessThan8x8Flag for P8x8 mode
      currMB->NoMbPartLessThan8x8Flag &= (Boolean) ((currMB->b8x8[i].mode == 0 && p_Vid->active_sps->direct_8x8_inference_flag) 
                                                    || (currMB->b8x8[i].mode ==4));
    }

    no_bits += currSlice->writeMotionInfo2NAL (currMB);
  }

  //============= Transform size flag for INTRA MBs =============
  //-------------------------------------------------------------
  //transform size flag for INTRA_4x4 and INTRA_8x8 modes
  if ((currMB->mb_type == I8MB || currMB->mb_type == I4MB) && p_Inp->Transform8x8Mode)
  {
    se.value1 = currMB->luma_transform_size_8x8_flag;
    se.type   = SE_MBTYPE;

#if TRACE
    snprintf(se.tracestring, TRACESTRING_SIZE, "transform_size_8x8_flag = %3d", currMB->luma_transform_size_8x8_flag);
#endif
    currSlice->writeMB_transform_size(currMB, &se, dataPart);

    mbBits->mb_mode = mbBits->mb_mode + (unsigned short) se.len;;
    no_bits         += se.len;
  }

  //===== BITS FOR INTRA PREDICTION MODES ====
  no_bits += writeIntraModes(currMB);
  //===== BITS FOR CHROMA INTRA PREDICTION MODE ====
  if (currMB->IntraChromaPredModeFlag && ((p_Vid->active_sps->chroma_format_idc != YUV400) && (p_Vid->active_sps->chroma_format_idc != YUV444)))
    no_bits += write_chroma_intra_pred_mode(currMB);
  else if(!rdopt) //GB CHROMA !!!!!
  {
    currMB->c_ipred_mode = DC_PRED_8;     
  }
  //setting c_ipred_mode to default is not the right place here
  //resetting in rdopt.c (but where ??)
  //with cabac and bframes maybe it could crash without this default
  //since cabac needs the right neighborhood for the later MBs

  //----- motion information -----
  if (currMB->mb_type !=0 && currMB->mb_type != P8x8 && (!is_intra(currMB)))
  {
    no_bits  += currSlice->writeMotionInfo2NAL (currMB);
  }

  if (currMB->mb_type!=0)
  {
    *coeff_rate  = write_CBP_and_Dquant (currMB);
    *coeff_rate += currSlice->writeCoeff16x16   (currMB, PLANE_Y);

    if ((p_Vid->active_sps->chroma_format_idc != YUV400) && ((p_Inp->separate_colour_plane_flag == 0)))
    {
      if(p_Vid->active_sps->chroma_format_idc == YUV444)
      {
        *coeff_rate += currSlice->writeCoeff16x16(currMB, PLANE_U);
        *coeff_rate += currSlice->writeCoeff16x16(currMB, PLANE_V);
      }
      else
        *coeff_rate  += write_chroma_coeff (currMB);
    }
    no_bits  += *coeff_rate;
  }
  
  return no_bits;
}


int write_i_slice_MB_layer (Macroblock *currMB, int rdopt, int *coeff_rate)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Slice *currSlice = currMB->p_Slice;

  int             mb_nr      = currMB->mbAddrX;
  Macroblock     *prevMB     = currMB->PrevMB; 

  SyntaxElement   se;
  BitCounter      *mbBits    = &currMB->bits;  
  const int*      partMap    = assignSE2partition[currSlice->partition_mode];
  // choose the appropriate data partition
  DataPartition*  dataPart   = &(currSlice->partArr[partMap[SE_MBTYPE]]);

  int             no_bits    = 0;
  int             WriteFrameFieldMBInHeader = 0;

  if (currSlice->mb_aff_frame_flag)
  {
    if ((mb_nr & 0x01) == 0)
    {
      WriteFrameFieldMBInHeader = 1; // top field
    }
    else
    {
      if (prevMB->mb_type ? 0: 1)
      {
        WriteFrameFieldMBInHeader = 1; // bottom, if top was skipped
      }
    }

    //========= write mb_aff (I_SLICE) =========
    if(WriteFrameFieldMBInHeader)
    {
      se.value1 = currMB->mb_field;
      se.value2 = 0;
      se.type   = SE_MBTYPE;

      TRACE_SE (se.tracestring, "mb_field_decoding_flag");
      currSlice->writeFieldModeInfo(currMB, &se, dataPart);

      no_bits         += se.len;
    }
  }
  currMB->IntraChromaPredModeFlag = (char) is_intra(currMB);


  //========= write mb_type (I_SLICE) =========
  se.value1  = MBType2Value (currMB);
  se.value2  = 0;
  se.type    = SE_MBTYPE;

#if TRACE
  snprintf(se.tracestring, TRACESTRING_SIZE,   "mb_type (I_SLICE) (%2d,%2d) = %3d",currMB->mb_x, currMB->mb_y, currMB->mb_type);
#endif

  currSlice->writeMB_typeInfo (currMB, &se, dataPart);

  no_bits         += se.len;

  mbBits->mb_mode = mbBits->mb_mode + (unsigned short) no_bits;

  //init NoMbPartLessThan8x8Flag
  currMB->NoMbPartLessThan8x8Flag = TRUE;

  if (currMB->mb_type == IPCM)
  {
    no_bits += writeIPCMData(currMB, &(currSlice->partArr[partMap[SE_LUM_DC_INTRA]]));
    return no_bits;
  }

  dataPart = &(currSlice->partArr[partMap[SE_MBTYPE]]);

  //============= Transform size flag for INTRA MBs =============
  //-------------------------------------------------------------
  //transform size flag for INTRA_4x4 and INTRA_8x8 modes
  if ((currMB->mb_type == I8MB || currMB->mb_type == I4MB) && p_Inp->Transform8x8Mode)
  {
    se.value1 = currMB->luma_transform_size_8x8_flag;
    se.type   = SE_MBTYPE;

#if TRACE
    snprintf(se.tracestring, TRACESTRING_SIZE, "transform_size_8x8_flag = %3d", currMB->luma_transform_size_8x8_flag);
#endif
    currSlice->writeMB_transform_size(currMB, &se, dataPart);

    mbBits->mb_mode = mbBits->mb_mode + (unsigned short) se.len;;
    no_bits         += se.len;
  }

  //===== BITS FOR INTRA PREDICTION MODES ====
  no_bits += writeIntraModes(currMB);
  //===== BITS FOR CHROMA INTRA PREDICTION MODE ====
  if (currMB->IntraChromaPredModeFlag && ((p_Vid->active_sps->chroma_format_idc != YUV400) && (p_Vid->active_sps->chroma_format_idc != YUV444)))
    no_bits += write_chroma_intra_pred_mode(currMB);
  else if(!rdopt) //GB CHROMA !!!!!
  {
    currMB->c_ipred_mode = DC_PRED_8;     
  }
  //setting c_ipred_mode to default is not the right place here
  //resetting in rdopt.c (but where ??)
  //with cabac and bframes maybe it could crash without this default
  //since cabac needs the right neighborhood for the later MBs

  *coeff_rate  = write_CBP_and_Dquant (currMB);
  *coeff_rate += currSlice->writeCoeff16x16 (currMB, PLANE_Y);

  if ((p_Vid->active_sps->chroma_format_idc != YUV400) && ((p_Inp->separate_colour_plane_flag == 0)))
  {
    if(p_Vid->active_sps->chroma_format_idc == YUV444)
    {
      *coeff_rate += currSlice->writeCoeff16x16(currMB, PLANE_U);
      *coeff_rate += currSlice->writeCoeff16x16(currMB, PLANE_V);
    }
    else
      *coeff_rate  += write_chroma_coeff (currMB);
  }
  
  no_bits  += *coeff_rate;

  return no_bits;
}

void write_terminating_bit (Slice *currSlice, int bit)
{
  const int*     partMap  = assignSE2partition[currSlice->partition_mode];
  //--- write non-slice termination symbol if the macroblock is not the first one in its slice ---
  DataPartition* dataPart = &(currSlice->partArr[partMap[SE_MBTYPE]]);
  dataPart->bitstream->write_flag = 1;

  biari_encode_symbol_final(&(dataPart->ee_cabac), bit);
#if TRACE
  fprintf (p_Enc->p_trace, "      CABAC terminating bit = %d\n",bit);
#endif
}


/*!
 ************************************************************************
 * \brief
 *    Write chroma intra prediction mode.
 ************************************************************************
 */
int write_chroma_intra_pred_mode(Macroblock* currMB)
{
  Slice* currSlice = currMB->p_Slice;
  SyntaxElement   se;
  const int*      partMap   = assignSE2partition[currSlice->partition_mode];
  DataPartition*  dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);
  int             rate      = 0;
  

  //===== BITS FOR CHROMA INTRA PREDICTION MODES

  se.value1 = currMB->c_ipred_mode;
  se.value2 = 0;
  se.type = SE_INTRAPREDMODE;

  TRACE_SE(se.tracestring, "intra_chroma_pred_mode");
  currSlice->writeCIPredMode(currMB, &se, dataPart);

  currMB->bits.mb_uv_coeff += se.len;
  rate                       += se.len;

  return rate;
}


#if (MVC_EXTENSION_ENABLE)
int is_interview_mb(Macroblock *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  struct storable_picture **pList = currSlice->listX[0];
  int i,j, iRefIdx=-1, k;
  int interview_in_l0 = 1;
  int   step_h0 = (block_size[(currMB->mb_type == P8x8) ? 4 : currMB->mb_type][0] >> 2);
  int   step_v0 = (block_size[(currMB->mb_type == P8x8) ? 4 : currMB->mb_type][1] >> 2);
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;

  for(i=0; i<currSlice->listXsize[LIST_0]; i++)
  {
    if(pList[i]->view_id==0)
    {
      iRefIdx = i;
      break;
    }
  }
  if(iRefIdx == -1 && currSlice->slice_type == B_SLICE)
  {
    interview_in_l0 = 0;
    pList = currSlice->listX[LIST_1];
    for(i=0; i<currSlice->listXsize[LIST_1]; i++)
    {
      if(pList[i]->view_id==0)
      {
        iRefIdx = i;
        break;
      }
    }
  }

  if(iRefIdx<0)
  {
    return 0;
  }
  if(currMB->mb_type == PSKIP && currSlice->slice_type==P_SLICE)
  {
    return (iRefIdx==0);
  }
  for (j=0; j<4; j+=step_v0)
  {        
    for (i=0; i<4; i +=step_h0)
    {
      k = j + (i >> 1);
      if (interview_in_l0 && (currMB->b8x8[k].pdir == 0 || currMB->b8x8[k].pdir == 2))//has forward vector
      {
        if(motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_0] == iRefIdx)
          return 1;
      }
      else if (!interview_in_l0 && (currMB->b8x8[k].pdir == 1 || currMB->b8x8[k].pdir == 2))//has forward vector
      {
        if(motion[currMB->block_y + j][currMB->block_x + i].ref_idx[LIST_1] == iRefIdx)
          return 1;
      }
    }
  }

  return 0;
}
#endif
/*!
 ************************************************************************
 * \brief
 *    Passes the chosen syntax elements to the NAL
 ************************************************************************
 */
void write_macroblock (Macroblock* currMB, int eos_bit)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  BitCounter *mbBits = &currMB->bits;
  int i;

  // enable writing of trace file
#if TRACE
  if ( currMB->prev_recode_mb == FALSE )
  {
    for (i=0; i<currSlice->max_part_nr; ++i)
    {
      currSlice->partArr[i].bitstream->trace_enabled = TRUE;
    }
  }
#endif

  //--- constrain intra prediction ---
  if(p_Inp->UseConstrainedIntraPred && (currSlice->slice_type==P_SLICE || currSlice->slice_type==B_SLICE))
  {
    p_Vid->intra_block[currMB->mbAddrX] = is_intra(currMB);
  }

  //===== init and update number of intra macroblocks =====
  if (currMB->mbAddrX == 0)
  {
    p_Vid->intras = 0;
    p_Vid->iInterViewMBs =0;
  }

  if (is_intra(currMB))
    ++p_Vid->intras;
#if (MVC_EXTENSION_ENABLE)
  else if(p_Vid->num_of_layers==2 && p_Vid->dpb_layer_id==1)
  {
    int is_interview = is_interview_mb(currMB);

    if(is_interview)
      p_Vid->iInterViewMBs++; //p_Vid->enc_picture->mv_info

  }
#endif
  //--- write non-slice termination symbol if the macroblock is not the first one in its slice ---
  if (currSlice->symbol_mode==CABAC && currMB->mbAddrX != currSlice->start_mb_nr && eos_bit)
  {
    write_terminating_bit (currSlice, 0);
  }

#if TRACE
  // trace: write macroblock header
  if (p_Enc->p_trace)
  {
    fprintf(p_Enc->p_trace, "\n*********** Pic: %i (I/P) MB: %i Slice: %i **********\n\n", p_Vid->frame_no, currMB->mbAddrX, currSlice->slice_nr);
  }
#endif

  p_Vid->cabac_encoding = 1;

  currMB->write_mb = TRUE;
  //--- write macroblock ---
  currSlice->write_MB_layer (currMB, 0, &i); // i is temporary

  if (!((currMB->mb_type !=0 ) || ((currSlice->slice_type==B_SLICE) && currMB->cbp != 0) ))
  {
      reset_mb_nz_coeff(p_Vid, currMB->mbAddrX);  // CAVLC
  }

  //--- set total bit-counter ---
  mbBits->mb_total = mbBits->mb_mode  
    + mbBits->mb_y_coeff
    + mbBits->mb_inter 
    + mbBits->mb_cbp
    + mbBits->mb_delta_quant
    + mbBits->mb_uv_coeff
    + mbBits->mb_cb_coeff
    + mbBits->mb_cr_coeff;

  if ( p_Inp->RCEnable )
    rc_update_mb_stats(currMB);  

  /*record the total number of MBs*/
  ++(p_Vid->NumberofCodedMacroBlocks);
  
  p_Vid->p_Stats->bit_slice += mbBits->mb_total;

  p_Vid->cabac_encoding = 0;
}


/*!
 ************************************************************************
 * \brief
 *    Codes the reference frame
 ************************************************************************
 */
int writeReferenceFrame (Macroblock* currMB, int i, int j, int list_idx, int  ref)
{
  BitCounter      *mbBits   = &currMB->bits;
  Slice *currSlice = currMB->p_Slice;
  const int*      partMap   = assignSE2partition[currSlice->partition_mode];
  DataPartition*  dataPart  = &(currSlice->partArr[partMap[SE_REFFRAME]]);
  int             list      = list_idx + currMB->list_offset;
  SyntaxElement   se;   

  se.value1 = ref;
  se.type   = SE_REFFRAME;
  se.value2 = list_idx;

  currMB->subblock_x = (short) (i << 2); // position used for context determination
  currMB->subblock_y = (short) (j << 2); // position used for context determination

#if TRACE
  if (!list_idx)
    snprintf(se.tracestring, TRACESTRING_SIZE, "ref_idx_l0 = %d", se.value1);
  else
    snprintf(se.tracestring, TRACESTRING_SIZE, "ref_idx_l1 = %d", se.value1);
#endif

  currSlice->writeRefFrame[list](currMB, &se, dataPart);

  mbBits->mb_inter = mbBits->mb_inter + (unsigned short) se.len;
  
  return (se.len);
}


/*!
 ************************************************************************
 * \brief
 *    Writes motion vectors of an 8x8 block
 ************************************************************************
 */
int writeMotionVector8x8 (Macroblock *currMB, 
                          int  i0,
                          int  j0,
                          int  i1,
                          int  j1,
                          int  refframe,
                          int  list_idx,
                          int  mv_mode,
                          short bipred_me
                          )
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;

  PixelPos       block[4];
  int            i, j, k, l, m;
  int            curr_mvd;

  int            rate       = 0;
  int            step_h     = part_size[mv_mode][0];
  int            step_v     = part_size[mv_mode][1];
  SyntaxElement  se;
  BitCounter      *mbBits   = &currMB->bits;
  
  const int*     partMap    = assignSE2partition[currSlice->partition_mode];    
  DataPartition* dataPart = &(currSlice->partArr[partMap[SE_MVD]]);        
  int            refindex   = refframe;

  MotionVector **all_mv     = currSlice->all_mv[list_idx][refindex][mv_mode];
  MotionVector *cur_mv;
  MotionVector predMV; 
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  //MotionVector   **final_mv = &motion->mv[list_idx][currMB->block_y];
  short mvd[2];
  short         (*currMB_mvd)[4][2] = currMB->mvd[list_idx]; 
  se.type   = SE_MVD;

  if (bipred_me && is_bipred_enabled(p_Vid, mv_mode) && refindex == 0)
    all_mv = currSlice->bipred_mv[bipred_me - 1][list_idx][refindex][mv_mode]; 

  for (j = j0; j < j1; j += step_v)
  {
    currMB->subblock_y = (short) (j << 2); // position used for context determination    
    for (i = i0; i < i1; i += step_h)
    {
      currMB->subblock_x = (short) (i << 2); // position used for context determination      
      // When writing mv, let us use final mv instead of all_mv structure
      if (currMB->write_mb)
        cur_mv = &motion[currMB->block_y + j][currMB->block_x + i].mv[list_idx];
      else
        cur_mv = &all_mv[j][i];      

      // Lets recompute MV predictor. This should avoid any problems with alterations of the motion vectors after ME
      get_neighbors(currMB, block, i<<2, j<<2, step_h<<2);
      currMB->GetMVPredictor (currMB, block, &predMV, (short) refindex, p_Vid->enc_picture->mv_info, list_idx, (i<<2), (j<<2), step_h<<2, step_v<<2);
      //test_clip_mvs(p_Vid, cur_mv, currMB->write_mb);
      mvd[0] = cur_mv->mv_x - predMV.mv_x;
      mvd[1] = cur_mv->mv_y - predMV.mv_y;

      for (k=0; k<2; ++k)
      {
        curr_mvd = mvd[k];

        //--- store (oversampled) mvd ---
        for (l = j; l < j + step_v; ++l)
        {
          for (m = i; m < i + step_h; ++m)
          {
            currMB_mvd[l][m][k] = (short) curr_mvd;
          }
        }

        se.value1 = curr_mvd;
        se.value2  = (k << 1) + list_idx; // identifies the component and the direction; only used for context determination
#if TRACE
        if (k == 0)
        snprintf(se.tracestring, TRACESTRING_SIZE, "mvd_l%d (%d) = %3d  (org_mv %3d pred_mv %3d)",list_idx, k, curr_mvd, cur_mv->mv_x, predMV.mv_x);
        else
          snprintf(se.tracestring, TRACESTRING_SIZE, "mvd_l%d (%d) = %3d  (org_mv %3d pred_mv %3d)",list_idx, k, curr_mvd, cur_mv->mv_y, predMV.mv_y);
#endif
        currSlice->writeMVD (currMB, &se, dataPart);

        rate                    += se.len;
      }
    }
  }

  mbBits->mb_inter = mbBits->mb_inter + (unsigned short) rate;
  return rate;
}


/*!
 ************************************************************************
 * \brief
 *    Writes motion info (b slice)
 ************************************************************************
 */
int write_b_slice_motion_info_to_NAL (Macroblock* currMB)
{
  int   no_bits         = 0;
  //=== If multiple ref. frames, write reference frame for the MB ===
  if (currMB->mb_type != I4MB  && currMB->mb_type != I16MB && currMB->mb_type != I8MB  && currMB->mb_type != 0)
  {
    Slice* currSlice = currMB->p_Slice;
    VideoParameters *p_Vid = currSlice->p_Vid;
    PicMotionParams **motion = p_Vid->enc_picture->mv_info;
    int   k, j0, i0;

    int   step_h0 = (block_size[(currMB->mb_type == P8x8) ? 4 : currMB->mb_type][0] >> 2);
    int   step_v0 = (block_size[(currMB->mb_type == P8x8) ? 4 : currMB->mb_type][1] >> 2);
      
    // Disable for now so we can check MBAFF
    //if (currSlice->listXsize[0] > 1)
    {
      for (j0 = 0; j0 < 4; j0 += step_v0)
      {        
        for (i0 = 0; i0 < 4; i0 += step_h0)
        {
          k = j0 + (i0 >> 1);
          if (currMB->b8x8[k].mode !=0 && (currMB->b8x8[k].pdir == 0 || currMB->b8x8[k].pdir == 2))//has forward vector
          {
            no_bits += writeReferenceFrame (currMB, i0, j0, LIST_0, motion[currMB->block_y + j0][currMB->block_x + i0].ref_idx[LIST_0]);
          }
        }
      }
    }

    //if (currSlice->listXsize[1] > 1)
    {
      for (j0 = 0; j0 < 4; j0 += step_v0)
      {                
        for (i0 = 0; i0 < 4; i0 += step_h0)
        {
          k = j0 + (i0 >> 1);
          if (currMB->b8x8[k].mode !=0 && (currMB->b8x8[k].pdir == 1 || currMB->b8x8[k].pdir == 2))//has backward vector
          {
            no_bits += writeReferenceFrame (currMB, i0, j0, LIST_1, motion[currMB->block_y + j0][currMB->block_x + i0].ref_idx[LIST_1]);
          }
        }
      }
    }

    //===== write forward motion vectors =====
    for (j0=0; j0<4; j0+=step_v0)
    {
      for (i0=0; i0<4; i0+=step_h0)
      {
        k = j0 + (i0 >> 1);
        if (currMB->b8x8[k].mode !=0 && (currMB->b8x8[k].pdir == 0 || currMB->b8x8[k].pdir == 2))//has forward vector
        {

          no_bits  += writeMotionVector8x8 (currMB, i0, j0, i0 + step_h0, j0 + step_v0, motion[currMB->block_y + j0][currMB->block_x + i0].ref_idx[LIST_0], 
            LIST_0, currMB->b8x8[k].mode, currMB->b8x8[k].bipred);
        }
      }
    }

    //===== write backward motion vectors =====

    for (j0=0; j0<4; j0+=step_v0)
    {
      for (i0 = 0; i0 < 4; i0 += step_h0)
      {
        k=j0+(i0 >> 1);
        if (currMB->b8x8[k].mode !=0 && (currMB->b8x8[k].pdir == 1 || currMB->b8x8[k].pdir == 2))//has backward vector
        {          
          no_bits  += writeMotionVector8x8 (currMB, i0, j0, i0+step_h0, j0+step_v0, motion[currMB->block_y + j0][currMB->block_x + i0].ref_idx[LIST_1], 
            LIST_1, currMB->b8x8[k].mode, currMB->b8x8[k].bipred);
        }
      }
    }
  }

  return no_bits;
}


/*!
 ************************************************************************
 * \brief
 *    Writes motion info (p slice)
 ************************************************************************
 */
int write_p_slice_motion_info_to_NAL (Macroblock* currMB)
{
  int   no_bits         = 0;
  //=== If multiple ref. frames, write reference frame for the MB ===
  if (currMB->mb_type != I4MB  && currMB->mb_type != I16MB && currMB->mb_type != I8MB  && currMB->mb_type != 0)
  {
    Slice* currSlice = currMB->p_Slice;
    VideoParameters *p_Vid = currSlice->p_Vid;
    PicMotionParams **motion = p_Vid->enc_picture->mv_info;
    int   k, j0, i0;    
    int   step_h0         = (block_size[(currMB->mb_type == P8x8) ? 4 : currMB->mb_type][0] >> 2);
    int   step_v0         = (block_size[(currMB->mb_type == P8x8) ? 4 : currMB->mb_type][1] >> 2);

    // if CAVLC is turned on, a 8x8 macroblock with all ref=0 in a P-frame is signalled in macroblock mode
    if ((currMB->mb_type != P8x8) || !ZeroRef (currMB) || currSlice->symbol_mode==CABAC)
    {
      for (j0 = 0; j0 < 4; j0 += step_v0)
      {
        for (i0 = 0; i0 < 4; i0 += step_h0)
        {
          k = j0 + (i0 >> 1);
          if (currMB->b8x8[k].mode !=0 && (currMB->b8x8[k].pdir == 0 || currMB->b8x8[k].pdir == 2))
          {
            no_bits += writeReferenceFrame (currMB, i0, j0, LIST_0, motion[currMB->block_y + j0][currMB->block_x + i0].ref_idx[LIST_0]);
          }
        }
      }
    }
    
    //===== write forward motion vectors =====
    for (j0=0; j0<4; j0+=step_v0)
    {
      for (i0=0; i0<4; i0+=step_h0)
      {
        k = j0 + (i0 >> 1);
        if (currMB->b8x8[k].mode !=0 && (currMB->b8x8[k].pdir == 0 || currMB->b8x8[k].pdir == 2))//has forward vector
        {
          no_bits  += writeMotionVector8x8 (currMB, i0, j0, i0 + step_h0, j0 + step_v0, motion[currMB->block_y + j0][currMB->block_x + i0].ref_idx[LIST_0],
            LIST_0, currMB->b8x8[k].mode, currMB->b8x8[k].bipred);
        }
      }
    }
  }

  return no_bits;
}


/*!
 ************************************************************************
 * \brief
 *    Writes chrominance coefficients
 ************************************************************************
 */
static int write_chroma_coeff (Macroblock* currMB)
{
  Slice* currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;

  int             rate       = 0;
  int             block_rate = 0;
  SyntaxElement   se;
  const int*      partMap   = assignSE2partition[currSlice->partition_mode];
  int             cbp       = currMB->cbp;
  DataPartition*  dataPart;

  int   level;
  int   k, uv;
  int   b8, b4;
  int*  ACLevel;
  int*  ACRun;
  int*  DCLevel;
  int*  DCRun;

  static const int chroma_dc_context[3]=
  {
    CHROMA_DC, CHROMA_DC_2x4, CHROMA_DC_4x4
  };

  static const unsigned char chroma_ac_param[3][8][4] =
  {
    {{ 4, 20,  5, 21},
    {36, 52, 37, 53},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0}},
    {{ 4, 20,  5, 21},
    { 6, 22,  7, 23},
    {36, 52, 37, 53},
    {38, 54, 39, 55},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0}},
    {{ 4, 20,  5, 21},
    {36, 52, 37, 53},
    { 6, 22,  7, 23},
    {38, 54, 39, 55},
    { 8, 24,  9, 25},
    {40, 56, 41, 57},
    {10, 26, 11, 27},
    {42, 58, 43, 59}}
  };

  int   yuv = p_Vid->yuv_format - 1;
  //=====
  //=====   D C - C O E F F I C I E N T S
  //=====
  if (cbp > 15)  // check if any chroma bits in coded block pattern is set
  {
    if (currSlice->symbol_mode == CAVLC)
    {
      rate += currSlice->writeCoeff4x4_CAVLC (currMB, CHROMA_DC, 0, 0, 0);  //PLANE_U
      rate += currSlice->writeCoeff4x4_CAVLC (currMB, CHROMA_DC, 0, 0, 1);  //PLANE_V
    }
    else
    {
      currMB->is_intra_block = (byte) is_intra(currMB);
      se.type             = (currMB->is_intra_block ? SE_CHR_DC_INTRA : SE_CHR_DC_INTER);
      // choose the appropriate data partition
      dataPart            = &(currSlice->partArr[partMap[se.type]]);
      se.context          = chroma_dc_context[yuv];

      for (uv=0; uv < 2; ++uv)
      {
        p_Vid->is_v_block = uv;

        DCLevel = currSlice->cofDC[uv+1][0];
        DCRun   = currSlice->cofDC[uv+1][1];

        level = 1;
        for (k=0; k <= p_Vid->num_cdc_coeff && level != 0; ++k)
        {
          level = se.value1 = DCLevel[k]; // level
          se.value2 = DCRun  [k]; // run       

#if TRACE
          snprintf(se.tracestring, TRACESTRING_SIZE, "DC Chroma %2d: level =%3d run =%2d",k, level, se.value2);
#endif
          writeRunLevel_CABAC(currMB, &se, dataPart);

          block_rate += se.len;
        }
      }
      currMB->bits.mb_uv_coeff += block_rate;
      rate += block_rate;
    }
  }

  //=====
  //=====   A C - C O E F F I C I E N T S
  //=====
  uv=-1;
  if (cbp >> 4 == 2) // check if chroma bits in coded block pattern = 10b
  {
    if (currSlice->symbol_mode == CAVLC)
    {
      for (b8=4; b8 < (4 + p_Vid->num_blk8x8_uv); b8++)
      {
        rate += currSlice->writeCoeff4x4_CAVLC (currMB, CHROMA_AC, b8, 0, chroma_ac_param[yuv][b8 - 4][0]);
        rate += currSlice->writeCoeff4x4_CAVLC (currMB, CHROMA_AC, b8, 1, chroma_ac_param[yuv][b8 - 4][1]);
        rate += currSlice->writeCoeff4x4_CAVLC (currMB, CHROMA_AC, b8, 2, chroma_ac_param[yuv][b8 - 4][2]);
        rate += currSlice->writeCoeff4x4_CAVLC (currMB, CHROMA_AC, b8, 3, chroma_ac_param[yuv][b8 - 4][3]);
      }
    }
    else
    {
      block_rate = 0;
      currMB->is_intra_block = (byte) is_intra(currMB);
      se.context          = CHROMA_AC;
      se.type             = (currMB->is_intra_block ? SE_CHR_AC_INTRA : SE_CHR_AC_INTER);
      // choose the appropriate data partition
      dataPart = &(currSlice->partArr[partMap[se.type]]);

      for (b8=4; b8 < (4+p_Vid->num_blk8x8_uv); b8++)
      {
        for (b4=0; b4 < 4; b4++)
        {
          ACLevel = currSlice->cofAC[b8][b4][0];
          ACRun   = currSlice->cofAC[b8][b4][1];

          level=1;
          uv++;
          p_Vid->is_v_block = (uv >= (p_Vid->num_blk8x8_uv << 1));

          currMB->subblock_y = subblk_offset_y[yuv][b8 - 4][b4];
          currMB->subblock_x = subblk_offset_x[yuv][b8 - 4][b4];

          for (k=0; k < 16 && level != 0; k++)
          {
            level = se.value1 = ACLevel[k]; // level
            se.value2 = ACRun  [k]; // run

#if TRACE
            snprintf(se.tracestring, TRACESTRING_SIZE, "AC Chroma %2d: level =%3d run =%2d",k, level, se.value2);
#endif
            writeRunLevel_CABAC(currMB, &se, dataPart);
            block_rate += se.len;
          }
        }
      }
      currMB->bits.mb_uv_coeff += block_rate;
      rate += block_rate;
    }
  }

  return rate;
}

#if TRACE
static inline void trace_coeff(SyntaxElement *se, int plane, char *type, int k, int level, int run)
{
  if (plane == 0)
    snprintf(se->tracestring, TRACESTRING_SIZE, "Luma%s sng(%2d) level =%3d run =%2d", type, k, level,run);
  else if (plane == 1)
    snprintf(se->tracestring, TRACESTRING_SIZE, "Cb%s   sng(%2d) level =%3d run =%2d", type, k, level,run);
  else
    snprintf(se->tracestring, TRACESTRING_SIZE, "Cr%s   sng(%2d) level =%3d run =%2d", type, k, level,run);        
}
#endif

/*!
 ************************************************************************
 * \brief
 *    Writes Coeffs of an 4x4 block
 ************************************************************************
 */
int writeCoeff4x4_CABAC (Macroblock* currMB, ColorPlane plane, int b8, int b4, int intra4x4mode)
{
  Slice* currSlice = currMB->p_Slice;

  int             rate = 0;
  SyntaxElement   se;
  const int*      partMap   = assignSE2partition[currSlice->partition_mode];
  DataPartition*  dataPart;

  int level;
  int k;
  int pl_off=plane<<2;

  int *mb_bits_coeff = ((plane==0) ? &currMB->bits.mb_y_coeff : ((plane==1) ? &currMB->bits.mb_cb_coeff : &currMB->bits.mb_cr_coeff));
  int*  ACLevel = currSlice->cofAC[b8+pl_off][b4][0];
  int*  ACRun   = currSlice->cofAC[b8+pl_off][b4][1];

  currMB->subblock_x = ((b8&0x1)==0) ? (((b4&0x1)==0)? 0: 4) : (((b4&0x1)==0)? 8: 12); // horiz. position for coeff_count context
  currMB->subblock_y = (b8<2)        ? ((b4<2)       ? 0: 4) : ((b4<2)       ? 8: 12); // vert.  position for coeff_count context
  currMB->is_intra_block = (byte) intra4x4mode;
  se.context     = ((plane == 0) ? (LUMA_4x4) : ((plane==1) ? CB_4x4 : CR_4x4));

  // DC
  se.type = (intra4x4mode ? SE_LUM_DC_INTRA : SE_LUM_DC_INTER);

  // choose the appropriate data partition
  dataPart = &(currSlice->partArr[partMap[se.type]]);

  level = se.value1 = ACLevel[0]; // level
  se.value2 = ACRun  [0]; // run

#if TRACE
  trace_coeff(&se, plane, "4x4", 0, level, se.value2);
#endif

  writeRunLevel_CABAC(currMB, &se, dataPart);    
  rate                    += se.len;

  // AC Coefficients
  se.type  = (intra4x4mode ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER);
  // choose the appropriate data partition
  dataPart = &(currSlice->partArr[partMap[se.type]]);

  for(k = 1; k <= 16 && level != 0; k++)
  {
    level = se.value1 = ACLevel[k]; // level
    se.value2 = ACRun  [k]; // run    
#if TRACE
    trace_coeff(&se, plane, "4x4", k, level, se.value2);
#endif
    writeRunLevel_CABAC(currMB, &se, dataPart);    

    rate += se.len;
  }

  *mb_bits_coeff += rate;

  return rate;
}

/*!
************************************************************************
* \brief
*    Writes coefficients of an 8x8 block (CABAC)
************************************************************************
*/
int writeCoeff8x8_CABAC (Macroblock* currMB, ColorPlane plane, int b8, int intra_mode)
{
  Slice* currSlice = currMB->p_Slice;

  int             rate      = 0;
  SyntaxElement   se;   
  const int*      partMap   = assignSE2partition[currSlice->partition_mode];
  DataPartition*  dataPart;

  int   level;
  int   k;

  int pl_off = plane<<2;  
  int *mb_bits_coeff = ((plane==0) ? &currMB->bits.mb_y_coeff : ((plane==1) ? &currMB->bits.mb_cb_coeff : &currMB->bits.mb_cr_coeff));
  int*  ACLevel = currSlice->cofAC[b8+pl_off][0][0];
  int*  ACRun   = currSlice->cofAC[b8+pl_off][0][1];

  currMB->subblock_x = ((b8&0x1) == 0) ? 0 : 8;  // horiz. position for coeff_count context
  currMB->subblock_y =  (b8 < 2)       ? 0 : 8;  // vert.  position for coeff_count context
  currMB->is_intra_block = (byte) intra_mode;

  se.context = ((plane==0) ? (LUMA_8x8) : ((plane == 1)? CB_8x8 : CR_8x8));
  
  // DC
  level = se.value1 = ACLevel[0]; // level
  se.value2 = ACRun  [0]; // run

  se.type = (intra_mode ? SE_LUM_DC_INTRA : SE_LUM_DC_INTER);

  // choose the appropriate data partition
  dataPart = &(currSlice->partArr[partMap[currSlice->slice_type != B_SLICE ? se.type : SE_BFRAME]]);

#if TRACE
  trace_coeff(&se, plane, "8x8", 0, level, se.value2);
#endif

  writeRunLevel_CABAC(currMB, &se, dataPart);
  rate                    += se.len;

  // AC
  se.type        = (intra_mode ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER);
  // choose the appropriate data partition
  dataPart = &(currSlice->partArr[partMap[currSlice->slice_type != B_SLICE ? se.type : SE_BFRAME]]);

  for(k=1; k<=64 && level !=0; k++)
  {
    level = se.value1 = ACLevel[k]; // level
     se.value2 = ACRun  [k]; // run

#if TRACE
    trace_coeff(&se, plane, "8x8", k, level, se.value2);
#endif

    writeRunLevel_CABAC(currMB, &se, dataPart);
    rate += se.len;
  }

  *mb_bits_coeff += rate;
  return rate;
}

/*!
************************************************************************
* \brief
*    Writes Luma Coeff of an 8x8 block
************************************************************************
*/
int writeCoeff8x8 (Macroblock* currMB, ColorPlane pl, int block8x8, int block_mode, int transform_size_flag)
{
  Slice* currSlice = currMB->p_Slice;
  int  rate = 0;
  int  is_intra = (block_mode == IBLOCK) || (block_mode == I8MB);  

  //if (block_mode == I8MB)
    //assert(transform_size_flag == 1);

  if (currSlice->symbol_mode == CAVLC )
  {
    int  block_color_type = ((pl == 0) ? LUMA : ((pl == 1) ? CB : CR));

    rate += currSlice->writeCoeff4x4_CAVLC (currMB, block_color_type, block8x8, 0, is_intra);
    rate += currSlice->writeCoeff4x4_CAVLC (currMB, block_color_type, block8x8, 1, is_intra);
    rate += currSlice->writeCoeff4x4_CAVLC (currMB, block_color_type, block8x8, 2, is_intra);
    rate += currSlice->writeCoeff4x4_CAVLC (currMB, block_color_type, block8x8, 3, is_intra);
  }
  else // CABAC
  {
    if(transform_size_flag) // 8x8
      rate += writeCoeff8x8_CABAC (currMB, pl, block8x8, is_intra);
    else
    {
      rate += writeCoeff4x4_CABAC (currMB, pl, block8x8, 0, is_intra);
      rate += writeCoeff4x4_CABAC (currMB, pl, block8x8, 1, is_intra);
      rate += writeCoeff4x4_CABAC (currMB, pl, block8x8, 2, is_intra);
      rate += writeCoeff4x4_CABAC (currMB, pl, block8x8, 3, is_intra);
    }      
  }

  return rate;
}

/*!
 ************************************************************************
 * \brief
 *    Writes CBP, DQUANT of an macroblock
 ************************************************************************
 */
int write_CBP_and_Dquant (Macroblock* currMB)
{
  Slice* currSlice = currMB->p_Slice;  

  int             rate      = 0;
  BitCounter      *mbBits = &currMB->bits;
  SyntaxElement   se;
  const int*      partMap   = assignSE2partition[currSlice->partition_mode];
  int             cbp       = currMB->cbp;
  DataPartition*  dataPart;  
  
  se.value2 = 0; // Initialize value2 to avoid problems

  if (currMB->mb_type != I16MB)
  {
    VideoParameters *p_Vid = currMB->p_Vid;
    int  need_transform_size_flag;   //ADD-VG-24062004
    //=====   C B P   =====
    //---------------------
    se.value1 = cbp;    
    se.type   = SE_CBP;

    // choose the appropriate data partition
    dataPart = &(currSlice->partArr[partMap[se.type]]);

#if TRACE
    snprintf(se.tracestring, TRACESTRING_SIZE, "CBP (%2d,%2d) = %3d",currMB->mb_x, currMB->mb_y, cbp);
#endif
    currSlice->writeCBP (currMB, &se, dataPart);

    currMB->bits.mb_cbp = currMB->bits.mb_cbp + (unsigned short) se.len;
    rate               += se.len;

    //============= Transform Size Flag for INTER MBs =============
    //-------------------------------------------------------------
    need_transform_size_flag = (((currMB->mb_type >= 1 && currMB->mb_type <= 3)||
      (currSlice->slice_type == B_SLICE && currMB->mb_type == 0 && p_Vid->active_sps->direct_8x8_inference_flag) ||
      (currMB->NoMbPartLessThan8x8Flag))
      && currMB->mb_type != I8MB && currMB->mb_type != I4MB
      && (currMB->cbp&15 || currSlice->cmp_cbp[1]&15 || currSlice->cmp_cbp[2] & 15)
      && currSlice->p_Inp->Transform8x8Mode);

    if (need_transform_size_flag)
    {
      se.value1 = currMB->luma_transform_size_8x8_flag;
      se.type   = SE_MBTYPE;

#if TRACE
      snprintf(se.tracestring, TRACESTRING_SIZE, "transform_size_8x8_flag = %3d", currMB->luma_transform_size_8x8_flag);
#endif
      currSlice->writeMB_transform_size(currMB, &se, dataPart);

      mbBits->mb_mode = mbBits->mb_mode + (unsigned short) se.len;
      rate                   += se.len;
    }
  }

  //=====   DQUANT   =====
  //----------------------
  if (cbp!=0 || (currMB->mb_type == I16MB))
  {
    se.value1 = currMB->qp - currMB->prev_qp;
    se.type = SE_DELTA_QUANT;

    // choose the appropriate data partition
    dataPart = &(currSlice->partArr[partMap[se.type]]);
#if TRACE
    snprintf(se.tracestring, TRACESTRING_SIZE, "Delta QP (%2d,%2d) = %3d",currMB->mb_x, currMB->mb_y, se.value1);
#endif
    currSlice->writeDquant (currMB, &se, dataPart);
    currMB->bits.mb_delta_quant = currMB->bits.mb_delta_quant + (unsigned short) se.len;
    rate                       += se.len;
  }

  return rate;
}

/*!
************************************************************************
* \brief
*    Write Luma Coeffcients, as well as Cb and Cr Coefficients in the 
*    case of 444 common mode, of an macroblock
************************************************************************
*/
int writeCoeff16x16_CAVLC (Macroblock* currMB, ColorPlane plane)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;

  int cbp = currMB->cbp;

  int   i, k;
  int   rate = 0;

  int   b8, data_type; 
  int *mb_bits_coeff = ((plane==0) ? &currMB->bits.mb_y_coeff : ((plane==1) ? &currMB->bits.mb_cb_coeff : &currMB->bits.mb_cr_coeff));

  if (currSlice->P444_joined)
  {
    for (i=0; i < 4; i++)
      for (k=4*plane; k<4*(plane+1); k++)
        p_Vid->nz_coeff [currMB->mbAddrX][i][k] = 0;
  }
  else
  {
    reset_mb_nz_coeff(p_Vid, currMB->mbAddrX);
  }


  if (currMB->mb_type != I16MB)
  {
    //=====  L U M I N A N C E   =====
    //--------------------------------
    for (i=0; i<4; i++)  
    {
      if (cbp & (1<<i))
      {
        rate += writeCoeff8x8 (currMB, plane, i, currMB->b8x8[i].mode, currMB->luma_transform_size_8x8_flag);
      }
    }
  }
  else
  {
    //=====  L U M I N A N C E   f o r   1 6 x 1 6   =====
    //----------------------------------------------------
    // DC coeffs
    switch (plane)
    {
    case 0:
    default:
      data_type = LUMA_INTRA16x16DC;
      break;
    case 1:
      data_type = CB_INTRA16x16DC;
      break;
    case 2:
      data_type = CR_INTRA16x16DC;
      break;            
    }
    rate += currSlice->writeCoeff4x4_CAVLC (currMB, data_type, 0, 0, 0);  // CAVLC
    

    // AC coeffs
    if (cbp & 15)
    {
      switch (plane)
      {
      case 0:
      default:
        data_type = LUMA_INTRA16x16AC;
        break;
      case 1:
        data_type = CB_INTRA16x16AC;
        break;
      case 2:
        data_type = CR_INTRA16x16AC;
        break;            
      }

      for (b8 = 0; b8 < 4; b8 ++)
      {
        rate += currSlice->writeCoeff4x4_CAVLC (currMB, data_type, b8, 0, 0);
        rate += currSlice->writeCoeff4x4_CAVLC (currMB, data_type, b8, 1, 0);
        rate += currSlice->writeCoeff4x4_CAVLC (currMB, data_type, b8, 2, 0);
        rate += currSlice->writeCoeff4x4_CAVLC (currMB, data_type, b8, 3, 0);
      }     
    }
  }
  mb_bits_coeff += rate;
  return rate;
}


/*!
************************************************************************
* \brief
*    Write Luma Coeffcients, as well as Cb and Cr Coefficients in the 
*    case of 444 common mode, of an macroblock
************************************************************************
*/
int writeCoeff16x16_CABAC (Macroblock* currMB, ColorPlane plane)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;

  int             cbp = currMB->cbp;

  int             mb_x, mb_y, i, j, k;
  int             level;
  int             rate      = 0;
  int             block_rate = 0;  
  SyntaxElement   se;
  const int*      partMap   = assignSE2partition[currSlice->partition_mode];
  int   pl_off = plane<<2; 
  DataPartition*  dataPart;

  int   b8, b4;
  int*  DCLevel = currSlice->cofDC[plane][0];
  int*  DCRun   = currSlice->cofDC[plane][1];
  int*  ACLevel;
  int*  ACRun;
  int   data_type; 
  int*  mb_bits_coeff = ((plane==0) ? &currMB->bits.mb_y_coeff : ((plane==1) ? &currMB->bits.mb_cb_coeff : &currMB->bits.mb_cr_coeff));

  if (currSlice->P444_joined)
  {
    for (i=0; i < 4; i++)
      for (k=4*plane; k<4*(plane+1); k++)
        p_Vid->nz_coeff [currMB->mbAddrX][i][k] = 0;
  }
  else
  {
    reset_mb_nz_coeff(p_Vid, currMB->mbAddrX);
  }


  if (currMB->mb_type != I16MB)
  {
    //=====  L U M I N A N C E   =====
    //--------------------------------
    if (cbp != 0)
    {
      for (i=0; i<4; i++)  
      {
        if (cbp & (1<<i))
        {
          rate += writeCoeff8x8 (currMB, plane, i, currMB->b8x8[i].mode, currMB->luma_transform_size_8x8_flag);
        }
      }
    }
  }
  else
  {
    //=====  L U M I N A N C E   f o r   1 6 x 1 6   =====
    //----------------------------------------------------
    // DC coeffs    
    currMB->is_intra_block = TRUE;
    switch (plane)
    {
    case 0:
    default:
      data_type = LUMA_16DC;
      break;
    case 1:
      data_type = CB_16DC;
      break;
    case 2:
      data_type = CR_16DC;
      break;            
    }

    se.context = data_type;
    se.type    = SE_LUM_DC_INTRA;   // element is of type DC

    // choose the appropriate data partition
    dataPart = &(currSlice->partArr[partMap[se.type]]);

    level = 1; // get inside loop
    for (k=0; k<=16 && level!=0; k++)
    {
      level = se.value1 = DCLevel[k]; // level
      se.value2 = DCRun  [k]; // run   
#if TRACE
      trace_coeff(&se, plane, "16x16 DC", k, level, se.value2);
#endif
      writeRunLevel_CABAC(currMB, &se, dataPart);
      block_rate += se.len;
    }
    *mb_bits_coeff += block_rate;
    rate += block_rate;

    // AC coeffs
    if (cbp & 15)
    {
      switch (plane)
      {
      case 0:
      default:
        data_type = LUMA_16AC;
        break;
      case 1:
        data_type = CB_16AC;
        break;
      case 2:
        data_type = CR_16AC;
        break;            
      }

      se.context = data_type;
      se.type    = SE_LUM_AC_INTRA;   // element is of type AC

      // choose the appropriate data partition
      dataPart = &(currSlice->partArr[partMap[se.type]]);
      block_rate = 0;

      for (mb_y = 0; mb_y < 4; mb_y += 2)
      {
        for (mb_x = 0; mb_x < 4; mb_x += 2)
        {
          for (j = mb_y; j < mb_y + 2; j++)
          {              
            int j1 = 2*(j >> 1);
            int j2 = 2*(j & 0x01);
            currMB->subblock_y = (short) (j << 2);

            for (i=mb_x; i < mb_x+2; i++)
            {
              currMB->subblock_x = (short) (i << 2);
              b8      = j1 + (i >> 1) + pl_off;
              b4      = j2 + (i & 0x01);

              ACLevel = currSlice->cofAC[b8][b4][0];
              ACRun   = currSlice->cofAC[b8][b4][1];

              level=1; // get inside loop

              for (k=0; k < 16 && level !=0; k++)
              {
                level = se.value1 = ACLevel[k]; // level
                se.value2 = ACRun  [k]; // run
#if TRACE
                trace_coeff(&se, plane, "16x16 AC", k, level, se.value2);
#endif
                writeRunLevel_CABAC(currMB, &se, dataPart);
                block_rate += se.len;
              }
            }
          }
        }
      }

      rate += block_rate;
      *mb_bits_coeff += block_rate;
    }
  }

  return rate;
}

/*!
 ************************************************************************
 * \brief
 *    Get the Prediction from the Neighboring Blocks for Number of Nonzero Coefficients
 *
 *    Luma Blocks
 ************************************************************************
 */
int predict_nnz(Macroblock *currMB, int block_type, int i,int j)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  PixelPos pix;

  int pred_nnz = 0;
  int cnt      = 0;

  // left block
  get4x4Neighbour(currMB, (i << 2) - 1, (j << 2), p_Vid->mb_size[IS_LUMA], &pix);

  if (is_intra(currMB) && pix.available && p_Vid->active_pps->constrained_intra_pred_flag && ((currSlice->partition_mode != 0) && !currSlice->idr_flag))
  {
    pix.available &= p_Vid->intra_block[pix.mb_addr];
    if (!pix.available)
      cnt++;
  }

  if (pix.available)
  {
    switch (block_type)
    {
    case LUMA:
      pred_nnz += p_Vid->nz_coeff [pix.mb_addr ][pix.x][pix.y];
      cnt++;
      break;
    case CB:
      pred_nnz += p_Vid->nz_coeff [pix.mb_addr ][pix.x][4+pix.y];
      cnt++;
      break;
    case CR:
      pred_nnz += p_Vid->nz_coeff [pix.mb_addr ][pix.x][8+pix.y];
      cnt++;
      break;
    default:
      error("writeCoeff4x4_CAVLC: Invalid block type", 600);
      break;
    }
  }

  // top block
  get4x4Neighbour(currMB, (i<<2), (j<<2) - 1, p_Vid->mb_size[IS_LUMA], &pix);

  if (is_intra(currMB) && pix.available && p_Vid->active_pps->constrained_intra_pred_flag && ((currSlice->partition_mode != 0) && !currSlice->idr_flag))
  {
    pix.available &= p_Vid->intra_block[pix.mb_addr];
    if (!pix.available)
      cnt++;
  }

  if (pix.available)
  {
    switch (block_type)
    {
    case LUMA:
      pred_nnz += p_Vid->nz_coeff [pix.mb_addr ][pix.x][pix.y];
      cnt++;
      break;
    case CB:
      pred_nnz += p_Vid->nz_coeff [pix.mb_addr ][pix.x][4+pix.y];
      cnt++;
      break;
    case CR:
      pred_nnz += p_Vid->nz_coeff [pix.mb_addr ][pix.x][8+pix.y];
      cnt++;
      break;
    default:
      error("writeCoeff4x4_CAVLC: Invalid block type", 600);
      break;
    }
  }

  if (cnt==2)
  {
    pred_nnz++;
    pred_nnz>>=1;
  }

  return pred_nnz;
}


/*!
 ************************************************************************
 * \brief
 *    Get the Prediction from the Neighboring Blocks for Number of Nonzero Coefficients
 *
 *    Chroma Blocks
 ************************************************************************
 */
int predict_nnz_chroma(Macroblock *currMB, int i,int j)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  PixelPos pix;

  int pred_nnz = 0;
  int cnt      = 0;

  if (p_Vid->yuv_format != YUV444)
  {
    //YUV420 and YUV422
    // left block
    get4x4Neighbour(currMB, ((i & 0x01)<<2) - 1, ((j-4)<<2), p_Vid->mb_size[IS_CHROMA], &pix);

    if (is_intra(currMB) && pix.available && p_Vid->active_pps->constrained_intra_pred_flag && ((currSlice->partition_mode != 0) && !currSlice->idr_flag))
    {
      pix.available &= p_Vid->intra_block[pix.mb_addr];
      if (!pix.available)
        cnt++;
    }

    if (pix.available)
    {
      pred_nnz = p_Vid->nz_coeff [pix.mb_addr ][2 * (i >> 1) + pix.x][4 + pix.y];
      cnt++;
    }

    // top block
    get4x4Neighbour(currMB, ((i & 0x01)<<2), ((j-4)<<2) -1, p_Vid->mb_size[IS_CHROMA],  &pix);

    if (is_intra(currMB) && pix.available && p_Vid->active_pps->constrained_intra_pred_flag && ((currSlice->partition_mode != 0) && !currSlice->idr_flag))
    {
      pix.available &= p_Vid->intra_block[pix.mb_addr];
      if (!pix.available)
        cnt++;
    }

    if (pix.available)
    {
      pred_nnz += p_Vid->nz_coeff [pix.mb_addr ][2 * (i >> 1) + pix.x][4 + pix.y];
      cnt++;
    }
  }


  if (cnt==2)
  {
    pred_nnz++;
    pred_nnz>>=1;
  }

  return pred_nnz;
}

/*!
 ************************************************************************
 * \brief
 *    Writes coeff of an 4x4 block (CAVLC)
 *
 * \author
 *    Karl Lillevold <karll@real.com>
 *    contributions by James Au <james@ubvideo.com>
 ************************************************************************
 */
int writeCoeff4x4_CAVLC_normal (Macroblock* currMB, int block_type, int b8, int b4, int param)
{
  Slice* currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  int           no_bits    = 0;
  SyntaxElement se;
  DataPartition *dataPart;
  const int           *partMap   = assignSE2partition[currSlice->partition_mode];

  int k,level = 1,run = 0, vlcnum;
  int numcoeff = 0, lastcoeff = 0, numtrailingones = 0; 
  int numones = 0, totzeros = 0, zerosleft, numcoef;
  int numcoeff_vlc;
  int code, level_two_or_higher;
  int dptype = 0;
  int nnz, max_coeff_num = 0, cdc = 0, cac = 0;
  int subblock_x, subblock_y;
  int *mb_bits_coeff = &currMB->bits.mb_y_coeff;
#if TRACE
  char type[15];
#endif

  static const int incVlc[] = {0, 3, 6, 12, 24, 48, 32768};  // maximum vlc = 6


  int*  pLevel = NULL;
  int*  pRun = NULL;

  switch (block_type)
  {
  case LUMA:
    max_coeff_num = 16;

    pLevel = currSlice->cofAC[b8][b4][0];
    pRun   = currSlice->cofAC[b8][b4][1];
#if TRACE
    sprintf(type, "%s", "Luma");
#endif
    dptype = (is_intra (currMB)) ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER;
    break;

  case CHROMA_AC:
    max_coeff_num = 15;
    mb_bits_coeff = &currMB->bits.mb_uv_coeff;
    cac = 1;

    pLevel = currSlice->cofAC[b8][b4][0];
    pRun   = currSlice->cofAC[b8][b4][1];
#if TRACE
    sprintf(type, "%s", "ChrAC");
#endif
    dptype = (is_intra (currMB)) ? SE_CHR_AC_INTRA : SE_CHR_AC_INTER;
    break;

  case CHROMA_DC:
    max_coeff_num = p_Vid->num_cdc_coeff;
    mb_bits_coeff = &currMB->bits.mb_uv_coeff;
    cdc = 1;

    pLevel = currSlice->cofDC[param + 1][0];
    pRun   = currSlice->cofDC[param + 1][1];
#if TRACE
    sprintf(type, "%s", "ChrDC");
#endif
    dptype = (is_intra (currMB)) ? SE_CHR_DC_INTRA : SE_CHR_DC_INTER;
    break;

  case LUMA_INTRA16x16AC:
    max_coeff_num = 15;

    pLevel = currSlice->cofAC[b8][b4][0];
    pRun   = currSlice->cofAC[b8][b4][1];
#if TRACE
    sprintf(type, "%s", "Lum16AC");
#endif
    dptype = SE_LUM_AC_INTRA;
    break;

  case LUMA_INTRA16x16DC:
    max_coeff_num = 16;

    pLevel = currSlice->cofDC[0][0];
    pRun   = currSlice->cofDC[0][1];
#if TRACE
    sprintf(type, "%s", "Lum16DC");
#endif
    dptype = SE_LUM_DC_INTRA;
    break;


  default:
    error("writeCoeff4x4_CAVLC: Invalid block type", 600);
    break;
  }

  dataPart = &(currSlice->partArr[partMap[dptype]]);

  for(k = 0; (k <= ((cdc) ? p_Vid->num_cdc_coeff : 16)) && level != 0; k++)
  {
    level = pLevel[k]; // level
    run   = pRun[k];   // run

    if (level)
    {

      totzeros += run; 
      if (iabs(level) == 1)
      {
        numones ++;
        numtrailingones ++;
        numtrailingones = imin(numtrailingones, 3); // clip to 3
      }
      else
      {
        numtrailingones = 0;
      }
      numcoeff ++;
      lastcoeff = k;
    }
  }

  if (!cdc)
  {
    if (!cac)
    {
      // luma
      subblock_x = ((b8 & 0x1) == 0) ? (((b4 & 0x1) == 0) ? 0 : 1) : (((b4 & 0x1) == 0) ? 2 : 3);
      // horiz. position for coeff_count context
      subblock_y = (b8 < 2) ? ((b4 < 2) ? 0 : 1) : ((b4 < 2) ? 2 : 3);
      // vert.  position for coeff_count context
      nnz = predict_nnz(currMB, LUMA, subblock_x,subblock_y);
    }
    else
    {
      // chroma AC
      subblock_x = param >> 4;
      subblock_y = param & 15;
      nnz = predict_nnz_chroma(currMB, subblock_x, subblock_y);
    }
    p_Vid->nz_coeff [currMB->mbAddrX ][subblock_x][subblock_y] = numcoeff;

    numcoeff_vlc = (nnz < 2) ? 0 : ((nnz < 4) ? 1 : ((nnz < 8) ? 2 : 3));
  }
  else
  {
    // chroma DC (has its own VLC)
    // numcoeff_vlc not relevant
    numcoeff_vlc = 0;

    subblock_x = param;
    subblock_y = param;
  }

  se.type  = dptype;

  se.value1 = numcoeff;
  se.value2 = numtrailingones;
  se.len    = numcoeff_vlc; /* use len to pass vlcnum */

#if TRACE
  snprintf(se.tracestring,
    TRACESTRING_SIZE, "%s # c & tr.1s(%d,%d) vlc=%d #c=%d #t1=%d",
    type, subblock_x, subblock_y, numcoeff_vlc, numcoeff, numtrailingones);
#endif

  if (!cdc)
    writeSyntaxElement_NumCoeffTrailingOnes(&se, dataPart);
  else
    writeSyntaxElement_NumCoeffTrailingOnesChromaDC(p_Vid, &se, dataPart);

  *mb_bits_coeff += se.len;
  no_bits                += se.len;

  if (!numcoeff)
    return no_bits;

  if (numcoeff)
  {
    code = 0;
    for (k = lastcoeff; k > lastcoeff - numtrailingones; k--)
    {
      level = pLevel[k]; // level
#ifdef  _DEBUG
      if (iabs(level) > 1)
      {
        printf("ERROR: level > 1\n");
        exit(-1);
      }
#endif
      code <<= 1;

      code |= (level < 0);
    }

    if (numtrailingones)
    {
      se.type  = dptype;

      se.value2 = numtrailingones;
      se.value1 = code;

#if TRACE
      snprintf(se.tracestring,
        TRACESTRING_SIZE, "%s trailing ones sign (%d,%d)",
        type, subblock_x, subblock_y);
#endif

      writeSyntaxElement_VLC (&se, dataPart);
      *mb_bits_coeff += se.len;
      no_bits                += se.len;

    }

    // encode levels
    level_two_or_higher = (numcoeff > 3 && numtrailingones == 3) ? 0 : 1;

    vlcnum = (numcoeff > 10 && numtrailingones < 3) ? 1 : 0;

    for (k = lastcoeff - numtrailingones; k >= 0; k--)
    {
      level = pLevel[k]; // level

      se.value1 = level;
      se.type  = dptype;

#if TRACE
      snprintf(se.tracestring,
        TRACESTRING_SIZE, "%s lev (%d,%d) k=%d vlc=%d lev=%3d",
        type, subblock_x, subblock_y, k, vlcnum, level);
#endif

      if (level_two_or_higher)
      {
        level_two_or_higher = 0;

        if (se.value1 > 0)
          se.value1 --;
        else
          se.value1 ++;        
      }

      //    encode level

      if (vlcnum == 0)
        writeSyntaxElement_Level_VLC1(&se, dataPart, p_Vid->active_sps->profile_idc);
      else
        writeSyntaxElement_Level_VLCN(&se, vlcnum, dataPart, p_Vid->active_sps->profile_idc);

      // update VLC table
      if (iabs(level) > incVlc[vlcnum])
        vlcnum++;

      if ((k == lastcoeff - numtrailingones) && iabs(level) > 3)
        vlcnum = 2;

      *mb_bits_coeff += se.len;
      no_bits                += se.len;
    }

    // encode total zeroes
    if (numcoeff < max_coeff_num)
    {

      se.type  = dptype;
      se.value1 = totzeros;

      vlcnum = numcoeff - 1;

      se.len = vlcnum;

#if TRACE
      snprintf(se.tracestring,
        TRACESTRING_SIZE, "%s totalrun (%d,%d) vlc=%d totzeros=%3d",
        type, subblock_x, subblock_y, vlcnum, totzeros);
#endif
      if (!cdc)
        writeSyntaxElement_TotalZeros(&se, dataPart);
      else
        writeSyntaxElement_TotalZerosChromaDC(p_Vid, &se, dataPart);

      *mb_bits_coeff += se.len;
      no_bits                += se.len;
    }

    // encode run before each coefficient
    zerosleft = totzeros;
    numcoef = numcoeff;
    for (k = lastcoeff; k >= 0; k--)
    {
      run = pRun[k]; // run

      se.value1 = run;
      se.type   = dptype;

      // for last coeff, run is remaining totzeros
      // when zerosleft is zero, remaining coeffs have 0 run
      if ((!zerosleft) || (numcoeff <= 1 ))
        break;

      if (numcoef > 1 && zerosleft)
      {
        vlcnum = imin(zerosleft - 1, RUNBEFORE_NUM_M1);
        se.len = vlcnum;

#if TRACE
        snprintf(se.tracestring,
          TRACESTRING_SIZE, "%s run (%d,%d) k=%d vlc=%d run=%2d",
          type, subblock_x, subblock_y, k, vlcnum, run);
#endif

        writeSyntaxElement_Run(&se, dataPart);

        *mb_bits_coeff += se.len;
        no_bits                += se.len;

        zerosleft -= run;
        numcoef --;
      }
    }
  }

  return no_bits;
}

/*!
 ************************************************************************
 * \brief
 *    Writes coeff of an 4x4 block (CAVLC)
 *
 * \author
 *    Karl Lillevold <karll@real.com>
 *    contributions by James Au <james@ubvideo.com>
 ************************************************************************
 */
int writeCoeff4x4_CAVLC_444 (Macroblock* currMB, int block_type, int b8, int b4, int param)
{
  Slice* currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  int      no_bits    = 0;
  SyntaxElement se;
  DataPartition *dataPart;
  const int *partMap   = assignSE2partition[currSlice->partition_mode];

  int k,level = 1,run = 0, vlcnum;
  int numcoeff = 0, lastcoeff = 0, numtrailingones = 0; 
  int numones = 0, totzeros = 0, zerosleft, numcoef;
  int numcoeff_vlc;
  int code, level_two_or_higher;
  int dptype = 0;
  int nnz, max_coeff_num = 0, cdc = 0, cac = 0;
  int subblock_x, subblock_y;
  int *mb_bits_coeff = &currMB->bits.mb_y_coeff;
#if TRACE
  char type[15];
#endif

  static const int incVlc[] = {0, 3, 6, 12, 24, 48, 32768};  // maximum vlc = 6

  int*  pLevel = NULL;
  int*  pRun = NULL;

  switch (block_type)
  {
  case LUMA:
    max_coeff_num = 16;

    pLevel = currSlice->cofAC[b8][b4][0];
    pRun   = currSlice->cofAC[b8][b4][1];
#if TRACE
    sprintf(type, "%s", "Luma");
#endif
    dptype = (is_intra (currMB)) ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER;
    break;
  case LUMA_INTRA16x16DC:
    max_coeff_num = 16;

    pLevel = currSlice->cofDC[0][0];
    pRun   = currSlice->cofDC[0][1];
#if TRACE
    sprintf(type, "%s", "Lum16DC");
#endif
    dptype = SE_LUM_DC_INTRA;
    break;
  case LUMA_INTRA16x16AC:
    max_coeff_num = 15;

    pLevel = currSlice->cofAC[b8][b4][0];
    pRun   = currSlice->cofAC[b8][b4][1];
#if TRACE
    sprintf(type, "%s", "Lum16AC");
#endif
    dptype = SE_LUM_AC_INTRA;
    break;
  case CB:
    max_coeff_num = 16;
    mb_bits_coeff = &currMB->bits.mb_cb_coeff;

    pLevel = currSlice->cofAC[4+b8][b4][0];
    pRun   = currSlice->cofAC[4+b8][b4][1];
#if TRACE    
    sprintf(type, "%s", "CB");
#endif
    dptype = (is_intra (currMB)) ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER;
    break;
  case CB_INTRA16x16DC:
    max_coeff_num = 16;
    mb_bits_coeff = &currMB->bits.mb_cb_coeff;

    pLevel = currSlice->cofDC[1][0];
    pRun   = currSlice->cofDC[1][1];
#if TRACE    
    sprintf(type, "%s", "CB_16DC");
#endif
    dptype = SE_LUM_DC_INTRA;
    break;
  case CB_INTRA16x16AC:
    max_coeff_num = 15;
    mb_bits_coeff = &currMB->bits.mb_cb_coeff;

    pLevel = currSlice->cofAC[4+b8][b4][0];
    pRun   = currSlice->cofAC[4+b8][b4][1];
#if TRACE    
    sprintf(type, "%s", "CB_16AC");
#endif
    dptype = SE_LUM_AC_INTRA;
    break;

  case CR:
    max_coeff_num = 16;
    mb_bits_coeff = &currMB->bits.mb_cr_coeff;

    pLevel = currSlice->cofAC[8+b8][b4][0];
    pRun   = currSlice->cofAC[8+b8][b4][1];
#if TRACE    
    sprintf(type, "%s", "CR");
#endif
    dptype = (is_intra (currMB)) ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER;
    break;
  case CR_INTRA16x16DC:
    max_coeff_num = 16;
    mb_bits_coeff = &currMB->bits.mb_cr_coeff;

    pLevel = currSlice->cofDC[2][0];
    pRun   = currSlice->cofDC[2][1];
#if TRACE    
    sprintf(type, "%s", "CR_16DC");
#endif
    dptype = SE_LUM_DC_INTRA;
    break;
  case CR_INTRA16x16AC:
    max_coeff_num = 15;
    mb_bits_coeff = &currMB->bits.mb_cr_coeff;

    pLevel = currSlice->cofAC[8+b8][b4][0];
    pRun   = currSlice->cofAC[8+b8][b4][1];
#if TRACE    
    sprintf(type, "%s", "CR_16AC");
#endif
    dptype = SE_LUM_AC_INTRA;
    break;

  case CHROMA_DC:
    max_coeff_num = p_Vid->num_cdc_coeff;
    mb_bits_coeff = &currMB->bits.mb_uv_coeff;
    cdc = 1;

    pLevel = currSlice->cofDC[param+1][0];
    pRun   = currSlice->cofDC[param+1][1];
#if TRACE
    sprintf(type, "%s", "ChrDC");
#endif
    dptype = (is_intra (currMB)) ? SE_CHR_DC_INTRA : SE_CHR_DC_INTER;
    break;
  case CHROMA_AC:
    max_coeff_num = 15;
    mb_bits_coeff = &currMB->bits.mb_uv_coeff;
    cac = 1;

    pLevel = currSlice->cofAC[b8][b4][0];
    pRun   = currSlice->cofAC[b8][b4][1];
#if TRACE
    sprintf(type, "%s", "ChrAC");
#endif
    dptype = (is_intra (currMB)) ? SE_CHR_AC_INTRA : SE_CHR_AC_INTER;
    break;
  default:
    error("writeCoeff4x4_CAVLC: Invalid block type", 600);
    break;
  }

  dataPart = &(currSlice->partArr[partMap[dptype]]);

  for(k = 0; (k <= ((cdc) ? p_Vid->num_cdc_coeff : 16)) && level != 0; k++)
  {
    level = pLevel[k]; // level
    run   = pRun[k];   // run

    if (level)
    {

      totzeros += run; // lets add run always (even if zero) to avoid conditional
      if (iabs(level) == 1)
      {
        numones ++;
        numtrailingones ++;
        numtrailingones = imin(numtrailingones, 3); // clip to 3
      }
      else
      {
        numtrailingones = 0;
      }
      numcoeff ++;
      lastcoeff = k;
    }
  }

  if (!cdc)
  {
    if(block_type==LUMA || block_type==LUMA_INTRA16x16DC || block_type==LUMA_INTRA16x16AC
      ||block_type==CHROMA_AC)
    {
      if (!cac)
      {
        // luma
        subblock_x = ((b8 & 0x1) == 0) ? (((b4 & 0x1) == 0) ? 0 : 1) : (((b4 & 0x1) == 0) ? 2 : 3);
        // horiz. position for coeff_count context
        subblock_y = (b8 < 2) ? ((b4 < 2) ? 0 : 1) : ((b4 < 2) ? 2 : 3);
        // vert.  position for coeff_count context
        nnz = predict_nnz(currMB, LUMA, subblock_x,subblock_y);
      }
      else
      {
        // chroma AC
        subblock_x = param >> 4;
        subblock_y = param & 15;
        nnz = predict_nnz_chroma(currMB, subblock_x, subblock_y);
      }
      p_Vid->nz_coeff [currMB->mbAddrX ][subblock_x][subblock_y] = numcoeff;
    }
    else if (block_type==CB || block_type==CB_INTRA16x16DC 
      || block_type==CB_INTRA16x16AC)
    {   //CB in the common mode in 4:4:4 profiles
      subblock_x = ((b8 & 0x1) == 0)?(((b4 & 0x1) == 0) ? 0 : 1):(((b4 & 0x1) == 0) ? 2 : 3); 
      // horiz. position for coeff_count context
      subblock_y = (b8 < 2) ? ((b4 < 2) ? 0 : 1) : ((b4 < 2) ? 2 : 3); 
      // vert.  position for coeff_count context
      nnz = predict_nnz(currMB, CB, subblock_x,subblock_y);
      p_Vid->nz_coeff [currMB->mbAddrX ][subblock_x][4+subblock_y] = numcoeff;
    }
    else
    { //CR in the common mode in 4:4:4 profiles 
      subblock_x = ((b8 & 0x1) == 0)?(((b4 & 0x1) == 0) ? 0 : 1) : (((b4 & 0x1) == 0) ? 2 : 3); 
      // horiz. position for coeff_count context
      subblock_y = (b8 < 2) ? ((b4 < 2) ? 0 : 1) : ((b4 < 2) ? 2 : 3); 
      // vert.  position for coeff_count context
      nnz = predict_nnz(currMB, CR, subblock_x,subblock_y);
      p_Vid->nz_coeff [currMB->mbAddrX ][subblock_x][8+subblock_y] = numcoeff;
    }

    numcoeff_vlc = (nnz < 2) ? 0 : ((nnz < 4) ? 1 : ((nnz < 8) ? 2 : 3));
  }
  else
  {
    // chroma DC (has its own VLC)
    // numcoeff_vlc not relevant
    numcoeff_vlc = 0;

    subblock_x = param;
    subblock_y = param;
  }

  se.type  = dptype;

  se.value1 = numcoeff;
  se.value2 = numtrailingones;
  se.len    = numcoeff_vlc; /* use len to pass vlcnum */

#if TRACE
  snprintf(se.tracestring,
    TRACESTRING_SIZE, "%s # c & tr.1s(%d,%d) vlc=%d #c=%d #t1=%d",
    type, subblock_x, subblock_y, numcoeff_vlc, numcoeff, numtrailingones);
#endif

  if (!cdc)
    writeSyntaxElement_NumCoeffTrailingOnes(&se, dataPart);
  else
    writeSyntaxElement_NumCoeffTrailingOnesChromaDC(p_Vid, &se, dataPart);

  *mb_bits_coeff += se.len;
  no_bits                += se.len;

  if (!numcoeff)
    return no_bits;

  if (numcoeff)
  {
    code = 0;
    for (k = lastcoeff; k > lastcoeff - numtrailingones; k--)
    {
      level = pLevel[k]; // level
      
      if (iabs(level) > 1)
      {
        printf("ERROR: level > 1\n");
        exit(-1);
      }

      code <<= 1;

      if (level < 0)
      {
        code |= 0x1;
      }
    }

    if (numtrailingones)
    {
      se.type  = dptype;

      se.value2 = numtrailingones;
      se.value1 = code;

#if TRACE
      snprintf(se.tracestring,
        TRACESTRING_SIZE, "%s trailing ones sign (%d,%d)",
        type, subblock_x, subblock_y);
#endif

      writeSyntaxElement_VLC (&se, dataPart);
      *mb_bits_coeff += se.len;
      no_bits                += se.len;

    }

    // encode levels
    level_two_or_higher = (numcoeff > 3 && numtrailingones == 3) ? 0 : 1;

    vlcnum = (numcoeff > 10 && numtrailingones < 3) ? 1 : 0;

    for (k = lastcoeff - numtrailingones; k >= 0; k--)
    {
      level = pLevel[k]; // level

      se.value1 = level;
      se.type  = dptype;

#if TRACE
      snprintf(se.tracestring,
        TRACESTRING_SIZE, "%s lev (%d,%d) k=%d vlc=%d lev=%3d",
        type, subblock_x, subblock_y, k, vlcnum, level);
#endif

      if (level_two_or_higher)
      {
        level_two_or_higher = 0;

        if (se.value1 > 0)
          se.value1 --;
        else
          se.value1 ++;        
      }

      //    encode level

      if (vlcnum == 0)
        writeSyntaxElement_Level_VLC1(&se, dataPart, p_Vid->active_sps->profile_idc);
      else
        writeSyntaxElement_Level_VLCN(&se, vlcnum, dataPart, p_Vid->active_sps->profile_idc);

      // update VLC table
      if (iabs(level) > incVlc[vlcnum])
        vlcnum++;

      if ((k == lastcoeff - numtrailingones) && iabs(level) > 3)
        vlcnum = 2;

      *mb_bits_coeff += se.len;
      no_bits                += se.len;
    }

    // encode total zeroes
    if (numcoeff < max_coeff_num)
    {

      se.type  = dptype;
      se.value1 = totzeros;

      vlcnum = numcoeff - 1;

      se.len = vlcnum;

#if TRACE
      snprintf(se.tracestring,
        TRACESTRING_SIZE, "%s totalrun (%d,%d) vlc=%d totzeros=%3d",
        type, subblock_x, subblock_y, vlcnum, totzeros);
#endif
      if (!cdc)
        writeSyntaxElement_TotalZeros(&se, dataPart);
      else
        writeSyntaxElement_TotalZerosChromaDC(p_Vid, &se, dataPart);

      *mb_bits_coeff += se.len;
      no_bits                += se.len;
    }

    // encode run before each coefficient
    zerosleft = totzeros;
    numcoef = numcoeff;
    for (k = lastcoeff; k >= 0; k--)
    {
      run = pRun[k]; // run

      se.value1 = run;
      se.type   = dptype;

      // for last coeff, run is remaining totzeros
      // when zerosleft is zero, remaining coeffs have 0 run
      if ((!zerosleft) || (numcoeff <= 1 ))
        break;

      if (numcoef > 1 && zerosleft)
      {
        vlcnum = imin(zerosleft - 1, RUNBEFORE_NUM_M1);
        se.len = vlcnum;

#if TRACE
        snprintf(se.tracestring,
          TRACESTRING_SIZE, "%s run (%d,%d) k=%d vlc=%d run=%2d",
          type, subblock_x, subblock_y, k, vlcnum, run);
#endif

        writeSyntaxElement_Run(&se, dataPart);

        *mb_bits_coeff += se.len;
        no_bits                += se.len;

        zerosleft -= run;
        numcoef --;
      }
    }
  }

  return no_bits;
}
/*!
 ************************************************************************
 * \brief
 *    Change color plane for 4:4:4 Independent Mode
 *
 * \par Input:
 *    plane number
 ************************************************************************/
void change_plane_JV( VideoParameters *p_Vid, int nplane )
{
  p_Vid->colour_plane_id = (char) nplane;
  p_Vid->mb_data         = p_Vid->mb_data_JV[nplane];
  p_Vid->enc_picture     = p_Vid->enc_frame_picture_JV[nplane];
  p_Vid->pCurImg         = p_Vid->imgData.frm_data[nplane];
}

/*!
 ************************************************************************
 * \brief
 *    Make one frame picture from 4:4:4 plane
 ************************************************************************/
void make_frame_picture_JV(VideoParameters *p_Vid)
{
  int uv, line;
  int nsize;

  p_Vid->enc_frame_picture[0] = p_Vid->enc_frame_picture_JV[0];

  //copy;
  if(p_Vid->currentSlice->nal_reference_idc) 
  {
    nsize = (p_Vid->enc_frame_picture_JV[0]->size_y/BLOCK_SIZE)*(p_Vid->enc_frame_picture_JV[0]->size_x/BLOCK_SIZE)*sizeof(PicMotionParams);
    memcpy( &(p_Vid->enc_frame_picture[0]->JVmv_info[PLANE_Y][0][0]), &(p_Vid->enc_frame_picture_JV[PLANE_Y]->mv_info[0][0]), nsize);
    memcpy( &(p_Vid->enc_frame_picture[0]->JVmv_info[PLANE_U][0][0]), &(p_Vid->enc_frame_picture_JV[PLANE_U]->mv_info[0][0]), nsize);
    memcpy( &(p_Vid->enc_frame_picture[0]->JVmv_info[PLANE_V][0][0]), &(p_Vid->enc_frame_picture_JV[PLANE_V]->mv_info[0][0]), nsize);
  }

  for( uv=0; uv<2; uv++ )
  {
    for( line=0; line<p_Vid->height; line++ )
    {
      nsize = sizeof(imgpel) * p_Vid->width;
      memcpy( p_Vid->enc_frame_picture[0]->imgUV[uv][line], p_Vid->enc_frame_picture_JV[uv+1]->imgY[line], nsize );
    }
    free_storable_picture(p_Vid, p_Vid->enc_frame_picture_JV[uv+1]);
  }

}

Macroblock *alloc_mbs(VideoParameters *p_Vid, int mb_num, int layers)
{
  int i, j;
  Macroblock *pMBs = (Macroblock *)calloc(mb_num, sizeof(Macroblock)), *pMB;
  if(!pMBs)
    return NULL;
  for(i=0; i<mb_num; i++)
  {
    pMB = pMBs+i;
    for (j=0; j<2; j++)
    {
      get_mem2Dpel(&pMB->intra4x4_pred_buf[j], 3, 17);
      get_mem2Dpel(&pMB->intra8x8_pred_buf[j], 3, 25);
      get_mem2Dpel(&pMB->intra16x16_pred_buf[j], 3, 33);
    }
    pMB->intra4x4_pred = pMB->intra4x4_pred_buf[0];
    pMB->intra8x8_pred = pMB->intra8x8_pred_buf[0];
    pMB->intra16x16_pred = pMB->intra16x16_pred_buf[0];
  }
  return pMBs;
}

void setup_mbs(Macroblock *pMBs, int mb_num, int layer_id)
{
  int i;
  Macroblock *pMB;
  for(i=0; i<mb_num; i++)
  {
    pMB = pMBs+i;
    pMB->intra4x4_pred = pMB->intra4x4_pred_buf[layer_id];
    pMB->intra8x8_pred = pMB->intra8x8_pred_buf[layer_id];
    pMB->intra16x16_pred = pMB->intra16x16_pred_buf[layer_id];
  }
}

void free_mbs(Macroblock *pMBs, int mb_num)
{
  int i, j;
  Macroblock *pMB;
  if(!pMBs)
    return;
  for(i=0; i<mb_num; i++)
  {
    pMB = pMBs+i;
    for (j=0; j<2; j++)
    {
      free_mem2Dpel(pMB->intra4x4_pred_buf[j]);
      free_mem2Dpel(pMB->intra8x8_pred_buf[j]);
      free_mem2Dpel(pMB->intra16x16_pred_buf[j]);
    }
    pMB->intra4x4_pred = NULL;
    pMB->intra8x8_pred = NULL;
    pMB->intra16x16_pred = NULL;
  }
  free(pMBs);
}
