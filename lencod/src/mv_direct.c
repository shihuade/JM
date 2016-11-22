
/*!
 *************************************************************************************
 * \file mv_direct.c
 *
 * \brief
 *    Direct Motion Vector Generation
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Alexis Michael Tourapis         <alexismt@ieee.org>
 *
 *************************************************************************************
*/

#include "contributors.h"

#include <math.h>
#include <limits.h>
#include <time.h>

#include "global.h"

#include "image.h"
#include "mv_search.h"
#include "refbuf.h"
#include "memalloc.h"
#include "mb_access.h"
#include "macroblock.h"
#include "mc_prediction.h"
#include "conformance.h"
#include "mode_decision.h"

/*!
 ************************************************************************
 * \brief
 *    Calculate Temporal Direct Mode Motion Vectors
 ************************************************************************
 */
void Get_Direct_MV_Temporal (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice; 
  int   block_x, block_y, pic_block_x, pic_block_y, opic_block_x, opic_block_y;
  MotionVector *****all_mvs;
  int   mv_scale;
  int refList;
  int ref_idx;
  VideoParameters *p_Vid = currMB->p_Vid;
  int list_offset = currMB->list_offset;

  StorablePicture **list1 = currSlice->listX[LIST_1 + list_offset];

  PicMotionParams colocated;

  //temporal direct mode copy from decoder
  for (block_y = 0; block_y < 4; block_y++)
  {
    pic_block_y  = currMB->block_y + block_y;
    opic_block_y = (currMB->opix_y >> 2) + block_y;

    for (block_x = 0; block_x < 4; block_x++)
    {
      pic_block_x  = currMB->block_x + block_x;
      opic_block_x = (currMB->pix_x>>2) + block_x;

      all_mvs = currSlice->all_mv;
      if (p_Vid->active_sps->direct_8x8_inference_flag)
      {
        if(currMB->p_Inp->separate_colour_plane_flag && currMB->p_Vid->yuv_format==YUV444)
          colocated = list1[0]->JVmv_info[currMB->p_Slice->colour_plane_id][RSD(opic_block_y)][RSD(opic_block_x)];
        else
          colocated = list1[0]->mv_info[RSD(opic_block_y)][RSD(opic_block_x)];
        if(currSlice->mb_aff_frame_flag && currMB->mb_field && currSlice->listX[LIST_1][0]->coded_frame)
        {
          int iPosBlkY;
          if(currSlice->listX[LIST_1][0]->motion.mb_field[currMB->mbAddrX] )
            iPosBlkY = (opic_block_y>>2)*8+4*(currMB->mbAddrX&1)+(opic_block_y&0x03);
          else
            iPosBlkY = RSD(opic_block_y)*2;

          if(colocated.ref_idx[LIST_0]>=0)
            colocated.ref_pic[LIST_0] = list1[0]->frame->mv_info[iPosBlkY][RSD(opic_block_x)].ref_pic[LIST_0];
          if(colocated.ref_idx[LIST_1]>=0)
            colocated.ref_pic[LIST_1] = list1[0]->frame->mv_info[iPosBlkY][RSD(opic_block_x)].ref_pic[LIST_1];
        }        
      }
      else
      {
        if(currMB->p_Inp->separate_colour_plane_flag && currMB->p_Vid->yuv_format==YUV444)
          colocated = list1[0]->JVmv_info[currMB->p_Slice->colour_plane_id][opic_block_y][opic_block_x];
        else
          colocated = list1[0]->mv_info[opic_block_y][opic_block_x];
      }
      if(currSlice->mb_aff_frame_flag)
      {
        if(!currMB->mb_field && ((currSlice->listX[LIST_1][0]->coded_frame && currSlice->listX[LIST_1][0]->motion.mb_field[currMB->mbAddrX]) ||
          (!currSlice->listX[LIST_1][0]->coded_frame)))
        {
          if (iabs(p_Vid->enc_picture->poc - currSlice->listX[LIST_1+4][0]->poc)> iabs(p_Vid->enc_picture->poc -currSlice->listX[LIST_1+2][0]->poc) )
          {
            if ( p_Vid->active_sps->direct_8x8_inference_flag)
            {
              if(currMB->p_Inp->separate_colour_plane_flag && currMB->p_Vid->yuv_format==YUV444)
                colocated = currSlice->listX[LIST_1+2][0]->JVmv_info[currMB->p_Slice->colour_plane_id][RSD(opic_block_y)>>1][RSD(opic_block_x)];
              else
                colocated = currSlice->listX[LIST_1+2][0]->mv_info[RSD(opic_block_y)>>1][RSD(opic_block_x)];
            }
            else
            { 
              if(currMB->p_Inp->separate_colour_plane_flag && currMB->p_Vid->yuv_format==YUV444)
                colocated = currSlice->listX[LIST_1+2][0]->JVmv_info[currMB->p_Slice->colour_plane_id][(opic_block_y)>>1][(opic_block_x)];
              else
                colocated = currSlice->listX[LIST_1+2][0]->mv_info[(opic_block_y)>>1][opic_block_x];
            }
            if(currSlice->listX[LIST_1][0]->coded_frame)
            {
              int iPosBlkY = (RSD(opic_block_y)>>3)*8 + ((RSD(opic_block_y)>>1) & 0x03);
              if(colocated.ref_idx[LIST_0] >=0) // && !colocated.ref_pic[LIST_0])
                colocated.ref_pic[LIST_0] = currSlice->listX[LIST_1+2][0]->frame->mv_info[iPosBlkY][RSD(opic_block_x)].ref_pic[LIST_0];
              if(colocated.ref_idx[LIST_1] >=0) // && !colocated.ref_pic[LIST_1])
                colocated.ref_pic[LIST_1] = currSlice->listX[LIST_1+2][0]->frame->mv_info[iPosBlkY][RSD(opic_block_x)].ref_pic[LIST_1];
            }
          }
          else
          {
            if (p_Vid->active_sps->direct_8x8_inference_flag )
            {
              if(currMB->p_Inp->separate_colour_plane_flag && currMB->p_Vid->yuv_format==YUV444)
                colocated = currSlice->listX[LIST_1+4][0]->JVmv_info[currMB->p_Slice->colour_plane_id][RSD(opic_block_y)>>1][RSD(opic_block_x)];
              else
                colocated = currSlice->listX[LIST_1+4][0]->mv_info[RSD(opic_block_y)>>1][RSD(opic_block_x)];

            }
            else
            {
              if(currMB->p_Inp->separate_colour_plane_flag && currMB->p_Vid->yuv_format==YUV444)
                colocated = currSlice->listX[LIST_1+4][0]->JVmv_info[currMB->p_Slice->colour_plane_id][(opic_block_y)>>1][opic_block_x];
              else
                colocated = currSlice->listX[LIST_1+4][0]->mv_info[(opic_block_y)>>1][opic_block_x];
            }
            if(currSlice->listX[LIST_1][0]->coded_frame)
            {
              int iPosBlkY = (RSD(opic_block_y)>>3)*8 + ((RSD(opic_block_y)>>1) & 0x03)+4;
              if(colocated.ref_idx[LIST_0] >=0) // && !colocated.ref_pic[LIST_0])
                colocated.ref_pic[LIST_0] = currSlice->listX[LIST_1+4][0]->frame->mv_info[iPosBlkY][RSD(opic_block_x)].ref_pic[LIST_0];
              if(colocated.ref_idx[LIST_1] >=0)// && !colocated.ref_pic[LIST_1])
                colocated.ref_pic[LIST_1] = currSlice->listX[LIST_1+4][0]->frame->mv_info[iPosBlkY][RSD(opic_block_x)].ref_pic[LIST_1];
            }
          }
        }
      }
      else if(!p_Vid->active_sps->frame_mbs_only_flag && !currSlice->structure && !currSlice->listX[LIST_1][0]->coded_frame)
      {
        if (iabs(p_Vid->enc_picture->poc - list1[0]->bottom_field->poc)> iabs(p_Vid->enc_picture->poc -list1[0]->top_field->poc) )
        {
          colocated = p_Vid->active_sps->direct_8x8_inference_flag ? 
            list1[0]->top_field->mv_info[RSD(opic_block_y)>>1][RSD(opic_block_x)] : list1[0]->top_field->mv_info[(opic_block_y)>>1][opic_block_x];
        }
        else
        {
          colocated = p_Vid->active_sps->direct_8x8_inference_flag ? 
            list1[0]->bottom_field->mv_info[RSD(opic_block_y)>>1][RSD(opic_block_x)] : list1[0]->bottom_field->mv_info[(opic_block_y)>>1][opic_block_x];
        }
      }
      else if(!p_Vid->active_sps->frame_mbs_only_flag && currSlice->structure && list1[0]->coded_frame)
      {
        int iPosBlkY; 
        int currentmb = 2*(list1[0]->size_x>>4) * (opic_block_y >> 2)+ (opic_block_x>>2)*2 + ((opic_block_y>>1) & 0x01);
        if(currSlice->structure!=list1[0]->structure)
        {
          if (currSlice->structure == TOP_FIELD)
          {
            colocated = p_Vid->active_sps->direct_8x8_inference_flag ? 
              list1[0]->frame->top_field->mv_info[RSD(opic_block_y)][RSD(opic_block_x)] : list1[0]->frame->top_field->mv_info[opic_block_y][opic_block_x];
          }
          else
          {
            colocated = p_Vid->active_sps->direct_8x8_inference_flag ? 
              list1[0]->frame->bottom_field->mv_info[RSD(opic_block_y)][RSD(opic_block_x)] : list1[0]->frame->bottom_field->mv_info[opic_block_y][opic_block_x];
          }
        }

        if(!currSlice->listX[LIST_1][0]->frame->mb_aff_frame_flag || !list1[0]->frame->motion.mb_field[currentmb])
          iPosBlkY = 2*(RSD(opic_block_y));
        else
          iPosBlkY = (RSD(opic_block_y)>>2)*8 + (RSD(opic_block_y) & 0x03)+4*(currSlice->structure == BOTTOM_FIELD);
        if(colocated.ref_idx[LIST_0] >=0) // && !colocated.ref_pic[LIST_0])
          colocated.ref_pic[LIST_0] = list1[0]->frame->mv_info[iPosBlkY][RSD(opic_block_x)].ref_pic[LIST_0];
        if(colocated.ref_idx[LIST_1] >=0)// && !colocated.ref_pic[LIST_1])
          colocated.ref_pic[LIST_1] = list1[0]->frame->mv_info[iPosBlkY][RSD(opic_block_x)].ref_pic[LIST_1];
      }

      refList = ((colocated.ref_idx[LIST_0] == -1 || (p_Vid->view_id && colocated.ref_idx[LIST_0]==list1[0]->ref_pic_na[0]))? LIST_1 : LIST_0);
      ref_idx = colocated.ref_idx[refList];

      // next P is intra mode
      if (ref_idx == -1 || (p_Vid->view_id && ref_idx==list1[0]->ref_pic_na[refList]))
      {
        all_mvs[LIST_0][0][0][block_y][block_x] = zero_mv;
        all_mvs[LIST_1][0][0][block_y][block_x] = zero_mv;
        currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_0] = 0;
        currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_1] = 0;
        currSlice->direct_pdir[pic_block_y][pic_block_x] = 2;
      }
      // next P is skip or inter mode
      else
      {
        int mapped_idx=INVALIDINDEX;
        int iref;
        if (colocated.ref_pic[refList] == NULL) 
        {
           printf("invalid index found\n");
        }
        else
        {    
          if( (currSlice->mb_aff_frame_flag && ( (currMB->mb_field && colocated.ref_pic[refList]->structure==FRAME) || 
            (!currMB->mb_field && colocated.ref_pic[refList]->structure!=FRAME))) ||
            (!currSlice->mb_aff_frame_flag && ((currSlice->structure==FRAME && colocated.ref_pic[refList]->structure!=FRAME)||
            (currSlice->structure!=FRAME && colocated.ref_pic[refList]->structure==FRAME))) )
          {
            //! Frame with field co-located
            for (iref = 0; iref < imin(currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0 + list_offset]); iref++)
            {
              if (currSlice->listX[LIST_0 + list_offset][iref]->top_field == colocated.ref_pic[refList] ||
                currSlice->listX[LIST_0 + list_offset][iref]->bottom_field == colocated.ref_pic[refList] ||
                currSlice->listX[LIST_0 + list_offset][iref]->frame == colocated.ref_pic[refList] ) 
              {
                if ((p_Vid->field_picture==1) && (currSlice->listX[LIST_0 + list_offset][iref]->structure != currSlice->structure))
                {
                  mapped_idx=INVALIDINDEX;
                }
                else
                {
                  mapped_idx = iref;            
                  break;
                }
              }
              else //! invalid index. Default to zero even though this case should not happen
                mapped_idx=INVALIDINDEX;
            }
          }
          else
          {
            for (iref = 0; iref < imin(currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0 + list_offset]);iref++)
            {
              if(currSlice->listX[LIST_0 + list_offset][iref] == colocated.ref_pic[refList])
              {
                mapped_idx = iref;            
                break;
              }
              else //! invalid index. Default to zero even though this case should not happen
              {
                mapped_idx=INVALIDINDEX;
              }
            }
          }
        }
        if (mapped_idx != INVALIDINDEX)
        {
          MotionVector mv = colocated.mv[refList];
          mv_scale = currSlice->mvscale[LIST_0 + list_offset][mapped_idx];

          if((currSlice->mb_aff_frame_flag && !currMB->mb_field && colocated.ref_pic[refList]->structure!=FRAME) ||
            (!currSlice->mb_aff_frame_flag && currSlice->structure==FRAME && colocated.ref_pic[refList]->structure!=FRAME))
            mv.mv_y *= 2;
          else if((currSlice->mb_aff_frame_flag && currMB->mb_field && colocated.ref_pic[refList]->structure==FRAME) ||
            (!currSlice->mb_aff_frame_flag && currSlice->structure!=FRAME && colocated.ref_pic[refList]->structure==FRAME))
            mv.mv_y /= 2;

          if (mv_scale==9999)
          {
            // forward
            all_mvs[LIST_0][0][0][block_y][block_x] = mv;
            // backward
            all_mvs[LIST_1][0][0][block_y][block_x] = zero_mv;
          }
          else
          {
            // forward
            all_mvs[LIST_0][mapped_idx][0][block_y][block_x].mv_x = (short) ((mv_scale * mv.mv_x + 128) >> 8);
            all_mvs[LIST_0][mapped_idx][0][block_y][block_x].mv_y = (short) ((mv_scale * mv.mv_y + 128) >> 8);
            // backward
            all_mvs[LIST_1][         0][0][block_y][block_x].mv_x = (short) (((mv_scale - 256) * mv.mv_x + 128) >> 8);
            all_mvs[LIST_1][         0][0][block_y][block_x].mv_y = (short) (((mv_scale - 256) * mv.mv_y + 128) >> 8);

          }

          // Test Level Limits if satisfied.
          if ( out_of_bounds_mvs(p_Vid, &all_mvs[LIST_0][mapped_idx][0][block_y][block_x])|| out_of_bounds_mvs(p_Vid, &all_mvs[LIST_1][0][0][block_y][block_x]))
          {
            currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_0] = -1;
            currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_1] = -1;
            currSlice->direct_pdir[pic_block_y][pic_block_x] = -1;
          }
          else
          {
            currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_0] = (char) mapped_idx;
            currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_1] = 0;
            currSlice->direct_pdir[pic_block_y][pic_block_x] = 2;
          }
        }
        else
        {
          currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_0] = -1;
          currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_1] = -1;
          currSlice->direct_pdir[pic_block_y][pic_block_x] = -1;
        }
      }

      if (p_Vid->active_pps->weighted_bipred_idc == 1 && currSlice->direct_pdir[pic_block_y][pic_block_x] == 2)
      {
        int weight_sum, i;
        short l0_refX = currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_0];
        short l1_refX = currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_1];
        for (i=0;i< (p_Vid->active_sps->chroma_format_idc == YUV400 ? 1 : 3); i++)
        {
          weight_sum = currSlice->wbp_weight[0][l0_refX][l1_refX][i] + currSlice->wbp_weight[1][l0_refX][l1_refX][i];
          if (weight_sum < -128 ||  weight_sum > 127)
          {
            currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_0] = -1;
            currSlice->direct_ref_idx[pic_block_y][pic_block_x][LIST_1] = -1;
            currSlice->direct_pdir   [pic_block_y][pic_block_x]         = -1;
            break;
          }
        }
      }
    }
  }
}

static inline void set_direct_references(const PixelPos *mb, char *l0_rFrame, char *l1_rFrame, PicMotionParams **mv_info)
{
  if (mb->available)
  {
    char *ref_idx = mv_info[mb->pos_y][mb->pos_x].ref_idx;
    *l0_rFrame  = ref_idx[LIST_0];
    *l1_rFrame  = ref_idx[LIST_1];
  }
  else
  {
    *l0_rFrame  = -1;
    *l1_rFrame  = -1;
  }
}

static void set_direct_references_mb_field(const PixelPos *mb, char *l0_rFrame, char *l1_rFrame, PicMotionParams **mv_info, Macroblock *mb_data)
{
  if (mb->available)
  {
    char *ref_idx = mv_info[mb->pos_y][mb->pos_x].ref_idx;
    if (mb_data[mb->mb_addr].mb_field)
    {
      *l0_rFrame  = ref_idx[LIST_0];
      *l1_rFrame  = ref_idx[LIST_1];
    }
    else
    {
      *l0_rFrame  = (ref_idx[LIST_0] < 0) ? ref_idx[LIST_0] : ref_idx[LIST_0] * 2;
      *l1_rFrame  = (ref_idx[LIST_1] < 0) ? ref_idx[LIST_1] : ref_idx[LIST_1] * 2;
    }
  }
  else
  {
    *l0_rFrame  = -1;
    *l1_rFrame  = -1;
  }
}

static void set_direct_references_mb_frame(const PixelPos *mb, char *l0_rFrame, char *l1_rFrame, PicMotionParams **mv_info, Macroblock *mb_data)
{
  if (mb->available)
  {
    char *ref_idx = mv_info[mb->pos_y][mb->pos_x].ref_idx;
    if (mb_data[mb->mb_addr].mb_field)
    {
      *l0_rFrame  = (ref_idx[LIST_0] >> 1);
      *l1_rFrame  = (ref_idx[LIST_1] >> 1);
    }
    else
    {
      *l0_rFrame  = ref_idx[LIST_0];
      *l1_rFrame  = ref_idx[LIST_1];
    }
  }
  else
  {
    *l0_rFrame  = -1;
    *l1_rFrame  = -1;
  }
}

static void test_valid_direct(Slice *currSlice, seq_parameter_set_rbsp_t *active_sps, char  *direct_ref_idx, short l0_refX, short l1_refX, int pic_block_y, int pic_block_x)
{
  int weight_sum, i;
  Boolean invalid_wp = FALSE;
  for (i=0;i< (active_sps->chroma_format_idc == YUV400 ? 1 : 3); i++)
  {
    weight_sum = currSlice->wbp_weight[0][l0_refX][l1_refX][i] + currSlice->wbp_weight[1][l0_refX][l1_refX][i];
    if (weight_sum < -128 ||  weight_sum > 127)
    {
      invalid_wp = TRUE;
      break;
    }
  }
  if (invalid_wp == FALSE)
    currSlice->direct_pdir[pic_block_y][pic_block_x] = 2;
  else
  {
    direct_ref_idx[LIST_0] = -1;
    direct_ref_idx[LIST_1] = -1;
    currSlice->direct_pdir[pic_block_y][pic_block_x] = -1;
  }
}

/*!
*************************************************************************************
* \brief
*    Temporary function for colocated info when direct_inference is enabled. 
*
*************************************************************************************
*/
int get_colocated_info(Macroblock *currMB, StorablePicture *list1, int i, int j)
{
  if (list1->is_long_term)
    return 1;
  else
  {
    Slice *currSlice = currMB->p_Slice;
    VideoParameters *p_Vid = currMB->p_Vid;
    if( (currSlice->mb_aff_frame_flag) ||
      (!p_Vid->active_sps->frame_mbs_only_flag && ((!currSlice->structure && !list1->coded_frame) || (currSlice->structure!=list1->structure && list1->coded_frame))))
    {
      int jj = RSD(j);
      int ii = RSD(i);
      int jdiv = (jj>>1);
      int moving;
      PicMotionParams *fs = &list1->mv_info[jj][ii];

      if(currSlice->structure && currSlice->structure!=list1->structure && list1->coded_frame)
      {
         if(currSlice->structure == TOP_FIELD)
           fs = list1->top_field->mv_info[jj] + ii;
         else
           fs = list1->bottom_field->mv_info[jj] + ii;
      }
      else
      {
        if( (currSlice->mb_aff_frame_flag && ((!currMB->mb_field && list1->motion.mb_field[currMB->mbAddrX]) ||
          (!currMB->mb_field && !list1->coded_frame))) 
          || (!currSlice->mb_aff_frame_flag))
        {
          if (iabs(p_Vid->enc_picture->poc - list1->bottom_field->poc)> iabs(p_Vid->enc_picture->poc -list1->top_field->poc) )
          {
            fs = list1->top_field->mv_info[jdiv] + ii;
          }
          else
          {
            fs = list1->bottom_field->mv_info[jdiv] + ii;
          }
        }
      }
      moving = !((((fs->ref_idx[LIST_0] == 0)
        &&  (iabs(fs->mv[LIST_0].mv_x)>>1 == 0)
        &&  (iabs(fs->mv[LIST_0].mv_y)>>1 == 0)))
        || ((fs->ref_idx[LIST_0] == -1)
        &&  (fs->ref_idx[LIST_1] == 0)
        &&  (iabs(fs->mv[LIST_1].mv_x)>>1 == 0)
        &&  (iabs(fs->mv[LIST_1].mv_y)>>1 == 0)));
      return moving;
    }
    else
    {
      PicMotionParams *fs = &list1->mv_info[RSD(j)][RSD(i)];
      int moving;
      if(currMB->p_Vid->yuv_format == YUV444 && !currSlice->P444_joined)
        fs = &list1->JVmv_info[(int)(p_Vid->colour_plane_id)][RSD(j)][RSD(i)];
      moving= !((((fs->ref_idx[LIST_0] == 0)
        &&  (iabs(fs->mv[LIST_0].mv_x)>>1 == 0)
        &&  (iabs(fs->mv[LIST_0].mv_y)>>1 == 0)))
        || ((fs->ref_idx[LIST_0] == -1)
        &&  (fs->ref_idx[LIST_1] == 0)
        &&  (iabs(fs->mv[LIST_1].mv_x)>>1 == 0)
        &&  (iabs(fs->mv[LIST_1].mv_y)>>1 == 0)));

      return moving;  
    }
  }
}

/*!
*************************************************************************************
* \brief
*    Colocated info <= direct_inference is disabled. 
*************************************************************************************
*/
int get_colocated_info_4x4(Macroblock *currMB, StorablePicture *list1, int i, int j)
{
  if (list1->is_long_term)
    return 1;
  else
  {
    PicMotionParams *fs = &list1->mv_info[j][i];

    int moving = !((((fs->ref_idx[LIST_0] == 0)
      &&  (iabs(fs->mv[LIST_0].mv_x)>>1 == 0)
      &&  (iabs(fs->mv[LIST_0].mv_y)>>1 == 0)))
      || ((fs->ref_idx[LIST_0] == -1)
      &&  (fs->ref_idx[LIST_1] == 0)
      &&  (iabs(fs->mv[LIST_1].mv_x)>>1 == 0)
      &&  (iabs(fs->mv[LIST_1].mv_y)>>1 == 0)));

    return moving;  
  }
}

/*!
************************************************************************
* \brief
*    Calculate Spatial Direct Mode Motion Vectors 
************************************************************************
*/
void Get_Direct_MV_Spatial_Normal (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_Slice; 
  VideoParameters *p_Vid = currMB->p_Vid;
  PicMotionParams **mv_info = p_Vid->enc_picture->mv_info;
  char l0_refA, l0_refB, l0_refC;
  char l1_refA, l1_refB, l1_refC;
  char l0_refX,l1_refX;
  MotionVector pmvfw = zero_mv, pmvbw = zero_mv;

  int   block_x, block_y, pic_block_x, pic_block_y, opic_block_x, opic_block_y;
  MotionVector *****all_mvs;
  char  *direct_ref_idx;
  StorablePicture **list1 = currSlice->listX[LIST_1];

  PixelPos mb[4];  
  get_neighbors(currMB, mb, 0, 0, 16);

  set_direct_references(&mb[0], &l0_refA,  &l1_refA,  mv_info);
  set_direct_references(&mb[1], &l0_refB,  &l1_refB,  mv_info);
  set_direct_references(&mb[2], &l0_refC,  &l1_refC,  mv_info);

  l0_refX = (char) imin(imin((unsigned char) l0_refA, (unsigned char) l0_refB), (unsigned char) l0_refC);
  l1_refX = (char) imin(imin((unsigned char) l1_refA, (unsigned char) l1_refB), (unsigned char) l1_refC);

  if (l0_refX >= 0)
    currMB->GetMVPredictor (currMB, mb, &pmvfw, l0_refX, mv_info, LIST_0, 0, 0, 16, 16);

  if (l1_refX >= 0)
    currMB->GetMVPredictor (currMB, mb, &pmvbw, l1_refX, mv_info, LIST_1, 0, 0, 16, 16);

  if (l0_refX == -1 && l1_refX == -1)
  {
    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = currMB->block_y + block_y;
      for (block_x=0; block_x<4; block_x++)
      {
        pic_block_x  = currMB->block_x + block_x;
        direct_ref_idx = currSlice->direct_ref_idx[pic_block_y][pic_block_x];

        currSlice->all_mv[LIST_0][0][0][block_y][block_x] = zero_mv;
        currSlice->all_mv[LIST_1][0][0][block_y][block_x] = zero_mv;

        direct_ref_idx[LIST_0] = direct_ref_idx[LIST_1] = 0;

        if (p_Vid->active_pps->weighted_bipred_idc == 1)
          test_valid_direct(currSlice, currSlice->active_sps, direct_ref_idx, 0, 0, pic_block_y, pic_block_x);
        else
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 2;
      }
    }
  }
  else if (l0_refX == 0 || l1_refX == 0)
  {
    int (*get_colocated)(Macroblock *currMB, StorablePicture *list1, int i, int j) = 
      p_Vid->active_sps->direct_8x8_inference_flag ? get_colocated_info : get_colocated_info_4x4;

    int is_moving_block;
    for (block_y = 0; block_y < 4; block_y++)
    {
      pic_block_y  = currMB->block_y + block_y;
      opic_block_y = (currMB->opix_y >> 2) + block_y;

      for (block_x=0; block_x<4; block_x++)
      {
        pic_block_x    = currMB->block_x + block_x;
        direct_ref_idx = currSlice->direct_ref_idx[pic_block_y][pic_block_x];
        opic_block_x   = (currMB->pix_x >> 2) + block_x;

        all_mvs = currSlice->all_mv;
        is_moving_block = (get_colocated(currMB, list1[0], opic_block_x, opic_block_y) == 0);

        if (l0_refX < 0)
        {
          all_mvs[LIST_0][0][0][block_y][block_x] = zero_mv;
          direct_ref_idx[LIST_0] = -1;
        }
        else if ((l0_refX == 0) && is_moving_block)
        {
          all_mvs[LIST_0][0][0][block_y][block_x] = zero_mv;
          direct_ref_idx[LIST_0] = 0;
        }
        else
        {
          all_mvs[LIST_0][(short) l0_refX][0][block_y][block_x] = pmvfw;
          direct_ref_idx[LIST_0] = (char)l0_refX;
        }
        
        if (l1_refX < 0)
        {
          all_mvs[LIST_1][0][0][block_y][block_x] = zero_mv;
          direct_ref_idx[LIST_1] = -1;
        }
        else if((l1_refX == 0) && is_moving_block)
        {
          all_mvs[LIST_1][0][0][block_y][block_x] = zero_mv;
          direct_ref_idx[LIST_1] = 0;
        }
        else
        {
          all_mvs[LIST_1][(short) l1_refX][0][block_y][block_x] = pmvbw;
          direct_ref_idx[LIST_1] = (char)l1_refX;
        }

        if      (direct_ref_idx[LIST_1] == -1)
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 0;
        else if (direct_ref_idx[LIST_0] == -1)
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 1;
        else if (p_Vid->active_pps->weighted_bipred_idc == 1)
          test_valid_direct(currSlice, currSlice->active_sps, direct_ref_idx, l0_refX, l1_refX, pic_block_y, pic_block_x);
        else
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 2;
      }
    }
  }
  else
  {
    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = currMB->block_y + block_y;

      for (block_x=0; block_x<4; block_x++)
      {
        pic_block_x  = currMB->block_x + block_x;
        direct_ref_idx = currSlice->direct_ref_idx[pic_block_y][pic_block_x];

        all_mvs = currSlice->all_mv;

        if (l0_refX > 0)
        {
          all_mvs[LIST_0][(short) l0_refX][0][block_y][block_x] = pmvfw;
          direct_ref_idx[LIST_0]= (char)l0_refX;          
        }
        else
        {
          all_mvs[LIST_0][0][0][block_y][block_x] = zero_mv;
          direct_ref_idx[LIST_0]=-1;
        }

        if (l1_refX > 0)
        {
          all_mvs[LIST_1][(short) l1_refX][0][block_y][block_x] = pmvbw;
          direct_ref_idx[LIST_1] = (char)l1_refX;
        }
        else
        {
          all_mvs[LIST_1][0][0][block_y][block_x] = zero_mv;
          direct_ref_idx[LIST_1] = -1;
        }

        if      (direct_ref_idx[LIST_1] == -1)
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 0;
        else if (direct_ref_idx[LIST_0] == -1)
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 1;
        else if (p_Vid->active_pps->weighted_bipred_idc == 1)
          test_valid_direct(currSlice, currSlice->active_sps, direct_ref_idx, l0_refX, l1_refX, pic_block_y, pic_block_x);
        else
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 2;
      }
    }
  }
}


/*!
************************************************************************
* \brief
*    Calculate Spatial Direct Mode Motion Vectors 
************************************************************************
*/
void Get_Direct_MV_Spatial_MBAFF (Macroblock *currMB)
{
  char l0_refA, l0_refB, l0_refC;
  char l1_refA, l1_refB, l1_refC;
  char l0_refX,l1_refX;
  MotionVector pmvfw = zero_mv, pmvbw = zero_mv;

  int   block_x, block_y, pic_block_x, pic_block_y, opic_block_x, opic_block_y;
  MotionVector *****all_mvs;
  char  *direct_ref_idx;
  int is_moving_block;
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  PicMotionParams **mv_info = p_Vid->enc_picture->mv_info;
  StorablePicture **list1 = currSlice->listX[LIST_1 + currMB->list_offset];

  int (*get_colocated)(Macroblock *currMB, StorablePicture *list1, int i, int j) = 
    p_Vid->active_sps->direct_8x8_inference_flag ? get_colocated_info : get_colocated_info_4x4;

  PixelPos mb[4];  
  get_neighbors(currMB, mb, 0, 0, 16);


  if (currMB->mb_field)
  {
    set_direct_references_mb_field(&mb[0], &l0_refA, &l1_refA, mv_info, p_Vid->mb_data);
    set_direct_references_mb_field(&mb[1], &l0_refB, &l1_refB, mv_info, p_Vid->mb_data);
    set_direct_references_mb_field(&mb[2], &l0_refC, &l1_refC, mv_info, p_Vid->mb_data);
  }
  else
  {
    set_direct_references_mb_frame(&mb[0], &l0_refA, &l1_refA, mv_info, p_Vid->mb_data);
    set_direct_references_mb_frame(&mb[1], &l0_refB, &l1_refB, mv_info, p_Vid->mb_data);
    set_direct_references_mb_frame(&mb[2], &l0_refC, &l1_refC, mv_info, p_Vid->mb_data);
  }

  l0_refX = (char) imin(imin((unsigned char) l0_refA, (unsigned char) l0_refB), (unsigned char) l0_refC);
  l1_refX = (char) imin(imin((unsigned char) l1_refA, (unsigned char) l1_refB), (unsigned char) l1_refC);

  if (l0_refX >=0)
    currMB->GetMVPredictor (currMB, mb, &pmvfw, l0_refX, mv_info, LIST_0, 0, 0, 16, 16);

  if (l1_refX >=0)
    currMB->GetMVPredictor (currMB, mb, &pmvbw, l1_refX, mv_info, LIST_1, 0, 0, 16, 16);

  for (block_y=0; block_y<4; block_y++)
  {
    pic_block_y  = currMB->block_y + block_y;
    opic_block_y = (currMB->opix_y >> 2) + block_y;

    for (block_x=0; block_x<4; block_x++)
    {
      pic_block_x  = currMB->block_x + block_x;
      direct_ref_idx = currSlice->direct_ref_idx[pic_block_y][pic_block_x];
      opic_block_x = (currMB->pix_x >> 2) + block_x;
      is_moving_block = (get_colocated(currMB, list1[0], opic_block_x, opic_block_y) == 0);

      all_mvs = currSlice->all_mv;

      if (l0_refX >=0)
      {
        if (!l0_refX  && is_moving_block)
        {
          all_mvs[LIST_0][0][0][block_y][block_x] = zero_mv;
          direct_ref_idx[LIST_0] = 0;
        }
        else
        {
          all_mvs[LIST_0][(short) l0_refX][0][block_y][block_x] = pmvfw;
          direct_ref_idx[LIST_0] = (char)l0_refX;
        }
      }
      else
      {
        all_mvs[LIST_0][0][0][block_y][block_x] = zero_mv;
        direct_ref_idx[LIST_0] = -1;
      }

      if (l1_refX >=0)
      {
        if(l1_refX==0 && is_moving_block)
        {
          all_mvs[LIST_1][0][0][block_y][block_x] = zero_mv;
          direct_ref_idx[LIST_1] = (char)l1_refX;
        }
        else
        {
          all_mvs[LIST_1][(short) l1_refX][0][block_y][block_x] = pmvbw;
          direct_ref_idx[LIST_1] = (char)l1_refX;
        }
      }
      else
      {
        all_mvs[LIST_1][0][0][block_y][block_x] = zero_mv;
        direct_ref_idx[LIST_1] = -1;
      }

     // Test Level Limits if satisfied.

      // Test Level Limits if satisfied.
      if ((out_of_bounds_mvs(p_Vid, &all_mvs[LIST_0][l0_refX < 0? 0 : l0_refX][0][block_y][block_x])
        ||  out_of_bounds_mvs(p_Vid, &all_mvs[LIST_1][l1_refX < 0? 0 : l1_refX][0][block_y][block_x])))
      {
        direct_ref_idx[LIST_0] = -1;
        direct_ref_idx[LIST_1] = -1;
        currSlice->direct_pdir   [pic_block_y][pic_block_x]         = -1;
      }     
      else
      {
        if (l0_refX < 0 && l1_refX < 0)
        {
          direct_ref_idx[LIST_0] = direct_ref_idx[LIST_1] = 0;
          l0_refX = 0;
          l1_refX = 0;
        }

        if      (direct_ref_idx[LIST_1] == -1)
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 0;
        else if (direct_ref_idx[LIST_0] == -1)
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 1;
        else if (p_Vid->active_pps->weighted_bipred_idc == 1)
          test_valid_direct(currSlice, currSlice->active_sps, direct_ref_idx, l0_refX, l1_refX, pic_block_y, pic_block_x);
        else
          currSlice->direct_pdir[pic_block_y][pic_block_x] = 2;
      }
    }
  }
}
