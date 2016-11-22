
/*!
 *************************************************************************************
 * \file q_around.c
 *
 * \brief
 *    Quantization Adaptive Rounding (JVT-N011)
 * \author
 *    - Alexis Michael Tourapis    <alexismt@ieee.org>
 *    - Ehsan Maani                <emaan@dolby.com>
 *
 *************************************************************************************
 */

#include "global.h"
#include "memalloc.h"
#include "q_offsets.h"
#include "q_around.h"

static const int AdaptRndCrPos[2][5] =
{
  //  P,   B,   I,  SP,  SI
  {   4,   7,   1,   4,   1}, // Intra MB
  {  10,  13,  10,  10,  10}  // Inter MB
};

static const int AdaptRndPos[4][5] =
{
  //  P,   B,   I,  SP,  SI
  {   3,   6,   0,   3,   0}, // 4x4 Intra MB
  {   1,   2,   0,   1,   2}, // 8x8 Intra MB
  {   9,  12,   9,   9,   9}, // 4x4 Inter MB
  {   3,   4,   3,   3,   3}, // 8x8 Inter MB
};


/*!
************************************************************************
* \brief
*    copy data from to a temporary location
************************************************************************
*/
void store_adaptive_rounding_4x4 (VideoParameters *p_Vid, int****ARCofAdj, int mode, int block_y, int block_x)
{
  int j;

  for (j = block_y; j < block_y + 4; j++)
    memcpy(&ARCofAdj[0][DUMMY][j][block_x], &ARCofAdj[0][mode][j][block_x], BLOCK_SIZE * sizeof(int));

  if (p_Vid->P444_joined)
  {
    for (j = block_y; j < block_y + 4; j++)
    {
      memcpy(&ARCofAdj[1][DUMMY][j][block_x],&ARCofAdj[1][mode][j][block_x], BLOCK_SIZE * sizeof(int));
    }
    for (j = block_y; j < block_y + 4; j++)
    {
      memcpy(&ARCofAdj[2][DUMMY][j][block_x],&ARCofAdj[2][mode][j][block_x], BLOCK_SIZE * sizeof(int));
    }
  }
}


/*!
************************************************************************
* \brief
*    copy data from a temporary location to the actual location
************************************************************************
*/
void update_adaptive_rounding_4x4 (VideoParameters *p_Vid, int****ARCofAdj , int mode, int block_y, int block_x)
{
  int j;

  for (j = block_y; j < block_y + BLOCK_SIZE; j++)
    memcpy (&ARCofAdj[0][mode][j][block_x],&ARCofAdj[0][DUMMY][j][block_x], BLOCK_SIZE * sizeof(int));

  if (p_Vid->P444_joined)
  {
    for (j = 0; j < block_y + BLOCK_SIZE; j++)
    {
      memcpy (&ARCofAdj[1][mode][j][block_x], &ARCofAdj[1][DUMMY][j][block_x], BLOCK_SIZE * sizeof(int));
    }
    for (j = 0; j < block_y + BLOCK_SIZE; j++)
    {
      memcpy (&ARCofAdj[2][mode][j][block_x], &ARCofAdj[2][DUMMY][j][block_x], BLOCK_SIZE * sizeof(int));
    }
  }
}

/*!
************************************************************************
* \brief
*    copy data from to a temporary location
************************************************************************
*/
void store_adaptive_rounding_16x16 (VideoParameters *p_Vid, int****ARCofAdj, int mode)
{
    memcpy(&ARCofAdj[0][DUMMY][0][0], &ARCofAdj[0][mode][0][0], MB_PIXELS * sizeof(int));
    if (p_Vid->P444_joined)
    {
      memcpy(&ARCofAdj[1][DUMMY][0][0], &ARCofAdj[1][mode][0][0], MB_PIXELS * sizeof(int));
      memcpy(&ARCofAdj[2][DUMMY][0][0], &ARCofAdj[2][mode][0][0], MB_PIXELS * sizeof(int));
    }
}


/*!
************************************************************************
* \brief
*    copy data from a temporary location to the actual location
************************************************************************
*/
void update_adaptive_rounding_16x16(VideoParameters *p_Vid, int****ARCofAdj , int mode)
{  
  memcpy (&ARCofAdj[0][mode][0][0],&ARCofAdj[0][DUMMY][0][0], MB_PIXELS * sizeof(int));
  if (p_Vid->P444_joined)
  {
    memcpy (&ARCofAdj[1][mode][0][0], &ARCofAdj[1][DUMMY][0][0], MB_PIXELS * sizeof(int));
    memcpy (&ARCofAdj[2][mode][0][0], &ARCofAdj[2][DUMMY][0][0], MB_PIXELS * sizeof(int));
  }
}




/*!
************************************************************************
* \brief
*    update rounding offsets based on JVT-N011
************************************************************************
*/
void update_offset_params(Macroblock *currMB, int mode, byte luma_transform_size_8x8_flag)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  int is_inter = (mode != I4MB)&&(mode != I16MB) && (mode != I8MB);
  int luma_pos = AdaptRndPos[(is_inter<<1) + luma_transform_size_8x8_flag][p_Vid->type];
  int i,j;
  int qp = currMB->qp + p_Vid->bitdepth_luma_qp_scale;
  int cur_qp = p_Inp->AdaptRoundingFixed ? 0 : qp;
  int temp = 0;
  QuantParameters *p_Quant = p_Vid->p_Quant;
  int offsetRange = 1 << (OffsetBits - 1);
  int blk_mask = 0x03 + (luma_transform_size_8x8_flag<<2);
  int blk_shift = 2 + luma_transform_size_8x8_flag;  
  short **offsetList = luma_transform_size_8x8_flag ? p_Quant->OffsetList8x8[cur_qp] : p_Quant->OffsetList4x4[cur_qp];
  short *cur_offset_list = offsetList[luma_pos];

  int **fAdjust = luma_transform_size_8x8_flag ? p_Vid->ARCofAdj8x8[0][mode] : p_Vid->ARCofAdj4x4[0][mode];
  
  if (mode == IPCM) return;

  if( (p_Vid->active_sps->chroma_format_idc == YUV444) && (p_Inp->separate_colour_plane_flag != 0) )
  {
    if( luma_transform_size_8x8_flag )  // 8x8
      luma_pos += 5 * p_Vid->colour_plane_id;
    else  // 4x4
      luma_pos += p_Vid->colour_plane_id;
    cur_offset_list = offsetList[luma_pos];
  }
 
  for (j=0; j < MB_BLOCK_SIZE; j++)
  {
    int j_pos = ((j & blk_mask)<<blk_shift);
    for (i=0; i < MB_BLOCK_SIZE; i++)
    {
      temp = j_pos + (i & blk_mask);
      cur_offset_list[temp] = (short) iClip3(0, offsetRange, cur_offset_list[temp] + (short) fAdjust[j][i]);
    }
  }

  if(p_Vid->P444_joined)
  { 
    int **fAdjustCbCr;
    int uv;

    for(uv = 0; uv < 2; uv++)
    {
      luma_pos = AdaptRndPos[(is_inter<<1) + luma_transform_size_8x8_flag][p_Vid->type];
      fAdjustCbCr = luma_transform_size_8x8_flag ? p_Vid->ARCofAdj8x8[uv + 1][mode] : p_Vid->ARCofAdj4x4[uv + 1][mode];
      if(luma_transform_size_8x8_flag )  // 8x8
        luma_pos += 5 * (uv+1);
      else  // 4x4
        luma_pos += (uv+1);
      cur_offset_list = offsetList[luma_pos];
      for (j=0; j < MB_BLOCK_SIZE; j++)
      {
        int j_pos = ((j & blk_mask)<<blk_shift);
        for (i=0; i < MB_BLOCK_SIZE; i++)
        {
          temp = j_pos + (i & blk_mask);
          cur_offset_list[temp] = (short) iClip3(0, offsetRange, cur_offset_list[temp] + (short) fAdjustCbCr[j][i]);
        }
      }
    }
  }

  if ((p_Inp->AdaptRndChroma) && (p_Vid->yuv_format == YUV420 || p_Vid->yuv_format == YUV422 ))
  {  
    int u_pos = AdaptRndCrPos[is_inter][p_Vid->type];
    int v_pos = u_pos + 1;
    int k, jpos, uv = 1;

    for (k = u_pos; k <= v_pos; k++)
    {
      int **fAdjustChroma = (luma_transform_size_8x8_flag && mode == P8x8 )? p_Vid->ARCofAdj4x4[uv][4] : p_Vid->ARCofAdj4x4[uv][mode];
      uv++;
      cur_offset_list = p_Quant->OffsetList4x4[cur_qp][k];

      for (j = 0; j < p_Vid->mb_cr_size_y; j++)
      {
        jpos = ((j & 0x03)<<2);
        for (i = 0; i < p_Vid->mb_cr_size_x; i++)
        {
          temp = jpos + (i & 0x03);
          cur_offset_list[temp] = (short) iClip3(0, offsetRange, cur_offset_list[temp] + (short) fAdjustChroma[j][i]);
        }
      }
    }
  }
}


/*!
************************************************************************
* \brief
*    resets adaptive rounding p_Inp to zero
************************************************************************
*/
void reset_adaptive_rounding(VideoParameters *p_Vid)
{
  int maxplane;
  maxplane = (p_Vid->yuv_format == YUV400)?  1 : 3;
  memset(&p_Vid->ARCofAdj4x4[0][0][0][0], 0, maxplane * MAXMODE * MB_PIXELS * sizeof (int));
  if (p_Vid->p_Inp->Transform8x8Mode)
  {
    maxplane = (p_Vid->yuv_format == YUV400)?  1 : (p_Vid->P444_joined ? 3 : 1);
    memset(&p_Vid->ARCofAdj8x8[0][0][0][0], 0, maxplane * MAXMODE * MB_PIXELS * sizeof (int));
  }
}

/*!
************************************************************************
* \brief
*    resets adaptive rounding p_Inp for the direct mode to zero
************************************************************************
*/
void reset_adaptive_rounding_direct(VideoParameters *p_Vid)
{
  int maxplane, pl;
  maxplane = (p_Vid->yuv_format == YUV400)?  1 : 3;
  for (pl = 0; pl < maxplane; pl++)
    memset(&p_Vid->ARCofAdj4x4[pl][DUMMY][0][0], 0, MB_PIXELS * sizeof (int));

  if (p_Vid->p_Inp->Transform8x8Mode)
  {
    maxplane = (p_Vid->yuv_format == YUV400)?  1 : (p_Vid->P444_joined ? 3 : 1);
    for (pl = 0; pl < maxplane; pl++)
      memset(&p_Vid->ARCofAdj8x8[pl][DUMMY][0][0], 0, MB_PIXELS * sizeof (int));
  }
 
}

/*!
************************************************************************
* \brief
*    copy data from a temporary location to the actual location for mode P8x8
************************************************************************
*/
void update_adaptive_rounding_8x8(VideoParameters *p_Vid, InputParameters *p_Inp, RD_8x8DATA* dataTr, int**** ARCofAdj)
{
  int b8, pl, plane, j0, i0, j;

  if (p_Inp->AdaptiveRounding)
  {
    plane = (p_Vid->P444_joined) ?  3 : 1;
    for (pl = 0; pl < plane; pl++)
    {
      for (b8 = 0; b8 < 4; b8++)
      {
        j0   = 8*(b8 >> 1);
        i0   = 8*(b8 & 0x01);
        if (dataTr->part[b8].mode > 0)
        {
          for (j = j0; j < j0 + BLOCK_SIZE_8x8; j++)
          {
            memcpy(&ARCofAdj[pl][P8x8][j][i0], &ARCofAdj[pl][(short) dataTr->part[b8].mode][j][i0], BLOCK_SIZE_8x8 * sizeof(int));
          }
        }
      }
    }
  }
}


