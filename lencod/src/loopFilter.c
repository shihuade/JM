
/*!
 *************************************************************************************
 * \file loopFilter.c
 *
 * \brief
 *    Filter to reduce blocking artifacts on a macroblock level.
 *    The filter strength is QP dependent.
 *
 * \author
 *    Contributors:
 *    - Peter List       Peter.List@t-systems.de:  Original code                                 (13-Aug-2001)
 *    - Jani Lainema     Jani.Lainema@nokia.com:   Some bug fixing, removal of recursiveness     (16-Aug-2001)
 *    - Peter List       Peter.List@t-systems.de:  inplace filtering and various simplifications (10-Jan-2002)
 *    - Anthony Joch     anthony@ubvideo.com:      Simplified switching between filters and
 *                                                 non-recursive default filter.                 (08-Jul-2002)
 *    - Cristina Gomila  cristina.gomila@thomson.net: Simplification of the chroma deblocking
 *                                                    from JVT-E089                              (21-Nov-2002)
 *    - Alexis Michael Tourapis atour@dolby.com:   Speed/Architecture improvements               (08-Feb-2007)
 *************************************************************************************
 */

#include "global.h"
#include "image.h"
#include "mb_access.h"
#include "loop_filter.h"

extern void set_loop_filter_functions_mbaff (VideoParameters *p_Vid);
extern void set_loop_filter_functions_normal(VideoParameters *p_Vid);

static void DeblockMb(VideoParameters *p_Vid, imgpel **imgY, imgpel ***imgUV, int MbQAddr);

static void  init_Deblock(VideoParameters *p_Vid)
{
  unsigned int i;
  if (p_Vid->mb_aff_frame_flag == 1) 
  {
    set_loop_filter_functions_mbaff(p_Vid);
  }
  else
  {
    set_loop_filter_functions_normal(p_Vid);
  }

  for (i=0; i < p_Vid->PicSizeInMbs; i++)
  {
    if (p_Vid->mb_data[i].mb_type==IPCM)
    {
      p_Vid->mb_data[i].qp = 0;
      p_Vid->mb_data[i].qpc[0] = 0;
      p_Vid->mb_data[i].qpc[1] = 0;
    }
  }
}

#if (JM_PARALLEL_DEBLOCK == 0)
/*!
 *****************************************************************************************
 * \brief
 *    Filter all macroblocks in order of increasing macroblock address.
 *****************************************************************************************
 */
void DeblockFrame(VideoParameters *p_Vid, imgpel **imgY, imgpel ***imgUV)
{
  unsigned int i;
  init_Deblock(p_Vid);
  for (i=0; i < p_Vid->PicSizeInMbs; i++)
  {
    DeblockMb( p_Vid, imgY, imgUV, i ) ;
  }
}
#else
static void DeblockParallel(VideoParameters *p_Vid, imgpel **imgY, imgpel ***imgUV, unsigned int column, int block, int n_last)
{
  int i, j;
  
  for (j = 0; j < GROUP_SIZE; j++)
  {
    i = block++ * (p_Vid->PicWidthInMbs - 2) + column;

    DeblockMb( p_Vid, imgY, imgUV, i ) ;
    if (block == n_last) break;
  }
}

/*!
 *****************************************************************************************
 * \brief
 *    Filter all macroblocks in a diagonal manner to enable parallelization.
 *****************************************************************************************
 */
void DeblockFrame(VideoParameters *p_Vid, imgpel **imgY, imgpel ***imgUV)
{
  int iheightMBs =(p_Vid->PicSizeInMbs/p_Vid->PicWidthInMbs);
  unsigned int i, k = p_Vid->PicWidthInMbs + 2 * (iheightMBs - 1);
  init_Deblock(p_Vid);

  for (i = 0; i < k; i++)
  {
    int nn;    
    int n_last = imin(iheightMBs, (i >> 1) + 1);
    int n_start = (i < p_Vid->PicWidthInMbs) ? 0 : ((i - p_Vid->PicWidthInMbs) >> 1) + 1;

#if defined(OPENMP)
#pragma omp parallel for
#endif
    for (nn = n_start; nn < n_last; nn += GROUP_SIZE)
      DeblockParallel(p_Vid, imgY, imgUV, i, nn, n_last);
  }
}
#endif

/*!
 *****************************************************************************************
 * \brief
 *    Deblocking filter for one macroblock.
 *****************************************************************************************
 */

static void DeblockMb(VideoParameters *p_Vid, imgpel **imgY, imgpel ***imgUV, int MbQAddr)
{
  int           edge;
  byte          Strength[16];

  short         mb_x, mb_y;

  int           filterNon8x8LumaEdgesFlag[4] = {1,1,1,1};
  int           filterLeftMbEdgeFlag;
  int           filterTopMbEdgeFlag;
  int           edge_cr;
  Macroblock    *MbQ = &(p_Vid->mb_data[MbQAddr]) ; // current Mb
  Slice  *currSlice = MbQ->p_Slice;
  int           mvlimit = (p_Vid->structure!=FRAME) || (p_Vid->mb_aff_frame_flag && MbQ->mb_field) ? 2 : 4;
  seq_parameter_set_rbsp_t *active_sps = p_Vid->active_sps;
  p_Vid->mixedModeEdgeFlag = 0;

  // return, if filter is disabled
  if (MbQ->DFDisableIdc == 1) 
  {
    MbQ->DeblockCall = 0;
    return;
  }

  MbQ->DeblockCall = 1;
  get_mb_pos (p_Vid, MbQAddr, p_Vid->mb_size[IS_LUMA], &mb_x, &mb_y);

  if (MbQ->mb_type == I8MB)
    assert(MbQ->luma_transform_size_8x8_flag);

  filterNon8x8LumaEdgesFlag[1] =
    filterNon8x8LumaEdgesFlag[3] = !(MbQ->luma_transform_size_8x8_flag);

  filterLeftMbEdgeFlag = (mb_x != 0);
  filterTopMbEdgeFlag  = (mb_y != 0);

  if (p_Vid->mb_aff_frame_flag && mb_y == MB_BLOCK_SIZE && MbQ->mb_field)
    filterTopMbEdgeFlag = 0;

  if (MbQ->DFDisableIdc==2)
  {
    // don't filter at slice boundaries
    filterLeftMbEdgeFlag = MbQ->mbAvailA;
    // if this the bottom of a frame macroblock pair then always filter the top edge
    filterTopMbEdgeFlag  = (p_Vid->mb_aff_frame_flag && !MbQ->mb_field && (MbQAddr & 0x01)) ? 1 : MbQ->mbAvailB;
  }

  CheckAvailabilityOfNeighbors(MbQ);

  // Vertical deblocking
  for (edge = 0; edge < 4 ; ++edge )
  {
    // If cbp == 0 then deblocking for some macroblock types could be skipped                                                                  
    if (MbQ->cbp == 0)
    {
      if (filterNon8x8LumaEdgesFlag[edge] == 0 && active_sps->chroma_format_idc!=YUV444)
        continue;
      else if (edge > 0 && (currSlice->slice_type == P_SLICE || currSlice->slice_type == B_SLICE))
      {
        if ((MbQ->mb_type == 0 && currSlice->slice_type == P_SLICE) || (MbQ->mb_type == 1) || (MbQ->mb_type == 2))
          continue;
        else if ((edge & 0x01) && ((MbQ->mb_type == 3) || ((edge & 0x01) && MbQ->mb_type == 0 && currSlice->slice_type == B_SLICE && active_sps->direct_8x8_inference_flag)))
          continue;
      }
    }

    if( edge || filterLeftMbEdgeFlag )
    {
      // Strength for 4 blks in 1 stripe
      p_Vid->GetStrengthVer(Strength, MbQ, edge << 2, mvlimit); // Strength for 4 blks in 1 stripe

      if ( Strength[0] != 0 || Strength[1] != 0 || Strength[2] != 0 || Strength[3] !=0 ||
           Strength[4] != 0 || Strength[5] != 0 || Strength[6] != 0 || Strength[7] !=0 ||
           Strength[8] != 0 || Strength[9] != 0 || Strength[10] != 0 || Strength[11] !=0 ||
           Strength[12] != 0 || Strength[13] != 0 || Strength[14] != 0 || Strength[15] !=0 ) // only if one of the 16 Strength bytes is != 0
      {
        if (filterNon8x8LumaEdgesFlag[edge])
        {
          p_Vid->EdgeLoopLumaVer( PLANE_Y, imgY, Strength, MbQ, edge << 2) ;
          if (p_Vid->P444_joined)
          {
            p_Vid->EdgeLoopLumaVer(PLANE_U, imgUV[0], Strength, MbQ, edge << 2);
            p_Vid->EdgeLoopLumaVer(PLANE_V, imgUV[1], Strength, MbQ, edge << 2);
          }
        }
        if(p_Vid->yuv_format==YUV420 || p_Vid->yuv_format==YUV422 )
        {
          edge_cr = chroma_edge[0][edge][p_Vid->yuv_format];
          if( (imgUV != NULL) && (edge_cr >= 0))
          {
            p_Vid->EdgeLoopChromaVer( imgUV[0], Strength, MbQ, edge_cr, 0);
            p_Vid->EdgeLoopChromaVer( imgUV[1], Strength, MbQ, edge_cr, 1);
          }
        }
      }        
    }
  }//end edge

  // horizontal deblocking  
  for( edge = 0; edge < 4 ; ++edge )
  {
    // If cbp == 0 then deblocking for some macroblock types could be skipped
    if (MbQ->cbp == 0)
    {
      if (filterNon8x8LumaEdgesFlag[edge] == 0 && active_sps->chroma_format_idc==YUV420)
        continue;
      else if (edge > 0 && (currSlice->slice_type == P_SLICE || currSlice->slice_type == B_SLICE))
      {
        if (((MbQ->mb_type == PSKIP && currSlice->slice_type == P_SLICE) || (MbQ->mb_type == P16x16) || (MbQ->mb_type == P8x16)))
          continue;
        else if ((edge & 0x01) && (( MbQ->mb_type == P16x8) || (currSlice->slice_type == B_SLICE && MbQ->mb_type == BSKIP_DIRECT && active_sps->direct_8x8_inference_flag)))
          continue;
      }
    }

    if( edge || filterTopMbEdgeFlag )
    {
      // Strength for 4 blks in 1 stripe
      p_Vid->GetStrengthHor(Strength, MbQ, edge << 2, mvlimit);

      if ( Strength[0] != 0 || Strength[1] != 0 || Strength[2] != 0 || Strength[3] !=0 ||
           Strength[4] != 0 || Strength[5] != 0 || Strength[6] != 0 || Strength[7] !=0 ||
           Strength[8] != 0 || Strength[9] != 0 || Strength[10] != 0 || Strength[11] !=0 ||
           Strength[12] != 0 || Strength[13] != 0 || Strength[14] != 0 || Strength[15] !=0 ) // only if one of the 16 Strength bytes is != 0
      {
        if (filterNon8x8LumaEdgesFlag[edge])
        {
          p_Vid->EdgeLoopLumaHor( PLANE_Y, imgY, Strength, MbQ, edge << 2, p_Vid->width_padded) ;
          if (p_Vid->P444_joined)
          {
            p_Vid->EdgeLoopLumaHor(PLANE_U, imgUV[0], Strength, MbQ, edge << 2, p_Vid->width_padded);
            p_Vid->EdgeLoopLumaHor(PLANE_V, imgUV[1], Strength, MbQ, edge << 2, p_Vid->width_padded);
          }
        }
        if(p_Vid->yuv_format==YUV420 || p_Vid->yuv_format==YUV422 )
        {
          edge_cr = chroma_edge[1][edge][p_Vid->yuv_format];
          if( (imgUV != NULL) && (edge_cr >= 0))
          {
            p_Vid->EdgeLoopChromaHor( imgUV[0], Strength, MbQ, edge_cr, p_Vid->width_cr + p_Vid->pad_size_uv_x * 2, 0);
            p_Vid->EdgeLoopChromaHor( imgUV[1], Strength, MbQ, edge_cr, p_Vid->width_cr + p_Vid->pad_size_uv_x * 2, 1);
          }
        }
      }

      if (!edge && !MbQ->mb_field && p_Vid->mixedModeEdgeFlag) 
      {
        // this is the extra horizontal edge between a frame macroblock pair and a field above it
        MbQ->DeblockCall = 2;
        p_Vid->GetStrengthHor(Strength, MbQ, MB_BLOCK_SIZE, mvlimit); // Strength for 4 blks in 1 stripe
        //if( Strength[0] != 0 || Strength[1] != 0 || Strength[2] != 0 || Strength[3] != 0  )                      // only if one of the 4 Strength bytes is != 0
        {
          if (filterNon8x8LumaEdgesFlag[edge])
          {
            p_Vid->EdgeLoopLumaHor( PLANE_Y, imgY, Strength, MbQ, MB_BLOCK_SIZE, p_Vid->width_padded) ;
            if (p_Vid->P444_joined)
            {
              p_Vid->EdgeLoopLumaHor(PLANE_U, imgUV[0], Strength, MbQ, MB_BLOCK_SIZE, p_Vid->width_padded) ;
              p_Vid->EdgeLoopLumaHor(PLANE_V, imgUV[1], Strength, MbQ, MB_BLOCK_SIZE, p_Vid->width_padded) ;
            }
          }
          if( p_Vid->yuv_format == YUV420 || p_Vid->yuv_format==YUV422 )
          {
            edge_cr = chroma_edge[1][edge][p_Vid->yuv_format];
            if( (imgUV != NULL) && (edge_cr >= 0))
            {
              p_Vid->EdgeLoopChromaHor( imgUV[0], Strength, MbQ, MB_BLOCK_SIZE, p_Vid->width_cr+p_Vid->pad_size_uv_x*2, 0) ;
              p_Vid->EdgeLoopChromaHor( imgUV[1], Strength, MbQ, MB_BLOCK_SIZE, p_Vid->width_cr+p_Vid->pad_size_uv_x*2, 1) ;
            }
          }
        }
        MbQ->DeblockCall = 1;
      }
    }
  }//end edge

  MbQ->DeblockCall = 0;
}


