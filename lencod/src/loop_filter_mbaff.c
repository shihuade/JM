
/*!
 *************************************************************************************
 * \file loop_filter_mbaff.c
 *
 * \brief
 *    Loop Filter to reduce blocking artifacts on a macroblock level (MBAFF).
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



static void get_strength_ver_MBAff     (byte Strength[MB_BLOCK_SIZE], Macroblock *MbQ, int edge, int mvlimit);
static void get_strength_hor_MBAff     (byte Strength[MB_BLOCK_SIZE], Macroblock *MbQ, int edge, int mvlimit);
static void edge_loop_luma_ver_MBAff   (ColorPlane pl, imgpel** Img, byte Strength[MB_BLOCK_SIZE], Macroblock *MbQ, int edge);
static void edge_loop_luma_hor_MBAff   (ColorPlane pl, imgpel** Img, byte Strength[MB_BLOCK_SIZE], Macroblock *MbQ, int edge, int width);
static void edge_loop_chroma_ver_MBAff (imgpel** Img, byte Strength[MB_BLOCK_SIZE], Macroblock *MbQ, int edge, int uv);
static void edge_loop_chroma_hor_MBAff (imgpel** Img, byte Strength[MB_BLOCK_SIZE], Macroblock *MbQ, int edge, int width, int uv);


void set_loop_filter_functions_mbaff(VideoParameters *p_Vid)
{
  p_Vid->GetStrengthVer    = get_strength_ver_MBAff;
  p_Vid->GetStrengthHor    = get_strength_hor_MBAff;
  p_Vid->EdgeLoopLumaVer   = edge_loop_luma_ver_MBAff;
  p_Vid->EdgeLoopLumaHor   = edge_loop_luma_hor_MBAff;
  p_Vid->EdgeLoopChromaVer = edge_loop_chroma_ver_MBAff;
  p_Vid->EdgeLoopChromaHor = edge_loop_chroma_hor_MBAff;
}

/*!
 *********************************************************************************************
 * \brief
 *    returns a buffer of 16 Strength values for one stripe in a mb (for MBAFF)
 *********************************************************************************************
 */
static void get_strength_ver_MBAff(byte Strength[MB_BLOCK_SIZE], Macroblock *MbQ, int edge, int mvlimit)
{
  short  blkP, blkQ, idx;
  short  blk_x, blk_x2, blk_y, blk_y2 ;

  int    xQ, yQ;
  short  mb_x, mb_y;

  Macroblock *MbP;

  PixelPos pixP;
  VideoParameters *p_Vid = MbQ->p_Vid;
  PicMotionParams **mv_info = p_Vid->enc_picture->mv_info;

  for( idx = 0; idx < 16; ++idx )
  {    
    xQ = edge;
    yQ = idx;
    p_Vid->getNeighbour(MbQ, xQ - 1, yQ, p_Vid->mb_size[IS_LUMA], &pixP);
    blkQ = (short) ((yQ & 0xFFFC) + (xQ >> 2));
    blkP = (short) ((pixP.y & 0xFFFC) + (pixP.x >> 2));

    MbP = &(p_Vid->mb_data[pixP.mb_addr]);
    p_Vid->mixedModeEdgeFlag = (byte) (MbQ->mb_field != MbP->mb_field);   

    if (p_Vid->type==SP_SLICE || p_Vid->type==SI_SLICE)
    {
      Strength[idx] = (edge == 0) ? 4 : 3;
    }
    else
    {
      // Start with Strength=3. or Strength=4 for Mb-edge
      Strength[idx] = (edge == 0) ? 4 : 3;

      if(  !(MbP->mb_type==I4MB || MbP->mb_type==I16MB || MbP->mb_type==I8MB || MbP->mb_type==IPCM)
        && !(MbQ->mb_type==I4MB || MbQ->mb_type==I16MB || MbQ->mb_type==I8MB || MbQ->mb_type==IPCM) )
      {        
        if( ((MbQ->cbp_blk &  ((int64)1 << blkQ )) != 0) || ((MbP->cbp_blk &  ((int64)1 << blkP)) != 0) )
          Strength[idx] = 2 ;
        else
        {
          // if no coefs, but vector difference >= 1 set Strength=1
          // if this is a mixed mode edge then one set of reference pictures will be frame and the
          // other will be field
          if (p_Vid->mixedModeEdgeFlag)
          {
            (Strength[idx] = 1);
          }
          else
          {
            p_Vid->get_mb_block_pos (p_Vid->PicPos, MbQ->mbAddrX, &mb_x, &mb_y);
            blk_y  = (short) ((mb_y<<2) + (blkQ >> 2));
            blk_x  = (short) ((mb_x<<2) + (blkQ  & 3));
            blk_y2 = (short) (pixP.pos_y >> 2);
            blk_x2 = (short) (pixP.pos_x >> 2);
            {
              PicMotionParams *mv_info_p = &mv_info[blk_y ][blk_x ];
              PicMotionParams *mv_info_q = &mv_info[blk_y2][blk_x2];

              StorablePicturePtr ref_p0 = mv_info_p->ref_idx[LIST_0] == -1 ? NULL : mv_info_p->ref_pic[LIST_0];
              StorablePicturePtr ref_q0 = mv_info_q->ref_idx[LIST_0] == -1 ? NULL : mv_info_q->ref_pic[LIST_0];
              StorablePicturePtr ref_p1 = mv_info_p->ref_idx[LIST_1] == -1 ? NULL : mv_info_p->ref_pic[LIST_1];
              StorablePicturePtr ref_q1 = mv_info_q->ref_idx[LIST_1] == -1 ? NULL : mv_info_q->ref_pic[LIST_1];

              if ( ((ref_p0==ref_q0) && (ref_p1==ref_q1)) ||
                ((ref_p0==ref_q1) && (ref_p1==ref_q0)))
              {
                Strength[idx]=0;
                // L0 and L1 reference pictures of p0 are different; q0 as well
                if (ref_p0 != ref_p1)
                {
                  // compare MV for the same reference picture
                  if (ref_p0==ref_q0)
                  {
                    Strength[idx] =  (byte) (
                      compare_mvs(&mv_info_p->mv[LIST_0], &mv_info_q->mv[LIST_0], mvlimit) ||
                      compare_mvs(&mv_info_p->mv[LIST_1], &mv_info_q->mv[LIST_1], mvlimit));
                  }
                  else
                  {
                    Strength[idx] =  (byte) (
                      compare_mvs(&mv_info_p->mv[LIST_0], &mv_info_q->mv[LIST_1], mvlimit) ||
                      compare_mvs(&mv_info_p->mv[LIST_1], &mv_info_q->mv[LIST_0], mvlimit));
                  }
                }
                else
                { // L0 and L1 reference pictures of p0 are the same; q0 as well

                  Strength[idx] = (byte) ((
                    compare_mvs(&mv_info_p->mv[LIST_0], &mv_info_q->mv[LIST_0], mvlimit) ||
                    compare_mvs(&mv_info_p->mv[LIST_1], &mv_info_q->mv[LIST_1], mvlimit))
                    &&(
                    compare_mvs(&mv_info_p->mv[LIST_0], &mv_info_q->mv[LIST_1], mvlimit) ||
                    compare_mvs(&mv_info_p->mv[LIST_1], &mv_info_q->mv[LIST_0], mvlimit)));
                }
              }
              else
              {
                Strength[idx] = 1;
              }
            }
          }
        }
      }
    }
  }
}

/*!
 *********************************************************************************************
 * \brief
 *    returns a buffer of 16 Strength values for one stripe in a mb (for MBAFF)
 *********************************************************************************************
 */
static void get_strength_hor_MBAff(byte Strength[MB_BLOCK_SIZE], Macroblock *MbQ, int edge, int mvlimit)
{
  short  blkP, blkQ, idx;
  short  blk_x, blk_x2, blk_y, blk_y2 ;

  int    xQ, yQ;
  short  mb_x, mb_y;

  Macroblock *MbP;

  PixelPos pixP;
  VideoParameters *p_Vid = MbQ->p_Vid;
  PicMotionParams **mv_info = p_Vid->enc_picture->mv_info;


  for( idx = 0; idx < 16; ++idx )
  {    
    xQ = idx;
    yQ = (edge < MB_BLOCK_SIZE ? edge : 1);
    p_Vid->getNeighbour(MbQ, xQ, yQ - 1, p_Vid->mb_size[IS_LUMA], &pixP);
    blkQ = (short) ((yQ & 0xFFFC) + (xQ >> 2));
    blkP = (short) ((pixP.y & 0xFFFC) + (pixP.x >> 2));

    MbP = &(p_Vid->mb_data[pixP.mb_addr]);
    p_Vid->mixedModeEdgeFlag = (byte) (MbQ->mb_field != MbP->mb_field);   

    if (p_Vid->type==SP_SLICE || p_Vid->type==SI_SLICE)
    {
      Strength[idx] = (edge == 0 && (((!p_Vid->mb_aff_frame_flag && (p_Vid->structure==FRAME)) ||
        (p_Vid->mb_aff_frame_flag && !MbP->mb_field && !MbQ->mb_field)))) ? 4 : 3;
    }
    else
    {
      // Start with Strength=3. or Strength=4 for Mb-edge
      Strength[idx] = (edge == 0 && (((!p_Vid->mb_aff_frame_flag && (p_Vid->structure==FRAME)) ||
        (p_Vid->mb_aff_frame_flag && !MbP->mb_field && !MbQ->mb_field)))) ? 4 : 3;

      if(  !(MbP->mb_type==I4MB || MbP->mb_type==I16MB || MbP->mb_type==I8MB || MbP->mb_type==IPCM)
        && !(MbQ->mb_type==I4MB || MbQ->mb_type==I16MB || MbQ->mb_type==I8MB || MbQ->mb_type==IPCM) )
      {        
        if( ((MbQ->cbp_blk &  ((int64)1 << blkQ )) != 0) || ((MbP->cbp_blk &  ((int64)1 << blkP)) != 0) )
          Strength[idx] = 2 ;
        else
        {
          // if no coefs, but vector difference >= 1 set Strength=1
          // if this is a mixed mode edge then one set of reference pictures will be frame and the
          // other will be field
          if (p_Vid->mixedModeEdgeFlag)
          {
            (Strength[idx] = 1);
          }
          else
          {
            p_Vid->get_mb_block_pos (p_Vid->PicPos, MbQ->mbAddrX, &mb_x, &mb_y);
            blk_y  = (short) ((mb_y<<2) + (blkQ >> 2));
            blk_x  = (short) ((mb_x<<2) + (blkQ  & 3));
            blk_y2 = (short) (pixP.pos_y >> 2);
            blk_x2 = (short) (pixP.pos_x >> 2);
            {
              PicMotionParams *mv_info_p = &mv_info[blk_y ][blk_x ];
              PicMotionParams *mv_info_q = &mv_info[blk_y2][blk_x2];

            StorablePicturePtr ref_p0 = mv_info_p->ref_idx[LIST_0] == -1 ? NULL : mv_info_p->ref_pic[LIST_0];
            StorablePicturePtr ref_q0 = mv_info_q->ref_idx[LIST_0] == -1 ? NULL : mv_info_q->ref_pic[LIST_0];
            StorablePicturePtr ref_p1 = mv_info_p->ref_idx[LIST_1] == -1 ? NULL : mv_info_p->ref_pic[LIST_1];
            StorablePicturePtr ref_q1 = mv_info_q->ref_idx[LIST_1] == -1 ? NULL : mv_info_q->ref_pic[LIST_1];

              if ( ((ref_p0==ref_q0) && (ref_p1==ref_q1)) ||
                ((ref_p0==ref_q1) && (ref_p1==ref_q0)))
              {
                Strength[idx]=0;
                // L0 and L1 reference pictures of p0 are different; q0 as well
                if (ref_p0 != ref_p1)
                {
                  // compare MV for the same reference picture
                  if (ref_p0==ref_q0)
                  {
                    Strength[idx] =  (byte) (
                      compare_mvs(&mv_info_p->mv[LIST_0], &mv_info_q->mv[LIST_0], mvlimit) ||
                      compare_mvs(&mv_info_p->mv[LIST_1], &mv_info_q->mv[LIST_1], mvlimit));
                  }
                  else
                  {
                    Strength[idx] =  (byte) (
                      compare_mvs(&mv_info_p->mv[LIST_0], &mv_info_q->mv[LIST_1], mvlimit) ||
                      compare_mvs(&mv_info_p->mv[LIST_1], &mv_info_q->mv[LIST_0], mvlimit));
                  }
                }
                else
                { // L0 and L1 reference pictures of p0 are the same; q0 as well

                  Strength[idx] = (byte) ((
                    compare_mvs(&mv_info_p->mv[LIST_0], &mv_info_q->mv[LIST_0], mvlimit) ||
                    compare_mvs(&mv_info_p->mv[LIST_1], &mv_info_q->mv[LIST_1], mvlimit))
                    &&(
                    compare_mvs(&mv_info_p->mv[LIST_0], &mv_info_q->mv[LIST_1], mvlimit) ||
                    compare_mvs(&mv_info_p->mv[LIST_1], &mv_info_q->mv[LIST_0], mvlimit)));
                }
              }
              else
              {
                Strength[idx] = 1;
              }
            }
          }
        }
      }
    }
  }
}




/*!
 *****************************************************************************************
 * \brief
 *    Filters 16 pel block edge of Super MB Frame coded MBs
 *****************************************************************************************
 */
static void edge_loop_luma_ver_MBAff(ColorPlane pl, imgpel** Img, byte Strength[16], Macroblock *MbQ, int edge)
{
  int      pel, ap = 0, aq = 0, Strng ;
  int      C0, tc0, dif;
  imgpel   L2 = 0, L1, L0, R0, R1, R2 = 0, L3, R3;
  int      RL0;
  int      Alpha = 0, Beta = 0 ;
  const byte* ClipTab = NULL;
  int      small_gap;
  int      indexA, indexB;

  int      QP;
  int      xQ, yQ;

  PixelPos pixP, pixQ;

  VideoParameters *p_Vid = MbQ->p_Vid;
  int      bitdepth_scale = pl? p_Vid->bitdepth_scale[IS_CHROMA] : p_Vid->bitdepth_scale[IS_LUMA];
  int      max_imgpel_value = p_Vid->max_pel_value_comp[pl];

  int AlphaC0Offset = MbQ->DFAlphaC0Offset;
  int BetaOffset = MbQ->DFBetaOffset;  

  Macroblock *MbP;
  imgpel   *SrcPtrP, *SrcPtrQ;

  for( pel = 0 ; pel < MB_BLOCK_SIZE ; ++pel )
  {
    xQ = edge;
    yQ = pel;
    p_Vid->getNeighbour(MbQ, xQ - 1, yQ, p_Vid->mb_size[IS_LUMA], &pixP);     

    if (pixP.available || (MbQ->DFDisableIdc== 0))
    {
      if( (Strng = Strength[pel]) != 0)
      {
        p_Vid->getNeighbour(MbQ, xQ, yQ, p_Vid->mb_size[IS_LUMA], &pixQ);

        MbP = &(p_Vid->mb_data[pixP.mb_addr]);

        SrcPtrQ = &(Img[pixQ.pos_y][pixQ.pos_x]);
        SrcPtrP = &(Img[pixP.pos_y][pixP.pos_x]);

        // Average QP of the two blocks
        QP = pl? ((MbP->qpc[pl-1] + MbQ->qpc[pl-1] + 1) >> 1) : (MbP->qp + MbQ->qp + 1) >> 1;

        indexA = iClip3(0, MAX_QP, QP + AlphaC0Offset);
        indexB = iClip3(0, MAX_QP, QP + BetaOffset);

        Alpha   = ALPHA_TABLE[indexA] * bitdepth_scale;
        Beta    = BETA_TABLE [indexB] * bitdepth_scale;
        ClipTab = CLIP_TAB[indexA];

        L3  = SrcPtrP[-3];
        L2  = SrcPtrP[-2];
        L1  = SrcPtrP[-1];
        L0  = SrcPtrP[0] ;
        R0  = SrcPtrQ[0] ;      
        R1  = SrcPtrQ[ 1];      
        R2  = SrcPtrQ[ 2];
        R3  = SrcPtrQ[ 3];

        if( iabs( R0 - L0 ) < Alpha )
        {          
          if ((iabs( R0 - R1) < Beta )   && (iabs(L0 - L1) < Beta ))
          {
            if(Strng == 4 )    // INTRA strong filtering
            {
              RL0 = L0 + R0;
              small_gap = (iabs( R0 - L0 ) < ((Alpha >> 2) + 2));
              aq  = ( iabs( R0 - R2) < Beta ) & small_gap;               
              ap  = ( iabs( L0 - L2) < Beta ) & small_gap;

              if (ap)
              {
                SrcPtrP[-2 ] = (imgpel) ((((L3 + L2) << 1) + L2 + L1 + RL0 + 4) >> 3);
                SrcPtrP[-1 ] = (imgpel) (( L2 + L1 + L0 + R0 + 2) >> 2);
                SrcPtrP[ 0 ] = (imgpel) (( R1 + ((L1 + RL0) << 1) +  L2 + 4) >> 3);
              }
              else
              {
                SrcPtrP[ 0 ] = (imgpel) (((L1 << 1) + L0 + R1 + 2) >> 2) ;
              }

              if (aq)
              {
                SrcPtrQ[ 0 ] = (imgpel) (( L1 + ((R1 + RL0) << 1) +  R2 + 4) >> 3);
                SrcPtrQ[ 1 ] = (imgpel) (( R2 + R0 + R1 + L0 + 2) >> 2);
                SrcPtrQ[ 2 ] = (imgpel) ((((R3 + R2) << 1) + R2 + R1 + RL0 + 4) >> 3);
              }
              else
              {
                SrcPtrQ[ 0 ] = (imgpel) (((R1 << 1) + R0 + L1 + 2) >> 2);
              }
            }
            else   // normal filtering
            {              
              RL0 = (L0 + R0 + 1) >> 1;
              aq  = (iabs( R0 - R2) < Beta);
              ap  = (iabs( L0 - L2) < Beta);

              C0  = ClipTab[ Strng ] * bitdepth_scale;
              tc0  = (C0 + ap + aq) ;
              dif = iClip3( -tc0, tc0, (((R0 - L0) << 2) + (L1 - R1) + 4) >> 3) ;

              if( ap )
                *(SrcPtrP - 1) += iClip3( -C0,  C0, ( L2 + RL0 - (L1 << 1)) >> 1 ) ;

              *SrcPtrP  = (imgpel) iClip1 (max_imgpel_value, L0 + dif) ;
              *SrcPtrQ  = (imgpel) iClip1 (max_imgpel_value, R0 - dif) ;

              if( aq  )
                *(SrcPtrQ + 1) += iClip3( -C0,  C0, ( R2 + RL0 - (R1 << 1)) >> 1 ) ;
            }            
          }
        }
      }
    }
  }
}

/*!
 *****************************************************************************************
 * \brief
 *    Filters 16 pel block edge of Super MB Frame coded MBs
 *****************************************************************************************
 */
static void edge_loop_luma_hor_MBAff(ColorPlane pl, imgpel** Img, byte Strength[16], Macroblock *MbQ, int edge, int width)
{
  int      pel, ap = 0, aq = 0, Strng ;
  int      incP, incQ;
  int      C0, tc0, dif;
  imgpel   L2 = 0, L1, L0, R0, R1, R2 = 0, L3, R3;
  int      RL0;
  int      Alpha = 0, Beta = 0 ;
  const byte* ClipTab = NULL;
  int      small_gap;
  int      indexA, indexB;

  int      QP;
  int      xQ, yQ;

  PixelPos pixP, pixQ;
  VideoParameters *p_Vid = MbQ->p_Vid;
  int      bitdepth_scale = pl? p_Vid->bitdepth_scale[IS_CHROMA] : p_Vid->bitdepth_scale[IS_LUMA];
  int      max_imgpel_value = p_Vid->max_pel_value_comp[pl];

  int AlphaC0Offset = MbQ->DFAlphaC0Offset;
  int BetaOffset = MbQ->DFBetaOffset;

  Macroblock *MbP;
  imgpel   *SrcPtrP, *SrcPtrQ;

  for( pel = 0 ; pel < MB_BLOCK_SIZE ; ++pel )
  {
    xQ = pel;
    yQ = (edge < 16 ? edge : 1);
    p_Vid->getNeighbour(MbQ, xQ, yQ - 1, p_Vid->mb_size[IS_LUMA], &pixP);     

    if (pixP.available || (MbQ->DFDisableIdc== 0))
    {
      if( (Strng = Strength[pel]) != 0)
      {
        p_Vid->getNeighbour(MbQ, xQ, yQ, p_Vid->mb_size[IS_LUMA], &pixQ);

        MbP = &(p_Vid->mb_data[pixP.mb_addr]);

        incQ    = ((MbP->mb_field && !MbQ->mb_field) ? 2 * width : width);
        incP    = ((MbQ->mb_field && !MbP->mb_field) ? 2 * width : width);
        SrcPtrQ = &(Img[pixQ.pos_y][pixQ.pos_x]);
        SrcPtrP = &(Img[pixP.pos_y][pixP.pos_x]);

        // Average QP of the two blocks
        QP = pl? ((MbP->qpc[pl-1] + MbQ->qpc[pl-1] + 1) >> 1) : (MbP->qp + MbQ->qp + 1) >> 1;

        indexA = iClip3(0, MAX_QP, QP + AlphaC0Offset);
        indexB = iClip3(0, MAX_QP, QP + BetaOffset);

        Alpha   = ALPHA_TABLE[indexA] * bitdepth_scale;
        Beta    = BETA_TABLE [indexB] * bitdepth_scale;
        ClipTab = CLIP_TAB[indexA];

        L3  = SrcPtrP[-incP*3];
        L2  = SrcPtrP[-incP*2];
        L1  = SrcPtrP[-incP];
        L0  = SrcPtrP[0] ;
        R0  = SrcPtrQ[0] ;      
        R1  = SrcPtrQ[ incQ];      
        R2  = SrcPtrQ[ incQ*2];
        R3  = SrcPtrQ[ incQ*3];

        if( iabs( R0 - L0 ) < Alpha )
        {          
          if ((iabs( R0 - R1) < Beta )   && (iabs(L0 - L1) < Beta ))
          {
            if(Strng == 4 )    // INTRA strong filtering
            {
              RL0 = L0 + R0;
              small_gap = (iabs( R0 - L0 ) < ((Alpha >> 2) + 2));
              aq  = ( iabs( R0 - R2) < Beta ) & small_gap;               
              ap  = ( iabs( L0 - L2) < Beta ) & small_gap;

              if (ap)
              {
                SrcPtrP[-incP * 2] = (imgpel) ((((L3 + L2) << 1) + L2 + L1 + RL0 + 4) >> 3);
                SrcPtrP[-incP    ] = (imgpel) (( L2 + L1 + L0 + R0 + 2) >> 2);
                SrcPtrP[    0    ] = (imgpel) (( R1 + ((L1 + RL0) << 1) +  L2 + 4) >> 3);
              }
              else
              {
                SrcPtrP[     0     ] = (imgpel) (((L1 << 1) + L0 + R1 + 2) >> 2) ;
              }

              if (aq)
              {
                SrcPtrQ[    0     ] = (imgpel) (( L1 + ((R1 + RL0) << 1) +  R2 + 4) >> 3);
                SrcPtrQ[ incQ     ] = (imgpel) (( R2 + R0 + R1 + L0 + 2) >> 2);
                SrcPtrQ[ incQ * 2 ] = (imgpel) ((((R3 + R2) << 1) + R2 + R1 + RL0 + 4) >> 3);
              }
              else
              {
                SrcPtrQ[    0     ] = (imgpel) (((R1 << 1) + R0 + L1 + 2) >> 2);
              }
            }
            else   // normal filtering
            {              
              RL0 = (L0 + R0 + 1) >> 1;
              aq  = (iabs( R0 - R2) < Beta);
              ap  = (iabs( L0 - L2) < Beta);

              C0  = ClipTab[ Strng ] * bitdepth_scale;
              tc0  = (C0 + ap + aq) ;
              dif = iClip3( -tc0, tc0, (((R0 - L0) << 2) + (L1 - R1) + 4) >> 3) ;

              if( ap )
                *(SrcPtrP - incP) += iClip3( -C0,  C0, ( L2 + RL0 - (L1 << 1)) >> 1 ) ;

              *SrcPtrP  = (imgpel) iClip1 (max_imgpel_value, L0 + dif) ;
              *SrcPtrQ  = (imgpel) iClip1 (max_imgpel_value, R0 - dif) ;

              if( aq  )
                *(SrcPtrQ + incQ) += iClip3( -C0,  C0, ( R2 + RL0 - (R1 << 1)) >> 1 ) ;
            }            
          }
        }
      }
    }
  }
}

/*!
*****************************************************************************************
* \brief
*    Filters chroma block edge for MBAFF types
*****************************************************************************************
 */
static void edge_loop_chroma_ver_MBAff(imgpel** Img, byte Strength[16], Macroblock *MbQ, int edge, int uv)
{
  int      pel, Strng ;
  int      C0, tc0, dif;
  imgpel   L1, L0, R0, R1;
  int      Alpha = 0, Beta = 0;
  const byte* ClipTab = NULL;
  int      indexA, indexB;
  VideoParameters *p_Vid = MbQ->p_Vid;
  int      PelNum = pelnum_cr[0][p_Vid->yuv_format];
  int      StrengthIdx;
  int      QP;
  int      xQ, yQ;
  PixelPos pixP, pixQ;
  int      bitdepth_scale = p_Vid->bitdepth_scale[IS_CHROMA];
  int      max_imgpel_value = p_Vid->max_pel_value_comp[uv + 1];
  
  int      AlphaC0Offset = MbQ->DFAlphaC0Offset;
  int      BetaOffset    = MbQ->DFBetaOffset;
  Macroblock *MbP;
  imgpel   *SrcPtrP, *SrcPtrQ;

  for( pel = 0 ; pel < PelNum ; ++pel )
  {
    xQ = edge;
    yQ = pel;
    p_Vid->getNeighbour(MbQ, xQ, yQ, p_Vid->mb_size[IS_CHROMA], &pixQ);
    p_Vid->getNeighbour(MbQ, xQ - 1, yQ, p_Vid->mb_size[IS_CHROMA], &pixP);    
    MbP = &(p_Vid->mb_data[pixP.mb_addr]);    
    StrengthIdx = (PelNum == 8) ? ((MbQ->mb_field && !MbP->mb_field) ? pel << 1 :((pel >> 1) << 2) + (pel & 0x01)) : pel;

    if (pixP.available || (MbQ->DFDisableIdc == 0))
    {
      if( (Strng = Strength[StrengthIdx]) != 0)
      {
        SrcPtrQ = &(Img[pixQ.pos_y][pixQ.pos_x]);
        SrcPtrP = &(Img[pixP.pos_y][pixP.pos_x]);

        // Average QP of the two blocks
        QP = (MbP->qpc[uv] + MbQ->qpc[uv] + 1) >> 1;

        indexA = iClip3(0, MAX_QP, QP + AlphaC0Offset);
        indexB = iClip3(0, MAX_QP, QP + BetaOffset);

        Alpha   = ALPHA_TABLE[indexA] * bitdepth_scale;
        Beta    = BETA_TABLE [indexB] * bitdepth_scale;
        ClipTab = CLIP_TAB[indexA];

        L1  = SrcPtrP[-1];
        L0  = SrcPtrP[0] ;
        R0  = SrcPtrQ[0] ;      
        R1  = SrcPtrQ[ 1];      

        if( iabs( R0 - L0 ) < Alpha )
        {          
          if( ((iabs( R0 - R1) - Beta < 0)  && (iabs(L0 - L1) - Beta < 0 ))  )
          {
            if( Strng == 4 )    // INTRA strong filtering
            {
              SrcPtrQ[0] = (imgpel) ( ((R1 << 1) + R0 + L1 + 2) >> 2 );
              SrcPtrP[0] = (imgpel) ( ((L1 << 1) + L0 + R1 + 2) >> 2 );
            }
            else
            {
              C0  = ClipTab[ Strng ] * bitdepth_scale;
              tc0  = (C0 + 1);
              dif = iClip3( -tc0, tc0, ( ((R0 - L0) << 2) + (L1 - R1) + 4) >> 3 );

              SrcPtrP[0] = (imgpel) iClip1 ( max_imgpel_value, L0 + dif );
              SrcPtrQ[0] = (imgpel) iClip1 ( max_imgpel_value, R0 - dif );
            }
          }
        }
      }
    }
  }
}

/*!
*****************************************************************************************
* \brief
*    Filters chroma block edge for MBAFF types
*****************************************************************************************
 */
static void edge_loop_chroma_hor_MBAff(imgpel** Img, byte Strength[16], Macroblock *MbQ, int edge, int width, int uv)
{
  int      pel, Strng ;
  int      incP, incQ;
  int      C0, tc0, dif;
  imgpel   L1, L0, R0, R1;
  int      Alpha = 0, Beta = 0;
  const byte* ClipTab = NULL;
  int      indexA, indexB;
  VideoParameters *p_Vid = MbQ->p_Vid;
  int      PelNum = pelnum_cr[1][p_Vid->yuv_format];
  int      StrengthIdx;
  int      QP;
  int      xQ, yQ;
  PixelPos pixP, pixQ;
  int      bitdepth_scale = p_Vid->bitdepth_scale[IS_CHROMA];
  int      max_imgpel_value = p_Vid->max_pel_value_comp[uv + 1];
  
  int      AlphaC0Offset = MbQ->DFAlphaC0Offset;
  int      BetaOffset    = MbQ->DFBetaOffset;  
  Macroblock *MbP;
  imgpel   *SrcPtrP, *SrcPtrQ;

  for( pel = 0 ; pel < PelNum ; ++pel )
  {
    xQ = pel;
    yQ = (edge < 16? edge : 1);
    p_Vid->getNeighbour(MbQ, xQ, yQ, p_Vid->mb_size[IS_CHROMA], &pixQ);
    p_Vid->getNeighbour(MbQ, xQ, yQ - 1, p_Vid->mb_size[IS_CHROMA], &pixP);    
    MbP = &(p_Vid->mb_data[pixP.mb_addr]);    
    StrengthIdx = (PelNum == 8) ? ((MbQ->mb_field && !MbP->mb_field) ? pel << 1 :((pel >> 1) << 2) + (pel & 0x01)) : pel;

    if (pixP.available || (MbQ->DFDisableIdc == 0))
    {
      if( (Strng = Strength[StrengthIdx]) != 0)
      {
        incQ = ((MbP->mb_field && !MbQ->mb_field) ? 2 * width : width);
        incP = ((MbQ->mb_field && !MbP->mb_field) ? 2 * width : width);
        SrcPtrQ = &(Img[pixQ.pos_y][pixQ.pos_x]);
        SrcPtrP = &(Img[pixP.pos_y][pixP.pos_x]);

        // Average QP of the two blocks
        QP = (MbP->qpc[uv] + MbQ->qpc[uv] + 1) >> 1;

        indexA = iClip3(0, MAX_QP, QP + AlphaC0Offset);
        indexB = iClip3(0, MAX_QP, QP + BetaOffset);

        Alpha   = ALPHA_TABLE[indexA] * bitdepth_scale;
        Beta    = BETA_TABLE [indexB] * bitdepth_scale;
        ClipTab = CLIP_TAB[indexA];

        L1  = SrcPtrP[-incP];
        L0  = SrcPtrP[0] ;
        R0  = SrcPtrQ[0] ;      
        R1  = SrcPtrQ[ incQ];      

        if( iabs( R0 - L0 ) < Alpha )
        {          
          if( ((iabs( R0 - R1) - Beta < 0)  && (iabs(L0 - L1) - Beta < 0 ))  )
          {
            if( Strng == 4 )    // INTRA strong filtering
            {
              SrcPtrQ[0] = (imgpel) ( ((R1 << 1) + R0 + L1 + 2) >> 2 );
              SrcPtrP[0] = (imgpel) ( ((L1 << 1) + L0 + R1 + 2) >> 2 );
            }
            else
            {
              C0  = ClipTab[ Strng ] * bitdepth_scale;
              tc0  = (C0 + 1);
              dif = iClip3( -tc0, tc0, ( ((R0 - L0) << 2) + (L1 - R1) + 4) >> 3 );

              SrcPtrP[0] = (imgpel) iClip1 ( max_imgpel_value, L0 + dif );
              SrcPtrQ[0] = (imgpel) iClip1 ( max_imgpel_value, R0 - dif );
            }
          }
        }
      }
    }
  }
}


