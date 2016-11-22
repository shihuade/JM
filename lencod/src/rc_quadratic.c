
/*!
 ***************************************************************************
 * \file rc_quadratic.c
 *
 * \brief
 *    Rate Control algorithm
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Siwei Ma             <swma@jdl.ac.cn>
 *     - Zhengguo LI          <ezgli@lit.a-star.edu.sg>
 *     - Athanasios Leontaris <aleon@dolby.com>
 *
 * \date
 *   16 Jan. 2003
 **************************************************************************
 */

#include <math.h>
#include <limits.h>

#include "global.h"
#include "ratectl.h"


static const float THETA = 1.3636F;
static const float OMEGA = 0.9F;
static const float MINVALUE = 4.0F;
/*!
 *************************************************************************************
 * \brief
 *    Dynamically allocate memory needed for rate control
 *
 *************************************************************************************
 */
void rc_alloc_quadratic( VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic **p_quad )
{
  int rcBufSize = p_Vid->FrameSizeInMbs / p_Inp->basicunit;

  RCQuadratic *lprc;

  (*p_quad) = (RCQuadratic *) malloc ( sizeof( RCQuadratic ) );
  if (NULL==(*p_quad))
  {
    no_mem_exit("rc_alloc_quadratic: (*p_quad)");
  }
  lprc = *p_quad;

  lprc->PreviousFrameMAD = 1.0;
  lprc->CurrentFrameMAD = 1.0;
  lprc->Pprev_bits = 0;
  lprc->Target = 0;
  lprc->TargetField = 0;
  lprc->LowerBound = 0;
  lprc->UpperBound1 = INT_MAX;
  lprc->UpperBound2 = INT_MAX;
  lprc->Wp = 0.0;
  lprc->Wb = 0.0;
  lprc->AveWb = 0.0;
  lprc->PAveFrameQP   = p_Inp->qp[I_SLICE] + p_Vid->bitdepth_luma_qp_scale;
  lprc->m_Qc          = lprc->PAveFrameQP;
  lprc->FieldQPBuffer = lprc->PAveFrameQP;
  lprc->FrameQPBuffer = lprc->PAveFrameQP;
  lprc->PAverageQp    = lprc->PAveFrameQP;
  lprc->MyInitialQp   = lprc->PAveFrameQP;
  lprc->AveWb         = 0.0;

  lprc->BUPFMAD = (double*) calloc ((rcBufSize), sizeof (double));
  if (NULL==lprc->BUPFMAD)
  {
    no_mem_exit("rc_alloc_quadratic: lprc->BUPFMAD");
  }

  lprc->BUCFMAD = (double*) calloc ((rcBufSize), sizeof (double));
  if (NULL==lprc->BUCFMAD)
  {
    no_mem_exit("rc_alloc_quadratic: lprc->BUCFMAD");
  }

  lprc->FCBUCFMAD = (double*) calloc ((rcBufSize), sizeof (double));
  if (NULL==lprc->FCBUCFMAD)
  {
    no_mem_exit("rc_alloc_quadratic: lprc->FCBUCFMAD");
  }

  lprc->FCBUPFMAD = (double*) calloc ((rcBufSize), sizeof (double));
  if (NULL==lprc->FCBUPFMAD)
  {
    no_mem_exit("rc_alloc_quadratic: lprc->FCBUPFMAD");
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Copy JVT rate control objects
 *
 *************************************************************************************
 */
void rc_copy_quadratic( VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *dst, RCQuadratic *src )
{
  int rcBufSize = p_Vid->FrameSizeInMbs / p_Inp->basicunit;
  /* buffer original addresses for which memory has been allocated */
  double   *tmpBUPFMAD = dst->BUPFMAD;
  double   *tmpBUCFMAD = dst->BUCFMAD;
  double *tmpFCBUPFMAD = dst->FCBUPFMAD;
  double *tmpFCBUCFMAD = dst->FCBUCFMAD;

  /* copy object */
  memcpy( (void *)dst, (void *)src, sizeof(RCQuadratic) );

  /* restore original addresses */
  dst->BUPFMAD   = tmpBUPFMAD;
  dst->BUCFMAD   = tmpBUCFMAD;
  dst->FCBUPFMAD = tmpFCBUPFMAD;
  dst->FCBUCFMAD = tmpFCBUCFMAD;

  /* copy MADs */
  memcpy( (void *)dst->BUPFMAD,   (void *)src->BUPFMAD,   (rcBufSize) * sizeof (double) );
  memcpy( (void *)dst->BUCFMAD,   (void *)src->BUCFMAD,   (rcBufSize) * sizeof (double) );
  memcpy( (void *)dst->FCBUPFMAD, (void *)src->FCBUPFMAD, (rcBufSize) * sizeof (double) );
  memcpy( (void *)dst->FCBUCFMAD, (void *)src->FCBUCFMAD, (rcBufSize) * sizeof (double) );
}

/*!
 *************************************************************************************
 * \brief
 *    Free memory needed for rate control
 *
 *************************************************************************************
*/
void rc_free_quadratic(RCQuadratic **p_quad)
{
  if (NULL!=(*p_quad)->BUPFMAD)
  {
    free ((*p_quad)->BUPFMAD);
    (*p_quad)->BUPFMAD = NULL;
  }
  if (NULL!=(*p_quad)->BUCFMAD)
  {
    free ((*p_quad)->BUCFMAD);
    (*p_quad)->BUCFMAD = NULL;
  }
  if (NULL!=(*p_quad)->FCBUCFMAD)
  {
    free ((*p_quad)->FCBUCFMAD);
    (*p_quad)->FCBUCFMAD = NULL;
  }
  if (NULL!=(*p_quad)->FCBUPFMAD)
  {
    free ((*p_quad)->FCBUPFMAD);
    (*p_quad)->FCBUPFMAD = NULL;
  }
  if (NULL!=(*p_quad))
  {
    free ((*p_quad));
    (*p_quad) = NULL;
  }
}


/*!
 *************************************************************************************
 * \brief
 *    Initialize rate control parameters
 *
 *************************************************************************************
*/
void rc_init_seq(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen)
{
  double L1,L2,L3,bpp;
  int qp, i;

  switch ( p_Inp->RCUpdateMode )
  {
  case RC_MODE_0:
    p_Vid->updateQP = updateQPRC0;
    break;
  case RC_MODE_1:
    p_Vid->updateQP = updateQPRC1;
    break;
  case RC_MODE_2:
    p_Vid->updateQP = updateQPRC2;
    break;
  case RC_MODE_3:
    p_Vid->updateQP = updateQPRC3;
    break;
  default:
    p_Vid->updateQP = updateQPRC0;
    break;
  }
  // set function pointers
  p_Vid->rc_update_pict_frame_ptr = rc_update_pict_frame;
  p_Vid->rc_update_picture_ptr    = rc_update_picture;
  p_Vid->rc_init_pict_ptr         = rc_init_pict;

  p_quad->Xp=0;
  p_quad->Xb=0;

  p_quad->bit_rate = (float) p_Inp->bit_rate;
  p_quad->frame_rate = p_Vid->framerate;
  p_quad->PrevBitRate = p_quad->bit_rate;

  /*compute the total number of MBs in a frame*/
  if(p_Inp->basicunit > p_Vid->FrameSizeInMbs)
    p_Inp->basicunit = p_Vid->FrameSizeInMbs;
  if(p_Inp->basicunit < p_Vid->FrameSizeInMbs)
    p_quad->TotalNumberofBasicUnit = p_Vid->FrameSizeInMbs/p_Inp->basicunit;
  else
    p_quad->TotalNumberofBasicUnit = 1;

  /*initialize the parameters of fluid flow traffic model*/
  p_gen->CurrentBufferFullness = 0;
  p_quad->GOPTargetBufferLevel = (double) p_gen->CurrentBufferFullness;

  /*initialize the previous window size*/
  p_quad->m_windowSize    = 0;
  p_quad->MADm_windowSize = 0;
  p_gen->NumberofCodedBFrame = 0;
  p_quad->NumberofCodedPFrame = 0;
  p_gen->NumberofGOP         = 0;
  /*remaining # of bits in GOP */
  p_gen->RemainingBits = 0;
  /*control parameter */
  if(p_Inp->NumberBFrames>0)
  {
    p_quad->GAMMAP=0.25;
    p_quad->BETAP=0.9;
  }
  else
  {
    p_quad->GAMMAP=0.5;
    p_quad->BETAP=0.5;
  }

  /*quadratic rate-distortion model*/
  p_quad->PPreHeader=0;

  p_quad->Pm_X1 = p_quad->bit_rate * 1.0;
  p_quad->Pm_X2 = 0.0;
  /* linear prediction model for P picture*/
  p_quad->PMADPictureC1 = 1.0;
  p_quad->PMADPictureC2 = 0.0;

  // Initialize values
  for(i=0;i<21;i++)
  {
    p_quad->Pm_rgQp[i] = 0;
    p_quad->Pm_rgRp[i] = 0.0;
    p_quad->PPictureMAD[i] = 0.0;
  }

  //Define the largest variation of quantization parameters
  p_quad->PMaxQpChange = p_Inp->RCMaxQPChange;

  /*basic unit layer rate control*/
  p_quad->PAveHeaderBits1 = 0;
  p_quad->PAveHeaderBits3 = 0;
  p_quad->DDquant = (p_quad->TotalNumberofBasicUnit>=9 ? 1 : 2);

  p_quad->MBPerRow = p_Vid->PicWidthInMbs;

  /*adaptive field/frame coding*/
  p_gen->FieldControl=0;  

  if (p_Inp->SeinitialQP==0)
  {
    /*compute the initial QP*/
    bpp = 1.0*p_quad->bit_rate /(p_quad->frame_rate*p_Vid->size);

    if (p_Vid->width == 176)
    {
      L1 = 0.1;
      L2 = 0.3;
      L3 = 0.6;
    }
    else if (p_Vid->width == 352)
    {
      L1 = 0.2;
      L2 = 0.6;
      L3 = 1.2;
    }
    else
    {
      L1 = 0.6;
      L2 = 1.4;
      L3 = 2.4;
    }
    if (bpp<= L1)
      qp = 35;
    else if(bpp<=L2)
      qp = 25;
    else if(bpp<=L3)
      qp = 20;
    else
      qp = 10;
    p_Inp->SeinitialQP = qp;
  }

  // high bit-depth
  p_quad->bitdepth_qp_scale = p_Vid->bitdepth_luma_qp_scale;
}

/*!
 *************************************************************************************
 * \brief
 *    Initialize one GOP
 *
 *************************************************************************************
*/
void rc_init_GOP(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int np, int nb)
{
  //int OverBits;
  int GOPDquant;
  int64 AllocatedBits;

  // bit allocation for RC_MODE_3
  switch( p_Inp->RCUpdateMode )
  {
  case RC_MODE_3:
    {
      int sum = 0, tmp, level, levels = 0, num_frames[RC_MAX_TEMPORAL_LEVELS];
      float numer, denom;
      int gop = p_Inp->NumberBFrames + 1;
      int i_period = p_Inp->intra_period == 0 ? p_Inp->idr_period : (p_Inp->idr_period == 0 ? p_Inp->intra_period : imin(p_Inp->intra_period, p_Inp->idr_period) );

      memset( num_frames, 0, RC_MAX_TEMPORAL_LEVELS * sizeof(int) );
      // are there any B frames?
      if ( p_Inp->NumberBFrames )
      {
        if ( p_Inp->HierarchicalCoding == 1 ) // two layers: even/odd
        {
          levels = 2;
          num_frames[0] = p_Inp->NumberBFrames >> 1;
          num_frames[1] = (p_Inp->NumberBFrames - num_frames[0]) >= 0 ? (p_Inp->NumberBFrames - num_frames[0]) : 0;
        }
        else if ( p_Inp->HierarchicalCoding == 2 ) // binary hierarchical structure
        {
          // check if gop is power of two
          tmp = gop;
          while ( tmp )
          {
            sum += tmp & 1;
            tmp >>= 1;
          }
          assert( sum == 1 );

          // determine number of levels
          levels = 0;
          tmp = gop;
          while ( tmp > 1 )
          {
            tmp >>= 1; // divide by 2          
            num_frames[levels] = 1 << levels;
            levels++;          
          }
          assert( levels >= 1 && levels <= RC_MAX_TEMPORAL_LEVELS );        
        }
        else if ( p_Inp->HierarchicalCoding == 3 )
        {
          fprintf(stderr, "\n RCUpdateMode=3 and HierarchicalCoding == 3 are currently not supported"); // This error message should be moved elsewhere and have proper memory deallocation
          exit(1);
        }
        else // all frames of the same priority - level
        {
          levels = 1;
          num_frames[0] = p_Inp->NumberBFrames;
        }
        p_gen->temporal_levels = levels;      
      }
      else
      {
        for ( level = 0; level < RC_MAX_TEMPORAL_LEVELS; level++ )
        {
          p_Inp->RCBSliceBitRatio[level] = 0.0F;
        }
        p_gen->temporal_levels = 0;
      }
      // calculate allocated bits for each type of frame
      numer = (float)(( (!i_period ? 1 : i_period) * gop) * ((double)p_Inp->bit_rate / p_Inp->source.frame_rate));
      denom = 0.0F;

      for ( level = 0; level < levels; level++ )
      {
        denom += (float)(num_frames[level] * p_Inp->RCBSliceBitRatio[level]);
        p_gen->hierNb[level] = num_frames[level] * np;
      }
      denom += 1.0F;
      if ( i_period >= 1 )
      {
        denom *= (float)i_period;
        denom += (float)p_Inp->RCISliceBitRatio - 1.0F;
      }

      // set bit targets for each type of frame
      p_gen->RCPSliceBits = (int) floor( numer / denom + 0.5F );
      p_gen->RCISliceBits = i_period ? (int)(p_Inp->RCISliceBitRatio * p_gen->RCPSliceBits + 0.5) : 0;

      for ( level = 0; level < levels; level++ )
      {
        p_gen->RCBSliceBits[level] = (int)floor(p_Inp->RCBSliceBitRatio[level] * p_gen->RCPSliceBits + 0.5);
      }

      p_gen->NISlice = i_period ? ( p_Inp->no_frames / i_period ) : 0;
      p_gen->NPSlice = (p_Inp->no_frames / (1 + p_Inp->NumberBFrames)) - p_gen->NISlice; // approximate but good enough for sufficient number of frames
    }
    break;
  default:
    break;
  }

  /* check if the last GOP over uses its budget. If yes, the initial QP of the I frame in
  the coming  GOP will be increased.*/

  //OverBits=-(int)(p_gen->RemainingBits);

  /*initialize the lower bound and the upper bound for the target bits of each frame, HRD consideration*/
  p_quad->LowerBound  = (int)(p_gen->RemainingBits + p_quad->bit_rate / p_quad->frame_rate);
  p_quad->UpperBound1 = (int)(p_gen->RemainingBits + (p_quad->bit_rate * 2.048));

  /*compute the total number of bits for the current GOP*/
  AllocatedBits = (int64) floor((1 + np + nb) * p_quad->bit_rate / p_quad->frame_rate + 0.5);
  p_gen->RemainingBits += AllocatedBits;
  p_quad->Np = np;
  p_quad->Nb = nb;

  p_quad->GOPOverdue=FALSE;

  /*field coding*/
  //p_gen->NoGranularFieldRC = ( p_Inp->PicInterlace || !p_Inp->MbInterlace || p_Inp->basicunit != p_Vid->FrameSizeInMbs );
  if ( !p_Inp->PicInterlace && p_Inp->MbInterlace && p_Inp->basicunit == p_Vid->FrameSizeInMbs )
    p_gen->NoGranularFieldRC = 0;
  else
    p_gen->NoGranularFieldRC = 1;

  /*Compute InitialQp for each GOP*/
  p_quad->TotalPFrame=np;
  p_gen->NumberofGOP++;
  if(p_gen->NumberofGOP==1)
  {
    p_quad->MyInitialQp = p_Inp->SeinitialQP + p_quad->bitdepth_qp_scale;
    p_quad->CurrLastQP = p_quad->MyInitialQp - 1; //recent change -0;
    p_quad->QPLastGOP   = p_quad->MyInitialQp;

    p_quad->PAveFrameQP   = p_quad->MyInitialQp;
    p_quad->m_Qc          = p_quad->PAveFrameQP;
    p_quad->FieldQPBuffer = p_quad->PAveFrameQP;
    p_quad->FrameQPBuffer = p_quad->PAveFrameQP;
    p_quad->PAverageQp    = p_quad->PAveFrameQP;
  }
  else
  {
    /*adaptive field/frame coding*/
    if( p_Inp->PicInterlace == ADAPTIVE_CODING || p_Inp->MbInterlace )
    {
      if (p_gen->FieldFrame == 1)
      {
        p_quad->TotalQpforPPicture += p_quad->FrameQPBuffer;
        p_quad->QPLastPFrame = p_quad->FrameQPBuffer;
      }
      else
      {
        p_quad->TotalQpforPPicture += p_quad->FieldQPBuffer;
        p_quad->QPLastPFrame = p_quad->FieldQPBuffer;
      }
    }
    /*compute the average QP of P frames in the previous GOP*/
    p_quad->PAverageQp=(int)(1.0 * p_quad->TotalQpforPPicture / p_quad->NumberofPPicture+0.5);

    GOPDquant=(int)((1.0*(np+nb+1)/15.0) + 0.5);
    if(GOPDquant>2)
      GOPDquant=2;

    p_quad->PAverageQp -= GOPDquant;

    if (p_quad->PAverageQp > (p_quad->QPLastPFrame - 2))
      p_quad->PAverageQp--;

    // QP is constrained by QP of previous GOP
    p_quad->PAverageQp = iClip3(p_quad->QPLastGOP - 2, p_quad->QPLastGOP + 2, p_quad->PAverageQp);
    // Also clipped within range.
    p_quad->PAverageQp = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale,  p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale,  p_quad->PAverageQp);

    p_quad->MyInitialQp = p_quad->PAverageQp;
    p_quad->Pm_Qp       = p_quad->PAverageQp;
    p_quad->PAveFrameQP = p_quad->PAverageQp;
    p_quad->QPLastGOP   = p_quad->MyInitialQp;
    p_quad->PrevLastQP = p_quad->CurrLastQP;
    p_quad->CurrLastQP = p_quad->MyInitialQp - 1;
  }

  p_quad->TotalQpforPPicture=0;
  p_quad->NumberofPPicture=0;
  p_quad->NumberofBFrames=0;
}


/*!
 *************************************************************************************
 * \brief
 *    Initialize one picture
 *
 *************************************************************************************
*/
void rc_init_pict(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int fieldpic,int topfield,int targetcomputation, float mult)
{
  int tmp_T;

  /* compute the total number of basic units in a frame */
  if(p_Inp->MbInterlace)
    p_quad->TotalNumberofBasicUnit = p_Vid->FrameSizeInMbs / p_Vid->BasicUnit;
  else
    p_quad->TotalNumberofBasicUnit = p_Vid->FrameSizeInMbs / p_Inp->basicunit;

  p_Vid->NumberofCodedMacroBlocks = 0;

  /* Normally, the bandwidth for the VBR case is estimated by
     a congestion control algorithm. A bandwidth curve can be predefined if we only want to
     test the proposed algorithm */
  if(p_Inp->channel_type==1)
  {
    if(p_quad->NumberofCodedPFrame==58)
      p_quad->bit_rate *= 1.5;
    else if(p_quad->NumberofCodedPFrame==59)
      p_quad->PrevBitRate = p_quad->bit_rate;
  }

  /* predefine a target buffer level for each frame */
  if((fieldpic||topfield) && targetcomputation)
  {
    if ( p_Vid->type == P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && (p_Vid->number !=0)) )
    {
      /* Since the available bandwidth may vary at any time, the total number of
      bits is updated picture by picture*/
      if(p_quad->PrevBitRate!=p_quad->bit_rate)
        p_gen->RemainingBits +=(int) floor((p_quad->bit_rate-p_quad->PrevBitRate)*(p_quad->Np + p_quad->Nb)/p_quad->frame_rate+0.5);

      /* predefine the  target buffer level for each picture.
      frame layer rate control */
      if(p_Vid->BasicUnit == p_Vid->FrameSizeInMbs)
      {
        if(p_quad->NumberofPPicture==1)
        {
          p_quad->TargetBufferLevel = (double) p_gen->CurrentBufferFullness;
          p_quad->DeltaP = (p_gen->CurrentBufferFullness - p_quad->GOPTargetBufferLevel) / (p_quad->TotalPFrame-1);
          p_quad->TargetBufferLevel -= p_quad->DeltaP;
        }
        else if(p_quad->NumberofPPicture>1)
          p_quad->TargetBufferLevel -= p_quad->DeltaP;
      }
      /* basic unit layer rate control */
      else
      {
        if(p_quad->NumberofCodedPFrame>0)
        {
          /* adaptive frame/field coding */
          if(((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))&&(p_gen->FieldControl==1))
            memcpy((void *)p_quad->FCBUPFMAD,(void *)p_quad->FCBUCFMAD, p_quad->TotalNumberofBasicUnit * sizeof(double));
          else
            memcpy((void *)p_quad->BUPFMAD,(void *)p_quad->BUCFMAD, p_quad->TotalNumberofBasicUnit * sizeof(double));
        }

        if(p_gen->NumberofGOP==1)
        {
          if(p_quad->NumberofPPicture==1)
          {
            p_quad->TargetBufferLevel = (double) p_gen->CurrentBufferFullness;
            p_quad->DeltaP = (p_gen->CurrentBufferFullness - p_quad->GOPTargetBufferLevel)/(p_quad->TotalPFrame - 1);
            p_quad->TargetBufferLevel -= p_quad->DeltaP;
          }
          else if(p_quad->NumberofPPicture>1)
            p_quad->TargetBufferLevel -= p_quad->DeltaP;
        }
        else if(p_gen->NumberofGOP>1)
        {
          if(p_quad->NumberofPPicture==0)
          {
            p_quad->TargetBufferLevel = (double) p_gen->CurrentBufferFullness;
            p_quad->DeltaP = (p_gen->CurrentBufferFullness - p_quad->GOPTargetBufferLevel) / p_quad->TotalPFrame;
            p_quad->TargetBufferLevel -= p_quad->DeltaP;
          }
          else if(p_quad->NumberofPPicture>0)
            p_quad->TargetBufferLevel -= p_quad->DeltaP;
        }
      }

      if(p_quad->NumberofCodedPFrame==1)
        p_quad->AveWp = p_quad->Wp;

      if((p_quad->NumberofCodedPFrame<8)&&(p_quad->NumberofCodedPFrame>1))
        p_quad->AveWp = (p_quad->AveWp + p_quad->Wp * (p_quad->NumberofCodedPFrame-1))/p_quad->NumberofCodedPFrame;
      else if(p_quad->NumberofCodedPFrame>1)
        p_quad->AveWp = (p_quad->Wp + 7 * p_quad->AveWp) / 8;

      // compute the average complexity of B frames
      if(p_Inp->NumberBFrames>0)
      {
        // compute the target buffer level
        p_quad->TargetBufferLevel += (p_quad->AveWp * (p_Inp->NumberBFrames + 1)*p_quad->bit_rate\
          /(p_quad->frame_rate*(p_quad->AveWp+p_quad->AveWb*p_Inp->NumberBFrames))-p_quad->bit_rate/p_quad->frame_rate);
      }
    }
    else if ( p_Vid->type == B_SLICE )
    {
      /* update the total number of bits if the bandwidth is changed*/
      if(p_quad->PrevBitRate != p_quad->bit_rate)
        p_gen->RemainingBits +=(int) floor((p_quad->bit_rate-p_quad->PrevBitRate) * (p_quad->Np + p_quad->Nb) / p_quad->frame_rate+0.5);
      if(p_gen->NumberofCodedBFrame == 1)
      {
        if(p_quad->NumberofCodedPFrame == 1)
        {
          p_quad->AveWp = p_quad->Wp;
        }
        p_quad->AveWb = p_quad->Wb;
      }
      else if(p_gen->NumberofCodedBFrame > 1)
      {
        //compute the average weight
        if(p_gen->NumberofCodedBFrame<8)
          p_quad->AveWb = (p_quad->AveWb + p_quad->Wb*(p_gen->NumberofCodedBFrame-1)) / p_gen->NumberofCodedBFrame;
        else
          p_quad->AveWb = (p_quad->Wb + 7 * p_quad->AveWb) / 8;
      }
    }
    /* Compute the target bit for each frame */
    if( p_Vid->type == P_SLICE || ( (p_Vid->number != 0) && (p_Inp->RCUpdateMode == RC_MODE_1 || p_Inp->RCUpdateMode == RC_MODE_3 ) ) )
    {
      /* frame layer rate control */
      if(p_Vid->BasicUnit == p_Vid->FrameSizeInMbs || (p_Inp->RCUpdateMode == RC_MODE_3) )
      {
        if(p_quad->NumberofCodedPFrame>0)
        {
          if (p_Inp->RCUpdateMode == RC_MODE_3)
          {
            int level_idx = (p_Vid->type == B_SLICE && p_Inp->HierarchicalCoding) ? (p_Vid->p_curr_frm_struct->layer - 1) : 0;
            int bitrate = (p_Vid->type == B_SLICE) ? p_gen->RCBSliceBits[ level_idx ]
            : ( p_Vid->type == P_SLICE ? p_gen->RCPSliceBits : p_gen->RCISliceBits );
            int level, denom = p_gen->NISlice * p_gen->RCISliceBits + p_gen->NPSlice * p_gen->RCPSliceBits;

            if ( p_Inp->HierarchicalCoding )
            {
              for ( level = 0; level < p_gen->temporal_levels; level++ )
                denom += p_gen->hierNb[ level ] * p_gen->RCBSliceBits[ level ];
            }
            else
            {
              denom += p_gen->hierNb[0] * p_gen->RCBSliceBits[0];
            }
            // target due to remaining bits
            p_quad->Target = (int) floor( (float)(1.0 * bitrate * p_gen->RemainingBits) / (float)denom + 0.5F );
            // target given original taget rate and buffer considerations
            tmp_T  = imax(0, (int) floor( (double)bitrate - p_quad->GAMMAP * (p_gen->CurrentBufferFullness-p_quad->TargetBufferLevel) + 0.5) );
            // translate Target rate from B or I "domain" to P domain since the P RC model is going to be used to select the QP
            // for hierarchical coding adjust the target QP to account for different temporal levels
            switch( p_Vid->type )
            {
            case B_SLICE:
              p_quad->Target = (int) floor( (float)p_quad->Target / p_Inp->RCBoverPRatio + 0.5F);
              break;
            case I_SLICE:
              p_quad->Target = (int) floor( (float)p_quad->Target / (p_Inp->RCIoverPRatio * 4.0) + 0.5F); // 4x accounts for the fact that header bits reduce the percentage of texture
              break;
            case P_SLICE:
            default:
              break;
            }
          }
          else
          {
            p_quad->Target = (int) floor( p_quad->Wp * p_gen->RemainingBits / (p_quad->Np * p_quad->Wp + p_quad->Nb * p_quad->Wb) + 0.5);
            tmp_T  = imax(0, (int) floor(p_quad->bit_rate / p_quad->frame_rate - p_quad->GAMMAP * (p_gen->CurrentBufferFullness-p_quad->TargetBufferLevel) + 0.5));
            p_quad->Target = (int) floor(p_quad->BETAP * (p_quad->Target - tmp_T) + tmp_T + 0.5);
          }
        }
      }
      /* basic unit layer rate control */
      else
      {
        if(((p_gen->NumberofGOP == 1)&&(p_quad->NumberofCodedPFrame>0))
          || (p_gen->NumberofGOP > 1))
        {
          p_quad->Target = (int) (floor( p_quad->Wp * p_gen->RemainingBits / (p_quad->Np * p_quad->Wp + p_quad->Nb * p_quad->Wb) + 0.5));
          tmp_T  = imax(0, (int) (floor(p_quad->bit_rate / p_quad->frame_rate - p_quad->GAMMAP * (p_gen->CurrentBufferFullness-p_quad->TargetBufferLevel) + 0.5)));
          p_quad->Target = (int) (floor(p_quad->BETAP * (p_quad->Target - tmp_T) + tmp_T + 0.5));
        }
      }
      p_quad->Target = (int)(mult * p_quad->Target);

      /* HRD consideration */
      if ( p_Inp->RCUpdateMode != RC_MODE_3 || p_Vid->type == P_SLICE )
        p_quad->Target = iClip3(p_quad->LowerBound, p_quad->UpperBound2, p_quad->Target);
      if((topfield) || (fieldpic && ((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))))
        p_quad->TargetField=p_quad->Target;
    }
  }

  if(fieldpic || topfield)
  {
    /* frame layer rate control */
    p_gen->NumberofHeaderBits  = 0;
    p_gen->NumberofTextureBits = 0;

    /* basic unit layer rate control */
    if(p_Vid->BasicUnit < p_Vid->FrameSizeInMbs)
    {
      p_quad->TotalFrameQP = 0;
      p_gen->NumberofBasicUnitHeaderBits  = 0;
      p_gen->NumberofBasicUnitTextureBits = 0;
      p_gen->TotalMADBasicUnit = 0;
      if(p_gen->FieldControl==0)
        p_quad->NumberofBasicUnit = p_quad->TotalNumberofBasicUnit;
      else
        p_quad->NumberofBasicUnit = p_quad->TotalNumberofBasicUnit >> 1;
    }
  }

  if( ( p_Vid->type==P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && (p_Vid->number != 0)) ) && p_Vid->BasicUnit < p_Vid->FrameSizeInMbs && p_gen->FieldControl == 1 )
  {
    /* top field at basic unit layer rate control */
    if(topfield)
    {
      p_quad->bits_topfield=0;
      p_quad->Target=(int)(p_quad->TargetField*0.6);
    }
    /* bottom field at basic unit layer rate control */
    else
    {
      p_quad->Target=p_quad->TargetField-p_quad->bits_topfield;
      p_gen->NumberofBasicUnitHeaderBits=0;
      p_gen->NumberofBasicUnitTextureBits=0;
      p_gen->TotalMADBasicUnit=0;
      p_quad->NumberofBasicUnit=p_quad->TotalNumberofBasicUnit >> 1;
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    update one picture after frame/field encoding
 *
 * \param p_Vid
 *    Image encoding parameters for picture
 * \param p_Inp
 *    Input configuration parameters
 * \param p_quad
 *    quadratic model
 * \param p_gen
 *    generic rate control parameters 
 * \param nbits
 *    number of bits used for picture
 *
 *************************************************************************************
*/
void rc_update_pict(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int nbits)
{
  int delta_bits = (nbits - (int)floor(p_quad->bit_rate / p_quad->frame_rate + 0.5F) );
  // remaining # of bits in GOP
  p_gen->RemainingBits -= nbits; 
  p_gen->CurrentBufferFullness += delta_bits;

  // update the lower bound and the upper bound for the target bits of each frame, HRD consideration
  p_quad->LowerBound  -= (int) delta_bits;
  p_quad->UpperBound1 -= (int) delta_bits;
  p_quad->UpperBound2  = (int)(OMEGA * p_quad->UpperBound1);

  // update the parameters of quadratic R-D model
  if( p_Vid->type==P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && p_Vid->curr_frm_idx) )
  {
    updateRCModel(p_Vid, p_Inp, p_quad, p_gen);
    if ( p_Inp->RCUpdateMode == RC_MODE_3 )
      p_quad->PreviousWholeFrameMAD = ComputeFrameMAD(p_Vid);
  }
}

/*!
 *************************************************************************************
 * \brief
 *    update one picture after coding
 *
 * \param p_Vid
 *    video parameters for current picture
 * \param p_Inp
 *    input parameters from configuration
 * \param bits
 *    number of bits used for picture
 *
 *************************************************************************************
*/

void rc_update_picture( VideoParameters *p_Vid, InputParameters *p_Inp, int bits )
{
  rc_update_pict(p_Vid, p_Inp, p_Vid->p_rc_quad, p_Vid->p_rc_gen, bits);
}

int updateComplexity( VideoParameters *p_Vid, RCQuadratic *p_quad, RCGeneric *p_gen, Boolean is_updated, int nbits )
{
  double Avem_Qc;

  /* frame layer rate control */
  if(p_Vid->BasicUnit == p_Vid->FrameSizeInMbs)
    return ((int) floor(nbits * p_quad->m_Qc + 0.5));
  /* basic unit layer rate control */
  else
  {
    if( is_updated )
    {
      if( p_gen->NoGranularFieldRC == 0 || p_gen->FieldControl == 0 )
      {
        Avem_Qc = (double)p_quad->TotalFrameQP / (double)p_quad->TotalNumberofBasicUnit;
        return ((int)floor(nbits * Avem_Qc + 0.5));
      }
    }
    else if( p_Vid->type == B_SLICE )
      return ((int) floor(nbits * p_quad->m_Qc + 0.5));
  }
  return 0;
}

void updatePparams( RCQuadratic *p_quad, RCGeneric *p_gen, int complexity )
{
  p_quad->Xp = complexity;
  p_quad->Np--;
  p_quad->Wp = p_quad->Xp;
  p_quad->Pm_Hp = p_gen->NumberofHeaderBits;
  p_quad->NumberofCodedPFrame++;
  p_quad->NumberofPPicture++;
}

void updateBparams( RCQuadratic *p_quad, RCGeneric *p_gen, int complexity )
{
  p_quad->Xb = complexity;
  p_quad->Nb--;
  p_quad->Wb = p_quad->Xb / THETA;     
  p_quad->NumberofBFrames++;
  p_gen->NumberofCodedBFrame++;
}

/*! 
 *************************************************************************************
 * \brief
 *    update after frame encoding
 * \param p_Vid
 *    Image encoding parameters for picture
 * \param p_Inp
 *    Input configuration parameters
 * \param p_quad
 *    quadratic model
 * \param p_gen
 *    generic rate control parameters 
 * \param nbits
 *    number of bits used for frame
 *
 *************************************************************************************
*/
void rc_update_pict_frame(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int nbits)
{
  /* update the complexity weight of I, P, B frame */  
  int complexity = 0;

  switch( p_Inp->RCUpdateMode )
  {
  case RC_MODE_0:
  case RC_MODE_2:
  default:
    complexity = updateComplexity( p_Vid, p_quad, p_gen, (Boolean) (p_Vid->type == P_SLICE), nbits );
    if ( p_Vid->type == P_SLICE )
    {
      if( p_gen->NoGranularFieldRC == 0 || p_gen->FieldControl == 0 )
        updatePparams( p_quad, p_gen, complexity );
      else
        p_gen->NoGranularFieldRC = 0;
    }
    else if ( p_Vid->type == B_SLICE )
      updateBparams( p_quad, p_gen, complexity );
    break;
  case RC_MODE_1:
    complexity = updateComplexity( p_Vid, p_quad, p_gen, (Boolean) (p_Vid->number != 0), nbits );
    if ( p_Vid->number != 0 )
    {
      if( p_gen->NoGranularFieldRC == 0 || p_gen->FieldControl == 0 )
        updatePparams( p_quad, p_gen, complexity );
      else
        p_gen->NoGranularFieldRC = 0;
    }
    break;
  case RC_MODE_3:
    complexity = updateComplexity( p_Vid, p_quad, p_gen, (Boolean) (p_Vid->type == P_SLICE), nbits );
    if (p_Vid->type == I_SLICE && (p_Vid->number != 0))
      p_gen->NISlice--;

    if ( p_Vid->type == P_SLICE )
    {
      if( p_gen->NoGranularFieldRC == 0 || p_gen->FieldControl == 0 )
      {
        updatePparams( p_quad, p_gen, complexity );
        p_gen->NPSlice--;
      }
      else
        p_gen->NoGranularFieldRC = 0;
    }
    else if ( p_Vid->type == B_SLICE )
    {
      updateBparams( p_quad, p_gen, complexity );
      p_gen->hierNb[ p_Inp->HierarchicalCoding ? (p_Vid->p_curr_frm_struct->layer - 1) : 0 ]--;
    }
    break;
  }   
}


/*!
 *************************************************************************************
 * \brief
 *    update the parameters of quadratic R-D model
 *
 *************************************************************************************
*/
void updateRCModel (VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen)
{
  int n_windowSize;
  int i;
  double std = 0.0, threshold;
  int m_Nc = p_quad->NumberofCodedPFrame;
  Boolean MADModelFlag = FALSE;
  Boolean m_rgRejected[RC_MODEL_HISTORY];
  double  error       [RC_MODEL_HISTORY];

  if( p_Vid->type == P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && (p_Vid->number != 0)) )
  {
    /*frame layer rate control*/
    if(p_Vid->BasicUnit == p_Vid->FrameSizeInMbs)
    {
      p_quad->CurrentFrameMAD = ComputeFrameMAD(p_Vid);
      m_Nc=p_quad->NumberofCodedPFrame;
    }
    /*basic unit layer rate control*/
    else
    {
      /*compute the MAD of the current basic unit*/
      p_quad->CurrentFrameMAD = (double) ((p_gen->TotalMADBasicUnit >> 8)/p_Vid->BasicUnit);
      p_gen->TotalMADBasicUnit=0;

      /* compute the average number of header bits*/
      p_quad->CodedBasicUnit=p_quad->TotalNumberofBasicUnit-p_quad->NumberofBasicUnit;
      if(p_quad->CodedBasicUnit > 0)
      {
        p_quad->PAveHeaderBits1=(int)((double)(p_quad->PAveHeaderBits1*(p_quad->CodedBasicUnit-1)+
          p_gen->NumberofBasicUnitHeaderBits)/p_quad->CodedBasicUnit+0.5);
        if(p_quad->PAveHeaderBits3 == 0)
          p_quad->PAveHeaderBits2 = p_quad->PAveHeaderBits1;
        else
        {
          p_quad->PAveHeaderBits2 = (int)((double)(p_quad->PAveHeaderBits1 * p_quad->CodedBasicUnit+
            p_quad->PAveHeaderBits3 * p_quad->NumberofBasicUnit)/p_quad->TotalNumberofBasicUnit+0.5);
        }
      }
      
      if ((p_quad->NumberofBasicUnit >= p_quad->TotalNumberofBasicUnit) || (p_quad->NumberofBasicUnit<0))
      {
        fprintf(stderr, "Write into invalid memory in updateRCModel at frame %d, p_quad->NumberofBasicUnit %d\n", 
          p_Vid->framepoc, p_quad->NumberofBasicUnit);
      }

      /*update the record of MADs for reference*/
      if(((p_Inp->PicInterlace == ADAPTIVE_CODING) || (p_Inp->MbInterlace)) && (p_gen->FieldControl == 1))
        p_quad->FCBUCFMAD[p_quad->TotalNumberofBasicUnit-1-p_quad->NumberofBasicUnit]=p_quad->CurrentFrameMAD;
      else
        p_quad->BUCFMAD[p_quad->TotalNumberofBasicUnit-1-p_quad->NumberofBasicUnit]=p_quad->CurrentFrameMAD;

      if(p_quad->NumberofBasicUnit != 0)
        m_Nc = p_quad->NumberofCodedPFrame * p_quad->TotalNumberofBasicUnit + p_quad->CodedBasicUnit;
      else
        m_Nc = (p_quad->NumberofCodedPFrame-1) * p_quad->TotalNumberofBasicUnit + p_quad->CodedBasicUnit;
    }

    if(m_Nc > 1)
      MADModelFlag=TRUE;

    p_quad->PPreHeader = p_gen->NumberofHeaderBits;
    for (i = (RC_MODEL_HISTORY-2); i > 0; i--)
    {// update the history
      p_quad->Pm_rgQp[i] = p_quad->Pm_rgQp[i - 1];
      p_quad->m_rgQp[i]  = p_quad->Pm_rgQp[i];
      p_quad->Pm_rgRp[i] = p_quad->Pm_rgRp[i - 1];
      p_quad->m_rgRp[i]  = p_quad->Pm_rgRp[i];
    }
    p_quad->Pm_rgQp[0] = QP2Qstep(p_quad->m_Qc); //*1.0/p_quad->CurrentFrameMAD;
    /*frame layer rate control*/
    if(p_Vid->BasicUnit == p_Vid->FrameSizeInMbs)
      p_quad->Pm_rgRp[0] = p_gen->NumberofTextureBits*1.0/p_quad->CurrentFrameMAD;
    /*basic unit layer rate control*/
    else
      p_quad->Pm_rgRp[0] = p_gen->NumberofBasicUnitTextureBits*1.0/p_quad->CurrentFrameMAD;

    p_quad->m_rgQp[0] = p_quad->Pm_rgQp[0];
    p_quad->m_rgRp[0] = p_quad->Pm_rgRp[0];
    p_quad->m_X1 = p_quad->Pm_X1;
    p_quad->m_X2 = p_quad->Pm_X2;

    /*compute the size of window*/
    n_windowSize = (p_quad->CurrentFrameMAD>p_quad->PreviousFrameMAD)
      ? (int)(p_quad->PreviousFrameMAD/p_quad->CurrentFrameMAD * (RC_MODEL_HISTORY-1) )
      : (int)(p_quad->CurrentFrameMAD/p_quad->PreviousFrameMAD *(RC_MODEL_HISTORY-1));
    n_windowSize=iClip3(1, m_Nc, n_windowSize);
    n_windowSize=imin(n_windowSize,p_quad->m_windowSize+1);
    n_windowSize=imin(n_windowSize,(RC_MODEL_HISTORY-1));

    /*update the previous window size*/
    p_quad->m_windowSize=n_windowSize;

    for (i = 0; i < (RC_MODEL_HISTORY-1); i++)
    {
      m_rgRejected[i] = FALSE;
    }

    // initial RD model estimator
    RCModelEstimator (p_Vid, p_Inp, p_quad, n_windowSize, m_rgRejected);

    n_windowSize = p_quad->m_windowSize;
    // remove outlier

    for (i = 0; i < (int) n_windowSize; i++)
    {
      error[i] = p_quad->m_X1 / p_quad->m_rgQp[i] + p_quad->m_X2 / (p_quad->m_rgQp[i] * p_quad->m_rgQp[i]) - p_quad->m_rgRp[i];
      std += error[i] * error[i];
    }
    threshold = (n_windowSize == 2) ? 0 : sqrt (std / n_windowSize);
    for (i = 0; i < (int) n_windowSize; i++)
    {
      if (fabs(error[i]) > threshold)
        m_rgRejected[i] = TRUE;
    }
    // always include the last data point
    m_rgRejected[0] = FALSE;

    // second RD model estimator
    RCModelEstimator (p_Vid, p_Inp, p_quad, n_windowSize, m_rgRejected);

    if( MADModelFlag )
      updateMADModel(p_Vid, p_Inp, p_quad, p_gen);
    else if( p_Vid->type == P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && (p_Vid->number != 0)) )
      p_quad->PPictureMAD[0] = p_quad->CurrentFrameMAD;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Model Estimator
 *
 *************************************************************************************
*/
void RCModelEstimator (VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, int n_windowSize, Boolean *m_rgRejected)
{
  int n_realSize = n_windowSize;
  int i;
  double oneSampleQ = 0;
  double a00 = 0.0, a01 = 0.0, a10 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
  double MatrixValue;
  Boolean estimateX2 = FALSE;

  for (i = 0; i < n_windowSize; i++)
  {// find the number of samples which are not rejected
    if (m_rgRejected[i])
      n_realSize--;
  }

  // default RD model estimation results
  p_quad->m_X1 = p_quad->m_X2 = 0.0;

  for (i = 0; i < n_windowSize; i++)
  {
    if (!m_rgRejected[i])
      oneSampleQ = p_quad->m_rgQp[i];
  }
  for (i = 0; i < n_windowSize; i++)
  {// if all non-rejected Q are the same, take 1st order model
    if ((p_quad->m_rgQp[i] != oneSampleQ) && !m_rgRejected[i])
      estimateX2 = TRUE;
    if (!m_rgRejected[i])
      p_quad->m_X1 += (p_quad->m_rgQp[i] * p_quad->m_rgRp[i]) / n_realSize;
  }

  // take 2nd order model to estimate X1 and X2
  if ((n_realSize >= 1) && estimateX2)
  {
    for (i = 0; i < n_windowSize; i++)
    {
      if (!m_rgRejected[i])
      {
        a00  = a00 + 1.0;
        a01 += 1.0 / p_quad->m_rgQp[i];
        a10  = a01;
        a11 += 1.0 / (p_quad->m_rgQp[i] * p_quad->m_rgQp[i]);
        b0  += p_quad->m_rgQp[i] * p_quad->m_rgRp[i];
        b1  += p_quad->m_rgRp[i];
      }
    }
    // solve the equation of AX = B
    MatrixValue=a00*a11-a01*a10;
    if(fabs(MatrixValue) > 0.000001)
    {
      p_quad->m_X1 = (b0 * a11 - b1 * a01) / MatrixValue;
      p_quad->m_X2 = (b1 * a00 - b0 * a10) / MatrixValue;
    }
    else
    {
      p_quad->m_X1 = b0 / a00;
      p_quad->m_X2 = 0.0;
    }
  }
  if( p_Vid->type == P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && (p_Vid->number != 0)) )
  {
    p_quad->Pm_X1 = p_quad->m_X1;
    p_quad->Pm_X2 = p_quad->m_X2;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    update the parameters of linear prediction model
 *
 *************************************************************************************
*/
void updateMADModel (VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen)
{
  int    n_windowSize;
  int    i;
  double std = 0.0, threshold;
  int    m_Nc = p_quad->NumberofCodedPFrame;
  Boolean PictureRejected[RC_MODEL_HISTORY];
  double  error          [RC_MODEL_HISTORY];

  if(p_quad->NumberofCodedPFrame>0)
  {
    //assert (p_Vid->type!=P_SLICE);
    /*frame layer rate control*/
    if(p_Vid->BasicUnit == p_Vid->FrameSizeInMbs)
      m_Nc = p_quad->NumberofCodedPFrame;
    else // basic unit layer rate control
      m_Nc=p_quad->NumberofCodedPFrame*p_quad->TotalNumberofBasicUnit+p_quad->CodedBasicUnit;

    for (i = (RC_MODEL_HISTORY-2); i > 0; i--)
    {// update the history
      p_quad->PPictureMAD[i]  = p_quad->PPictureMAD[i - 1];
      p_quad->PictureMAD[i]   = p_quad->PPictureMAD[i];
      p_quad->ReferenceMAD[i] = p_quad->ReferenceMAD[i-1];
    }
    p_quad->PPictureMAD[0] = p_quad->CurrentFrameMAD;
    p_quad->PictureMAD[0]  = p_quad->PPictureMAD[0];

    if(p_Vid->BasicUnit == p_Vid->FrameSizeInMbs)
      p_quad->ReferenceMAD[0]=p_quad->PictureMAD[1];
    else
    {
      if(((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace)) &&(p_gen->FieldControl==1))
        p_quad->ReferenceMAD[0]=p_quad->FCBUPFMAD[p_quad->TotalNumberofBasicUnit-1-p_quad->NumberofBasicUnit];
      else
        p_quad->ReferenceMAD[0]=p_quad->BUPFMAD[p_quad->TotalNumberofBasicUnit-1-p_quad->NumberofBasicUnit];
    }
    p_quad->MADPictureC1 = p_quad->PMADPictureC1;
    p_quad->MADPictureC2 = p_quad->PMADPictureC2;

    /*compute the size of window*/
    n_windowSize = (p_quad->CurrentFrameMAD > p_quad->PreviousFrameMAD)
      ? (int) ((float)(RC_MODEL_HISTORY-1) * p_quad->PreviousFrameMAD / p_quad->CurrentFrameMAD)
      : (int) ((float)(RC_MODEL_HISTORY-1) * p_quad->CurrentFrameMAD / p_quad->PreviousFrameMAD);
    n_windowSize = iClip3(1, (m_Nc-1), n_windowSize);
    n_windowSize=imin(n_windowSize, imin(20, p_quad->MADm_windowSize + 1));

    /*update the previous window size*/
    p_quad->MADm_windowSize=n_windowSize;

    for (i = 0; i < (RC_MODEL_HISTORY-1); i++)
    {
      PictureRejected[i] = FALSE;
    }

    //update the MAD for the previous frame
    if( p_Vid->type == P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && (p_Vid->number != 0)) )
      p_quad->PreviousFrameMAD=p_quad->CurrentFrameMAD;

    // initial MAD model estimator
    MADModelEstimator (p_Vid, p_Inp, p_quad, n_windowSize, PictureRejected);

    // remove outlier
    for (i = 0; i < n_windowSize; i++)
    {
      error[i] = p_quad->MADPictureC1 * p_quad->ReferenceMAD[i] + p_quad->MADPictureC2 - p_quad->PictureMAD[i];
      std += (error[i] * error[i]);
    }

    threshold = (n_windowSize == 2) ? 0 : sqrt (std / n_windowSize);
    for (i = 0; i < n_windowSize; i++)
    {
      if (fabs(error[i]) > threshold)
        PictureRejected[i] = TRUE;
    }
    // always include the last data point
    PictureRejected[0] = FALSE;

    // second MAD model estimator
    MADModelEstimator (p_Vid, p_Inp, p_quad, n_windowSize, PictureRejected);
  }
}


/*!
 *************************************************************************************
 * \brief
 *    MAD mode estimator
 *
 *************************************************************************************
*/
void MADModelEstimator (VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, int n_windowSize, Boolean *PictureRejected)
{
  int    n_realSize = n_windowSize;
  int    i;
  double oneSampleQ = 0.0;
  double a00 = 0.0, a01 = 0.0, a10 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
  double MatrixValue;
  Boolean estimateX2 = FALSE;

  for (i = 0; i < n_windowSize; i++)
  {// find the number of samples which are not rejected
    if (PictureRejected[i])
      n_realSize--;
  }

  // default MAD model estimation results
  p_quad->MADPictureC1 = p_quad->MADPictureC2 = 0.0;

  for (i = 0; i < n_windowSize; i++)
  {
    if (!PictureRejected[i])
      oneSampleQ = p_quad->PictureMAD[i];
  }

  for (i = 0; i < n_windowSize; i++)
  {// if all non-rejected MAD are the same, take 1st order model
    if ((p_quad->PictureMAD[i] != oneSampleQ) && !PictureRejected[i])
      estimateX2 = TRUE;
    if (!PictureRejected[i])
      p_quad->MADPictureC1 += p_quad->PictureMAD[i] / (p_quad->ReferenceMAD[i]*n_realSize);
  }

  // take 2nd order model to estimate X1 and X2
  if ((n_realSize >= 1) && estimateX2)
  {
    for (i = 0; i < n_windowSize; i++)
    {
      if (!PictureRejected[i])
      {
        a00  = a00 + 1.0;
        a01 += p_quad->ReferenceMAD[i];
        a10  = a01;
        a11 += p_quad->ReferenceMAD[i] * p_quad->ReferenceMAD[i];
        b0  += p_quad->PictureMAD[i];
        b1  += p_quad->PictureMAD[i]   * p_quad->ReferenceMAD[i];
      }
    }
    // solve the equation of AX = B
    MatrixValue = a00 * a11 - a01 * a10;
    if(fabs(MatrixValue) > 0.000001)
    {
      p_quad->MADPictureC2 = (b0 * a11 - b1 * a01) / MatrixValue;
      p_quad->MADPictureC1 = (b1 * a00 - b0 * a10) / MatrixValue;
    }
    else
    {
      p_quad->MADPictureC1 = b0/a01;
      p_quad->MADPictureC2 = 0.0;
    }
  }
  if( p_Vid->type == P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && (p_Vid->number != 0)) )
  {
    p_quad->PMADPictureC1 = p_quad->MADPictureC1;
    p_quad->PMADPictureC2 = p_quad->MADPictureC2;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    compute a  quantization parameter for each frame (RC_MODE_0)
 *
 *************************************************************************************
*/
int updateQPRC0(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int topfield)
{
  int m_Bits;
  int BFrameNumber;
  int StepSize;
  int SumofBasicUnit;
  int MaxQpChange, m_Qp, m_Hp;

  /* frame layer rate control */
  if( p_Vid->BasicUnit == p_Vid->FrameSizeInMbs )
  {
    /* fixed quantization parameter is used to coded I frame, the first P frame and the first B frame
    the quantization parameter is adjusted according the available channel bandwidth and
    the type of video */
    /*top field*/
    if((topfield) || (p_gen->FieldControl==0))
    {
      if (p_Vid->type==I_SLICE)
      {
        p_quad->m_Qc = p_quad->MyInitialQp;
        return p_quad->m_Qc;
      }
      else if(p_Vid->type == B_SLICE)
      {
        if(p_Inp->NumberBFrames==1)
        {
          if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
            updateQPInterlace( p_quad, p_gen );

          p_quad->m_Qc = imin(p_quad->PrevLastQP, p_quad->CurrLastQP) + 2;
          p_quad->m_Qc = imax(p_quad->m_Qc, imax(p_quad->PrevLastQP, p_quad->CurrLastQP));
          p_quad->m_Qc = imax(p_quad->m_Qc, p_quad->CurrLastQP + 1);
          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping
        }
        else
        {
          BFrameNumber = (p_quad->NumberofBFrames + 1) % p_Inp->NumberBFrames;
          if(BFrameNumber==0)
            BFrameNumber = p_Inp->NumberBFrames;

          /*adaptive field/frame coding*/
          if(BFrameNumber==1)
          {
            if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
              updateQPInterlace( p_quad, p_gen );
          }

          if((p_quad->CurrLastQP-p_quad->PrevLastQP)<=(-2*p_Inp->NumberBFrames-3))
            StepSize=-3;
          else  if((p_quad->CurrLastQP-p_quad->PrevLastQP)==(-2*p_Inp->NumberBFrames-2))
            StepSize=-2;
          else if((p_quad->CurrLastQP-p_quad->PrevLastQP)==(-2*p_Inp->NumberBFrames-1))
            StepSize=-1;
          else if((p_quad->CurrLastQP-p_quad->PrevLastQP)==(-2*p_Inp->NumberBFrames))
            StepSize=0;
          else if((p_quad->CurrLastQP-p_quad->PrevLastQP)==(-2*p_Inp->NumberBFrames+1))
            StepSize=1;
          else
            StepSize=2;

          p_quad->m_Qc  = p_quad->PrevLastQP + StepSize;
          p_quad->m_Qc += iClip3( -2 * (BFrameNumber - 1), 2*(BFrameNumber-1),
            (BFrameNumber-1)*(p_quad->CurrLastQP-p_quad->PrevLastQP)/(p_Inp->NumberBFrames-1));
          p_quad->m_Qc  = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping
        }
        return p_quad->m_Qc;
      }
      else if( p_Vid->type == P_SLICE && p_quad->NumberofPPicture == 0 )
      {
        p_quad->m_Qc=p_quad->MyInitialQp;

        if(p_gen->FieldControl==0)
          updateQPNonPicAFF( p_Vid->active_sps, p_quad );
        return p_quad->m_Qc;
      }
      else
      {
        /*adaptive field/frame coding*/
        if( ( p_Inp->PicInterlace == ADAPTIVE_CODING || p_Inp->MbInterlace ) && p_gen->FieldControl == 0 )
          updateQPInterlaceBU( p_quad, p_gen );

        p_quad->m_X1 = p_quad->Pm_X1;
        p_quad->m_X2 = p_quad->Pm_X2;
        p_quad->MADPictureC1 = p_quad->PMADPictureC1;
        p_quad->MADPictureC2 = p_quad->PMADPictureC2;
        p_quad->PreviousPictureMAD = p_quad->PPictureMAD[0];

        MaxQpChange = p_quad->PMaxQpChange;
        m_Qp = p_quad->Pm_Qp;
        m_Hp = p_quad->PPreHeader;

        /* predict the MAD of current picture*/
        p_quad->CurrentFrameMAD = p_quad->MADPictureC1*p_quad->PreviousPictureMAD + p_quad->MADPictureC2;

        /*compute the number of bits for the texture*/
        if(p_quad->Target < 0)
        {
          p_quad->m_Qc=m_Qp+MaxQpChange;
          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping
        }
        else
        {
          m_Bits = p_quad->Target-m_Hp;
          m_Bits = imax(m_Bits, (int)(p_quad->bit_rate/(MINVALUE*p_quad->frame_rate)));

          updateModelQPFrame( p_quad, m_Bits );

          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // clipping
          p_quad->m_Qc = iClip3(m_Qp-MaxQpChange, m_Qp+MaxQpChange, p_quad->m_Qc); // control variation
        }

        if( p_gen->FieldControl == 0 )
          updateQPNonPicAFF( p_Vid->active_sps, p_quad );

        return p_quad->m_Qc;
      }
    }
    /*bottom field*/
    else
    {
      if( p_Vid->type == P_SLICE && p_gen->NoGranularFieldRC == 0 )
        updateBottomField( p_Inp, p_quad );
      return p_quad->m_Qc;
    }
  }
  /*basic unit layer rate control*/
  else
  {
    /*top field of I frame*/
    if (p_Vid->type == I_SLICE)
    {
      p_quad->m_Qc = p_quad->MyInitialQp;
      return p_quad->m_Qc;
    }
    else if( p_Vid->type == B_SLICE )
    {
      /*top field of B frame*/
      if((topfield)||(p_gen->FieldControl==0))
      {
        if(p_Inp->NumberBFrames==1)
        {
          /*adaptive field/frame coding*/
          if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
            updateQPInterlace( p_quad, p_gen );

          if(p_quad->PrevLastQP==p_quad->CurrLastQP)
            p_quad->m_Qc=p_quad->PrevLastQP+2;
          else
            p_quad->m_Qc = ((p_quad->PrevLastQP+p_quad->CurrLastQP) >> 1) + 1;
          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping
        }
        else
        {
          BFrameNumber=(p_quad->NumberofBFrames+1)%p_Inp->NumberBFrames;
          if(BFrameNumber==0)
            BFrameNumber=p_Inp->NumberBFrames;

          /*adaptive field/frame coding*/
          if(BFrameNumber==1)
          {
            if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
              updateQPInterlace( p_quad, p_gen );
          }

          if((p_quad->CurrLastQP-p_quad->PrevLastQP)<=(-2*p_Inp->NumberBFrames-3))
            StepSize=-3;
          else  if((p_quad->CurrLastQP-p_quad->PrevLastQP)==(-2*p_Inp->NumberBFrames-2))
            StepSize=-2;
          else if((p_quad->CurrLastQP-p_quad->PrevLastQP)==(-2*p_Inp->NumberBFrames-1))
            StepSize=-1;
          else if((p_quad->CurrLastQP-p_quad->PrevLastQP)==(-2*p_Inp->NumberBFrames))
            StepSize=0;//0
          else if((p_quad->CurrLastQP-p_quad->PrevLastQP)==(-2*p_Inp->NumberBFrames+1))
            StepSize=1;//1
          else
            StepSize=2;//2
          p_quad->m_Qc=p_quad->PrevLastQP+StepSize;
          p_quad->m_Qc +=
            iClip3( -2*(BFrameNumber-1), 2*(BFrameNumber-1), (BFrameNumber-1)*(p_quad->CurrLastQP-p_quad->PrevLastQP)/(p_Inp->NumberBFrames-1) );
          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping
        }
        return p_quad->m_Qc;
      }
      /*bottom field of B frame*/
      else
      {
        return p_quad->m_Qc;
      }
    }
    else if( p_Vid->type == P_SLICE )
    {
      if( (p_gen->NumberofGOP == 1) && (p_quad->NumberofPPicture == 0) )
      {
        if((p_gen->FieldControl==0)||((p_gen->FieldControl==1) && (p_gen->NoGranularFieldRC==0)))
          return updateFirstP( p_Vid, p_Inp, p_quad, p_gen, topfield );
      }
      else
      {
        p_quad->m_X1=p_quad->Pm_X1;
        p_quad->m_X2=p_quad->Pm_X2;
        p_quad->MADPictureC1=p_quad->PMADPictureC1;
        p_quad->MADPictureC2=p_quad->PMADPictureC2;

        m_Qp=p_quad->Pm_Qp;

        if(p_gen->FieldControl==0)
          SumofBasicUnit=p_quad->TotalNumberofBasicUnit;
        else
          SumofBasicUnit=p_quad->TotalNumberofBasicUnit>>1;

        /*the average QP of the previous frame is used to coded the first basic unit of the current frame or field*/
        if(p_quad->NumberofBasicUnit==SumofBasicUnit)
          return updateFirstBU( p_Vid, p_Inp, p_quad, p_gen, topfield );
        else
        {
          /*compute the number of remaining bits*/
          p_quad->Target -= (p_gen->NumberofBasicUnitHeaderBits + p_gen->NumberofBasicUnitTextureBits);
          p_gen->NumberofBasicUnitHeaderBits  = 0;
          p_gen->NumberofBasicUnitTextureBits = 0;
          if(p_quad->Target<0)
            return updateNegativeTarget( p_Vid, p_Inp, p_quad, p_gen, topfield, m_Qp );
          else
          {
            /*predict the MAD of current picture*/
            predictCurrPicMAD( p_Inp, p_quad, p_gen );

            /*compute the total number of bits for the current basic unit*/
            updateModelQPBU( p_Vid, p_Inp, p_quad, m_Qp );

            p_quad->TotalFrameQP +=p_quad->m_Qc;
            p_quad->Pm_Qp=p_quad->m_Qc;
            p_quad->NumberofBasicUnit--;
            if( p_quad->NumberofBasicUnit == 0 && p_Vid->type == P_SLICE )
              updateLastBU( p_Vid, p_Inp, p_quad, p_gen, topfield );

            return p_quad->m_Qc;
          }
        }
      }
    }
  }
  return p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    compute a  quantization parameter for each frame
 *
 *************************************************************************************
*/
int updateQPRC1(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int topfield)
{
  int m_Bits;
  int SumofBasicUnit;
  int MaxQpChange, m_Qp, m_Hp;

  /* frame layer rate control */
  if( p_Vid->BasicUnit == p_Vid->FrameSizeInMbs )
  {
    /* fixed quantization parameter is used to coded I frame, the first P frame and the first B frame
    the quantization parameter is adjusted according the available channel bandwidth and
    the type of vide */
    /*top field*/
    if((topfield) || (p_gen->FieldControl==0))
    {
      if (p_Vid->number == 0)
      {
        p_quad->m_Qc = p_quad->MyInitialQp;
        return p_quad->m_Qc;
      }
      else if( p_quad->NumberofPPicture == 0 && (p_Vid->number != 0))
      {
        p_quad->m_Qc=p_quad->MyInitialQp;

        if(p_gen->FieldControl==0)
          updateQPNonPicAFF( p_Vid->active_sps, p_quad );
        return p_quad->m_Qc;
      }
      else
      {
        /*adaptive field/frame coding*/
        if( ( p_Inp->PicInterlace == ADAPTIVE_CODING || p_Inp->MbInterlace ) && p_gen->FieldControl == 0 )
          updateQPInterlaceBU( p_quad, p_gen );

        p_quad->m_X1 = p_quad->Pm_X1;
        p_quad->m_X2 = p_quad->Pm_X2;
        p_quad->MADPictureC1 = p_quad->PMADPictureC1;
        p_quad->MADPictureC2 = p_quad->PMADPictureC2;
        p_quad->PreviousPictureMAD = p_quad->PPictureMAD[0];

        MaxQpChange = p_quad->PMaxQpChange;
        m_Qp = p_quad->Pm_Qp;
        m_Hp = p_quad->PPreHeader;

        /* predict the MAD of current picture*/
        p_quad->CurrentFrameMAD=p_quad->MADPictureC1*p_quad->PreviousPictureMAD + p_quad->MADPictureC2;

        /*compute the number of bits for the texture*/
        if(p_quad->Target < 0)
        {
          p_quad->m_Qc=m_Qp+MaxQpChange;
          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping
        }
        else
        {
          m_Bits = p_quad->Target-m_Hp;
          m_Bits = imax(m_Bits, (int)(p_quad->bit_rate/(MINVALUE*p_quad->frame_rate)));

          updateModelQPFrame( p_quad, m_Bits );

          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // clipping
          p_quad->m_Qc = iClip3(m_Qp-MaxQpChange, m_Qp+MaxQpChange, p_quad->m_Qc); // control variation
        }

        if( p_gen->FieldControl == 0 )
          updateQPNonPicAFF( p_Vid->active_sps, p_quad );

        return p_quad->m_Qc;
      }
    }
    /*bottom field*/
    else
    {
      if( p_gen->NoGranularFieldRC == 0 )
        updateBottomField( p_Inp, p_quad );
      return p_quad->m_Qc;
    }
  }
  /*basic unit layer rate control*/
  else
  {
    /*top field of I frame*/
    if (p_Vid->number == 0)
    {
      p_quad->m_Qc = p_quad->MyInitialQp;
      return p_quad->m_Qc;
    }
    else
    {
      if((p_gen->NumberofGOP==1)&&(p_quad->NumberofPPicture==0))
      {
        if((p_gen->FieldControl==0)||((p_gen->FieldControl==1) && (p_gen->NoGranularFieldRC==0)))
          return updateFirstP( p_Vid, p_Inp, p_quad, p_gen, topfield );
      }
      else
      {
        p_quad->m_X1=p_quad->Pm_X1;
        p_quad->m_X2=p_quad->Pm_X2;
        p_quad->MADPictureC1=p_quad->PMADPictureC1;
        p_quad->MADPictureC2=p_quad->PMADPictureC2;

        m_Qp=p_quad->Pm_Qp;

        if(p_gen->FieldControl==0)
          SumofBasicUnit=p_quad->TotalNumberofBasicUnit;
        else
          SumofBasicUnit=p_quad->TotalNumberofBasicUnit>>1;

        /*the average QP of the previous frame is used to coded the first basic unit of the current frame or field*/
        if(p_quad->NumberofBasicUnit==SumofBasicUnit)
          return updateFirstBU( p_Vid, p_Inp, p_quad, p_gen, topfield );
        else
        {
          /*compute the number of remaining bits*/
          p_quad->Target -= (p_gen->NumberofBasicUnitHeaderBits + p_gen->NumberofBasicUnitTextureBits);
          p_gen->NumberofBasicUnitHeaderBits  = 0;
          p_gen->NumberofBasicUnitTextureBits = 0;
          if(p_quad->Target<0)
            return updateNegativeTarget( p_Vid, p_Inp, p_quad, p_gen, topfield, m_Qp );
          else
          {
            /*predict the MAD of current picture*/
            predictCurrPicMAD( p_Inp, p_quad, p_gen );

            /*compute the total number of bits for the current basic unit*/
            updateModelQPBU( p_Vid, p_Inp, p_quad, m_Qp );

            p_quad->TotalFrameQP +=p_quad->m_Qc;
            p_quad->Pm_Qp=p_quad->m_Qc;
            p_quad->NumberofBasicUnit--;
            if((p_quad->NumberofBasicUnit==0) && (p_Vid->number != 0))
              updateLastBU( p_Vid, p_Inp, p_quad, p_gen, topfield );

            return p_quad->m_Qc;
          }
        }
      }
    }
  }
  return p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    compute a  quantization parameter for each frame
 *
 *************************************************************************************
*/
int updateQPRC2(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int topfield)
{
  int m_Bits;
  int SumofBasicUnit;
  int MaxQpChange, m_Qp, m_Hp;

  /* frame layer rate control */
  if( p_Vid->BasicUnit == p_Vid->FrameSizeInMbs )
  {
    /* fixed quantization parameter is used to coded I frame, the first P frame and the first B frame
    the quantization parameter is adjusted according the available channel bandwidth and
    the type of vide */
    /*top field*/
    if((topfield) || (p_gen->FieldControl==0))
    {
      if (p_Vid->number == 0)
      {
        p_quad->m_Qc = p_quad->MyInitialQp;
        return p_quad->m_Qc;
      }
      else if (p_Vid->type==I_SLICE)
      {
        if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
          updateQPInterlace( p_quad, p_gen );

        if ( p_Vid->p_curr_frm_struct->idr_flag )
          p_quad->m_Qc = p_quad->MyInitialQp;
        else
          p_quad->m_Qc = p_quad->CurrLastQP; // Set QP to average qp of last P frame

        return p_quad->m_Qc;
      }
      else if(p_Vid->type == B_SLICE)
      {
        int prevQP = imax(p_quad->PrevLastQP, p_quad->CurrLastQP);
        // for more than one consecutive B frames the below call will overwrite the old anchor frame QP with the current value
        // it should be called once in the B-frame sequence....this should be modified for the BU < frame as well
        if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
          updateQPInterlace( p_quad, p_gen );

        if (p_Inp->HierarchicalCoding)
        {
          p_quad->m_Qc = prevQP + p_Vid->p_curr_frm_struct->layer;
        }
        else
          p_quad->m_Qc = prevQP + 2 - p_Vid->nal_reference_idc;
        p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping

        return p_quad->m_Qc;
      }
      else if( p_Vid->type == P_SLICE && p_quad->NumberofPPicture == 0 )
      {
        p_quad->m_Qc=p_quad->MyInitialQp;

        if(p_gen->FieldControl==0)
          updateQPNonPicAFF( p_Vid->active_sps, p_quad );
        return p_quad->m_Qc;
      }
      else
      {
        /*adaptive field/frame coding*/
        if( ( p_Inp->PicInterlace == ADAPTIVE_CODING || p_Inp->MbInterlace ) && p_gen->FieldControl == 0 )
          updateQPInterlaceBU( p_quad, p_gen );

        p_quad->m_X1 = p_quad->Pm_X1;
        p_quad->m_X2 = p_quad->Pm_X2;
        p_quad->MADPictureC1 = p_quad->PMADPictureC1;
        p_quad->MADPictureC2 = p_quad->PMADPictureC2;
        p_quad->PreviousPictureMAD = p_quad->PPictureMAD[0];

        MaxQpChange = p_quad->PMaxQpChange;
        m_Qp = p_quad->Pm_Qp;
        m_Hp = p_quad->PPreHeader;

        /* predict the MAD of current picture*/
        p_quad->CurrentFrameMAD=p_quad->MADPictureC1*p_quad->PreviousPictureMAD + p_quad->MADPictureC2;

        /*compute the number of bits for the texture*/
        if(p_quad->Target < 0)
        {
          p_quad->m_Qc=m_Qp+MaxQpChange;
          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping
        }
        else
        {
          m_Bits = p_quad->Target-m_Hp;
          m_Bits = imax(m_Bits, (int)(p_quad->bit_rate/(MINVALUE*p_quad->frame_rate)));

          updateModelQPFrame( p_quad, m_Bits );

          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // clipping
          p_quad->m_Qc = iClip3(m_Qp-MaxQpChange, m_Qp+MaxQpChange, p_quad->m_Qc); // control variation
        }

        if( p_gen->FieldControl == 0 )
          updateQPNonPicAFF( p_Vid->active_sps, p_quad );

        return p_quad->m_Qc;
      }
    }
    /*bottom field*/
    else
    {
      if( p_Vid->type==P_SLICE && p_gen->NoGranularFieldRC == 0 )
        updateBottomField( p_Inp, p_quad );
      return p_quad->m_Qc;
    }
  }
  /*basic unit layer rate control*/
  else
  {
    /*top field of I frame*/
    if (p_Vid->number == 0)
    {
      p_quad->m_Qc = p_quad->MyInitialQp;
      return p_quad->m_Qc;
    }
    else if (p_Vid->type==I_SLICE)
    {
      /*adaptive field/frame coding*/
      if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
        updateQPInterlace( p_quad, p_gen );

      p_quad->m_Qc = p_quad->PrevLastQP; // Set QP to average qp of last P frame
      p_quad->PrevLastQP = p_quad->CurrLastQP;
      p_quad->CurrLastQP = p_quad->PrevLastQP;
      p_quad->PAveFrameQP = p_quad->CurrLastQP;

      return p_quad->m_Qc;
    }
    else if(p_Vid->type == B_SLICE)
    {
      int prevQP = imax(p_quad->PrevLastQP, p_quad->CurrLastQP);
      if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
        updateQPInterlace( p_quad, p_gen );

      if (p_Inp->HierarchicalCoding)
      {
        p_quad->m_Qc = prevQP + p_Vid->p_curr_frm_struct->layer;
      }
      else
        p_quad->m_Qc = prevQP + 2 - p_Vid->nal_reference_idc;

      p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping

      return p_quad->m_Qc;

    }
    else if( p_Vid->type == P_SLICE )
    {
      if((p_gen->NumberofGOP==1)&&(p_quad->NumberofPPicture==0))
      {
        if((p_gen->FieldControl==0)||((p_gen->FieldControl==1) && (p_gen->NoGranularFieldRC==0)))
          return updateFirstP( p_Vid, p_Inp, p_quad, p_gen, topfield );
      }
      else
      {
        p_quad->m_X1=p_quad->Pm_X1;
        p_quad->m_X2=p_quad->Pm_X2;
        p_quad->MADPictureC1=p_quad->PMADPictureC1;
        p_quad->MADPictureC2=p_quad->PMADPictureC2;

        m_Qp=p_quad->Pm_Qp;

        if(p_gen->FieldControl==0)
          SumofBasicUnit=p_quad->TotalNumberofBasicUnit;
        else
          SumofBasicUnit=p_quad->TotalNumberofBasicUnit>>1;

        /*the average QP of the previous frame is used to coded the first basic unit of the current frame or field*/
        if(p_quad->NumberofBasicUnit==SumofBasicUnit)
          return updateFirstBU( p_Vid, p_Inp, p_quad, p_gen, topfield );
        else
        {
          /*compute the number of remaining bits*/
          p_quad->Target -= (p_gen->NumberofBasicUnitHeaderBits + p_gen->NumberofBasicUnitTextureBits);
          p_gen->NumberofBasicUnitHeaderBits  = 0;
          p_gen->NumberofBasicUnitTextureBits = 0;
          if(p_quad->Target<0)
            return updateNegativeTarget( p_Vid, p_Inp, p_quad, p_gen, topfield, m_Qp );
          else
          {
            /*predict the MAD of current picture*/
            predictCurrPicMAD( p_Inp, p_quad, p_gen );

            /*compute the total number of bits for the current basic unit*/
            updateModelQPBU( p_Vid, p_Inp, p_quad, m_Qp );

            p_quad->TotalFrameQP +=p_quad->m_Qc;
            p_quad->Pm_Qp=p_quad->m_Qc;
            p_quad->NumberofBasicUnit--;
            if((p_quad->NumberofBasicUnit==0) && p_Vid->type == P_SLICE )
              updateLastBU( p_Vid, p_Inp, p_quad, p_gen, topfield );

            return p_quad->m_Qc;
          }
        }
      }
    }
  }
  return p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    compute a  quantization parameter for each frame
 *
 *************************************************************************************
*/
int updateQPRC3(VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int topfield)
{
  int m_Bits;
  int SumofBasicUnit;
  int MaxQpChange, m_Qp, m_Hp;

  /* frame layer rate control */
  if( p_Vid->BasicUnit == p_Vid->FrameSizeInMbs || p_Vid->type != P_SLICE )
  {
    /* fixed quantization parameter is used to coded I frame, the first P frame and the first B frame
    the quantization parameter is adjusted according the available channel bandwidth and
    the type of video */
    /*top field*/
    if((topfield) || (p_gen->FieldControl==0))
    {
      if (p_Vid->number == 0)
      {
        if((p_Inp->PicInterlace == ADAPTIVE_CODING) || (p_Inp->MbInterlace))
          updateQPInterlace( p_quad, p_gen );
        p_quad->m_Qc = p_quad->MyInitialQp;
        return p_quad->m_Qc;
      }
      else if( p_Vid->type == P_SLICE && p_quad->NumberofPPicture == 0 )
      {
        p_quad->m_Qc=p_quad->MyInitialQp;

        if(p_gen->FieldControl==0)
          updateQPNonPicAFF( p_Vid->active_sps, p_quad );
        return p_quad->m_Qc;
      }
      else
      {
        if( ( (p_Vid->type == B_SLICE && (p_Vid->p_curr_frm_struct->layer && p_Vid->p_curr_frm_struct->atom_idx == 1)) || p_Vid->type == I_SLICE) && ((p_Inp->PicInterlace == ADAPTIVE_CODING) || (p_Inp->MbInterlace)) )
          updateQPInterlace( p_quad, p_gen );
        /*adaptive field/frame coding*/
        if( p_Vid->type == P_SLICE && ( p_Inp->PicInterlace == ADAPTIVE_CODING || p_Inp->MbInterlace ) && p_gen->FieldControl == 0 )
          updateQPInterlaceBU( p_quad, p_gen );

        p_quad->m_X1 = p_quad->Pm_X1;
        p_quad->m_X2 = p_quad->Pm_X2;
        p_quad->MADPictureC1 = p_quad->PMADPictureC1;
        p_quad->MADPictureC2 = p_quad->PMADPictureC2;
        p_quad->PreviousPictureMAD = p_quad->PPictureMAD[0];

        MaxQpChange = p_quad->PMaxQpChange;
        m_Qp = p_quad->Pm_Qp;
        m_Hp = p_quad->PPreHeader;

        if ( p_Vid->BasicUnit < p_Vid->FrameSizeInMbs && p_Vid->type != P_SLICE )
        {
          // when RC_MODE_3 is set and basic unit is smaller than a frame, note that:
          // the linear MAD model and the quadratic QP model operate on small units and not on a whole frame;
          // we therefore have to account for this
          p_quad->PreviousPictureMAD = p_quad->PreviousWholeFrameMAD;
        }
        if ( p_Vid->type == I_SLICE )
          m_Hp = 0; // it is usually a very small portion of the total I_SLICE bit budget

        /* predict the MAD of current picture*/
        p_quad->CurrentFrameMAD=p_quad->MADPictureC1*p_quad->PreviousPictureMAD + p_quad->MADPictureC2;

        /*compute the number of bits for the texture*/
        if(p_quad->Target < 0)
        {
          p_quad->m_Qc=m_Qp+MaxQpChange;
          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // Clipping
        }
        else
        {
          if ( p_Vid->type != P_SLICE )
          {
            if ( p_Vid->BasicUnit < p_Vid->FrameSizeInMbs )
              m_Bits =(p_quad->Target-m_Hp)/p_quad->TotalNumberofBasicUnit;
            else
              m_Bits =p_quad->Target-m_Hp;
          }
          else {
            m_Bits = p_quad->Target-m_Hp;
            m_Bits = imax(m_Bits, (int)(p_quad->bit_rate/(MINVALUE*p_quad->frame_rate)));
          }          
          updateModelQPFrame( p_quad, m_Bits );

          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // clipping
          if ( p_Vid->type == P_SLICE )
            p_quad->m_Qc = iClip3(m_Qp-MaxQpChange, m_Qp+MaxQpChange, p_quad->m_Qc); // control variation
        }

        if( p_Vid->type == P_SLICE && p_gen->FieldControl == 0 )
          updateQPNonPicAFF( p_Vid->active_sps, p_quad );

        if ( p_Vid->type == B_SLICE )
        {
          // hierarchical adjustment
          int prevqp = ((p_quad->PrevLastQP+p_quad->CurrLastQP) >> 1) + 1;
          if ( p_Inp->HierarchicalCoding && p_Vid->p_curr_frm_struct->layer)
            p_quad->m_Qc -= (p_Vid->p_curr_frm_struct->p_atom->gop_levels - p_Vid->p_curr_frm_struct->layer);
          // check bounds
          p_quad->m_Qc = iClip3(prevqp - (p_Inp->HierarchicalCoding ? 0 : 5), prevqp + 5, p_quad->m_Qc); // control variation
          p_quad->m_Qc = iClip3(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // clipping
        }
        return p_quad->m_Qc;
      }
    }
    /*bottom field*/
    else
    {
      if( p_Vid->type==P_SLICE && p_gen->NoGranularFieldRC == 0 )
        updateBottomField( p_Inp, p_quad );
      return p_quad->m_Qc;
    }
  }
  /*basic unit layer rate control*/
  else
  {
    /*top field of I frame*/
    if (p_Vid->number == 0)
    {
      if((p_Inp->PicInterlace == ADAPTIVE_CODING) || (p_Inp->MbInterlace))
        updateQPInterlace( p_quad, p_gen );
      p_quad->m_Qc = p_quad->MyInitialQp;
      return p_quad->m_Qc;
    }
    else if( p_Vid->type == P_SLICE )
    {
      if((p_gen->NumberofGOP==1)&&(p_quad->NumberofPPicture==0))
      {
        if((p_gen->FieldControl==0)||((p_gen->FieldControl==1) && (p_gen->NoGranularFieldRC==0)))
          return updateFirstP( p_Vid, p_Inp, p_quad, p_gen, topfield );
      }
      else
      {
        if( ( (p_Vid->type == B_SLICE && (p_Vid->p_curr_frm_struct->layer && p_Vid->p_curr_frm_struct->atom_idx == 1)) || p_Vid->type == I_SLICE) && ((p_Inp->PicInterlace == ADAPTIVE_CODING) || (p_Inp->MbInterlace)) )
          updateQPInterlace( p_quad, p_gen );
        p_quad->m_X1=p_quad->Pm_X1;
        p_quad->m_X2=p_quad->Pm_X2;
        p_quad->MADPictureC1=p_quad->PMADPictureC1;
        p_quad->MADPictureC2=p_quad->PMADPictureC2;

        m_Qp=p_quad->Pm_Qp;

        if(p_gen->FieldControl==0)
          SumofBasicUnit=p_quad->TotalNumberofBasicUnit;
        else
          SumofBasicUnit=p_quad->TotalNumberofBasicUnit>>1;

        /*the average QP of the previous frame is used to coded the first basic unit of the current frame or field*/
        if(p_quad->NumberofBasicUnit==SumofBasicUnit)
          return updateFirstBU( p_Vid, p_Inp, p_quad, p_gen, topfield );
        else
        {
          /*compute the number of remaining bits*/
          p_quad->Target -= (p_gen->NumberofBasicUnitHeaderBits + p_gen->NumberofBasicUnitTextureBits);
          p_gen->NumberofBasicUnitHeaderBits  = 0;
          p_gen->NumberofBasicUnitTextureBits = 0;
          if(p_quad->Target<0)
            return updateNegativeTarget( p_Vid, p_Inp, p_quad, p_gen, topfield, m_Qp );
          else
          {
            /*predict the MAD of current picture*/
            predictCurrPicMAD( p_Inp, p_quad, p_gen );

            /*compute the total number of bits for the current basic unit*/
            updateModelQPBU( p_Vid, p_Inp, p_quad, m_Qp );

            p_quad->TotalFrameQP +=p_quad->m_Qc;
            p_quad->Pm_Qp=p_quad->m_Qc;
            p_quad->NumberofBasicUnit--;
            if((p_quad->NumberofBasicUnit==0) && p_Vid->type == P_SLICE )
              updateLastBU( p_Vid, p_Inp, p_quad, p_gen, topfield );

            return p_quad->m_Qc;
          }
        }
      }
    }
  }
  return p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    Save previous QP values for interlaced coding
 *
 *************************************************************************************
*/
void updateQPInterlace( RCQuadratic *p_quad, RCGeneric *p_gen )
{
  if(p_gen->FieldControl==0)
  {
    /*previous choice is frame coding*/
    if(p_gen->FieldFrame==1)
    {
      p_quad->PrevLastQP=p_quad->CurrLastQP;
      p_quad->CurrLastQP=p_quad->FrameQPBuffer;
    }
    /*previous choice is field coding*/
    else
    {
      p_quad->PrevLastQP=p_quad->CurrLastQP;
      p_quad->CurrLastQP=p_quad->FieldQPBuffer;
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Save previous QP values for the case of non-PicAFF
 *
 *************************************************************************************
*/
void updateQPNonPicAFF( seq_parameter_set_rbsp_t *active_sps, RCQuadratic *p_quad )
{
  if(active_sps->frame_mbs_only_flag)
  {
    p_quad->TotalQpforPPicture +=p_quad->m_Qc;
    p_quad->PrevLastQP=p_quad->CurrLastQP;
    p_quad->CurrLastQP=p_quad->m_Qc;
    p_quad->Pm_Qp=p_quad->m_Qc;
  }
  /*adaptive field/frame coding*/
  else
    p_quad->FrameQPBuffer=p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    Update QP values for bottom field in field coding
 *************************************************************************************
*/
void updateBottomField( InputParameters *p_Inp, RCQuadratic *p_quad )
{
  /*field coding*/
  if(p_Inp->PicInterlace==FIELD_CODING)
  {
    p_quad->TotalQpforPPicture +=p_quad->m_Qc;
    p_quad->PrevLastQP=p_quad->CurrLastQP+1;
    p_quad->CurrLastQP=p_quad->m_Qc;//+0 Recent change 13/1/2003
    p_quad->Pm_Qp=p_quad->m_Qc;
  }
  /*adaptive field/frame coding*/
  else
    p_quad->FieldQPBuffer=p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    update QP variables for P frames
 *************************************************************************************
*/
int updateFirstP( VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int topfield )
{

  /*top field of the first P frame*/
  p_quad->m_Qc=p_quad->MyInitialQp;
  p_gen->NumberofBasicUnitHeaderBits=0;
  p_gen->NumberofBasicUnitTextureBits=0;
  p_quad->NumberofBasicUnit--;
  /*bottom field of the first P frame*/
  if((!topfield)&&(p_quad->NumberofBasicUnit==0))
  {
    /*frame coding or field coding*/
    if((p_Vid->active_sps->frame_mbs_only_flag)||(p_Inp->PicInterlace==FIELD_CODING))
    {
      p_quad->TotalQpforPPicture +=p_quad->m_Qc;
      p_quad->PrevLastQP=p_quad->CurrLastQP;
      p_quad->CurrLastQP=p_quad->m_Qc;
      p_quad->PAveFrameQP=p_quad->m_Qc;
      p_quad->PAveHeaderBits3=p_quad->PAveHeaderBits2;
    }
    /*adaptive frame/field coding*/
    else if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
    {
      if(p_gen->FieldControl==0)
      {
        p_quad->FrameQPBuffer=p_quad->m_Qc;
        p_quad->FrameAveHeaderBits=p_quad->PAveHeaderBits2;
      }
      else
      {
        p_quad->FieldQPBuffer=p_quad->m_Qc;
        p_quad->FieldAveHeaderBits=p_quad->PAveHeaderBits2;
      }
    }
  }
  p_quad->Pm_Qp=p_quad->m_Qc;
  p_quad->TotalFrameQP +=p_quad->m_Qc;
  return p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    update QP when bit target is negative
 *************************************************************************************
*/
int updateNegativeTarget( VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int topfield, int m_Qp )
{
  int PAverageQP;

  if(p_quad->GOPOverdue==TRUE)
    p_quad->m_Qc=m_Qp+2;
  else
    p_quad->m_Qc=m_Qp+p_quad->DDquant;//2

  p_quad->m_Qc = imin(p_quad->m_Qc, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale);  // clipping
  if(p_Inp->basicunit>=p_quad->MBPerRow)
    p_quad->m_Qc = imin(p_quad->m_Qc, p_quad->PAveFrameQP + 6);
  else
    p_quad->m_Qc = imin(p_quad->m_Qc, p_quad->PAveFrameQP + 3);

  p_quad->TotalFrameQP +=p_quad->m_Qc;
  p_quad->NumberofBasicUnit--;
  if(p_quad->NumberofBasicUnit==0)
  {
    if((!topfield)||(p_gen->FieldControl==0))
    {
      /*frame coding or field coding*/
      if((p_Vid->active_sps->frame_mbs_only_flag)||(p_Inp->PicInterlace==FIELD_CODING))
      {
        PAverageQP=(int)((double)p_quad->TotalFrameQP/(double)p_quad->TotalNumberofBasicUnit+0.5);
        if (p_quad->NumberofPPicture == (p_Inp->intra_period - 2))
          p_quad->QPLastPFrame = PAverageQP;

        p_quad->TotalQpforPPicture +=PAverageQP;
        if(p_quad->GOPOverdue==TRUE)
        {
          p_quad->PrevLastQP=p_quad->CurrLastQP+1;
          p_quad->CurrLastQP=PAverageQP;
        }
        else
        {
          if((p_quad->NumberofPPicture==0)&&(p_gen->NumberofGOP>1))
          {
            p_quad->PrevLastQP=p_quad->CurrLastQP;
            p_quad->CurrLastQP=PAverageQP;
          }
          else if(p_quad->NumberofPPicture>0)
          {
            p_quad->PrevLastQP=p_quad->CurrLastQP+1;
            p_quad->CurrLastQP=PAverageQP;
          }
        }
        p_quad->PAveFrameQP=PAverageQP;
        p_quad->PAveHeaderBits3=p_quad->PAveHeaderBits2;
      }
      /*adaptive field/frame coding*/
      else if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
      {
        if(p_gen->FieldControl==0)
        {
          PAverageQP=(int)((double)p_quad->TotalFrameQP/(double)p_quad->TotalNumberofBasicUnit+0.5);
          p_quad->FrameQPBuffer=PAverageQP;
          p_quad->FrameAveHeaderBits=p_quad->PAveHeaderBits2;
        }
        else
        {
          PAverageQP=(int)((double)p_quad->TotalFrameQP/(double)p_quad->TotalNumberofBasicUnit+0.5);
          p_quad->FieldQPBuffer=PAverageQP;
          p_quad->FieldAveHeaderBits=p_quad->PAveHeaderBits2;
        }
      }
    }
  }
  if(p_quad->GOPOverdue==TRUE)
    p_quad->Pm_Qp=p_quad->PAveFrameQP;
  else
    p_quad->Pm_Qp=p_quad->m_Qc;

  return p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    update QP for the first Basic Unit in the picture
 *************************************************************************************
*/
int updateFirstBU( VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int topfield )
{
  /*adaptive field/frame coding*/
  if(((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))&&(p_gen->FieldControl==0))
  {
    /*previous choice is frame coding*/
    if(p_gen->FieldFrame==1)
    {
      if(p_quad->NumberofPPicture>0)
        p_quad->TotalQpforPPicture +=p_quad->FrameQPBuffer;
      p_quad->PAveFrameQP=p_quad->FrameQPBuffer;
      p_quad->PAveHeaderBits3=p_quad->FrameAveHeaderBits;
    }
    /*previous choice is field coding*/
    else
    {
      if(p_quad->NumberofPPicture>0)
        p_quad->TotalQpforPPicture +=p_quad->FieldQPBuffer;
      p_quad->PAveFrameQP=p_quad->FieldQPBuffer;
      p_quad->PAveHeaderBits3=p_quad->FieldAveHeaderBits;
    }
  }

  if(p_quad->Target<=0)
  {
    p_quad->m_Qc = p_quad->PAveFrameQP + 2;
    if(p_quad->m_Qc > (p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale))
      p_quad->m_Qc = p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale;

    if(topfield||(p_gen->FieldControl==0))
      p_quad->GOPOverdue=TRUE;
  }
  else
  {
    p_quad->m_Qc=p_quad->PAveFrameQP;
  }
  p_quad->TotalFrameQP +=p_quad->m_Qc;
  p_quad->NumberofBasicUnit--;
  p_quad->Pm_Qp = p_quad->PAveFrameQP;

  return p_quad->m_Qc;
}

/*!
 *************************************************************************************
 * \brief
 *    update QP for the last Basic Unit in the picture
 *************************************************************************************
*/
void updateLastBU( VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen, int topfield )
{
  int PAverageQP;

  if((!topfield)||(p_gen->FieldControl==0))
  {
    /*frame coding or field coding*/
    if((p_Vid->active_sps->frame_mbs_only_flag)||(p_Inp->PicInterlace==FIELD_CODING))
    {
      PAverageQP=(int)((double)p_quad->TotalFrameQP/(double) p_quad->TotalNumberofBasicUnit+0.5);
      if (p_quad->NumberofPPicture == (p_Inp->intra_period - 2))
        p_quad->QPLastPFrame = PAverageQP;

      p_quad->TotalQpforPPicture +=PAverageQP;
      p_quad->PrevLastQP=p_quad->CurrLastQP;
      p_quad->CurrLastQP=PAverageQP;
      p_quad->PAveFrameQP=PAverageQP;
      p_quad->PAveHeaderBits3=p_quad->PAveHeaderBits2;
    }
    else if((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))
    {
      if(p_gen->FieldControl==0)
      {
        PAverageQP=(int)((double) p_quad->TotalFrameQP/(double)p_quad->TotalNumberofBasicUnit+0.5);
        p_quad->FrameQPBuffer=PAverageQP;
        p_quad->FrameAveHeaderBits=p_quad->PAveHeaderBits2;
      }
      else
      {
        PAverageQP=(int)((double) p_quad->TotalFrameQP/(double) p_quad->TotalNumberofBasicUnit+0.5);
        p_quad->FieldQPBuffer=PAverageQP;
        p_quad->FieldAveHeaderBits=p_quad->PAveHeaderBits2;
      }
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    update current picture MAD
 *************************************************************************************
*/
void predictCurrPicMAD( InputParameters *p_Inp, RCQuadratic *p_quad, RCGeneric *p_gen )
{
  int i;
  if(((p_Inp->PicInterlace==ADAPTIVE_CODING)||(p_Inp->MbInterlace))&&(p_gen->FieldControl==1))
  {
    p_quad->CurrentFrameMAD=p_quad->MADPictureC1*p_quad->FCBUPFMAD[p_quad->TotalNumberofBasicUnit-p_quad->NumberofBasicUnit]+p_quad->MADPictureC2;
    p_quad->TotalBUMAD=0;
    for(i=p_quad->TotalNumberofBasicUnit-1; i>=(p_quad->TotalNumberofBasicUnit-p_quad->NumberofBasicUnit);i--)
    {
      p_quad->CurrentBUMAD=p_quad->MADPictureC1*p_quad->FCBUPFMAD[i]+p_quad->MADPictureC2;
      p_quad->TotalBUMAD +=p_quad->CurrentBUMAD*p_quad->CurrentBUMAD;
    }
  }
  else
  {
    p_quad->CurrentFrameMAD=p_quad->MADPictureC1*p_quad->BUPFMAD[p_quad->TotalNumberofBasicUnit-p_quad->NumberofBasicUnit]+p_quad->MADPictureC2;
    p_quad->TotalBUMAD=0;
    for(i=p_quad->TotalNumberofBasicUnit-1; i>=(p_quad->TotalNumberofBasicUnit-p_quad->NumberofBasicUnit);i--)
    {
      p_quad->CurrentBUMAD=p_quad->MADPictureC1*p_quad->BUPFMAD[i]+p_quad->MADPictureC2;
      p_quad->TotalBUMAD +=p_quad->CurrentBUMAD*p_quad->CurrentBUMAD;
    }
  }
}

/*!
 *************************************************************************************
 * \brief
 *    update QP using the quadratic model for basic unit coding
 *************************************************************************************
*/
void updateModelQPBU( VideoParameters *p_Vid, InputParameters *p_Inp, RCQuadratic *p_quad, int m_Qp )
{
  double dtmp, m_Qstep;
  int m_Bits;
  /*compute the total number of bits for the current basic unit*/
  m_Bits =(int)(p_quad->Target * p_quad->CurrentFrameMAD * p_quad->CurrentFrameMAD / p_quad->TotalBUMAD);
  /*compute the number of texture bits*/
  m_Bits -=p_quad->PAveHeaderBits2;

  m_Bits=imax(m_Bits,(int)(p_quad->bit_rate/(MINVALUE*p_quad->frame_rate*p_quad->TotalNumberofBasicUnit)));

  dtmp = p_quad->CurrentFrameMAD * p_quad->CurrentFrameMAD * p_quad->m_X1 * p_quad->m_X1 \
    + 4 * p_quad->m_X2 * p_quad->CurrentFrameMAD * m_Bits;
  if ((p_quad->m_X2 == 0.0) || (dtmp < 0) || ((sqrt (dtmp) - p_quad->m_X1 * p_quad->CurrentFrameMAD) <= 0.0))  // fall back 1st order mode
    m_Qstep = (float)(p_quad->m_X1 * p_quad->CurrentFrameMAD / (double) m_Bits);
  else // 2nd order mode
    m_Qstep = (float) ((2 * p_quad->m_X2 * p_quad->CurrentFrameMAD) / (sqrt (dtmp) - p_quad->m_X1 * p_quad->CurrentFrameMAD));

  p_quad->m_Qc = Qstep2QP(m_Qstep, p_quad->bitdepth_qp_scale);
  p_quad->m_Qc = imin(m_Qp+p_quad->DDquant,  p_quad->m_Qc); // control variation

  if(p_Inp->basicunit>=p_quad->MBPerRow)
    p_quad->m_Qc = imin(p_quad->PAveFrameQP+6, p_quad->m_Qc);
  else
    p_quad->m_Qc = imin(p_quad->PAveFrameQP+3, p_quad->m_Qc);

  p_quad->m_Qc = iClip3(m_Qp-p_quad->DDquant, p_Vid->RCMaxQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc); // clipping
  if(p_Inp->basicunit>=p_quad->MBPerRow)
    p_quad->m_Qc = imax(p_quad->PAveFrameQP-6, p_quad->m_Qc);
  else
    p_quad->m_Qc = imax(p_quad->PAveFrameQP-3, p_quad->m_Qc);

  p_quad->m_Qc = imax(p_Vid->RCMinQP + p_quad->bitdepth_qp_scale, p_quad->m_Qc);
}

/*!
 *************************************************************************************
 * \brief
 *    update QP variables for interlaced pictures and basic unit coding
 *************************************************************************************
*/
void updateQPInterlaceBU( RCQuadratic *p_quad, RCGeneric *p_gen )
{
  /*previous choice is frame coding*/
  if(p_gen->FieldFrame==1)
  {
    p_quad->TotalQpforPPicture +=p_quad->FrameQPBuffer;
    p_quad->Pm_Qp=p_quad->FrameQPBuffer;
  }
  /*previous choice is field coding*/
  else
  {
    p_quad->TotalQpforPPicture +=p_quad->FieldQPBuffer;
    p_quad->Pm_Qp=p_quad->FieldQPBuffer;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    update QP with quadratic model
 *************************************************************************************
*/
void updateModelQPFrame( RCQuadratic *p_quad, int m_Bits )
{
  double dtmp, m_Qstep;

  dtmp = p_quad->CurrentFrameMAD * p_quad->m_X1 * p_quad->CurrentFrameMAD * p_quad->m_X1
    + 4 * p_quad->m_X2 * p_quad->CurrentFrameMAD * m_Bits;
  if ((p_quad->m_X2 == 0.0) || (dtmp < 0) || ((sqrt (dtmp) - p_quad->m_X1 * p_quad->CurrentFrameMAD) <= 0.0)) // fall back 1st order mode
    m_Qstep = (float) (p_quad->m_X1 * p_quad->CurrentFrameMAD / (double) m_Bits);
  else // 2nd order mode
    m_Qstep = (float) ((2 * p_quad->m_X2 * p_quad->CurrentFrameMAD) / (sqrt (dtmp) - p_quad->m_X1 * p_quad->CurrentFrameMAD));

  p_quad->m_Qc = Qstep2QP(m_Qstep, p_quad->bitdepth_qp_scale);
}

/*!
 *************************************************************************************
 * \brief
 *    rate control at the MB level
 *************************************************************************************
*/
int rc_handle_mb( Macroblock *currMB, int prev_mb )
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  Macroblock     *prevMB = currMB->PrevMB; 
  int mb_qp = p_Vid->qp;
  RCGeneric   *p_gen   = p_Vid->p_rc_gen;
  RCQuadratic *p_quad = p_Vid->p_rc_quad;

  if (prev_mb > -1)
  {
    if ( p_Inp->MbInterlace == ADAPTIVE_CODING && !p_Vid->bot_MB && currMB->mb_field )
      mb_qp = prevMB->qp;
  }

  // frame layer rate control
  if (p_Inp->basicunit != p_Vid->FrameSizeInMbs)
  {
    // each I or B frame has only one QP
    if ( ((p_Vid->type == I_SLICE || p_Vid->type == B_SLICE) && p_Inp->RCUpdateMode != RC_MODE_1 ) || !(p_Vid->number) )
    {
      return mb_qp;
    }
    else if ( p_Vid->type == P_SLICE || p_Inp->RCUpdateMode == RC_MODE_1 )
    {
      if (!p_Vid->write_macroblock) //write macroblock
      {
        if (prev_mb > -1) 
        {      
          if (!((p_Inp->MbInterlace) && p_Vid->bot_MB)) //top macroblock
          {
            if (prevMB->prev_cbp != 1)
            {
              mb_qp = prevMB->prev_qp;
            }
          }
        }
      }

      // compute the quantization parameter for each basic unit of P frame

      if (!p_Vid->write_macroblock)
      {
        if(!((p_Inp->MbInterlace) && p_Vid->bot_MB))
        {
          if(p_Inp->RCUpdateMode <= MAX_RC_MODE && (p_Vid->NumberofCodedMacroBlocks > 0) && (p_Vid->NumberofCodedMacroBlocks % p_Vid->BasicUnit == 0))
          {
            updateRCModel(p_Vid, p_Inp, p_quad, p_gen);
            // frame coding
            if(p_Vid->active_sps->frame_mbs_only_flag)
            {
              p_Vid->BasicUnitQP = p_Vid->updateQP(p_Vid, p_Inp, p_quad, p_gen, p_gen->TopFieldFlag) - p_quad->bitdepth_qp_scale;
            }
            // picture adaptive field/frame coding
            else if(p_Inp->MbInterlace || ((p_Inp->PicInterlace!=FRAME_CODING) && (p_gen->NoGranularFieldRC==0)))
            {
              p_Vid->BasicUnitQP = p_Vid->updateQP(p_Vid, p_Inp, p_quad, p_gen, p_gen->TopFieldFlag) - p_quad->bitdepth_qp_scale;
            }
          }

          if(p_Vid->current_mb_nr==0)
            p_Vid->BasicUnitQP = mb_qp;


          mb_qp = p_Vid->BasicUnitQP;
          mb_qp = iClip3(MIN_QP - p_Vid->bitdepth_luma_qp_scale, MAX_QP, mb_qp);
        }
      }
    }
  }
  return mb_qp;
}

/*!
 *************************************************************************************
 * \brief
 *    initialize rate control model for the top field
 *************************************************************************************
*/
void rc_init_top_field ( VideoParameters *p_Vid, InputParameters *p_Inp )
{
  RCGeneric   *p_gen   = p_Vid->p_rc_gen;
  RCQuadratic *p_quad = p_Vid->p_rc_quad;

  p_Vid->BasicUnit = p_Inp->basicunit;
  p_gen->TopFieldFlag = 1;
  p_Vid->rc_init_pict_ptr(p_Vid, p_Inp, p_quad, p_gen, 0, 1, (p_Inp->PicInterlace == FIELD_CODING), 1.0F); 
  p_Vid->p_curr_frm_struct->qp = p_Vid->qp = p_Vid->updateQP(p_Vid, p_Inp, p_quad, p_gen, 1) - p_quad->bitdepth_qp_scale;
}

/*!
 *************************************************************************************
 * \brief
 *    initialize rate control model for the bottom field
 *************************************************************************************
*/
void rc_init_bottom_field ( VideoParameters *p_Vid, InputParameters *p_Inp, int TopFieldBits )
{
  RCGeneric   *p_gen   = p_Vid->p_rc_gen;
  RCQuadratic *p_quad = p_Vid->p_rc_quad;

  p_quad->bits_topfield = TopFieldBits;
  p_gen->TopFieldFlag = 0;
  p_Vid->rc_init_pict_ptr(p_Vid, p_Inp, p_quad, p_gen, 0,0,0, 1.0F); 
  p_Vid->p_curr_frm_struct->qp = p_Vid->qp = p_Vid->updateQP(p_Vid, p_Inp, p_quad, p_gen, 0) - p_quad->bitdepth_qp_scale; 
}

/*!
 *************************************************************************************
 * \brief
 *    initialize rate control for RDPictureDecision
 *************************************************************************************
*/
void rc_init_frame_rdpic( VideoParameters *p_Vid, InputParameters *p_Inp, float rateRatio )
{
  RCGeneric *p_gen = p_Vid->p_rc_gen;
  RCGeneric *p_gen_init = p_Vid->p_rc_gen_init;
  RCQuadratic *p_quad = p_Vid->p_rc_quad;
  RCQuadratic *p_quad_init = p_Vid->p_rc_quad_init;

  switch (p_Inp->RCUpdateMode)
  {
  case RC_MODE_0:  case RC_MODE_1:  case RC_MODE_2:  case RC_MODE_3:
    // re-store the initial RC model
    rc_copy_quadratic( p_Vid, p_Inp, p_quad, p_quad_init );
    rc_copy_generic( p_Vid, p_gen, p_gen_init );
    p_Vid->rc_init_pict_ptr(p_Vid, p_Inp, p_quad, p_gen, 1, 0, 1, rateRatio );
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp = p_Vid->updateQP(p_Vid, p_Inp, p_quad, p_gen, 0) - p_quad->bitdepth_qp_scale;
    break;
  default:
    break;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    allocate rate control memory
 *************************************************************************************
*/
void rc_allocate_memory( VideoParameters *p_Vid, InputParameters *p_Inp )
{
  switch (p_Inp->RCUpdateMode)
  {
  case RC_MODE_0:
  case RC_MODE_1:
  case RC_MODE_2:
  case RC_MODE_3:
    rc_alloc_generic( p_Vid, &p_Vid->p_rc_gen );
    rc_alloc_quadratic( p_Vid, p_Inp, &p_Vid->p_rc_quad );

    if ( p_Inp->RDPictureDecision || p_Inp->MbInterlace == ADAPTIVE_CODING || p_Inp->PicInterlace == ADAPTIVE_CODING )
    {
      rc_alloc_generic( p_Vid, &p_Vid->p_rc_gen_init );
      rc_alloc_quadratic( p_Vid, p_Inp, &p_Vid->p_rc_quad_init );
      rc_alloc_generic( p_Vid, &p_Vid->p_rc_gen_best );
      rc_alloc_quadratic( p_Vid, p_Inp, &p_Vid->p_rc_quad_best );
    }
    break;
  default:
    break;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    free rate control memory
 *************************************************************************************
*/
void rc_free_memory( VideoParameters *p_Vid, InputParameters *p_Inp )
{
  switch (p_Inp->RCUpdateMode)
  {
  case RC_MODE_0:
  case RC_MODE_1:
  case RC_MODE_2:
  case RC_MODE_3:
    rc_free_generic( &p_Vid->p_rc_gen );
    rc_free_quadratic( &p_Vid->p_rc_quad );

    if ( p_Inp->RDPictureDecision || p_Inp->MbInterlace == ADAPTIVE_CODING || p_Inp->PicInterlace == ADAPTIVE_CODING )
    {
      rc_free_generic( &p_Vid->p_rc_gen_init );
      rc_free_quadratic( &p_Vid->p_rc_quad_init );
      rc_free_generic( &p_Vid->p_rc_gen_best );
      rc_free_quadratic( &p_Vid->p_rc_quad_best );
    }
    break;
  default:
    break;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    update coding statistics after MB coding
 *************************************************************************************
*/
void rc_update_mb_stats(Macroblock *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp; 
  BitCounter *mbBits = &currMB->bits;
  RCGeneric *p_gen = p_Vid->p_rc_gen;

  // Rate control
  p_Vid->NumberofMBHeaderBits = mbBits->mb_mode + mbBits->mb_inter
                            + mbBits->mb_cbp + mbBits->mb_delta_quant;
  p_Vid->NumberofMBTextureBits = mbBits->mb_y_coeff + mbBits->mb_uv_coeff;

  switch (p_Inp->RCUpdateMode)
  {
  case RC_MODE_0:  case RC_MODE_1:  case RC_MODE_2:  case RC_MODE_3:
    p_gen->NumberofTextureBits += p_Vid->NumberofMBTextureBits;
    p_gen->NumberofHeaderBits  += p_Vid->NumberofMBHeaderBits;
    // basic unit layer rate control
    if(p_Vid->BasicUnit < p_Vid->FrameSizeInMbs)
    {
      p_gen->NumberofBasicUnitHeaderBits  += p_Vid->NumberofMBHeaderBits;
      p_gen->NumberofBasicUnitTextureBits += p_Vid->NumberofMBTextureBits;
    }
    break;
  default:
    break;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    save state of rate control model
 *************************************************************************************
*/
void rc_save_state( VideoParameters *p_Vid, InputParameters *p_Inp )
{
  switch (p_Inp->RCUpdateMode)
  {
  case RC_MODE_0:  case RC_MODE_1:  case RC_MODE_2:  case RC_MODE_3:
    rc_copy_quadratic( p_Vid, p_Inp, p_Vid->p_rc_quad_best, p_Vid->p_rc_quad );
    rc_copy_generic( p_Vid, p_Vid->p_rc_gen_best, p_Vid->p_rc_gen );
    break;
  default:
    break;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    restore state of rate control model
 *************************************************************************************
*/
void rc_restore_state( VideoParameters *p_Vid, InputParameters *p_Inp )
{
  switch (p_Inp->RCUpdateMode)
  {
  case RC_MODE_0:  case RC_MODE_1:  case RC_MODE_2:  case RC_MODE_3:
    rc_copy_quadratic( p_Vid, p_Inp, p_Vid->p_rc_quad, p_Vid->p_rc_quad_best );
    rc_copy_generic( p_Vid, p_Vid->p_rc_gen, p_Vid->p_rc_gen_best );
    break;
  default:
    break;
  }
}
