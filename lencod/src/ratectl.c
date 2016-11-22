
/*!
 ***************************************************************************
 * \file ratectl.c
 *
 * \brief
 *    Rate Control algorithm
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Siwei Ma <swma@jdl.ac.cn>
 *     - Zhengguo LI<ezgli@lit.a-star.edu.sg>
 *
 * \date
 *   16 Jan. 2003
 **************************************************************************
 */

#include <math.h>
#include <limits.h>

#include "global.h"
#include "ratectl.h"


/*!
 *************************************************************************************
 * \brief
 *    Update Rate Control Parameters
 *************************************************************************************
 */
void rc_store_mad(Macroblock *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  RCGeneric *p_gen = p_Vid->p_rc_gen;

  p_gen->MADofMB[currMB->mbAddrX] = ComputeMBMAD(currMB->p_Slice->diffy);

  if(p_Inp->basicunit < p_Vid->FrameSizeInMbs)
  {
    p_gen->TotalMADBasicUnit += p_gen->MADofMB[currMB->mbAddrX];
  }  
}

/*!
 *************************************************************************************
 * \brief
 *    map QP to Qstep
 *
 *************************************************************************************
*/
double QP2Qstep( int QP )
{
  int i;
  double Qstep;
  static const double QP2QSTEP[6] = { 0.625, 0.6875, 0.8125, 0.875, 1.0, 1.125 };

  Qstep = QP2QSTEP[QP % 6];
  for( i=0; i<(QP/6); i++)
    Qstep *= 2;

  return Qstep;
}


/*!
 *************************************************************************************
 * \brief
 *    map Qstep to QP
 *
 *************************************************************************************
*/
int Qstep2QP( double Qstep, int qp_offset )
{
  int q_per = 0, q_rem = 0;

  if( Qstep < QP2Qstep(MIN_QP))
    return MIN_QP;
  else if (Qstep > QP2Qstep(MAX_QP + qp_offset) )
    return (MAX_QP + qp_offset);

  while( Qstep > QP2Qstep(5) )
  {
    Qstep /= 2.0;
    q_per++;
  }

  if (Qstep <= 0.65625)
  {
    //Qstep = 0.625;
    q_rem = 0;
  }
  else if (Qstep <= 0.75)
  {
    //Qstep = 0.6875;
    q_rem = 1;
  }
  else if (Qstep <= 0.84375)
  {
    //Qstep = 0.8125;
    q_rem = 2;
  }
  else if (Qstep <= 0.9375)
  {
    //Qstep = 0.875;
    q_rem = 3;
  }
  else if (Qstep <= 1.0625)
  {
    //Qstep = 1.0;
    q_rem = 4;
  }
  else
  {
    //Qstep = 1.125;
    q_rem = 5;
  }

  return (q_per * 6 + q_rem);
}

/*!
 ************************************************************************************
 * \brief
 *    calculate MAD for the current macroblock
 *
 * \return
 *    calculated MAD
 *
 *************************************************************************************
*/
int ComputeMBMAD(int diffy[16][16])
{
  int k, l, sum = 0;

  for (k = 0; k < 16; k++)
    for (l = 0; l < 16; l++)
      sum += iabs(diffy[k][l]);

  return sum;
}

/*!
 *************************************************************************************
 * \brief
 *    Compute Frame MAD
 *
 *************************************************************************************
*/
double ComputeFrameMAD(VideoParameters *p_Vid)
{
  int64 TotalMAD = 0;
  unsigned int i;
  for(i = 0; i < p_Vid->FrameSizeInMbs; i++)
    TotalMAD += p_Vid->p_rc_gen->MADofMB[i];
  return (double)TotalMAD / (256.0 * (double)p_Vid->FrameSizeInMbs);
}


/*!
 *************************************************************************************
 * \brief
 *    Copy JVT rate control objects
 *
 *************************************************************************************
*/
void rc_copy_generic( VideoParameters *p_Vid, RCGeneric *dst, RCGeneric *src )
{
  /* buffer original addresses for which memory has been allocated */
  int *tmpMADofMB = dst->MADofMB;

  /* copy object */

  // This could be written as: *dst = *src;
  memcpy( (void *)dst, (void *)src, sizeof(RCGeneric) );

  /* restore original addresses */
  dst->MADofMB = tmpMADofMB;

  /* copy MADs */
  memcpy( (void *)dst->MADofMB, (void *)src->MADofMB, p_Vid->FrameSizeInMbs * sizeof (int) );
}

/*!
 *************************************************************************************
 * \brief
 *    Dynamically allocate memory needed for generic rate control
 *
 *************************************************************************************
 */
void rc_alloc_generic( VideoParameters *p_Vid, RCGeneric **p_quad )
{
  *p_quad = (RCGeneric *) malloc ( sizeof( RCGeneric ) );
  if (NULL == *p_quad)
  {
    no_mem_exit("rc_alloc_generic: rc_alloc_generic");
  }
  (*p_quad)->MADofMB = (int *) calloc (p_Vid->FrameSizeInMbs, sizeof (int));
  if (NULL == (*p_quad)->MADofMB)
  {
    no_mem_exit("rc_alloc_generic: (*p_quad)->MADofMB");
  }
  (*p_quad)->FieldFrame = 1;
}


/*!
 *************************************************************************************
 * \brief
 *    Free memory needed for generic rate control
 *
 *************************************************************************************
 */
void rc_free_generic(RCGeneric **p_quad)
{
  if (NULL!=(*p_quad)->MADofMB)
  {
    free ((*p_quad)->MADofMB);
    (*p_quad)->MADofMB = NULL;
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
 *    Initialize GOP Level Rate Control parameters
 *
 *************************************************************************************
 */
void rc_init_gop_params(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int np, nb;
  RCQuadratic *p_quad = p_Vid->p_rc_quad;
  RCGeneric *p_gen = p_Vid->p_rc_gen;

  switch( p_Inp->RCUpdateMode )
  {
  case RC_MODE_1:
  case RC_MODE_3: 
    if ( !(p_Vid->curr_frm_idx) )
    {
      /* number of P frames */
      np = (int) (p_Inp->no_frames + p_Inp->NumberBFrames) / (1 + p_Inp->NumberBFrames) - 1 + 1; // approximate but good enough (hack...)
      /* number of B frames */
      nb = np * p_Inp->NumberBFrames;

      rc_init_GOP(p_Vid, p_Inp, p_quad, p_gen, np, nb);
    }
    break;
  case RC_MODE_0:
  case RC_MODE_2:
    if (p_Inp->idr_period == 0)
    {
      if ( !(p_Vid->curr_frm_idx) )
      {
        /* number of P frames */
        np = (int) (p_Inp->no_frames + p_Inp->NumberBFrames) / (1 + p_Inp->NumberBFrames) - 1 + 1; // approximate but good enough (hack...)
        /* number of B frames */
        nb = np * p_Inp->NumberBFrames;
        rc_init_GOP(p_Vid, p_Inp, p_quad, p_gen, np, nb);
      }
    }
    else if ( p_Vid->p_curr_frm_struct->idr_flag )  
    {
      int M = p_Inp->NumberBFrames + 1;
      int n = p_Inp->idr_period;

      /* last GOP may contain less frames */
      if ((p_Vid->curr_frm_idx / p_Inp->idr_period) >= (p_Inp->no_frames / p_Inp->idr_period))
      {
        n = p_Inp->no_frames - p_Vid->curr_frm_idx;
      }

      /* number of P frames */
      np = (p_Vid->curr_frm_idx == 0) ? 1 + ((n - 2) / M) : (n - 1) / M; 
      /* number of B frames */
      nb = n - np - 1;
      rc_init_GOP(p_Vid, p_Inp, p_quad, p_gen, np, nb);
    }
    break;
  default:
    break;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Initialize Frame Level Rate Control parameters
 *
 *************************************************************************************
 */

void rc_init_frame(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  switch( p_Inp->RCUpdateMode )
  {
  case RC_MODE_0:  case RC_MODE_1:  case RC_MODE_2:  case RC_MODE_3:

  // update the number of MBs in the basic unit for MBAFF coding
  if( (p_Inp->MbInterlace) && (p_Inp->basicunit < p_Vid->FrameSizeInMbs) && (p_Vid->type == P_SLICE || (p_Inp->RCUpdateMode == RC_MODE_1 && p_Vid->number) ) )
    p_Vid->BasicUnit = p_Inp->basicunit << 1;
  else
    p_Vid->BasicUnit = p_Inp->basicunit;

    if ( p_Inp->RDPictureDecision )
    {    
      rc_copy_quadratic( p_Vid, p_Inp, p_Vid->p_rc_quad_init, p_Vid->p_rc_quad ); // store rate allocation quadratic...    
      rc_copy_generic( p_Vid, p_Vid->p_rc_gen_init, p_Vid->p_rc_gen ); // ...and generic model
    }
    p_Vid->rc_init_pict_ptr(p_Vid, p_Inp, p_Vid->p_rc_quad, p_Vid->p_rc_gen, 1,0,1, 1.0F);

    if( p_Vid->active_sps->frame_mbs_only_flag)
      p_Vid->p_rc_gen->TopFieldFlag=0;

    p_Vid->p_curr_frm_struct->qp = p_Vid->qp = p_Vid->updateQP(p_Vid, p_Inp, p_Vid->p_rc_quad, p_Vid->p_rc_gen, 0) - p_Vid->p_rc_quad->bitdepth_qp_scale;
    break;
  default:
    break;
  }
}

/*!
 *************************************************************************************
 * \brief
 *    Initialize Sequence Level Rate Control parameters
 *
 *************************************************************************************
 */

void rc_init_sequence(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  switch( p_Inp->RCUpdateMode )
  {
  case RC_MODE_0:  case RC_MODE_1:  case RC_MODE_2:  case RC_MODE_3:
    rc_init_seq(p_Vid, p_Inp, p_Vid->p_rc_quad, p_Vid->p_rc_gen);
    break;
  default:
    break;
  }
}

void rc_store_slice_header_bits( VideoParameters *p_Vid, InputParameters *p_Inp, int len )
{
  switch (p_Inp->RCUpdateMode)
  {
  case RC_MODE_0:  case RC_MODE_1:  case RC_MODE_2:  case RC_MODE_3:
    p_Vid->p_rc_gen->NumberofHeaderBits +=len;

    // basic unit layer rate control
    if(p_Vid->BasicUnit < p_Vid->FrameSizeInMbs)
      p_Vid->p_rc_gen->NumberofBasicUnitHeaderBits +=len;
    break;
  default:
    break;
  }
}


/*!
*************************************************************************************
* \brief
*    Update Rate Control Difference
*************************************************************************************
*/
void rc_store_diff(int diffy[16][16], imgpel **p_curImg, int cpix_x,imgpel **prediction)
{
  int i, j;
  int *iDst;
  imgpel *Src1, *Src2;  

  for(j = 0; j < MB_BLOCK_SIZE; j++)
  {
    iDst = diffy[j];
    Src1 = &p_curImg[j][cpix_x];
    Src2 = prediction[j];
    for (i = 0; i < MB_BLOCK_SIZE; i++)
    {
      iDst[i] = Src1[i] - Src2[i];
    }
  }
}

void rc_store_diff_16b(int diffy[16][16], imgpel **p_curImg, int cpix_x,imgpel **prediction)
{
  int i, j;
  int *iDst;
  uint16 *Src1, *Src2;  

  for(j = 0; j < MB_BLOCK_SIZE; j++)
  {
    iDst = diffy[j];
    Src1 = (uint16*)(p_curImg[j]) +cpix_x;
    Src2 = (uint16*)(prediction[j]);
    for (i = 0; i < MB_BLOCK_SIZE; i++)
    {
      iDst[i] = Src1[i] - Src2[i];
    }
  }
}
