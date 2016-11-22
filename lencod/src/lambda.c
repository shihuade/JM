/*!
***************************************************************************
*
* \file lambda.c
*
* \brief
*    Lambda computation related functions
*
* \date
*    13 September 2009
*
* \author
*    Alexis Michael Tourapis   alexismt@ieee.org
**************************************************************************/

#include "global.h"
#include "slice.h"

static void SetLambda(VideoParameters *p_Vid, int j, int qp, double lambda_scale)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int k;
  p_Vid->lambda_md[j][qp] *= lambda_scale;

  for (k = F_PEL; k <= Q_PEL; k++)
  {
    //p_Vid->lambda_me[j][qp][k] =  (p_Inp->MEErrorMetric[k] == ERROR_SSE) ? p_Vid->lambda_md[j][qp] : sqrt(p_Vid->lambda_md[j][qp]);
    //p_Vid->lambda_me[j][qp][k] =  sqrt(p_Vid->lambda_md[j][qp]);
    p_Vid->lambda_me[j][qp][k] =  (p_Inp->MEErrorMetric[k] == ERROR_SSE && p_Inp->MESoftenSSEMetric == 0) ? p_Vid->lambda_md[j][qp] : sqrt(p_Vid->lambda_md[j][qp]);
    p_Vid->lambda_mf[j][qp][k] = LAMBDA_FACTOR (p_Vid->lambda_me[j][qp][k]);
  }
}

static void CalcMaxLamdaMD(VideoParameters *p_Vid, double *p_lambda_md)
{
  double max_lambda_md;
  int iBits;

  if(p_Vid->p_Inp->ProfileIDC >= FREXT_HP)
    iBits=128;  //Spec. Page 306
  else
    iBits=80;

  if(p_Vid->yuv_format == YUV420)
    iBits += 3072; //(256+128)*8;
  else if(p_Vid->yuv_format == YUV422)
    iBits += 4096; //(256+256)*8;
  else if(p_Vid->yuv_format == YUV444)
    iBits += 6144; //(256*3)*8;
  else if(p_Vid->yuv_format == YUV400)
    iBits += 2048; //256*8;

  max_lambda_md =  floor(((double)DISTBLK_MAX)/iBits/(1<<LAMBDA_ACCURACY_BITS));
#if JCOST_OVERFLOWCHECK
  {
    distblk cost = weight_cost(LAMBDA_FACTOR(max_lambda_md), iBits);
    assert(cost>=0 && cost<=DISTBLK_MAX);
  }
#endif
  *p_lambda_md = max_lambda_md;  
}

static void ClipLambda(double *p_lambda_max, double *p_lambda)
{
  if(*p_lambda > *p_lambda_max)
  {
    //printf("Clip: %lf -> %lf\n", *p_lambda, *p_lambda_max);   
    *p_lambda = *p_lambda_max;
  }
}

void set_rdoq_lambda( Slice *currSlice )
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  int qp, j = currSlice->slice_type;

  for (qp = -currSlice->bitdepth_luma_qp_scale; qp < 52; qp++)
  {
    p_Vid->lambda_rdoq[j][qp] = p_Vid->lambda_md[j][qp];
  }
}

void get_implicit_lambda_p_slice(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int qp;
  double qp_temp;
  double lambda_md;
  double lambda_scale = p_Inp->DisableDistanceLambdaScale ? 1.0 : 1.0 - dClip3(0.0,0.5,0.05 * (double) p_Inp->jumpd);
  //limit lambda for mode decison;
  int bLimitsLambdaMD = ((p_Inp->EnableIPCM > 0) && (IMGTYPE==0));
  double dMaxLambdaMD =0;

  if(bLimitsLambdaMD)
    CalcMaxLamdaMD(p_Vid, &dMaxLambdaMD);

  for (qp = -currSlice->bitdepth_luma_qp_scale; qp < 52; qp++)
  {
    qp_temp = (double)qp + currSlice->bitdepth_luma_qp_scale - SHIFT_QP;

    if (p_Inp->NumberBFrames > 0)
    {
      lambda_md = 0.68 * pow (2.0, qp_temp/3.0);
    }
    else
      lambda_md = 0.85 * pow (2.0, qp_temp/3.0);

    // Scale lambda due to hadamard qpel only consideration
    lambda_md = ((p_Inp->MEErrorMetric[H_PEL] == ERROR_SATD && p_Inp->MEErrorMetric[Q_PEL] == ERROR_SATD) ? 1.00 : 0.95) * lambda_md;
    p_Vid->lambda_md[P_SLICE][qp] = lambda_scale * lambda_md;

    //clip lambda; 
    if(bLimitsLambdaMD)
      ClipLambda(&dMaxLambdaMD, &p_Vid->lambda_md[P_SLICE][qp]);

    SetLambda(p_Vid, P_SLICE, qp, 1.0);

    if (p_Inp->CtxAdptLagrangeMult == 1)
    {
      int lambda_qp = (qp >= 32 && !p_Inp->RCEnable) ? imax(0, qp - 4) : imax(0, qp - 6);
      p_Vid->lambda_mf_factor[P_SLICE][qp] = log (p_Vid->lambda_me[P_SLICE][lambda_qp][Q_PEL] + 1.0) / log (2.0);
    }
  } 
}

void get_implicit_lambda_b_slice(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int qp;
  double qp_temp;
  double lambda_md;
  FrameUnitStruct *p_cur_frm = p_Vid->p_curr_frm_struct;
  //limit lambda for mode decison;
  int bLimitsLambdaMD = ((p_Inp->EnableIPCM > 0) && (IMGTYPE==0));
  double dMaxLambdaMD =0;

  if(bLimitsLambdaMD)
    CalcMaxLamdaMD(p_Vid, &dMaxLambdaMD);

  for (qp = -currSlice->bitdepth_luma_qp_scale; qp < 52; qp++)
  {
    qp_temp = (double)qp + currSlice->bitdepth_luma_qp_scale - SHIFT_QP;

    if (p_Inp->NumberBFrames > 0)
    {
      lambda_md = 0.68 * pow (2.0, qp_temp/3.0)
        * (p_cur_frm->layer != 0 ? dClip3(2.00, 4.00, (qp_temp / 6.0)) : 1.0);
    }
    else
      lambda_md = 0.85 * pow (2.0, qp_temp/3.0);

    // Scale lambda due to hadamard qpel only consideration
    lambda_md = ((p_Inp->MEErrorMetric[H_PEL] == ERROR_SATD && p_Inp->MEErrorMetric[Q_PEL] == ERROR_SATD) ? 1.00 : 0.95) * lambda_md;

    if (p_cur_frm->layer != 0 && currSlice->nal_reference_idc)
    {
      if (p_Inp->HierarchicalCoding == 2)
        lambda_md *= (1.0 - dmin(0.4, 0.2 * (double) (p_cur_frm->p_atom->gop_levels - p_cur_frm->layer)));
      else
        lambda_md *= 0.80;
    }
    p_Vid->lambda_md[B_SLICE][qp] = lambda_md;

    //clip lambda; 
    if(bLimitsLambdaMD)
      ClipLambda(&dMaxLambdaMD, &p_Vid->lambda_md[B_SLICE][qp]);

    SetLambda(p_Vid, B_SLICE, qp, 1.0);

    if (p_Inp->CtxAdptLagrangeMult == 1)
    {
      int lambda_qp = (qp >= 32 && !p_Inp->RCEnable) ? imax(0, qp - 4) : imax(0, qp - 6);
      p_Vid->lambda_mf_factor[B_SLICE][qp] = log (p_Vid->lambda_me[B_SLICE][lambda_qp][Q_PEL] + 1.0) / log (2.0);
    }
  }
}

void get_implicit_lambda_i_slice(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int qp;
  double qp_temp;
  double lambda_md;
  double lambda_scale = p_Inp->DisableDistanceLambdaScale ? 1.0 : 1.0 - dClip3(0.0,0.5,0.05 * (double) p_Inp->jumpd);
  //limit lambda for mode decison;
  int bLimitsLambdaMD = ((p_Inp->EnableIPCM > 0) && (IMGTYPE==0));
  double dMaxLambdaMD =0;

  if(bLimitsLambdaMD)
    CalcMaxLamdaMD(p_Vid, &dMaxLambdaMD);

  for (qp = -currSlice->bitdepth_luma_qp_scale; qp < 52; qp++)
  {
    qp_temp = (double)qp + currSlice->bitdepth_luma_qp_scale - SHIFT_QP;

    if(p_Inp->UseRDOQuant && p_Vid->qp==qp)
      lambda_md = 0.57 * pow (2.0, qp_temp/3.0); 
    else  if (p_Inp->NumberBFrames > 0)
    {
      lambda_md = 0.68 * pow (2.0, qp_temp/3.0);
    }
    else
      lambda_md = 0.85 * pow (2.0, qp_temp/3.0);

    // Scale lambda due to hadamard qpel only consideration
    lambda_md = ((p_Inp->MEErrorMetric[H_PEL] == ERROR_SATD && p_Inp->MEErrorMetric[Q_PEL] == ERROR_SATD) ? 1.00 : 0.95) * lambda_md;
    p_Vid->lambda_md[I_SLICE][qp] = lambda_scale * lambda_md;

    //clip lambda; 
    if(bLimitsLambdaMD)
      ClipLambda(&dMaxLambdaMD, &p_Vid->lambda_md[I_SLICE][qp]);

    SetLambda(p_Vid, I_SLICE, qp, 1.0);

    if (p_Inp->CtxAdptLagrangeMult == 1)
    {
      int lambda_qp = (qp >= 32 && !p_Inp->RCEnable) ? imax(0, qp - 4) : imax(0, qp - 6);
      p_Vid->lambda_mf_factor[I_SLICE][qp] = log (p_Vid->lambda_me[I_SLICE][lambda_qp][Q_PEL] + 1.0) / log (2.0);
    }
  }
}

void get_implicit_lambda_sp_slice(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int qp;
  double qp_temp;
  double lambda_md;
  double lambda_scale = p_Inp->DisableDistanceLambdaScale ? 1.0 : 1.0 - dClip3(0.0,0.5,0.05 * (double) p_Inp->jumpd);
  //limit lambda for mode decison;
  int bLimitsLambdaMD = ((p_Inp->EnableIPCM > 0) && (IMGTYPE==0));
  double dMaxLambdaMD =0;

  int slice_index = currSlice->slice_type;

  if(bLimitsLambdaMD)
    CalcMaxLamdaMD(p_Vid, &dMaxLambdaMD);

  for (qp = -currSlice->bitdepth_luma_qp_scale; qp < 52; qp++)
  {
    qp_temp = (double)qp + currSlice->bitdepth_luma_qp_scale - SHIFT_QP;

    if(p_Inp->UseRDOQuant && p_Vid->type == I_SLICE && p_Vid->qp==qp)
      lambda_md = 0.57 * pow (2.0, qp_temp/3.0); 
    else  if (p_Inp->NumberBFrames > 0)
    {
      lambda_md = 0.68 * pow (2.0, qp_temp/3.0) * dClip3(1.4,3.0,(qp_temp / 12.0));
    }
    else
      lambda_md = 0.85 * pow (2.0, qp_temp/3.0) * dClip3(1.4, 3.0,(qp_temp / 12.0));

    // Scale lambda due to hadamard qpel only consideration
    lambda_md = ((p_Inp->MEErrorMetric[H_PEL] == ERROR_SATD && p_Inp->MEErrorMetric[Q_PEL] == ERROR_SATD) ? 1.00 : 0.95) * lambda_md;

    p_Vid->lambda_md[slice_index][qp] = lambda_scale * lambda_md;

    //clip lambda; 
    if(bLimitsLambdaMD)
      ClipLambda(&dMaxLambdaMD, &p_Vid->lambda_md[slice_index][qp]);

    SetLambda(p_Vid, slice_index, qp, 1.0);

    if (p_Inp->CtxAdptLagrangeMult == 1)
    {
      int lambda_qp = (qp >= 32 && !p_Inp->RCEnable) ? imax(0, qp - 4) : imax(0, qp - 6);
      p_Vid->lambda_mf_factor[slice_index][qp] = log (p_Vid->lambda_me[slice_index][lambda_qp][Q_PEL] + 1.0) / log (2.0);
    }
  }
}

void get_explicit_lambda(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int qp;
  double qp_temp;
  double lambda_scale = p_Inp->DisableDistanceLambdaScale ? 1.0 : 1.0 - dClip3(0.0,0.5,0.05 * (double) p_Inp->jumpd);
  //limit lambda for mode decison;
  int bLimitsLambdaMD = ((p_Inp->EnableIPCM > 0) && (IMGTYPE==0));
  double dMaxLambdaMD =0;
#if (MVC_EXTENSION_ENABLE)
  double tmp_lambda_md = 0.0;
#endif

  int j = currSlice->slice_type;

  if(bLimitsLambdaMD)
    CalcMaxLamdaMD(p_Vid, &dMaxLambdaMD);

  for (qp = -currSlice->bitdepth_luma_qp_scale; qp < 52; qp++)
  {
    qp_temp = (double)qp + currSlice->bitdepth_luma_qp_scale - SHIFT_QP;

    if (j == B_SLICE && currSlice->nal_reference_idc)
      p_Vid->lambda_md[j][qp] = p_Inp->LambdaWeight[5] * pow (2, qp_temp/3.0);
    else
      p_Vid->lambda_md[j][qp] = p_Inp->LambdaWeight[j] * pow (2, qp_temp/3.0);

#if (MVC_EXTENSION_ENABLE)
    tmp_lambda_md = p_Vid->lambda_md[j][qp];

    if (currSlice->view_id)
    {
      p_Vid->lambda_md[j][qp] = p_Vid->lambda_md[j][qp] * p_Inp->enh_layer_me_lambda_multiplier;
    }
#endif

    //clip lambda; 
    if(bLimitsLambdaMD)
      ClipLambda(&dMaxLambdaMD, &p_Vid->lambda_md[j][qp]);

    SetLambda(p_Vid, j, qp, ((p_Inp->MEErrorMetric[H_PEL] == ERROR_SATD && p_Inp->MEErrorMetric[Q_PEL] == ERROR_SATD) ? 1.00 : 0.95));

#if (MVC_EXTENSION_ENABLE)
    if (currSlice->view_id)
    {
      p_Vid->lambda_md[j][qp] = tmp_lambda_md * p_Inp->enh_layer_lambda_multiplier;
      //clip lambda; 
      if(bLimitsLambdaMD)
        ClipLambda(&dMaxLambdaMD, &p_Vid->lambda_md[j][qp]);
      p_Vid->lambda_md[j][qp] *= ((p_Inp->MEErrorMetric[H_PEL] == ERROR_SATD && p_Inp->MEErrorMetric[Q_PEL] == ERROR_SATD) ? 1.00 : 0.95);
    }
#endif
  }

  if (j != B_SLICE)
  {
    for (qp = -currSlice->bitdepth_luma_qp_scale; qp < 52; qp++)
      p_Vid->lambda_md[j][qp] *= lambda_scale;
  }
}


void get_fixed_lambda(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int qp;
  //limit lambda for mode decison;
  int bLimitsLambdaMD = ((p_Inp->EnableIPCM > 0) && (IMGTYPE==0));
  double dMaxLambdaMD =0;

  int j = currSlice->slice_type;

  if(bLimitsLambdaMD)
    CalcMaxLamdaMD(p_Vid, &dMaxLambdaMD);

  for (qp = -currSlice->bitdepth_luma_qp_scale; qp < 52; qp++)
  {
    if (j == B_SLICE && currSlice->nal_reference_idc)
      p_Vid->lambda_md[j][qp] = p_Inp->FixedLambda[5];
    else
    p_Vid->lambda_md[j][qp] = p_Inp->FixedLambda[j];
    //clip lambda; 
    if(bLimitsLambdaMD)
      ClipLambda(&dMaxLambdaMD, &p_Vid->lambda_md[j][qp]);

    SetLambda(p_Vid, j, qp, ((p_Inp->MEErrorMetric[H_PEL] == ERROR_SATD && p_Inp->MEErrorMetric[Q_PEL] == ERROR_SATD) ? 1.00 : 0.95));
  }
}

