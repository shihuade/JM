
/*!
 *************************************************************************************
 * \file context_ini.c
 *
 * \brief
 *    CABAC context initializations
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Detlev Marpe
 *    - Heiko Schwarz
 **************************************************************************************
 */

#define CONTEXT_INI_C

#include <math.h>

#include "global.h"

#include "ctx_tables.h"
#include "biariencode.h"
#include "memalloc.h"

#define DEFAULT_CTX_MODEL   0
#define RELIABLE_COUNT      32.0
#define FRAME_TYPES         4
#define FIXED               0

// These essentially are constants
static const double probabilities[64] =
{
  0.500000, 0.474609, 0.450507, 0.427629,    0.405912, 0.385299, 0.365732, 0.347159,
  0.329530, 0.312795, 0.296911, 0.281833,    0.267520, 0.253935, 0.241039, 0.228799,
  0.217180, 0.206151, 0.195682, 0.185744,    0.176312, 0.167358, 0.158859, 0.150792,
  0.143134, 0.135866, 0.128966, 0.122417,    0.116200, 0.110299, 0.104698, 0.099381,
  0.094334, 0.089543, 0.084996, 0.080680,    0.076583, 0.072694, 0.069002, 0.065498,
  0.062172, 0.059014, 0.056018, 0.053173,    0.050473, 0.047909, 0.045476, 0.043167,
  0.040975, 0.038894, 0.036919, 0.035044,    0.033264, 0.031575, 0.029972, 0.028450,
  0.027005, 0.025633, 0.024332, 0.023096,    0.021923, 0.020810, 0.019753, 0.018750
};

void create_context_memory (VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int k;
  int num_mb  = p_Vid->FrameSizeInMbs; // number of macroblocks for frame
  double log2 = log(2.0);

  p_Vid->num_mb_per_slice  = (p_Inp->slice_mode == 1 ? p_Inp->slice_argument : num_mb);
  p_Vid->number_of_slices  = (num_mb + p_Vid->num_mb_per_slice - 1) / p_Vid->num_mb_per_slice;
  get_mem3Dint(&p_Vid->initialized, 3, FRAME_TYPES, p_Vid->number_of_slices);
  //===== set all context sets as "uninitialized" =====
  memset(&p_Vid->initialized[0][0][0], 0, 3 * FRAME_TYPES * p_Vid->number_of_slices * sizeof(int));
  get_mem3Dint(&p_Vid->modelNumber, 3, FRAME_TYPES, p_Vid->number_of_slices);

  //----- init tables -----
  for( k=0; k<64; k++ )
  {
    p_Vid->probability[k + 64] = probabilities[k];
    p_Vid->probability[k] = 1.0 - probabilities[63 - k];
    p_Vid->entropy    [k] = log( p_Vid->probability[      k] ) / log2;
    p_Vid->entropy[127-k] = log( probabilities[63 - k] ) / log2;
    p_Vid->enorm      [k] = p_Vid->entropy[k] - p_Vid->entropy[127-k];
    p_Vid->enorm[127 - k] = -p_Vid->enorm[k];
  }
}

void free_context_memory (VideoParameters *p_Vid)
{
  free_mem3Dint(p_Vid->initialized);
  free_mem3Dint(p_Vid->modelNumber);
}

#define BIARI_CTX_INIT2(qp, ii,jj,ctx,tab) \
{ \
  for (i=0; i<ii; i++) \
  for (j=0; j<jj; j++) \
  { \
    biari_init_context (qp, &(ctx[i][j]), &(tab[i][j][0])); \
  } \
}


static inline void binary_context_init1(int qp, int jj, BiContextType *ctx, const char table[][2])
{ 
  int j;
  for (j=0; j<jj; j++)
  {
    biari_init_context (qp, &(ctx[j]), &(table[j][0]));
  }
}

static inline void binary_context_init2(int qp, int ii, int jj, BiContextType ctx[][11], const char table[][11][2])
{ 
  int i, j;
  for (i = 0; i < ii; i++)
  {
    for (j = 0; j < jj; j++)
    {
      biari_init_context (qp, &(ctx[i][j]), &(table[i][j][0]));      
    }
  }
}

void SetCtxModelNumber (Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;

  int frame_field = p_Vid->field_picture;
  int img_type    = currSlice->slice_type;
  int ctx_number  = currSlice->start_mb_nr / p_Vid->num_mb_per_slice;

  if(img_type == I_SLICE)
  {
    currSlice->model_number=DEFAULT_CTX_MODEL;
    return;
  }

  if(p_Inp->context_init_method==FIXED)
  {
    currSlice->model_number = p_Inp->model_number;
    return;
  }

  if (p_Vid->initialized [frame_field][img_type][ctx_number])
  {
    currSlice->model_number = p_Vid->modelNumber[frame_field][img_type][ctx_number];
  }
  else if (ctx_number && p_Vid->initialized[frame_field][img_type][ctx_number-1])
  {
    currSlice->model_number = p_Vid->modelNumber[frame_field][img_type][ctx_number-1];
  }
  else
  {
    currSlice->model_number = DEFAULT_CTX_MODEL;
  }
}

void init_contexts (Slice *currSlice)
{
  MotionInfoContexts*  mc = currSlice->mot_ctx;
  TextureInfoContexts* tc = currSlice->tex_ctx;
  int model_number = currSlice->model_number;
  int qp = imax(0, currSlice->qp);
  int i, j;

  if ((currSlice->slice_type == I_SLICE) || (currSlice->slice_type == SI_SLICE))
  {
    //--- motion coding contexts ---
    BIARI_CTX_INIT2 (qp, 3, NUM_MB_TYPE_CTX,   mc->mb_type_contexts,     INIT_MB_TYPE_I[model_number]);

    //--- texture coding contexts ---
    binary_context_init1 (qp,  NUM_TRANSFORM_SIZE_CTX,  tc->transform_size_contexts,    INIT_TRANSFORM_SIZE_I[model_number][0]);
    binary_context_init1 (qp,             NUM_IPR_CTX,  tc->ipr_contexts,     INIT_IPR_I[model_number][0]);
    binary_context_init1 (qp,             NUM_CIPR_CTX, tc->cipr_contexts,    INIT_CIPR_I[model_number][0]);
    BIARI_CTX_INIT2 (qp, 3,               NUM_CBP_CTX,  tc->cbp_contexts,     INIT_CBP_I[model_number]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_BCBP_CTX, tc->bcbp_contexts,    INIT_BCBP_I[model_number]);
    binary_context_init1 (qp, NUM_DELTA_QP_CTX,         tc->delta_qp_contexts,INIT_DELTA_QP_I[model_number][0]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_MAP_CTX,  tc->map_contexts[0],  INIT_MAP_I[model_number]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_LAST_CTX, tc->last_contexts[0], INIT_LAST_I[model_number]);  
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_ONE_CTX,  tc->one_contexts,     INIT_ONE_I[model_number]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_ABS_CTX,  tc->abs_contexts,     INIT_ABS_I[model_number]);
#if ENABLE_FIELD_CTX
    binary_context_init1 (qp, NUM_MB_AFF_CTX,           tc->mb_aff_contexts,  INIT_MB_AFF_I[model_number][0]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_MAP_CTX,  tc->map_contexts[1],  INIT_FLD_MAP_I[model_number]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_LAST_CTX, tc->last_contexts[1], INIT_FLD_LAST_I[model_number]);
#endif
  }
  else
  {
    //--- motion coding contexts ---
    BIARI_CTX_INIT2 (qp, 3, NUM_MB_TYPE_CTX,   mc->mb_type_contexts,     INIT_MB_TYPE_P[model_number]);
    BIARI_CTX_INIT2 (qp, 2, NUM_B8_TYPE_CTX,   mc->b8_type_contexts,     INIT_B8_TYPE_P[model_number]);
    BIARI_CTX_INIT2 (qp, 2, NUM_MV_RES_CTX,    mc->mv_res_contexts,      INIT_MV_RES_P[model_number]);
    BIARI_CTX_INIT2 (qp, 2, NUM_REF_NO_CTX,    mc->ref_no_contexts,      INIT_REF_NO_P[model_number]);

    //--- texture coding contexts ---
    binary_context_init1(qp, NUM_TRANSFORM_SIZE_CTX, tc->transform_size_contexts, INIT_TRANSFORM_SIZE_P[model_number][0]);
    binary_context_init1(qp, NUM_IPR_CTX,            tc->ipr_contexts,            INIT_IPR_P[model_number][0]);
    binary_context_init1(qp, NUM_CIPR_CTX,           tc->cipr_contexts,           INIT_CIPR_P[model_number][0]);
    BIARI_CTX_INIT2 (qp, 3,               NUM_CBP_CTX,  tc->cbp_contexts,     INIT_CBP_P[model_number]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_BCBP_CTX, tc->bcbp_contexts,    INIT_BCBP_P[model_number]);
    binary_context_init1 (qp, NUM_DELTA_QP_CTX,              tc->delta_qp_contexts,INIT_DELTA_QP_P[model_number][0]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_MAP_CTX,  tc->map_contexts[0],  INIT_MAP_P[model_number]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_LAST_CTX, tc->last_contexts[0], INIT_LAST_P[model_number]);  
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_ONE_CTX,  tc->one_contexts,     INIT_ONE_P[model_number]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_ABS_CTX,  tc->abs_contexts,     INIT_ABS_P[model_number]);
#if ENABLE_FIELD_CTX
    binary_context_init1 (qp, NUM_MB_AFF_CTX,                tc->mb_aff_contexts,  INIT_MB_AFF_P[model_number][0]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_MAP_CTX,  tc->map_contexts[1],  INIT_FLD_MAP_P[model_number]);
    BIARI_CTX_INIT2 (qp, NUM_BLOCK_TYPES, NUM_LAST_CTX, tc->last_contexts[1], INIT_FLD_LAST_P[model_number]);
#endif
  }
}


double XRate (VideoParameters *p_Vid, BiContextTypePtr ctx, const char* model)
{
  int     ctx_state, mod_state;
  double  weight, xr = 0.0;
  int     qp = imax(0, p_Vid->qp);

  weight    = dmin (1.0, (double)ctx->count/(double)RELIABLE_COUNT);

  mod_state = ((model[0]*qp)>>4)+model[1];
  mod_state = iClip3(0, 127, mod_state);
  ctx_state = (ctx->MPS ? 64 + ctx->state : 63 - ctx->state);

  xr -= weight * (p_Vid->probability[ctx_state] * p_Vid->enorm[mod_state] + p_Vid->entropy[127 - mod_state]);

  return xr;
}

/*
static inline void add_xrate2(VideoParameters *p_Vid, int ii, int jj, BiContextType **ctx, const char ***tab, double *xr)
{ 
  int i, j;
  for (i=0; i<ii; i++) 
  for (j=0; j<jj; j++) 
  { 
    *xr += XRate (p_Vid, &(ctx[i][j]), &(tab[i][j][0]));
  } 
}
*/

#define ADD_XRATE2(p_Vid, ii,jj,ctx,tab) \
{ \
  for (i=0; i<ii; i++) \
  for (j=0; j<jj; j++) \
  { \
    xr += XRate (p_Vid, &(ctx[i][j]), &(tab[i][j][0])); \
  } \
}

#define ADD_XRATE1(p_Vid, jj,ctx,tab) \
{ \
  for (j=0; j<jj; j++) \
  { \
    xr += XRate (p_Vid, &(ctx[j]), &(tab[j][0])); \
  } \
}

void GetCtxModelNumber (Slice *currSlice, int* mnumber, MotionInfoContexts* mc, TextureInfoContexts* tc)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  int     model, j, i;
  double  xr, min_xr = 1e30;

  if (currSlice->slice_type == I_SLICE)
  {
    for (model = 0; model < NUM_CTX_MODELS_I; model++)
    {
      xr = 0.0;
      //--- motion coding contexts ---
      ADD_XRATE2 (p_Vid, 3, NUM_MB_TYPE_CTX,   mc->mb_type_contexts,     INIT_MB_TYPE_I [model]);

      //--- texture coding contexts ---
      ADD_XRATE1 (p_Vid,    NUM_TRANSFORM_SIZE_CTX,  tc->transform_size_contexts, INIT_TRANSFORM_SIZE_I[model][0]);
      ADD_XRATE1 (p_Vid,                   NUM_IPR_CTX,  tc->ipr_contexts,       INIT_IPR_I [model][0]);
      ADD_XRATE1 (p_Vid,                   NUM_CIPR_CTX, tc->cipr_contexts,      INIT_CIPR_I[model][0]);
      ADD_XRATE2 (p_Vid, 3,                NUM_CBP_CTX,  tc->cbp_contexts,       INIT_CBP_I [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_BCBP_CTX, tc->bcbp_contexts,      INIT_BCBP_I[model]);
      ADD_XRATE1 (p_Vid, NUM_DELTA_QP_CTX,               tc->delta_qp_contexts,    INIT_DELTA_QP_I[model][0]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_MAP_CTX,  tc->map_contexts[0],    INIT_MAP_I [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_LAST_CTX, tc->last_contexts[0],   INIT_LAST_I[model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_ONE_CTX,  tc->one_contexts,       INIT_ONE_I     [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_ABS_CTX,  tc->abs_contexts,       INIT_ABS_I     [model]);
#if ENABLE_FIELD_CTX
      ADD_XRATE1 (p_Vid, NUM_MB_AFF_CTX,                 tc->mb_aff_contexts,    INIT_MB_AFF_I  [model][0]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_MAP_CTX,  tc->map_contexts[1],    INIT_FLD_MAP_I [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_LAST_CTX, tc->last_contexts[1],   INIT_FLD_LAST_I[model]);
#endif

      if (xr < min_xr)
      {
        min_xr    = xr;
        *mnumber  = model;
      }
    }
  }
  else
  {
    for (model=0; model<NUM_CTX_MODELS_P; model++)
    {
      xr = 0.0;
      //--- motion coding contexts ---
      ADD_XRATE2 (p_Vid, 3, NUM_MB_TYPE_CTX,   mc->mb_type_contexts,     INIT_MB_TYPE_P [model]);
      ADD_XRATE2 (p_Vid, 2, NUM_B8_TYPE_CTX,   mc->b8_type_contexts,     INIT_B8_TYPE_P [model]);
      ADD_XRATE2 (p_Vid, 2, NUM_MV_RES_CTX,    mc->mv_res_contexts,      INIT_MV_RES_P  [model]);
      ADD_XRATE2 (p_Vid, 2, NUM_REF_NO_CTX,    mc->ref_no_contexts,      INIT_REF_NO_P  [model]);

      //--- texture coding contexts ---
      ADD_XRATE1 (p_Vid,    NUM_TRANSFORM_SIZE_CTX,  tc->transform_size_contexts, INIT_TRANSFORM_SIZE_P[model][0]);
      ADD_XRATE1 (p_Vid,                   NUM_IPR_CTX,  tc->ipr_contexts,       INIT_IPR_P  [model][0]);
      ADD_XRATE1 (p_Vid,                   NUM_CIPR_CTX, tc->cipr_contexts,      INIT_CIPR_P [model][0]);
      ADD_XRATE2 (p_Vid, 3,                NUM_CBP_CTX,  tc->cbp_contexts,       INIT_CBP_P  [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_BCBP_CTX, tc->bcbp_contexts,      INIT_BCBP_P [model]);
      ADD_XRATE1 (p_Vid, NUM_DELTA_QP_CTX,               tc->delta_qp_contexts,  INIT_DELTA_QP_P[model][0]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_MAP_CTX,  tc->map_contexts[0],    INIT_MAP_P  [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_LAST_CTX, tc->last_contexts[0],   INIT_LAST_P [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_ONE_CTX,  tc->one_contexts,       INIT_ONE_P      [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_ABS_CTX,  tc->abs_contexts,       INIT_ABS_P      [model]);
#if ENABLE_FIELD_CTX
      ADD_XRATE1 (p_Vid, NUM_MB_AFF_CTX,                 tc->mb_aff_contexts,    INIT_MB_AFF_P  [model][0]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_MAP_CTX,  tc->map_contexts[1],    INIT_FLD_MAP_P  [model]);
      ADD_XRATE2 (p_Vid, NUM_BLOCK_TYPES,  NUM_LAST_CTX, tc->last_contexts[1],   INIT_FLD_LAST_P [model]);
#endif

      if (xr < min_xr)
      {
        min_xr    = xr;
        *mnumber  = model;
      }
    }
  }
}

#undef ADD_XRATE2
#undef ADD_XRATE1


void store_contexts (Slice *currSlice)
{
  if( currSlice->p_Inp->context_init_method )
  {
    VideoParameters *p_Vid = currSlice->p_Vid;
    int frame_field = p_Vid->field_picture;
    int img_type    = currSlice->slice_type;
    int ctx_number  = currSlice->start_mb_nr / p_Vid->num_mb_per_slice;

    p_Vid->initialized [frame_field][img_type][ctx_number] = 1;
    GetCtxModelNumber (currSlice, p_Vid->modelNumber[frame_field][img_type] + ctx_number, currSlice->mot_ctx, currSlice->tex_ctx);
  }
  else
  {
    // do nothing
  }
}


void update_field_frame_contexts (VideoParameters *p_Vid, int field)
{
  int i, j;

  if (field)
  {
    // set frame contexts
    for (j=0; j<FRAME_TYPES; j++)
    {
      for (i=0; i<p_Vid->number_of_slices; i++)
      {
        p_Vid->initialized [0][j][i] = p_Vid->initialized [1][j][i>>1];
        p_Vid->modelNumber[0][j][i] = p_Vid->modelNumber[1][j][i>>1];
      }
    }
  }
  else
  {
    // set field contexts
    for (j=0; j<FRAME_TYPES; j++)
    {
      for (i=0; i<((p_Vid->number_of_slices+1)>>1); i++)
      {
        p_Vid->initialized [1][j][i] = p_Vid->initialized [0][j][i<<1];
        p_Vid->modelNumber[1][j][i] = p_Vid->modelNumber[0][j][i<<1];
      }
    }
  }
}

