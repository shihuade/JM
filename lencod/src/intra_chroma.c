/*!
*************************************************************************************
 * \file intra_chroma.c
 *
 * \brief
 *    Intra chroma mode functions
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Alexis Michael Tourapis         <alexismt@ieee.org>
 *************************************************************************************
 */

#include "contributors.h"

#include "global.h"
#include "image.h"
#include "mb_access.h"
#include "block.h"
#include "mv_search.h"

void get_difference_4x4(imgpel **src, imgpel **prd, short *diff, int pos_x, int block_x)
{
  int i, j;
  short *p_diff = diff;
  imgpel *img_prd, *img_src;
  for (j = 0; j < BLOCK_SIZE; j++)
  {
    img_src = &src[j][pos_x + block_x];
    img_prd = &prd[j][block_x];    

    for (i = 0; i < BLOCK_SIZE; i++)
      *p_diff++ = img_src[i] - img_prd[i];
  }
}

/*!
 ************************************************************************
 * \brief
 *    Vertical Chroma Prediction
 ************************************************************************
 */
static inline void get_iChroma_vertical(imgpel **cur_pred, imgpel *PredPel, int cr_MB_x, int cr_MB_y)
{
  int j;
  for (j = 0; j < cr_MB_y; j++)
    memcpy(cur_pred[j], &PredPel[1], cr_MB_x * sizeof(imgpel));
}


/*!
 ************************************************************************
 * \brief
 *    Horizontal Chroma Prediction
 ************************************************************************
 */
static inline void get_iChroma_horizontal(imgpel **cur_pred, imgpel *PredPel, int cr_MB_x, int cr_MB_y)
{
  int i, j;
  imgpel *prediction = &PredPel[16];

  for (j = 0; j < cr_MB_y; j++)
  {  
    prediction++;
    for (i = 0; i < cr_MB_x; i++)
    {
      cur_pred[j][i]  = *prediction;
    }    
  }
}

/*!
 ************************************************************************
 * \brief
 *    DC Chroma Prediction
 ************************************************************************
 */
static inline void get_iChroma_dc(imgpel **cur_pred, imgpel *PredPel, int left_available, int up_available)
{
  int i, j, s0 = 0, s1 = 0, s2 = 0;

  if (up_available)
  {
    for (i = 1; i < MB_BLOCK_SIZE + 1; ++i)
      s1 += PredPel[i];
  }

  if (left_available)
  {
    for (i = 17; i < 33; ++i)
      s2 += PredPel[i];    // sum vert pix
  }

  if (up_available)
  {
    s0 = left_available
      ? rshift_rnd_sf((s1 + s2),(MB_BLOCK_SHIFT + 1)) // no edge
      : rshift_rnd_sf(s1, MB_BLOCK_SHIFT);          // left edge
  }
  else
  {
    s0 = left_available
      ? rshift_rnd_sf(s2, MB_BLOCK_SHIFT)           // upper edge
      : PredPel[1];                              // top left corner, nothing to predict from
  }

  for (j = 0; j < MB_BLOCK_SIZE; j++)
  {
    for (i = 0; i < MB_BLOCK_SIZE; i++)
      cur_pred[j][i] = (imgpel) s0;    
  }
}

/*!
 ************************************************************************
 * \brief
 *    Plane Chroma Prediction
 ************************************************************************
 */
static inline void get_iChroma_plane(imgpel **cur_pred, imgpel *PredPel, int max_imgpel_value)
{
  int i, j;
  // plane prediction
  int ih=0, iv=0;
  int ib, ic, iaa;
  imgpel *t_pred = &PredPel[25];
  imgpel *u_pred = &PredPel[8];
  imgpel *b_pred = &PredPel[23];

  for (i = 1; i < 8;++i)
  {
    ih += i*(*(u_pred + i) - *(u_pred - i));
    iv += i*(*t_pred++ - *b_pred--);
  }
  ih += 8 * (*(u_pred + 8) - PredPel[0]);
  iv += 8 * (*t_pred++ - PredPel[0]);

  ib = (5 * ih + 32) >> 6;
  ic = (5 * iv + 32) >> 6;

  iaa=16 * (PredPel[16] + PredPel[32]);

  for (j=0;j< MB_BLOCK_SIZE;++j)
  {
    for (i=0;i< MB_BLOCK_SIZE;++i)
    {
      cur_pred[j][i]= (imgpel) iClip1( max_imgpel_value, rshift_rnd_sf((iaa + (i - 7) * ib + (j - 7) * ic), 5));// store plane prediction
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Set intra Chroma prediction samples
 *
 *  \par Input:
 *
 *  \par Output:
 *      none
 ************************************************************************
 */
void set_intrapred_chroma(Macroblock *currMB, int *left_available, int *up_available, int *all_available)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  int i;
  imgpel  *PredPelU = currMB->intra16x16_pred[PLANE_U];  // array of predictor pels
  imgpel  *PredPelV = currMB->intra16x16_pred[PLANE_V];  // array of predictor pels
  imgpel **img_enc_u = p_Vid->enc_picture->imgUV[0];
  imgpel **img_enc_v = p_Vid->enc_picture->imgUV[1];

  PixelPos pix_b;
  PixelPos pix_a;
  PixelPos pix_d;
  int      cr_MB_x = p_Vid->mb_cr_size_x;
  int      cr_MB_y = p_Vid->mb_cr_size_y;

  imgpel dc_pred_value_chroma = p_Vid->dc_pred_value_comp[1];

  int *mb_size = p_Vid->mb_size[IS_CHROMA];

  p_Vid->getNeighbour(currMB, -1,  -1, mb_size, &pix_d);
  p_Vid->getNeighbour(currMB, -1,   0, mb_size, &pix_a);
  p_Vid->getNeighbour(currMB,  0,  -1, mb_size, &pix_b);

  if (p_Inp->UseConstrainedIntraPred == 0)
  {
    *up_available   = pix_b.available;
    *left_available = pix_a.available;
    *all_available  = pix_d.available;
  }
  else
  {
    *up_available   = pix_b.available ? p_Vid->intra_block[pix_b.mb_addr] : 0;
    *left_available = pix_a.available ? p_Vid->intra_block[pix_a.mb_addr] : 0;
    *all_available  = pix_d.available ? p_Vid->intra_block[pix_d.mb_addr] : 0;
  }

  // form predictor pels
  if (*up_available)
  {
    memcpy(&PredPelU[1], &img_enc_u[pix_b.pos_y][pix_b.pos_x], cr_MB_x * sizeof(imgpel));
    memcpy(&PredPelV[1], &img_enc_v[pix_b.pos_y][pix_b.pos_x], cr_MB_x * sizeof(imgpel));
  }
  else
  {
    for (i = 1; i < cr_MB_x + 1; ++i)
      PredPelU[i] = (imgpel) dc_pred_value_chroma;
    for (i = 1; i < cr_MB_x + 1; ++i)
      PredPelV[i] = (imgpel) dc_pred_value_chroma;
  }

  if (*left_available)
  {
    int pos_y = pix_a.pos_y;
    int pos_x = pix_a.pos_x;

    for (i = 1; i < cr_MB_y + 1; ++i)
    {
      PredPelU[i + cr_MB_x] = img_enc_u[pos_y  ][pos_x];
      PredPelV[i + cr_MB_x] = img_enc_v[pos_y++][pos_x];
    }
  }
  else
  {
    for (i = cr_MB_x + 1; i < cr_MB_x + cr_MB_y + 1; ++i)
      PredPelU[i] = (imgpel) dc_pred_value_chroma;
    for (i = cr_MB_x + 1; i < cr_MB_x + cr_MB_y + 1; ++i)
      PredPelV[i] = (imgpel) dc_pred_value_chroma;
  }

  if (*all_available)
  {
    PredPelU[0] = img_enc_u[pix_d.pos_y][pix_d.pos_x];
    PredPelV[0] = img_enc_v[pix_d.pos_y][pix_d.pos_x];
  }
  else
  {
    PredPelU[0] = dc_pred_value_chroma;
    PredPelV[0] = dc_pred_value_chroma;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Set intra Chroma prediction samples (MBAFF)
 *
 *  \par Input:
 *
 *  \par Output:
 *      none
 ************************************************************************
 */
void set_intrapred_chroma_mbaff(Macroblock *currMB, ColorPlane pl, int *left_available, int *up_available, int *all_available)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  int i;
  imgpel  *PredPelU = currMB->intra16x16_pred[PLANE_U];  // array of predictor pels
  imgpel  *PredPelV = currMB->intra16x16_pred[PLANE_V];  // array of predictor pels
  imgpel **img_enc_u = p_Vid->enc_picture->imgUV[0];
  imgpel **img_enc_v = p_Vid->enc_picture->imgUV[1];

  PixelPos pix_b;
  PixelPos pix_a[17];
  int      cr_MB_x = p_Vid->mb_cr_size_x;
  int      cr_MB_y = p_Vid->mb_cr_size_y;

  imgpel dc_pred_value_chroma = p_Vid->dc_pred_value_comp[1];

  int *mb_size = p_Vid->mb_size[IS_CHROMA];

  for (i = 0; i < cr_MB_y + 1; ++i)
  {
    p_Vid->getNeighbour(currMB, -1,  i - 1, mb_size, &pix_a[i]);
  }

  p_Vid->getNeighbour(currMB,    0,   -1, mb_size, &pix_b);

  if (p_Inp->UseConstrainedIntraPred == 0)
  {
    *up_available   = pix_b.available;
    *left_available = pix_a[1].available;
    *all_available  = pix_a[0].available;
  }
  else
  {
    *up_available  = pix_b.available ? p_Vid->intra_block[pix_b.mb_addr] : 0;
    for (i=1, *left_available=1; i<17;++i)
      *left_available &= pix_a[i].available ? p_Vid->intra_block[pix_a[i].mb_addr]: 0;
    *all_available = pix_a[0].available ? p_Vid->intra_block[pix_a[0].mb_addr]: 0;
  }

  // form predictor pels
  if (*up_available)
  {
    memcpy(&PredPelU[1], &img_enc_u[pix_b.pos_y][pix_b.pos_x], cr_MB_x * sizeof(imgpel));
    memcpy(&PredPelV[1], &img_enc_v[pix_b.pos_y][pix_b.pos_x], cr_MB_x * sizeof(imgpel));
  }
  else
  {
    for (i = 1; i < cr_MB_x + 1; ++i)
      PredPelU[i] = (imgpel) dc_pred_value_chroma;
    for (i = 1; i < cr_MB_x + 1; ++i)
      PredPelV[i] = (imgpel) dc_pred_value_chroma;
  }

  if (*left_available)
  {
    for (i = 1; i < cr_MB_y + 1; ++i)
      PredPelU[i + cr_MB_x] = img_enc_u[pix_a[i].pos_y][pix_a[i].pos_x];
    for (i = 1; i < cr_MB_y + 1; ++i)
      PredPelV[i + cr_MB_x] = img_enc_v[pix_a[i].pos_y][pix_a[i].pos_x];
  }
  else
  {
    for (i = cr_MB_x + 1; i < cr_MB_x + cr_MB_y + 1; ++i)
      PredPelU[i] = (imgpel) dc_pred_value_chroma;
    for (i = cr_MB_x + 1; i < cr_MB_x + cr_MB_y + 1; ++i)
      PredPelV[i] = (imgpel) dc_pred_value_chroma;
  }

  if (*all_available)
  {
    PredPelU[0] = img_enc_u[pix_a[0].pos_y][pix_a[0].pos_x];
    PredPelV[0] = img_enc_v[pix_a[0].pos_y][pix_a[0].pos_x];
  }
  else
  {
    PredPelU[0] = dc_pred_value_chroma;
    PredPelV[0] = dc_pred_value_chroma;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Generate Chroma intra prediction block
 *
 *  \par Input:
 *     Starting point of current Chroma block image posision
 *
 *  \par Output:
 *      none
 ************************************************************************
 */
void get_intrapred_chroma(Macroblock *currMB, ColorPlane pl, int iChroma_mode, int left_available, int up_available)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  imgpel   *PredPel = currMB->intra16x16_pred[pl];  // array of predictor pels
  imgpel ***curr_mpr_16x16 = currMB->p_Slice->mpr_16x16[pl];
  int       cr_MB_x = p_Vid->mb_cr_size_x;
  int       cr_MB_y = p_Vid->mb_cr_size_y;

  switch (iChroma_mode)
  {
  case VERT_PRED_8 :    
    get_iChroma_vertical(curr_mpr_16x16[VERT_PRED_16], PredPel, cr_MB_x, cr_MB_y);
    break;
  case HOR_PRED_8 :
    get_iChroma_horizontal(curr_mpr_16x16[HOR_PRED_16], PredPel, cr_MB_x, cr_MB_y);
    break;
  case DC_PRED_8 :
    get_iChroma_dc(curr_mpr_16x16[DC_PRED_16], PredPel, left_available, up_available);
    break;
  case PLANE_8 :
    get_iChroma_plane(curr_mpr_16x16[PLANE_16], PredPel, currMB->p_Vid->max_imgpel_value);
    break;
  default:
    printf("invalid prediction mode \n");
    break;
  }
}

void rdo_low_intra_chroma_decision_mbaff(Macroblock *currMB, int mb_available_up, int mb_available_left[2], int mb_available_up_left)
{                       // since ipredmodes could be overwritten => encoder-decoder-mismatches
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int i, j, uv;
  int      cr_MB_x = p_Vid->mb_cr_size_x;
  int      cr_MB_y = p_Vid->mb_cr_size_y;
  int      block_x, block_y;
  PixelPos pix_a[17];  //!< pixel positions p(-1, -1..15)
  imgpel  **image = NULL;
  imgpel ***curr_mpr_16x16 = NULL;


  short diff  [16];
  short *p_diff;
  distblk  cost, min_cost = DISTBLK_MAX;
  int      mode;
  int      best_mode = DC_PRED_8;  //just an initilaization here, should always be overwritten
  imgpel *img_org = NULL, *img_prd = NULL;

  for (i=0;i<cr_MB_y;i++)
  {
    p_Vid->getNeighbour(currMB, 0 , i, p_Vid->mb_size[IS_CHROMA], &pix_a[i]);
  }

  if ( p_Vid->mb_aff_frame_flag && p_Vid->field_mode )
  {
    for (i=0;i<cr_MB_y;i++)
    {
      pix_a[i].pos_y = pix_a[i].pos_y >> 1;
    }
  }

  for (mode=DC_PRED_8; mode<=PLANE_8; mode++)
  {
    if (((currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE) || !p_Inp->IntraDisableInterOnly) && p_Inp->ChromaIntraDisable == 1 && mode!=DC_PRED_8)
      continue;

    if ((mode==VERT_PRED_8 && !mb_available_up) ||
      (mode==HOR_PRED_8 && (!mb_available_left[0] || !mb_available_left[1])) ||
      (mode==PLANE_8 && (!mb_available_left[0] || !mb_available_left[1] || !mb_available_up || !mb_available_up_left)))
      continue;

    cost = 0;
    for (uv = 1; uv < 3; uv++)
    {
      image = p_Vid->pImgOrg[uv];
      curr_mpr_16x16 = currSlice->mpr_16x16[uv];

      for (block_y = 0; block_y < cr_MB_y; block_y += 4)
      {
        for (block_x = 0; block_x < cr_MB_x; block_x += 4)
        {
          p_diff = &diff[0];
          for (j = block_y; j < block_y + 4; j++)
          {
            img_prd = curr_mpr_16x16[mode][j];
            img_org = &image[pix_a[j].pos_y][pix_a[j].pos_x];
            for (i = block_x; i < block_x + 4; i++)
              *p_diff++ = img_org[i] - img_prd[i];
          }
          cost += p_Vid->distortion4x4(diff, min_cost);
          if (cost > min_cost) break;
        }
        if (cost > min_cost) break;
      }
      if (cost > min_cost) break;
    }
    if (cost < min_cost)
    {
      best_mode = mode;
      min_cost = cost;
    }
  }
  currMB->c_ipred_mode = (char) best_mode;
}


void rdo_low_intra_chroma_decision(Macroblock *currMB, int mb_available_up, int mb_available_left[2], int mb_available_up_left)
{                       // since ipredmodes could be overwritten => encoder-decoder-mismatches
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int uv;
  int      cr_MB_x = p_Vid->mb_cr_size_x;
  int      cr_MB_y = p_Vid->mb_cr_size_y;
  int      block_x, block_y;
  PixelPos pix_a;  //!< pixel positions p(-1, 0)

  imgpel  **image = NULL;
  imgpel ***curr_mpr_16x16 = NULL;

  short diff  [16];

  distblk  cost, min_cost = DISTBLK_MAX;
  int      mode;
  int      best_mode = DC_PRED_8;  //just an initialization here, should always be overwritten

  p_Vid->getNeighbour(currMB, 0 , 0, p_Vid->mb_size[IS_CHROMA], &pix_a);

  for (mode = DC_PRED_8; mode <= PLANE_8; mode++)
  {
    if (((currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE) || !p_Inp->IntraDisableInterOnly) && p_Inp->ChromaIntraDisable == 1 && mode!=DC_PRED_8)
      continue;

    if ((mode==VERT_PRED_8 && !mb_available_up) ||
      (mode==HOR_PRED_8 && (!mb_available_left[0] || !mb_available_left[1])) ||
      (mode==PLANE_8 && (!mb_available_left[0] || !mb_available_left[1] || !mb_available_up || !mb_available_up_left)))
      continue;

    cost = 0;
    
    for (uv = 1; uv < 3; uv++)
    {
      int pos_x = pix_a.pos_x;
      int pos_y = pix_a.pos_y;
      image = p_Vid->pImgOrg[uv];
      curr_mpr_16x16 = currSlice->mpr_16x16[uv];

      for (block_y = 0; block_y < cr_MB_y; block_y += BLOCK_SIZE)
      {
        for (block_x = 0; block_x < cr_MB_x; block_x += BLOCK_SIZE)
        {          
          get_difference_4x4(&image[pos_y + block_y], &curr_mpr_16x16[mode][block_y], diff, pos_x, block_x);
          cost += p_Vid->distortion4x4(diff, min_cost);
          if (cost > min_cost) break;
        }
        if (cost > min_cost) break;
      }
      if (cost > min_cost) break;
    }

    if (cost < min_cost)
    {
      best_mode = mode;
      min_cost = cost;
    }
  }
  currMB->c_ipred_mode = (char) best_mode;
}


/*!
 ************************************************************************
 * \brief
 *    Intra prediction of the chrminance layers of one macroblock
 ************************************************************************
 */
void intra_chroma_prediction (Macroblock *currMB, int *mb_up, int *mb_left, int*mb_up_left)
{
  int s, i, j;

  int      uv;
  int      b8, b4;
  imgpel   vline[16];

  int      mb_available_up;
  int      mb_available_left[2];
  int      mb_available_up_left;

  PixelPos pix_c;  //!< pixel position  p(0,-1)
  PixelPos pix_d;
  PixelPos pix_a;  //!< pixel positions p(-1, -1..15)

  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int      cr_MB_x = p_Vid->mb_cr_size_x;
  int      cr_MB_y = p_Vid->mb_cr_size_y;
  imgpel **cur_pred = NULL;

  imgpel *hline = NULL;

  int      yuv = p_Vid->yuv_format - 1;
  int      dc_pred_value_chroma = p_Vid->dc_pred_value_comp[1];
  int      max_imgpel_value_uv  = p_Vid->max_pel_value_comp[1];

  static const int block_pos[3][4][4]= //[yuv][b8][b4]
  {
    { {0, 1, 2, 3},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}},
    { {0, 1, 2, 3},{2, 3, 2, 3},{0, 0, 0, 0},{0, 0, 0, 0}},
    { {0, 1, 2, 3},{1, 1, 3, 3},{2, 3, 2, 3},{3, 3, 3, 3}}
  };
  
  p_Vid->getNeighbour(currMB, -1, -1, p_Vid->mb_size[IS_CHROMA], &pix_d);
  p_Vid->getNeighbour(currMB, -1,  0, p_Vid->mb_size[IS_CHROMA], &pix_a);
  p_Vid->getNeighbour(currMB,  0, -1, p_Vid->mb_size[IS_CHROMA], &pix_c);

  mb_available_up      = pix_c.available;
  mb_available_up_left = pix_d.available;
  mb_available_left[0] = mb_available_left[1] = pix_a.available;

  if(p_Inp->UseConstrainedIntraPred)
  {
    mb_available_up      = pix_c.available ? p_Vid->intra_block[pix_c.mb_addr] : 0;
    mb_available_left[0] = mb_available_left[1] = pix_a.available ? p_Vid->intra_block[pix_a.mb_addr] : 0;
    mb_available_up_left = pix_d.available ? p_Vid->intra_block[pix_d.mb_addr] : 0;
  }

  if (mb_up)
    *mb_up = mb_available_up;
  if (mb_left)
    *mb_left = mb_available_left[0];
  if (mb_up_left)
    *mb_up_left = mb_available_up_left;

  // compute all chroma intra prediction modes for both U and V
  for (uv=0; uv<2; uv++)
  {
    imgpel **image = p_Vid->enc_picture->imgUV[uv];
    imgpel ***curr_mpr_16x16 = currSlice->mpr_16x16[uv + 1];

    // DC prediction
    for(b8=0; b8<p_Vid->num_blk8x8_uv >> 1;b8++)
    {
      for (b4 = 0; b4 < 4; b4++)
      {
        int block_y = subblk_offset_y[yuv][b8][b4];
        int block_x = subblk_offset_x[yuv][b8][b4];
        int blk_x = block_x;

        s = dc_pred_value_chroma;

        //===== get prediction value =====
        switch (block_pos[yuv][b8][b4])
        {
        case 0:  //===== TOP LEFT =====
          {
            int s0 = 0, s2 = 0;
            if (mb_available_up)       
            {
              int pos_x = pix_c.pos_x + blk_x;
              int pos_y = pix_c.pos_y;

              for (i = 0; i < BLOCK_SIZE; i++)  
                s0 += image[pos_y][pos_x++];
            }
            if (mb_available_left[0]) 
            {
              int pos_x = pix_a.pos_x;
              int pos_y = pix_a.pos_y + block_y; 

              for (i = 0; i < BLOCK_SIZE;i++)
                s2 += image[pos_y++][pos_x];
            }
            if (mb_available_up && mb_available_left[0])  
              s = (s0 + s2 + 4) >> 3;
            else if (mb_available_up)                          
              s = (s0 + 2) >> 2;
            else if (mb_available_left[0])                     
              s = (s2 + 2) >> 2;
          }
          break;
        case 1: //===== TOP RIGHT =====
          {
            int s1 = 0, s2 = 0;
            if (mb_available_up)
            {
              int pos_x = pix_c.pos_x + blk_x;
              int pos_y = pix_c.pos_y;
              for (i = 0; i < BLOCK_SIZE; i++)  
                s1 += image[pos_y][pos_x++];
            }
            else if (mb_available_left[0])
            {
              int pos_x = pix_a.pos_x;
              int pos_y = pix_a.pos_y + block_y; 

              for (i = 0; i < BLOCK_SIZE; i++)  
                s2 += image[pos_y++][pos_x];
            }
            if      (mb_available_up)       
              s  = (s1   +2) >> 2;
            else if (mb_available_left[0])                    
              s  = (s2   +2) >> 2;
          }
          break;
        case 2: //===== BOTTOM LEFT =====
          if      (mb_available_left[0])  
          {
            int pos_x = pix_a.pos_x;
            int pos_y = pix_a.pos_y + block_y; 
            int s3 = 0;

            for (i = 0; i < BLOCK_SIZE; i++)
              s3 += image[pos_y++][pos_x];
            s  = (s3 + 2) >> 2;
          }
          else if (mb_available_up)       
          {
            int pos_x = pix_c.pos_x + blk_x;
            int pos_y = pix_c.pos_y;

            int s0 = 0;
            for (i = 0; i < BLOCK_SIZE; i++)  
              s0 += image[pos_y][pos_x++];
            s  = (s0 + 2) >> 2;
          }
          break;
        case 3: //===== BOTTOM RIGHT =====
          {
            int s1 = 0, s3 = 0;
            if (mb_available_up)       
              for (i=blk_x;i<(blk_x+4);i++)  
                s1 += image[pix_c.pos_y][pix_c.pos_x + i];
            if (mb_available_left[0])  
            {
              int pos_x = pix_a.pos_x;
              int pos_y = pix_a.pos_y + block_y; 
              for (i = 0; i < BLOCK_SIZE;i++)  
                s3 += image[pos_y++][pos_x];
            }
            if      (mb_available_up && mb_available_left[0])  
              s  = (s1 + s3 + 4) >> 3;
            else if (mb_available_up)                          
              s  = (s1 + 2) >> 2;
            else if (mb_available_left[0])                     
              s  = (s3 + 2) >> 2;
          }
          break;
        }

        //===== prediction =====
        cur_pred = curr_mpr_16x16[DC_PRED_8];
        for (j = block_y; j < block_y+4; j++)
        {
          for (i = block_x; i < block_x+4; i++)
          {
            cur_pred[j][i] = (imgpel) s;
          }
        }
      }
    }

    // vertical prediction    
    if (mb_available_up)
    {
      cur_pred = curr_mpr_16x16[VERT_PRED_8];      
      hline = &image[pix_c.pos_y][pix_c.pos_x];
      for (j=0; j<cr_MB_y; j++)
        memcpy(cur_pred[j], hline, cr_MB_x * sizeof(imgpel));
    }

    // horizontal prediction
    if (mb_available_left[0])
    {
      int pos_x = pix_a.pos_x;
      int pos_y = pix_a.pos_y;
      cur_pred = curr_mpr_16x16[HOR_PRED_8];
      for (i=0; i<cr_MB_y; i++)
        vline[i] = image[pos_y++][pos_x];
      
      for (j=0; j<cr_MB_y; j++)
      {
        int predictor = vline[j];
        for (i = 0; i < cr_MB_x; i++)        
          cur_pred[j][i] = (imgpel) predictor;
      }
    }

    // plane prediction
    if (mb_available_left[0] && mb_available_up && mb_available_up_left)
    {
      int cr_x = (cr_MB_x >> 1);
      int cr_y = (cr_MB_y >> 1);

      int iaa, iv, ib, ic;
      int ih = cr_x * (hline[cr_MB_x-1] - image[pix_d.pos_y][pix_d.pos_x]);
      
      for (i = 0; i < cr_x - 1; i++)
        ih += (i + 1)*(hline[cr_x + i] - hline[cr_x - 2 - i]);

      iv = cr_y * (vline[cr_MB_y-1] - image[pix_d.pos_y][pix_d.pos_x]);
      for (i = 0; i < cr_y - 1; i++)
        iv += (i + 1) * (vline[cr_y + i] - vline[cr_y - 2 - i]);

      if (cr_MB_x == 8)
        ib = (17 * ih + 2 * cr_MB_x) >> 5;
      else
        ib = ( 5 * ih + 2 * cr_MB_x) >> 6;

      if (cr_MB_y == 8)
        ic = (17 * iv + 2 * cr_MB_y) >> 5;
      else
        ic = ( 5 * iv + 2 * cr_MB_y) >> 6;

      iaa = 16 * (hline[cr_MB_x - 1] + vline[cr_MB_y - 1]);
      cur_pred = curr_mpr_16x16[PLANE_8];
      iaa += (1 - cr_x) * ib + (1 - cr_y) * ic;
      for (j = 0; j < cr_MB_y; j++)
        for (i = 0; i < cr_MB_x; i++)
          cur_pred[j][i]= (imgpel) iClip1( max_imgpel_value_uv, (iaa + i * ib + j * ic + 16)>>5);
    }
  }

  if (!p_Inp->rdopt)      // the rd-opt part does not work correctly (see encode_one_macroblock)
  {                       // since ipredmodes could be overwritten => encoder-decoder-mismatches
    currSlice->rdo_low_intra_chroma_decision(currMB, mb_available_up, mb_available_left, mb_available_up_left);
  }
}

/*!
 ************************************************************************
 * \brief
 *    Intra prediction of the chrminance layers of one macroblock
 ************************************************************************
 */
void intra_chroma_prediction_mbaff(Macroblock *currMB, int *mb_up, int *mb_left, int*mb_up_left)
{
  int s, i, j;

  int      uv;
  int      b8,b4;
  imgpel   vline[16];

  int      mb_available_up;
  int      mb_available_left[2];
  int      mb_available_up_left;

  PixelPos pix_b;        //!< pixel position  p(0,-1)
  PixelPos pix_a[17];  //!< pixel positions p(-1, -1..15)

  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  int      cr_MB_x = p_Vid->mb_cr_size_x;
  int      cr_MB_y = p_Vid->mb_cr_size_y;
  imgpel **cur_pred = NULL;
  imgpel ***curr_mpr_16x16 = NULL;
  imgpel *hline = NULL;

  int      yuv = p_Vid->yuv_format - 1;
  int      dc_pred_value_chroma = p_Vid->dc_pred_value_comp[1];
  int      max_imgpel_value_uv  = p_Vid->max_pel_value_comp[1];
  int *mb_size = p_Vid->mb_size[IS_CHROMA];

  static const int block_pos[3][4][4]= //[yuv][b8][b4]
  {
    { {0, 1, 4, 5},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}},
    { {0, 1, 2, 3},{4, 5, 4, 5},{0, 0, 0, 0},{0, 0, 0, 0}},
    { {0, 1, 2, 3},{1, 1, 3, 3},{4, 5, 4, 5},{5, 5, 5, 5}}
  };

  for (i = 0; i < cr_MB_y + 1; ++i)
  {
    p_Vid->getNeighbour(currMB, -1 , i-1 , mb_size, &pix_a[i]);
  }

  p_Vid->getNeighbour(currMB, 0 , -1 , mb_size, &pix_b);

  if (p_Inp->UseConstrainedIntraPred == 0)
  {
    mb_available_up                             = pix_b.available;
    mb_available_up_left                        = pix_a[0].available;
    mb_available_left[0] = mb_available_left[1] = pix_a[1].available;
  }
  else
  {
    mb_available_up = pix_b.available ? p_Vid->intra_block[pix_b.mb_addr] : 0;
    for (i=0, mb_available_left[0] = 1; i < (cr_MB_y >> 1); i++)
      mb_available_left[0]  &= pix_a[i + 1].available ? p_Vid->intra_block[pix_a[i+1].mb_addr]: 0;
    for (i=(cr_MB_y>>1), mb_available_left[1]=1; i<cr_MB_y;i++)
      mb_available_left[1] &= pix_a[i+1].available ? p_Vid->intra_block[pix_a[i+1].mb_addr]: 0;
    mb_available_up_left = pix_a[0].available ? p_Vid->intra_block[pix_a[0].mb_addr]: 0;
  }

  if (mb_up)
    *mb_up = mb_available_up;
  if (mb_left)
    *mb_left = mb_available_left[0] && mb_available_left[1];
  if (mb_up_left)
    *mb_up_left = mb_available_up_left;


  // compute all chroma intra prediction modes for both U and V
  for (uv=0; uv<2; uv++)
  {
    imgpel **image = p_Vid->enc_picture->imgUV[uv];
    curr_mpr_16x16 = currSlice->mpr_16x16[uv + 1];

    // DC prediction
    for(b8=0; b8<p_Vid->num_blk8x8_uv >> 1;b8++)
    {
      for (b4 = 0; b4 < 4; b4++)
      {
        int block_y = subblk_offset_y[yuv][b8][b4];
        int block_x = subblk_offset_x[yuv][b8][b4];
        int blk_x = block_x;
        int blk_y = block_y + 1;

        s = dc_pred_value_chroma;

        //===== get prediction value =====
        switch (block_pos[yuv][b8][b4])
        {
        case 0:  //===== TOP TOP-LEFT =====
          {
            int s0 = 0, s2 = 0;
            if      (mb_available_up)       
              for (i = blk_x; i < (blk_x + 4); i++)  
                s0 += image[pix_b.pos_y][pix_b.pos_x + i];
            if      (mb_available_left[0])  
              for (i = blk_y; i < (blk_y + 4);i++)  
                s2 += image[pix_a[i].pos_y][pix_a[i].pos_x];
            if      (mb_available_up && mb_available_left[0])  
              s  = (s0 + s2 + 4) >> 3;
            else if (mb_available_up)                          
              s  = (s0   + 2) >> 2;
            else if (mb_available_left[0])                     
              s  = (s2   + 2) >> 2;
          }
          break;
        case 1: //===== TOP TOP-RIGHT =====
          {
            int s1 = 0, s2 = 0;
            if      (mb_available_up)       
              for (i=blk_x;i<(blk_x+4);i++)  
                s1 += image[pix_b.pos_y][pix_b.pos_x + i];
            else if (mb_available_left[0])  
              for (i=blk_y;i<(blk_y+4);i++) 
                s2 += image[pix_a[i].pos_y][pix_a[i].pos_x];
            if      (mb_available_up)       
              s  = (s1   +2) >> 2;
            else if (mb_available_left[0])                    
              s  = (s2   +2) >> 2;
          }
          break;
        case 2:  //===== TOP BOTTOM-LEFT =====
          if      (mb_available_left[0])  
          {
            int s3 = 0;
            for (i=blk_y;i<(blk_y+4);i++)  
              s3 += image[pix_a[i].pos_y][pix_a[i].pos_x];
            s  = (s3 + 2) >> 2;
          }
          else if (mb_available_up)       
          {
            int s0 = 0;
            for (i=blk_x;i<(blk_x+4);i++)  
              s0 += image[pix_b.pos_y][pix_b.pos_x + i];
            s  = (s0 + 2) >> 2;
          }
          break;
        case 3: //===== TOP BOTTOM-RIGHT =====
          {
            int s1 = 0, s3 = 0;
            if      (mb_available_up) 
              for (i=blk_x;i<(blk_x+4);i++)
                s1 += image[pix_b.pos_y][pix_b.pos_x + i];
            if      (mb_available_left[0]) 
              for (i=blk_y;i<(blk_y+4);i++)
                s3 += image[pix_a[i].pos_y][pix_a[i].pos_x];
            if      (mb_available_up && mb_available_left[0])
              s  = (s1 + s3 + 4) >> 3;
            else if (mb_available_up)
              s  = (s1 + 2) >> 2;
            else if (mb_available_left[0])
              s  = (s3 + 2) >> 2;
          }
          break;

        case 4: //===== BOTTOM LEFT =====
          if      (mb_available_left[1])  
          {
            int s3 = 0;
            for (i=blk_y;i<(blk_y+4);i++)  
              s3 += image[pix_a[i].pos_y][pix_a[i].pos_x];
            s  = (s3 + 2) >> 2;
          }
          else if (mb_available_up)       
          {
            int s0 = 0;
            for (i=blk_x;i<(blk_x+4);i++)  
              s0 += image[pix_b.pos_y][pix_b.pos_x + i];
            s  = (s0 + 2) >> 2;
          }
          break;
        case 5: //===== BOTTOM RIGHT =====
          {
            int s1 = 0, s3 = 0;
            if      (mb_available_up)       
              for (i=blk_x;i<(blk_x+4);i++)  
                s1 += image[pix_b.pos_y][pix_b.pos_x + i];
            if      (mb_available_left[1])  
              for (i=blk_y;i<(blk_y+4);i++)  
                s3 += image[pix_a[i].pos_y][pix_a[i].pos_x];
            if      (mb_available_up && mb_available_left[1])  
              s  = (s1 + s3 + 4) >> 3;
            else if (mb_available_up)                          
              s  = (s1 + 2) >> 2;
            else if (mb_available_left[1])                     
              s  = (s3 + 2) >> 2;
          }
          break;
        }

        //===== prediction =====
        cur_pred = curr_mpr_16x16[DC_PRED_8];
        for (j = block_y; j < block_y+4; j++)
        {
          for (i = block_x; i < block_x+4; i++)
          {
            cur_pred[j][i] = (imgpel) s;
          }
        }
      }
    }

    // vertical prediction    
    if (mb_available_up)
    {
      cur_pred = curr_mpr_16x16[VERT_PRED_8];      
      hline = &image[pix_b.pos_y][pix_b.pos_x];
      for (j=0; j<cr_MB_y; j++)
        memcpy(cur_pred[j], hline, cr_MB_x * sizeof(imgpel));
    }

    // horizontal prediction
    if (mb_available_left[0] && mb_available_left[1])
    {
      cur_pred = curr_mpr_16x16[HOR_PRED_8];
      for (i=0; i<cr_MB_y; i++)
        vline[i] = image[pix_a[i+1].pos_y][pix_a[i+1].pos_x];
      for (j=0; j<cr_MB_y; j++)
      {
        int predictor = vline[j];
        for (i=0; i<cr_MB_x; i++)        
          cur_pred[j][i] = (imgpel) predictor;
      }
    }

    // plane prediction
    if (mb_available_left[0] && mb_available_left[1] && mb_available_up && mb_available_up_left)
    {
      int cr_x = (cr_MB_x >> 1);
      int cr_y = (cr_MB_y >> 1);

      int iaa, iv, ib, ic;
      int ih = cr_x * (hline[cr_MB_x-1] - image[pix_a[0].pos_y][pix_a[0].pos_x]);
      for (i = 0; i < cr_x - 1; i++)
        ih += (i+1)*(hline[cr_x + i] - hline[cr_x - 2 - i]);

      iv = cr_y * (vline[cr_MB_y-1] - image[pix_a[0].pos_y][pix_a[0].pos_x]);
      for (i = 0; i < cr_y - 1; i++)
        iv += (i + 1) * (vline[cr_y + i] - vline[cr_y - 2 - i]);

      if (cr_MB_x == 8)
        ib = (17 * ih + 2 * cr_MB_x) >> 5;
      else
        ib = ( 5 * ih + 2 * cr_MB_x) >> 6;
      if (cr_MB_y == 8)
        ic = (17 * iv + 2 * cr_MB_y) >> 5;
      else
        ic = ( 5 * iv + 2 * cr_MB_y) >> 6;

      iaa = 16 * (hline[cr_MB_x - 1] + vline[cr_MB_y - 1]);
      cur_pred = curr_mpr_16x16[PLANE_8];
      iaa += (1 - cr_x) * ib + (1 - cr_y) * ic;
      for (j = 0; j < cr_MB_y; j++)
        for (i = 0; i < cr_MB_x; i++)
          cur_pred[j][i]= (imgpel) iClip1( max_imgpel_value_uv, (iaa + i * ib + j * ic + 16)>>5);
    }
  }

  if (!p_Inp->rdopt)      // the rd-opt part does not work correctly (see encode_one_macroblock)
  {                       // since ipredmodes could be overwritten => encoder-decoder-mismatches
    currSlice->rdo_low_intra_chroma_decision(currMB, mb_available_up, mb_available_left, mb_available_up_left);
  }
}

/*!
 **************************************************************************************
 * \brief
 *    RD Decision for Intra prediction mode of the chrominance layers of one macroblock
 **************************************************************************************
 */
void intra_chroma_RD_decision (Macroblock *currMB, RD_PARAMS *enc_mb)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  imgpel** image;
  int      block_x, block_y;  
  int      mb_available_up;
  int      mb_available_left;
  int      mb_available_up_left;
  int      uv;
  int      mode;
  int      best_mode = DC_PRED_8;  //just an initialization here, should always be overwritten
  distblk  cost;
  // pick lowest cost prediction mode
  distblk  min_cost = DISTBLK_MAX;

  PixelPos pix_a;        //!< pixel positions p(-1, -1..15)
  PixelPos pix_b;        //!< pixel position p(-1, -1)
  PixelPos pix_c;        //!< pixel position p( 0, -1)
  PixelPos pix_d;
  int      cr_MB_x = p_Vid->mb_cr_size_x;
  int      cr_MB_y = p_Vid->mb_cr_size_y;
  imgpel ***curr_mpr_16x16;

  short diff  [16];

  p_Vid->getNeighbour(currMB, -1 , -1 , p_Vid->mb_size[IS_CHROMA], &pix_b);
  p_Vid->getNeighbour(currMB, -1 ,  0 , p_Vid->mb_size[IS_CHROMA], &pix_d);
  p_Vid->getNeighbour(currMB,  0 , -1 , p_Vid->mb_size[IS_CHROMA], &pix_c);
  p_Vid->getNeighbour(currMB,  0 ,  0 , p_Vid->mb_size[IS_CHROMA], &pix_a);
  
  mb_available_up      = pix_c.available;
  mb_available_up_left = pix_b.available;
  mb_available_left    = pix_d.available;

  if(p_Inp->UseConstrainedIntraPred)
  {
    mb_available_up      = pix_c.available ? p_Vid->intra_block[pix_c.mb_addr] : 0;
    mb_available_left    = pix_d.available ? p_Vid->intra_block[pix_d.mb_addr] : 0;
    mb_available_up_left = pix_b.available ? p_Vid->intra_block[pix_b.mb_addr] : 0;
  }

  //printf("af pix_a %d %d\n", pix_a.pos_x, pix_a.pos_y);
  for (mode = DC_PRED_8; mode <= PLANE_8; ++mode)
  {
    if ((mode==VERT_PRED_8 && !mb_available_up) ||
      (mode==HOR_PRED_8 && (!mb_available_left)) ||
      (mode==PLANE_8 && (!mb_available_left || !mb_available_up || !mb_available_up_left)))
      continue;
    
    cost = weight_cost(enc_mb->lambda_mf[Q_PEL], p_Vid->mvbits[ mode ]); // exp golomb coding cost for mode signaling

    for (uv = 1; uv < 3; ++uv)
    {
      int pos_x = pix_a.pos_x;
      int pos_y = pix_a.pos_y;
      image = p_Vid->pImgOrg[uv];
      curr_mpr_16x16 = currSlice->mpr_16x16[uv];

      for (block_y=0; block_y<cr_MB_y; block_y+=4)
      {
        for (block_x=0; block_x<cr_MB_x; block_x+=4)
        {
          get_difference_4x4(&image[pos_y + block_y], &curr_mpr_16x16[mode][block_y], diff, pos_x, block_x);
          cost += p_Vid->distortion4x4(diff, min_cost);
          if (cost > min_cost) break;
        }
        if (cost > min_cost) break;
      }
      if (cost > min_cost) break;
    }

    if (cost < min_cost)
    {
      best_mode = mode;
      min_cost = cost;
    }
  }
  currMB->c_ipred_mode = (char) best_mode;
}

/*!
 **************************************************************************************
 * \brief
 *    RD Decision for Intra prediction mode of the chrominance layers of one macroblock
 **************************************************************************************
 */
void intra_chroma_RD_decision_mbaff (Macroblock *currMB, RD_PARAMS *enc_mb)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;
  int      i, j, k;
  imgpel** image;
  int      block_x, block_y;  
  int      mb_available_up;
  int      mb_available_left[2];
  int      mb_available_up_left;
  int      uv;
  int      mode;
  int      best_mode = DC_PRED_8;  //just an initialization here, should always be overwritten
  distblk  cost;
  // pick lowest cost prediction mode
  distblk  min_cost = DISTBLK_MAX;

  PixelPos pix_c;        //!< pixel position  p(0,-1)
  PixelPos pix_a[17];  //!< pixel positions p(-1, -1..15)
  int      cr_MB_x = p_Vid->mb_cr_size_x;
  int      cr_MB_y = p_Vid->mb_cr_size_y;
  imgpel *img_org, *img_prd;
  imgpel ***curr_mpr_16x16;
  short diff  [16];

  for (i=0;i<cr_MB_y+1;++i)
  {
    p_Vid->getNeighbour(currMB, -1 , i-1 , p_Vid->mb_size[IS_CHROMA], &pix_a[i]);
  }
  p_Vid->getNeighbour(currMB, 0 , -1 , p_Vid->mb_size[IS_CHROMA], &pix_c);
  
  mb_available_up                             = pix_c.available;
  mb_available_up_left                        = pix_a[0].available;
  mb_available_left[0] = mb_available_left[1] = pix_a[1].available;

  if(p_Inp->UseConstrainedIntraPred)
  {
    mb_available_up = pix_c.available ? p_Vid->intra_block[pix_c.mb_addr] : 0;
    for (i=1, mb_available_left[0] = 1; i < (cr_MB_y>>1) + 1; ++i)
      mb_available_left[0]  &= pix_a[i].available ? p_Vid->intra_block[pix_a[i].mb_addr]: 0;

    for (i=(cr_MB_y>>1) + 1, mb_available_left[1]=1; i < cr_MB_y + 1;++i)
      mb_available_left[1] &= pix_a[i].available ? p_Vid->intra_block[pix_a[i].mb_addr]: 0;

    mb_available_up_left = pix_a[0].available ? p_Vid->intra_block[pix_a[0].mb_addr]: 0;
  }

  for (i=0;i<cr_MB_y;++i)
  {
    p_Vid->getNeighbour(currMB, 0, i, p_Vid->mb_size[IS_CHROMA], &pix_a[i]);
  }
  if ( currSlice->mb_aff_frame_flag && p_Vid->field_mode )
  {
    for (i=0;i<cr_MB_y;++i)
    {
      pix_a[i].pos_y = pix_a[i].pos_y >> 1;
    }
  }

  for (mode=DC_PRED_8; mode<=PLANE_8; ++mode)
  {
    if ((mode==VERT_PRED_8 && !mb_available_up) ||
      (mode==HOR_PRED_8 && (!mb_available_left[0] || !mb_available_left[1])) ||
      (mode==PLANE_8 && (!mb_available_left[0] || !mb_available_left[1] || !mb_available_up || !mb_available_up_left)))
      continue;

    cost = 0;
    for (uv = 1; uv < 3; ++uv)
    {
      image = p_Vid->pImgOrg[uv];
      curr_mpr_16x16 = currSlice->mpr_16x16[uv];
      for (block_y=0; block_y<cr_MB_y; block_y+=4)
      {
        for (block_x=0; block_x<cr_MB_x; block_x+=4)
        {
          for (k=0,j=block_y; j<block_y+4; ++j)
          {
            img_prd = curr_mpr_16x16[mode][j];
            img_org = &image[pix_a[j].pos_y][pix_a[j].pos_x];

            for (i = block_x; i < block_x + 4; ++i)
              diff[k++] = img_org[i] - img_prd[i];
          }

          cost += p_Vid->distortion4x4(diff, min_cost);
          if (cost > min_cost) break;
        }
        if (cost > min_cost) break;
      }
      if (cost > min_cost) break;
    }

    //cost += (int) (enc_mb->lambda_me[Q_PEL] * p_Vid->mvbits[ mode ]); // exp golomb coding cost for mode signaling
    cost += weight_cost(enc_mb->lambda_mf[Q_PEL], p_Vid->mvbits[ mode ]); // exp golomb coding cost for mode signaling

    if (cost < min_cost)
    {
      best_mode = mode;
      min_cost = cost;
    }
  }
  currMB->c_ipred_mode = (char) best_mode;
}
