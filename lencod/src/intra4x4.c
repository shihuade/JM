
/*!
 *************************************************************************************
 * \file intra4x4.c
 *
 * \brief
 *    Intra 4x4 mode functions
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

void generate_pred_error_4x4(imgpel **cur_img, imgpel **prd_img, imgpel **cur_prd, int **m7, int pic_opix_x, int block_x)
{
  int j, i, *m7_line;
  imgpel *cur_line, *prd_line;

  for (j = 0; j < BLOCK_SIZE; j++)
  {
    m7_line = &m7[j][block_x];
    cur_line = &cur_img[j][pic_opix_x];
    prd_line = prd_img[j];
    memcpy(&cur_prd[j][block_x], prd_line, BLOCK_SIZE * sizeof(imgpel));

    for (i = 0; i < BLOCK_SIZE; i++)
    {
      *m7_line++ = (int) (*cur_line++ - *prd_line++);
    }
  }        
}

// Notation for comments regarding prediction and predictors.
// The pels of the 4x4 block are labelled a..p. The predictor pels above
// are labelled A..H, from the left I..P, and from above left X, as follows:
//
//  X A B C D E F G H
//  I a b c d
//  J e f g h
//  K i j k l
//  L m n o p
//

// Predictor array index definitions
#define P_X (PredPel[0])
#define P_A (PredPel[1])
#define P_B (PredPel[2])
#define P_C (PredPel[3])
#define P_D (PredPel[4])
#define P_E (PredPel[5])
#define P_F (PredPel[6])
#define P_G (PredPel[7])
#define P_H (PredPel[8])
#define P_I (PredPel[9])
#define P_J (PredPel[10])
#define P_K (PredPel[11])
#define P_L (PredPel[12])

/*!
 ************************************************************************
 * \brief
 *    Vertical 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_vertical(imgpel **cur_pred, imgpel *PredPel)
{
  memcpy(cur_pred[0], &PredPel[1], BLOCK_SIZE * sizeof(imgpel));
  memcpy(cur_pred[1], &PredPel[1], BLOCK_SIZE * sizeof(imgpel));
  memcpy(cur_pred[2], &PredPel[1], BLOCK_SIZE * sizeof(imgpel));
  memcpy(cur_pred[3], &PredPel[1], BLOCK_SIZE * sizeof(imgpel));
}

/*!
 ************************************************************************
 * \brief
 *    Horizontal 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_horizontal(imgpel **cur_pred, imgpel *PredPel)
{
  int i;

  for (i=0; i < BLOCK_SIZE; i++)
  {
    cur_pred[i][0]  =
    cur_pred[i][1]  =
    cur_pred[i][2]  =
    cur_pred[i][3]  = (imgpel) (&P_I)[i];
  }
}

/*!
 ************************************************************************
 * \brief
 *    DC 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_dc(imgpel **cur_pred, imgpel *PredPel, int left_available, int up_available)
{
  int i, j, s0 = 0;
  if (up_available && left_available)
  {
    // no edge
    s0 = (P_A + P_B + P_C + P_D + P_I + P_J + P_K + P_L + 4) >> (BLOCK_SHIFT + 1);
  }
  else if (!up_available && left_available)
  {
    // upper edge
    s0 = (P_I + P_J + P_K + P_L + 2) >> BLOCK_SHIFT;;
  }
  else if (up_available && !left_available)
  {
    // left edge
    s0 = (P_A + P_B + P_C + P_D + 2) >> BLOCK_SHIFT;
  }
  else //if (!up_available && !left_available)
  {
    // top left corner, nothing to predict from
    s0 = P_A; // P_A already set to p_Vid->dc_pred_value;
  }

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < BLOCK_SIZE; i++)
      cur_pred[j][i] = (imgpel) s0;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Diagonal Down Left 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_downleft(imgpel **cur_pred, imgpel *PredPel)
{
  cur_pred[0][0] = (imgpel) ((P_A + P_C + ((P_B) << 1) + 2) >> 2);
  cur_pred[0][1] =
  cur_pred[1][0] = (imgpel) ((P_B + P_D + ((P_C) << 1) + 2) >> 2);
  cur_pred[0][2] =
  cur_pred[1][1] =
  cur_pred[2][0] = (imgpel) ((P_C + P_E + ((P_D) << 1) + 2) >> 2);
  cur_pred[0][3] =
  cur_pred[1][2] =
  cur_pred[2][1] =
  cur_pred[3][0] = (imgpel) ((P_D + P_F + ((P_E) << 1) + 2) >> 2);
  cur_pred[1][3] =
  cur_pred[2][2] =
  cur_pred[3][1] = (imgpel) ((P_E + P_G + ((P_F)<<1) + 2) >> 2);
  cur_pred[2][3] =
  cur_pred[3][2] = (imgpel) ((P_F + P_H + ((P_G)<<1) + 2) >> 2);
  cur_pred[3][3] = (imgpel) ((P_G + 3*(P_H) + 2) >> 2);
}

/*!
 ************************************************************************
 * \brief
 *    Diagonal Down Right 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_downright(imgpel **cur_pred, imgpel *PredPel)
{
  cur_pred[3][0] = (imgpel) ((P_L + 2*P_K + P_J + 2) >> 2);
  cur_pred[2][0] =
  cur_pred[3][1] = (imgpel) ((P_K + 2*P_J + P_I + 2) >> 2);
  cur_pred[1][0] =
  cur_pred[2][1] =
  cur_pred[3][2] = (imgpel) ((P_J + 2*P_I + P_X + 2) >> 2);
  cur_pred[0][0] =
  cur_pred[1][1] =
  cur_pred[2][2] =
  cur_pred[3][3] = (imgpel) ((P_I + 2*P_X + P_A + 2) >> 2);
  cur_pred[0][1] =
  cur_pred[1][2] =
  cur_pred[2][3] = (imgpel) ((P_X + 2*P_A + P_B + 2) >> 2);
  cur_pred[0][2] =
  cur_pred[1][3] = (imgpel) ((P_A + 2*P_B + P_C + 2) >> 2);
  cur_pred[0][3] = (imgpel) ((P_B + 2*P_C + P_D + 2) >> 2);
}


/*!
 ************************************************************************
 * \brief
 *    Vertical Left 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_vertleft(imgpel **cur_pred, imgpel *PredPel)
{
  cur_pred[0][0] = (imgpel) ((P_A + P_B + 1) >> 1);
  cur_pred[0][1] =
  cur_pred[2][0] = (imgpel) ((P_B + P_C + 1) >> 1);
  cur_pred[0][2] =
  cur_pred[2][1] = (imgpel) ((P_C + P_D + 1) >> 1);
  cur_pred[0][3] =
  cur_pred[2][2] = (imgpel) ((P_D + P_E + 1) >> 1);
  cur_pred[2][3] = (imgpel) ((P_E + P_F + 1) >> 1);
  cur_pred[1][0] = (imgpel) ((P_A + ((P_B)<<1) + P_C + 2) >> 2);
  cur_pred[1][1] =
  cur_pred[3][0] = (imgpel) ((P_B + ((P_C)<<1) + P_D + 2) >> 2);
  cur_pred[1][2] =
  cur_pred[3][1] = (imgpel) ((P_C + ((P_D)<<1) + P_E + 2) >> 2);
  cur_pred[1][3] =
  cur_pred[3][2] = (imgpel) ((P_D + ((P_E)<<1) + P_F + 2) >> 2);
  cur_pred[3][3] = (imgpel) ((P_E + ((P_F)<<1) + P_G + 2) >> 2);
}

/*!
 ************************************************************************
 * \brief
 *    Vertical Right 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_vertright(imgpel **cur_pred, imgpel *PredPel)
{
  cur_pred[0][0] =
  cur_pred[2][1] = (imgpel) ((P_X + P_A + 1) >> 1);
  cur_pred[0][1] =
  cur_pred[2][2] = (imgpel) ((P_A + P_B + 1) >> 1);
  cur_pred[0][2] =
  cur_pred[2][3] = (imgpel) ((P_B + P_C + 1) >> 1);
  cur_pred[0][3] = (imgpel) ((P_C + P_D + 1) >> 1);  
  cur_pred[1][0] =
  cur_pred[3][1] = (imgpel) ((P_I + 2*P_X + P_A + 2) >> 2);
  cur_pred[1][1] =
  cur_pred[3][2] = (imgpel) ((P_X + 2*P_A + P_B + 2) >> 2);
  cur_pred[1][2] =
  cur_pred[3][3] = (imgpel) ((P_A + 2*P_B + P_C + 2) >> 2);
  cur_pred[1][3] = (imgpel) ((P_B + 2*P_C + P_D + 2) >> 2);
  cur_pred[2][0] = (imgpel) ((P_X + 2*P_I + P_J + 2) >> 2);
  cur_pred[3][0] = (imgpel) ((P_I + 2*P_J + P_K + 2) >> 2);
}

/*!
 ************************************************************************
 * \brief
 *    Horizontal Down 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_hordown(imgpel **cur_pred, imgpel *PredPel)
{
  cur_pred[0][0] =
  cur_pred[1][2] = (imgpel) ((P_X + P_I + 1) >> 1);
  cur_pred[0][1] =
  cur_pred[1][3] = (imgpel) ((P_I + 2*P_X + P_A + 2) >> 2);
  cur_pred[0][2] = (imgpel) ((P_X + 2*P_A + P_B + 2) >> 2);
  cur_pred[0][3] = (imgpel) ((P_A + 2*P_B + P_C + 2) >> 2);
  cur_pred[1][0] =
  cur_pred[2][2] = (imgpel) ((P_I + P_J + 1) >> 1);
  cur_pred[1][1] =
  cur_pred[2][3] = (imgpel) ((P_X + 2*P_I + P_J + 2) >> 2);
  cur_pred[2][0] =
  cur_pred[3][2] = (imgpel) ((P_J + P_K + 1) >> 1);
  cur_pred[2][1] =
  cur_pred[3][3] = (imgpel) ((P_I + 2*P_J + P_K + 2) >> 2);
  cur_pred[3][0] = (imgpel) ((P_K + P_L + 1) >> 1);
  cur_pred[3][1] = (imgpel) ((P_J + 2*P_K + P_L + 2) >> 2);
}


/*!
 ************************************************************************
 * \brief
 *    Horizontal Up 4x4 Prediction
 ************************************************************************
 */
static inline void get_i4x4_horup(imgpel **cur_pred, imgpel *PredPel)
{
  cur_pred[0][0] = (imgpel) ((P_I + P_J + 1) >> 1);
  cur_pred[0][1] = (imgpel) ((P_I + 2*P_J + P_K + 2) >> 2);
  cur_pred[0][2] =
  cur_pred[1][0] = (imgpel) ((P_J + P_K + 1) >> 1);
  cur_pred[0][3] =
  cur_pred[1][1] = (imgpel) ((P_J + 2*P_K + P_L + 2) >> 2);
  cur_pred[1][2] =
  cur_pred[2][0] = (imgpel) ((P_K + P_L + 1) >> 1);
  cur_pred[1][3] =
  cur_pred[2][1] = (imgpel) ((P_K + 2*P_L + P_L + 2) >> 2);
  cur_pred[3][0] =
  cur_pred[2][2] =
  cur_pred[2][3] =
  cur_pred[3][1] =
  cur_pred[3][2] =
  cur_pred[3][3] = (imgpel) P_L;
}

/*!
 ************************************************************************
 * \brief
 *    Make intra 4x4 prediction according to all 9 prediction modes.
 *    The routine uses left and upper neighbouring points from
 *    previous coded blocks to do this (if available). Notice that
 *    inaccessible neighbouring points are signalled with a negative
 *    value in the predmode array .
 *
 *  \par Input:
 *     Starting point of current 4x4 block image posision
 *
 *  \par Output:
 *      none
 ************************************************************************
 */
void set_intrapred_4x4_mbaff(Macroblock *currMB, ColorPlane pl, int img_x,int img_y, int *left_available, int *up_available, int *all_available)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  int i;
  imgpel  *PredPel = currMB->intra4x4_pred[pl];  // array of predictor pels
  imgpel   **img_enc = p_Vid->enc_picture->p_curr_img;

  int ioff = (img_x & 15);
  int joff = (img_y & 15);

  PixelPos pix_a[4];
  PixelPos pix_b, pix_c, pix_d;

  int block_available_up;
  int block_available_left;
  int block_available_up_left;
  int block_available_up_right;
  int *mb_size = p_Vid->mb_size[IS_LUMA];

  for (i = 0; i < 4; i++)
  {
    p_Vid->getNeighbour(currMB, ioff -1 , joff + i , mb_size, &pix_a[i]);
  }

  p_Vid->getNeighbour(currMB, ioff     , joff -1 , mb_size, &pix_b);
  p_Vid->getNeighbour(currMB, ioff + 4 , joff -1 , mb_size, &pix_c);
  p_Vid->getNeighbour(currMB, ioff - 1 , joff -1 , mb_size, &pix_d);

  pix_c.available = pix_c.available && !((ioff==4) && ((joff==4)||(joff==12)));

  if (p_Inp->UseConstrainedIntraPred)
  {
    for (i=0, block_available_left=1; i<4;i++)
      block_available_left  &= pix_a[i].available ? p_Vid->intra_block[pix_a[i].mb_addr]: 0;
    block_available_up       = pix_b.available ? p_Vid->intra_block [pix_b.mb_addr] : 0;
    block_available_up_right = pix_c.available ? p_Vid->intra_block [pix_c.mb_addr] : 0;
    block_available_up_left  = pix_d.available ? p_Vid->intra_block [pix_d.mb_addr] : 0;
  }
  else
  {
    block_available_left     = pix_a[0].available;
    block_available_up       = pix_b.available;
    block_available_up_right = pix_c.available;
    block_available_up_left  = pix_d.available;
  }

  *left_available = block_available_left;
  *up_available   = block_available_up;
  *all_available  = block_available_up && block_available_left && block_available_up_left;

  i = (img_x & 15);

  // form predictor pels
  if (block_available_up)
  {
    memcpy(&PredPel[1], &img_enc[pix_b.pos_y][pix_b.pos_x], BLOCK_SIZE * sizeof(imgpel));
  }
  else
  {
    P_A = P_B = P_C = P_D = (imgpel) p_Vid->dc_pred_value;
  }

  if (block_available_up_right)
  {
    memcpy(&PredPel[5], &img_enc[pix_c.pos_y][pix_c.pos_x], BLOCK_SIZE * sizeof(imgpel));
  }
  else
  {
    P_E = P_F = P_G = P_H = P_D;
  }

  if (block_available_left)
  {
    P_I = img_enc[pix_a[0].pos_y][pix_a[0].pos_x];
    P_J = img_enc[pix_a[1].pos_y][pix_a[1].pos_x];
    P_K = img_enc[pix_a[2].pos_y][pix_a[2].pos_x];
    P_L = img_enc[pix_a[3].pos_y][pix_a[3].pos_x];
  }
  else
  {
    P_I = P_J = P_K = P_L = p_Vid->dc_pred_value;
  }

  if (block_available_up_left)
  {
    P_X = img_enc[pix_d.pos_y][pix_d.pos_x];
  }
  else
  {
    P_X = p_Vid->dc_pred_value;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Make intra 4x4 prediction according to all 9 prediction modes.
 *    The routine uses left and upper neighbouring points from
 *    previous coded blocks to do this (if available). Notice that
 *    inaccessible neighbouring points are signalled with a negative
 *    value in the predmode array .
 *
 *  \par Input:
 *     Starting point of current 4x4 block image posision
 *
 *  \par Output:
 *      none
 ************************************************************************
 */
void set_intrapred_4x4(Macroblock *currMB, ColorPlane pl, int img_x,int img_y, int *left_available, int *up_available, int *all_available)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  InputParameters *p_Inp = currMB->p_Inp;

  imgpel  *PredPel = currMB->intra4x4_pred[pl];  // array of predictor pels
  imgpel   **img_enc = p_Vid->enc_picture->p_curr_img;

  int ioff = (img_x & 15);
  int joff = (img_y & 15);

  PixelPos pix_a, pix_b, pix_c, pix_d;

  int block_available_up;
  int block_available_left;
  int block_available_up_left;
  int block_available_up_right;
  int *mb_size = p_Vid->mb_size[IS_LUMA];

  p_Vid->getNeighbour(currMB, ioff - 1, joff    , mb_size, &pix_a);
  p_Vid->getNeighbour(currMB, ioff    , joff - 1, mb_size, &pix_b);
  p_Vid->getNeighbour(currMB, ioff + 4, joff - 1, mb_size, &pix_c);
  p_Vid->getNeighbour(currMB, ioff - 1, joff - 1, mb_size, &pix_d);

  pix_c.available = pix_c.available && !((ioff==4) && ((joff==4)||(joff==12)));

  if (p_Inp->UseConstrainedIntraPred)
  {
    block_available_left     = pix_a.available ? p_Vid->intra_block [pix_a.mb_addr]: 0;
    block_available_up       = pix_b.available ? p_Vid->intra_block [pix_b.mb_addr] : 0;
    block_available_up_right = pix_c.available ? p_Vid->intra_block [pix_c.mb_addr] : 0;
    block_available_up_left  = pix_d.available ? p_Vid->intra_block [pix_d.mb_addr] : 0;
  }
  else
  {
    block_available_left     = pix_a.available;
    block_available_up       = pix_b.available;
    block_available_up_right = pix_c.available;
    block_available_up_left  = pix_d.available;
  }

  *left_available = block_available_left;
  *up_available   = block_available_up;
  *all_available  = block_available_up && block_available_left && block_available_up_left;

  // form predictor pels
  if (block_available_up)
  {
    memcpy(&PredPel[1], &img_enc[pix_b.pos_y][pix_b.pos_x], BLOCK_SIZE * sizeof(imgpel));
  }
  else
  {
    P_A = P_B = P_C = P_D = (imgpel) p_Vid->dc_pred_value;
  }

  if (block_available_up_right)
  {
    memcpy(&PredPel[5], &img_enc[pix_c.pos_y][pix_c.pos_x], BLOCK_SIZE * sizeof(imgpel));
  }
  else
  {
    P_E = P_F = P_G = P_H = P_D;
  }

  if (block_available_left)
  {
    int pos_y = pix_a.pos_y;
    int pos_x = pix_a.pos_x;
    P_I = img_enc[pos_y++][pos_x];
    P_J = img_enc[pos_y++][pos_x];
    P_K = img_enc[pos_y++][pos_x];
    P_L = img_enc[pos_y  ][pos_x];
  }
  else
  {
    P_I = P_J = P_K = P_L = p_Vid->dc_pred_value;
  }

  if (block_available_up_left)
  {
    P_X = img_enc[pix_d.pos_y][pix_d.pos_x];
  }
  else
  {
    P_X = p_Vid->dc_pred_value;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Generate 4x4 intra prediction block
 *
 *  \par Input:
 *     Starting point of current 4x4 block image posision
 *
 *  \par Output:
 *      none
 ************************************************************************
 */
void get_intrapred_4x4(Macroblock *currMB, ColorPlane pl, int i4x4_mode, int img_x, int img_y, int left_available, int up_available)
{
  imgpel        *PredPel = currMB->intra4x4_pred[pl];  // array of predictor pels
  imgpel ***curr_mpr_4x4 = currMB->p_Slice->mpr_4x4[pl];

  // Note that currently prediction values are always placed starting from (0,0) and not according to block position. 

  switch (i4x4_mode)
  {
  case VERT_PRED :    
    get_i4x4_vertical(curr_mpr_4x4[VERT_PRED], PredPel);
    break;
  case HOR_PRED :
    get_i4x4_horizontal(curr_mpr_4x4[HOR_PRED], PredPel);
    break;
  case DC_PRED :
    get_i4x4_dc(curr_mpr_4x4[DC_PRED], PredPel, left_available, up_available);
    break;
  case DIAG_DOWN_LEFT_PRED :
    get_i4x4_downleft(curr_mpr_4x4[DIAG_DOWN_LEFT_PRED], PredPel);
    break;
  case DIAG_DOWN_RIGHT_PRED :
    get_i4x4_downright(curr_mpr_4x4[DIAG_DOWN_RIGHT_PRED], PredPel);
    break;
  case VERT_RIGHT_PRED :
    get_i4x4_vertright(curr_mpr_4x4[VERT_RIGHT_PRED], PredPel);
    break;
  case HOR_DOWN_PRED :
    get_i4x4_hordown(curr_mpr_4x4[HOR_DOWN_PRED], PredPel);
    break;
  case VERT_LEFT_PRED :
    get_i4x4_vertleft(curr_mpr_4x4[VERT_LEFT_PRED], PredPel);
    break;
  case HOR_UP_PRED :
    get_i4x4_horup(curr_mpr_4x4[HOR_UP_PRED], PredPel);
    break;
  default:
    printf("invalid prediction mode \n");
    break;
  }
}
