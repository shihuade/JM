/*!
 *************************************************************************************
 * \file get_block_otf.c
 *
 * \brief
 *    get predicted (chroma|luma) block with subpel interpolation computes on-the-fly
 *
 *
 *************************************************************************************
 */

#include "global.h"
#include "block.h"

#include "macroblock.h"
#include "image.h"
#include "mb_access.h"
#include "get_block_otf.h"
#include "refbuf_otf.h"


/*!
 ************************************************************************
 * \brief
 *    Integer positions
 ************************************************************************
 */ 

static void get_block_00(imgpel *block, imgpel *cur_img,  int block_size_y, int block_size_x, int shift_x)
{
  int j;
  
  for (j = 0; j < block_size_y; j += 1)
  { 
    memcpy(block, cur_img, block_size_x * sizeof(imgpel));
    block   += block_size_x ;
    cur_img += shift_x;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Qpel (1,0) horizontal
 ************************************************************************
 */ 
static void get_luma_10(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos , int max_imgpel_value)
{
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line, *cur_line;
  int i, j;
  int result;
  
  for (j = 0; j < block_size_y; j++)
  {
    cur_line = &(cur_imgY[j][x_pos]); 
    p0 = &cur_imgY[j][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;
    orig_line = block+j*block_size_x;            

    for (i = 0; i < block_size_x; i++)
    {        
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
      *orig_line = (imgpel) ((*orig_line + *(cur_line++) + 1 ) >> 1);
      orig_line++;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Half horizontal
 ************************************************************************
 */ 
static void get_luma_20(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos , int max_imgpel_value)
 {
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line;
  int i, j;
  int result;

  for (j = 0; j < block_size_y; j++)
  {
    p0 = &cur_imgY[j][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;
    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {        
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line++ = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Qpel (3,0) horizontal
 ************************************************************************
 */ 
static void get_luma_30(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos , int max_imgpel_value)
{
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line, *cur_line;
  int i, j;
  int result;
  
  for (j = 0; j < block_size_y; j++)
  {
    cur_line = &(cur_imgY[j][x_pos + 1]);
    p0 = &cur_imgY[j][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;
    orig_line = block + j*block_size_x ;            

    for (i = 0; i < block_size_x; i++)
    {        
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
      *orig_line = (imgpel) ((*orig_line + *(cur_line++) + 1 ) >> 1);
      orig_line++;
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Qpel vertical (0, 1)
 ************************************************************************
 */ 
static void get_luma_01(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line, *cur_line;
  int i, j;
  int result;
  int jj = 0;
  p0 = &(cur_imgY[ - 2][x_pos]);
  for (j = 0; j < block_size_y; j++)
  {                  
    p1 = p0 + shift_x;          
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    orig_line = block + j*block_size_x ;
    cur_line = &(cur_imgY[jj++][x_pos]);

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
      *orig_line = (imgpel) ((*orig_line + *(cur_line++) + 1 ) >> 1);
      orig_line++;
    }
    p0 = p1 - block_size_x;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Half vertical
 ************************************************************************
 */ 
static void get_luma_02(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line;
  int i, j;
  int result;
  p0 = &(cur_imgY[ - 2][x_pos]);
  for (j = 0; j < block_size_y; j++)
  {                  
    p1 = p0 + shift_x;          
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    orig_line = block + j*block_size_x ;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line++ = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
    }
    p0 = p1 - block_size_x;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Qpel vertical (0, 3)
 ************************************************************************
 */ 
static void get_luma_03(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line, *cur_line;
  int i, j;
  int result;
  int jj = 1;

  p0 = &(cur_imgY[ -2][x_pos]);
  for (j = 0; j < block_size_y; j++)
  {                  
    p1 = p0 + shift_x;          
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    orig_line = block + j*block_size_x ;
    cur_line = &(cur_imgY[jj++][x_pos]);

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
      *orig_line = (imgpel) ((*orig_line + *(cur_line++) + 1 ) >> 1);
      orig_line++;
    }
    p0 = p1 - block_size_x;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Hpel horizontal, Qpel vertical (2, 1)
 ************************************************************************
 */ 
static void get_luma_21(imgpel *block, imgpel **cur_imgY, int *tmp_res, int block_size_y, int block_size_x, int x_pos, int max_imgpel_value)
{
  int i, j;
  /* Vertical & horizontal interpolation */
  int *tmp_line;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  int    *x0, *x1, *x2, *x3, *x4, *x5;  
  imgpel *orig_line;  
  int result;      

  int jj = -2;

  for (j = 0; j < block_size_y + 5; j++)
  {
    p0 = &cur_imgY[jj++][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;          
    tmp_line  = tmp_res + j*block_size_x ;

    for (i = 0; i < block_size_x; i++)
    {        
      *(tmp_line++) = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));
    }
  }  

  jj = 2;
  for (j = 0; j < block_size_y; j++)
  {
    tmp_line  = tmp_res+ jj*block_size_x; jj++;
    x0 = tmp_res + j*block_size_x ;
    x1 = tmp_res + (j + 1)*block_size_x;
    x2 = tmp_res + (j + 2)*block_size_x;
    x3 = tmp_res + (j + 3)*block_size_x;
    x4 = tmp_res + (j + 4)*block_size_x;
    x5 = tmp_res + (j + 5)*block_size_x;
    orig_line = block + j*block_size_x ;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*x0++ + *x5++) - 5 * (*x1++ + *x4++) + 20 * (*x2++ + *x3++);

      *orig_line = (imgpel) iClip1(max_imgpel_value, ((result + 512)>>10));
      *orig_line = (imgpel) ((*orig_line + iClip1(max_imgpel_value, ((*(tmp_line++) + 16) >> 5)) + 1 )>> 1);
      orig_line++;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Hpel horizontal, Hpel vertical (2, 2)
 ************************************************************************
 */ 
static void get_luma_22(imgpel *block, imgpel **cur_imgY, int *tmp_res, int block_size_y, int block_size_x, int x_pos, int max_imgpel_value)
{
  int i, j;
  /* Vertical & horizontal interpolation */
  int *tmp_line;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  int    *x0, *x1, *x2, *x3, *x4, *x5;  
  imgpel *orig_line;  
  int result;      

  int jj = - 2;

  for (j = 0; j < block_size_y + 5; j++)
  {
    p0 = &cur_imgY[jj++][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;          
    tmp_line  = tmp_res + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {        
      *(tmp_line++) = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));
    }
  }

  for (j = 0; j < block_size_y; j++)
  {
    x0 = tmp_res + j*block_size_x ;
    x1 = tmp_res + (j + 1)*block_size_x;
    x2 = tmp_res + (j + 2)*block_size_x;
    x3 = tmp_res + (j + 3)*block_size_x;
    x4 = tmp_res + (j + 4)*block_size_x;
    x5 = tmp_res + (j + 5)*block_size_x;
    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*x0++ + *x5++) - 5 * (*x1++ + *x4++) + 20 * (*x2++ + *x3++);

      *(orig_line++) = (imgpel) iClip1(max_imgpel_value, ((result + 512)>>10));
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Hpel horizontal, Qpel vertical (2, 3)
 ************************************************************************
 */ 
static void get_luma_23(imgpel *block, imgpel **cur_imgY, int *tmp_res, int block_size_y, int block_size_x, int x_pos, int max_imgpel_value)
{
  int i, j;
  /* Vertical & horizontal interpolation */
  int *tmp_line;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  int    *x0, *x1, *x2, *x3, *x4, *x5;  
  imgpel *orig_line;  
  int result;      

  int jj = -2;

  for (j = 0; j < block_size_y + 5; j++)
  {
    p0 = &cur_imgY[jj++][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;          
    tmp_line  = tmp_res + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {        
      *(tmp_line++) = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));
    }
  }

  jj = 3;
  for (j = 0; j < block_size_y; j++)
  {
    tmp_line  = tmp_res+ jj*block_size_x; jj++;
    x0 = tmp_res + j*block_size_x ;
    x1 = tmp_res + (j + 1)*block_size_x;
    x2 = tmp_res + (j + 2)*block_size_x;
    x3 = tmp_res + (j + 3)*block_size_x;
    x4 = tmp_res + (j + 4)*block_size_x;
    x5 = tmp_res + (j + 5)*block_size_x;
    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*x0++ + *x5++) - 5 * (*x1++ + *x4++) + 20 * (*x2++ + *x3++);

      *orig_line = (imgpel) iClip1(max_imgpel_value, ((result + 512)>>10));
      *orig_line = (imgpel) ((*orig_line + iClip1(max_imgpel_value, ((*(tmp_line++) + 16) >> 5)) + 1 )>> 1);
      orig_line++;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Qpel horizontal, Hpel vertical (1, 2)
 ************************************************************************
 */ 
static void get_luma_12(imgpel *block, imgpel **cur_imgY, int *tmp_res, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  int i, j;
  int *tmp_line;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;        
  int    *x0, *x1, *x2, *x3, *x4, *x5;  
  imgpel *orig_line;  
  int result;      

  p0 = &(cur_imgY[ -2][x_pos - 2]);
  for (j = 0; j < block_size_y; j++)
  {                    
    p1 = p0 + shift_x;
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    tmp_line  = tmp_res + j*(block_size_x+5);

    for (i = 0; i < block_size_x + 5; i++)
    {
      *(tmp_line++)  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));
    }
    p0 = p1 - (block_size_x + 5);
  }

  for (j = 0; j < block_size_y; j++)
  {
    tmp_line  = tmp_res + j*(block_size_x+5) + 2 ;
    orig_line = block + j*block_size_x ;
    x0 = tmp_res + j*(block_size_x+5) ;
    x1 = x0 + 1;
    x2 = x1 + 1;
    x3 = x2 + 1;
    x4 = x3 + 1;
    x5 = x4 + 1;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(x0++) + *(x5++)) - 5 * (*(x1++) + *(x4++)) + 20 * (*(x2++) + *(x3++));

      *orig_line = (imgpel) iClip1(max_imgpel_value, ((result + 512)>>10));
      *orig_line = (imgpel) ((*orig_line + iClip1(max_imgpel_value, ((*(tmp_line++) + 16)>>5))+1)>>1);
      orig_line ++;
    }
  }  
}

/*!
 ************************************************************************
 * \brief
 *    Qpel horizontal, Hpel vertical (3, 2)
 ************************************************************************
 */ 
static void get_luma_32(imgpel *block, imgpel **cur_imgY, int *tmp_res, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  int i, j;
  int *tmp_line;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;        
  int    *x0, *x1, *x2, *x3, *x4, *x5;  
  imgpel *orig_line;  
  int result;      

  p0 = &(cur_imgY[ -2][x_pos - 2]);
  for (j = 0; j < block_size_y; j++)
  {                    
    p1 = p0 + shift_x;
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    tmp_line  = tmp_res + j*(block_size_x+5);

    for (i = 0; i < block_size_x + 5; i++)
    {
      *(tmp_line++)  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));
    }
    p0 = p1 - (block_size_x + 5);
  }

  for (j = 0; j < block_size_y; j++)
  {
    tmp_line  = tmp_res + j*(block_size_x+5) + 3;
    orig_line = block + j*block_size_x;
    x0 = tmp_res + j*(block_size_x+5);
    x1 = x0 + 1;
    x2 = x1 + 1;
    x3 = x2 + 1;
    x4 = x3 + 1;
    x5 = x4 + 1;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(x0++) + *(x5++)) - 5 * (*(x1++) + *(x4++)) + 20 * (*(x2++) + *(x3++));

      *orig_line = (imgpel) iClip1(max_imgpel_value, ((result + 512)>>10));
      *orig_line = (imgpel) ((*orig_line + iClip1(max_imgpel_value, ((*(tmp_line++) + 16)>>5))+1)>>1);
      orig_line ++;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Qpel horizontal, Qpel vertical (3, 3)
 ************************************************************************
 */ 
static void get_luma_33(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  int i, j;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line;  
  int result;      

  int jj = 1;

  for (j = 0; j < block_size_y; j++)
  {
    p0 = &cur_imgY[jj++][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;

    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {        
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *(orig_line++) = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
    }
  }

  p0 = &(cur_imgY[-2][x_pos + 1]);
  for (j = 0; j < block_size_y; j++)
  {        
    p1 = p0 + shift_x;
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line = (imgpel) ((*orig_line + iClip1(max_imgpel_value, ((result + 16) >> 5)) + 1) >> 1);
      orig_line++;
    }
    p0 = p1 - block_size_x ;
  }      
}


/*!
 ************************************************************************
 * \brief
 *    Qpel horizontal, Qpel vertical (1, 1)
 ************************************************************************
 */ 
static void get_luma_11(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  int i, j;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line;  
  int result;      

  int jj = 0;

  for (j = 0; j < block_size_y; j++)
  {
    p0 = &cur_imgY[jj++][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;

    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {        
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *(orig_line++) = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
    }
  }

  p0 = &(cur_imgY[-2][x_pos]);
  for (j = 0; j < block_size_y; j++)
  {        
    p1 = p0 + shift_x;
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line = (imgpel) ((*orig_line + iClip1(max_imgpel_value, ((result + 16) >> 5)) + 1) >> 1);
      orig_line++;
    }
    p0 = p1 - block_size_x ;
  }      
}

/*!
 ************************************************************************
 * \brief
 *    Qpel horizontal, Qpel vertical (1, 3)
 ************************************************************************
 */ 
static void get_luma_13(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  /* Diagonal interpolation */
  int i, j;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line;  
  int result;      

  int jj = 1;

  for (j = 0; j < block_size_y; j++)
  {
    p0 = &cur_imgY[jj++][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;

    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {        
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *(orig_line++) = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
    }
  }

  p0 = &(cur_imgY[-2][x_pos]);
  for (j = 0; j < block_size_y; j++)
  {        
    p1 = p0 + shift_x;
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line = (imgpel) ((*orig_line + iClip1(max_imgpel_value, ((result + 16) >> 5)) + 1) >> 1);
      orig_line++;
    }
    p0 = p1 - block_size_x ;
  }      
}

/*!
 ************************************************************************
 * \brief
 *    Qpel horizontal, Qpel vertical (3, 1)
 ************************************************************************
 */ 
static void get_luma_31(imgpel *block, imgpel **cur_imgY, int block_size_y, int block_size_x, int x_pos, int shift_x, int max_imgpel_value)
{
  /* Diagonal interpolation */
  int i, j;
  imgpel *p0, *p1, *p2, *p3, *p4, *p5;
  imgpel *orig_line;  
  int result;      

  int jj = 0;

  for (j = 0; j < block_size_y; j++)
  {
    p0 = &cur_imgY[jj++][x_pos - 2];
    p1 = p0 + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p4 = p3 + 1;
    p5 = p4 + 1;

    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {        
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *(orig_line++) = (imgpel) iClip1(max_imgpel_value, ((result + 16)>>5));
    }
  }

  p0 = &(cur_imgY[-2][x_pos + 1]);
  for (j = 0; j < block_size_y; j++)
  {        
    p1 = p0 + shift_x;
    p2 = p1 + shift_x;
    p3 = p2 + shift_x;
    p4 = p3 + shift_x;
    p5 = p4 + shift_x;
    orig_line = block + j*block_size_x;

    for (i = 0; i < block_size_x; i++)
    {
      result  = (*(p0++) + *(p5++)) - 5 * (*(p1++) + *(p4++)) + 20 * (*(p2++) + *(p3++));

      *orig_line = (imgpel) ((*orig_line + iClip1(max_imgpel_value, ((result + 16) >> 5)) + 1) >> 1);
      orig_line++;
    }
    p0 = p1 - block_size_x ;
  }      
}

/*!
 ************************************************************************
 * \brief
 *    Interpolation on-the-fly of 1/4 subpixel
 ************************************************************************
 */ 

static inline void get_block_luma_otf(  VideoParameters *p_Vid,  //!< video encoding parameters for current picture
                      imgpel*   mpred,         //!< array of prediction values (row by row)
                      int*   tmp_pred,         //!< array of temporary prediction values (row by row), used for some hal-pel interpolations
                      int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                      int    pic_pix_y,        //!< motion shifted vertical   coordinate of block
                      int    block_size_x,   //!< horizontal block size
                      int    block_size_y,   //!< vertical block size
                      StorablePicture *ref,    //!< reference picture list
                      int    pl                //!< plane
                    )
{
  imgpel **ref_block = (p_Vid->P444_joined && pl>PLANE_Y)? ref->imgUV[pl-1] : ref->imgY;
  int    dx = (pic_pix_x & 0x03);
  int    dy = (pic_pix_y & 0x03);
  int    x_pos = iClip3(-IMG_PAD_SIZE_X+2,  ref->size_x_pad-2, pic_pix_x>>2);
  int    y_pos = iClip3(-IMG_PAD_SIZE_Y+2, ref->size_y_pad-2, pic_pix_y>>2);

  if (dx == 0 && dy == 0)
    get_block_00(mpred, &(ref_block[y_pos][x_pos]), block_size_y, block_size_x, p_Vid->padded_size_x);
  else
  { /* other positions */
    if (dy == 0) /* No vertical interpolation */
    {         
      if (dx == 1)
        get_luma_10(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->max_imgpel_value);
      else if (dx == 2)
        get_luma_20(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->max_imgpel_value);
      else
        get_luma_30(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->max_imgpel_value);
    }
    else if (dx == 0) /* No horizontal interpolation */        
    {         
      if (dy == 1)
        get_luma_01(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
      else if (dy == 2)
        get_luma_02(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
      else
        get_luma_03(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
    }
    else if (dx == 2)  /* Vertical & horizontal interpolation */
    {  
      if (dy == 1)
        get_luma_21(mpred, &ref_block[y_pos], tmp_pred, block_size_y, block_size_x, x_pos, p_Vid->max_imgpel_value);
      else if (dy == 2)
        get_luma_22(mpred, &ref_block[y_pos], tmp_pred, block_size_y, block_size_x, x_pos, p_Vid->max_imgpel_value);
      else
        get_luma_23(mpred, &ref_block[y_pos], tmp_pred, block_size_y, block_size_x, x_pos, p_Vid->max_imgpel_value);
    }
    else if (dy == 2)
    {
      if (dx == 1)
        get_luma_12(mpred, &ref_block[y_pos], tmp_pred, block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
      else
        get_luma_32(mpred, &ref_block[y_pos], tmp_pred, block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
    }
    else
    {
      if (dx == 1)
      {
        if (dy == 1)
          get_luma_11(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
        else
          get_luma_13(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
      }
      else
      {
        if (dy == 1)
          get_luma_31(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
        else
          get_luma_33(mpred, &ref_block[y_pos], block_size_y, block_size_x, x_pos, p_Vid->padded_size_x, p_Vid->max_imgpel_value);
      }
    }
  }
}

void get_block_luma_otf_L2(  VideoParameters *p_Vid,  //!< video encoding parameters for current picture
                      imgpel*   mpred,         //!< array of prediction values (row by row)
                      int*   tmp_pred,         //!< array of temporary prediction values (row by row), used for some hal-pel interpolations
                      int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                      int    pic_pix_y,        //!< motion shifted vertical   coordinate of block
                      int    block_size_x,   //!< horizontal block size
                      int    block_size_y,   //!< vertical block size
                      StorablePicture *ref,    //!< reference picture list
                      int    pl                //!< plane
                    )
{
  get_block_luma_otf( p_Vid, mpred, tmp_pred, pic_pix_x, pic_pix_y, block_size_x, block_size_y, ref, pl ) ;
}

void get_block_luma_otf_L1(  VideoParameters *p_Vid,  //!< video encoding parameters for current picture
                      imgpel*   mpred,         //!< array of prediction values (row by row)
                      int*   tmp_pred,         //!< array of temporary prediction values (row by row), used for some hal-pel interpolations
                      int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                      int    pic_pix_y,        //!< motion shifted vertical   coordinate of block
                      int    block_size_x,   //!< horizontal block size
                      int    block_size_y,   //!< vertical block size
                      StorablePicture *ref,    //!< reference picture list
                      int    pl                //!< plane
                    )
{
  if( is_qpel( pic_pix_x, pic_pix_y, 0x03, 0x03 ) )
  {
    get_block_luma_otf( p_Vid, mpred, tmp_pred, pic_pix_x, pic_pix_y, block_size_x, block_size_y, ref, pl ) ;
  }
  else
  {
    int j; 
    imgpel *ref_line ;
    ref->p_curr_img_sub = (p_Vid->P444_joined && pl>PLANE_Y) ? (ref->imgUV_sub[pl-1]):(ref->imgY_sub); // select plane for 4:4:4 compatibility
    ref_line = UMVLine4X_otf ( ref, pic_pix_y, pic_pix_x ) ;
    for (j = 0; j < block_size_y; j++) 
    {
      memcpy(mpred, ref_line, block_size_x * sizeof(imgpel));
      ref_line += p_Vid->padded_size_x;
      mpred += block_size_x;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Chroma (0,X)
 ************************************************************************
 */ 
static void get_chroma_0X(imgpel *block, imgpel *cur_img, int span, int block_size_y, int block_size_x, int w00, int w01, int total_scale)
{
  imgpel *cur_row = cur_img;
  imgpel *nxt_row = cur_img + span;


  imgpel *cur_line, *cur_line_p1;
  imgpel *blk_line;
  int result;
  int i, j;
  for (j = 0; j < block_size_y; j++)
  {
      cur_line    = cur_row;
      cur_line_p1 = nxt_row;
      blk_line = block;
      block += block_size_x ;
      cur_row = nxt_row;
      nxt_row += span;
    for (i = 0; i < block_size_x; i++)
    {
      result = (w00 * *cur_line++ + w01 * *cur_line_p1++);
      *(blk_line++) = (imgpel) rshift_rnd_sf(result, total_scale);
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Chroma (X,0)
 ************************************************************************
 */ 
static void get_chroma_X0(imgpel *block, imgpel *cur_img, int span, int block_size_y, int block_size_x, int w00, int w10, int total_scale)
{
  imgpel *cur_row = cur_img;
  imgpel *cur_line, *cur_line_p1;
  imgpel *blk_line;
  int result;
  int i, j;
  for (j = 0; j < block_size_y; j++)
  {
    cur_line    = cur_row;
    cur_line_p1 = cur_line + 1;
    blk_line = block;
    block += block_size_x ;
    cur_row += span;
    for (i = 0; i < block_size_x; i++)
    {
      result = (w00 * *cur_line++ + w10 * *cur_line_p1++);
      //*(blk_line++) = (imgpel) iClip1(max_imgpel_value, rshift_rnd_sf(result, total_scale));
      *(blk_line++) = (imgpel) rshift_rnd_sf(result, total_scale);
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Chroma (X,X)
 ************************************************************************
 */ 
static void get_chroma_XX(imgpel *block, imgpel *cur_img, int span, int block_size_y, int block_size_x, int w00, int w01, int w10, int w11, int total_scale)
{ 
  imgpel *cur_row = cur_img;
  imgpel *nxt_row = cur_img + span;
  imgpel *cur_line, *cur_line_p1;
  imgpel *blk_line;
  int result;
  int i, j;
  for (j = 0; j < block_size_y; j++)
  {
    cur_line    = cur_row;
    cur_line_p1 = nxt_row;
    blk_line = block;
    block += block_size_x ;
    cur_row = nxt_row;
    nxt_row += span;
    for (i = 0; i < block_size_x; i++)
    {
      result  = (w00 * *(cur_line++) + w01 * *(cur_line_p1++));
      result += (w10 * *(cur_line  ) + w11 * *(cur_line_p1  ));
      *(blk_line++) = (imgpel) rshift_rnd_sf(result, total_scale);
    }
  }
  
}

static inline void get_block_chroma_otf (        VideoParameters *p_Vid, //!< video encoding parameters for current picture
                                   imgpel* mpred,           //!< array to store prediction values
                                   int*   tmp_pred,
                                   int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                                   int    pic_pix_y,        //!< motion shifted vertical  block
                                   int    block_size_x,   //!< horizontal block size
                                   int    block_size_y,   //!< vertical block size                                      
                                   StorablePicture *ref, //!< reference picture list
                                   int    uv   )         //!< chroma component
{
  int     dx = (pic_pix_x & p_Vid->chroma_mask_mv_x) ;
  int     dy = (pic_pix_y & p_Vid->chroma_mask_mv_y) ;
  int     x_pos = iClip3(-ref->pad_size_uv_x+1, ref->size_x_cr_pad-1, pic_pix_x >> p_Vid->chroma_shift_x) ;
  int     y_pos = iClip3(-ref->pad_size_uv_y+1, ref->size_y_cr_pad-1, pic_pix_y >> p_Vid->chroma_shift_y) ;
  int     total_scale = p_Vid->chroma_shift_x + p_Vid->chroma_shift_y ;
  
  
  imgpel  *ref_line = &(ref->imgUV[ uv-1 ][y_pos][x_pos]) ;

  if (dx == 0 && dy == 0) 
  {
    get_block_00(mpred, ref_line, block_size_y, block_size_x, p_Vid->cr_padded_size_x);
  }
  else 
  {
    short dxcur = (short) (p_Vid->chroma_mask_mv_x + 1 - dx);
    short dycur = (short) (p_Vid->chroma_mask_mv_y + 1 - dy);
    short w00 = dxcur * dycur;
    if (dx == 0)
    {
      short w01 = dxcur * dy;
      get_chroma_0X(mpred, ref_line, p_Vid->cr_padded_size_x, block_size_y, block_size_x, w00, w01, total_scale);
    }
    else if (dy == 0)
    {
      short w10 = dx * dycur;
      get_chroma_X0(mpred, ref_line, p_Vid->cr_padded_size_x, block_size_y, block_size_x, w00, w10, total_scale);
    }
    else
    {
      short w01 = dxcur * dy;
      short w10 = dx * dycur;
      short w11 = dx * dy;
      get_chroma_XX(mpred, ref_line, p_Vid->cr_padded_size_x, block_size_y, block_size_x, w00, w01, w10, w11, total_scale);
    }
  }
}


void get_block_chroma_otf_L2 (        VideoParameters *p_Vid, //!< video encoding parameters for current picture
                                   imgpel* mpred,           //!< array to store prediction values
                                   int*   tmp_pred,
                                   int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                                   int    pic_pix_y,        //!< motion shifted vertical  block
                                   int    block_size_x,   //!< horizontal block size
                                   int    block_size_y,   //!< vertical block size                                      
                                   StorablePicture *ref, //!< reference picture list
                                   int    uv   )         //!< chroma component
{
  get_block_chroma_otf ( p_Vid, mpred, tmp_pred, pic_pix_x, pic_pix_y, block_size_x, block_size_y, ref, uv );
}


void me_get_block_chroma_otf_L1 (        VideoParameters *p_Vid, //!< video encoding parameters for current picture
                                   imgpel* mpred,           //!< array to store prediction values
                                   int*   tmp_pred,
                                   int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                                   int    pic_pix_y,        //!< motion shifted vertical  block
                                   int    block_size_x,   //!< horizontal block size
                                   int    block_size_y,   //!< vertical block size                                      
                                   StorablePicture *ref, //!< reference picture list
                                   int    uv   )         //!< chroma component
{
  if( is_qpel( pic_pix_x, pic_pix_y, ref->chroma_mask_mv_x, ref->chroma_mask_mv_y) )
  {
    get_block_chroma_otf ( p_Vid, mpred, tmp_pred, pic_pix_x, pic_pix_y, block_size_x, block_size_y, ref, uv );
  }
  else
  {
    int     j;
    imgpel *ref_line = UMVLine8X_chroma_otf (ref, uv, pic_pix_y, pic_pix_x);
    for (j = 0; j < block_size_y; j++) 
    {
      memcpy(mpred, ref_line, block_size_x * sizeof(imgpel));
      ref_line += p_Vid->cr_padded_size_x;
      mpred += block_size_x;
    }
  }
}

void mc_get_block_chroma_otf_L1 (        VideoParameters *p_Vid, //!< video encoding parameters for current picture
                                   imgpel* mpred,           //!< array to store prediction values
                                   int*   tmp_pred,
                                   int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                                   int    pic_pix_y,        //!< motion shifted vertical  block
                                   int    block_size_x,   //!< horizontal block size
                                   int    block_size_y,   //!< vertical block size                                      
                                   StorablePicture *ref, //!< reference picture list
                                   int    uv   )         //!< chroma component
{
  if( is_qpel( pic_pix_x, pic_pix_y, ref->chroma_mask_mv_x, ref->chroma_mask_mv_y ) )
  {
    get_block_chroma_otf ( p_Vid, mpred, tmp_pred, pic_pix_x, pic_pix_y, block_size_x, block_size_y, ref, uv );
  }
  else
  {
    int     j;
    imgpel *ref_line = UMVLine4Xcr_otf (ref, uv, pic_pix_y, pic_pix_x);
    for (j = 0; j < block_size_y; j++) 
    {
      memcpy(mpred, ref_line, block_size_x * sizeof(imgpel));
      ref_line += p_Vid->cr_padded_size_x;
      mpred += block_size_x;
    }
  }
}

