/*!
*************************************************************************************
* \file rdoq_cavlc.c
*
* \brief
*    Rate Distortion Optimized Quantization based on VCEG-AH21 related to CAVLC
*************************************************************************************
*/

#include "contributors.h"

#include <math.h>
#include <float.h>

#include "global.h"
#include "image.h"
#include "fmo.h"
#include "macroblock.h"
#include "mb_access.h"
#include "rdoq.h"
#include "block.h"

/*!
************************************************************************
* \brief
*    estimate VLC for Coeff Level
************************************************************************
*/
int estSyntaxElement_Level_VLC1(SyntaxElement *se)
{
  int level  = se->value1;
  int sign   = (level < 0 ? 1 : 0);
  int levabs = iabs(level);

  if (levabs < 8)
  {
    se->len = levabs * 2 + sign - 1;
  }
  else if (levabs < 16)
  {
    // escape code1
    se->len = 19;
  }
  else
  {
    // escape code2
    se->len = 28;
  }

  return (se->len);
}



/*!
************************************************************************
* \brief
*    estimate VLC for Coeff Level
************************************************************************
*/
int estSyntaxElement_Level_VLCN(SyntaxElement *se, int vlc)//, DataPartition *this_dataPart)
{
  int level = se->value1;
  int levabs = iabs(level) - 1;  

  int shift = vlc - 1;
  int escape = (15 << shift);

  if (levabs < escape)
  {
    se->len = ((levabs) >> shift) + 1 + vlc;
  }
  else
  {
    se->len = 28;
  }

  return (se->len);
}

int cmp(const void *arg1, const void *arg2)
{
  if(((levelDataStruct *)arg2)->levelDouble != ((levelDataStruct *)arg1)->levelDouble)
    return (((levelDataStruct *)arg2)->levelDouble - ((levelDataStruct *)arg1)->levelDouble);
  else
    return (((levelDataStruct *)arg1)->coeff_ctr - ((levelDataStruct *)arg2)->coeff_ctr);
}

/*!
************************************************************************
* \brief
*    estimate CAVLC bits
************************************************************************
*/
int est_CAVLC_bits (VideoParameters *p_Vid, int level_to_enc[16], int sign_to_enc[16], int nnz, int block_type)
{
  int           no_bits    = 0;
  SyntaxElement se;

  int coeff_ctr, scan_pos = 0;
  int k, level = 1, run = -1, vlcnum;
  int numcoeff = 0, lastcoeff = 0, numtrailingones = 0; 
  int numones = 0, totzeros = 0, zerosleft, numcoef;
  int numcoeff_vlc;
  int level_two_or_higher;
  int max_coeff_num = 0, cdc = (block_type == CHROMA_DC ? 1 : 0);
  int yuv = p_Vid->yuv_format - 1;
  static const int incVlc[] = {0, 3, 6, 12, 24, 48, 32768};  // maximum vlc = 6

  int  pLevel[16] = {0};
  int  pRun[16] = {0};

  static const int Token_lentab[3][4][17] = 
  {
    {
      { 1, 6, 8, 9,10,11,13,13,13,14,14,15,15,16,16,16,16},
      { 0, 2, 6, 8, 9,10,11,13,13,14,14,15,15,15,16,16,16},
      { 0, 0, 3, 7, 8, 9,10,11,13,13,14,14,15,15,16,16,16},
      { 0, 0, 0, 5, 6, 7, 8, 9,10,11,13,14,14,15,15,16,16}
    },
    {                                                  
      { 2, 6, 6, 7, 8, 8, 9,11,11,12,12,12,13,13,13,14,14},
      { 0, 2, 5, 6, 6, 7, 8, 9,11,11,12,12,13,13,14,14,14},
      { 0, 0, 3, 6, 6, 7, 8, 9,11,11,12,12,13,13,13,14,14},
      { 0, 0, 0, 4, 4, 5, 6, 6, 7, 9,11,11,12,13,13,13,14}
    },
    {                                                  
      { 4, 6, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 9,10,10,10,10},
      { 0, 4, 5, 5, 5, 5, 6, 6, 7, 8, 8, 9, 9, 9,10,10,10},
      { 0, 0, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,10,10,10},
      { 0, 0, 0, 4, 4, 4, 4, 4, 5, 6, 7, 8, 8, 9,10,10,10}
    }
  };

  static const int Totalzeros_lentab[TOTRUN_NUM][16] = 
  {
    { 1,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9},  
    { 3,3,3,3,3,4,4,4,4,5,5,6,6,6,6},  
    { 4,3,3,3,4,4,3,3,4,5,5,6,5,6},  
    { 5,3,4,4,3,3,3,4,3,4,5,5,5},  
    { 4,4,4,3,3,3,3,3,4,5,4,5},  
    { 6,5,3,3,3,3,3,3,4,3,6},  
    { 6,5,3,3,3,2,3,4,3,6},  
    { 6,4,5,3,2,2,3,3,6},  
    { 6,6,4,2,2,3,2,5},  
    { 5,5,3,2,2,2,4},  
    { 4,4,3,3,1,3},  
    { 4,4,2,1,3},  
    { 3,3,1,2},  
    { 2,2,1},  
    { 1,1},  
  };
  static const int Run_lentab[TOTRUN_NUM][16] = 
  {
    {1,1},
    {1,2,2},
    {2,2,2,2},
    {2,2,2,3,3},
    {2,2,3,3,3,3},
    {2,3,3,3,3,3,3},
    {3,3,3,3,3,3,3,4,5,6,7,8,9,10,11},
  };
  static const int Token_lentab_cdc[3][4][17] =
  {
    //YUV420
   {{ 2, 6, 6, 6, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    { 0, 1, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    { 0, 0, 3, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    { 0, 0, 0, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
    //YUV422
   {{ 1, 7, 7, 9, 9,10,11,12,13, 0, 0, 0, 0, 0, 0, 0, 0},
    { 0, 2, 7, 7, 9,10,11,12,12, 0, 0, 0, 0, 0, 0, 0, 0},
    { 0, 0, 3, 7, 7, 9,10,11,12, 0, 0, 0, 0, 0, 0, 0, 0},
    { 0, 0, 0, 5, 6, 7, 7,10,11, 0, 0, 0, 0, 0, 0, 0, 0}},
    //YUV444
   {{ 1, 6, 8, 9,10,11,13,13,13,14,14,15,15,16,16,16,16},
    { 0, 2, 6, 8, 9,10,11,13,13,14,14,15,15,15,16,16,16},
    { 0, 0, 3, 7, 8, 9,10,11,13,13,14,14,15,15,16,16,16},
    { 0, 0, 0, 5, 6, 7, 8, 9,10,11,13,14,14,15,15,16,16}}
  };
  static const int Totalzeros_lentab_cdc[3][TOTRUN_NUM][16] =
  {
    //YUV420
   {{ 1,2,3,3},
    { 1,2,2},
    { 1,1}},
    //YUV422
   {{ 1,3,3,4,4,4,5,5},
    { 3,2,3,3,3,3,3},
    { 3,3,2,2,3,3},
    { 3,2,2,2,3},
    { 2,2,2,2},
    { 2,2,1},
    { 1,1}},
    //YUV444
   {{ 1,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9},
    { 3,3,3,3,3,4,4,4,4,5,5,6,6,6,6},
    { 4,3,3,3,4,4,3,3,4,5,5,6,5,6},
    { 5,3,4,4,3,3,3,4,3,4,5,5,5},
    { 4,4,4,3,3,3,3,3,4,5,4,5},
    { 6,5,3,3,3,3,3,3,4,3,6},
    { 6,5,3,3,3,2,3,4,3,6},
    { 6,4,5,3,2,2,3,3,6},
    { 6,6,4,2,2,3,2,5},
    { 5,5,3,2,2,2,4},
    { 4,4,3,3,1,3},
    { 4,4,2,1,3},
    { 3,3,1,2},
    { 2,2,1},
    { 1,1}}
  };

  max_coeff_num = ( (block_type == CHROMA_DC) ? p_Vid->num_cdc_coeff : 
  ( (block_type == LUMA_INTRA16x16AC || block_type == CB_INTRA16x16AC || block_type == CR_INTRA16x16AC || block_type == CHROMA_AC) ? 15 : 16) );

  //convert zigzag scan to (run, level) pairs
  for (coeff_ctr = 0; coeff_ctr < max_coeff_num; coeff_ctr++)
  {
    run++;
    level = level_to_enc[coeff_ctr];
    if (level != 0)
    {
      pLevel[scan_pos] = isignab(level, sign_to_enc[coeff_ctr]);
      pRun  [scan_pos] = run;
      ++scan_pos;
      run = -1;                     // reset zero level counter
    }
  }

  level = 1;
  for(k = 0; (k < max_coeff_num) && level != 0; k++)    
  {
    level = pLevel[k]; // level
    run   = pRun[k];   // run

    if (level)
    {
      totzeros += run; // lets add run always (even if zero) to avoid conditional
      if (iabs(level) == 1)
      {
        numones ++;
        numtrailingones ++;
        numtrailingones = imin(numtrailingones, 3); // clip to 3
      }
      else
      {
        numtrailingones = 0;
      }
      numcoeff ++;
      lastcoeff = k;
    }
  }

  if (!cdc)
  {
    numcoeff_vlc = (nnz < 2) ? 0 : ((nnz < 4) ? 1 : ((nnz < 8) ? 2 : 3));
  }
  else
  {
    // chroma DC (has its own VLC)
    // numcoeff_vlc not relevant
    numcoeff_vlc = 0;
  }

  se.value1 = numcoeff;
  se.value2 = numtrailingones;
  se.len    = numcoeff_vlc; /* use len to pass vlcnum */

  if (!cdc)
  {
    if (se.len == 3)
      no_bits += 6;  // 4 + 2 bit FLC
    else
      no_bits += Token_lentab[se.len][se.value2][se.value1];
  }
  else
    no_bits += Token_lentab_cdc[yuv][se.value2][se.value1];  

  if (!numcoeff)
    return no_bits;
  else
  {
    if (numtrailingones)
      no_bits += numtrailingones;

    // encode levels
    level_two_or_higher = (numcoeff > 3 && numtrailingones == 3) ? 0 : 1;

    vlcnum = (numcoeff > 10 && numtrailingones < 3) ? 1 : 0;

    for (k = lastcoeff - numtrailingones; k >= 0; k--)
    {
      level = pLevel[k]; // level

      se.value1 = level;

      if (level_two_or_higher)
      {
        level_two_or_higher = 0;
        if (se.value1 > 0)
          se.value1 --;
        else
          se.value1 ++;
      }

      //    encode level
      if (vlcnum == 0)
        estSyntaxElement_Level_VLC1(&se);
      else
        estSyntaxElement_Level_VLCN(&se, vlcnum);

      // update VLC table
      if (iabs(level) > incVlc[vlcnum])
        vlcnum++;

      if ((k == lastcoeff - numtrailingones) && iabs(level) > 3)
        vlcnum = 2;

      no_bits  += se.len;
    }

    // encode total zeroes
    if (numcoeff < max_coeff_num)
    {
      se.value1 = totzeros;

      vlcnum = numcoeff-1;

      se.len = vlcnum;

      if (!cdc)
        no_bits += Totalzeros_lentab[se.len][se.value1];
      else
        no_bits += Totalzeros_lentab_cdc[yuv][se.len][se.value1];
    }

    // encode run before each coefficient
    zerosleft = totzeros;
    numcoef = numcoeff;
    for (k = lastcoeff; k >= 0; k--)
    {
      run = pRun[k]; // run

      se.value1 = run;

      // for last coeff, run is remaining totzeros
      // when zerosleft is zero, remaining coeffs have 0 run
      if ((!zerosleft) || (numcoeff <= 1 ))
        break;

      if (numcoef > 1 && zerosleft) 
      {
        vlcnum = imin(zerosleft - 1, RUNBEFORE_NUM_M1);
        se.len = vlcnum;
        no_bits += Run_lentab[se.len][se.value1];
        zerosleft -= run;
        numcoef --;
      }
    }
  }

  return no_bits;
}


/*!
****************************************************************************
* \brief
*    estimate run and level for CAVLC 
****************************************************************************
*/
void est_RunLevel_CAVLC(Macroblock *currMB, levelDataStruct *levelData, int *levelTrellis, int block_type, 
                        int b8, int b4, int coeff_num, double lambda)
{
  int k, lastnonzero = -1, coeff_ctr;
  int level_to_enc[16] = {0}, sign_to_enc[16] = {0};
  int cstat, bestcstat = 0; 
  int nz_coeff=0;
  double lagr, lagrAcc = 0, minlagr = 0;
  VideoParameters *p_Vid = currMB->p_Vid;

  int subblock_x = ((b8 & 0x1) == 0) ? (((b4 & 0x1) == 0) ? 0 : 1) : (((b4 & 0x1) == 0) ? 2 : 3); 
  // horiz. position for coeff_count context  
  int subblock_y = (b8 < 2) ? ((b4 < 2) ? 0 : 1) :((b4 < 2) ? 2 : 3); 
  // vert.  position for coeff_count context      
  int nnz; 
  levelDataStruct *dataLevel = &levelData[0];

  if (block_type != CHROMA_AC)
    nnz = predict_nnz(currMB, LUMA, subblock_x, subblock_y); 
  else
    nnz = predict_nnz_chroma(currMB, currMB->subblock_x >> 2, (currMB->subblock_y >> 2) + 4);

  for (coeff_ctr=0;coeff_ctr < coeff_num;coeff_ctr++)
  { 
    levelTrellis[coeff_ctr] = 0;

    for(k=0; k < dataLevel->noLevels; k++)
    {
      dataLevel->errLevel[k] /= 32768;
    }

    lagrAcc += dataLevel->errLevel[imax(0, dataLevel->noLevels - 1)];

    level_to_enc[coeff_ctr] = dataLevel->pre_level;
    sign_to_enc[coeff_ctr]  = dataLevel->sign;

    if(dataLevel->noLevels > 1)
    {
      dataLevel->coeff_ctr = coeff_ctr;
      lastnonzero = coeff_ctr;
    }
    else
      dataLevel->coeff_ctr = -1;
    dataLevel++;
  }

  if(lastnonzero != -1)
  {
    //sort the coefficients based on their absolute value
    qsort(levelData, lastnonzero + 1, sizeof(levelDataStruct), cmp);
    dataLevel = &levelData[lastnonzero];

    for(coeff_ctr = lastnonzero; coeff_ctr >= 0; coeff_ctr--) // go over all coeff
    {
      if(dataLevel->noLevels == 1)
      {
        dataLevel--;
        continue;
      }

      lagrAcc -= dataLevel->errLevel[dataLevel->noLevels-1];
      for(cstat=0; cstat<dataLevel->noLevels; cstat++) // go over all states of cur coeff k
      {
        level_to_enc[dataLevel->coeff_ctr] = dataLevel->level[cstat];
        lagr = lagrAcc + dataLevel->errLevel[cstat];
        lagr += lambda * est_CAVLC_bits( p_Vid, level_to_enc, sign_to_enc, nnz, block_type);
        if(cstat==0 || lagr<minlagr)
        {
          minlagr = lagr;
          bestcstat = cstat;
        }
      }

      lagrAcc += dataLevel->errLevel[bestcstat];
      level_to_enc[dataLevel->coeff_ctr] = dataLevel->level[bestcstat];
      dataLevel--;
    }

    for(coeff_ctr = 0; coeff_ctr <= lastnonzero; coeff_ctr++)
    {
      levelTrellis[coeff_ctr] = level_to_enc[coeff_ctr];
      if (level_to_enc[coeff_ctr] != 0)
        nz_coeff++;
    }
  }

  p_Vid->nz_coeff [p_Vid->current_mb_nr ][subblock_x][subblock_y] = nz_coeff;
}

/*!
****************************************************************************
* \brief
*    Initialize levelData 
****************************************************************************
*/
void init_trellis_data_4x4_CAVLC(Macroblock *currMB, int **tblock, int block_x, int qp_per, int qp_rem, LevelQuantParams **q_params,
                                 const byte *p_scan, levelDataStruct *dataLevel, int type)
{
  Slice *currSlice = currMB->p_Slice;
  int i, j, coeff_ctr; 
  int *m7;
  int end_coeff_ctr = ( ( type == LUMA_4x4 ) ? 16 : 15 );
  int q_bits = Q_BITS + qp_per; 
  int q_offset = ( 1 << (q_bits - 1) );
  int scaled_coeff, level, lowerInt, k;
  double err, estErr;


  for (coeff_ctr = 0; coeff_ctr < end_coeff_ctr; coeff_ctr++)
  {
    i = *p_scan++;  // horizontal position
    j = *p_scan++;  // vertical position

    m7 = &tblock[j][block_x + i];

    if (*m7 == 0)
    {
      dataLevel->levelDouble = 0;
      dataLevel->level[0] = 0;
      dataLevel->noLevels = 1;
      err = 0.0;
      dataLevel->errLevel[0] = 0.0;
      dataLevel->pre_level = 0;
      dataLevel->sign = 0;
    }
    else
    {
      estErr = ((double) estErr4x4[qp_rem][j][i]) / currSlice->norm_factor_4x4;

      scaled_coeff = iabs(*m7) * q_params[j][i].ScaleComp;
      dataLevel->levelDouble = scaled_coeff;
      level = (scaled_coeff >> q_bits);

      lowerInt = ((scaled_coeff - (level << q_bits)) < q_offset )? 1 : 0;
      
      dataLevel->level[0] = 0;
      if (level == 0 && lowerInt == 1)
      {
        dataLevel->noLevels = 1;
      }
      else if (level == 0 && lowerInt == 0)
      {
        dataLevel->level[1] = 1;
        dataLevel->noLevels = 2;
      }
      else if (level > 0 && lowerInt == 1)
      {
        dataLevel->level[1] = level;
        dataLevel->noLevels = 2;
      }
      else
      {
        dataLevel->level[1] = level;
        dataLevel->level[2] = level + 1;
        dataLevel->noLevels = 3;
      }

      for (k = 0; k < dataLevel->noLevels; k++)
      {
        err = (double)(dataLevel->level[k] << q_bits) - (double)scaled_coeff;
        dataLevel->errLevel[k] = (err * err * estErr); 
      }

      if(dataLevel->noLevels == 1)
        dataLevel->pre_level = 0;
      else
        dataLevel->pre_level = (iabs (*m7) * q_params[j][i].ScaleComp + q_params[j][i].OffsetComp) >> q_bits;
      dataLevel->sign = isign(*m7);
    }
    dataLevel++;
  }
}

/*!
****************************************************************************
* \brief
*    Initialize levelData for Luma DC
****************************************************************************
*/
void init_trellis_data_DC_CAVLC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, 
                         LevelQuantParams *q_params, const byte *p_scan, 
                         levelDataStruct *dataLevel)
{
  Slice *currSlice = currMB->p_Slice;
  int i, j, coeff_ctr, end_coeff_ctr = 16;
  int q_bits   = Q_BITS + qp_per + 1; 
  int q_offset = ( 1 << (q_bits - 1) );
  int scaled_coeff, level, lowerInt, k;
  int *m7;
  double err, estErr = (double) estErr4x4[qp_rem][0][0] / currSlice->norm_factor_4x4; // note that we could also use int64

  for (coeff_ctr = 0; coeff_ctr < end_coeff_ctr; coeff_ctr++)
  {
    i = *p_scan++;  // horizontal position
    j = *p_scan++;  // vertical position
    m7 = &tblock[j][i];

    if (*m7 == 0)
    {
      dataLevel->levelDouble = 0;
      dataLevel->level[0] = 0;
      dataLevel->noLevels = 1;
      err = 0.0;
      dataLevel->errLevel[0] = 0.0;
      dataLevel->pre_level = 0;
      dataLevel->sign = 0;
    }
    else
    {
      scaled_coeff = iabs(*m7) * q_params->ScaleComp;
      dataLevel->levelDouble = scaled_coeff;
      level = (scaled_coeff >> q_bits);

      lowerInt=( (scaled_coeff - (level<<q_bits)) < q_offset )? 1 : 0;

      dataLevel->level[0] = 0;    
      if (level == 0 && lowerInt == 1)
      {
        dataLevel->noLevels = 1;
      }
      else if (level == 0 && lowerInt == 0)
      {
        dataLevel->level[1] = 1;
        dataLevel->noLevels = 2;
      }
      else if (level > 0 && lowerInt == 1)
      {
        dataLevel->level[1] = level;
        dataLevel->noLevels = 2;
      }
      else
      {
        dataLevel->level[1] = level;
        dataLevel->level[2] = level + 1;
        dataLevel->noLevels = 3;
      }

      for (k = 0; k < dataLevel->noLevels; k++)
      {
        err = (double)(dataLevel->level[k] << q_bits) - (double)scaled_coeff;
        dataLevel->errLevel[k] = (err * err * estErr); 
      }

      if(dataLevel->noLevels == 1)
        dataLevel->pre_level = 0;
      else
        dataLevel->pre_level = (iabs (*m7) * q_params->ScaleComp + q_params->OffsetComp) >> q_bits;
      dataLevel->sign = isign(*m7);
    }
    dataLevel++;
  }
}

/*!
****************************************************************************
* \brief
*    Initialize levelData 
****************************************************************************
*/
void init_trellis_data_8x8_CAVLC(Macroblock *currMB, int **tblock, int block_x, int qp_per, int qp_rem, LevelQuantParams **q_params, 
                                 const byte *p_scan, levelDataStruct levelData[4][16])
{
  Slice *currSlice = currMB->p_Slice;
  int i, j, block, coeff_ctr;
  int *m7;
  int q_bits   = Q_BITS_8 + qp_per;
  int q_offset = ( 1 << (q_bits - 1) );
  double err, estErr;
  int scaled_coeff, level, lowerInt, k;
  
  levelDataStruct *dataLevel = &levelData[0][0];  

  for (coeff_ctr = 0; coeff_ctr < 16; coeff_ctr++)
  {
    for (block = 0; block < 4; block++)
    {
      i = *p_scan++;  // horizontal position
      j = *p_scan++;  // vertical position

      m7 = &tblock[j][block_x + i];

      dataLevel = &levelData[block][coeff_ctr];
      if (*m7 == 0)
      {
        dataLevel->levelDouble = 0;
        dataLevel->level[0] = 0;
        dataLevel->noLevels = 1;
        err = 0.0;
        dataLevel->errLevel[0] = 0.0;
        dataLevel->pre_level = 0;
        dataLevel->sign = 0;
      }
      else
      {
        estErr = (double) estErr8x8[qp_rem][j][i] / currSlice->norm_factor_8x8;

        scaled_coeff = iabs(*m7) * q_params[j][i].ScaleComp;
        dataLevel->levelDouble = scaled_coeff;
        level = (scaled_coeff >> q_bits);

        lowerInt = ((scaled_coeff - (level << q_bits)) < q_offset ) ? 1 : 0;

        dataLevel->level[0] = 0;
        if (level == 0 && lowerInt == 1)
        {
          dataLevel->noLevels = 1;
        }
        else if (level == 0 && lowerInt == 0)
        {
          dataLevel->level[1] = 1;
          dataLevel->noLevels = 2;
        }
        else if (level > 0 && lowerInt == 1)
        {
          if (level > 1)
          {
            dataLevel->level[1] = level - 1;
            dataLevel->level[2] = level;
            dataLevel->noLevels = 3;
          }
          else
          {
            dataLevel->level[1] = level;
            dataLevel->noLevels = 2;
          }
        }
        else
        {
          dataLevel->level[1] = level;
          dataLevel->level[2] = level + 1;
          dataLevel->noLevels = 3;
        }

        for (k = 0; k < dataLevel->noLevels; k++)
        {
          err = (double)(dataLevel->level[k] << q_bits) - (double)scaled_coeff;
          dataLevel->errLevel[k] = err * err * estErr; 
        }

        if(dataLevel->noLevels == 1)
          dataLevel->pre_level = 0;
        else
          dataLevel->pre_level = (iabs (*m7) * q_params[j][i].ScaleComp + q_params[j][i].OffsetComp) >> q_bits;
        dataLevel->sign = isign(*m7);
      }
    }
  }
}


