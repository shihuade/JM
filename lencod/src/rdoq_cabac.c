/*!
 *************************************************************************************
 * \file rdoq_cabac.c
 *
 * \brief
 *    Rate Distortion Optimized Quantization based on VCEG-AH21 related to CABAC
 *************************************************************************************
 */

#include "contributors.h"

#include <math.h>
#include <float.h>

#include "global.h"
#include "cabac.h"
#include "image.h"
#include "fmo.h"
#include "macroblock.h"
#include "mb_access.h"
#include "rdoq.h"

#define RDOQ_SQ 0

static const int entropyBits[128]= 
{
     895,    943,    994,   1048,   1105,   1165,   1228,   1294, 
    1364,   1439,   1517,   1599,   1686,   1778,   1875,   1978, 
    2086,   2200,   2321,   2448,   2583,   2725,   2876,   3034, 
    3202,   3380,   3568,   3767,   3977,   4199,   4435,   4684, 
    4948,   5228,   5525,   5840,   6173,   6527,   6903,   7303, 
    7727,   8178,   8658,   9169,   9714,  10294,  10914,  11575, 
   12282,  13038,  13849,  14717,  15650,  16653,  17734,  18899, 
   20159,  21523,  23005,  24617,  26378,  28306,  30426,  32768, 
   32768,  35232,  37696,  40159,  42623,  45087,  47551,  50015, 
   52479,  54942,  57406,  59870,  62334,  64798,  67262,  69725, 
   72189,  74653,  77117,  79581,  82044,  84508,  86972,  89436, 
   91900,  94363,  96827,  99291, 101755, 104219, 106683, 109146, 
  111610, 114074, 116538, 119002, 121465, 123929, 126393, 128857, 
  131321, 133785, 136248, 138712, 141176, 143640, 146104, 148568, 
  151031, 153495, 155959, 158423, 160887, 163351, 165814, 168278, 
  170742, 173207, 175669, 178134, 180598, 183061, 185525, 187989
};


static int biari_no_bits(signed short symbol, BiContextTypePtr bi_ct )
{
  int ctx_state, estBits;

  symbol = (short) (symbol != 0);

  ctx_state = (symbol == bi_ct->MPS) ? 64 + bi_ct->state : 63 - bi_ct->state;
  estBits = entropyBits[127 - ctx_state];

  return(estBits);
}

static int biari_state(signed short symbol, BiContextTypePtr bi_ct )
{ 
  int ctx_state;

  symbol = (short) (symbol != 0);
  ctx_state = (symbol == bi_ct->MPS) ? 64 + bi_ct->state : 63 - bi_ct->state;

  return(ctx_state);
}

/*!
****************************************************************************
* \brief
*    estimate bit cost for each CBP bit
****************************************************************************
*/
static void est_CBP_block_bit (Macroblock* currMB, int type)
{
  Slice *currSlice = currMB->p_Slice;
  estBitsCabacStruct *cabacEstBits = &currSlice->estBitsCabac[type];
  int ctx;

  for (ctx=0; ctx<=3; ctx++)
  {
    cabacEstBits->blockCbpBits[ctx][0]=biari_no_bits(0, currSlice->tex_ctx->bcbp_contexts[type2ctx_bcbp[type]]+ctx);

    cabacEstBits->blockCbpBits[ctx][1]=biari_no_bits(1, currSlice->tex_ctx->bcbp_contexts[type2ctx_bcbp[type]]+ctx);
  }
}

/*!
****************************************************************************
* \brief
*    estimate CABAC bit cost for significant coefficient map
****************************************************************************
*/
static void est_significance_map(Macroblock* currMB, int type)
{
  Slice *currSlice = currMB->p_Slice;
  int   k;
  int   k1  = maxpos[type]-1;
#if ENABLE_FIELD_CTX
  int   fld = ( currSlice->structure!=FRAME || currMB->mb_field );
#else
  int   fld = 0;
#endif
  BiContextTypePtr  map_ctx   = currSlice->tex_ctx->map_contexts [fld][type2ctx_map [type]];
  BiContextTypePtr  last_ctx  = currSlice->tex_ctx->last_contexts[fld][type2ctx_last[type]];
  const byte *pos2ctxmap  = fld ? pos2ctx_map_int [type] : pos2ctx_map[type];
  const byte *pos2ctxlast = pos2ctx_last[type];

  estBitsCabacStruct *cabacEstBits = &currSlice->estBitsCabac[type];

  for (k = 0; k < k1; k++) // if last coeff is reached, it has to be significant
  {
    cabacEstBits->significantBits[pos2ctxmap[k]][0] = biari_no_bits  (0,  map_ctx + pos2ctxmap[k]);

    cabacEstBits->significantBits[pos2ctxmap[k]][1] = biari_no_bits  (1,  map_ctx + pos2ctxmap[k]);

    cabacEstBits->lastBits[pos2ctxlast[k]][0] = biari_no_bits(0, last_ctx+pos2ctxlast[k]);

    cabacEstBits->lastBits[pos2ctxlast[k]][1] = biari_no_bits(1, last_ctx+pos2ctxlast[k]);
  }
  // if last coeff is reached, it has to be significant
  cabacEstBits->significantBits[pos2ctxmap[k1]][0]=0;
  cabacEstBits->significantBits[pos2ctxmap[k1]][1]=0;
  cabacEstBits->lastBits[pos2ctxlast[k1]][0]=0;
  cabacEstBits->lastBits[pos2ctxlast[k1]][1]=0;
}

/*!
****************************************************************************
* \brief
*    estimate bit cost of significant coefficient
****************************************************************************
*/
static void est_significant_coefficients (Macroblock* currMB, int type)
{
  Slice *currSlice = currMB->p_Slice;
  int   ctx;
  int maxCtx = imin(4, max_c2[type]);
  estBitsCabacStruct *cabacEstBits = &currSlice->estBitsCabac[type];

  for (ctx=0; ctx<=4; ctx++){    
    cabacEstBits->greaterOneBits[0][ctx][0]=
      biari_no_bits (0, currSlice->tex_ctx->one_contexts[type2ctx_one[type]] + ctx);

    cabacEstBits->greaterOneBits[0][ctx][1]=
      biari_no_bits (1, currSlice->tex_ctx->one_contexts[type2ctx_one[type]] + ctx);
  }

  for (ctx=0; ctx<=maxCtx; ctx++){
    cabacEstBits->greaterOneBits[1][ctx][0]=
      biari_no_bits(0, currSlice->tex_ctx->abs_contexts[type2ctx_abs[type]] + ctx);

    cabacEstBits->greaterOneState[ctx]=biari_state(0, currSlice->tex_ctx->abs_contexts[type2ctx_abs[type]] + ctx);

    cabacEstBits->greaterOneBits[1][ctx][1]=
      biari_no_bits(1, currSlice->tex_ctx->abs_contexts[type2ctx_abs[type]] + ctx);
  }
}

/*!
****************************************************************************
* \brief
*    estimate unary exp golomb bit cost
****************************************************************************
*/
static int est_unary_exp_golomb_level_encode(Macroblock *currMB, unsigned int symbol, int ctx, int type)
{
  Slice *currSlice = currMB->p_Slice;
  estBitsCabacStruct *cabacEstBits = &currSlice->estBitsCabac[type];
  unsigned int l = symbol, k = 1;
  unsigned int exp_start = 13; // 15-2 : 0,1 level decision always sent
  int estBits;

  if (symbol==0)
  {
    estBits = cabacEstBits->greaterOneBits[1][ctx][0];
    return (estBits);
  }
  else
  {
    estBits = cabacEstBits->greaterOneBits[1][ctx][1];
    // this could be done using min of the two conditionals * value
    while (((--l)>0) && (++k <= exp_start))
    {
      estBits += cabacEstBits->greaterOneBits[1][ctx][1];
    }
    
    if (symbol < exp_start)
    {
      estBits += cabacEstBits->greaterOneBits[1][ctx][0];
    }
    else 
    {
      estBits += est_exp_golomb_encode_eq_prob(symbol - exp_start);
    }
  }
  return(estBits);
}

void precalculate_unary_exp_golomb_level(VideoParameters *p_Vid)
{
  int state, ctx_state0, ctx_state1, estBits0, estBits1, symbol;

  for (state=0; state<=63; state++)
  {
    // symbol 0 is MPS
    ctx_state0 = 64 + state;
    estBits0   = entropyBits[127-ctx_state0];
    ctx_state1 = 63 - state;
    estBits1   = entropyBits[127-ctx_state1];

    for (symbol = 0; symbol < MAX_PREC_COEFF; symbol++)
    {
      p_Vid->precalcUnaryLevelTab[ctx_state0][symbol] = est_unary_exp_golomb_level_bits(symbol, estBits0, estBits1);

      // symbol 0 is LPS
      p_Vid->precalcUnaryLevelTab[ctx_state1][symbol] =est_unary_exp_golomb_level_bits(symbol, estBits1, estBits0);
    }
  }
}

int est_unary_exp_golomb_level_bits(unsigned int symbol, int bits0, int bits1)
{
  unsigned int l,k;
  unsigned int exp_start = 13; // 15-2 : 0,1 level decision always sent
  int estBits;

  if (symbol == 0)
  {
    return (bits0);
  }
  else
  {
    estBits = bits1;
    l = symbol;
    k = 1;
    while (((--l)>0) && (++k <= exp_start))
    {
      estBits += bits1;
    }
    if (symbol < exp_start)
    {
      estBits += bits0;
    }
    else 
    {
      estBits += est_exp_golomb_encode_eq_prob(symbol - exp_start);
    }
  }
  return(estBits);
}

/*!
****************************************************************************
* \brief
*    estimate exp golomb bit cost 
****************************************************************************
*/
int est_exp_golomb_encode_eq_prob(unsigned int symbol)
{
  int k = 0, estBits = 0;

  for(;;)
  {
    if (symbol >= (unsigned int)(1<<k))   
    {
      estBits++;
      symbol -= (1 << k);
      k++;
    }
    else                  
    {
      estBits += (k + 1);  
      break;
    }
  }
  return(estBits);
}

/*!
****************************************************************************
* \brief
*   estimate bit cost for CBP, significant map and significant coefficients
****************************************************************************
*/
void estRunLevel_CABAC (Macroblock *currMB, int context) // writes CABAC run/level 
{
  est_CBP_block_bit  (currMB, context);      
  //===== encode significance map =====
  est_significance_map         (currMB, context);      
  //===== encode significant coefficients =====
  est_significant_coefficients (currMB, context);
}


/*!
****************************************************************************
* \brief
*    estimate CABAC CBP bits
****************************************************************************
*/
int est_write_and_store_CBP_block_bit(Macroblock* currMB, int type) 
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currMB->p_Vid;
  estBitsCabacStruct *cabacEstBits = &currSlice->estBitsCabac[type];  

  int y_ac        = (type==LUMA_16AC || type==LUMA_8x8 || type==LUMA_8x4 || type==LUMA_4x8 || type==LUMA_4x4
    || type==CB_16AC || type==CB_8x8 || type==CB_8x4 || type==CB_4x8 || type==CB_4x4
    || type==CR_16AC || type==CR_8x8 || type==CR_8x4 || type==CR_4x8 || type==CR_4x4);
  int y_dc        = (type==LUMA_16DC || type==CB_16DC || type==CR_16DC); 
  int u_ac        = (type==CHROMA_AC && !p_Vid->is_v_block);
  int v_ac        = (type==CHROMA_AC &&  p_Vid->is_v_block);
  int chroma_dc   = (type==CHROMA_DC || type==CHROMA_DC_2x4 || type==CHROMA_DC_4x4);
  int u_dc        = (chroma_dc && !p_Vid->is_v_block);
  int v_dc        = (chroma_dc &&  p_Vid->is_v_block);
  int j           = (y_ac || u_ac || v_ac ? currMB->subblock_y : 0);
  int i           = (y_ac || u_ac || v_ac ? currMB->subblock_x : 0);
  int bit;
  int default_bit =  (is_intra(currMB) ? 1 : 0);
  int upper_bit   = default_bit;
  int left_bit    = default_bit;
  int ctx, estBits = 0;

  int bit_pos_a   = 0;
  int bit_pos_b   = 0;

  PixelPos block_a, block_b;

  if (y_ac || y_dc)
  {
    get4x4Neighbour(currMB, i - 1, j   , p_Vid->mb_size[IS_LUMA], &block_a);
    get4x4Neighbour(currMB, i,     j -1, p_Vid->mb_size[IS_LUMA], &block_b);
    if (y_ac)
    {
      if (block_a.available)
        bit_pos_a = 4*block_a.y + block_a.x;
      if (block_b.available)
        bit_pos_b = 4*block_b.y + block_b.x;
    }
  }
  else
  {
    get4x4Neighbour(currMB, i - 1, j    , p_Vid->mb_size[IS_CHROMA], &block_a);
    get4x4Neighbour(currMB, i,     j - 1, p_Vid->mb_size[IS_CHROMA], &block_b);
    if (u_ac||v_ac)
    {
      if (block_a.available)
        bit_pos_a = (block_a.y << 2) + block_a.x;
      if (block_b.available)
        bit_pos_b = (block_b.y << 2) + block_b.x;
    }
  }

  bit = (y_dc ? 0 : y_ac ? 1 : u_dc ? 17 : v_dc ? 18 : u_ac ? 19 : 35);

  if (p_Vid->enc_picture->chroma_format_idc!=YUV444 || (currMB->p_Inp->separate_colour_plane_flag != 0))
  {
    if (type!=LUMA_8x8)
    {
      if (block_b.available)
      {
        if(p_Vid->mb_data[block_b.mb_addr].mb_type==IPCM)
          upper_bit = 1;
        else
          upper_bit = get_bit(p_Vid->mb_data[block_b.mb_addr].cbp_bits[0], bit + bit_pos_b);
      }

      if (block_a.available)
      {
        if(p_Vid->mb_data[block_a.mb_addr].mb_type==IPCM)
          left_bit = 1;
        else
          left_bit = get_bit(p_Vid->mb_data[block_a.mb_addr].cbp_bits[0], bit + bit_pos_a);
      }

      ctx = 2*upper_bit+left_bit;
      //===== encode symbol =====
      estBits = cabacEstBits->blockCbpBits[ctx][0] - cabacEstBits->blockCbpBits[ctx][1];
    }
  }
  else 
  {
    if (block_b.available)
    {
      if(p_Vid->mb_data[block_b.mb_addr].mb_type == IPCM)
        upper_bit=1;
      else
      {
        if(type==LUMA_8x8)
          upper_bit = get_bit(p_Vid->mb_data[block_b.mb_addr].cbp_bits_8x8[0], bit + bit_pos_b);
        else if (type==CB_8x8)
          upper_bit = get_bit(p_Vid->mb_data[block_b.mb_addr].cbp_bits_8x8[1], bit + bit_pos_b);
        else if (type==CR_8x8)
          upper_bit = get_bit(p_Vid->mb_data[block_b.mb_addr].cbp_bits_8x8[2], bit + bit_pos_b);
        else if ((type==CB_4x4)||(type==CB_4x8)||(type==CB_8x4)||(type==CB_16AC)||(type==CB_16DC))
          upper_bit = get_bit(p_Vid->mb_data[block_b.mb_addr].cbp_bits[1], bit + bit_pos_b);
        else if ((type==CR_4x4)||(type==CR_4x8)||(type==CR_8x4)||(type==CR_16AC)||(type==CR_16DC))
          upper_bit = get_bit(p_Vid->mb_data[block_b.mb_addr].cbp_bits[2], bit + bit_pos_b);
        else
          upper_bit = get_bit(p_Vid->mb_data[block_b.mb_addr].cbp_bits[0], bit + bit_pos_b);
      }
    }

    if (block_a.available)
    {
      if(p_Vid->mb_data[block_a.mb_addr].mb_type==IPCM)
        left_bit = 1;
      else
      {
        if(type==LUMA_8x8)
          left_bit = get_bit(p_Vid->mb_data[block_a.mb_addr].cbp_bits_8x8[0],bit + bit_pos_a);
        else if (type==CB_8x8)
          left_bit = get_bit(p_Vid->mb_data[block_a.mb_addr].cbp_bits_8x8[1],bit + bit_pos_a);
        else if (type==CR_8x8)
          left_bit = get_bit(p_Vid->mb_data[block_a.mb_addr].cbp_bits_8x8[2],bit + bit_pos_a);
        else if ((type==CB_4x4)||(type==CB_4x8)||(type==CB_8x4)||(type==CB_16AC)||(type==CB_16DC))
          left_bit = get_bit(p_Vid->mb_data[block_a.mb_addr].cbp_bits[1], bit + bit_pos_a);
        else if ((type==CR_4x4)||(type==CR_4x8)||(type==CR_8x4)||(type==CR_16AC)||(type==CR_16DC))
          left_bit = get_bit(p_Vid->mb_data[block_a.mb_addr].cbp_bits[2], bit + bit_pos_a);
        else
          left_bit = get_bit(p_Vid->mb_data[block_a.mb_addr].cbp_bits[0], bit + bit_pos_a);
      }
    }

    ctx = 2*upper_bit+left_bit;

    //===== encode symbol =====
    estBits = cabacEstBits->blockCbpBits[ctx][0] - cabacEstBits->blockCbpBits[ctx][1];
  }

  return(estBits);
}
/*!
****************************************************************************
* \brief
*    Rate distortion optimized trellis quantization
****************************************************************************
*/
void est_writeRunLevel_CABAC(Macroblock *currMB, levelDataStruct levelData[], int levelTabMin[], int type, double lambda, int kInit, int kStop, 
                             int noCoeff, int estCBP)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Slice *currSlice = currMB->p_Slice;
  estBitsCabacStruct *cabacEstBits = &currSlice->estBitsCabac[type];
  int   k, i;
  int   estBits;
  double lagr, lagrMin=0, lagrTabMin, lagrTab;
  int   c1 = 1, c2 = 0, c1Tab[3], c2Tab[3];
  int   iBest, levelTab[64];
  int   ctx, greater_one, last, maxK = maxpos[type];
  double   lagrAcc, lagrLastMin=0, lagrLast;
  int      kBest=0, kStart, first;
  levelDataStruct *dataLevel;
  int *last_bits, *significant_bits;

  memset(levelTabMin, 0, (maxK + 1) * sizeof(int));

  if (noCoeff > 0)
  {
    if (noCoeff > 1)
    {
      kStart = kInit; kBest = 0; first = 1; 
      lagrAcc = 0.0; 

      for (k = kStart; k <= kStop; k++)
      {
        lagrAcc += levelData[k].errLevel[0];
      }

      if (levelData[kStart].noLevels > 2)
      {
        last_bits = cabacEstBits->lastBits[pos2ctx_last[type][kStart]];
        lagrAcc -= levelData[kStart].errLevel[0];
        lagrLastMin = lambda * (last_bits[1] - last_bits[0]) + lagrAcc;

        kBest = kStart;
        kStart++;
        first = 0;
      }

      for (k = kStart; k <= kStop; k++)
      {        
        dataLevel = &levelData[k];
        significant_bits = cabacEstBits->significantBits[pos2ctx_map[type][k]];
        lagrMin  = dataLevel->errLevel[0] + lambda * significant_bits[0];
        lagrAcc -= dataLevel->errLevel[0];

        if (dataLevel->noLevels > 1)
        { 
          last_bits = cabacEstBits->lastBits[pos2ctx_last[type][k]];
          estBits = SIGN_BITS + significant_bits[1] + cabacEstBits->greaterOneBits[0][4][0];

          lagr = dataLevel->errLevel[1] + lambda * (estBits);

          lagrLast = lagr + lambda * last_bits[1] + lagrAcc;
          lagr     = lagr + lambda * last_bits[0];

          lagrMin = (lagr < lagrMin) ? lagr : lagrMin;

          if ((lagrLast < lagrLastMin) || (first == 1))
          {
            kBest = k;
            first = 0;
            lagrLastMin = lagrLast;
          }
        }
        lagrAcc += lagrMin;
      }
      kStart = kBest;
    }
    else
    {
      kStart = kStop;
    }

    lagrTabMin = 0.0;
    for (k = 0; k <= kStart; k++)
    {
      lagrTabMin += levelData[k].errLevel[0];
    }
    // Initial Lagrangian calculation
    lagrTab = 0.0;

    //////////////////////////

    lagrTabMin += (lambda*estCBP);
    iBest = 0; first = 1;
    for (k = kStart; k >= 0; k--)
    {
      significant_bits = cabacEstBits->significantBits[pos2ctx_map[type][k]];
      last_bits        = cabacEstBits->lastBits[pos2ctx_last[type][k]];
      dataLevel = &levelData[k];
      last = (k == kStart);

      if (!last)
      {
        lagrMin = dataLevel->errLevel[0] + lambda * significant_bits[0];
        iBest = 0;
        first = 0;
      }

      for (i = 1; i < dataLevel->noLevels; i++)
      {
        estBits = SIGN_BITS + significant_bits[1];
        estBits += last_bits[last];

        // greater than 1
        greater_one = (dataLevel->level[i] > 1);

        c1Tab[i] = c1;   
        c2Tab[i] = c2;

        ctx = imin(c1Tab[i], 4);  
        estBits += cabacEstBits->greaterOneBits[0][ctx][greater_one];

        // magnitude if greater than 1
        if (greater_one)
        {
          ctx = imin(c2Tab[i], max_c2[type]);
          if ( (dataLevel->level[i] - 2) < MAX_PREC_COEFF)
          {
            estBits += p_Vid->precalcUnaryLevelTab[cabacEstBits->greaterOneState[ctx]][dataLevel->level[i] - 2];
          }
          else
          {
            estBits += est_unary_exp_golomb_level_encode(currMB, dataLevel->level[i] - 2, ctx, type);
          }

          c1Tab[i] = 0;
          c2Tab[i]++;
        }
        else if (c1Tab[i])
        {
          c1Tab[i]++;
        }

        lagr = dataLevel->errLevel[i] + lambda * estBits;
        if ((lagr < lagrMin) || (first == 1))
        {
          iBest = i;
          lagrMin=lagr;
          first = 0;
        }
      }

      if (iBest > 0)
      {
        c1 = c1Tab[iBest]; 
        c2 = c2Tab[iBest];
      }

      levelTab[k] = dataLevel->level[iBest];
      lagrTab += lagrMin;
    }
    ///////////////////////////////////

    if (lagrTab < lagrTabMin)
    {
      memcpy(levelTabMin, levelTab, (kStart + 1) * sizeof(int));
    }
  }
}


/*!
****************************************************************************
* \brief
*    Initialize levelData 
****************************************************************************
*/
int init_trellis_data_4x4_CABAC(Macroblock *currMB, int **tblock, 
                                struct quant_methods *q_method, const byte *p_scan, 
                                levelDataStruct *dataLevel, int* kStart, int* kStop, int type)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  LevelQuantParams **q_params_4x4 = q_method->q_params;
  QuantParameters *p_Quant = p_Vid->p_Quant;
  int  qp = q_method->qp;
  int   qp_per = p_Quant->qp_per_matrix[qp];
  int   qp_rem = p_Quant->qp_rem_matrix[qp];
  int   block_x = q_method->block_x;

  Slice *currSlice = currMB->p_Slice;
  int noCoeff = 0;
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
      dataLevel->level[0] = 0;
      dataLevel->levelDouble = 0;
      dataLevel->errLevel[0] = 0.0;
      dataLevel->noLevels = 1;
      //err = 0.0;      
    }
    else
    {
      estErr = ((double) estErr4x4[qp_rem][j][i]) / currSlice->norm_factor_4x4;

      scaled_coeff = iabs(*m7) * q_params_4x4[j][i].ScaleComp;
      dataLevel->levelDouble = scaled_coeff;
      level = (scaled_coeff >> q_bits);

      lowerInt = ((scaled_coeff - (level << q_bits)) < q_offset )? 1 : 0;

#if RDOQ_SQ
      dataLevel->level[0] = max(0, level - 1);
#else
      dataLevel->level[0] = 0;
#endif
      if (level == 0)
      {
        if (lowerInt == 1)
        {
          dataLevel->noLevels = 1;
        }
        else
        {
          dataLevel->level[1] = 1;
          dataLevel->noLevels = 2;
          *kStop = coeff_ctr;
          noCoeff++;
        }
      }
      else if (lowerInt == 1)
      {
        dataLevel->level[1] = level;
        dataLevel->noLevels = 2;
        *kStop = coeff_ctr;
        noCoeff++;
      }
      else
      {
        dataLevel->level[1] = level;
        dataLevel->level[2] = level + 1;
        dataLevel->noLevels = 3;
        *kStop  = coeff_ctr;
        *kStart = coeff_ctr;
        noCoeff++;
      }

      for (k = 0; k < dataLevel->noLevels; k++)
      {
        err = (double)(dataLevel->level[k] << q_bits) - (double)scaled_coeff;
        dataLevel->errLevel[k] = (err * err * estErr); 
      }
    }

    dataLevel++;
  }
  return (noCoeff);
}

/*
****************************************************************************
* \brief
*    Initialize levelData 
****************************************************************************
*/
int init_trellis_data_8x8_CABAC(Macroblock *currMB, int **tblock, int block_x, int qp_per, int qp_rem, LevelQuantParams **q_params, const byte *p_scan, 
                      levelDataStruct *dataLevel, int* kStart, int* kStop)
{
  Slice *currSlice = currMB->p_Slice;
  int noCoeff = 0;
  int *m7;
  int i, j, coeff_ctr, end_coeff_ctr = 64;
  int q_bits = Q_BITS_8 + qp_per;
  int q_offset = ( 1 << (q_bits - 1) );
  double err, estErr; 
  int level, lowerInt, k;
  int scaled_coeff;
  
  for (coeff_ctr = 0; coeff_ctr < end_coeff_ctr; coeff_ctr++)
  {
    i = *p_scan++;  // horizontal position
    j = *p_scan++;  // vertical position
    
    m7 = &tblock[j][block_x + i];

    if (*m7 == 0)
    {
      dataLevel->level[0] = 0;
      dataLevel->levelDouble = 0;      
      dataLevel->errLevel[0] = 0.0;
      dataLevel->noLevels = 1;
      //err = 0.0;
    }
    else
    {
      estErr = (double) estErr8x8[qp_rem][j][i] / currSlice->norm_factor_8x8;

      scaled_coeff = iabs(*m7) * q_params[j][i].ScaleComp;
      dataLevel->levelDouble = scaled_coeff;
      level = (scaled_coeff >> q_bits);

      lowerInt = ((scaled_coeff - (level << q_bits)) < q_offset )? 1 : 0;

#if RDOQ_SQ
      dataLevel->level[0] = max(0, level - 1);
#else
      dataLevel->level[0] = 0;
#endif
      if (level == 0)
      {
        if (lowerInt == 1)
        {
          dataLevel->noLevels = 1;
        }
        else
        {
          dataLevel->level[1] = 1;
          dataLevel->noLevels = 2;
          *kStop = coeff_ctr;
          noCoeff++;
        }
      }
      else if (lowerInt == 1)
      {
        dataLevel->level[1] = level;
        dataLevel->noLevels = 2;
        *kStop = coeff_ctr;
        noCoeff++;
      }
      else
      {
        dataLevel->level[1] = level;
        dataLevel->level[2] = level + 1;
        dataLevel->noLevels = 3;
        *kStop  = coeff_ctr;
        *kStart = coeff_ctr;
        noCoeff++;
      }

      for (k = 0; k < dataLevel->noLevels; k++)
      {
        err = (double)(dataLevel->level[k] << q_bits) - (double)scaled_coeff;
        dataLevel->errLevel[k] = (err * err * estErr); 
      }
    }
 
    dataLevel++;
  }
  return (noCoeff);
}

/*!
****************************************************************************
* \brief
*    Initialize levelData for Luma DC
****************************************************************************
*/
int init_trellis_data_DC_CABAC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, 
                         LevelQuantParams *q_params, const byte *p_scan, 
                         levelDataStruct *dataLevel, int* kStart, int* kStop)
{
  Slice *currSlice = currMB->p_Slice;
  int noCoeff = 0;
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
      dataLevel->level[0] = 0;
      dataLevel->levelDouble = 0;    
      dataLevel->errLevel[0] = 0.0;
      dataLevel->noLevels = 1;
      //err = 0.0;  
    }
    else
    {
      scaled_coeff = iabs(*m7) * q_params->ScaleComp;
      dataLevel->levelDouble = scaled_coeff;
      level = (scaled_coeff >> q_bits);

      lowerInt=( (scaled_coeff - (level<<q_bits)) < q_offset )? 1 : 0;

#if RDOQ_SQ
      dataLevel->level[0] = max(0, level - 1);
#else
      dataLevel->level[0] = 0;
#endif
      if (level == 0)
      {
        if (lowerInt == 1)
        {
          dataLevel->noLevels = 1;
        }
        else
        {
          dataLevel->level[1] = 1;
          dataLevel->noLevels = 2;
          *kStop = coeff_ctr;
          noCoeff++;
        }
      }
      else if (lowerInt == 1)
      {
        dataLevel->level[1] = level;
        dataLevel->noLevels = 2;
        *kStop = coeff_ctr;
        noCoeff++;
      }
      else
      {
        dataLevel->level[1] = level;
        dataLevel->level[2] = level + 1;
        dataLevel->noLevels = 3;
        *kStop  = coeff_ctr;
        *kStart = coeff_ctr;
        noCoeff++;
      }

      for (k = 0; k < dataLevel->noLevels; k++)
      {
        err = (double)(dataLevel->level[k] << q_bits) - (double)scaled_coeff;
        dataLevel->errLevel[k] = (err * err * estErr); 
      }
    }
    dataLevel++;
  }
  return (noCoeff);
}



