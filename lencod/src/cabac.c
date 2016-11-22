/*!
 *************************************************************************************
 * \file cabac.c
 *
 * \brief
 *    CABAC entropy coding routines
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Detlev Marpe
 **************************************************************************************
 */

#include "global.h"

#include "cabac.h"
#include "biariencode.h"
#include "image.h"
#include "mb_access.h"

#if TRACE
  #define CABAC_TRACE if (dp->bitstream->trace_enabled) trace2out_cabac (se)
#else
  #define CABAC_TRACE
#endif

/***********************************************************************
                   Constant declarations
***********************************************************************
*/



//===== position -> ctx for MAP =====
//--- zig-zag scan ----
static const byte  pos2ctx_map8x8 [] = {
  0,  1,  2,  3,  4,  5,  5,  4,  4,  3,  3,  4,  4,  4,  5,  5,
  4,  4,  4,  4,  3,  3,  6,  7,  7,  7,  8,  9, 10,  9,  8,  7,
  7,  6, 11, 12, 13, 11,  6,  7,  8,  9, 14, 10,  9,  8,  6, 11,
  12, 13, 11,  6,  9, 14, 10,  9, 11, 12, 13, 11 ,14, 10, 12, 14
}; // 15 CTX

static const byte  pos2ctx_map8x4 [] = {
  0,  1,  2,  3,  4,  5,  7,  8,  9, 10, 11,  9,  8,  6,  7,  8,
  9, 10, 11,  9,  8,  6, 12,  8,  9, 10, 11,  9, 13, 13, 14, 14
}; // 15 CTX

static const byte  pos2ctx_map4x4 [] = {
  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 14
}; // 15 CTX

static const byte  pos2ctx_map2x4c[] = {
  0,  0,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2
}; // 15 CTX

static const byte  pos2ctx_map4x4c[] = {
  0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2
}; // 15 CTX

const byte* pos2ctx_map    [] = {
  pos2ctx_map4x4, pos2ctx_map4x4, pos2ctx_map8x8, pos2ctx_map8x4,
  pos2ctx_map8x4, pos2ctx_map4x4, pos2ctx_map4x4, pos2ctx_map4x4,
  pos2ctx_map2x4c, pos2ctx_map4x4c, 
  pos2ctx_map4x4, pos2ctx_map4x4, pos2ctx_map8x8,pos2ctx_map8x4,
  pos2ctx_map8x4, pos2ctx_map4x4,  //Cb component
  pos2ctx_map4x4, pos2ctx_map4x4, pos2ctx_map8x8,pos2ctx_map8x4,
  pos2ctx_map8x4,pos2ctx_map4x4  //Cr component
};

//--- interlace scan ----
//Taken from ABT
static const byte  pos2ctx_map8x8i[] = {
  0,  1,  1,  2,  2,  3,  3,  4,  5,  6,  7,  7,  7,  8,  4,  5,
  6,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 11, 12, 11,
  9,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 13, 13,  9,
  9, 10, 10,  8, 13, 13,  9,  9, 10, 10, 14, 14, 14, 14, 14, 14
}; // 15 CTX

static const byte  pos2ctx_map8x4i[] = { 
  0,  1,  2,  3,  4,  5,  6,  3,  4,  5,  6,  3,  4,  7,  6,  8,
  9,  7,  6,  8,  9, 10, 11, 12, 12, 10, 11, 13, 13, 14, 14, 14
}; // 15 CTX

static const byte  pos2ctx_map4x8i[] = {
  0,  1,  1,  1,  2,  3,  3,  4,  4,  4,  5,  6,  2,  7,  7,  8,
  8,  8,  5,  6,  9, 10, 10, 11, 11, 11, 12, 13, 13, 14, 14, 14
}; // 15 CTX

const byte* pos2ctx_map_int[] = {
  pos2ctx_map4x4, pos2ctx_map4x4, pos2ctx_map8x8i,pos2ctx_map8x4i,
  pos2ctx_map4x8i,pos2ctx_map4x4, pos2ctx_map4x4, pos2ctx_map4x4,
  pos2ctx_map2x4c, pos2ctx_map4x4c,
  //444_TEMP_NOTE: the followings are addded for the 4:4:4 common mode};
  pos2ctx_map4x4, pos2ctx_map4x4, pos2ctx_map8x8i,pos2ctx_map8x4i,
  pos2ctx_map8x4i,pos2ctx_map4x4, //Cb component
  pos2ctx_map4x4, pos2ctx_map4x4, pos2ctx_map8x8i,pos2ctx_map8x4i,
  pos2ctx_map8x4i,pos2ctx_map4x4  //Cr component}
};  


//===== position -> ctx for LAST =====
static const byte  pos2ctx_last8x8 [] = {
  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
  3,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  4,  4,  4,
  5,  5,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8
}; //  9 CTX

static const byte  pos2ctx_last8x4 [] = {
  0,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,
  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,  6,  6,  7,  7,  8,  8
}; //  9 CTX

static const byte  pos2ctx_last4x4 [] = {
  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
}; // 15 CTX

static const byte  pos2ctx_last2x4c[] = {
  0,  0,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2
}; // 15 CTX

static const byte  pos2ctx_last4x4c[] = { 
  0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2
}; // 15 CTX

const byte* pos2ctx_last    [] = {
  pos2ctx_last4x4, pos2ctx_last4x4, pos2ctx_last8x8, pos2ctx_last8x4,
  pos2ctx_last8x4, pos2ctx_last4x4, pos2ctx_last4x4, pos2ctx_last4x4,
  pos2ctx_last2x4c, pos2ctx_last4x4c,
  pos2ctx_last4x4, pos2ctx_last4x4, pos2ctx_last8x8,pos2ctx_last8x4,
  pos2ctx_last8x4, pos2ctx_last4x4,  //Cb component
  pos2ctx_last4x4, pos2ctx_last4x4, pos2ctx_last8x8,pos2ctx_last8x4,
  pos2ctx_last8x4, pos2ctx_last4x4 //Cr component
};

/***********************************************************************
 * L O C A L L Y   D E F I N E D   F U N C T I O N   P R O T O T Y P E S
 ***********************************************************************
 */


/*!
 ************************************************************************
 * \brief
 *    Exp Golomb binarization and encoding
 ************************************************************************
 */
static void exp_golomb_encode_eq_prob( EncodingEnvironmentPtr eep_dp,
                                unsigned int symbol,
                                int k)
{
  for(;;)
  {
    if (symbol >= (unsigned int)(1<<k))
    {
      biari_encode_symbol_eq_prob(eep_dp, 1);   //first unary part
      symbol = symbol - (1<<k);
      k++;
    }
    else
    {
      biari_encode_symbol_eq_prob(eep_dp, 0);   //now terminated zero of unary part
      while (k--)                               //next binary part
        biari_encode_symbol_eq_prob(eep_dp, ((symbol>>k)&1));
      break;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Unary binarization and encoding of a symbol by using
 *    one or two distinct models for the first two and all
 *    remaining bins
 *
 ************************************************************************/
static void unary_bin_encode(EncodingEnvironmentPtr eep_dp,
                      unsigned int symbol,
                      BiContextTypePtr ctx,
                      int ctx_offset)
{  
  if (symbol==0)
  {
    biari_encode_symbol(eep_dp, 0, ctx );
    return;
  }
  else
  {
    biari_encode_symbol(eep_dp, 1, ctx );
    ctx += ctx_offset;
    while ((--symbol) > 0)
      biari_encode_symbol(eep_dp, 1, ctx);
    biari_encode_symbol(eep_dp, 0, ctx);
  }
}

/*!
 ************************************************************************
 * \brief
 *    Unary binarization and encoding of a symbol by using
 *    one or two distinct models for the first two and all
 *    remaining bins; no terminating "0" for max_symbol
 *    (finite symbol alphabet)
 ************************************************************************
 */
static void unary_bin_max_encode(EncodingEnvironmentPtr eep_dp,
                          unsigned int symbol,
                          BiContextTypePtr ctx,
                          int ctx_offset,
                          unsigned int max_symbol)
{
  if (symbol==0)
  {
    biari_encode_symbol(eep_dp, 0, ctx );
    return;
  }
  else
  {
    unsigned int l = symbol;
    biari_encode_symbol(eep_dp, 1, ctx );
    
    ctx += ctx_offset;
    while ((--l)>0)
      biari_encode_symbol(eep_dp, 1, ctx);
    if (symbol < max_symbol)
      biari_encode_symbol(eep_dp, 0, ctx);
  }
}

/*!
 ************************************************************************
 * \brief
 *    Exp-Golomb for Level Encoding
*
************************************************************************/
static void unary_exp_golomb_level_encode( EncodingEnvironmentPtr eep_dp,
                                    unsigned int symbol,
                                    BiContextTypePtr ctx)
{
  if (symbol==0)
  {
    biari_encode_symbol(eep_dp, 0, ctx );
    return;
  }
  else
  {
    unsigned int l=symbol;
    unsigned int k = 1;

    biari_encode_symbol(eep_dp, 1, ctx );
    while (((--l)>0) && (++k <= 13))
      biari_encode_symbol(eep_dp, 1, ctx);
    if (symbol < 13) 
      biari_encode_symbol(eep_dp, 0, ctx);
    else 
      exp_golomb_encode_eq_prob(eep_dp,symbol - 13, 0);
  }
}


/*!
 ************************************************************************
 * \brief
 *    Exp-Golomb for MV Encoding
*
************************************************************************/
static void unary_exp_golomb_mv_encode(EncodingEnvironmentPtr eep_dp,
                                unsigned int symbol,
                                BiContextTypePtr ctx,
                                unsigned int max_bin)
{  
  if (symbol==0)
  {
    biari_encode_symbol(eep_dp, 0, ctx );
    return;
  }
  else
  {
    unsigned int bin = 1;
    unsigned int l = symbol, k = 1;
    biari_encode_symbol(eep_dp, 1, ctx++ );

    while (((--l)>0) && (++k <= 8))
    {
      biari_encode_symbol(eep_dp, 1, ctx  );
      if ((++bin) == 2) 
        ++ctx;
      if (bin == max_bin) 
        ++ctx;
    }
    if (symbol < 8) 
      biari_encode_symbol(eep_dp, 0, ctx);
    else 
      exp_golomb_encode_eq_prob(eep_dp, symbol - 8, 3);
  }
}

/*!
 ************************************************************************
 * \brief
 *    Check for available neighbouring blocks
 *    and set pointers in current macroblock
 ************************************************************************
 */
void CheckAvailabilityOfNeighborsCABAC(Macroblock *currMB)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  PixelPos up, left;
  int *mb_size = p_Vid->mb_size[IS_LUMA];

  p_Vid->getNeighbour(currMB, -1,  0, mb_size, &left);
  p_Vid->getNeighbour(currMB,  0, -1, mb_size, &up);

  if (up.available)
    currMB->mb_up = &p_Vid->mb_data[up.mb_addr];
  else
    currMB->mb_up = NULL;

  if (left.available)
    currMB->mb_left = &p_Vid->mb_data[left.mb_addr];
  else
    currMB->mb_left = NULL;
}

/*!
 ************************************************************************
 * \brief
 *    Allocation of contexts models for the motion info
 *    used for arithmetic encoding
 ************************************************************************
 */
MotionInfoContexts* create_contexts_MotionInfo(void)
{
  MotionInfoContexts* enco_ctx;

  enco_ctx = (MotionInfoContexts*) calloc(1, sizeof(MotionInfoContexts) );
  if( enco_ctx == NULL )
    no_mem_exit("create_contexts_MotionInfo: enco_ctx");

  return enco_ctx;
}


/*!
 ************************************************************************
 * \brief
 *    Allocates of contexts models for the texture info
 *    used for arithmetic encoding
 ************************************************************************
 */
TextureInfoContexts* create_contexts_TextureInfo(void)
{
  TextureInfoContexts*  enco_ctx = (TextureInfoContexts*) calloc(1, sizeof(TextureInfoContexts) );
  if( enco_ctx == NULL )
    no_mem_exit("create_contexts_TextureInfo: enco_ctx");

  return enco_ctx;
}

/*!
 ************************************************************************
 * \brief
 *    Frees the memory of the contexts models
 *    used for arithmetic encoding of the motion info.
 ************************************************************************
 */
void delete_contexts_MotionInfo(MotionInfoContexts *enco_ctx)
{
  free_pointer( enco_ctx );
}

/*!
 ************************************************************************
 * \brief
 *    Frees the memory of the contexts models
 *    used for arithmetic encoding of the texture info.
 ************************************************************************
 */
void delete_contexts_TextureInfo(TextureInfoContexts *enco_ctx)
{
  free_pointer( enco_ctx );
}


/*!
 ***************************************************************************
 * \brief
 *    This function is used to arithmetically encode the field
 *    mode info of a given MB  in the case of mb-based frame/field decision
 ***************************************************************************
 */
void writeFieldModeInfo_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);  
  int curr_len = arienco_bits_written(eep_dp);
  VideoParameters *p_Vid = currMB->p_Vid;
  Macroblock *mb_data = p_Vid->mb_data;

  int a = currMB->mbAvailA ? mb_data[currMB->mbAddrA].mb_field : 0;
  int b = currMB->mbAvailB ? mb_data[currMB->mbAddrB].mb_field : 0;

  int act_ctx = a + b;

#if ENABLE_FIELD_CTX
  TextureInfoContexts *ctx = dp->p_Slice->tex_ctx;
  
  biari_encode_symbol(eep_dp, (se->value1 != 0), &ctx->mb_aff_contexts[act_ctx]);
#endif

  se->context = act_ctx;

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
***************************************************************************
* \brief
*    This function is used to arithmetically encode the mb_skip_flag.
***************************************************************************
*/
void writeMB_Pskip_flagInfo_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  Slice *currSlice = currMB->p_Slice;
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  MotionInfoContexts *ctx = currSlice->mot_ctx;
  int        curr_mb_type = se->value1;
  int a = (currMB->mb_left != NULL && (currMB->mb_left->skip_flag == 0)) ? 1 : 0;
  int b = (currMB->mb_up   != NULL && (currMB->mb_up  ->skip_flag == 0)) ? 1 : 0;
  int act_ctx = a + b;

  if (curr_mb_type==0) // SKIP
    biari_encode_symbol(eep_dp, 1, &ctx->mb_type_contexts[1][act_ctx]);
  else
    biari_encode_symbol(eep_dp, 0, &ctx->mb_type_contexts[1][act_ctx]);

  currMB->skip_flag = (curr_mb_type==0) ? 1 : 0;

  se->context = act_ctx;
  se->value1  = 1 - currMB->skip_flag;

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
***************************************************************************
* \brief
*    This function is used to arithmetically encode the mb_skip_flag for B slices.
***************************************************************************
*/
void writeMB_Bskip_flagInfo_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  Slice *currSlice = currMB->p_Slice;
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  MotionInfoContexts *ctx = currSlice->mot_ctx;
  int a = (currMB->mb_left != NULL && (currMB->mb_left->skip_flag == 0)) ? 1 : 0;
  int b = (currMB->mb_up   != NULL && (currMB->mb_up  ->skip_flag == 0)) ? 1 : 0;
  int act_ctx = a + b;

  act_ctx += 7;

  if (se->value1==0 && se->value2==0) // DIRECT mode, no coefficients
    biari_encode_symbol (eep_dp, 1, &ctx->mb_type_contexts[2][act_ctx]);
  else
    biari_encode_symbol (eep_dp, 0, &ctx->mb_type_contexts[2][act_ctx]);

  currMB->skip_flag = (se->value1==0 && se->value2==0) ? 1 : 0;

  se->context = act_ctx;
  se->value1  = 1 - currMB->skip_flag;

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
***************************************************************************
* \brief
*    This function is used to arithmetically encode the macroblock
*    intra_pred_size flag info of a given MB.
***************************************************************************
*/

void writeMB_transform_size_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  TextureInfoContexts *ctx      = (dp->p_Slice)->tex_ctx;

  int curr_len = arienco_bits_written(eep_dp);
  int act_sym = currMB->luma_transform_size_8x8_flag;


  int b = (currMB->mb_up == NULL) ? 0 : currMB->mb_up->luma_transform_size_8x8_flag;
  int a = (currMB->mb_left == NULL) ? 0 :currMB->mb_left->luma_transform_size_8x8_flag;
  int act_ctx     = a + b;

  se->context = act_ctx; // store context
  biari_encode_symbol(eep_dp, (act_sym != 0), ctx->transform_size_contexts + act_ctx );

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ***************************************************************************
 * \brief
 *    This function is used to arithmetically encode the macroblock
 *    type info of a given MB.
 ***************************************************************************
 */

void writeMB_P_typeInfo_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  
  BiContextType *mb_type_contexts = dp->p_Slice->mot_ctx->mb_type_contexts[1];

  int act_ctx = 0;  
  int mode_sym = 0;
  int act_sym = se->value1;
  int mode16x16 = 7;

  if (act_sym >= mode16x16)
  {
    mode_sym = act_sym - mode16x16;
    act_sym  = mode16x16; // 16x16 mode info
  }

  switch (act_sym)
  {
  case 0:
    break;
  case 1:
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[5]);
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[6]);
    break;
  case 2:
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[5]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[7]);
    break;
  case 3:
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[5]);
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[7]);
    break;
  case 4:
  case 5:
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[5]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[6]);
    break;
  case 6:
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[7]);
    break;
  case 7:
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[7]);
    break;
  default:
    printf ("Unsupported MB-MODE in writeMB_I_typeInfo_CABAC!\n");
    exit (1);
  }

  if(act_sym == mode16x16) // additional info for 16x16 Intra-mode
  {
    if( mode_sym == 24 )
    {
      biari_encode_symbol_final(eep_dp, 1 );
      dp->bitstream->write_flag = 1;
      se->len = (arienco_bits_written(eep_dp) - curr_len);
      CABAC_TRACE;
      return;
    }
    biari_encode_symbol_final(eep_dp, 0 );

    act_ctx = 8;
    act_sym = mode_sym/12;
    biari_encode_symbol(eep_dp, act_sym, mb_type_contexts + act_ctx ); // coding of AC/no AC
    mode_sym = mode_sym % 12;

    act_sym = mode_sym >> 2; // coding of cbp: 0,1,2
    act_ctx = 9;
    if (act_sym==0)
    {
      biari_encode_symbol(eep_dp, 0, mb_type_contexts + act_ctx );
    }
    else
    {
      biari_encode_symbol(eep_dp, 1, mb_type_contexts + act_ctx );
      biari_encode_symbol(eep_dp, (act_sym!=1), mb_type_contexts + act_ctx );
    }

    mode_sym = mode_sym & 0x03; // coding of I pred-mode: 0,1,2,3
    act_ctx  = 10;
    act_sym  = mode_sym >> 1;
    biari_encode_symbol(eep_dp, act_sym, mb_type_contexts + act_ctx );
    act_sym  = (mode_sym & 0x01);
    biari_encode_symbol(eep_dp, act_sym, mb_type_contexts + act_ctx );
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ***************************************************************************
 * \brief
 *    This function is used to arithmetically encode the macroblock
 *    type info of a given MB in a B slice.
 ***************************************************************************
 */

void writeMB_B_typeInfo_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);  
  
  int csym;  
  int act_sym = se->value1;  

  int mode_sym = 0;
  int mode16x16 = 24;

  MotionInfoContexts *ctx    = dp->p_Slice->mot_ctx;
  BiContextType *mb_type_contexts = ctx->mb_type_contexts[2];

  int a = (currMB->mb_left == NULL) ? 0 : ((currMB->mb_left->mb_type != 0) ? 1 : 0 );
  int b = (currMB->mb_up   == NULL) ? 0 : ((currMB->mb_up->mb_type   != 0) ? 1 : 0 );
  int act_ctx = a + b;
  se->context = act_ctx; // store context

  if (act_sym >= mode16x16)
  {
    mode_sym = act_sym - mode16x16;
    act_sym  = mode16x16; // 16x16 mode info
  }

  if (act_sym==0)
  {
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[act_ctx]);
  }
  else if (act_sym<=2)
  {
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[act_ctx]);
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[4]);
    csym =  (act_sym-1 != 0);
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
  }
  else if (act_sym<=10)
  {
    int temp_sym = act_sym - 3;
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[act_ctx]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 0, &mb_type_contexts[5]);
    csym =  ((temp_sym >> 2)&0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
    csym =  ((temp_sym >> 1)&0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
    csym =  (temp_sym & 0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
  }
  else if (act_sym==11 || act_sym==22)
  {
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[act_ctx]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[5]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[6]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[6]);
    csym =  (act_sym != 11);
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
  }
  else
  {
    int temp_sym = (act_sym > 22) ? act_sym - 13 : act_sym - 12;    
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[act_ctx]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[4]);
    biari_encode_symbol (eep_dp, 1, &mb_type_contexts[5]);
    csym = ((temp_sym >> 3)&0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
    csym = ((temp_sym >> 2)&0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
    csym = ((temp_sym >> 1)&0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
    csym = (temp_sym & 0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &mb_type_contexts[6]);
  }


  if(act_sym == mode16x16) // additional info for 16x16 Intra-mode
  {    
    mb_type_contexts = ctx->mb_type_contexts[1];
    if( mode_sym == 24 )
    {
      biari_encode_symbol_final(eep_dp, 1 );
      dp->bitstream->write_flag = 1;
      se->len = (arienco_bits_written(eep_dp) - curr_len);
      CABAC_TRACE;
      return;
    }
    biari_encode_symbol_final(eep_dp, 0 );

    act_ctx = 8;
    act_sym = mode_sym/12;
    biari_encode_symbol(eep_dp, act_sym, mb_type_contexts + act_ctx ); // coding of AC/no AC
    mode_sym = mode_sym % 12;

    act_sym = mode_sym >> 2; // coding of cbp: 0,1,2
    act_ctx = 9;
    if (act_sym==0)
    {
      biari_encode_symbol(eep_dp, 0, mb_type_contexts + act_ctx );
    }
    else
    {
      biari_encode_symbol(eep_dp, 1, mb_type_contexts + act_ctx );
      biari_encode_symbol(eep_dp, (act_sym!=1), mb_type_contexts + act_ctx );
    }

    mode_sym = mode_sym & 0x03; // coding of I pred-mode: 0,1,2,3
    act_ctx  = 10;
    act_sym  = mode_sym >> 1;
    biari_encode_symbol(eep_dp, act_sym, mb_type_contexts + act_ctx );
    act_sym  = (mode_sym & 0x01);
    biari_encode_symbol(eep_dp, act_sym, mb_type_contexts + act_ctx );
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ***************************************************************************
 * \brief
 *    This function is used to arithmetically encode the macroblock
 *    type info of a given MB in an I Slice.
 ***************************************************************************
 */
void writeMB_I_typeInfo_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  int a, b;
  int act_ctx = 0;
  int act_sym;
  Slice *currSlice = dp->p_Slice;  
  int mode_sym = 0;

  MotionInfoContexts *ctx         = currSlice->mot_ctx;

  if (currMB->mb_up == NULL)
    b = 0;
  else
    b = ((currMB->mb_up->mb_type != I4MB &&  currMB->mb_up->mb_type != I8MB) ? 1 : 0 );

  if (currMB->mb_left == NULL)
    a = 0;
  else
    a = ((currMB->mb_left->mb_type != I4MB &&  currMB->mb_left->mb_type != I8MB) ? 1 : 0 );

  act_ctx     = a + b;
  act_sym     = se->value1;
  se->context = act_ctx; // store context

  if (act_sym==0) // 4x4 Intra
  {
    biari_encode_symbol(eep_dp, 0, ctx->mb_type_contexts[0] + act_ctx );
  }
  else if( act_sym == 25 ) // PCM-MODE
  {
    biari_encode_symbol(eep_dp, 1, ctx->mb_type_contexts[0] + act_ctx );
    biari_encode_symbol_final(eep_dp, 1);
  }
  else // 16x16 Intra
  {
    biari_encode_symbol(eep_dp, 1, ctx->mb_type_contexts[0] + act_ctx );

    biari_encode_symbol_final(eep_dp, 0);

    mode_sym = act_sym-1; // Values in the range of 0...23
    act_ctx  = 4;
    act_sym  = mode_sym/12;
    biari_encode_symbol(eep_dp, act_sym, ctx->mb_type_contexts[0] + act_ctx ); // coding of AC/no AC
    mode_sym = mode_sym % 12;
    act_sym  = mode_sym / 4; // coding of cbp: 0,1,2
    act_ctx  = 5;
    if (act_sym==0)
    {
      biari_encode_symbol(eep_dp, 0, ctx->mb_type_contexts[0] + act_ctx );
    }
    else
    {
      biari_encode_symbol(eep_dp, 1, ctx->mb_type_contexts[0] + act_ctx );
      act_ctx=6;
      biari_encode_symbol(eep_dp, (act_sym!=1), ctx->mb_type_contexts[0] + act_ctx );
    }
    mode_sym = mode_sym & 0x03; // coding of I pred-mode: 0,1,2,3
    act_sym  = mode_sym >> 1;
    act_ctx  = 7;
    biari_encode_symbol(eep_dp, act_sym, ctx->mb_type_contexts[0] + act_ctx );
    act_ctx  = 8;
    act_sym  = mode_sym & 0x01;
    biari_encode_symbol(eep_dp, act_sym, ctx->mb_type_contexts[0] + act_ctx );
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ***************************************************************************
 * \brief
 *    This function is used to arithmetically encode the 8x8 block
 *    type info
 ***************************************************************************
 */
void writeB8_B_typeInfo_CABAC(SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  BiContextType *b8_type_contexts = dp->p_Slice->mot_ctx->b8_type_contexts[1];
  int curr_len = arienco_bits_written(eep_dp);
  int act_sym = se->value1;
  signed short csym;

  if (act_sym==0)
  {
    biari_encode_symbol (eep_dp, 0, &b8_type_contexts[0]);
    dp->bitstream->write_flag = 1;
    se->len = (arienco_bits_written(eep_dp) - curr_len);
    CABAC_TRACE;
    return;
  }
  else
  {
    biari_encode_symbol (eep_dp, 1, &b8_type_contexts[0]);
    act_sym--;
  }

  if (act_sym < 2)
  {
    biari_encode_symbol (eep_dp, 0, &b8_type_contexts[1]);
    biari_encode_symbol (eep_dp, (act_sym!=0), &b8_type_contexts[3]);
  }
  else if (act_sym < 6)
  {
    biari_encode_symbol (eep_dp, 1, &b8_type_contexts[1]);
    biari_encode_symbol (eep_dp, 0, &b8_type_contexts[2]);
    csym = (short) (((act_sym - 2) >> 1) & 0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &b8_type_contexts[3]);
    csym = (short) ((act_sym - 2) & 0x01) != 0;
    biari_encode_symbol (eep_dp, csym, &b8_type_contexts[3]);
  }
  else
  {
    biari_encode_symbol (eep_dp, 1, &b8_type_contexts[1]);
    biari_encode_symbol (eep_dp, 1, &b8_type_contexts[2]);
    csym = (short) (((act_sym - 6)>> 2) & 0x01);
    if (csym)
    {
      biari_encode_symbol (eep_dp, 1, &b8_type_contexts[3]);
      csym = (short) ((act_sym - 6) & 0x01) != 0;
      biari_encode_symbol (eep_dp, csym, &b8_type_contexts[3]);
    }
    else
    {
      biari_encode_symbol (eep_dp, 0, &b8_type_contexts[3]);
      csym = (short) (((act_sym - 6) >> 1) & 0x01) != 0;
      biari_encode_symbol (eep_dp, csym, &b8_type_contexts[3]);
      csym = (short) ((act_sym - 6) & 0x01) != 0;
      biari_encode_symbol (eep_dp, csym, &b8_type_contexts[3]);
    }
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ***************************************************************************
 * \brief
 *    This function is used to arithmetically encode the 8x8 block
 *    type info
 ***************************************************************************
 */
void writeB8_typeInfo_CABAC(SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  BiContextType (*b8_type_contexts) = dp->p_Slice->mot_ctx->b8_type_contexts[0];
  int curr_len = arienco_bits_written(eep_dp);

  switch (se->value1)
  {
  case 0:
    biari_encode_symbol (eep_dp, 1, &b8_type_contexts[1]);
    break;
  case 1:
    biari_encode_symbol (eep_dp, 0, &b8_type_contexts[1]);
    biari_encode_symbol (eep_dp, 0, &b8_type_contexts[3]);
    break;
  case 2:
    biari_encode_symbol (eep_dp, 0, &b8_type_contexts[1]);
    biari_encode_symbol (eep_dp, 1, &b8_type_contexts[3]);
    biari_encode_symbol (eep_dp, 1, &b8_type_contexts[4]);
    break;
  case 3:
    biari_encode_symbol (eep_dp, 0, &b8_type_contexts[1]);
    biari_encode_symbol (eep_dp, 1, &b8_type_contexts[3]);
    biari_encode_symbol (eep_dp, 0, &b8_type_contexts[4]);
    break;
  }   

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ****************************************************************************
 * \brief
 *    This function is used to arithmetically encode a pair of
 *    intra prediction modes of a given MB.
 ****************************************************************************
 */
void writeIntraPredMode_CABAC(SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  BiContextType *ipr_contexts = (dp->p_Slice)->tex_ctx->ipr_contexts;

  // use_most_probable_mode
  if (se->value1 == -1)
    biari_encode_symbol(eep_dp, 1, ipr_contexts);
  else
  {
    biari_encode_symbol(eep_dp, 0, ipr_contexts);

    // remaining_mode_selector
    biari_encode_symbol(eep_dp, ( se->value1 & 0x1    ), ipr_contexts+1);
    biari_encode_symbol(eep_dp, ((se->value1 & 0x2)>>1), ipr_contexts+1);
    biari_encode_symbol(eep_dp, ((se->value1 & 0x4)>>2), ipr_contexts+1);
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ****************************************************************************
 * \brief
 *    This function is used to arithmetically encode the reference
 *    parameter of a given MB.
 ****************************************************************************
 */
void writeRefPic_P_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  Slice *currSlice = dp->p_Slice;
  VideoParameters *p_Vid = dp->p_Vid;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  MotionInfoContexts  *ctx    = currSlice->mot_ctx;

  int                 addctx  = 0;

  int   a = 0, b = 0;
  int   act_ctx;
  int   act_sym;
  int   list = se->value2;  

  PixelPos block_a, block_b;
  int *mb_size = p_Vid->mb_size[IS_LUMA];

  get4x4Neighbour(currMB, currMB->subblock_x - 1, currMB->subblock_y    , mb_size, &block_a);
  get4x4Neighbour(currMB, currMB->subblock_x,     currMB->subblock_y - 1, mb_size, &block_b);

  if (block_b.available)
  {
    if (p_Vid->mb_aff_frame_flag && (currMB->mb_field == FALSE) && (p_Vid->mb_data[block_b.mb_addr].mb_field == TRUE))
      b = (motion[block_b.pos_y][block_b.pos_x].ref_idx[list] > 1 ? 2 : 0);
    else
      b = (motion[block_b.pos_y][block_b.pos_x].ref_idx[list] > 0 ? 2 : 0);
  }

  if (block_a.available)
  {
    if (p_Vid->mb_aff_frame_flag && (currMB->mb_field == FALSE) && (p_Vid->mb_data[block_a.mb_addr].mb_field == TRUE))
      a = (motion[block_a.pos_y][block_a.pos_x].ref_idx[list] > 1 ? 1 : 0);
    else
      a = (motion[block_a.pos_y][block_a.pos_x].ref_idx[list] > 0 ? 1 : 0);
  }

  act_ctx     = a + b;
  se->context = act_ctx; // store context
  act_sym     = se->value1;

  if (act_sym==0)
  {
    biari_encode_symbol(eep_dp, 0, ctx->ref_no_contexts[addctx] + act_ctx );
  }
  else
  {
    biari_encode_symbol(eep_dp, 1, ctx->ref_no_contexts[addctx] + act_ctx);
    act_sym--;
    act_ctx=4;
    unary_bin_encode(eep_dp, act_sym, ctx->ref_no_contexts[addctx] + act_ctx, 1);
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ****************************************************************************
 * \brief
 *    This function is used to arithmetically encode the reference
 *    parameter of a given MB.
 ****************************************************************************
 */
void writeRefPic_B_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  Slice *currSlice = dp->p_Slice;
  VideoParameters *p_Vid = dp->p_Vid;
  PicMotionParams **motion = p_Vid->enc_picture->mv_info;
  MotionInfoContexts  *ctx    = currSlice->mot_ctx;
  int                 addctx  = 0;

  int   a = 0, b = 0;
  int   act_ctx;
  int   act_sym;
  int   list = se->value2;

  int   b8a, b8b;

  PixelPos block_a, block_b;
  int *mb_size = p_Vid->mb_size[IS_LUMA];

  get4x4Neighbour(currMB, currMB->subblock_x - 1, currMB->subblock_y    , mb_size, &block_a);
  get4x4Neighbour(currMB, currMB->subblock_x,     currMB->subblock_y - 1, mb_size, &block_b);

  if (block_b.available)
  {
    b8b=((block_b.x >> 1) & 0x01)+2*((block_b.y >> 1) & 0x01);

    if (((p_Vid->mb_data[block_b.mb_addr].mb_type == 0) && !p_Vid->giRDOpt_B8OnlyFlag) || (p_Vid->mb_data[block_b.mb_addr].b8x8[b8b].mode==0))
      b=0;
    else
    {
      if (p_Vid->mb_aff_frame_flag && (currMB->mb_field == FALSE) && (p_Vid->mb_data[block_b.mb_addr].mb_field == TRUE))
        b = (motion[block_b.pos_y][block_b.pos_x].ref_idx[list] > 1 ? 1 : 0);
      else
        b = (motion[block_b.pos_y][block_b.pos_x].ref_idx[list] > 0 ? 1 : 0);
    }
  }

  if (block_a.available)
  {
    b8a=((block_a.x >> 1) & 0x01)+2*((block_a.y >> 1) & 0x01);
    if (((p_Vid->mb_data[block_a.mb_addr].mb_type == 0) && !p_Vid->giRDOpt_B8OnlyFlag) || (p_Vid->mb_data[block_a.mb_addr].b8x8[b8a].mode ==0))
      a=0;
    else
    {
      if (p_Vid->mb_aff_frame_flag && (currMB->mb_field == FALSE) && (p_Vid->mb_data[block_a.mb_addr].mb_field == TRUE))
        a = (motion[block_a.pos_y][block_a.pos_x].ref_idx[list] > 1 ? 1 : 0);
      else
        a = (motion[block_a.pos_y][block_a.pos_x].ref_idx[list] > 0 ? 1 : 0);
    }
  }

  act_ctx     = a + 2*b;
  se->context = act_ctx; // store context
  act_sym     = se->value1;

  if (act_sym==0)
  {
    biari_encode_symbol(eep_dp, 0, ctx->ref_no_contexts[addctx] + act_ctx );
  }
  else
  {
    biari_encode_symbol(eep_dp, 1, ctx->ref_no_contexts[addctx] + act_ctx);
    act_sym--;
    act_ctx=4;
    unary_bin_encode(eep_dp, act_sym, ctx->ref_no_contexts[addctx] + act_ctx, 1);
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ****************************************************************************
 * \brief
 *    This function is used to arithmetically encode the coded
 *    block pattern of a given delta quant.
 ****************************************************************************
 */
void writeDquant_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);

  TextureInfoContexts *ctx = dp->p_Slice->tex_ctx;
 
  int dquant  = se->value1;
  int sign    = (dquant <= 0) ? 0 : -1;
  int act_sym = (iabs(dquant) << 1) + sign;
  int act_ctx = ((currMB->prev_dqp != 0) ? 1 : 0);

  if (act_sym == 0)
  {
    biari_encode_symbol(eep_dp, 0, ctx->delta_qp_contexts + act_ctx );
  }
  else
  {
    biari_encode_symbol(eep_dp, 1, ctx->delta_qp_contexts + act_ctx);
    act_ctx=2;
    act_sym--;
    unary_bin_encode(eep_dp, act_sym,ctx->delta_qp_contexts+act_ctx,1);
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ****************************************************************************
 * \brief
 *    This function is used to arithmetically encode the motion
 *    vector data of a B-frame MB.
 ****************************************************************************
 */
void writeMVD_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  VideoParameters *p_Vid      = currMB->p_Vid;
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);

  int curr_len = arienco_bits_written(eep_dp);
  MotionInfoContexts  *ctx    = dp->p_Slice->mot_ctx;

  int i = currMB->subblock_x;
  int j = currMB->subblock_y;
  int a, b;
  int act_ctx;
  int act_sym;
  int mv_pred_res;
  int mv_local_err;
  int mv_sign;
  int list_idx = se->value2 & 0x01;
  int k = (se->value2>>1); // MVD component

  PixelPos block_a, block_b;

  int *mb_size = p_Vid->mb_size[IS_LUMA];

  get4x4Neighbour(currMB, i - 1, j    , mb_size, &block_a);
  get4x4Neighbour(currMB, i    , j - 1, mb_size, &block_b);

  if (block_b.available)
  {
    b = iabs(p_Vid->mb_data[block_b.mb_addr].mvd[list_idx][block_b.y][block_b.x][k]);
    if (p_Vid->mb_aff_frame_flag && (k==1))
    {
      if ((currMB->mb_field==0) && (p_Vid->mb_data[block_b.mb_addr].mb_field==1))
        b *= 2;
      else if ((currMB->mb_field==1) && (p_Vid->mb_data[block_b.mb_addr].mb_field==0))
        b /= 2;
    }
  }
  else
    b=0;

  if (block_a.available)
  {
    a = iabs(p_Vid->mb_data[block_a.mb_addr].mvd[list_idx][block_a.y][block_a.x][k]);
    if (p_Vid->mb_aff_frame_flag && (k==1))
    {
      if ((currMB->mb_field==0) && (p_Vid->mb_data[block_a.mb_addr].mb_field==1))
        a *= 2;
      else if ((currMB->mb_field==1) && (p_Vid->mb_data[block_a.mb_addr].mb_field==0))
        a /= 2;
    }
  }
  else
    a = 0;

  if ((mv_local_err = a + b)<3)
    act_ctx = 5 * k;
  else
  {
    if (mv_local_err>32)
      act_ctx = 5 * k + 3;
    else
      act_ctx = 5 * k + 2;
  }

  mv_pred_res = se->value1;
  se->context = act_ctx;

  act_sym = iabs(mv_pred_res);

  if (act_sym == 0)
    biari_encode_symbol(eep_dp, 0, &ctx->mv_res_contexts[0][act_ctx] );
  else
  {
    biari_encode_symbol(eep_dp, 1, &ctx->mv_res_contexts[0][act_ctx] );
    act_sym--;
    act_ctx=5*k;
    unary_exp_golomb_mv_encode(eep_dp,act_sym,ctx->mv_res_contexts[1]+act_ctx,3);
    mv_sign = (mv_pred_res<0) ? 1: 0;
    biari_encode_symbol_eq_prob(eep_dp, mv_sign);
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}


/*!
 ****************************************************************************
 * \brief
 *    This function is used to arithmetically encode the chroma
 *    intra prediction mode of an 8x8 block
 ****************************************************************************
 */
void writeCIPredMode_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  TextureInfoContexts *ctx     = dp->p_Slice->tex_ctx;

  int                 act_sym  = se->value1;
  
  Macroblock          *MbUp   = currMB->mb_up;
  Macroblock          *MbLeft = currMB->mb_left;

  int b = (MbUp   != NULL) ? (((MbUp->c_ipred_mode != 0) && (MbUp->mb_type != IPCM)) ? 1 : 0) : 0;
  int a = (MbLeft != NULL) ? (((MbLeft->c_ipred_mode != 0) && (MbLeft->mb_type != IPCM)) ? 1 : 0) : 0;

  int act_ctx = a + b;

  if (act_sym==0)
    biari_encode_symbol(eep_dp, 0, ctx->cipr_contexts + act_ctx );
  else
  {
    biari_encode_symbol(eep_dp, 1, ctx->cipr_contexts + act_ctx );
    unary_bin_max_encode(eep_dp,(unsigned int) (act_sym-1),ctx->cipr_contexts + 3,0,2);
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}


/*!
 ****************************************************************************
 * \brief
 *    This function is used to arithmetically encode the coded
 *    block pattern of an 8x8 block
 ****************************************************************************
 */
void writeCBP_BIT_CABAC (Macroblock* currMB, int b8, int bit, int cbp, EncodingEnvironmentPtr eep_dp, TextureInfoContexts *ctx)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  PixelPos block_a;
  int a = 0, b = 0;

  int mb_x=(b8 & 0x01)<<1;
  int mb_y=(b8 >> 1)<<1;

  if (mb_y == 0)
  {
    if (!((currMB->mb_up == NULL) || ((currMB->mb_up)->mb_type==IPCM)))
    {
      b = (( ((currMB->mb_up)->cbp & (1<<(2+(mb_x>>1)))) == 0) ? 1 : 0);   //VG-ADD
    }

  }
  else
    b = ( ((cbp & (1<<(mb_x >> 1))) == 0) ? 1: 0);

  if (mb_x == 0)
  {
    get4x4Neighbour(currMB, (mb_x << 2) - 1, (mb_y << 2), p_Vid->mb_size[IS_LUMA], &block_a);

    if (block_a.available && (!(p_Vid->mb_data[block_a.mb_addr].mb_type==IPCM)))
    {
      a = (( (p_Vid->mb_data[block_a.mb_addr].cbp & (1<<(2*(block_a.y>>1)+1))) == 0) ? 1 : 0); //VG-ADD
    }    
  }
  else
    a = ( ((cbp & (1<<mb_y)) == 0) ? 1: 0);

  //===== WRITE BIT =====
  biari_encode_symbol (eep_dp, bit, ctx->cbp_contexts[0] + a+2*b);
}

/*!
****************************************************************************
* \brief
*    This function is used to arithmetically encode the coded
*    block pattern of a macroblock
****************************************************************************
*/
void writeCBP_CABAC(Macroblock *currMB, SyntaxElement *se, DataPartition *dp)
{
  Slice *currSlice = currMB->p_Slice;
  VideoParameters *p_Vid = currSlice->p_Vid;
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);
  TextureInfoContexts *ctx = currSlice->tex_ctx;

  int a0 = 0, a1 = 0, b0 = 0, b1 = 0;
  int curr_cbp_ctx;
  int cbp = se->value1; // symbol to encode
  int cbp_bit;
  int b8;

  for (b8=0; b8<4; ++b8)
  {
    writeCBP_BIT_CABAC (currMB, b8, cbp&(1<<b8), cbp, eep_dp, ctx);
  }

  if ((p_Vid->yuv_format != YUV400) && (p_Vid->yuv_format != YUV444) )
  {
    // coding of chroma part
    if (currMB->mb_up != NULL)
    {
      if((currMB->mb_up)->mb_type == IPCM || ((currMB->mb_up)->cbp > 15))
        b0 = 2;
    }

    if (currMB->mb_left != NULL)
    {
      if((currMB->mb_left)->mb_type==IPCM || ((currMB->mb_left)->cbp > 15))
        a0 = 1;
    }

    curr_cbp_ctx = a0 + b0;
    cbp_bit = (cbp > 15 ) ? 1 : 0;
    biari_encode_symbol(eep_dp, cbp_bit, ctx->cbp_contexts[1] + curr_cbp_ctx );

    if (cbp > 15)
    {
      if (currMB->mb_up != NULL)
      {
        if((currMB->mb_up)->mb_type == IPCM || 
          (((currMB->mb_up)->cbp > 15) && ( ((currMB->mb_up)->cbp >> 4) == 2)))
          b1 = 2;
      }

      if (currMB->mb_left != NULL)
      {
        if((currMB->mb_left)->mb_type==IPCM || 
          (((currMB->mb_left)->cbp > 15) && ( ((currMB->mb_left)->cbp >> 4) == 2)))
          a1 = 1;
      }

      curr_cbp_ctx = a1 + b1;
      cbp_bit = ((cbp>>4) == 2) ? 1 : 0;
      biari_encode_symbol(eep_dp, cbp_bit, ctx->cbp_contexts[2] + curr_cbp_ctx );
    }
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}

/*!
 ****************************************************************************
 * \brief
 *    Write CBP4-BIT (for 444 formats)
 ****************************************************************************
 */
void write_and_store_CBP_block_bit_444 (Macroblock* currMB, EncodingEnvironmentPtr eep_dp, int type, int cbp_bit, TextureInfoContexts*  tex_ctx)
{
  VideoParameters *p_Vid = currMB->p_Vid;  
  Macroblock *mb_data = p_Vid->mb_data;

  int y_ac        = (type==LUMA_16AC || type==LUMA_8x8 || type==LUMA_8x4 || type==LUMA_4x8 || type==LUMA_4x4
                     || type==CB_16AC || type==CB_8x8 || type==CB_8x4 || type==CB_4x8 || type==CB_4x4
                     || type==CR_16AC || type==CR_8x8 || type==CR_8x4 || type==CR_4x8 || type==CR_4x4);
  int y_dc        = (type==LUMA_16DC || type==CB_16DC || type==CR_16DC); 
  int u_ac        = (type==CHROMA_AC && !p_Vid->is_v_block);
  int v_ac        = (type==CHROMA_AC &&  p_Vid->is_v_block);
  int chroma_dc   = (type==CHROMA_DC || type==CHROMA_DC_2x4 || type==CHROMA_DC_4x4);
  int u_dc        = (chroma_dc && !p_Vid->is_v_block);
  int v_dc        = (chroma_dc &&  p_Vid->is_v_block);
  int j           =  (y_ac || u_ac || v_ac ? currMB->subblock_y : 0);
  int i           =  (y_ac || u_ac || v_ac ? currMB->subblock_x : 0);
  int bit         =  (y_dc ? 0 : y_ac ? 1 : u_dc ? 17 : v_dc ? 18 : u_ac ? 19 : 23);
  int default_bit =  (currMB->is_intra_block ? 1 : 0);
  int upper_bit   = default_bit;
  int left_bit    = default_bit;
  int ctx;

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
        bit_pos_a = (block_a.y << 2) + block_a.x;
      if (block_b.available)
        bit_pos_b = (block_b.y << 2) + block_b.x;
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

  bit = (y_dc ? 0 : y_ac ? 1 + j + (i >> 2) : u_dc ? 17 : v_dc ? 18 : u_ac ? 19 + j + (i >> 2): 35 + j + (i >> 2));
  //--- set bits for current block ---
  if (cbp_bit)
  {    
    if (type==LUMA_8x8)
    {
      currMB->cbp_bits[0] |= ((int64) 0x33 << bit);

      if (p_Vid->enc_picture->chroma_format_idc == YUV444)
      {
        currMB->cbp_bits_8x8[0] |= ((int64) 0x33 << bit);
      }
    }
    else if (type==CB_8x8)
    {
      currMB->cbp_bits_8x8[1] |= ((int64) 0x33 << bit);
      currMB->cbp_bits[1]     |= ((int64) 0x33 << bit);
    }
    else if (type==CR_8x8)
    {
      currMB->cbp_bits_8x8[2] |= ((int64) 0x33 << bit);
      currMB->cbp_bits[2]     |= ((int64) 0x33 << bit);
    }
    else if (type==LUMA_8x4)
    {      
      currMB->cbp_bits[0]     |= ((int64) 0x03 << bit);
    }
    else if (type==LUMA_4x8)
    {      
      currMB->cbp_bits[0]     |= ((int64) 0x11 << bit);
    }
    else if (type==CB_8x4)
    {
      currMB->cbp_bits[1]     |= ((int64) 0x03 << bit);
    }
    else if (type==CR_8x4)
    {
      currMB->cbp_bits[2]     |= ((int64) 0x03 << bit);
    }
    else if (type==CB_4x8)
    {
      currMB->cbp_bits[1]     |= ((int64) 0x11 << bit);
    }
    else if (type==CR_4x8)
    {
      currMB->cbp_bits[2]     |= ((int64) 0x11 << bit);
    }
    else if ((type==CB_4x4)||(type==CB_16AC)||(type==CB_16DC))
    {
      currMB->cbp_bits[1]     |= ((int64) 0x01 << bit);
    }
    else if ((type == CR_4x4)||(type == CR_16AC)||(type == CR_16DC))
    {
      currMB->cbp_bits[2]     |= ((int64) 0x01 << bit);
    }
    else
    {
      currMB->cbp_bits[0]     |= ((int64) 0x01 << bit);
    }
  }

  bit = (y_dc ? 0 : y_ac ? 1 : u_dc ? 17 : v_dc ? 18 : u_ac ? 19 : 35);

  if (p_Vid->enc_picture->chroma_format_idc!=YUV444)
  {
    if (type!=LUMA_8x8)
    {
      if (block_b.available)
      {
        if(mb_data[block_b.mb_addr].mb_type==IPCM)
          upper_bit = 1;
        else
          upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits[0],bit+bit_pos_b);
      }


      if (block_a.available)
      {
        if(mb_data[block_a.mb_addr].mb_type == IPCM)
          left_bit = 1;
        else
          left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits[0], bit + bit_pos_a);
      }

      ctx = (upper_bit << 1) + left_bit;

      //===== encode symbol =====
      biari_encode_symbol (eep_dp, cbp_bit, tex_ctx->bcbp_contexts[type2ctx_bcbp[type]] + ctx);
    }
  }
  else if( (currMB->p_Inp->separate_colour_plane_flag != 0) )
  {
    if (type!=LUMA_8x8)
    {
      if (block_b.available)
      {
        if(mb_data[block_b.mb_addr].mb_type==IPCM)
          upper_bit = 1;
        else
          upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits[0],bit+bit_pos_b);
      }


      if (block_a.available)
      {
        if(mb_data[block_a.mb_addr].mb_type == IPCM)
          left_bit = 1;
        else
          left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits[0], bit + bit_pos_a);
      }

      ctx = (upper_bit << 1) + left_bit;

      //===== encode symbol =====
      biari_encode_symbol (eep_dp, cbp_bit, tex_ctx->bcbp_contexts[type2ctx_bcbp[type]] + ctx);
    }
  }
  else 
  {
    if (block_b.available)
    {
      if(mb_data[block_b.mb_addr].mb_type == IPCM)
      {
        upper_bit=1;
      }
      else if((type==LUMA_8x8 || type==CB_8x8 || type==CR_8x8) &&
         !mb_data[block_b.mb_addr].luma_transform_size_8x8_flag)
      {
        upper_bit=0;
      }
      else
      {
        if(type==LUMA_8x8)
          upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits_8x8[0], bit + bit_pos_b);
        else if (type==CB_8x8)
          upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits_8x8[1], bit + bit_pos_b);
        else if (type==CR_8x8)
          upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits_8x8[2], bit + bit_pos_b);
        else if ((type==CB_4x4)||(type==CB_4x8)||(type==CB_8x4)||(type==CB_16AC)||(type==CB_16DC))
          upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits[1], bit + bit_pos_b);
        else if ((type==CR_4x4)||(type==CR_4x8)||(type==CR_8x4)||(type==CR_16AC)||(type==CR_16DC))
          upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits[2], bit + bit_pos_b);
        else
          upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits[0], bit + bit_pos_b);
      }
    }

    if (block_a.available)
    {
      if(mb_data[block_a.mb_addr].mb_type==IPCM)
      {
        left_bit = 1;
      }
      else if((type==LUMA_8x8 || type==CB_8x8 || type==CR_8x8) &&
         !mb_data[block_a.mb_addr].luma_transform_size_8x8_flag)
      {
        left_bit = 0;
      }
      else
      {
        if(type==LUMA_8x8)
          left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits_8x8[0],bit + bit_pos_a);
        else if (type==CB_8x8)
          left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits_8x8[1],bit + bit_pos_a);
        else if (type==CR_8x8)
          left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits_8x8[2],bit + bit_pos_a);
        else if ((type==CB_4x4)||(type==CB_4x8)||(type==CB_8x4)||(type==CB_16AC)||(type==CB_16DC))
          left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits[1],bit + bit_pos_a);
        else if ((type==CR_4x4)||(type==CR_4x8)||(type==CR_8x4)||(type==CR_16AC)||(type==CR_16DC))
          left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits[2],bit + bit_pos_a);
        else
          left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits[0],bit + bit_pos_a);
      }
    }

    ctx = 2*upper_bit+left_bit;

    //===== encode symbol =====
    biari_encode_symbol (eep_dp, cbp_bit, tex_ctx->bcbp_contexts[type2ctx_bcbp[type]]+ctx);
  }
}


/*!
 ****************************************************************************
 * \brief
 *    Write CBP4-BIT
 ****************************************************************************
 */
void write_and_store_CBP_block_bit (Macroblock* currMB, EncodingEnvironmentPtr eep_dp, int type, int cbp_bit, TextureInfoContexts*  tex_ctx)
{
  VideoParameters *p_Vid = currMB->p_Vid;
  Macroblock *mb_data = p_Vid->mb_data;
  int y_ac        = (type==LUMA_16AC || type==LUMA_8x8 || type==LUMA_8x4 || type==LUMA_4x8 || type==LUMA_4x4);
  int y_dc        = (type==LUMA_16DC); 
  int u_ac        = (type==CHROMA_AC && !p_Vid->is_v_block);
  int v_ac        = (type==CHROMA_AC &&  p_Vid->is_v_block);
  int chroma_dc   = (type==CHROMA_DC || type==CHROMA_DC_2x4 || type==CHROMA_DC_4x4);
  int u_dc        = (chroma_dc && !p_Vid->is_v_block);
  int v_dc        = (chroma_dc &&  p_Vid->is_v_block);
  int j           = (y_ac || u_ac || v_ac ? currMB->subblock_y : 0);
  int i           = (y_ac || u_ac || v_ac ? currMB->subblock_x : 0);
  int bit         =  (y_dc ? 0 : y_ac ? 1 : u_dc ? 17 : v_dc ? 18 : u_ac ? 19 : 23);
  int default_bit = (currMB->is_intra_block ? 1 : 0);
  int upper_bit   = default_bit;
  int left_bit    = default_bit;
  int ctx;

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

  bit = (y_dc ? 0 : y_ac ? 1 + j + (i >> 2): u_dc ? 17 : v_dc ? 18 : u_ac ? 19 + j + (i >> 2): 35 + j + (i >> 2));
  //--- set bits for current block ---
  if (cbp_bit)
  {    
    if (type==LUMA_8x8)
    {
      currMB->cbp_bits[0] |= ((int64) 0x33 << bit);
    }
    else if (type==LUMA_8x4)
    {      
      currMB->cbp_bits[0] |= ((int64) 0x03 << bit);
    }
    else if (type==LUMA_4x8)
    {      
      currMB->cbp_bits[0] |= ((int64) 0x11 << bit);
    }
    else
    {
      currMB->cbp_bits[0] |= ((int64) 0x01 << bit);
    }
  }

  bit = (y_dc ? 0 : y_ac ? 1 : u_dc ? 17 : v_dc ? 18 : u_ac ? 19 : 35);

  if (type!=LUMA_8x8)
  {
    if (block_b.available)
    {
      if(mb_data[block_b.mb_addr].mb_type==IPCM)
        upper_bit = 1;
      else
        upper_bit = get_bit(mb_data[block_b.mb_addr].cbp_bits[0],bit+bit_pos_b);
    }


    if (block_a.available)
    {
      if(mb_data[block_a.mb_addr].mb_type == IPCM)
        left_bit = 1;
      else
        left_bit = get_bit(mb_data[block_a.mb_addr].cbp_bits[0], bit + bit_pos_a);
    }

    ctx = (upper_bit << 1) + left_bit;

    //===== encode symbol =====    
    biari_encode_symbol (eep_dp, cbp_bit, tex_ctx->bcbp_contexts[type2ctx_bcbp[type]] + ctx);
  }
}

/*!
****************************************************************************
* \brief
*    Write Significance MAP
****************************************************************************
*/
void write_significance_map (Macroblock* currMB, EncodingEnvironmentPtr eep_dp, int type, int coeff[], int coeff_ctr, TextureInfoContexts*  tex_ctx)
{
  int   k;
  int   sig, last;
  int   k0 = 0;
  int   k1 = maxpos[type];

#if ENABLE_FIELD_CTX
  VideoParameters *p_Vid = currMB->p_Vid;
  int               fld       = ( p_Vid->structure!=FRAME || currMB->mb_field );
#else
  int               fld       = 0;
#endif
  BiContextTypePtr  map_ctx   = tex_ctx->map_contexts [fld][type2ctx_map [type]];
  BiContextTypePtr  last_ctx  = tex_ctx->last_contexts[fld][type2ctx_last[type]];
  const byte *pos2ctxmap  = fld ? pos2ctx_map_int [type] : pos2ctx_map  [type];
  const byte *pos2ctxlast = pos2ctx_last[type];

  if (!c1isdc[type])
  {
    ++k0; 
    ++k1; 
    --coeff;
  }

  for (k=k0; k<k1; ++k) // if last coeff is reached, it has to be significant
  {
    sig   = (coeff[k] != 0);
    biari_encode_symbol  (eep_dp, sig,  map_ctx + pos2ctxmap[k]);
    if (sig)
    {
      last = (--coeff_ctr == 0);

      biari_encode_symbol(eep_dp, last, last_ctx + pos2ctxlast[k]);
      if (last) 
        return;
    }
  }
}


/*!
 ****************************************************************************
 * \brief
 *    Write Levels
 ****************************************************************************
 */
void write_significant_coefficients (EncodingEnvironmentPtr eep_dp, int type, int coeff[], TextureInfoContexts*  tex_ctx, int coeff_cnt)
{
  BiContextType *one_contexts = tex_ctx->one_contexts[type2ctx_one[type]];
  BiContextType *abs_contexts = tex_ctx->abs_contexts[type2ctx_abs[type]];  
  int   absLevel;
  int   ctx;
  short sign;
  int greater_one;
  int   c1 = 1;
  int   c2 = 0;
  int   i;

  for (i=maxpos[type]; (i>=0) && (coeff_cnt); i--)
  {
    if (coeff[i]!=0)
    {
      coeff_cnt--;

      if (coeff[i] > 0)
      {
        absLevel =  coeff[i];  
        sign = 0;
      }
      else            
      {
        absLevel = -coeff[i];  
        sign = 1;
      }

      greater_one = (absLevel > 1);

      //--- if coefficient is one ---
      ctx = imin(c1, 4);
      biari_encode_symbol (eep_dp, greater_one, one_contexts + ctx);

      if (greater_one)
      {
        ctx = imin(c2++, max_c2[type]);
        unary_exp_golomb_level_encode(eep_dp, absLevel - 2, abs_contexts + ctx);
        c1 = 0;
      }
      else if (c1)
      {
        c1++;
      }
      biari_encode_symbol_eq_prob (eep_dp, sign);
    }
  }
}



/*!
 ****************************************************************************
 * \brief
 *    Write Block-Transform Coefficients
 ****************************************************************************
 */
void writeRunLevel_CABAC (Macroblock* currMB, SyntaxElement *se, DataPartition *dp)
{  
  Slice *currSlice = currMB->p_Slice;
  EncodingEnvironmentPtr eep_dp = &(dp->ee_cabac);
  int curr_len = arienco_bits_written(eep_dp);

  //--- accumulate run-level information ---
  if (se->value1 != 0)
  {
    currSlice->pos += se->value2;
    currSlice->coeff[currSlice->pos++] = se->value1;
    ++currSlice->coeff_ctr;
  }
  else
  {
    TextureInfoContexts *tex_ctx = currSlice->tex_ctx;
    //===== encode CBP-BIT =====
    if (currSlice->coeff_ctr > 0)
    {
      currSlice->write_and_store_CBP_block_bit  (currMB, eep_dp, se->context, 1, tex_ctx);
      //===== encode significance map =====
      write_significance_map         (currMB, eep_dp, se->context, currSlice->coeff, currSlice->coeff_ctr, tex_ctx);
      //===== encode significant coefficients =====
      write_significant_coefficients (eep_dp, se->context, currSlice->coeff, tex_ctx, currSlice->coeff_ctr);

      memset(currSlice->coeff, 0 , 64 * sizeof(int));
    }
    else
      currSlice->write_and_store_CBP_block_bit  (currMB, eep_dp, se->context, 0, tex_ctx);

    //--- reset counters ---
    currSlice->pos = currSlice->coeff_ctr = 0;
  }

  dp->bitstream->write_flag = 1;
  se->len = (arienco_bits_written(eep_dp) - curr_len);
  CABAC_TRACE;
}


