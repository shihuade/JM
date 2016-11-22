
/*!
 *************************************************************************************
 * \file biariencode.c
 *
 * \brief
 *   Routines for binary arithmetic encoding.
 *
 *   This modified implementation of the M Coder is based on JVT-U084 
 *   with the choice of M_BITS = 16.
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Detlev Marpe
 *    - Gabi Blaettermann
 *    - Gunnar Marten
 *************************************************************************************
 */

#include "global.h"
#include "biariencode.h"

// Range table for LPS
static const byte renorm_table_32[32]={6,5,4,4,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};


void reset_pic_bin_count(VideoParameters *p_Vid)
{
  p_Vid->pic_bin_count = 0;
}

int get_pic_bin_count(VideoParameters *p_Vid)
{
  return p_Vid->pic_bin_count;
}

/*!
 ************************************************************************
 * \brief
 *    Allocates memory for the EncodingEnvironment struct
 ************************************************************************
 */
EncodingEnvironmentPtr arienco_create_encoding_environment(void)
{
  EncodingEnvironmentPtr eep;

  if ( (eep = (EncodingEnvironmentPtr) calloc(1,sizeof(EncodingEnvironment))) == NULL)
    no_mem_exit("arienco_create_encoding_environment: eep");

  return eep;
}

/*!
 ************************************************************************
 * \brief
 *    Frees memory of the EncodingEnvironment struct
 ************************************************************************
 */
void arienco_delete_encoding_environment(EncodingEnvironmentPtr eep)
{
  free_pointer(eep);
}

/*!
 ************************************************************************
 * inline functions for writing bytes of code
 ***********************************************************************
 */

static inline void put_one_byte_final(EncodingEnvironmentPtr eep, unsigned int b)
{
  eep->Ecodestrm[(*eep->Ecodestrm_len)++] = (byte) b;
}

static forceinline void put_buffer(EncodingEnvironmentPtr eep)
{
  while(eep->Epbuf>=0)
  {
    eep->Ecodestrm[(*eep->Ecodestrm_len)++] =  (byte) ((eep->Ebuffer>>((eep->Epbuf--)<<3))&0xFF); 
  }
  while(eep->C > 7)
  {
    eep->C-=8;
    ++(eep->E);
  }
  eep->Ebuffer = 0; 
}

static inline void put_one_byte(EncodingEnvironmentPtr eep, int b) 
{ 
  if(eep->Epbuf == 3)
  { 
    put_buffer(eep);
  } 
  eep->Ebuffer <<= 8;
  eep->Ebuffer += (b);

  ++(eep->Epbuf);
}

static inline void put_one_word(EncodingEnvironmentPtr eep, int b) 
{
  if (eep->Epbuf >= 3)
  { 
    put_buffer(eep);
  }
  eep->Ebuffer <<= 16;
  eep->Ebuffer += (b);

  eep->Epbuf += 2;
}

static forceinline void propagate_carry(EncodingEnvironmentPtr eep)
{
  ++(eep->Ebuffer); 
  while (eep->Echunks_outstanding > 0) 
  { 
    put_one_word(eep, 0); 
    --(eep->Echunks_outstanding); 
  }
}

static inline void put_last_chunk_plus_outstanding(EncodingEnvironmentPtr eep, unsigned int l) 
{
  while (eep->Echunks_outstanding > 0)
  {
    put_one_word(eep, 0xFFFF);
    --(eep->Echunks_outstanding);
  }
  put_one_word(eep, l);
}

static inline void put_last_chunk_plus_outstanding_final(EncodingEnvironmentPtr eep, unsigned int l) 
{
  while (eep->Echunks_outstanding > 0)
  {
    put_one_word(eep, 0xFFFF);
    --(eep->Echunks_outstanding);
  }
  put_one_byte(eep, l);
}

/*!
************************************************************************
* \brief
*    Initializes the EncodingEnvironment E and C values to zero
************************************************************************
*/
void arienco_reset_EC(EncodingEnvironmentPtr eep)
{
  eep->E = 0;
  eep->C = 0;
}

/*!
 ************************************************************************
 * \brief
 *    Initializes the EncodingEnvironment for the arithmetic coder
 ************************************************************************
 */
void arienco_start_encoding(EncodingEnvironmentPtr eep,
                            unsigned char *code_buffer,
                            int *code_len )
{
  eep->Elow = 0;
  eep->Echunks_outstanding = 0;
  eep->Ebuffer = 0;
  eep->Epbuf = -1;  // to remove redundant chunk ^^
  eep->Ebits_to_go = BITS_TO_LOAD + 1; // to swallow first redundant bit

  eep->Ecodestrm = code_buffer;
  eep->Ecodestrm_len = code_len;

  eep->Erange = HALF;
}

/*!
************************************************************************
* \brief
*    add slice bin number to picture bin counter
*    should be only used when slice is terminated
************************************************************************
*/
void set_pic_bin_count(VideoParameters *p_Vid, EncodingEnvironmentPtr eep)
{
  p_Vid->pic_bin_count += (eep->E << 3) + eep->C; // no of processed bins
}

/*!
 ************************************************************************
 * \brief
 *    Terminates the arithmetic codeword, writes stop bit and stuffing bytes (if any)
 ************************************************************************
 */
void arienco_done_encoding(Macroblock *currMB, EncodingEnvironmentPtr eep)
{
  unsigned int low = eep->Elow;  
  int remaining_bits = BITS_TO_LOAD - eep->Ebits_to_go; // output (2 + remaining) bits for terminating the codeword + one stop bit
  unsigned char mask;
  BitCounter *mbBits = &currMB->bits;

  //p_Vid->pic_bin_count += eep->E*8 + eep->C; // no of processed bins

  if (remaining_bits <= 5) // one terminating byte 
  {
    mbBits->mb_stuffing += (5 - remaining_bits);
    mask = (unsigned char) (255 - ((1 << (6 - remaining_bits)) - 1)); 
    low = (low >> (MASK_BITS)) & mask; // mask out the (2+remaining_bits) MSBs
    low += (32 >> remaining_bits);       // put the terminating stop bit '1'

    put_last_chunk_plus_outstanding_final(eep, low);
    put_buffer(eep);
  }
  else if(remaining_bits <= 13)            // two terminating bytes
  {
    mbBits->mb_stuffing += (13 - remaining_bits);
    put_last_chunk_plus_outstanding_final(eep, ((low >> (MASK_BITS)) & 0xFF)); // mask out the 8 MSBs for output

    put_buffer(eep);
    if (remaining_bits > 6)
    {
      mask = (unsigned char) (255 - ((1 << (14 - remaining_bits)) - 1)); 
      low = (low >> (B_BITS)) & mask; 
      low += (0x2000 >> remaining_bits);     // put the terminating stop bit '1'
      put_one_byte_final(eep, low);
    }
    else
    {
      put_one_byte_final(eep, 128); // second byte contains terminating stop bit '1' only
    }
  }
  else             // three terminating bytes
  { 
    put_last_chunk_plus_outstanding(eep, ((low >> B_BITS) & B_LOAD_MASK)); // mask out the 16 MSBs for output
    put_buffer(eep);
    mbBits->mb_stuffing += (21 - remaining_bits);

    if (remaining_bits > 14)
    {
      mask = (unsigned char) (255 - ((1 << (22 - remaining_bits)) - 1)); 
      low = (low >> (MAX_BITS - 24)) & mask; 
      low += (0x200000 >> remaining_bits);       // put the terminating stop bit '1'
      put_one_byte_final(eep, low);
    }
    else
    {
      put_one_byte_final(eep, 128); // third byte contains terminating stop bit '1' only
    }
  }
  eep->Ebits_to_go = 8;
}


/*!
 ************************************************************************
 * \brief
 *    Actually arithmetic encoding of one binary symbol by using
 *    the probability estimate of its associated context model
 ************************************************************************
 */
void biari_encode_symbol(EncodingEnvironmentPtr eep, int symbol, BiContextTypePtr bi_ct )
{
  static const byte rLPS_table_64x4[64][4]=
  {
    { 128, 176, 208, 240},
    { 128, 167, 197, 227},
    { 128, 158, 187, 216},
    { 123, 150, 178, 205},
    { 116, 142, 169, 195},
    { 111, 135, 160, 185},
    { 105, 128, 152, 175},
    { 100, 122, 144, 166},
    {  95, 116, 137, 158},
    {  90, 110, 130, 150},
    {  85, 104, 123, 142},
    {  81,  99, 117, 135},
    {  77,  94, 111, 128},
    {  73,  89, 105, 122},
    {  69,  85, 100, 116},
    {  66,  80,  95, 110},
    {  62,  76,  90, 104},
    {  59,  72,  86,  99},
    {  56,  69,  81,  94},
    {  53,  65,  77,  89},
    {  51,  62,  73,  85},
    {  48,  59,  69,  80},
    {  46,  56,  66,  76},
    {  43,  53,  63,  72},
    {  41,  50,  59,  69},
    {  39,  48,  56,  65},
    {  37,  45,  54,  62},
    {  35,  43,  51,  59},
    {  33,  41,  48,  56},
    {  32,  39,  46,  53},
    {  30,  37,  43,  50},
    {  29,  35,  41,  48},
    {  27,  33,  39,  45},
    {  26,  31,  37,  43},
    {  24,  30,  35,  41},
    {  23,  28,  33,  39},
    {  22,  27,  32,  37},
    {  21,  26,  30,  35},
    {  20,  24,  29,  33},
    {  19,  23,  27,  31},
    {  18,  22,  26,  30},
    {  17,  21,  25,  28},
    {  16,  20,  23,  27},
    {  15,  19,  22,  25},
    {  14,  18,  21,  24},
    {  14,  17,  20,  23},
    {  13,  16,  19,  22},
    {  12,  15,  18,  21},
    {  12,  14,  17,  20},
    {  11,  14,  16,  19},
    {  11,  13,  15,  18},
    {  10,  12,  15,  17},
    {  10,  12,  14,  16},
    {   9,  11,  13,  15},
    {   9,  11,  12,  14},
    {   8,  10,  12,  14},
    {   8,   9,  11,  13},
    {   7,   9,  11,  12},
    {   7,   9,  10,  12},
    {   7,   8,  10,  11},
    {   6,   8,   9,  11},
    {   6,   7,   9,  10},
    {   6,   7,   8,   9},
    {   2,   2,   2,   2}
  };
  static const byte AC_next_state_MPS_64[64] =
  {
    1,2,3,4,5,6,7,8,9,10,
    11,12,13,14,15,16,17,18,19,20,
    21,22,23,24,25,26,27,28,29,30,
    31,32,33,34,35,36,37,38,39,40,
    41,42,43,44,45,46,47,48,49,50,
    51,52,53,54,55,56,57,58,59,60,
    61,62,62,63
  };

  static const byte AC_next_state_LPS_64[64] =
  {
    0, 0, 1, 2, 2, 4, 4, 5, 6, 7,
    8, 9, 9,11,11,12,13,13,15,15,
    16,16,18,18,19,19,21,21,22,22,
    23,24,24,25,26,26,27,27,28,29,
    29,30,30,30,31,32,32,33,33,33,
    34,34,35,35,35,36,36,36,37,37,
    37,38,38,63
  };

  unsigned int low = eep->Elow;
  unsigned int range = eep->Erange;
  int bl = eep->Ebits_to_go;
  unsigned int rLPS = rLPS_table_64x4[bi_ct->state][(range>>6) & 3]; 

  range -= rLPS;

  ++(eep->C);
  bi_ct->count += eep->p_Vid->cabac_encoding;

  /* covers all cases where code does not bother to shift down symbol to be 
  * either 0 or 1, e.g. in some cases for cbp, mb_Type etc the code simply 
  * masks off the bit position and passes in the resulting value */
  //symbol = (short) (symbol != 0);

  if ((symbol != 0) == bi_ct->MPS)  //MPS
  {
    bi_ct->state = AC_next_state_MPS_64[bi_ct->state]; // next state

    if( range >= QUARTER ) // no renorm
    {
      eep->Erange = range;
      return;
    }
    else 
    {   
      range<<=1;
      if( --bl > MIN_BITS_TO_GO )  // renorm once, no output
      {
        eep->Erange = range;
        eep->Ebits_to_go = bl;
        return;
      }
    }
  }
  else         //LPS
  {
    unsigned int renorm = renorm_table_32[(rLPS >> 3) & 0x1F];

    low += range << bl;
    range = (rLPS << renorm);
    bl -= renorm;

    if (!bi_ct->state)
      bi_ct->MPS ^= 0x01;               // switch MPS if necessary

    bi_ct->state = AC_next_state_LPS_64[bi_ct->state]; // next state

    if (low >= ONE) // output of carry needed
    {
      low -= ONE;
      propagate_carry(eep);
    }

    if( bl > MIN_BITS_TO_GO )
    {
      eep->Elow   = low;
      eep->Erange = range;      
      eep->Ebits_to_go = bl;
      return;
    }
  }

  //renorm needed
  eep->Elow = (low << BITS_TO_LOAD )& (ONE_M1);
  low = (low >> B_BITS) & B_LOAD_MASK; // mask out the 8/16 MSBs for output

  if (low < B_LOAD_MASK) // no carry possible, output now
  {
    put_last_chunk_plus_outstanding(eep, low);
  }
  else          // low == "FF.."; keep it, may affect future carry
  {
    ++(eep->Echunks_outstanding);
  }
  eep->Erange = range;  
  eep->Ebits_to_go = bl + BITS_TO_LOAD;
}

/*!
 ************************************************************************
 * \brief
 *    Arithmetic encoding of one binary symbol assuming 
 *    a fixed prob. distribution with p(symbol) = 0.5
 ************************************************************************
 */
void biari_encode_symbol_eq_prob(EncodingEnvironmentPtr eep, int symbol)
{
  unsigned int low = eep->Elow;
  --(eep->Ebits_to_go);  
  ++(eep->C);

  if (symbol != 0)
  {
    low += eep->Erange << eep->Ebits_to_go;
    if (low >= ONE) // output of carry needed
    {
      low -= ONE;
      propagate_carry(eep);
    }
  }
  if(eep->Ebits_to_go == MIN_BITS_TO_GO)  // renorm needed
  {
    eep->Elow = (low << BITS_TO_LOAD )& (ONE_M1);
    low = (low >> B_BITS) & B_LOAD_MASK; // mask out the 8/16 MSBs for output
    if (low < B_LOAD_MASK)      // no carry possible, output now
    {
      put_last_chunk_plus_outstanding(eep, low);}
    else          // low == "FF"; keep it, may affect future carry
    {
      ++(eep->Echunks_outstanding);
    }

    eep->Ebits_to_go = BITS_TO_LOAD;
    return;
  }
  else         // no renorm needed
  {
    eep->Elow = low;
    return;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Arithmetic encoding for last symbol before termination
 ************************************************************************
 */
void biari_encode_symbol_final(EncodingEnvironmentPtr eep, int symbol)
{
  unsigned int range = eep->Erange - 2;
  unsigned int low = eep->Elow;
  int bl = eep->Ebits_to_go; 

  ++(eep->C);

  if (symbol == 0) // MPS
  {
    if( range >= QUARTER ) // no renorm
    {
      eep->Erange = range;
      return;
    }
    else 
    {   
      range<<=1;
      if( --bl > MIN_BITS_TO_GO )  // renorm once, no output
      {
        eep->Erange = range;
        eep->Ebits_to_go = bl;
        return;
      }
    }
  }
  else     // LPS
  {
    low += (range << bl);
    range = 2;

    if (low >= ONE) // output of carry needed
    {
      low -= ONE; // remove MSB, i.e., carry bit
      propagate_carry(eep);
    }
    bl -= 7; // 7 left shifts needed to renormalize

    range <<= 7;
    if( bl > MIN_BITS_TO_GO )
    {
      eep->Erange = range;
      eep->Elow   = low;
      eep->Ebits_to_go = bl;
      return;
    }
  }


  //renorm needed
  eep->Elow = (low << BITS_TO_LOAD ) & (ONE_M1);
  low = (low >> B_BITS) & B_LOAD_MASK; // mask out the 8/16 MSBs
  if (low < B_LOAD_MASK)
  {  // no carry possible, output now
    put_last_chunk_plus_outstanding(eep, low);
  }
  else
  {  // low == "FF"; keep it, may affect future carry
    ++(eep->Echunks_outstanding);
  }

  eep->Erange = range;
  bl += BITS_TO_LOAD;
  eep->Ebits_to_go = bl;
}

/*!
 ************************************************************************
 * \brief
 *    Initializes a given context with some pre-defined probability state
 ************************************************************************
 */
void biari_init_context (int qp, BiContextTypePtr ctx, const char* ini)
{
  int pstate = ((ini[0]* qp) >> 4) + ini[1];

  if ( pstate >= 64 )
  {
    pstate = imin(126, pstate);
    ctx->state = (uint8) (pstate - 64);
    ctx->MPS   = 1;
  }
  else
  {
    pstate = imax(1, pstate);
    ctx->state = (uint8) (63 - pstate);
    ctx->MPS   = 0;
  }

  ctx->count = 0;
}

