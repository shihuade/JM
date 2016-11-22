
/*!
 ***************************************************************************
 * \file rdopt_coding_state.c
 *
 * \brief
 *    Storing/restoring coding state for
 *    Rate-Distortion optimized mode decision
 *
 * \author
 *    Heiko Schwarz
 *
 * \date
 *    17. April 2001
 **************************************************************************/

#include "global.h"

#include "rdopt_coding_state.h"
#include "cabac.h"
#include "memalloc.h"

/*!
 ************************************************************************
 * \brief
 *    delete structure for storing coding state
 ************************************************************************
 */
void delete_coding_state (CSobj *cs)
{
  if (cs != NULL)
  {
    //=== structures of data partition array ===
    if (cs->encenv    != NULL)
    {
      free (cs->encenv);
      cs->encenv = NULL;
    }
    if (cs->bitstream != NULL)
    {
      free (cs->bitstream);
      cs->bitstream = NULL;
    }

    //=== contexts for binary arithmetic coding ===
    delete_contexts_MotionInfo  (cs->mot_ctx);
    delete_contexts_TextureInfo (cs->tex_ctx);

    if (cs->cbp_bits_8x8 != NULL)
    {
      free (cs->cbp_bits_8x8);
      cs->cbp_bits_8x8 = NULL;
    }

    //=== coding state structure ===
    free (cs);
    cs=NULL;
  }
}


/*!
 ************************************************************************
 * \brief
 *    create structure for storing coding state
 ************************************************************************
 */
CSobj *create_coding_state (InputParameters *p_Inp)
{
  CSobj *cs;

  //=== coding state structure ===
  if ((cs = (CSobj *) calloc (1, sizeof(CSobj))) == NULL)
    no_mem_exit("init_coding_state: cs"); 

  //=== important variables of data partition array ===
  cs->no_part = p_Inp->partition_mode == 0 ? 1 : 3;
  if (p_Inp->symbol_mode == CABAC)
  {
    if ((cs->encenv = (EncodingEnvironment*) calloc (cs->no_part, sizeof(EncodingEnvironment))) == NULL)
      no_mem_exit("init_coding_state: cs->encenv");

    if ((cs->bitstream = (Bitstream*) calloc (cs->no_part, sizeof(Bitstream))) == NULL)
      no_mem_exit("init_coding_state: cs->bitstream");
    //=== context for binary arithmetic coding ===
    cs->mot_ctx = create_contexts_MotionInfo ();
    cs->tex_ctx = create_contexts_TextureInfo();

  }
  else
  {
    cs->encenv = NULL;
    if ((cs->bitstream = (Bitstream*) calloc (cs->no_part, sizeof(Bitstream))) == NULL)
      no_mem_exit("init_coding_state: cs->bitstream");

    cs->mot_ctx = NULL;
    cs->tex_ctx = NULL;
  }
  
  if (p_Inp->ProfileIDC == FREXT_Hi444)
  {
    if ((cs->cbp_bits_8x8 = (int64 *) calloc (3, sizeof(int64))) == NULL)
      no_mem_exit("init_coding_state: cs->cbp_bits_8x8"); 
  }
  else
    cs->cbp_bits_8x8 = NULL;

  return cs;
}

/*!
 ************************************************************************
 * \brief
 *    store coding state (for non rdo case. basically a dummy function)
 ************************************************************************
 */
static void store_coding_state_nordo (Macroblock *currMB, CSobj *cs)
{
}


/*!
 ************************************************************************
 * \brief
 *    store cavlc coding state (for rd-optimized mode decision)
 ************************************************************************
 */
void store_coding_state_cavlc (Macroblock *currMB, CSobj *cs)
{
  int  i;
  Slice *currSlice = currMB->p_Slice;
  int  i_last = currSlice->idr_flag? 1 : cs->no_part;  

  //=== important variables of data partition array ===
  for (i = 0; i < i_last; i++)
  {
    cs->bitstream[i] = *currSlice->partArr[i].bitstream;
  }

  //=== syntax element number and bitcounters ===
  cs->bits = currMB->bits;

  //=== elements of current macroblock ===
  if (currMB->mb_type <= P8x8)
    memcpy (cs->mvd, currMB->mvd, BLOCK_CONTEXT * sizeof(short));
  memcpy (cs->cbp_bits, currMB->cbp_bits, 3 * sizeof(int64));

  if (currSlice->P444_joined)
    memcpy (cs->cbp_bits_8x8, currMB->cbp_bits_8x8, 3 * sizeof(int64));
}

/*!
 ************************************************************************
 * \brief
 *    store cabac coding state (for rd-optimized mode decision)
 ************************************************************************
 */
static void store_coding_state_cabac (Macroblock *currMB, CSobj *cs)
{
  int  i;
  Slice *currSlice = currMB->p_Slice;
  int  i_last = currSlice->idr_flag? 1:cs->no_part;  
  DataPartition *partArr = &currSlice->partArr[0];

  //=== important variables of data partition array ===
  //only one partition for an IDR picture
  for (i = 0; i < i_last; i++)
  {
    cs->bitstream[i] = *partArr->bitstream;
    cs->encenv[i]    = (partArr++)->ee_cabac;    
  }

  //=== contexts for binary arithmetic coding ===
  *cs->mot_ctx = *currSlice->mot_ctx;
  *cs->tex_ctx = *currSlice->tex_ctx;

  //=== syntax element number and bitcounters ===
  cs->bits = currMB->bits;

  //=== elements of current macroblock ===
  if (currMB->mb_type <= P8x8)
    memcpy (cs->mvd, currMB->mvd, BLOCK_CONTEXT * sizeof(short));
  memcpy (cs->cbp_bits, currMB->cbp_bits, 3 * sizeof(int64));

  if (currSlice->P444_joined)
    memcpy (cs->cbp_bits_8x8, currMB->cbp_bits_8x8, 3 * sizeof(int64));
}

/*!
 ************************************************************************
 * \brief
 *    restore coding state (for non rdo case. basically a dummy function)
 ************************************************************************
 */
static void reset_coding_state_nordo (Macroblock *currMB, CSobj *cs)
{
}

/*!
 ************************************************************************
 * \brief
 *    restore coding state (for rd-optimized mode decision)
 ************************************************************************
 */
void reset_coding_state_cavlc (Macroblock *currMB, CSobj *cs)
{
  int  i;
  Slice *currSlice = currMB->p_Slice;
  int  i_last = currSlice->idr_flag? 1:cs->no_part;   

  //=== important variables of data partition array ===
  //only one partition for an IDR picture
  for (i = 0; i < i_last; i++)
  {
    //--- parameters of encoding environments ---
    *currSlice->partArr[i].bitstream = cs->bitstream[i];
  }

  //=== syntax element number and bit counters ===
  currMB->bits = cs->bits;

  //=== elements of current macroblock ===
  if (currMB->mb_type <= P8x8)
    memcpy (currMB->mvd, cs->mvd, BLOCK_CONTEXT * sizeof(short));
  memcpy (currMB->cbp_bits, cs->cbp_bits, 3 * sizeof(int64));
  if(currSlice->P444_joined)
    memcpy (currMB->cbp_bits_8x8, cs->cbp_bits_8x8, 3 * sizeof(int64));
}


/*!
 ************************************************************************
 * \brief
 *    restore coding state (for rd-optimized mode decision)
 ************************************************************************
 */
static void reset_coding_state_cabac (Macroblock *currMB, CSobj *cs)
{
  int  i;
  Slice *currSlice = currMB->p_Slice;
  int  i_last = currSlice->idr_flag? 1:cs->no_part;   
  DataPartition *partArr = &currSlice->partArr[0];

  //=== important variables of data partition array ===
  //only one partition for an IDR picture
  for (i = 0; i < i_last; i++)
  {
    //--- parameters of encoding environments ---
    *partArr->bitstream   = cs->bitstream[i];
    (partArr++)->ee_cabac = cs->encenv[i];
  }

  //=== contexts for binary arithmetic coding ===
  *currSlice->mot_ctx = *cs->mot_ctx;
  *currSlice->tex_ctx = *cs->tex_ctx;

  //=== syntax element number and bit counters ===
  currMB->bits = cs->bits;

  //=== elements of current macroblock ===
  if (currMB->mb_type <= P8x8)
    memcpy (currMB->mvd, cs->mvd, BLOCK_CONTEXT * sizeof(short));

  memcpy (currMB->cbp_bits, cs->cbp_bits, 3 * sizeof(int64));

  if(currSlice->P444_joined)
    memcpy (currMB->cbp_bits_8x8, cs->cbp_bits_8x8, 3 * sizeof(int64));
}




/*!
 ************************************************************************
 * \brief
 *    Select methods given entropy coding type
 ************************************************************************
 */
void init_coding_state_methods(Slice *currSlice)
{
  if (currSlice->p_Inp->rdopt == 0)
  {
    currSlice->reset_coding_state = reset_coding_state_nordo;
    currSlice->store_coding_state = store_coding_state_nordo;
  }
  else
  {
    if (currSlice->symbol_mode == CABAC)
    {
      currSlice->reset_coding_state = reset_coding_state_cabac;
      currSlice->store_coding_state = store_coding_state_cabac;
    }
    else
    {
      currSlice->reset_coding_state = reset_coding_state_cavlc;
      currSlice->store_coding_state = store_coding_state_cavlc;
    }
  }
}
