
/*!
 ***************************************************************************
 * \file
 *    rdopt_coding_state.h
 *
 * \author
 *    Heiko Schwarz
 *
 * \date
 *    17. April 2001
 *
 * \brief
 *    Headerfile for storing/restoring coding state
 *    (for rd-optimized mode decision)
 **************************************************************************
 */

#ifndef _RD_OPT_CS_H_
#define _RD_OPT_CS_H_

struct coding_state {

  // important variables of data partition array
  int                  no_part;
  Bitstream            *bitstream;
  EncodingEnvironment  *encenv;

  // contexts for binary arithmetic coding
  MotionInfoContexts   *mot_ctx;
  TextureInfoContexts  *tex_ctx;

  // bit counter
  BitCounter            bits;

  // elements of current macroblock
  short                 mvd[2][BLOCK_MULTIPLE][BLOCK_MULTIPLE][2];
  int64                 cbp_bits[3];
  int64                 *cbp_bits_8x8;
};

typedef struct coding_state CSobj;

extern void  delete_coding_state  (CSobj *);  //!< delete structure
extern CSobj *create_coding_state  (InputParameters *p_Inp);       //!< create structure

extern void init_coding_state_methods(Slice *currSlice);  //!< Init methods given entropy coding

extern void store_coding_state_cavlc (Macroblock *currMB, CSobj *cs);
extern void reset_coding_state_cavlc (Macroblock *currMB, CSobj *cs);

#endif

