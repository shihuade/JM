
/*!
 ***************************************************************************
 * \file
 *    biariencode.h
 *
 * \brief
 *    Headerfile for binary arithmetic encoding routines
 *
 * \author
 *    - Detlev Marpe
 *    - Gabi Blaettermann
 *    - Gunnar Marten
 *
 *    Copyright (C) 2000 HEINRICH HERTZ INSTITUTE All Rights Reserved.
 *
 * \date
 *    21. Oct 2000
 **************************************************************************
 */


#ifndef _BIARIENCOD_H_
#define _BIARIENCOD_H_


/************************************************************************
 * D e f i n i t i o n s
 ***********************************************************************
 */

// some definitions to increase the readability of the source code

#define B_BITS         10    // Number of bits to represent the whole coding interval
#define BITS_TO_LOAD   16
#define MAX_BITS       26          //(B_BITS + BITS_TO_LOAD)
#define MASK_BITS      18          //(MAX_BITS - 8)
#define ONE            0x04000000  //(1 << MAX_BITS)
#define ONE_M1         0x03FFFFFF  //(ONE - 1)
#define HALF           0x01FE      //(1 << (B_BITS-1)) - 2
#define QUARTER        0x0100      //(1 << (B_BITS-2))
#define MIN_BITS_TO_GO 0
#define B_LOAD_MASK    0xFFFF      // ((1<<BITS_TO_LOAD) - 1)

extern int get_pic_bin_count(VideoParameters *p_Vid);
extern void reset_pic_bin_count(VideoParameters *p_Vid);
extern void set_pic_bin_count  (VideoParameters *p_Vid, EncodingEnvironmentPtr eep);

extern void arienco_start_encoding(EncodingEnvironmentPtr eep, unsigned char *code_buffer, int *code_len);
extern void arienco_reset_EC      (EncodingEnvironmentPtr eep);
extern void arienco_done_encoding (Macroblock *currMB, EncodingEnvironmentPtr eep);
extern void biari_init_context    (int qp, BiContextTypePtr ctx, const char* ini);
extern void biari_encode_symbol   (EncodingEnvironmentPtr eep, int symbol, BiContextTypePtr bi_ct );
extern void biari_encode_symbol_eq_prob(EncodingEnvironmentPtr eep, int symbol);
extern void biari_encode_symbol_final(EncodingEnvironmentPtr eep, int symbol);

/*!
************************************************************************
* \brief
*    Returns the number of currently written bits
************************************************************************
*/
static inline int arienco_bits_written(EncodingEnvironmentPtr eep)
{
  return (((*eep->Ecodestrm_len) + eep->Epbuf + 1) << 3) + (eep->Echunks_outstanding * BITS_TO_LOAD) + BITS_TO_LOAD - eep->Ebits_to_go;
}

#endif  // BIARIENCOD_H

