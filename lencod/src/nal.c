
/*!
 **************************************************************************************
 * \file
 *    nal.c
 * \brief
 *    Handles the operations on converting String of Data Bits (SODB)
 *    to Raw Byte Sequence Payload (RBSP), and then
 *    onto Encapsulate Byte Sequence Payload (EBSP).
 *  \date 14 June 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Shankar Regunathan                  <shanre@microsoft.de>
 *      - Stephan Wenger                      <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */

#include "contributors.h"
#include "global.h"
#include "enc_statistics.h"
#include "biariencode.h"
#include "nal.h"

static const int MbWidthC  [4]= { 0, 8, 8,  16};
static const int MbHeightC [4]= { 0, 8, 16, 16};

 /*!
 ************************************************************************
 * \brief
 *    Converts String Of Data Bits (SODB) to Raw Byte Sequence
 *    Packet (RBSP)
 * \param currStream
 *        Bitstream which contains data bits.
 * \return None
 * \note currStream is byte-aligned at the end of this function
 *
 ************************************************************************
*/

void SODBtoRBSP(Bitstream *currStream)
{
  currStream->byte_buf <<= 1;
  currStream->byte_buf |= 1;
  currStream->bits_to_go--;
  currStream->byte_buf <<= currStream->bits_to_go;
  currStream->streamBuffer[currStream->byte_pos++] = currStream->byte_buf;
  currStream->bits_to_go = 8;
  currStream->byte_buf = 0;
}


/*!
************************************************************************
*  \brief
*     This function add emulation_prevention_three_byte for all occurrences
*     of the following byte sequences in the stream
*       0x000000  -> 0x00000300
*       0x000001  -> 0x00000301
*       0x000002  -> 0x00000302
*       0x000003  -> 0x00000303
*
*  \param NaluBuffer
*            pointer to target buffer
*  \param rbsp
*            pointer to source buffer
*  \param rbsp_size
*           Size of source
*  \return
*           Size target buffer after emulation prevention.
*
************************************************************************
*/

int RBSPtoEBSP(byte *NaluBuffer, unsigned char *rbsp, int rbsp_size)
{
  int j     = 0;
  int count = 0;
  int i;

  for(i = 0; i < rbsp_size; i++)
  {
    if(count == ZEROBYTES_SHORTSTARTCODE && !(rbsp[i] & 0xFC))
    {
      NaluBuffer[j] = 0x03;
      j++;
      count = 0;
    }
    NaluBuffer[j] = rbsp[i];
    if(rbsp[i] == 0x00)
      count++;
    else
      count = 0;
    j++;
  }

  return j;
}

/*!
************************************************************************
*  \brief
*     This function adds cabac_zero_word syntax elements at the end of the
*     NAL unit to
*
*  \param p_Vid
*            VideoParameters structure
*  \param nalu
*            target NAL unit
*  \param cur_stats
*            currently used statistics parameters
*  \return
*           number of added bytes
*
************************************************************************
*/
int addCabacZeroWords(VideoParameters *p_Vid, NALU_t *nalu, StatParameters *cur_stats)
{
  seq_parameter_set_rbsp_t *active_sps = p_Vid->active_sps;

  int stuffing_bytes = 0;
  int i = 0;

  byte *buf = &nalu->buf[nalu->len];

  int RawMbBits = 256 * p_Vid->bitdepth_luma + 2 * MbWidthC[active_sps->chroma_format_idc] * MbHeightC[active_sps->chroma_format_idc] * p_Vid->bitdepth_chroma;
  int min_num_bytes = ((96 * get_pic_bin_count(p_Vid)) - (RawMbBits * (int)p_Vid->PicSizeInMbs *3) + 1023) / 1024;

  if (min_num_bytes > p_Vid->bytes_in_picture)
  {
    stuffing_bytes = min_num_bytes - p_Vid->bytes_in_picture;
    printf ("Inserting %d/%d cabac_zero_word syntax elements/bytes (Clause 7.4.2.10)\n", ((stuffing_bytes + 2)/3), stuffing_bytes);  

    for (i = 0; i < stuffing_bytes; i+=3 )
    {
      *buf++ = 0x00; // CABAC zero word
      *buf++ = 0x00;
      *buf++ = 0x03;
    }
    cur_stats->bit_use_stuffing_bits[p_Vid->type] += (i<<3);
    nalu->len += i;
  }  

  return i;
}

