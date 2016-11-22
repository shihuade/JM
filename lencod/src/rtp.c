
/*!
 *****************************************************************************
 * \file rtp.c
 *
 * \brief
 *    Functions to handle RTP headers and packets per RFC 3984
 *    with restricted functionality.
 *
 * \author
 *    Stephan Wenger    stewe@cs.tu-berlin.de
 *****************************************************************************/

#ifdef WIN32
#include <Winsock2.h>
#else
#include <netinet/in.h>
#endif

#include "global.h"
#include "rtp.h"
#include "sei.h"

// A little trick to avoid those horrible #if TRACE all over the source code
#if TRACE
#define SYMTRACESTRING(s) strncpy(sym.tracestring,s,TRACESTRING_SIZE)
#else
#define SYMTRACESTRING(s) // to nothing
#endif

/*!
 *****************************************************************************
 *
 * \brief
 *    ComposeRTPpacket composes the complete RTP packet using the various
 *    structure members of the RTPpacket_t structure
 *
 * \return
 *    0 in case of success
 *    negative error code in case of failure
 *
 * \par Parameters
 *    Caller is responsible to allocate enough memory for the generated packet
 *    in parameter->packet. Typically a malloc of 12+paylen bytes is sufficient
 *
 * \par Side effects
 *    none
 *
 * \note
 *    Function contains assert() tests for debug purposes (consistency checks
 *    for RTP header fields
 *
 * \date
 *    30 September 2001
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 *****************************************************************************/

int ComposeRTPPacket (RTPpacket_t *p)

{
  unsigned int temp32;
  uint16 temp16;

  // Consistency checks through assert, only used for debug purposes
  assert (p->v == 2);
  assert (p->p == 0);
  assert (p->x == 0);
  assert (p->cc == 0);    // mixer designers need to change this one
  assert (p->m == 0 || p->m == 1);
  assert (p->pt < 128);
  assert (p->payload != NULL);
  assert (p->paylen < 65536 - 40);  // 2**16 -40 for IP/UDP/RTP header
  assert (p->packet != NULL);

  // Compose RTP header, little endian

  p->packet[0] = (byte)
    ( ((p->v  & 0x03) << 6)
    | ((p->p  & 0x01) << 5)
    | ((p->x  & 0x01) << 4)
    | ((p->cc & 0x0F) << 0) );

  p->packet[1] = (byte)
    ( ((p->m  & 0x01) << 7)
    | ((p->pt & 0x7F) << 0) );

  // sequence number, msb first
  temp16 = htons((uint16)p->seq);
  memcpy (&p->packet[2], &temp16, 2);  // change to shifts for unified byte sex

  //declare a temporary variable to perform network byte order conversion
  temp32 = htonl(p->timestamp);
  memcpy (&p->packet[4], &temp32, 4);  // change to shifts for unified byte sex

  temp32 = htonl(p->ssrc);
  memcpy (&p->packet[8], &temp32, 4);// change to shifts for unified byte sex

  // Copy payload

  memcpy (&p->packet[12], p->payload, p->paylen);
  p->packlen = p->paylen+12;
  return 0;
}



/*!
 *****************************************************************************
 *
 * \brief
 *    WriteRTPPacket writes the supplied RTP packet to the output file
 *
 * \return
 *    0 in case of access
 *    <0 in case of write failure (typically fatal)
 *
 * \param p
 *    the RTP packet to be written (after ComposeRTPPacket() )
 * \param f
 *    output file
 *
 * \date
 *    October 23, 2001
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 *****************************************************************************/

int WriteRTPPacket (RTPpacket_t *p, FILE *f)

{
  int intime = -1;

  assert (f != NULL);
  assert (p != NULL);


  if (1 != fwrite (&p->packlen, 4, 1, f))
    return -1;
  if (1 != fwrite (&intime, 4, 1, f))
    return -1;
  if (1 != fwrite (p->packet, p->packlen, 1, f))
    return -1;
  return 0;
}





/*!
 *****************************************************************************
 *
 * \brief
 *    int RTPWriteNALU write a NALU to the RTP file
 *
 * \return
 *    Number of bytes written to output file
 *
 * \par Side effects
 *    Packet written, RTPSequenceNumber and RTPTimestamp updated
 *
 * \date
 *    December 13, 2002
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 *****************************************************************************/


int WriteRTPNALU (VideoParameters *p_Vid, NALU_t *n, FILE **f_rtp)
{
  RTPpacket_t *p;
  
  byte first_byte;

  assert ((*f_rtp) != NULL);
  assert (n != NULL);
  assert (n->len < 65000);

  first_byte = (byte)
    (n->forbidden_bit << 7      |
     n->nal_reference_idc << 5  |
     n->nal_unit_type );

  // Set RTP structure elements and alloca() memory foor the buffers
  if ((p = (RTPpacket_t *) malloc (sizeof (RTPpacket_t))) == NULL)
    no_mem_exit ("RTPWriteNALU-1");
  if ((p->packet = malloc (MAXRTPPACKETSIZE)) == NULL)
    no_mem_exit ("RTPWriteNALU-2");
  if ((p->payload = malloc (MAXRTPPACKETSIZE)) == NULL)
    no_mem_exit ("RTPWriteNALU-3");

  p->v=2;
  p->p=0;
  p->x=0;
  p->cc=0;
  p->m=(n->startcodeprefix_len==4)&1;     // a long startcode of Annex B sets marker bit of RTP
                                          // Not exactly according to the RTP paylaod spec, but
                                          // good enough for now (hopefully).
                                          //! For error resilience work, we need the correct
                                          //! marker bit.  Introduce a nalu->marker and set it in
                                          //! terminate_slice()?
  p->pt=H264PAYLOADTYPE;
  p->seq = p_Vid->CurrentRTPSequenceNumber++;
  p->timestamp = p_Vid->CurrentRTPTimestamp;
  p->ssrc=H264SSRC;
  p->paylen = 1 + n->len;

  p->payload[0] = first_byte;
  memcpy (p->payload+1, n->buf, n->len);

  // Generate complete RTP packet
  if (ComposeRTPPacket (p) < 0)
  {
    printf ("Cannot compose RTP packet, exit\n");
    exit (-1);
  }
  if (WriteRTPPacket (p, *f_rtp) < 0)
  {
    printf ("Cannot write %d bytes of RTP packet to outfile, exit\n", p->packlen);
    exit (-1);
  }
  free (p->packet);
  free (p->payload);
  free (p);
  return (n->len * 8);
}


/*!
 ********************************************************************************************
 * \brief
 *    RTPUpdateTimestamp: patches the RTP timestamp depending on the TR
 *
 * \param p_Vid
 *    Image parameters for current picture encoding
 * \param tr
 *    tr: TRof the following NALUs
 *
 * \return
 *    none.
 *
 ********************************************************************************************
*/


void RTPUpdateTimestamp (VideoParameters *p_Vid, int tr)
{
  int delta;
  static int oldtr = -1;

  if (oldtr == -1)            // First invocation
  {
    p_Vid->CurrentRTPTimestamp = 0;  //! This is a violation of the security req. of
                              //! RTP (random timestamp), but easier to debug
    oldtr = 0;
    return;
  }

  /*! The following code assumes a wrap around of TR at 256, and
      needs to be changed as soon as this is no more true.

      The support for B frames is a bit tricky, because it is not easy to distinguish
      between a natural wrap-around of the tr, and the intentional going back of the
      tr because of a B frame.  It is solved here by a heuristic means: It is assumed that
      B frames are never "older" than 10 tr ticks.  Everything higher than 10 is considered
      a wrap around.
  */

  delta = tr - oldtr;

  if (delta < -10)        // wrap-around
    delta+=256;

  p_Vid->CurrentRTPTimestamp += delta * RTP_TR_TIMESTAMP_MULT;
  oldtr = tr;
}


/*!
 ********************************************************************************************
 * \brief
 *    Opens the output file for the RTP packet stream
 *
 * \param p_Vid
 *    Image parameters for current picture encoding
 * \param Filename
 *    The filename of the file to be opened
 *
 * \return
 *    none.  Function terminates the program in case of an error
 *
 ********************************************************************************************
*/

void OpenRTPFile (char *Filename, FILE **f_rtp)
{
  if (((*f_rtp) = fopen (Filename, "wb")) == NULL)
  {
    printf ("Fatal: cannot open bitstream file '%s', exit (-1)\n", Filename);
    exit (-1);
  }
}


/*!
 ********************************************************************************************
 * \brief
 *    Closes the output file for the RTP packet stream
 *
 * \return
 *    none.  Function terminates the program in case of an error
 *
 ********************************************************************************************
*/

void CloseRTPFile (FILE *f_rtp)
{
  fclose(f_rtp);
}

