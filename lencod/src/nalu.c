
/*!
 ************************************************************************
 * \file  nalu.c
 *
 * \brief
 *    Common NALU support functions
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Stephan Wenger   <stewe@cs.tu-berlin.de>
 ************************************************************************
 */

#include "global.h"
#include "nalu.h"
#include "nal.h"

/*!
 *************************************************************************************
 * \brief
 *    Converts an RBSP to a NALU
 *
 * \param rbsp
 *    byte buffer with the rbsp
 * \param nalu
 *    nalu structure to be filled
 * \param rbsp_size
 *    size of the rbsp in bytes
 * \param nal_unit_type
 *    as in JVT doc
 * \param nal_reference_idc
 *    as in JVT doc
 * \param UseAnnexbLongStartcode
 *    some incomprehensible CABAC stuff
 * \param UseAnnexbLongStartcode
 *    when 1 and when using AnnexB bytestreams, then use a long startcode prefix
 *
 * \return
 *    length of the NALU in bytes
 *************************************************************************************
 */

int RBSPtoNALU (unsigned char *rbsp, NALU_t *nalu, int rbsp_size, int nal_unit_type, int nal_reference_idc, int UseAnnexbLongStartcode)
{
  int len;

  assert (nalu != NULL);
  assert (nal_reference_idc <=3 && nal_reference_idc >=0);
#if (MVC_EXTENSION_ENABLE)
  assert (nal_unit_type > 0 && nal_unit_type <= NALU_TYPE_SLC_EXT);
#else
  assert (nal_unit_type > 0 && nal_unit_type <= NALU_TYPE_FILL);
#endif
  assert (rbsp_size < MAXRBSPSIZE);
  
  nalu->startcodeprefix_len = UseAnnexbLongStartcode ? 4 : 3;
  nalu->forbidden_bit       = 0;  
  nalu->nal_reference_idc   = (NalRefIdc) nal_reference_idc;
  nalu->nal_unit_type       = (NaluType) nal_unit_type;

#if (MVC_EXTENSION_ENABLE)
  if(nal_unit_type==NALU_TYPE_PREFIX || nal_unit_type==NALU_TYPE_SLC_EXT)
  {
    nalu->svc_extension_flag = 0;
    //nalu->non_idr_flag       = (nal_reference_idc==NALU_PRIORITY_HIGHEST) ? 0:1;
    nalu->reserved_one_bit   = 1;

  }
  else
    nalu->svc_extension_flag = 0;
#endif

  
  len = RBSPtoEBSP (nalu->buf, rbsp, rbsp_size);
  nalu->len = len;

  return len;
}

/*!
 ************************************************************************
 *  \brief
 *     write AUD NALU
 ************************************************************************
 */

int Write_AUD_NALU( VideoParameters *p_Vid )
{  
  int     RBSPlen = 0;
  int     len;
  byte    rbsp[MAXRBSPSIZE];
  NALU_t *nalu = AllocNALU( MAXNALUSIZE );

  switch( p_Vid->type )
  {
  case I_SLICE:
    p_Vid->primary_pic_type = 0;
    break;
  case P_SLICE:
    p_Vid->primary_pic_type = 1;
    break;
  case B_SLICE:
    p_Vid->primary_pic_type = 2;
    break;
  }
  RBSPlen = 1;
  rbsp[0] = (byte) (p_Vid->primary_pic_type << 5);
  rbsp[0] |= (1 << 4);

  // write RBSP into NALU
  RBSPtoNALU( rbsp, nalu, RBSPlen, NALU_TYPE_AUD, NALU_PRIORITY_DISPOSABLE, 1 );
  // write NALU into bitstream
  len     = p_Vid->WriteNALU( p_Vid, nalu, p_Vid->f_out );

  FreeNALU( nalu );

  return len;
}

/*!
 ************************************************************************
 *  \brief
 *     write Filler Data NALU (results to num_bytes + 4 bytes written)
 ************************************************************************
 */

int Write_Filler_Data_NALU( VideoParameters *p_Vid, int num_bytes )
{  
  int     RBSPlen = num_bytes - 1;
  int     len, bytes_written = 0;
  byte    rbsp[MAXRBSPSIZE];
  byte    filler_byte = (byte)0xFF;
  byte    trailing_byte = (byte)0x80;
  NALU_t *nalu = AllocNALU( MAXNALUSIZE );

  num_bytes = iClip3( 1, (MAXRBSPSIZE - 2), num_bytes );
  assert( num_bytes > 0 && num_bytes < (MAXRBSPSIZE - 1) );

  num_bytes = imax( 2, num_bytes ); // one byte for the NAL header and one for the rbsp trailing byte

  if ( RBSPlen > 1 )
  {
    while ( bytes_written < (RBSPlen - 1) )
    {
      rbsp[ bytes_written++ ] = filler_byte;
    }
  }
  rbsp[ bytes_written++ ] = trailing_byte; // rbsp_trailing_bits    
  assert( num_bytes == (bytes_written + 1) );  

  // write RBSP into NALU
  RBSPtoNALU( rbsp, nalu, RBSPlen, NALU_TYPE_FILL, NALU_PRIORITY_DISPOSABLE, 1 );
  // write NALU into bitstream
  len = p_Vid->WriteNALU( p_Vid, nalu, p_Vid->f_out );
  p_Vid->bytes_in_picture += (nalu->len + 1);

  FreeNALU( nalu );

  return len;
}
