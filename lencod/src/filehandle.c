
/*!
 **************************************************************************************
 * \file
 *    filehandle.c
 * \brief
 *    Start and terminate sequences
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Thomas Stockhammer            <stockhammer@ei.tum.de>
 *      - Detlev Marpe
 ***************************************************************************************
 */

#include "contributors.h"

#include "global.h"
#include "enc_statistics.h"

#include "rtp.h"
#include "annexb.h"
#include "parset.h"
#include "mbuffer.h"


/*!
 ************************************************************************
 * \brief
 *    Error handling procedure. Print error message to stderr and exit
 *    with supplied code.
 * \param text
 *    Error message
 * \param code
 *    Exit code
 ************************************************************************
 */
void error(char *text, int code)
{
  fprintf(stderr, "%s\n", text);
  flush_dpb(p_Enc->p_Vid->p_Dpb_layer[0], &p_Enc->p_Inp->output);
  flush_dpb(p_Enc->p_Vid->p_Dpb_layer[1], &p_Enc->p_Inp->output);
  exit(code);
}

/*!
 ************************************************************************
 * \brief
 *     This function generates and writes the PPS
 *
 ************************************************************************
 */
int write_PPS(VideoParameters *p_Vid, int len, int PPS_id)
{
  NALU_t *nalu;
  nalu = NULL;
  nalu = GeneratePic_parameter_set_NALU (p_Vid, PPS_id);
  len += p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
  FreeNALU (nalu);

  return len;
}

/*!
 ************************************************************************
 * \brief
 *    This function opens the output files and generates the
 *    appropriate sequence header
 ************************************************************************
 */
int start_sequence(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int i,len=0, total_pps = (p_Inp->GenerateMultiplePPS) ? 3 : 1;
  NALU_t *nalu;

  switch(p_Inp->of_mode)
  {
  case PAR_OF_ANNEXB:      
    p_Vid->WriteNALU = WriteAnnexbNALU;
    p_Vid->f_out = &p_Vid->f_annexb;
    OpenAnnexbFile (p_Inp->outfile, p_Vid->f_out);
    break;
  case PAR_OF_RTP:      
    p_Vid->WriteNALU = WriteRTPNALU;
    p_Vid->f_out = &p_Vid->f_rtp;
    OpenRTPFile (p_Inp->outfile, p_Vid->f_out);
    break;
  default:
    snprintf(errortext, ET_SIZE, "Output File Mode %d not supported", p_Inp->of_mode);
    error(errortext,1);
  }

  // Access Unit Delimiter NALU
  if ( p_Inp->SendAUD )
  {
    len += Write_AUD_NALU(p_Vid);
  }

  //! As a sequence header, here we write both sequence and picture
  //! parameter sets.  As soon as IDR is implemented, this should go to the
  //! IDR part, as both parsets have to be transmitted as part of an IDR.
  //! An alternative may be to consider this function the IDR start function.

  nalu = NULL;
  nalu = GenerateSeq_parameter_set_NALU (p_Vid);
  len += p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
  FreeNALU (nalu);

#if (MVC_EXTENSION_ENABLE)
  if(p_Vid->num_of_layers==2)
  {
    int bits;
    nalu = NULL;
    nalu = GenerateSubsetSeq_parameter_set_NALU (p_Vid);
    bits = p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
    len += bits;
    p_Vid->p_Stats->bit_ctr_parametersets_n_v[1] = bits;
    FreeNALU (nalu);
  }
  else
  {
    p_Vid->p_Stats->bit_ctr_parametersets_n_v[1] = 0;
  }
#endif

  //! Lets write now the Picture Parameter sets. Output will be equal to the total number of bits spend here.
  for (i=0;i<total_pps;i++)
  {
     len = write_PPS(p_Vid, len, i);
  }

  if (p_Inp->GenerateSEIMessage)
  {
    nalu = NULL;
    nalu = GenerateSEImessage_NALU(p_Inp);
    len += p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
    FreeNALU (nalu);
  }

  p_Vid->p_Stats->bit_ctr_parametersets_n = len;
#if (MVC_EXTENSION_ENABLE)
  if(p_Inp->num_of_views==2)
  {
    p_Vid->p_Stats->bit_ctr_parametersets_n_v[0] = len - p_Vid->p_Stats->bit_ctr_parametersets_n_v[1];
  }
#endif
  return 0;
}

int end_of_stream(VideoParameters *p_Vid)
{
  int bits;
  NALU_t *nalu;

  nalu = AllocNALU(MAXNALUSIZE);
  nalu->startcodeprefix_len = 4;
  nalu->forbidden_bit       = 0;  
  nalu->nal_reference_idc   = 0;
  nalu->nal_unit_type       = NALU_TYPE_EOSTREAM;
  nalu->len = 0;
  bits = p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
  
  p_Vid->p_Stats->bit_ctr_parametersets += bits;
  FreeNALU (nalu);
  return bits;
}

/*!
 ************************************************************************
 * \brief
 *    This function opens the output files and generates the
 *    appropriate sequence header
 ************************************************************************
 */
int rewrite_paramsets(VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  int i,len=0, total_pps = (p_Inp->GenerateMultiplePPS) ? 3 : 1;
  NALU_t *nalu;

  // Access Unit Delimiter NALU
  if ( p_Inp->SendAUD )
  {
    len += Write_AUD_NALU(p_Vid);
  }

  //! As a sequence header, here we write both sequence and picture
  //! parameter sets.  As soon as IDR is implemented, this should go to the
  //! IDR part, as both parsets have to be transmitted as part of an IDR.
  //! An alternative may be to consider this function the IDR start function.

  nalu = NULL;
  nalu = GenerateSeq_parameter_set_NALU (p_Vid);
  len += p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
  FreeNALU (nalu);

#if (MVC_EXTENSION_ENABLE)
  if(p_Vid->num_of_layers==2)
  {
    int bits;
    nalu = GenerateSubsetSeq_parameter_set_NALU (p_Vid);
    bits = p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
    len += bits;
    p_Vid->p_Stats->bit_ctr_parametersets_n_v[1] = bits;
    FreeNALU (nalu);
  }
#endif

  //! Lets write now the Picture Parameter sets. Output will be equal to the total number of bits spend here.
  for (i=0;i<total_pps;i++)
  {
    len = write_PPS(p_Vid, len, i);
  }

  if (p_Inp->GenerateSEIMessage)
  {
    nalu = NULL;
    nalu = GenerateSEImessage_NALU(p_Inp);
    len += p_Vid->WriteNALU (p_Vid, nalu, p_Vid->f_out);
    FreeNALU (nalu);
  }

  p_Vid->p_Stats->bit_ctr_parametersets_n = len;
#if (MVC_EXTENSION_ENABLE)
  if(p_Inp->num_of_views==2)
  {
    p_Vid->p_Stats->bit_ctr_parametersets_n_v[0] = len - p_Vid->p_Stats->bit_ctr_parametersets_n_v[1];
  }
#endif
  return 0;
}

/*!
 ************************************************************************
 * \brief
 *     This function terminates the sequence and closes the
 *     output files
 ************************************************************************
 */
int terminate_sequence(VideoParameters *p_Vid, InputParameters *p_Inp)
{
//  Bitstream *currStream;

  // Mainly flushing of everything
  // Add termination symbol, etc.

  switch(p_Inp->of_mode)
  {
  case PAR_OF_ANNEXB:
    CloseAnnexbFile(*p_Vid->f_out);
    break;
  case PAR_OF_RTP:
    CloseRTPFile(*p_Vid->f_out);
    return 0;
  default:
    snprintf(errortext, ET_SIZE, "Output File Mode %d not supported", p_Inp->of_mode);
    error(errortext,1);
  }
  return 1;   // make lint happy
}

