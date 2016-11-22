
/*!
 *************************************************************************************
 * \file explicit_seq.h
 *
 * \brief
 *    Functions for explicit sequence support
 *
 * \author
 *     Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Alexis Michael Tourapis          <alexismt@ieee.org>
 *************************************************************************************
 */

#ifndef _EXPLICIT_SEQ_H_
#define _EXPLICIT_SEQ_H_

// Definition of structures used for explicit sequence representation
typedef struct
{
  int seq_number;
  int slice_type;
  int is_idr;
  int reference_idc;
  int frame_qp;
  int deblocking;
  int is_field;
} ExpFrameInfo;

typedef struct exp_seq_info
{
  int no_frames;
  ExpFrameInfo *info;
} ExpSeqInfo;

extern void ReadExplicitSeqFile    (ExpSeqInfo *seq_info, FILE *exp_file, int coding_index);
extern void OpenExplicitSeqFile    (VideoParameters *p_Vid, InputParameters *p_Inp);
extern void CloseExplicitSeqFile   (VideoParameters *p_Vid);
#endif
