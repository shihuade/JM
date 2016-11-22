
/*!
 ***************************************************************************
 *
 * \file filehandle.h
 *
 * \brief
 *
 * \date
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *
 **************************************************************************/

#ifndef _FILEHANDLE_H_
#define _FILEHANDLE_H_

extern int  rewrite_paramsets (VideoParameters *p_Vid);
extern int  start_sequence    (VideoParameters *p_Vid, InputParameters *p_Inp);
extern int  terminate_sequence(VideoParameters *p_Vid, InputParameters *p_Inp);
extern int  write_PPS         (VideoParameters *p_Vid, int len, int PPS_id);
extern int  end_of_stream     (VideoParameters *p_Vid);

#endif
