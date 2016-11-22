
/*!
 ***************************************************************************
 *
 * \file intrarefresh.h
 *
 * \brief
 *    Pseudo-Raqndom Intra macroblock refresh support
 *
 * \date
 *    16 June 2002
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 **************************************************************************/

#ifndef _INTRAREFRESH_H_
#define _INTRAREFRESH_H_

extern void RandomIntraInit(VideoParameters *p_Vid, int xsize, int ysize, int refresh);
extern void RandomIntraUninit(VideoParameters *p_Vid);
extern int  RandomIntra (VideoParameters *p_Vid, int mb);   //! returns 1 for MBs that need forced Intra
extern void RandomIntraNewPicture (VideoParameters *p_Vid);  //! to be called once per picture


#endif //_INTRAREFRESH_H_
