
/*!
 **************************************************************************************
 * \file
 *    output.h
 * \brief
 *    Picture writing routine headers
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Karsten Suehring
 ***************************************************************************************
 */

#ifndef _OUTPUT_H_
#define _OUTPUT_H_

extern void flush_direct_output(VideoParameters *p_Vid, FrameFormat *output, int p_out);
extern void write_out_picture  ( StorablePicture *p, FrameFormat *output, int p_out);
extern void write_stored_frame (VideoParameters *p_Vid, FrameStore *fs, FrameFormat *output, int p_out);
extern void direct_output      (VideoParameters *p_Vid, StorablePicture *p, FrameFormat *output, int p_out);
extern void direct_output_paff (VideoParameters *p_Vid, StorablePicture *p, FrameFormat *output, int p_out);
extern void init_out_buffer    (VideoParameters *p_Vid);
extern void uninit_out_buffer  (VideoParameters *p_Vid);


#endif //_OUTPUT_H_
