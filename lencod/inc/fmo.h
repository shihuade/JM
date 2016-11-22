
/*!
 ***************************************************************************
 *
 * \file fmo.h
 *
 * \brief
 *    Support for Flexible Macroblock Ordering
 *
 * \date
 *    16 June 2002
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 **************************************************************************/

#ifndef _FMO_H_
#define _FMO_H_

extern int  FmoInit                       (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps, seq_parameter_set_rbsp_t * sps);
extern void FmoUninit                     (VideoParameters *p_Vid);
extern int  FmoFinit                      (seq_parameter_set_rbsp_t * sps);
extern int  FmoMB2SliceGroup              (VideoParameters *p_Vid, int mb);
extern int  FmoGetFirstMBOfSliceGroup     (VideoParameters *p_Vid, int SliceGroupID);
extern int  FmoGetFirstMacroblockInSlice  (VideoParameters *p_Vid, int SliceGroup);
extern int  FmoGetNextMBNr                (VideoParameters *p_Vid, int CurrentMbNr);
extern int  FmoGetPreviousMBNr            (VideoParameters *p_Vid, int CurrentMbNr);
extern int  FmoGetLastCodedMBOfSliceGroup (VideoParameters *p_Vid, int SliceGroupID);
extern int  FmoStartPicture               (VideoParameters *p_Vid);
extern int  FmoEndPicture                 (void);
extern int  FmoSliceGroupCompletelyCoded  (VideoParameters *p_Vid, int SliceGroupID);
extern void FmoSetLastMacroblockInSlice   (VideoParameters *p_Vid, int mb);


#endif
