
/*!
 ************************************************************************
 * \file conformance.h
 *
 * \brief
 *   Level & Profile Related definitions  
 *
 * \author
 *    Alexis Michael Tourapis         <alexismt@ieee.org>       \n
 *
 ************************************************************************
 */

#ifndef _CONFORMANCE_H_
#define _CONFORMANCE_H_

extern Boolean is_valid_level       (unsigned int profileIdc, unsigned int levelIdc);
extern int     get_level_index      (unsigned int profile_idc, unsigned int level_idc, unsigned int constrained_set3_flag);
extern void    profile_check        (InputParameters *p_Inp);
extern void    level_check          (VideoParameters *p_Vid, InputParameters *p_Inp);
extern void    update_mv_limits     (VideoParameters *p_Vid, byte is_field);
extern void    clip_mv_range        (VideoParameters *p_Vid, int search_range, MotionVector *mv, int res);
extern int     out_of_bounds_mvs    (VideoParameters *p_Vid, const MotionVector *mv);
extern void    test_clip_mvs        (VideoParameters *p_Vid, MotionVector *mv, Boolean write_mb);
extern Boolean CheckPredictionParams(Macroblock  *currMB, Block8x8Info *b8x8info, int mode);

extern unsigned int getMaxMBPS(unsigned int profile_idc, unsigned int level_idc, unsigned int constrained_set3_flag);
extern unsigned int getMinCR  (unsigned int profile_idc, unsigned int level_idc, unsigned int constrained_set3_flag);
extern unsigned int getMaxBR  (unsigned int profile_idc, unsigned int level_idc, unsigned int constrained_set3_flag);
extern unsigned int getMaxCPB (unsigned int profile_idc, unsigned int level_idc, unsigned int constrained_set3_flag);

typedef enum{
  LEVEL_1   =  0,
  LEVEL_1b  =  1,
  LEVEL_1_1 =  2,
  LEVEL_1_2 =  3,
  LEVEL_1_3 =  4,
  LEVEL_2   =  5,
  LEVEL_2_1 =  6,
  LEVEL_2_2 =  7,
  LEVEL_3   =  8,
  LEVEL_3_1 =  9,
  LEVEL_3_2 = 10,
  LEVEL_4   = 11,
  LEVEL_4_1 = 12,
  LEVEL_4_2 = 13,
  LEVEL_5   = 14,
  LEVEL_5_1 = 15,
  LEVEL_5_2 = 16,
  LEVEL_6   = 17,
  LEVEL_6_1 = 18,
  LEVEL_6_2 = 19,
  LEVEL_UNCONSTRAINED = 20
} LevelId;

#endif

