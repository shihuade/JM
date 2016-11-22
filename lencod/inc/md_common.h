/*!
 ***************************************************************************
 * \file
 *    md_common.h
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Alexis Michael Tourapis         <alexismt@ieee.org>
 *
 * \date
 *    4. October 2008
 *
 * \brief
 *    Headerfile for common mode functions
 **************************************************************************
 */

#ifndef _MD_COMMON_H_
#define _MD_COMMON_H_
#include "mv_search.h"
#include "me_distortion.h"

extern void SetMotionVectorsMBPSlice (Macroblock* currMB);
extern void SetMotionVectorsMBBSlice (Macroblock* currMB);
extern void SetMotionVectorsMBISlice (Macroblock* currMB);
extern void copy_image_data    (imgpel  **imgBuf1, imgpel  **imgBuf2, int off1, int off2, int width, int height);
extern void copy_image_data_16x16 (imgpel  **imgBuf1, imgpel  **imgBuf2, int off1, int off2);
extern void copy_image_data_8x8   (imgpel  **imgBuf1, imgpel  **imgBuf2, int off1, int off2);
extern void copy_image_data_4x4   (imgpel  **imgBuf1, imgpel  **imgBuf2, int off1, int off2);
extern void ResetRD8x8Data     (VideoParameters *p_Vid, RD_8x8DATA *rd_data);
extern void set_chroma_pred_mode  (Macroblock *currMB, RD_PARAMS enc_mb, int *mb_available, char chroma_pred_mode_range[2]);
#endif

