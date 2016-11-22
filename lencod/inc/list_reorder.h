/*!
 ***************************************************************************
 * \file
 *    list_reorder.h
 *
 * \date
 *    25 Feb 2009
 *
 * \brief
 *    Headerfile for slice-related functions
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Athanasios Leontaris            <aleon@dolby.com>
 *     - Karsten Suehring
 *     - Alexis Michael Tourapis         <alexismt@ieee.org> 
 **************************************************************************
 */

#ifndef _LIST_REORDER_H_
#define _LIST_REORDER_H_

#include "global.h"
#include "mbuffer.h"

extern void init_ref_pic_list_reordering( Slice *currSlice, int refReorderMethod, int useDistortionReordering);
extern void reorder_lists               ( Slice *currSlice );
extern void wp_mcprec_reorder_lists     ( Slice *currSlice );


extern void poc_ref_pic_reorder_frame_default( Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no );
extern void poc_ref_pic_reorder_field        ( Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no );
extern void poc_ref_pic_reorder_field_enh    ( Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no );

extern void tlyr_ref_pic_reorder_frame_default   ( Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no );
extern void reorder_against_default_ref_pic_lists( Slice *currSlice, int cur_list );
extern void poc_ref_pic_reorder_frame_enh        ( Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no );
extern void mse_ref_pic_reorder_frame            ( Slice *currSlice, unsigned num_ref_idx_lX_active, int list_no );
extern void set_default_ref_pic_lists( Slice *currSlice );

#endif
