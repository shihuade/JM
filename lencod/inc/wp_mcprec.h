/*!
 ***************************************************************************
 * \file
 *    wp_mcprec.h
 *
 * \brief
 *    Headerfile for Improved Motion Compensatation Precision Scheme using Weighted Prediction
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Athanasios Leontaris            <aleon@dolby.com>
 *
 * \date
 *    16 July 2008
 *
 **************************************************************************
 */

#ifndef _WP_MCPREC_H_
#define _WP_MCPREC_H_

typedef struct
{
  int PicNum;              // PicNum/FrameNum
  int POCNum;              // POC
}
WeightedPredRefX;

typedef struct
{
  int algorithm;
}
WPXPass;

typedef struct wpx_object
{
  int               num_wp_ref_list[2];         // num of elements in each of the above matrices [LIST]
  WeightedPredRefX *wp_ref_list[2];             // structure with reordering and WP information for ref frames [LIST]
  WPXPass          *curr_wp_rd_pass;
  WPXPass           wp_rd_passes[3];            // frame_picture [0...4] (MultiRefWeightedPred == 2)
}
WPXObject;

extern void   wpxInitWPXObject( VideoParameters *p_Vid );
extern void   wpxFreeWPXObject( VideoParameters *p_Vid );
extern void   wpxInitWPXPasses( VideoParameters *p_Vid, InputParameters *p_Inp );
extern void   wpxModifyRefPicList( Slice *currSlice );
// Note that at some point, InputParameters p_Inp contents should be copied into VideoParameters *p_Vid. 
// This would eliminate need of having to use both structures
extern int    wpxDetermineWP( Slice *currSlice, int clist, int n );
extern void   wpxAdaptRefNum( Slice *currSlice );

#endif
