
/*!
 ************************************************************************
 * \file
 *     me_hme.h
 *
 * \author
 *    Alexis Michael Tourapis        <alexis.tourapis@gmail.com>
 *
 * \date
 *    05 December 2013
 *
 * \brief
 *    Header file for Hierarchical Motion Estimation (for pre-analysis) 
 **************************************************************************
 */


#ifndef _ME_HME_H_
#define _ME_HME_H_
#include "defines.h"

typedef enum 
{
  MECOST_CALC_RDNORM = 0,
  MECOST_CALC_RDN_PLUS_MVN,
  MECOST_CALC_RDN_PLUS_MVN_FAST,
  MECOST_CALC_RDN_PLUS_NP,
  MECOST_CALC_RDN_PLUS_NP_FAST,
} HMEMECostCalcMethod;

typedef struct hmeBlockInfo
{
  int plist;            //Prediction list indicator  
  int ref[2];           // reference index for L0 and/or L1
  MotionVector mv[2];   // Motion vector for L0  and/or L1
  // char weights[2];   // Weight for L0 and/or L1
  // char offset[2];    // Offset for L0 and/or L1
  int dist;             // rate distortion cost for block;
  int sad;              // distortion cost for block
} HmeBlockInfo_t;

typedef struct hme_info
{
  int iImageHeight;
  int iImageWidth;
  int iPyramidLevels;
  int HMEBlockSizeIdx;  // Currently only supports value of 3 => 8x8;
  int curLevel; 
  int iMaxRefNum;       //number of reference pictures
  int iMVCostMtd;       //?

  //hme image data;
  int imgtype;
  imgpel ***p_orig_img_pointer;            //[level][y][x];
  imgpel ***p_orig_img_pointer_layers[2];  //[level][y][x];
  
  //HME
  struct storable_picture *pTmpEncPic;
  Slice* pTmpSlice;
  SearchWindow *p_HMESW;  //[ref];
  SearchWindow *p_HMESWMin;

  MotionVector *****p_hme_mv;  //[level][list][ref][py][px];
  distblk      *****p_hme_mcost;  //[level][list][ref][py][px];  //[py][px] is ok;
  distblk      *****p_hme_mdist;  //[level][list][ref][py][px], SAD only, does not include motion cost
  
  int64  ***hme_distortion;     //! [level][list][ref] overall distortion information 
  int64  ***poc;
  int64  hme_tot_time;
  int64  hme_time;
  
  //analysis result;
  int distmin;
  int distmax;
  int distmean;
  int distvar;

  int MBLevelTransformSelection;
  //stored information; //for future extension;
  int SearchMode;
  int Transform8x8Mode;
  int iMaxBlkDist;
  //end;

  // whether to perform HME refinement
  int refine_hme; 
  int weighted_bipred_idc_save;

  // ref picture reordering related memory
  int ***hme_blk_good_store;
  int ***hme_blk_weight_store, ***hme_blk_offset_store; 
  int ***hme_blk_res_DC_store;
  int ***hme_blk_seg_store;
  int hme_blk_good_cnt[2]; 
  int hme_blk_seg_cnt[2][3];
  int hme_blk_tot_cnt; 
  struct wp_reorder_type **pWPReorderType;  // level/list
  int ***ref_idx_map;   // level/list/ref
  int ***dup_ref_flag;  // level/list/ref
  int ***wp_ref_list;   // level/list/ref
  int **most_used_ref; // level/list
  int *perform_reorder_pass; // level

  int hme_ref_pic_removal_flag[2][MAX_REFERENCE_PICTURES];
  int hme_ref_pic_removal_cnt[2];

  //function;
  distblk (*pf_computeSAD8x8_hme)(StorablePicture *ref1,
                                  MEBlock *mv_block,
                                  distblk min_mcost,
                                  MotionVector *cand
                                  );
  distblk (*pf_computeSAD16x16_hme)(StorablePicture *ref1,
                                  MEBlock *mv_block,
                                  distblk min_mcost,
                                  MotionVector *cand
                                  );
  distblk (*pf_computeSADWP8x8_hme)(StorablePicture *ref1,
                              MEBlock *mv_block,
                              distblk min_mcost,
                              MotionVector *cand
                              );
  distblk (*pf_computeSADWP16x16_hme)(StorablePicture *ref1,
                                    MEBlock *mv_block,
                                    distblk min_mcost,
                                    MotionVector *cand
                                    );
  distblk (*pf_computeSAD_hme)(StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
  distblk (*pf_computeBiPredSAD_hme)(StorablePicture *ref1, 
                                    StorablePicture *ref2, 
                                    MEBlock *mv_block,
                                    distblk min_mcost,
                                    MotionVector *cand1,
                                    MotionVector *cand2);
  distblk (*pf_computeSADWP_hme)(StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
  distblk (*pf_computeBiPredSADWP_hme)(StorablePicture *ref1, 
                                      StorablePicture *ref2, 
                                      MEBlock *mv_block,
                                      distblk min_mcost,
                                      MotionVector *cand1,
                                      MotionVector *cand2);

} HMEInfo_t;

extern void invoke_HME    (VideoParameters *p_Vid, int pic_idx);
extern int  InitHMEInfo   (VideoParameters *p_Vid, InputParameters *p_Inp);
extern void FreeHMEInfo   (VideoParameters *p_Vid);
extern void SetMELambda(VideoParameters *p_Vid, int *lambda_mf);
extern void HMEInitFrame  (VideoParameters *p_Vid, HMEInfo_t *pHMEInfo);
extern void HMEResizeSearchRange(VideoParameters *p_Vid, HMEInfo_t *pHMEInfo);
extern void HMEStoreInfo  (VideoParameters *p_Vid, HMEInfo_t *pHMEInfo);
extern void HMERestoreInfo(VideoParameters *p_Vid, HMEInfo_t *pHMEInfo);
extern void HMESearch     (Slice *currSlice);
extern void HMELevelMotionSearch(Slice *currSlice, MEBlock *mv_block, int *lambda_factor);
extern void hme_get_neighbors(PixelPos *pBlkPos, int bx, int by, int iMaxBlkX);
extern void hme_get_neighbors2(PixelPos *pBlkPos, int bx, int by, int iMaxBlkX, int iMaxBlkY);
extern void reduce_ref_pic_with_hme_info(Slice *currSlice, HMEInfo_t *pHMEInfo, int *lambda_factor);

//HME;
extern void AllocHMEMemory      (imgpel ****pHmeImage, VideoParameters *p_Vid, int size_y, int size_x, int offset_y, int offset_x, int iStartLevel);
extern void FreeHMEMemory       (imgpel ****pHmeImage, VideoParameters *p_Vid, int iStartLevel, int iPadY, int iPadX);

extern void AllocateHMEMVInfo   (MotionVector ******p_hme_mv, VideoParameters *p_Vid);
extern void AllocateHMEMECost   (distblk ******p_hme_mcost, VideoParameters *p_Vid);
extern void FreeHMEMVInfo       (MotionVector ******p_hme_mv, VideoParameters *p_Vid);
extern void FreeHMEMECost       (distblk ******p_hme_mcost, VideoParameters *p_Vid);
extern void AllocateHMEMEDist   (distblk ******p_hme_mdist, VideoParameters *p_Vid);
extern void FreeHMEMEDist       (distblk ******p_hme_mdist, VideoParameters *p_Vid);

#endif

