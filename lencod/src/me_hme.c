
/*!
*************************************************************************************
* \file me_hme.c
*
* \brief
*    HME Motion Estimation using EPZS
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*      - Yuwen He
*      - Alexis Michael Tourapis 
*
*************************************************************************************
*/

#include "contributors.h"
#include <limits.h>

#include "global.h"
#include "image.h"
#include "memalloc.h"
#include "mb_access.h"
#include "refbuf.h"
#include "macroblock.h"
#include "me_distortion.h"
#include "me_epzs.h"
#include "me_epzs_common.h"
#include "mv_search.h"

#include "me_hme.h"
#include "hme_distortion.h"
#include "resize.h"

#include "wp.h"
#include "slice.h"
#include "list_reorder.h"


// private
static void SetMVFromUpperLevel (Slice *currSlice, int pyr_level);
static void HMEPicMotionSearch  (Slice *currSlice, MEBlock *mv_block, int *lambda_factor);
static distblk HMEBlockMotionSearch(MotionVector **p_pic_mv, distblk **p_pic_mcost, distblk **p_pic_mdist, MEBlock *mv_block, int *lambda_factor);
static distblk HME_EPZSIntPelBlockMotionSearch_Enh (MotionVector *pred_mv, MEBlock *mv_block, MotionVector **pic_mv, distblk **pic_mcost, int lambda_factor);

static void prepare_enc_frame_picture_hme (VideoParameters *p_Vid)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  StorablePicture *stored_pic = p_Vid->pHMEInfo->pTmpEncPic;
  
  p_Vid->ThisPOC          = p_Vid->framepoc;
  stored_pic->poc         = p_Vid->framepoc;
  stored_pic->top_poc     = p_Vid->toppoc;
  stored_pic->bottom_poc  = p_Vid->bottompoc;
  stored_pic->frame_poc   = p_Vid->framepoc;
  stored_pic->pic_num     = p_Vid->frame_num;
  stored_pic->frame_num   = p_Vid->frame_num;
  stored_pic->coded_frame = 1;
  stored_pic->mb_aff_frame_flag = p_Vid->mb_aff_frame_flag = (Boolean) (p_Inp->MbInterlace != FRAME_CODING);
  
  p_Vid->get_mb_block_pos    = p_Vid->mb_aff_frame_flag ? get_mb_block_pos_mbaff : get_mb_block_pos_normal;
  p_Vid->getNeighbour        = p_Vid->mb_aff_frame_flag ? getAffNeighbour : getNonAffNeighbour;
  p_Vid->enc_picture         = stored_pic;
  
  copy_params(p_Vid, p_Vid->enc_picture, p_Vid->active_sps);
}

void invoke_HME(VideoParameters *p_Vid, int pic_idx)
{
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int set_lists;

  HMEInitFrame(p_Vid, pHMEInfo);
  HMEStoreInfo(p_Vid, pHMEInfo);
  prepare_enc_frame_picture_hme( p_Vid);

  p_Vid->currentPicture = p_Vid->frame_pic[pic_idx];

  pHMEInfo->p_orig_img_pointer[0] = p_Vid->imgData.frm_data[0];
  GenerateImagePyramid(p_Vid, pHMEInfo->iImageWidth, pHMEInfo->iImageHeight, pHMEInfo->p_orig_img_pointer, 0, 0);
  
  // prepare HME. This code is copied from Yuwen's original implementation
  // and could use some substantial improvements.
  // The HME should not be using the existing slice code and be mixing up the processes like this
  // but instead be clean and independent. Calls to init_slice should not be allowed.

  InitWP(p_Vid, p_Inp, (p_Inp->WPMethod == 3 && pic_idx == 0) ? 1:0);
  p_Vid->is_hme = 1;
  init_slice(p_Vid, &(pHMEInfo->pTmpSlice), 0);
  p_Vid->is_hme = 0;  

  if(pHMEInfo->pTmpSlice->p_EPZS->EPZSMap)
  {
    HMEResizeSearchRange(p_Vid, pHMEInfo);
  }

  p_Vid->ThisPOC = pHMEInfo->pTmpSlice->framepoc;
  set_lists = TRUE;
  //do HME;
  HMESearch(pHMEInfo->pTmpSlice);
  //end and recover the information;
  free_slice(pHMEInfo->pTmpSlice);
  
  p_Vid->frame_pic[pic_idx]->no_slices = 0;
  HMERestoreInfo(p_Vid, pHMEInfo); 

  p_Vid->enc_picture = NULL;
}

void AllocHMEMemory(imgpel ****pHmeImage, VideoParameters *p_Vid, int size_y, int size_x, int offset_y, int offset_x, int iStartLevel)
{
   int i, iSx, iSy;
   int iPyramidLevels = p_Vid->pHMEInfo->iPyramidLevels;

   *pHmeImage = (imgpel ***)malloc(sizeof(imgpel**)* iPyramidLevels);
   for(i=0; i<iStartLevel; i++)
     (*pHmeImage)[i] = NULL;
   for(i=iStartLevel; i< iPyramidLevels; i++)
   {
     iSx = (size_x >> i); 
     iSy = (size_y >> i);
     get_mem2Dpel_pad(&((*pHmeImage)[i]), iSy, iSx, offset_y, offset_x);
   }   
}

void FreeHMEMemory(imgpel ****pHmeImage, VideoParameters *p_Vid, int iStartLevel, int iPadY, int iPadX)
{
   if(pHmeImage && (*pHmeImage))
   {
      int i;
      int iPyramidLevels = p_Vid->pHMEInfo->iPyramidLevels;
      for(i=0; i<iStartLevel; i++)
        (*pHmeImage)[i] = NULL;
      for(i=iStartLevel; i<iPyramidLevels; i++)
      {
        free_mem2Dpel_pad((*pHmeImage)[i], iPadY, iPadX);
       (*pHmeImage)[i]=NULL;
      }
      free((*pHmeImage));
      (*pHmeImage) = NULL;
   }
}

void AllocateHMEMVInfo(MotionVector ******p_hme_mv, VideoParameters *p_Vid)
{
  int iListNum=2;
  int i, iMaxRefNum, iSx, iSy;
  int iPyramidLevels = p_Vid->pHMEInfo->iPyramidLevels;

  iMaxRefNum = p_Vid->pHMEInfo->iMaxRefNum; //p_Inp->num_ref_frames;
  *p_hme_mv = (MotionVector*****) malloc(iPyramidLevels*sizeof(MotionVector****));
  for(i=0; i<iPyramidLevels; i++)
  {
    iSx = (p_Vid->width  >>(i + 3));
    iSy = (p_Vid->height >>(i + 3));
    get_mem4Dmv(&(*p_hme_mv)[i], iListNum, iMaxRefNum, iSy, iSx); //[level][list][ref][py][px];
  }
}

void AllocateHMEMECost(distblk ******p_hme_mcost, VideoParameters *p_Vid)
{
  int iListNum=2;
  int i, iMaxRefNum, iSx, iSy;
  int iPyramidLevels = p_Vid->pHMEInfo->iPyramidLevels;

  iMaxRefNum = p_Vid->pHMEInfo->iMaxRefNum; //p_Inp->num_ref_frames;
  *p_hme_mcost = (distblk*****) malloc(iPyramidLevels*sizeof(distblk****));
  for(i=0; i<iPyramidLevels; i++)
  {
    iSx = (p_Vid->width  >> (i + 3));
    iSy = (p_Vid->height >> (i + 3));
    get_mem4Ddistblk(&(*p_hme_mcost)[i], iListNum, iMaxRefNum, iSy, iSx); //[level][list][ref][py][px];
  }
}

void AllocateHMEDistInfo(distblk ******p_hme_mdist, VideoParameters *p_Vid)
{
  int iListNum=2;
  int i, iMaxRefNum, iSx, iSy;
  int iPyramidLevels = p_Vid->pHMEInfo->iPyramidLevels;

  iMaxRefNum = p_Vid->pHMEInfo->iMaxRefNum; //p_Inp->num_ref_frames;
  *p_hme_mdist = (distblk*****) malloc(iPyramidLevels*sizeof(distblk****));
  for(i=0; i<iPyramidLevels; i++)
  {
    iSx = (p_Vid->width  >> (i + 3));
    iSy = (p_Vid->height >> (i + 3));
    get_mem4Ddistblk(&(*p_hme_mdist)[i], iListNum, iMaxRefNum, iSy, iSx); //[level][list][ref][py][px];
  }
}

void FreeHMEMVInfo(MotionVector ******p_hme_mv, VideoParameters *p_Vid)
{
  int i;

  if(p_hme_mv && *p_hme_mv)
  {
    int iPyramidLevels = p_Vid->pHMEInfo->iPyramidLevels;
    for(i=0; i<iPyramidLevels; i++)
    {
      free_mem4Dmv((*p_hme_mv)[i]);
      (*p_hme_mv)[i] = NULL;
    }
    free(*p_hme_mv);
    *p_hme_mv = NULL;
  }
}

void FreeHMEMECost(distblk ******p_hme_mcost, VideoParameters *p_Vid)
{
  int i;

  if(p_hme_mcost && *p_hme_mcost)
  {
    int iPyramidLevels = p_Vid->pHMEInfo->iPyramidLevels;
    for(i=0; i<iPyramidLevels; i++)
    {
      free_mem4Ddistblk((*p_hme_mcost)[i]);
      (*p_hme_mcost)[i] = NULL;
    }
    free(*p_hme_mcost);
    *p_hme_mcost = NULL;
  }
}

void FreeHMEMEDist(distblk ******p_hme_mdist, VideoParameters *p_Vid)
{
  int i;

  if(p_hme_mdist && *p_hme_mdist)
  {
    int iPyramidLevels = p_Vid->pHMEInfo->iPyramidLevels;
    for(i=0; i<iPyramidLevels; i++)
    {
      free_mem4Ddistblk((*p_hme_mdist)[i]); //[level][list][ref][py][px];
      (*p_hme_mdist)[i] = NULL;
    }
    free(*p_hme_mdist);
    *p_hme_mdist = NULL;
  }
}


void CalcSearchRange(VideoParameters *p_Vid, InputParameters *p_Inp, int iMaxRefNum)
{
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;
  SearchWindow *pDstSW = pHMEInfo->p_HMESW, *pSW;

  int ref;
  int iSRx = p_Inp->search_range[0]<<2;
  int iSRy = p_Inp->search_range[0]<<2;
  int iSRxMin = 16;
  int iSRyMin = 16;

  //----- set search range ---
  
  for(ref=0; ref<iMaxRefNum; ref++)
  {
    //pDstSW[ref] = p_Vid->searchRange;
    pSW = pDstSW + ref;
    pSW->min_x = -iSRx;
    pSW->max_x = iSRx;
    pSW->min_y = -iSRy;
    pSW->max_y = iSRy;
    
    pSW = pHMEInfo->p_HMESWMin+ref;
    pSW->min_x = -iSRxMin;
    pSW->max_x = iSRxMin;
    pSW->min_y = -iSRyMin;
    pSW->max_y = iSRyMin;
  }
}

int InitHMEInfo(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  HMEInfo_t *pHMEInfo = (HMEInfo_t *)calloc(1, sizeof(HMEInfo_t));
  int pic_size_x_in_blk, pic_size_y_in_blk; 

  pic_size_x_in_blk = p_Vid->width >>3;
  pic_size_y_in_blk = p_Vid->height>>3;

  if(!pHMEInfo)
  {
    p_Vid->pHMEInfo = NULL;
    return -1;
  }
  p_Vid->pHMEInfo = pHMEInfo;
  
  if(p_Inp->PyramidLevels >0)
    pHMEInfo->iPyramidLevels = p_Inp->PyramidLevels;
  else
  {
    int i = imax(1, (int)(log(imin(p_Vid->width/88, p_Vid->height/72))/log(2.0)+1));
    pHMEInfo->iPyramidLevels = i;
  }
  pHMEInfo->HMEBlockSizeIdx = 3;
  pHMEInfo->Transform8x8Mode = -1;

  pHMEInfo->iMaxRefNum = p_Inp->num_ref_frames;

  if(p_Inp->PicInterlace != FRAME_CODING)
  {
    pHMEInfo->iMaxRefNum <<= 1;
  }

  //allocate the original images;
  AllocHMEMemory(&(pHMEInfo->p_orig_img_pointer_layers[0]), p_Vid, p_Vid->height, p_Vid->width, 0, 0, 1);
  pHMEInfo->p_orig_img_pointer_layers[1] = NULL;

  pHMEInfo->p_orig_img_pointer = pHMEInfo->p_orig_img_pointer_layers[0];
  pHMEInfo->p_orig_img_pointer[0] = NULL;
  AllocateHMEMVInfo(&pHMEInfo->p_hme_mv, p_Vid);
  AllocateHMEMECost(&pHMEInfo->p_hme_mcost, p_Vid);  
  AllocateHMEDistInfo(&pHMEInfo->p_hme_mdist, p_Vid);
  get_mem3Dint64(&pHMEInfo->hme_distortion, pHMEInfo->iPyramidLevels, 3, pHMEInfo->iMaxRefNum);
  get_mem3Dint64(&pHMEInfo->poc, pHMEInfo->iPyramidLevels, 3, pHMEInfo->iMaxRefNum);
 
  pHMEInfo->p_HMESW = (SearchWindow *)malloc( pHMEInfo->iMaxRefNum * sizeof(SearchWindow));
  if(!pHMEInfo->p_HMESW)
    return -1;
  pHMEInfo->p_HMESWMin = (SearchWindow *)malloc( pHMEInfo->iMaxRefNum * sizeof(SearchWindow));
  if(!pHMEInfo->p_HMESWMin)
    return -1;
  CalcSearchRange(p_Vid, p_Inp, pHMEInfo->iMaxRefNum);

  {
    int nal_reference_idc = p_Vid->nal_reference_idc;
    p_Vid->nal_reference_idc = NALU_PRIORITY_DISPOSABLE;  
    pHMEInfo->pTmpEncPic = alloc_storable_picture(p_Vid, (PictureStructure) FRAME, 16, 16, 8, 8);
    p_Vid->nal_reference_idc = nal_reference_idc;
  }

  get_mem3Dint(&pHMEInfo->hme_blk_good_store, 2, pic_size_y_in_blk, pic_size_x_in_blk);

  {
    pHMEInfo->hme_blk_weight_store = NULL;
    pHMEInfo->hme_blk_offset_store = NULL;
    pHMEInfo->hme_blk_res_DC_store = NULL;
    pHMEInfo->hme_blk_seg_store = NULL; 
    pHMEInfo->pWPReorderType = NULL;
    pHMEInfo->ref_idx_map = NULL;
    pHMEInfo->dup_ref_flag = NULL;
    pHMEInfo->wp_ref_list = NULL;
    pHMEInfo->most_used_ref = NULL;
    pHMEInfo->perform_reorder_pass = NULL;
  }

  return 0; 
}

void FreeHMEInfo(VideoParameters *p_Vid)
{
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;
  if(pHMEInfo)
  {
    if(p_Vid->p_Inp->num_of_views==2)
    {
      if(pHMEInfo->p_orig_img_pointer_layers[0] != pHMEInfo->p_orig_img_pointer_layers[1])
      {
        FreeHMEMemory(&(pHMEInfo->p_orig_img_pointer_layers[1]), p_Vid, 1, 0, 0);
      }
      else
        pHMEInfo->p_orig_img_pointer_layers[1] = NULL;
    }
    
    FreeHMEMemory(&(pHMEInfo->p_orig_img_pointer_layers[0]), p_Vid, 1, 0, 0);
    pHMEInfo->p_orig_img_pointer = NULL;
    FreeHMEMVInfo(&pHMEInfo->p_hme_mv, p_Vid);
    FreeHMEMECost(&pHMEInfo->p_hme_mcost, p_Vid);
    FreeHMEMEDist(&pHMEInfo->p_hme_mdist, p_Vid);
    if(pHMEInfo->p_HMESW)
    {
      free(pHMEInfo->p_HMESW);
      pHMEInfo->p_HMESW = NULL;
    }
    if(pHMEInfo->p_HMESWMin)
    {
      free(pHMEInfo->p_HMESWMin);
      pHMEInfo->p_HMESWMin = NULL;
    }
    if(pHMEInfo->pTmpEncPic)
    {
      free_storable_picture(p_Vid, pHMEInfo->pTmpEncPic);
      pHMEInfo->pTmpEncPic = NULL;
    }
    
    if (pHMEInfo->hme_distortion)
      free_mem3Dint64(pHMEInfo->hme_distortion);
    if (pHMEInfo->hme_distortion)
      free_mem3Dint64(pHMEInfo->poc);
    if(pHMEInfo->hme_blk_weight_store)
      free_mem3Dint(pHMEInfo->hme_blk_weight_store);
    if(pHMEInfo->hme_blk_offset_store)
      free_mem3Dint(pHMEInfo->hme_blk_offset_store);
    if(pHMEInfo->hme_blk_res_DC_store)
      free_mem3Dint(pHMEInfo->hme_blk_res_DC_store);
    if(pHMEInfo->hme_blk_good_store)
      free_mem3Dint(pHMEInfo->hme_blk_good_store);
    if(pHMEInfo->hme_blk_seg_store)
      free_mem3Dint(pHMEInfo->hme_blk_seg_store);
    if(pHMEInfo->pWPReorderType)
    {
      if(*pHMEInfo->pWPReorderType)
        free((void *)(*pHMEInfo->pWPReorderType));
      else
        error ("FreeHMEInfo: trying to free unused memory *pHMEInfo->pWPReorderType",100);

      free(pHMEInfo->pWPReorderType);
    }
    if(pHMEInfo->ref_idx_map)
      free_mem3Dint(pHMEInfo->ref_idx_map);
    if(pHMEInfo->dup_ref_flag)
      free_mem3Dint(pHMEInfo->dup_ref_flag);
    if(pHMEInfo->wp_ref_list)
      free_mem3Dint(pHMEInfo->wp_ref_list);
    if(pHMEInfo->most_used_ref)
      free_mem2Dint(pHMEInfo->most_used_ref);
    if(pHMEInfo->perform_reorder_pass)
      free(pHMEInfo->perform_reorder_pass);

    free(pHMEInfo);

    p_Vid->pHMEInfo = NULL;
  }
}

void HMEInitFrame(VideoParameters *p_Vid, HMEInfo_t *pHMEInfo)
{
  pHMEInfo->hme_time = 0;
  pHMEInfo->iImageHeight = p_Vid->field_picture? (p_Vid->height>>1):p_Vid->height;
  pHMEInfo->iImageWidth  = p_Vid->width;
  pHMEInfo->p_orig_img_pointer = pHMEInfo->p_orig_img_pointer_layers[p_Vid->dpb_layer_id];

  //set functions;
  pHMEInfo->pf_computeSAD_hme = computeSAD_hme;
}

void HMEResizeSearchRange(VideoParameters *p_Vid, HMEInfo_t *pHMEInfo)
{
  uint16 **EPZSMap = pHMEInfo->pTmpSlice->p_EPZS->EPZSMap;
  InputParameters *p_Inp = p_Vid->p_Inp;
  int iSearchRangeY = (2 * p_Inp->search_range[0] + 1) << 2;
  int iSearchRangeX = (2 * p_Inp->search_range[0] + 1) << 2;

  if(EPZSMap)
  {
     free_mem2Dshort ((short **) EPZSMap);
  }
  get_mem2Dshort ((short ***) &EPZSMap, iSearchRangeY, iSearchRangeX);
	pHMEInfo->pTmpSlice->p_EPZS->EPZSMap = EPZSMap;
	
}

void HMEStoreInfo(VideoParameters *p_Vid, HMEInfo_t *pHMEInfo)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  pHMEInfo->SearchMode = p_Inp->SearchMode[p_Vid->view_id];
}

void HMERestoreInfo(VideoParameters *p_Vid, HMEInfo_t *pHMEInfo)
{
  InputParameters *p_Inp = p_Vid->p_Inp;
  p_Inp->SearchMode[p_Vid->view_id] = (SearchType) pHMEInfo->SearchMode;
}

void SetMELambda(VideoParameters *p_Vid, int *lambda_mf)
{
  //===== SET LAGRANGE PARAMETERS =====
  // Note that these are now computed at the slice level to reduce
  // computations and cleanup code.
  int slice_type = p_Vid->type;
  int iHMEQP = p_Vid->masterQP;
  if((p_Vid->type == B_SLICE) && p_Vid->nal_reference_idc)
  {
    slice_type = 5;
  }

  lambda_mf[F_PEL] = p_Vid->lambda_mf[slice_type][iHMEQP][F_PEL];
  lambda_mf[H_PEL] = p_Vid->lambda_mf[slice_type][iHMEQP][H_PEL];
  lambda_mf[Q_PEL] = p_Vid->lambda_mf[slice_type][iHMEQP][Q_PEL];
}

void hme_init_mv_block(VideoParameters *p_Vid, MEBlock *mv_block, short blocktype)
{
  Slice *currSlice = p_Vid->currentSlice;
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;

  mv_block->blocktype         = blocktype;
  mv_block->blocksize_x       = block_size[blocktype][0];  // horizontal block size
  mv_block->blocksize_y       = block_size[blocktype][1];  // vertical block size
  mv_block->block_x = 0;
  mv_block->block_y = 0;

  mv_block->mv[LIST_0].mv_x   = 0;
  mv_block->mv[LIST_0].mv_y   = 0;
  mv_block->mv[LIST_1].mv_x   = 0;
  mv_block->mv[LIST_1].mv_y   = 0;
  // Init WP parameters
  mv_block->p_Vid             = p_Vid;
  mv_block->p_Slice           = currSlice;
  mv_block->cost              = INT_MAX;
  mv_block->search_pos2       = 9;
  mv_block->search_pos4       = 9;

  get_mem2Dpel(&mv_block->orig_pic, 1, mv_block->blocksize_x * mv_block->blocksize_y);

  mv_block->ChromaMEEnable = 0; //p_Inp->ChromaMEEnable;

  mv_block->apply_bi_weights = 0;
  mv_block->apply_weights     = 0;

  mv_block->computePredFPel   = pHMEInfo->pf_computeSAD_hme; //computeSAD_hme : computeSAD_hme_16b; //p_Vid->computeUniPred[F_PEL];
  mv_block->computeBiPredFPel = NULL;
}

//do HME over all levels;
void HMESearch(Slice *currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  //InputParameters *p_Inp = p_Vid->p_Inp;

  MEBlock  mv_block;

  int lambda_factor[3];
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;
  int blocktype = SMB8x8;

#if GET_METIME
  TIME_T me_time_start;
  TIME_T me_time_end;
  int64 me_tmp_time;
  gettime( &me_time_start );    // start time ms
#endif

  currSlice->set_lagrangian_multipliers(currSlice);
  SetMELambda(p_Vid, lambda_factor);  

  hme_init_mv_block(p_Vid, &mv_block, (short) blocktype);

  // Motion estimation for current picture 
  HMEPicMotionSearch (currSlice, &mv_block, lambda_factor);

  free_mv_block(&mv_block);

#if GET_METIME
  gettime(&me_time_end);   // end time ms
  me_tmp_time = timediff (&me_time_start, &me_time_end);
  pHMEInfo->hme_tot_time += me_tmp_time;
  pHMEInfo->hme_time += me_tmp_time;
#endif

}

static void SetMVFromUpperLevel(Slice *currSlice, int pyr_level)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;
  MotionVector *****p_hme_mv = pHMEInfo->p_hme_mv; //[level][list][ref][py][px];
  int i, j;
  int list;// = mv_block->list;
  int ref; //= mv_block->ref_idx;
  int blkwidth  = (pHMEInfo->iImageWidth  >> (pyr_level + 3));
  int blkheight = (pHMEInfo->iImageHeight >> (pyr_level + 3));
  int numlists = (currSlice->slice_type == B_SLICE)? 2: 1;

  if(pyr_level < pHMEInfo->iPyramidLevels-1)
  {
    for (list=0; list<numlists; list++)
      for (ref=0; ref<currSlice->listXsize[list]; ref++)
      {
        MotionVector **p_mv0 = p_hme_mv[pyr_level][list][ref];
        MotionVector **p_mv1 = p_hme_mv[pyr_level+1][list][ref];
        MotionVector mv_scaled;
        int ii, jj;
        for(j=0; j<(blkheight>>1); j++)
        { 
          jj = j<<1;
          for(i=0; i<(blkwidth>>1); i++)
          {
            ii = i<<1;
            mv_scaled.mv_x = p_mv1[j][i].mv_x <<1;
            mv_scaled.mv_y = p_mv1[j][i].mv_y <<1;
            p_mv0[jj][ii] = 
            p_mv0[jj][ii+1] = 
            p_mv0[jj+1][ii] = 
            p_mv0[jj+1][ii+1] = mv_scaled;
          }
          if(blkwidth&1)
          {
            p_mv0[jj][blkwidth-1] = p_mv0[jj][blkwidth-2];
            p_mv0[jj+1][blkwidth-1] = p_mv0[jj+1][blkwidth-2];
          }
        }
        if(blkheight&1)
        {
          memcpy(p_mv0[blkheight-1], p_mv0[blkheight-2], blkwidth*sizeof(MotionVector));
        }
      }
  }
  else
  {
    for(list=0; list<numlists; list++)
       memset(*p_hme_mv[pyr_level][list][0], 0, currSlice->listXsize[list]*blkwidth*blkheight*sizeof(MotionVector));
  }
}

void hme_get_original_block(HMEInfo_t *pHMEInfo, int level, MEBlock *mv_block)
{
  //==================================
  //=====   GET ORIGINAL BLOCK   =====
  //==================================
  imgpel *orig_pic_tmp = mv_block->orig_pic[0];
  int   bsx       = mv_block->blocksize_x;
  int   pic_pix_x = mv_block->pos_x;
  int   j;
  imgpel **cur_img = &pHMEInfo->p_orig_img_pointer[level][mv_block->pos_y];
  int   bytes = sizeof(imgpel);
  int   imgpels = bytes/sizeof(imgpel);

  for (j = 0; j < mv_block->blocksize_y; j++)
  {
    memcpy(orig_pic_tmp,&cur_img[j][pic_pix_x*imgpels], bsx * bytes);
    orig_pic_tmp += bsx*imgpels;
  }
}

void HMEGetPMV(VideoParameters *p_Vid, MotionVector **p_pic_mv, int level, int bx, int by, MotionVector *pred)
{
  int img_bx = (p_Vid->width>>level)>>3;

  MotionVector mv[3];
  if(!by)
  {
    if(!bx)  //top left;
      *pred = p_pic_mv[by][bx];  //use upper layer MV
    else  //top row;  
      *pred = p_pic_mv[by][bx-1];
  }
  else
  {
   if(bx)
     mv[0] = p_pic_mv[by][bx-1];
   else
   {
     mv[0] = p_pic_mv[by][bx]; //use upper layer MV
   }
   //
   mv[1] = p_pic_mv[by-1][bx];
   if(bx < img_bx-1)
     mv[2] = p_pic_mv[by-1][bx+1];
   else
     mv[2] = p_pic_mv[by-1][bx-1];

   //perform median filtering
   pred->mv_x = mv[0].mv_x + mv[1].mv_x + mv[2].mv_x - smin(mv[0].mv_x, smin(mv[1].mv_x, mv[2].mv_x)) - smax(mv[0].mv_x, smax(mv[1].mv_x, mv[2].mv_x));
   pred->mv_y = mv[0].mv_y + mv[1].mv_y + mv[2].mv_y - smin(mv[0].mv_y, smin(mv[1].mv_y, mv[2].mv_y)) - smax(mv[0].mv_y, smax(mv[1].mv_y, mv[2].mv_y));
  }
}

void HMESetSearchRange(SearchWindow *pSrcSW,   //the search window at layer 0;
                       int level,              //current pyramid level;
                       SearchWindow *pCurrSW,   //current search window;
                       SearchWindow *pSWMin     //search window limitation;
                       )
{
  int tmp;
  tmp = pSrcSW->max_x >> level;
  pCurrSW->max_x = tmp < pSWMin->max_x? pSWMin->max_x: tmp;
  tmp = pSrcSW->min_x >> level;
  pCurrSW->min_x = tmp > pSWMin->min_x? pSWMin->min_x: tmp;
  tmp = pSrcSW->max_y >> level;
  pCurrSW->max_y = tmp < pSWMin->max_y? pSWMin->max_y: tmp;  
  tmp = pSrcSW->min_y >> level;
  pCurrSW->min_y = tmp > pSWMin->min_y? pSWMin->min_y: tmp;
}


void hme_get_neighbors(PixelPos *pBlkPos, int bx, int by, int iMaxBlkX)
{
   if(!by)
   {
     pBlkPos[1].available = pBlkPos[2].available = pBlkPos[3].available = 0;
     if(!bx)
       pBlkPos[0].available = 0;
     else
     {
       pBlkPos[0].available = 1;
       pBlkPos[0].pos_x = pBlkPos[0].x = (short) (bx - 1);
       pBlkPos[0].pos_y = pBlkPos[0].y = (short) by;
     }
   }
   else
   {
     pBlkPos[1].available = 1;
     pBlkPos[1].pos_x = pBlkPos[1].x = (short) bx;
     pBlkPos[1].pos_y = pBlkPos[1].y = (short) (by - 1);
     pBlkPos[0].available = pBlkPos[2].available = pBlkPos[3].available = 1;
     if(!bx)
     {
       pBlkPos[0].available = pBlkPos[3].available = 0;
     }
     else if(bx >=iMaxBlkX-1)
     {
       pBlkPos[2].available = 0;
     }

     if(pBlkPos[0].available)
     {
       pBlkPos[0].pos_x = pBlkPos[0].x = (short) (bx - 1);
       pBlkPos[0].pos_y = pBlkPos[0].y = (short) by;
     }
     if(pBlkPos[2].available)
     {
       pBlkPos[2].pos_x = pBlkPos[2].x = (short) (bx + 1);
       pBlkPos[2].pos_y = pBlkPos[2].y = (short) (by - 1);
     }
     if(pBlkPos[3].available)
     {
       pBlkPos[3].pos_x = pBlkPos[3].x = (short) (bx - 1);
       pBlkPos[3].pos_y = pBlkPos[3].y = (short) (by - 1);
     }
   }
}


void hme_get_neighbors2(PixelPos *pBlkPos, int bx, int by, int iMaxBlkX, int iMaxBlkY)
{
   if(!bx)
   {
     pBlkPos[0].available = 0;
     pBlkPos[2].available = 1;
     pBlkPos[2].pos_x = pBlkPos[2].x = (short) (bx + 1);
     pBlkPos[2].pos_y = pBlkPos[2].y = (short) by;
   }
   else if(bx == iMaxBlkX-1)
   {
     pBlkPos[2].available = 0;
     pBlkPos[0].available = 1;
     pBlkPos[0].pos_x = pBlkPos[0].x = (short) (bx - 1);
     pBlkPos[0].pos_y = pBlkPos[0].y = (short) by;
   }
   else
   {
     pBlkPos[0].available = 1;
     pBlkPos[0].pos_x = pBlkPos[0].x = (short) (bx - 1);
     pBlkPos[0].pos_y = pBlkPos[0].y = (short) by;
     pBlkPos[2].available = 1;
     pBlkPos[2].pos_x = pBlkPos[2].x = (short) (bx + 1);
     pBlkPos[2].pos_y = pBlkPos[2].y = (short) by;
   }

   if(!by)
   {
     pBlkPos[1].available = 0;
     pBlkPos[3].available = 1;
     pBlkPos[3].pos_x = pBlkPos[3].x = (short) bx;
     pBlkPos[3].pos_y = pBlkPos[3].y = (short) (by+1);
   }
   else if(by == iMaxBlkY-1)
   {
     pBlkPos[3].available = 0;
     pBlkPos[1].available = 1;
     pBlkPos[1].pos_x = pBlkPos[1].x = (short) bx;
     pBlkPos[1].pos_y = pBlkPos[1].y = (short) (by-1);
   }
   else
   {
     pBlkPos[1].available = 1;
     pBlkPos[1].pos_x = pBlkPos[1].x = (short) bx;
     pBlkPos[1].pos_y = pBlkPos[1].y = (short) (by-1);
     pBlkPos[3].available = 1;
     pBlkPos[3].pos_x = pBlkPos[3].x = (short) bx;
     pBlkPos[3].pos_y = pBlkPos[3].y = (short) (by+1);
   }
}

void hme_SetSearchRange(MEBlock *mv_block, MotionVector *mv)
{
    MotionVector center = pad_MVs (*mv, mv_block);
    SearchWindow *pSW = &mv_block->searchRange;
    int SW_x = (pSW->max_x - pSW->min_x);
    int SW_y = (pSW->max_y - pSW->min_y);
    int bChanged=0;
    int b0 = center.mv_x+pSW->min_x, b1;

    int leftBoundary = -((IMG_PAD_SIZE_X-4)*4);
    int rightBoundary = (mv_block->hme_ref_size_x_max-4)*4;
    int topBoundary = -((IMG_PAD_SIZE_Y-4)*4);
    int bottomBoundary = (mv_block->hme_ref_size_y_max-4)*4;

    //return;

    if(b0 < leftBoundary)
    {
      b0 = leftBoundary; 
      bChanged=1;
    }
    else if(b0+SW_x > rightBoundary)
    {
      b0 = rightBoundary - (SW_x);
      bChanged=1;
    }
    if(bChanged)
    {
      b1 = b0+(SW_x); 
      if(b1 > rightBoundary)
      {
        b1 = rightBoundary;
        SW_x = (b1 - b0)>>1;
        pSW->min_x = -SW_x;
        pSW->max_x = SW_x;
      }
      mv->mv_x = (short) (((b0 + b1)>>1) - mv_block->pos_x_padded);
    }

    bChanged = 0;
    b0 =  center.mv_y+pSW->min_y;
    if(b0 < topBoundary)
    {
      b0 = topBoundary;
      bChanged = 1;
    }
    else if(b0 + SW_y > bottomBoundary)
    {
      b0 = bottomBoundary-SW_y;
      bChanged = 1;
    }
    if(bChanged)
    {
      b1 = b0+SW_y;
      if(b1 > bottomBoundary)
      {
        b1 = bottomBoundary;
        SW_y = (b1 - b0)>>1;
        pSW->min_y = -SW_y;
        pSW->max_y = SW_y;
      }
      mv->mv_y = (short) (((b0 + b1)>>1) - mv_block->pos_y_padded);
    }
}

void HMELevelMotionSearch(Slice *currSlice, MEBlock *mv_block, int *lambda_factor)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;
  distblk **p_pic_mcost; 
  MotionVector **p_pic_mv;
  distblk **p_pic_mdist; 
  SearchWindow *currPicSW = pHMEInfo->p_HMESW;
  SearchWindow *currPicSWMin = pHMEInfo->p_HMESWMin;

  int pyr_level = mv_block->hme_level;

  int by, bx;
  int pic_size_x, pic_size_y;
  int list, ref; 
  int numlists  = (currSlice->slice_type == B_SLICE) ? 2 : 1;

  for (list = 0; list < numlists; list++)
  {
     for (ref = 0; ref < currSlice->listXsize[list]; ref++) 
     {
        pHMEInfo->hme_distortion[pyr_level][list][ref] = 0;
        pHMEInfo->poc[pyr_level][list][ref] = currSlice->listX[list][ref]->poc;
     }
  }

  pic_size_x = pHMEInfo->iImageWidth  >> pyr_level;
  pic_size_y = pHMEInfo->iImageHeight >> pyr_level;
  
  for (list = 0; list < numlists; list++)
  {
    //----- set arrays -----
    mv_block->list = (char) list;
    // changed order since ideally we could use other reference
    // results to optimize other references.
    // Otherwise, some operations could be avoided as was in the original code.
    for (ref=0; ref < currSlice->listXsize[list+0]; ref++) 
    {
       mv_block->ref_idx = (char) ref;
       for(by=0; by<(pic_size_y>>3); by++)
       {
         for(bx=0; bx<(pic_size_x>>3); bx++)
         {
          //set position;
          mv_block->pos_x2 = (short) bx;
          mv_block->pos_y2 = (short) by;
          mv_block->pos_x = (short) (bx << 3);
          mv_block->pos_y = (short) (by << 3);
          mv_block->pos_x_padded = (short) (mv_block->pos_x << 2); // + IMG_PAD_SIZE_X_TIMES4;
          mv_block->pos_y_padded = (short) (mv_block->pos_y << 2); // + IMG_PAD_SIZE_Y_TIMES4;
          hme_get_neighbors(mv_block->block, bx, by, (pic_size_x>>3));

          hme_get_original_block(pHMEInfo, pyr_level, mv_block);


          PrepareMEParams(p_Vid->currentSlice, mv_block, FALSE, list, ref);

          p_pic_mv    = pHMEInfo->p_hme_mv[pyr_level][list][ref];
          p_pic_mcost = pHMEInfo->p_hme_mcost[pyr_level][list][ref];
          p_pic_mdist = pHMEInfo->p_hme_mdist[pyr_level][list][ref];


          HMESetSearchRange(currPicSW+ref, pyr_level, &(mv_block->searchRange), currPicSWMin+ref);

          pHMEInfo->hme_distortion[pyr_level][list][ref] += HMEBlockMotionSearch(p_pic_mv, p_pic_mcost, p_pic_mdist, mv_block, lambda_factor);
        }
      }
    }
  }
  
//for (list = 0; list < numlists; list++)
  //   for (ref=0; ref < currSlice->listXsize[list+0]; ref++) 
    //    printf(" level %d list %d ref %d distortion %lld\n", pyr_level, list, ref, pHMEInfo->hme_distortion[pyr_level][list][ref]);

}


// ref cost adjustment 
static inline distblk ref_cost_adj(const Slice *currSlice, int lambda, short ref, int list_offset, int list_size)
{
  distblk old_cost, new_cost; 

  assert(list_size == 2 || list_size == 1);

  if (currSlice->listXsize[list_offset] <= 1)
    old_cost = 0;
  else
  {
    VideoParameters *p_Vid = currSlice->p_Vid;
#if JCOST_CALC_SCALEUP
    old_cost = ( ((distblk)lambda) *((distblk)(p_Vid->refbits[(ref)])) );
#else
#if (USE_RND_COST)    
    old_cost = (rshift_rnd_sf((lambda) * (p_Vid->refbits[(ref)]), LAMBDA_ACCURACY_BITS));
#else
    old_cost = ((lambda *(p_Vid->refbits[(ref)]))>> LAMBDA_ACCURACY_BITS);
#endif
#endif
  }

  if (list_size == 1)
    new_cost = 0;
  else  // list_size == 2
  {
#if JCOST_CALC_SCALEUP
    new_cost = ( ((distblk)lambda) *((distblk)(1)) );
#else
#if (USE_RND_COST)    
    new_cost = (rshift_rnd_sf((lambda) * (1), LAMBDA_ACCURACY_BITS));
#else
    new_cost = ((lambda *(1))>> LAMBDA_ACCURACY_BITS);
#endif
#endif
  }
  
  return new_cost-old_cost;
}


/*******************************************************************
current search order(1): level->by->bx->list->ref;
an alternative oder(2) is: level->list->ref->by->bx;
(1): load source once on each level;
(2): it can make cache more efficient since it focus on each reference over all blocks;
*******************************************************************/
static void HMEPicMotionSearch(Slice *currSlice, MEBlock *mv_block, int *lambda_factor)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  int pyr_level;
  int pic_size_x, pic_size_y;
 HMEInfo_t *pHMEInfo = p_Vid->pHMEInfo;

  int iLowestLevel = 0;
  
  if(pHMEInfo->perform_reorder_pass)
    memset(pHMEInfo->perform_reorder_pass, 0, sizeof(int)*pHMEInfo->iPyramidLevels);

  assert(currSlice->slice_type != I_SLICE && currSlice->slice_type != SI_SLICE && currSlice->listXsize[0] <= p_Vid->pHMEInfo->iMaxRefNum);
  assert((currSlice->slice_type != B_SLICE) || (currSlice->slice_type == B_SLICE && currSlice->listXsize[1] <= p_Vid->pHMEInfo->iMaxRefNum));
  for (pyr_level = pHMEInfo->iPyramidLevels - 1; pyr_level >= iLowestLevel; pyr_level--)
  {
     pic_size_x = pHMEInfo->iImageWidth  >> pyr_level;
     pic_size_y = pHMEInfo->iImageHeight >> pyr_level;
     SetMVFromUpperLevel(currSlice, pyr_level);
     
     mv_block->hme_level = (short) pyr_level;
     mv_block->hme_ref_size_x_pad = pic_size_x+IMG_PAD_SIZE_X*2;
     mv_block->hme_ref_size_y_pad = pic_size_y+IMG_PAD_SIZE_Y*2;
     mv_block->hme_ref_size_x_max = pic_size_x+IMG_PAD_SIZE_X-mv_block->blocksize_x;
     mv_block->hme_ref_size_y_max = pic_size_y+IMG_PAD_SIZE_Y-mv_block->blocksize_y;

     HMELevelMotionSearch(currSlice, mv_block, lambda_factor);

  }
}

static distblk HMEBlockMotionSearch(MotionVector **p_pic_mv, distblk **p_pic_mcost, distblk **p_pic_mdist, MEBlock *mv_block, int *lambda_factor)
{
  VideoParameters *p_Vid = mv_block->p_Vid;
  int list = mv_block->list;
  //int ref = mv_block->ref_idx;
  int level = mv_block->hme_level;
  
  MotionVector *mv = &mv_block->mv[list], pred;
  int bx = mv_block->pos_x2;
  int by = mv_block->pos_y2;
  distblk   min_mcost = DISTBLK_MAX;

  //get PMV;
  HMEGetPMV(p_Vid, p_pic_mv, level, bx, by, &pred);
  mv->mv_x = ((pred.mv_x+1)>>2)<<2;
  mv->mv_y = ((pred.mv_y+1)>>2)<<2;

  //do integer search;
  min_mcost = HME_EPZSIntPelBlockMotionSearch_Enh(&pred, mv_block, p_pic_mv, p_pic_mcost, lambda_factor[F_PEL] / 2);

  //set the cost and mv;
  p_pic_mv[by][bx] = *mv;
  p_pic_mcost[by][bx] = min_mcost;
  p_pic_mdist[by][bx] = min_mcost - mv_cost(p_Vid, lambda_factor[F_PEL], mv, &pred);

  //return min_mcost;
  return p_pic_mcost[by][bx];
}

distblk hme_GetNeighborMECost(distblk **pic_mcost, MEBlock *mv_block)
{
  int bx = mv_block->pos_x2;
  int by = mv_block->pos_y2;
  PixelPos *block = mv_block->block;
  distblk  stopTh;

  if(block[0].available && block[1].available)
  {
    stopTh = distblkmin(pic_mcost[by][bx-1], pic_mcost[by-1][bx]);
  }
  else if(block[0].available)
    stopTh = pic_mcost[by][bx-1];
  else if(block[1].available)
  {
    stopTh = pic_mcost[by-1][bx];
  }
  else
    stopTh = 0;
  return stopTh;
}



distblk
hme_EPZSDetermineStopCriterion (EPZSParameters * p_EPZS, distblk **pic_mcost, MEBlock * mv_block, distblk lambda_dist)
{
  int blocktype = mv_block->blocktype;
  //int blockshape_x = (mv_block->blocksize_x >> 2);
  int bx = mv_block->pos_x2;
  int by = mv_block->pos_y2;
  PixelPos *block = mv_block->block;
  distblk  sadA, sadB, sadC, stopCriterion;
  sadA = block[0].available ? pic_mcost[by][bx-1] : DISTBLK_MAX;
  sadB = block[1].available ? pic_mcost[by-1][bx] : DISTBLK_MAX;
  sadC = block[2].available ? pic_mcost[by-1][bx+1] : (block[3].available ? pic_mcost[by-1][bx-1] : DISTBLK_MAX);
  
  stopCriterion = distblkmin (sadA, distblkmin (sadB, sadC));
  stopCriterion = distblkmax (stopCriterion, p_EPZS->minthres[blocktype]);
  stopCriterion = distblkmin (stopCriterion, p_EPZS->maxthres[blocktype] + lambda_dist);
  stopCriterion = (9 * distblkmax (p_EPZS->medthres[blocktype] + lambda_dist, stopCriterion) + 2 * p_EPZS->medthres[blocktype]) >> 3;

  return stopCriterion + lambda_dist;
}


short
hme_EPZSSpatialPredictors (EPZSParameters * p_EPZS, 
                           MEBlock *mv_block, 
                           MotionVector **pic_mv
                           )
{
  PixelPos * block = mv_block->block;
  SPoint *point = p_EPZS->predictor->point;
  int blk_x = mv_block->pos_x2;
  int blk_y = mv_block->pos_y2;
  //int level = mv_block->hme_level;
  /****************
  3 | 1 | 2
  ---------
  0 | x | 7
  ---------
  4 | 5 | 6
  *****************/
  assert(mv_block->blocktype>=1);
  // zero predictor
  (point++)->motion = zero_mv;

  // Left Predictor
   if (block[0].available)
   {
     point->motion = pic_mv[blk_y][(blk_x-1)];
     point += (*(int*)(&point->motion)!=0);
   }
   /*else
   {
     (point)->motion.mv_x = level>0? (16>>level): 12;
     (point++)->motion.mv_y = 0;
   }*/

   // Up predictor
   if (block[1].available)
   {
      point->motion = pic_mv[blk_y-1][blk_x];
      point += (*(int*)(&point->motion)!=0);
   }
   /*else
   {
    (point)->motion.mv_x = 0;
    (point++)->motion.mv_y = level>0? (16>>level): 12;
   }*/

   // Up-Right predictor
   if(block[2].available)
   {
     point->motion = pic_mv[blk_y-1][blk_x+1];
     //++point;
     point += (*(int*)(&point->motion)!=0);
   }
   /*else
   {
     (point)->motion.mv_x = level>0? (-16>>level): -12;
     (point++)->motion.mv_y = 0;
   }*/

   //Up-Left predictor
   if (block[3].available)
   {
     point->motion = pic_mv[blk_y-1][blk_x-1];
     //++point;
     point += (*(int*)(&point->motion)!=0);
   }
   /*else
   {
    (point)->motion.mv_x = 0;
    (point++)->motion.mv_y = level>0? (-16>>level): -12;
   }*/

  return (short) (point  - p_EPZS->predictor->point);
}

void
hme_EPZSWindowPredictors (MotionVector * mv, EPZSStructure * predictor, int *prednum, EPZSStructure * windowPred, int level)
{
  int pos;
  SPoint *wPoint = &windowPred->point[0];
  SPoint *pPoint = &predictor->point[(*prednum)];
  MotionVector tmpMV;
  for (pos = 0; pos < windowPred->searchPoints; ++pos)
  {
    tmpMV.mv_x = (short) ((wPoint)->motion.mv_x >> level);
    tmpMV.mv_y = (short) ((wPoint++)->motion.mv_y>>level);
    (pPoint++)->motion = add_MVs (tmpMV, mv);
  }
  *prednum += windowPred->searchPoints;
}


static distblk                             //  ==> minimum motion cost after search
HME_EPZSIntPelBlockMotionSearch_Enh (
                                     MotionVector * pred_mv,  // <--  motion vector predictor in sub-pel units
                                     MEBlock * mv_block,      // <--  motion vector information
                                     MotionVector **pic_mv,
                                     distblk **pic_mcost,
                                     int lambda_factor        // <--  lagrangian parameter for determining motion cost
                                     )
{
  VideoParameters *p_Vid = mv_block->p_Vid;
  Slice *currSlice = p_Vid->currentSlice;
  InputParameters *p_Inp = p_Vid->p_Inp;
  EPZSParameters *p_EPZS = currSlice->p_EPZS;

  int blocktype = mv_block->blocktype;

  int list = mv_block->list;
  int cur_list = list; 
  short ref = mv_block->ref_idx;
  MotionVector *mv = &mv_block->mv[list];
  SearchWindow *searchRange = &mv_block->searchRange;
  int mapCenter_x = searchRange->max_x - mv->mv_x;
  int mapCenter_y = searchRange->max_y - mv->mv_y;
  StorablePicture *ref_picture = currSlice->listX[cur_list][ref];

  distblk lambda_dist = weighted_cost(lambda_factor, 2);
  distblk stopCriterion = p_EPZS->medthres[blocktype] + lambda_dist;

  distblk min_mcost = DISTBLK_MAX;
  distblk mcost = DISTBLK_MAX;


  EPZSStructure *searchPatternF = p_EPZS->searchPattern;
  uint16 **EPZSMap = &p_EPZS->EPZSMap[mapCenter_y];
  uint16 *EPZSPoint = &p_EPZS->EPZSMap[searchRange->max_y][searchRange->max_x];

  MotionVector center = pad_MVs (*mv, mv_block);
  MotionVector pred = pad_MVs (*pred_mv, mv_block);
  MotionVector tmp = *mv, cand = center, tmv;

  distblk neighbor_dist = hme_GetNeighborMECost(pic_mcost, mv_block);

  if(neighbor_dist == 0)
      neighbor_dist = (distblk)(stopCriterion*0.5);

  ++p_EPZS->BlkCount;
  if (p_EPZS->BlkCount == 0)
  {
    ++p_EPZS->BlkCount;
  }


  // Clear EPZSMap
  *EPZSPoint = p_EPZS->BlkCount;

  //--- initialize motion cost (cost for motion vector) and check ---
  min_mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred);

  //--- add residual cost to motion cost ---
  min_mcost += mv_block->computePredFPel (ref_picture, mv_block, DISTBLK_MAX-min_mcost, &cand);

  //check another predictor;
  //tmv = pic_mv[mv_block->pos_y2][mv_block->pos_x2];
  tmv.mv_x = ((pic_mv[mv_block->pos_y2][mv_block->pos_x2].mv_x+1)>>2)<<2;
  tmv.mv_y = ((pic_mv[mv_block->pos_y2][mv_block->pos_x2].mv_y+1)>>2)<<2;
  if(*(int*)&tmv != *(int*)&tmp)
  {
    if ((iabs (tmv.mv_x - mv->mv_x) - searchRange->max_x <= 0) && (iabs (tmv.mv_y - mv->mv_y) - searchRange->max_y <= 0))
    {
      EPZSMap[tmv.mv_y][mapCenter_x + tmv.mv_x] = p_EPZS->BlkCount;
      cand = pad_MVs (tmv, mv_block);
      //--- set motion cost (cost for motion vector) and check ---
      mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred);
      if(mcost < min_mcost)
      {
        mcost += mv_block->computePredFPel (ref_picture, mv_block, min_mcost-mcost, &cand);

        if((mcost < min_mcost))
        { //small enough, then early terminate;
          tmp = tmv;
          min_mcost = mcost;
        }
      }
    }
  }
  //! If p_EPZS->medthres satisfied, then terminate, otherwise generate Predictors
  //! Condition could be strengthened by consideration distortion of adjacent partitions.
  if (min_mcost > stopCriterion)
  {
    SPoint *p_EPZS_point = p_EPZS->predictor->point;
    Boolean checkMedian = FALSE;
    distblk second_mcost = DISTBLK_MAX;
    int prednum = 5;
    int conditionEPZS;
    MotionVector tmp2 = {0, 0};
    int pos;
    //short invalid_refs = 0;

    //stopCriterion = EPZSDetermineStopCriterion (p_EPZS, prevSad, mv_block, lambda_dist);
    if(mv_block->block[0].available || mv_block->block[1].available || mv_block->block[2].available || mv_block->block[3].available)
       stopCriterion = hme_EPZSDetermineStopCriterion (p_EPZS, pic_mcost, mv_block, lambda_dist);

    //! Add Spatial Predictors in predictor list.
    //! Scheme adds zero, left, top-left, top, top-right. Note that top-left adds very little
    //! in terms of performance and could be removed with little penalty if any.
    prednum = hme_EPZSSpatialPredictors(p_EPZS, mv_block/*->block*/, pic_mv);
    /*
    if (p_Inp->EPZSSpatialMem)
      EPZSSpatialMemPredictorsEnhanced (p_EPZS, mv_block, cur_list, &prednum, ref_picture->size_x >> 2);
    */
    
    // Temporal predictors
    {
    //  hme_EPZSTemporalPredictors(p_Vid->pHMEInfo, p_EPZS, mv_block, &prednum, stopCriterion, min_mcost);
    }
    

    //! Window Size Based Predictors
    //! Basically replaces a Hierarchical ME concept and helps escaping local minima, or
    //! determining large motion variations.
    //! Following predictors can be adjusted further (i.e. removed, conditioned etc)
    //! based on distortion, correlation of adjacent MVs, complexity etc. These predictors
    //! and their conditioning could also be moved after all other predictors have been
    //! tested. Adaptation could also be based on type of material and coding mode (i.e.
    //! field/frame coding,MBAFF etc considering the higher dependency with opposite parity field
    //conditionEPZS = ((min_mcost > stopCriterion)
    // && (p_Inp->EPZSFixed > 1 || (p_Inp->EPZSFixed && currSlice->slice_type == P_SLICE)));
    //conditionEPZS = ((ref == 0) && (min_mcost > stopCriterion)
    //&& (p_Inp->EPZSFixed > 1 || (p_Inp->EPZSFixed && currSlice->slice_type == P_SLICE)));
    //conditionEPZS = ((min_mcost > stopCriterion) && ((ref < 2 && blocktype < 4)

    conditionEPZS = ((min_mcost > 3 * stopCriterion) && ((ref < 2 && blocktype < 4) || (ref < 1 && blocktype == 4))
      && (p_Inp->EPZSFixed > 1 || (p_Inp->EPZSFixed && currSlice->slice_type == P_SLICE)) && (mv_block->hme_level==p_Vid->pHMEInfo->iPyramidLevels-1));

    if (conditionEPZS)
      hme_EPZSWindowPredictors (&tmp /*&(pic_mv[mv_block->pos_y2][mv_block->pos_x2]) mv*/, p_EPZS->predictor, &prednum, 
      (/*(invalid_refs > 2) && */ (ref < 1 + (currSlice->structure != FRAME /*|| currMB->list_offset*/))&& (min_mcost > 2*neighbor_dist))//FAST_MD
      ? p_EPZS->window_predictor_ext : p_EPZS->window_predictor, mv_block->hme_level);

    /*
    //! Blocktype/Reference dependent predictors.
    //! Since already mvs for other blocktypes/references have been computed, we can reuse
    //! them in order to easier determine the optimal point. Use of predictors could depend
    //! on cost,
    //conditionEPZS = (ref == 0 || (ref > 0 && min_mcost > stopCriterion) || currSlice->structure != FRAME || currMB->list_offset);
    conditionEPZS = (ref == 0 || (ref > 0 && min_mcost > 2 * stopCriterion));
     
    if (conditionEPZS && currMB->mbAddrX != 0 && p_Inp->EPZSBlockType)
      EPZSBlockTypePredictorsMB (currSlice, mv_block, p_EPZS_point, &prednum);
    */
    //! Check all predictors
    for (pos = 0; pos < prednum; ++pos)
    {
      tmv = p_EPZS_point[pos].motion;
      //if (((iabs (tmv.mv_x - mv->mv_x) > searchRange->max_x || iabs (tmv.mv_y - mv->mv_y) > searchRange->max_y)) && (tmv.mv_x || tmv.mv_y))
      if ((iabs (tmv.mv_x - mv->mv_x) - searchRange->max_x <= 0) && (iabs (tmv.mv_y - mv->mv_y) - searchRange->max_y <= 0))
      {
        EPZSPoint = &EPZSMap[tmv.mv_y][mapCenter_x + tmv.mv_x];
        if (*EPZSPoint != p_EPZS->BlkCount)
        {
          *EPZSPoint = p_EPZS->BlkCount;
          cand = pad_MVs (tmv, mv_block);

          //--- set motion cost (cost for motion vector) and check ---
          mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred);

          if (mcost < second_mcost)
          {
            mcost += mv_block->computePredFPel (ref_picture, mv_block, second_mcost - mcost, &cand);
            
            //--- check if motion cost is less than minimum cost ---
            if (mcost < min_mcost)
            {
              tmp2 = tmp;
              tmp = tmv;
              second_mcost = min_mcost;
              min_mcost = mcost;
              checkMedian = TRUE;
            }
            //else if (mcost < second_mcost && (tmp.mv_x != tmv.mv_x || tmp.mv_y != tmv.mv_y))
            else if (mcost < second_mcost)
            {
              tmp2 = tmv;
              second_mcost = mcost;
              checkMedian = TRUE;
            }
          }
        }
      }
    }


    //! Refine using EPZS pattern if needed
    //! Note that we are using a conservative threshold method. Threshold
    //! could be tested after checking only a certain number of predictors
    //! instead of the full set. Code could be easily modified for this task.
    if (min_mcost > stopCriterion)
    {
      const int mv_range = 10;
      int patternStop = 0, pointNumber = 0, checkPts, nextLast = 0;
      int totalCheckPts = 0, motionDirection = 0;

      //! Adapt pattern based on different conditions.
      if (p_Inp->EPZSPattern != 0)
      {
        if ((min_mcost < stopCriterion + ((3 * p_EPZS->medthres[blocktype]) >> 1)))
        {
          if ((tmp.mv_x == 0 && tmp.mv_y == 0) 
            || (iabs (tmp.mv_x - mv->mv_x) < (mv_range) && iabs (tmp.mv_y - mv->mv_y) < (mv_range)))
            searchPatternF = p_Vid->sdiamond;
          else
            searchPatternF = p_Vid->square;
        }
        else if (ref > 0 && blocktype != 1)
          searchPatternF = p_Vid->square;
        else
          searchPatternF = p_EPZS->searchPattern;
      }

      //! center on best predictor
      center = tmp;

      for (;;)
      {
        totalCheckPts = searchPatternF->searchPoints;
        do
        {
          checkPts = totalCheckPts;
          do
          {
            tmv = add_MVs (center, &(searchPatternF->point[pointNumber].motion));

            if (((iabs (tmv.mv_x - mv->mv_x) - searchRange->max_x) <= 0) && ((iabs (tmv.mv_y - mv->mv_y) - searchRange->max_y) <= 0))
            {
              EPZSPoint = &EPZSMap[tmv.mv_y][mapCenter_x + tmv.mv_x];
              if (*EPZSPoint != p_EPZS->BlkCount)
              {
                *EPZSPoint = p_EPZS->BlkCount;
                cand = pad_MVs (tmv, mv_block);

                mcost = mv_cost (p_Vid, lambda_factor, &cand, &pred);
                if (mcost < min_mcost)
                {
                  mcost += mv_block->computePredFPel (ref_picture, mv_block, min_mcost - mcost, &cand);
              
                  if (mcost < min_mcost)
                  {
                    tmp = tmv;
                    min_mcost = mcost;
                    motionDirection = pointNumber;
                  }
                }
              }
            }
            ++pointNumber;
            if (pointNumber >= searchPatternF->searchPoints)
              pointNumber -= searchPatternF->searchPoints;
            checkPts--;
          }
          while (checkPts > 0);

          if (nextLast || ((tmp.mv_x == center.mv_x) && (tmp.mv_y == center.mv_y)))
          {
            patternStop = searchPatternF->stopSearch;
            searchPatternF = searchPatternF->nextpattern;
            totalCheckPts = searchPatternF->searchPoints;
            nextLast = searchPatternF->nextLast;
            motionDirection = 0;
            pointNumber = 0;
          }
          else
          {
            totalCheckPts = searchPatternF->point[motionDirection].next_points;
            pointNumber = searchPatternF->point[motionDirection].start_nmbr;
            center = tmp;
          }
        }
        while (patternStop != 1);

        //! Check Second best predictor with EPZS pattern
        conditionEPZS = (checkMedian == TRUE) 
          && (ref == 0 || (ref > 0 && min_mcost < 2 * neighbor_dist))
          && (min_mcost > (( 3 * stopCriterion) >> 1)) && (p_Inp->EPZSDual > 0);

        if (!conditionEPZS)
          break;

        pointNumber = 0;
        patternStop = 0;
        motionDirection = 0;
        nextLast = 0;

        if ((tmp.mv_x == 0 && tmp.mv_y == 0) || (tmp.mv_x == mv->mv_x && tmp.mv_y == mv->mv_y))
        {
          if (iabs (tmp.mv_x - mv->mv_x) < (mv_range) && iabs (tmp.mv_y - mv->mv_y) < (mv_range))
            searchPatternF = p_Vid->sdiamond;
          else
            searchPatternF = p_Vid->square;
        }
        else
          searchPatternF = p_EPZS->searchPatternD;

        //! Second best. Note that following code is identical as for best predictor.
        center = tmp2;
        checkMedian = FALSE;
      }
    }
  }


  *mv = tmp;

  return (min_mcost);
}

void
hme_EPZSTemporalPredictors (
                        HMEInfo_t *pHMEInfo,
                        EPZSParameters * p_EPZS,            //! <-- EPZS structure
                        MEBlock *mv_block,                  //! <-- motion estimation information block
                        int *prednum, 
                        distblk stopCriterion, 
                        distblk min_mcost)
{
  int o_block_x = mv_block->pos_x2;
  int o_block_y = mv_block->pos_y2;
  int list      = mv_block->list;
  int ref       = mv_block->ref_idx;
  PixelPos * block = mv_block->block;
  SPoint * point = p_EPZS->predictor->point;
  int mvScale = p_EPZS->mv_scale[list][ref][0];
  MotionVector **col_mv = pHMEInfo->p_hme_mv[mv_block->hme_level][list][0];
  MotionVector *tmp_mv, *scaledMV;

  tmp_mv = col_mv[o_block_y]+o_block_x;
  if(*(int*)(tmp_mv))
  {  
    scaledMV = &point[(*prednum)].motion;
    scale_mv(scaledMV, mvScale, tmp_mv, 8);
    (*prednum)++;
  }


  if( (min_mcost > stopCriterion && ref < 2) )
  {
    //Left
    if (block[0].available)
    {
      tmp_mv = col_mv[o_block_y]+(o_block_x - 1);
      if(*(int*)(tmp_mv))
      {
        *prednum += add_predictor (&point[*prednum].motion, *tmp_mv, mvScale, 8);
      }        
    }
    // Up
    if (block[1].available)
    {
      tmp_mv = col_mv[o_block_y - 1] + o_block_x;
      if(*(int*)tmp_mv)
      {
        *prednum += add_predictor (&point[*prednum].motion, *tmp_mv, mvScale, 8);
      }
    }
    // upright;
    if (block[2].available)
    {
      tmp_mv = col_mv[o_block_y - 1] + o_block_x+1;
      if(*(int*)tmp_mv)
      {
        *prednum += add_predictor (&point[*prednum].motion, *tmp_mv, mvScale, 8);
      }
    }
    // upleft;
    if (block[3].available)
    {
      tmp_mv = col_mv[o_block_y - 1] + o_block_x-1;
      if(*(int*)tmp_mv)
      {
        *prednum += add_predictor (&point[*prednum].motion, *tmp_mv, mvScale, 8);
      }
    }
  }
}
