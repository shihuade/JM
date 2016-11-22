
/*!
 ************************************************************************
 * \file mv_search.h
 *
 * \brief
 *   array definition for motion search
 *
 * \author
 *    Inge Lille-Langoy               <inge.lille-langoy@telenor.com>   \n
 *    Alexis Michael Tourapis         <alexis.tourapis@dolby.com>       \n
 *
 ************************************************************************
 */

#ifndef _MV_SEARCH_H_
#define _MV_SEARCH_H_

#if 1
#define dist_scale_f(x)           (min_mcost)    
#else
#define dist_scale_f(x)           dist_scale(x)    
#endif


extern void get_neighbors(Macroblock *currMB, PixelPos *block, int mb_x, int mb_y, int blockshape_x);
extern void set_access_method(int *access_method, MotionVector *blk, int min_x, int min_y, int max_x, int max_y);

extern void PrepareMEParams      (Slice *currSlice, MEBlock *mv_block, int ChromaMEEnable, int list, int ref);
extern void PrepareBiPredMEParams(Slice *currSlice, MEBlock *mv_block, int ChromaMEEnable, int list, int list_offset, int ref);

extern void init_motion_search_module  (VideoParameters *p_Vid, InputParameters *p_Inp);
extern void clear_motion_search_module (VideoParameters *p_Vid, InputParameters *p_Inp);

extern void  PartitionMotionSearch    (Macroblock *currMB, int, int, int*);
extern void  SubPartitionMotionSearch (Macroblock *currMB, int, int, int*);

extern void  Get_Direct_MV_Spatial_MBAFF  (Macroblock *currMB);
extern void  Get_Direct_MV_Spatial_Normal (Macroblock *currMB);
extern void  Get_Direct_MV_Temporal       (Macroblock *currMB);

extern void  FindSkipModeMotionVector     (Macroblock *currMB);

extern void init_ME_engine    (Macroblock *currMB);
extern distblk  BlockMotionSearch (Macroblock *currMB, MEBlock *mv_block, int,int, int*);
extern void init_mv_block     (Macroblock *currMB, MEBlock *mv_block, short blocktype, int list, char ref_idx, short mb_x, short mb_y);
extern void get_original_block(VideoParameters *p_Vid, MEBlock *mv_block);
extern void free_mv_block     (MEBlock *mv_block);
extern void update_mv_block   (Macroblock *currMB, MEBlock *mv_block, int h, int v);
extern void get_search_range(MEBlock *mv_block, InputParameters *p_Inp, short ref, int blocktype);

static inline void add_mvs(MotionVector *mv0, const MotionVector *mv1)
{
  mv0->mv_x = (short) (mv0->mv_x + mv1->mv_x);
  mv0->mv_y = (short) (mv0->mv_y + mv1->mv_y);
}

static inline MotionVector add_MVs(MotionVector mv0, const MotionVector *mv1)
{
  mv0.mv_x = (short) (mv0.mv_x + mv1->mv_x);
  mv0.mv_y = (short) (mv0.mv_y + mv1->mv_y);
  
  return (mv0);
}

static inline MotionVector pad_MVs(MotionVector mv0, MEBlock *mv_block)
{
  mv0.mv_x = (short) (mv0.mv_x + mv_block->pos_x_padded);
  mv0.mv_y = (short) (mv0.mv_y + mv_block->pos_y_padded);
  
  return (mv0);
}

static inline int64 overflow_weight_cost(int lambda, int bits)
{  
#if JCOST_CALC_SCALEUP  
  return ( ((int64)lambda) * ((int64)(bits)) );
#else
#if (USE_RND_COST)
  return (i64_rshift_rnd_sf(((int64)lambda) * ((int64)(bits)) , LAMBDA_ACCURACY_BITS));
#else
  return (((int64)lambda) * ((int64)(bits)) >> LAMBDA_ACCURACY_BITS);
#endif
#endif
}

static inline distblk weight_cost(int lambda, int bits)
{  
#if JCOST_CALC_SCALEUP  
  return ( ((distblk)lambda) * ((distblk)(bits)) );
#else
#if (USE_RND_COST)
  return (rshift_rnd_sf((lambda) * (bits), LAMBDA_ACCURACY_BITS));
#else
  return (((lambda) * (bits)) >> LAMBDA_ACCURACY_BITS);
#endif
#endif
}

static inline distblk mv_cost(const VideoParameters *p_Vid, int lambda, const MotionVector *mv, const MotionVector *pmv)
{
#if JCOST_CALC_SCALEUP
  int *mvbits = p_Vid->mvbits;
  return ( ((distblk)lambda) *((distblk)(mvbits[mv->mv_x - pmv->mv_x] + mvbits[mv->mv_y - pmv->mv_y])) );
#else
#if (USE_RND_COST)
  return (rshift_rnd_sf((lambda *(p_Vid->mvbits[mv->mv_x - pmv->mv_x] + p_Vid->mvbits[mv->mv_y - pmv->mv_y])), LAMBDA_ACCURACY_BITS));
#else
  return ((lambda *(p_Vid->mvbits[mv->mv_x - pmv->mv_x] + p_Vid->mvbits[mv->mv_y - pmv->mv_y]))>> LAMBDA_ACCURACY_BITS);
#endif
#endif
}

static inline distblk ref_cost(const Slice *currSlice, int lambda, short ref, int list_offset)
{
  if (currSlice->listXsize[list_offset] <= 1)
    return 0;
  else
  {
    VideoParameters *p_Vid = currSlice->p_Vid;
#if JCOST_CALC_SCALEUP
    return ( ((distblk)lambda) *((distblk)(p_Vid->refbits[(ref)])) );
#else
#if (USE_RND_COST)    
    return (rshift_rnd_sf((lambda) * (p_Vid->refbits[(ref)]), LAMBDA_ACCURACY_BITS));
#else
    return ((lambda *(p_Vid->refbits[(ref)]))>> LAMBDA_ACCURACY_BITS);
#endif
#endif
  }
}

static inline int GetMaxMVD(MotionVector *pMV, MotionVector *pRefMV)
{
  int i32MVx = iabs(pMV->mv_x - pRefMV->mv_x);
  int i32MVy = iabs(pMV->mv_y - pRefMV->mv_y);
  return imax(i32MVx, i32MVy);
}
#endif

