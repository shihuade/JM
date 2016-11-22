
/*!
*************************************************************************************
* \file me_epzs_common.c
*
* \brief
*    Common functions for EPZS scheme 
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*      - Alexis Michael Tourapis <alexismt@ieee.org>
*
*************************************************************************************
*/

#include <limits.h>

#include "contributors.h"
#include "global.h"
#include "image.h"
#include "memalloc.h"
#include "mb_access.h"
#include "refbuf.h"
#include "macroblock.h"
#include "me_distortion.h"
#include "me_epzs.h"
#include "me_hme.h"
#include "me_epzs_common.h"
#include "me_epzs_int.h"
#include "me_fullsearch.h"
#include "mv_search.h"

static const short BLOCK_PARENT[8] = { 1, 1, 1, 1, 2, 4, 4, 5 };  //!< {skip, 16x16, 16x8, 8x16, 8x8, 8x4, 4x8, 4x4}
static const int MIN_THRES_BASE[8] = { 0, 64, 32, 32, 16, 8, 8, 4 };
//static const int MED_THRES_BASE[8] = { 0, 256, 128, 128, 64, 32, 32, 16 };
static const int MED_THRES_BASE[8] = { 0, 192,  96,  96, 48, 24, 24, 12 };
static const int MAX_THRES_BASE[8] = { 0, 768, 384, 384, 192, 96, 96, 48 };

// Other definitions
static const char EPZS_PATTERN[6][20] = { "Diamond", "Square", "Extended Diamond", "Large Diamond", "SBP Large Diamond", "PMVFAST" };
static const char EPZS_DUAL_PATTERN[7][20] =
{ "Disabled", "Diamond", "Square", "Extended Diamond", "Large Diamond", "SBP Large Diamond", "PMVFAST" };
static const char EPZS_FIXED_PREDICTORS[4][20] = { "Disabled", "All P", "All P + B", "Aggressive" };
static const char EPZS_OTHER_PREDICTORS[2][20] = { "Disabled", "Enabled" };
static const char EPZS_SUBPEL_METHOD[3][20]    = { "Full", "Basic", "Enhanced" };

//! Define EPZS Refinement patterns
static const short pattern_data[7][12][4] =
{
  { // Small Diamond pattern
    {  0,  4,  3, 3 }, {  4,  0,  0, 3 }, {  0, -4,  1, 3 }, { -4,  0, 2, 3 }
  },
  { // Square pattern
    {  0,  4,  7, 3 }, {  4,  4,  7, 5 }, {  4,  0,  1, 3 }, {  4, -4, 1, 5 },
    {  0, -4,  3, 3 }, { -4, -4,  3, 5 }, { -4,  0,  5, 3 }, { -4,  4, 5, 5 }
  },
  { // Enhanced Diamond pattern
    { -4,  4, 10, 5 }, {  0,  8, 10, 8 }, {  0,  4, 10, 7 }, {  4,  4, 1, 5 },
    {  8,  0, 1,  8 }, {  4,  0,  1, 7 }, {  4, -4,  4, 5 }, {  0, -8, 4, 8 },
    {  0, -4, 4,  7 }, { -4, -4,  7, 5 }, { -8,  0,  7, 8 }, { -4,  0, 7, 7 }

  },
  { // Large Diamond pattern
    {  0,  8, 6,  5 }, {  4,  4, 0,  3 }, {  8,  0, 0,  5 }, {  4, -4, 2, 3 },
    {  0, -8, 2,  5 }, { -4, -4, 4,  3 }, { -8,  0, 4,  5 }, { -4,  4, 6, 3 }
  },
  { // Extended Subpixel pattern
    {  0,  8, 6, 12 }, {  4,  4, 0, 12 }, {  8,  0, 0, 12 }, {  4, -4, 2, 12 },
    {  0, -8, 2, 12 }, { -4, -4, 4, 12 }, { -8,  0, 4, 12 }, { -4,  4, 6, 12 },
    {  0,  2, 6, 12 }, {  2,  0, 0, 12 }, {  0, -2, 2, 12 }, { -2,  0, 4, 12 }
  }
};

/*!
************************************************************************
* \brief
*    Allocate EPZS pattern memory
*
* \param searchpoints
*    number of searchpoints to allocate
*
* \return
*    the allocated EPZSStructure structure
************************************************************************
*/
static EPZSStructure *
allocEPZSpattern (int searchpoints)
{
  EPZSStructure *s;
  s = calloc (1, sizeof (EPZSStructure));

  if (NULL == s)
    no_mem_exit ("alloc_EPZSpattern: s");

  s->searchPoints = searchpoints;
  s->point = (SPoint *) calloc (searchpoints, sizeof (SPoint));

  return s;
}

/*!
************************************************************************
* \brief
*    Free EPZS pattern memory.
*
* \param p
*    structure to be freed
*
************************************************************************
*/
static void
freeEPZSpattern (EPZSStructure * p)
{
  if (p)
  {
    free ((SPoint *) p->point);
    free (p);
    p = NULL;
  }
}

/*!
************************************************************************
* \brief
*    Assign EPZS pattern 
*
*
************************************************************************
*/
static void
assignEPZSpattern (EPZSStructure * pattern, int type, int stopSearch, int nextLast, EPZSStructure * nextpattern)
{
  int i;

  for (i = 0; i < pattern->searchPoints; ++i)
  {
    pattern->point[i].motion.mv_x = pattern_data[type][i][0];
    pattern->point[i].motion.mv_y = pattern_data[type][i][1];
    pattern->point[i].start_nmbr = pattern_data[type][i][2];
    pattern->point[i].next_points = pattern_data[type][i][3];
  }
  pattern->stopSearch = stopSearch;
  pattern->nextLast = nextLast;
  pattern->nextpattern = nextpattern;
}

/*!
************************************************************************
* \brief
*    Setup EPZS ME engine
************************************************************************
*/
void EPZS_setup_engine(Macroblock *currMB, InputParameters *p_Inp)
{
  currMB->IntPelME = (p_Inp->EPZSSubPelGrid) ? EPZS_integer_motion_estimation : EPZS_motion_estimation;
  currMB->BiPredME = (p_Inp->EPZSSubPelGrid) ? EPZS_integer_bipred_motion_estimation : EPZS_bipred_motion_estimation;
  if (p_Inp->EPZSSubPelME == 1)
    currMB->SubPelME = EPZS_sub_pel_motion_estimation;
  else if (p_Inp->EPZSSubPelME == 2)
    currMB->SubPelME = full_sub_pel_motion_estimation;
  else
    currMB->SubPelME = sub_pel_motion_estimation;
  if (p_Inp->EPZSSubPelMEBiPred == 1)
    currMB->SubPelBiPredME = EPZS_sub_pel_bipred_motion_estimation;
  else if (p_Inp->EPZSSubPelMEBiPred == 2)
    currMB->SubPelBiPredME = full_sub_pel_bipred_motion_estimation;
  else
    currMB->SubPelBiPredME = sub_pel_bipred_motion_estimation;
}

/*!
************************************************************************
* \brief
*    EPZS Global Initialization
************************************************************************
*/
int
EPZSInit (VideoParameters * p_Vid)
{

  int memory_size = 0;

  //! Definition of pottential EPZS patterns.
  //! It is possible to also define other patterns, or even use
  //! resizing patterns (such as the PMVFAST scheme. These patterns
  //! are only shown here as reference, while the same also holds
  //! for this implementation (i.e. new conditions could be added
  //! on adapting predictors, or thresholds etc. Note that search
  //! could also be performed on subpel positions directly while
  //! pattern needs not be restricted on integer positions only.

  //! Allocate memory and assign search patterns
  if(!p_Vid->sdiamond)
  {
   p_Vid->sdiamond = allocEPZSpattern (4);
   assignEPZSpattern (p_Vid->sdiamond, SDIAMOND, TRUE, TRUE, p_Vid->sdiamond);
  }
  if(!p_Vid->square)
  {
   p_Vid->square = allocEPZSpattern (8);
   assignEPZSpattern (p_Vid->square, SQUARE, TRUE, TRUE, p_Vid->square);
  }
  if(!p_Vid->ediamond)
  {
   p_Vid->ediamond = allocEPZSpattern (12);
   assignEPZSpattern (p_Vid->ediamond, EDIAMOND, TRUE, TRUE, p_Vid->ediamond);
  }
  if(!p_Vid->ldiamond)
  {
   p_Vid->ldiamond = allocEPZSpattern (8);
   assignEPZSpattern (p_Vid->ldiamond, LDIAMOND, TRUE, TRUE, p_Vid->ldiamond);
  }
  if(!p_Vid->sbdiamond)
  {
   p_Vid->sbdiamond = allocEPZSpattern (12);
   assignEPZSpattern (p_Vid->sbdiamond, SBDIAMOND, FALSE, TRUE, p_Vid->sdiamond);
  }
  if(!p_Vid->pmvfast)
  {
   p_Vid->pmvfast = allocEPZSpattern (8);
   assignEPZSpattern (p_Vid->pmvfast, LDIAMOND, FALSE, TRUE, p_Vid->sdiamond);
  }

  return memory_size;
}

/*!
************************************************************************
* \brief
*    Delete EPZS Alocated memory
************************************************************************
*/
void
EPZSDelete (VideoParameters * p_Vid)
{
  // Free search patterns
  if(p_Vid->pmvfast)
  {
    freeEPZSpattern (p_Vid->pmvfast);
    p_Vid->pmvfast = NULL;
  }
  if(p_Vid->sbdiamond)
  {
    freeEPZSpattern (p_Vid->sbdiamond);
    p_Vid->sbdiamond = NULL;
  }
  if(p_Vid->ldiamond)
  {
   freeEPZSpattern (p_Vid->ldiamond);
   p_Vid->ldiamond = NULL;
  }
  if(p_Vid->ediamond)
  {
   freeEPZSpattern (p_Vid->ediamond);
   p_Vid->ediamond = NULL;
  }
  if(p_Vid->sdiamond)
  {
    freeEPZSpattern (p_Vid->sdiamond);
    p_Vid->sdiamond = NULL;
  }
  if(p_Vid->square)
  {
   freeEPZSpattern (p_Vid->square);
   p_Vid->square = NULL;
  }
}

/*!
************************************************************************
* \brief
*    Allocate co-located memory
*
* \param size_x
*    horizontal luma size
* \param size_y
*    vertical luma size
* \param mb_adaptive_frame_field_flag
*    flag that indicates macroblock adaptive frame/field coding
*
* \return
*    the allocated EPZSColocParams structure
************************************************************************
*/
static EPZSColocParams *
allocEPZScolocated (int size_x, int size_y, int mb_adaptive_frame_field_flag)
{
  EPZSColocParams *s;
  s = calloc (1, sizeof (EPZSColocParams));
  if (NULL == s)
    no_mem_exit ("alloc_EPZScolocated: s");

  s->size_x = size_x;
  s->size_y = size_y;
  get_mem3Dmv (&(s->frame), 2, (size_y >> BLOCK_SHIFT), (size_x >> BLOCK_SHIFT));

  if (mb_adaptive_frame_field_flag)
  {
    get_mem3Dmv (&(s->top), 2, (size_y >> (BLOCK_SHIFT + 1)), (size_x >> BLOCK_SHIFT));
    get_mem3Dmv (&(s->bot), 2, (size_y >> (BLOCK_SHIFT + 1)), (size_x >> BLOCK_SHIFT));
  }

  s->mb_adaptive_frame_field_flag = mb_adaptive_frame_field_flag;

  return s;
}

/*!
************************************************************************
* \brief
*    Free co-located memory.
*
* \param p
*    structure to be freed
*
************************************************************************
*/
static void
freeEPZScolocated (EPZSColocParams * p)
{

  if (p)
  {
    free_mem3Dmv (p->frame);
    if (p->mb_adaptive_frame_field_flag)
    {
      free_mem3Dmv (p->top);
      free_mem3Dmv (p->bot);
    }

    free (p);
    p = NULL;
  }
}

/*!
************************************************************************
* \brief
*    EPZS Search Window Predictor Initialization
************************************************************************
*/
static void
EPZSWindowPredictorInit (short search_range, EPZSStructure * predictor, short mode)
{
  short pos;
  short searchpos, fieldsearchpos;
  int prednum = -1;
  short i;
  short search_range_qpel = 2;
  SPoint *point = predictor->point;

  if (mode == 0)
  {
    for (pos = (short) (RoundLog2 (search_range) - 2); pos > -1; pos--)
    {
      searchpos = ((search_range << search_range_qpel) >> pos);

     for (i = 1; i >= -1; i -= 2)
      {
        point[++prednum].motion.mv_x = i * searchpos;
        point[  prednum].motion.mv_y = 0;
        point[++prednum].motion.mv_x = i * searchpos;
        point[  prednum].motion.mv_y = i * searchpos;
        point[++prednum].motion.mv_x = 0;
        point[  prednum].motion.mv_y = i * searchpos;
        point[++prednum].motion.mv_x = -i * searchpos;
        point[  prednum].motion.mv_y = i * searchpos;
      }
    }
  }
  else // if (mode == 0)
  {
    for (pos = (short) (RoundLog2 (search_range) - 2); pos > -1; pos--)
    {
      searchpos = ((search_range << search_range_qpel) >> pos);

      //fieldsearchpos = ((3 * searchpos + 1) << search_range_qpel) >> (pos + 1);
      fieldsearchpos = (3 * searchpos + 2) >> 2;
      for (i = 1; i >= -1; i -= 2)
      {
        point[++prednum].motion.mv_x = i * fieldsearchpos;
        point[  prednum].motion.mv_y = 0;
        point[++prednum].motion.mv_x = i * fieldsearchpos;
        point[  prednum].motion.mv_y = i * fieldsearchpos;
        point[++prednum].motion.mv_x = 0;
        point[  prednum].motion.mv_y = i * fieldsearchpos;
        point[++prednum].motion.mv_x = -i * fieldsearchpos;
        point[  prednum].motion.mv_y = i * fieldsearchpos;
      }
      
      for (i = 1; i >= -1; i -= 2)
      {
        point[++prednum].motion.mv_x = i * searchpos;
        point[  prednum].motion.mv_y = 0;
        point[++prednum].motion.mv_x = i * searchpos;
        point[  prednum].motion.mv_y = i * fieldsearchpos;
        point[++prednum].motion.mv_x = i * searchpos;
        point[  prednum].motion.mv_y = i * searchpos;
        point[++prednum].motion.mv_x = i * fieldsearchpos;
        point[  prednum].motion.mv_y = i * searchpos;
        point[++prednum].motion.mv_x = 0;
        point[  prednum].motion.mv_y = i * searchpos;
        point[++prednum].motion.mv_x = -i * fieldsearchpos;
        point[  prednum].motion.mv_y = i * searchpos;
        point[++prednum].motion.mv_x = -i * searchpos;
        point[  prednum].motion.mv_y = i * searchpos;
        point[++prednum].motion.mv_x = -i * searchpos;
        point[  prednum].motion.mv_y = i * fieldsearchpos;
      }
    }
  }

  predictor->searchPoints = prednum;
}

/*!
************************************************************************
* \brief
*    EPZS Global Initialization
************************************************************************
*/
int
EPZSStructInit (Slice * currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  EPZSParameters *p_EPZS = currSlice->p_EPZS;
  int max_list_number = p_Vid->mb_aff_frame_flag ? 6 : 2;
  int pel_error_me = 1 << (p_Vid->bitdepth_luma - 8);
  int pel_error_me_cr = 1 << (p_Vid->bitdepth_chroma - 8);
  int i, memory_size = 0;
  double chroma_weight =
    p_Inp->ChromaMEEnable ? pel_error_me_cr * p_Inp->ChromaMEWeight * (double) (p_Vid->width_cr * p_Vid->height_cr) /
    (double) (p_Vid->width * p_Vid->height) : 0;
  int searchlevels = RoundLog2 (p_Inp->search_range[p_Vid->view_id]) - 1;
  int searcharray =
    p_Inp->BiPredMotionEstimation ? (2 * imax (p_Inp->search_range[p_Vid->view_id], p_Inp->BiPredMESearchRange[p_Vid->view_id]) +
    1) << 2 : (2 * p_Inp->search_range[p_Vid->view_id] + 1) << 2;
  p_EPZS->p_Vid = p_Vid;
  p_EPZS->BlkCount = 1;

  //! In this implementation we keep threshold limits fixed.
  //! However one could adapt these limits based on lagrangian
  //! optimization considerations (i.e. qp), while also allow
  //! adaptation of the limits themselves based on content or complexity.

  for (i = 0; i < 8; ++i)
  {
#if (MVC_EXTENSION_ENABLE)
    if ( (currSlice->view_id) && (p_Inp->EnableEnhLayerEPZSScalers) )
    {
      p_EPZS->medthres[i] = p_Inp->EPZSMedThresScale[currSlice->view_id] * (MED_THRES_BASE[i] * pel_error_me + (int) (MED_THRES_BASE[i] * chroma_weight + 0.5));
      p_EPZS->maxthres[i] = p_Inp->EPZSMaxThresScale[currSlice->view_id] * (MAX_THRES_BASE[i] * pel_error_me + (int) (MAX_THRES_BASE[i] * chroma_weight + 0.5));
      p_EPZS->minthres[i] = p_Inp->EPZSMinThresScale[currSlice->view_id] * (MIN_THRES_BASE[i] * pel_error_me + (int) (MIN_THRES_BASE[i] * chroma_weight + 0.5));
      p_EPZS->subthres[i] = p_Inp->EPZSSubPelThresScale[currSlice->view_id] * (MED_THRES_BASE[i] * pel_error_me + (int) (MED_THRES_BASE[i] * chroma_weight + 0.5));
    }
    else
    {
      p_EPZS->medthres[i] = p_Inp->EPZSMedThresScale[0] * (MED_THRES_BASE[i] * pel_error_me + (int) (MED_THRES_BASE[i] * chroma_weight + 0.5));
      p_EPZS->maxthres[i] = p_Inp->EPZSMaxThresScale[0] * (MAX_THRES_BASE[i] * pel_error_me + (int) (MAX_THRES_BASE[i] * chroma_weight + 0.5));
      p_EPZS->minthres[i] = p_Inp->EPZSMinThresScale[0] * (MIN_THRES_BASE[i] * pel_error_me + (int) (MIN_THRES_BASE[i] * chroma_weight + 0.5));
      p_EPZS->subthres[i] = p_Inp->EPZSSubPelThresScale[0] * (MED_THRES_BASE[i] * pel_error_me + (int) (MED_THRES_BASE[i] * chroma_weight + 0.5));
    }
#else
    p_EPZS->medthres[i] = p_Inp->EPZSMedThresScale * (MED_THRES_BASE[i] * pel_error_me + (int) (MED_THRES_BASE[i] * chroma_weight + 0.5));
    p_EPZS->maxthres[i] = p_Inp->EPZSMaxThresScale * (MAX_THRES_BASE[i] * pel_error_me + (int) (MAX_THRES_BASE[i] * chroma_weight + 0.5));
    p_EPZS->minthres[i] = p_Inp->EPZSMinThresScale * (MIN_THRES_BASE[i] * pel_error_me + (int) (MIN_THRES_BASE[i] * chroma_weight + 0.5));
    p_EPZS->subthres[i] = p_Inp->EPZSSubPelThresScale * (MED_THRES_BASE[i] * pel_error_me + (int) (MED_THRES_BASE[i] * chroma_weight + 0.5));
#endif
    up_scale(&p_EPZS->medthres[i]);
    up_scale(&p_EPZS->maxthres[i]);
    up_scale(&p_EPZS->minthres[i]);
    up_scale(&p_EPZS->subthres[i]);
  }
  //! Allocate and assign window based predictors.
  //! Other window types could also be used, while method could be
  //! made a bit more adaptive (i.e. patterns could be assigned
  //! based on neighborhood
  p_EPZS->window_predictor = allocEPZSpattern (searchlevels * 8);
  p_EPZS->window_predictor_ext = allocEPZSpattern (searchlevels * 24);
  EPZSWindowPredictorInit ((short) p_Inp->search_range[p_Vid->view_id], p_EPZS->window_predictor, 0);
  EPZSWindowPredictorInit ((short) p_Inp->search_range[p_Vid->view_id], p_EPZS->window_predictor_ext, 1);

  //! Also assign search predictor memory
  // hierarchical + maxwindow + spatial + blocktype + temporal + memspatial
#if (MVC_EXTENSION_ENABLE)
  p_EPZS->predictor = allocEPZSpattern (10 + searchlevels * 24 + 5 + 5 + 9 * (p_Inp->EPZSTemporal[0] | p_Inp->EPZSTemporal[1]) + 3 * (p_Inp->EPZSSpatialMem));
#else
  p_EPZS->predictor = allocEPZSpattern (10 + searchlevels * 24 + 5 + 5 + 9 * (p_Inp->EPZSTemporal) + 3 * (p_Inp->EPZSSpatialMem));
#endif

  //! Finally assign memory for all other elements
  //! (distortion, EPZSMap, and temporal predictors)

  //memory_size += get_offset_mem2Dshort(&EPZSMap, searcharray, searcharray, (searcharray>>1), (searcharray>>1));
  memory_size += get_mem3Ddistblk (&(p_EPZS->distortion), max_list_number, 7, (p_Vid->width + MB_BLOCK_SIZE)/ BLOCK_SIZE);
  if (p_Inp->BiPredMotionEstimation)
    memory_size += get_mem3Ddistblk (&(p_EPZS->bi_distortion), max_list_number, 7, (p_Vid->width + MB_BLOCK_SIZE) / BLOCK_SIZE);
  memory_size += get_mem3Ddistblk (&(p_EPZS->distortion_hpel), max_list_number, 7, (p_Vid->width + MB_BLOCK_SIZE)/ BLOCK_SIZE);
  memory_size += get_mem2Dshort ((short ***) &(p_EPZS->EPZSMap), searcharray, searcharray);

  if (p_Inp->EPZSSpatialMem)
  {
#if EPZSREF
    memory_size += get_mem5Dmv (&(p_EPZS->p_motion), 6, p_Vid->max_num_references, 7, 4, p_Vid->width / BLOCK_SIZE);
#else 
    memory_size += get_mem4Dmv (&(p_EPZS->p_motion), 6, 7, 4, p_Vid->width / BLOCK_SIZE);
#endif
  }

#if (MVC_EXTENSION_ENABLE)
  if ( p_Inp->EPZSTemporal[currSlice->view_id] ) 
#else
  if (p_Inp->EPZSTemporal)
#endif
  {
    p_EPZS->p_colocated = allocEPZScolocated (p_Vid->width, p_Vid->height, p_Vid->active_sps->mb_adaptive_frame_field_flag);
  }

  switch (p_Inp->EPZSPattern)
  {
  case 5:
    p_EPZS->searchPattern = p_Vid->pmvfast;
    break;
  case 4:
    p_EPZS->searchPattern = p_Vid->sbdiamond;
    break;
  case 3:
    p_EPZS->searchPattern = p_Vid->ldiamond;
    break;
  case 2:
    p_EPZS->searchPattern = p_Vid->ediamond;
    break;
  case 1:
    p_EPZS->searchPattern = p_Vid->square;
    break;
  case 0:
  default:
    p_EPZS->searchPattern = p_Vid->sdiamond;
    break;
  }

  switch (p_Inp->EPZSDual)
  {
  case 6:
    p_EPZS->searchPatternD = p_Vid->pmvfast;
    break;
  case 5:
    p_EPZS->searchPatternD = p_Vid->sbdiamond;
    break;
  case 4:
    p_EPZS->searchPatternD = p_Vid->ldiamond;
    break;
  case 3:
    p_EPZS->searchPatternD = p_Vid->ediamond;
    break;
  case 2:
    p_EPZS->searchPatternD = p_Vid->square;
    break;
  case 1:
  default:
    p_EPZS->searchPatternD = p_Vid->sdiamond;
    break;
  }

  return memory_size;
}

/*!
************************************************************************
* \brief
*    Delete EPZS Alocated memory
************************************************************************
*/
void
EPZSStructDelete (Slice * currSlice)
{
  InputParameters *p_Inp = currSlice->p_Inp;
  EPZSParameters *p_EPZS = currSlice->p_EPZS;
#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->EPZSTemporal[currSlice->view_id])
#else
  if (p_Inp->EPZSTemporal)
#endif
    freeEPZScolocated (p_EPZS->p_colocated);

  //free_offset_mem2Dshort(EPZSMap, searcharray, (searcharray>>1), (searcharray>>1));
  free_mem2Dshort ((short **) p_EPZS->EPZSMap);
  free_mem3Ddistblk (p_EPZS->distortion);
  free_mem3Ddistblk (p_EPZS->distortion_hpel);

  if (p_Inp->BiPredMotionEstimation)
    free_mem3Ddistblk (p_EPZS->bi_distortion);

  freeEPZSpattern (p_EPZS->window_predictor_ext);
  freeEPZSpattern (p_EPZS->window_predictor);
  freeEPZSpattern (p_EPZS->predictor);

  if (p_Inp->EPZSSpatialMem)
  {
#if EPZSREF
    free_mem5Dmv (p_EPZS->p_motion);
#else
    free_mem4Dmv (p_EPZS->p_motion);
#endif
  }

  free (currSlice->p_EPZS);
  currSlice->p_EPZS = NULL;
}

//! For ME purposes restricting the co-located partition is not necessary.
/*!
************************************************************************
* \brief
*    EPZS Slice Level Initialization
************************************************************************
*/
void
EPZSSliceInit (Slice * currSlice)
{
  VideoParameters *p_Vid = currSlice->p_Vid;
  InputParameters *p_Inp = currSlice->p_Inp;
  StorablePicture *p_Pic = p_Vid->enc_picture;
  EPZSColocParams *p = currSlice->p_EPZS->p_colocated;
  StorablePicture ***listX = currSlice->listX;
  StorablePicture *fs, *fs_top, *fs_bottom;
  StorablePicture *fs1, *fs_top1, *fs_bottom1, *fsx;
  EPZSParameters *p_EPZS = currSlice->p_EPZS;
  int i, j, k, jj, jdiv, loffset;
  int prescale, iTRb, iTRp;
  int list = (currSlice->slice_type == B_SLICE) ? LIST_1 : LIST_0;
  int tempmv_scale[2];
  int epzs_scale[2][6][MAX_LIST_SIZE];
  int iref;
  int invmv_precision = 8;

  // Lets compute scaling factoes between all references in lists.
  // Needed to scale spatial predictors.
  for (j = LIST_0; j < 2 + (currSlice->mb_aff_frame_flag << 2); ++j)
  {
    for (k = 0; k < currSlice->listXsize[j]; ++k)
    {
      for (i = 0; i < currSlice->listXsize[j]; ++i)
      {
        if ((j >> 1) == 0)
        {
          iTRb = iClip3 (-128, 127, p_Pic->poc - listX[j][i]->poc);
          iTRp = iClip3 (-128, 127, p_Pic->poc - listX[j][k]->poc);
        }
        else if ((j >> 1) == 1)
        {
          iTRb = iClip3 (-128, 127, p_Pic->top_poc - listX[j][i]->poc);
          iTRp = iClip3 (-128, 127, p_Pic->top_poc - listX[j][k]->poc);
        }
        else
        {
          iTRb = iClip3 (-128, 127, p_Pic->bottom_poc - listX[j][i]->poc);
          iTRp = iClip3 (-128, 127, p_Pic->bottom_poc - listX[j][k]->poc);
        }

        if (iTRp != 0)
        {
          prescale = (16384 + iabs (iTRp / 2)) / iTRp;
          p_EPZS->mv_scale[j][i][k] = iClip3 (-2048, 2047, rshift_rnd_sf ((iTRb * prescale), 6));
        }
        else
          p_EPZS->mv_scale[j][i][k] = 256;
      }
    }
  }

#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->EPZSTemporal[currSlice->view_id])
#else
  if (p_Inp->EPZSTemporal)
#endif
  {
    MotionVector **MotionVector0 = p->frame[LIST_0];
    MotionVector **MotionVector1 = p->frame[LIST_1];

    fs_top = fs_bottom = fs = listX[list][0];
    if (currSlice->listXsize[list] > 1)
      fs_top1 = fs_bottom1 = fs1 = listX[list][1];
    else
      fs_top1 = fs_bottom1 = fs1 = listX[list][0];
    for (j = 0; j < 6; ++j)
    {
      for (i = 0; i < 6; ++i)
      {
        epzs_scale[0][j][i] = 256;
        epzs_scale[1][j][i] = 256;
      }
    }

    for (j = 0; j < 2 + (currSlice->mb_aff_frame_flag << 2); j += 2)
    {
      for (i = 0; i < currSlice->listXsize[j]; ++i)
      {
        if (j == 0)
          iTRb = iClip3 (-128, 127, p_Pic->poc - listX[LIST_0 + j][i]->poc);
        else if (j == 2)
          iTRb = iClip3 (-128, 127, p_Pic->top_poc - listX[LIST_0 + j][i]->poc);
        else
          iTRb = iClip3 (-128, 127, p_Pic->bottom_poc - listX[LIST_0 + j][i]->poc);
        iTRp = iClip3 (-128, 127, listX[list + j][0]->poc - listX[LIST_0 + j][i]->poc);

        if (iTRp != 0)
        {
          prescale = (16384 + iabs (iTRp / 2)) / iTRp;
          prescale = iClip3 (-2048, 2047, rshift_rnd_sf ((iTRb * prescale), 6));

          //prescale = (iTRb * prescale + 32) >> 6;
        }
        else                    // This could not happen but lets use it in case that reference is removed.
          prescale = 256;

        epzs_scale[0][j][i] = rshift_rnd_sf ((p_EPZS->mv_scale[j][0][i] * prescale), 8);
        epzs_scale[0][j + 1][i] = prescale - 256;

        if (currSlice->listXsize[list + j] > 1)
        {
          iTRp = iClip3 (-128, 127, listX[list + j][1]->poc - listX[LIST_0 + j][i]->poc);
          if (iTRp != 0)
          {
            prescale = (16384 + iabs (iTRp / 2)) / iTRp;
            prescale = iClip3 (-2048, 2047, rshift_rnd_sf ((iTRb * prescale), 6));
            //prescale = (iTRb * prescale + 32) >> 6;
          }
          else                  // This could not happen but lets use it for case that reference is removed.
            prescale = 256;

          epzs_scale[1][j][i] = rshift_rnd_sf ((p_EPZS->mv_scale[j][1][i] * prescale), 8);
          epzs_scale[1][j + 1][i] = prescale - 256;
        }
        else
        {
          epzs_scale[1][j][i] = epzs_scale[0][j][i];
          epzs_scale[1][j + 1][i] = epzs_scale[0][j + 1][i];
        }
      }
    }

    if (currSlice->mb_aff_frame_flag)
    {
      fs_top = listX[list + 2][0];
      fs_bottom = listX[list + 4][0];

      if (currSlice->listXsize[0] > 1)
      {
        fs_top1 = listX[list + 2][1];
        fs_bottom1 = listX[list + 4][1];
      }
    }
    else
    {
      if (currSlice->structure != FRAME)
      {
        if ((currSlice->structure != fs->structure) && (fs->coded_frame))
        {
          if (currSlice->structure == TOP_FIELD)
          {
            fs_top = fs_bottom = fs = listX[list][0]->top_field;
            fs_top1 = fs_bottom1 = fs1 = listX[list][0]->bottom_field;
          }
          else
          {
            fs_top = fs_bottom = fs = listX[list][0]->bottom_field;
            fs_top1 = fs_bottom1 = fs1 = listX[list][0]->top_field;
          }
        }
      }
    }

    if (!currSlice->active_sps->frame_mbs_only_flag)
    {
      if (currSlice->mb_aff_frame_flag)
      {
        for (j = 0; j < fs->size_y >> 2; ++j)
        {
          jj = j >> 1;
          jdiv = jj + 4 * (j >> 3);
          for (i = 0; i < fs->size_x >> 2; ++i)
          {
            if (fs->mv_info[j][i].field_frame)
            {
              //! Assign frame buffers for field MBs
              //! Check whether we should use top or bottom field mvs.
              //! Depending on the assigned poc values.
              if (iabs (p_Pic->poc - fs_bottom->poc) > iabs (p_Pic->poc - fs_top->poc))
              {
                tempmv_scale[LIST_0] = 256;
                tempmv_scale[LIST_1] = 0;

                if (fs->mv_info[jdiv][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
                {
                  fsx = fs_top1;
                  loffset = 1;
                }
                else
                {
                  fsx = fs_top;
                  loffset = 0;
                }

                if (fs->mv_info[jdiv][i] .ref_pic[LIST_0] != NULL)
                {
                  for (iref = 0; iref < imin (currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0]); ++iref)
                  {
                    if (currSlice->listX[LIST_0][iref] == fs->mv_info[jdiv][i].ref_pic[LIST_0])
                    {
                      tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0][iref];
                      tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1][iref];
                      break;
                    }
                  }

                  compute_scaled (&MotionVector0[j][i], &MotionVector1[j][i], tempmv_scale, &fsx->mv_info[jj][i].mv[LIST_0], invmv_precision);
                }
                else
                {
                  MotionVector0[j][i] = zero_mv;
                  MotionVector1[j][i] = zero_mv;
                }
              }
              else
              {
                tempmv_scale[LIST_0] = 256;
                tempmv_scale[LIST_1] = 0;
                if (fs->mv_info[jdiv + 4][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
                {
                  fsx = fs_bottom1;
                  loffset = 1;
                }
                else
                {
                  fsx = fs_bottom;
                  loffset = 0;
                }

                if (fs->mv_info[jdiv + 4][i].ref_pic[LIST_0] != NULL)
                {
                  for (iref = 0; iref < imin (currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0]); ++iref)
                  {
                    if (currSlice->listX[LIST_0][iref] == fs->mv_info[jdiv + 4][i].ref_pic[LIST_0])
                    {
                      tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0][iref];
                      tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1][iref];
                      break;
                    }
                  }

                  compute_scaled (&MotionVector0[j][i], &MotionVector1[j][i], tempmv_scale, &fsx->mv_info[jj][i].mv[LIST_0], invmv_precision);
                }
                else
                {
                  MotionVector0[j][i] = zero_mv;
                  MotionVector1[j][i] = zero_mv;
                }
              }
            }
            else
            {
              tempmv_scale[LIST_0] = 256;
              tempmv_scale[LIST_1] = 0;

              if (fs->mv_info[j][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
              {
                fsx = fs1;
                loffset = 1;
              }
              else
              {
                fsx = fs;
                loffset = 0;
              }

              if (fsx->mv_info[j][i].ref_pic[LIST_0] != NULL)
              {
                for (iref = 0; iref < imin (currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0]); ++iref)
                {
                  if (currSlice->listX[LIST_0][iref] == fsx->mv_info[j][i].ref_pic[LIST_0])
                  {
                    tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0][iref];
                    tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1][iref];
                    break;
                  }
                }
                compute_scaled (&MotionVector0[j][i], &MotionVector1[j][i], tempmv_scale, &fsx->mv_info[j][i].mv[LIST_0], invmv_precision);
              }
              else
              {
                MotionVector0[j][i] = zero_mv;
                MotionVector1[j][i] = zero_mv;
              }
            }
          }
        }
      }
      else
      {
        for (j = 0; j < fs->size_y >> 2; ++j)
        {
          jj = j >> 1;
          jdiv = jj + 4 * (j >> 3);
          for (i = 0; i < fs->size_x >> 2; ++i)
          {
            tempmv_scale[LIST_0] = 256;
            tempmv_scale[LIST_1] = 0;
            if (fs->mv_info[j][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
            {
              fsx = fs1;
              loffset = 1;
            }
            else
            {
              fsx = fs;
              loffset = 0;
            }

            if (fsx->mv_info[j][i].ref_pic[LIST_0] != NULL)
            {
              for (iref = 0; iref < imin (currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0]); ++iref)
              {
                if (currSlice->listX[LIST_0][iref] == fsx->mv_info[j][i].ref_pic[LIST_0])
                {
                  tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0][iref];
                  tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1][iref];

                  break;
                }
              }

              compute_scaled (&MotionVector0[j][i], &MotionVector1[j][i], tempmv_scale, &fsx->mv_info[j][i].mv[LIST_0], invmv_precision);
            }
            else
            {
              MotionVector0[j][i] = zero_mv;
              MotionVector1[j][i] = zero_mv;
            }
          }
        }
      }

      //! Generate field MVs from Frame MVs
      if (currSlice->structure || currSlice->mb_aff_frame_flag)
      {
        for (j = 0; j < fs->size_y >> 3; ++j)
        {
          for (i = 0; i < fs->size_x >> 2; ++i)
          {
            if (!currSlice->mb_aff_frame_flag)
            {
              tempmv_scale[LIST_0] = 256;
              tempmv_scale[LIST_1] = 0;
              if (fs->mv_info[j][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
              {
                fsx = fs1;
                loffset = 1;
              }
              else
              {
                fsx = fs;
                loffset = 0;
              }

              if (fsx->mv_info[j][i].ref_pic[LIST_0] != NULL)
              {
                for (iref = 0; iref < imin (currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0]); ++iref)
                {
                  if (currSlice->listX[LIST_0][iref] == fsx->mv_info[j][i].ref_pic[LIST_0])
                  {
                    tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0][iref];
                    tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1][iref];
                    break;
                  }
                }
                compute_scaled (&MotionVector0[j][i], &MotionVector1[j][i], tempmv_scale, &fsx->mv_info[j][i].mv[LIST_0], invmv_precision);
              }
              else
              {
                MotionVector0[j][i] = zero_mv;
                MotionVector1[j][i] = zero_mv;
              }
            }
            else
            {
              tempmv_scale[LIST_0] = 256;
              tempmv_scale[LIST_1] = 0;
              if (fs_bottom->mv_info[j][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
              {
                fsx = fs_bottom1;
                loffset = 1;
              }
              else
              {
                fsx = fs_bottom;
                loffset = 0;
              }

              if (fsx->mv_info[j][i].ref_pic[LIST_0] != NULL)
              {
                for (iref = 0; iref < imin (2 * currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0 + 4]); ++iref)
                {
                  if (currSlice->listX[LIST_0 + 4][iref] == fsx->mv_info[j][i].ref_pic[LIST_0])
                  {
                    tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0 + 4][iref];
                    tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1 + 4][iref];
                    break;
                  }
                }

                compute_scaled (&p->bot[LIST_0][j][i], &p->bot[LIST_1][j][i], tempmv_scale, &fsx->mv_info[j][i].mv[LIST_0], invmv_precision);
              }
              else
              {
                p->bot[LIST_0][j][i] = zero_mv;
                p->bot[LIST_1][j][i] = zero_mv;
              }

              if (!fs->mv_info[2 * j][i].field_frame)
              {
                p->bot[LIST_0][j][i].mv_y = (p->bot[LIST_0][j][i].mv_y + 1) >> 1;
                p->bot[LIST_1][j][i].mv_y = (p->bot[LIST_1][j][i].mv_y + 1) >> 1;
              }

              tempmv_scale[LIST_0] = 256;
              tempmv_scale[LIST_1] = 0;
              if (fs_top->mv_info[j][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
              {
                fsx = fs_top1;
                loffset = 1;
              }
              else
              {
                fsx = fs_top;
                loffset = 0;
              }
              if (fsx->mv_info[j][i].ref_pic[LIST_0] != NULL)
              {
                for (iref = 0; iref < imin (2 * currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0 + 2]); ++iref)
                {
                  if (currSlice->listX[LIST_0 + 2][iref] == fsx->mv_info[j][i].ref_pic[LIST_0])
                  {
                    tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0 + 2][iref];
                    tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1 + 2][iref];
                    break;
                  }
                }

                compute_scaled (&p->top[LIST_0][j][i], &p->top[LIST_1][j][i], tempmv_scale, &fsx->mv_info[j][i].mv[LIST_0], invmv_precision);
              }
              else
              {
                p->top[LIST_0][j][i] = zero_mv;
                p->top[LIST_1][j][i] = zero_mv;
              }

              if (!fs->mv_info[2 * j][i].field_frame)
              {
                p->top[LIST_0][j][i].mv_y = (p->top[LIST_0][j][i].mv_y + 1) >> 1;
                p->top[LIST_1][j][i].mv_y = (p->top[LIST_1][j][i].mv_y + 1) >> 1;
              }
            }
          }
        }
      }

      //! Use inference flag to remap mvs/references
      //! Frame with field co-located
      if (!currSlice->structure)
      {
        for (j = 0; j < fs->size_y >> 2; ++j)
        {
          jj = j >> 1;
          jdiv = (j >> 1) + ((j >> 3) << 2);
          for (i = 0; i < fs->size_x >> 2; ++i)
          {
            if (fs->mv_info[j][i].field_frame)
            {
              tempmv_scale[LIST_0] = 256;
              tempmv_scale[LIST_1] = 0;
              if (fs->mv_info[jdiv][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
              {
                fsx = fs1;
                loffset = 1;
              }
              else
              {
                fsx = fs;
                loffset = 0;
              }

              if (fsx->mv_info[jdiv][i].ref_pic[LIST_0] != NULL)
              {
                for (iref = 0; iref < imin (currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0]); ++iref)
                {
                  if (currSlice->listX[LIST_0][iref] == fsx->mv_info[jdiv][i].ref_pic[LIST_0])
                  {
                    tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0][iref];
                    tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1][iref];
                    break;
                  }
                }
                if (iabs (p_Pic->poc - fsx->bottom_field->poc) > iabs (p_Pic->poc - fsx->top_field->poc))
                {
                  compute_scaled (&MotionVector0[j][i], &MotionVector1[j][i], tempmv_scale,
                    &fsx->top_field->mv_info[jj][i].mv[LIST_0], invmv_precision);
                }
                else
                {
                  compute_scaled (&MotionVector0[j][i], &MotionVector1[j][i], tempmv_scale,
                    &fsx->bottom_field->mv_info[jj][i].mv[LIST_0], invmv_precision);
                }
              }
              else
              {
                MotionVector0[j][i] = zero_mv;
                MotionVector1[j][i] = zero_mv;
              }
            }
          }
        }
      }
    }
    else
    {
      for (j = 0; j < fs->size_y >> 2; ++j)
      {
        for (i = 0; i < fs->size_x >> 2; ++i)
        {
          tempmv_scale[LIST_0] = 256;
          tempmv_scale[LIST_1] = 0;
          if (fs->mv_info[j][i].ref_pic[LIST_0] == NULL && currSlice->listXsize[LIST_0] > 1)
          {
            fsx = fs1;
            loffset = 1;
          }
          else
          {
            fsx = fs;
            loffset = 0;
          }
          //if (fsx->mv_info[j][i].ref_pic[LIST_0] != NULL)
          if (fsx->mv_info[j][i].ref_pic[LIST_0] != NULL && (!p_Vid->view_id || ((fsx->ref_pic_na[0]<0 || fsx->mv_info[j][i].ref_idx[LIST_0] != fsx->ref_pic_na[0]))))
          {
            for (iref = 0; iref < imin (currSlice->num_ref_idx_active[LIST_0], currSlice->listXsize[LIST_0]); ++iref)
            {
              if (currSlice->listX[LIST_0][iref] == fsx->mv_info[j][i].ref_pic[LIST_0])
              {
                tempmv_scale[LIST_0] = epzs_scale[loffset][LIST_0][iref];
                tempmv_scale[LIST_1] = epzs_scale[loffset][LIST_1][iref];
                break;
              }
            }
            compute_scaled (&MotionVector0[j][i], &MotionVector1[j][i], tempmv_scale, &fsx->mv_info[j][i].mv[LIST_0], invmv_precision);
          }
          else
          {
            MotionVector0[j][i] = zero_mv;
            MotionVector1[j][i] = zero_mv;
          }
        }
      }
    }

    if (!currSlice->active_sps->frame_mbs_only_flag)
    {
      for (j = 0; j < fs->size_y >> 2; ++j)
      {
        for (i = 0; i < fs->size_x >> 2; ++i)
        {
          if ((!currSlice->mb_aff_frame_flag && !currSlice->structure && fs->mv_info[j][i].field_frame)
            || (currSlice->mb_aff_frame_flag && fs->mv_info[j][i].field_frame))
          {
            MotionVector0[j][i].mv_y *= 2;
            MotionVector1[j][i].mv_y *= 2;
          }
          else if (currSlice->structure && !fs->mv_info[j][i].field_frame)
          {
            MotionVector0[j][i].mv_y = (short) rshift_rnd_sf (MotionVector0[j][i].mv_y, 1);
            MotionVector1[j][i].mv_y = (short) rshift_rnd_sf (MotionVector1[j][i].mv_y, 1);
          }
        }
      }
    }
  }

}


static void
is_block_available (Macroblock * currMB, StorablePicture * ref_picture, MEBlock * mv_block, int block_available[4])
{
  if ((mv_block->block_y << 2) > 0)
  {
    if ((mv_block->block_x << 2) < 8) // first column of 8x8 blocks
    {
      if ((mv_block->block_y << 2) == 8)
      {
        block_available[0] = (mv_block->blocksize_x != MB_BLOCK_SIZE) || (currMB->mb_x < (ref_picture->size_x >> 4) - 1);
      }
      else
      {
        block_available[0] = ((mv_block->block_x << 2) + mv_block->blocksize_x != 8)
          || (currMB->mb_x < (ref_picture->size_x >> 4) - 1);
      }
    }
    else
    {
      block_available[0] = ((mv_block->block_x << 2) + mv_block->blocksize_x != MB_BLOCK_SIZE) || (currMB->mb_x < (ref_picture->size_x >> 4) - 1);
    }
  }
  else
  {
    block_available[0] = ((mv_block->block_x << 2) + mv_block->blocksize_x != MB_BLOCK_SIZE) || (currMB->mb_x < (ref_picture->size_x >> 4) - 1);
  }

  block_available[1] = ((mv_block->block_y << 2) + mv_block->blocksize_y != MB_BLOCK_SIZE) || ((currMB->mb_y < (ref_picture->size_y >> 4) - 1));
  block_available[2] = mv_block->block[0].available;
  block_available[3] = mv_block->block[1].available;
}

/*!
************************************************************************
* \brief
*    EPZS Block Type Predictors
************************************************************************
*/
void
EPZSBlockTypePredictorsMB (Slice * currSlice, MEBlock * mv_block, SPoint * point, int *prednum)
{
  int blocktype = mv_block->blocktype;
  int block_x   = mv_block->block_x;
  int block_y   = mv_block->block_y;
  int list      = mv_block->list;
  int ref       = mv_block->ref_idx;
  EPZSParameters *p_EPZS = currSlice->p_EPZS;
  MotionVector ****all_mv = currSlice->all_mv[list];
  MotionVector *cur_mv = &point[*prednum].motion;

  if (blocktype != 1)
  {
    *cur_mv = all_mv[ref][BLOCK_PARENT[blocktype]][block_y][block_x];

    //*prednum += ((cur_mv->mv_x | cur_mv->mv_y) != 0);
    *prednum += (*((int *) cur_mv) != 0);

    if(BLOCK_PARENT[blocktype] !=1)
    {
      cur_mv  = &point[*prednum].motion;
      *cur_mv = all_mv[ref][1][block_y][block_x];
      //*prednum += ((cur_mv->mv_x | cur_mv->mv_y) != 0);
      *prednum += (*((int *) cur_mv) != 0);
    }
  }

  if (ref > 0)
  {
    cur_mv = &point[*prednum].motion;
    scale_mv (cur_mv, p_EPZS->mv_scale[list][ref][ref - 1], &all_mv[ref - 1][blocktype][block_y][block_x], 8);
    //*prednum += ((cur_mv->mv_x | cur_mv->mv_y) != 0);
    *prednum += (*((int *) cur_mv) != 0);

    if (ref > 1)
    {
      cur_mv = &point[*prednum].motion;
      scale_mv (cur_mv, p_EPZS->mv_scale[list][ref][0], &all_mv[0][blocktype][block_y][block_x], 8);
      //*prednum += ((cur_mv->mv_x | cur_mv->mv_y) != 0);
      *prednum += (*((int *) cur_mv) != 0);
    }
  }
}

/*!
***********************************************************************
* \brief
*    Spatial Predictors
*    AMT/HYC
***********************************************************************
*/
short
EPZS_spatial_predictors (EPZSParameters * p_EPZS, MEBlock *mv_block, int list, int list_offset, short ref, struct pic_motion_params **mv_info)
{
  PixelPos * block = mv_block->block;
  short refA = 0, refB = 0, refC = 0, refD = 0;
  VideoParameters *p_Vid = p_EPZS->p_Vid;
  int *mot_scale = p_EPZS->mv_scale[list + list_offset][ref];
  SPoint *point = p_EPZS->predictor->point;

  // zero predictor
  (point++)->motion = zero_mv;

  // Non MB-AFF mode
  if (!p_Vid->mb_aff_frame_flag)
  {
    refA = block[0].available ? (short) mv_info[block[0].pos_y][block[0].pos_x].ref_idx[list] : -1;
    refB = block[1].available ? (short) mv_info[block[1].pos_y][block[1].pos_x].ref_idx[list] : -1;
    refC = block[2].available ? (short) mv_info[block[2].pos_y][block[2].pos_x].ref_idx[list] : -1;
    refD = block[3].available ? (short) mv_info[block[3].pos_y][block[3].pos_x].ref_idx[list] : -1;

    // Left Predictor
    if (block[0].available)
    {
      scale_mv (&point->motion, mot_scale[refA], &mv_info[block[0].pos_y][block[0].pos_x].mv[list], 8);
      /*
      if (*((int*) &point->motion) == 0)
      {
      point->motion.mv_x = 12;
      point->motion.mv_y = 0;
      }
      */
      ++point;
    }
    else
    {
      (point)->motion.mv_x = 12;
      (point++)->motion.mv_y = 0;
    }
    // Up predictor
    if (block[1].available)
    {
      scale_mv (&point->motion, mot_scale[refB], &mv_info[block[1].pos_y][block[1].pos_x].mv[list], 8);
      /*
      if (*((int*) &point->motion) == 0)
      {
      point->motion.mv_x = 0;
      point->motion.mv_y = 12;
      }
      */
      ++point;
    }
    else
    {
      (point)->motion.mv_x = 0;
      (point++)->motion.mv_y = 12;
    }

    // Up-Right predictor
    if (block[2].available)
    {
      scale_mv (&point->motion, mot_scale[refC], &mv_info[block[2].pos_y][block[2].pos_x].mv[list], 8);
      /*
      if (*((int*) &point->motion) == 0)
      {
      point->motion.mv_x = -12;
      point->motion.mv_y = 0;
      }
      */
      ++point;
    }
    else
    {
      (point)->motion.mv_x = -12;
      (point++)->motion.mv_y = 0;
    }

    //Up-Left predictor
    if (block[3].available)
    {
      scale_mv (&point->motion, mot_scale[refD], &mv_info[block[3].pos_y][block[3].pos_x].mv[list], 8);
      /*
      if (*((int*) &point->motion) == 0)
      {
      point->motion.mv_x = 0;
      point->motion.mv_y = -12;
      }
      */
      ++point;
    }
    else
    {
      (point)->motion.mv_x = 0;
      (point++)->motion.mv_y = -12;
    }
  }
  else                          // MB-AFF mode
  {
    // Field Macroblock
    if (list_offset)
    {
      refA = block[0].available ? p_Vid->mb_data[block[0].mb_addr].mb_field ? (short) mv_info[block[0].pos_y][block[0].pos_x].ref_idx[list]
      : (short) mv_info[block[0].pos_y][block[0].pos_x].ref_idx[list] * 2 : -1;
      refB = block[1].available ? p_Vid->mb_data[block[1].mb_addr].mb_field ? (short) mv_info[block[1].pos_y][block[1].pos_x].ref_idx[list]
      : (short) mv_info[block[1].pos_y][block[1].pos_x].ref_idx[list] * 2 : -1;
      refC = block[2].available ? p_Vid->mb_data[block[2].mb_addr].mb_field ? (short) mv_info[block[2].pos_y][block[2].pos_x].ref_idx[list]
      : (short) mv_info[block[2].pos_y][block[2].pos_x].ref_idx[list] * 2 : -1;
      refD = block[3].available ? p_Vid->mb_data[block[3].mb_addr].mb_field ? (short) mv_info[block[3].pos_y][block[3].pos_x].ref_idx[list]
      : (short) mv_info[block[3].pos_y][block[3].pos_x].ref_idx[list] * 2 : -1;

      // Left Predictor
      if (block[0].available)
      {
        scale_mv (&point->motion, mot_scale[refA], &mv_info[block[0].pos_y][block[0].pos_x].mv[list], 8);
        if (!p_Vid->mb_data[block[0].mb_addr].mb_field)
          //point->motion.mv_y = (short) rshift_rnd_sf(point->motion.mv_y, 1);
          point->motion.mv_y <<= 1;
        ++point;
      }
      else
      {
        (point)->motion.mv_x = 12;
        (point++)->motion.mv_y = 0;
      }

      // Up predictor
      if (block[1].available)
      {
        scale_mv (&point->motion, mot_scale[refB], &mv_info[block[1].pos_y][block[1].pos_x].mv[list], 8);
        if (!p_Vid->mb_data[block[1].mb_addr].mb_field)
          //point->motion.mv_y = (short) rshift_rnd_sf(point->motion.mv_y, 1);
          point->motion.mv_y <<= 1;
        ++point;
      }
      else
      {
        (point)->motion.mv_x = 0;
        (point++)->motion.mv_y = 12;
      }

      // Up-Right predictor
      if (block[2].available)
      {
        scale_mv (&point->motion, mot_scale[refC], &mv_info[block[2].pos_y][block[2].pos_x].mv[list], 8);
        if (!p_Vid->mb_data[block[2].mb_addr].mb_field)
          //point->motion.mv_y = (short) rshift_rnd_sf(point->motion.mv_y, 1);
          point->motion.mv_y <<= 1;
        ++point;
      }
      else
      {
        (point)->motion.mv_x = -12;
        (point++)->motion.mv_y = 0;
      }

      //Up-Left predictor
      if (block[3].available)
      {
        scale_mv (&point->motion, mot_scale[refD], &mv_info[block[3].pos_y][block[3].pos_x].mv[list], 8);
        if (!p_Vid->mb_data[block[3].mb_addr].mb_field)
          //point->motion.mv_y = (short) rshift_rnd_sf(point->motion.mv_y, 1);
          point->motion.mv_y <<= 1;
        ++point;
      }
      else
      {
        (point)->motion.mv_x = 0;
        (point++)->motion.mv_y = -12;
      }
    }
    else                        // Frame macroblock
    {
      refA = block[0].available
        ? p_Vid->mb_data[block[0].mb_addr].mb_field
        ? (short) mv_info[block[0].pos_y][block[0].pos_x].ref_idx[list] >> 1 : (short) mv_info[block[0].pos_y][block[0].pos_x].ref_idx[list] : -1;
      refB = block[1].available
        ? p_Vid->mb_data[block[1].mb_addr].mb_field
        ? (short) mv_info[block[1].pos_y][block[1].pos_x].ref_idx[list] >> 1 : (short) mv_info[block[1].pos_y][block[1].pos_x].ref_idx[list] : -1;
      refC = block[2].available
        ? p_Vid->mb_data[block[2].mb_addr].mb_field
        ? (short) mv_info[block[2].pos_y][block[2].pos_x].ref_idx[list] >> 1 : (short) mv_info[block[2].pos_y][block[2].pos_x].ref_idx[list] : -1;
      refD = block[3].available
        ? p_Vid->mb_data[block[3].mb_addr].mb_field
        ? (short) mv_info[block[3].pos_y][block[3].pos_x].ref_idx[list] >> 1 : (short) mv_info[block[3].pos_y][block[3].pos_x].ref_idx[list] : -1;

      // Left Predictor
      if (block[0].available)
      {
        scale_mv (&point->motion, mot_scale[refA], &mv_info[block[0].pos_y][block[0].pos_x].mv[list], 8);
        if (p_Vid->mb_data[block[0].mb_addr].mb_field)
          point->motion.mv_y = (short) rshift_rnd_sf (point->motion.mv_y, 1);
        ++point;
      }
      else
      {
        (point)->motion.mv_x = 12;
        (point++)->motion.mv_y = 0;
      }

      // Up predictor
      if (block[1].available)
      {
        scale_mv (&point->motion, mot_scale[refB], &mv_info[block[1].pos_y][block[1].pos_x].mv[list], 8);
        if (p_Vid->mb_data[block[1].mb_addr].mb_field)
          point->motion.mv_y = (short) rshift_rnd_sf (point->motion.mv_y, 1);
        ++point;
      }
      else
      {
        (point)->motion.mv_x = 0;
        (point++)->motion.mv_y = 12;
      }

      // Up-Right predictor
      if (block[2].available)
      {
        scale_mv (&point->motion, mot_scale[refC], &mv_info[block[2].pos_y][block[2].pos_x].mv[list], 8);
        if (p_Vid->mb_data[block[2].mb_addr].mb_field)
          point->motion.mv_y = (short) rshift_rnd_sf (point->motion.mv_y, 1);
        ++point;
      }
      else
      {
        (point)->motion.mv_x = -12;
        (point++)->motion.mv_y = 0;
      }

      //Up-Left predictor
      if (block[3].available)
      {
        scale_mv (&point->motion, mot_scale[refD], &mv_info[block[3].pos_y][block[3].pos_x].mv[list], 8);
        if (p_Vid->mb_data[block[3].mb_addr].mb_field)
          point->motion.mv_y = (short) rshift_rnd_sf (point->motion.mv_y, 1);
        ++point;
      }
      else
      {
        (point)->motion.mv_x = 12;
        (point++)->motion.mv_y = 0;
      }
    }
  }

  return ((refA == -1) + (refB == -1) + ((refC == -1) && (refD == -1)));
}

/*!
***********************************************************************
* \brief
*    Temporal Predictors
*    AMT/HYC
***********************************************************************
*/
void
EPZS_temporal_predictors (Macroblock *currMB,                 //! <-- Current Macroblock
                        StorablePicture *ref_picture,       //! <-- Current reference picture
                        EPZSParameters * p_EPZS,            //! <-- EPZS structure
                        MEBlock *mv_block,                  //! <-- motion estimation information block
                        int *prednum, 
                        distblk stopCriterion, 
                        distblk min_mcost)
{
  int list_offset  = currMB->list_offset;
  int blockshape_x = (mv_block->blocksize_x >> 2);  // horizontal block size in 4-pel units
  int blockshape_y = (mv_block->blocksize_y >> 2);  // vertical block size in 4-pel units
  int o_block_x = mv_block->pos_x2;
  int o_block_y = mv_block->pos_y2;
  int list      = mv_block->list;
  int ref       = mv_block->ref_idx;

  EPZSColocParams *p_Coloc = p_EPZS->p_colocated;
  SPoint * point = p_EPZS->predictor->point;
  int mvScale = p_EPZS->mv_scale[list + list_offset][ref][0];
  MotionVector **col_mv = (list_offset == 0) ? p_Coloc->frame[list] : (list_offset == 2) ? p_Coloc->top[list] : p_Coloc->bot[list];
  MotionVector *cur_mv = &point[*prednum].motion;

  *prednum += add_predictor (cur_mv, col_mv[o_block_y][o_block_x], mvScale, 8);
  if (min_mcost > stopCriterion && ref < 2)
  {
    int block_available[4];
    is_block_available (currMB, ref_picture, mv_block, block_available);

    if (block_available[2])
    {
      *prednum += add_predictor (&point[*prednum].motion, col_mv[o_block_y][o_block_x - 1], mvScale, 8);
      //Up_Left
      if (block_available[3])
      {
        *prednum += add_predictor (&point[*prednum].motion, col_mv[o_block_y - 1][o_block_x - 1], mvScale, 8);
      }

      //Down_Left
      if (block_available[1])
      {
        *prednum += add_predictor (&point[*prednum].motion, col_mv[o_block_y + blockshape_y][o_block_x - 1], mvScale, 8);
      }
    }

    // Up
    if (block_available[3])
    {
      *prednum += add_predictor (&point[*prednum].motion, col_mv[o_block_y - 1][o_block_x], mvScale, 8);
    }

    // Up - Right
    if (block_available[0])
    {
      *prednum += add_predictor (&point[*prednum].motion, col_mv[o_block_y][o_block_x + blockshape_x], mvScale, 8);
      if (block_available[3])
      {
        *prednum += add_predictor (&point[*prednum].motion, col_mv[o_block_y - 1][o_block_x + blockshape_x], mvScale, 8);
      }

      if (block_available[1])
      {
        *prednum += add_predictor (&point[*prednum].motion, col_mv[o_block_y + blockshape_y][o_block_x + blockshape_x], mvScale, 8);
      }
    }

    if (block_available[1])
    {
      *prednum += add_predictor (&point[*prednum].motion, col_mv[o_block_y + blockshape_y][o_block_x], mvScale, 8);
    }
  }
}


/*!
************************************************************************
* \brief
*    EPZS Block Type Predictors
************************************************************************
*/
void
EPZSBlockTypePredictors (Slice * currSlice, MEBlock *mv_block, SPoint * point, int *prednum)
{
  int blocktype = mv_block->blocktype;
  int block_x   = mv_block->block_x;
  int block_y   = mv_block->block_y;
  int list      = mv_block->list;
  int ref       = mv_block->ref_idx; 
  MotionVector ****all_mv = currSlice->all_mv[list];
  MotionVector *cur_mv = &point[*prednum].motion;

  *cur_mv = all_mv[ref][BLOCK_PARENT[blocktype]][block_y][block_x];
  
  //*prednum += ((cur_mv->mv_x | cur_mv->mv_y) != 0);
  *prednum += (*((int *) cur_mv) != 0);

  if ((ref > 0) && (currSlice->structure != FRAME))
  {
    EPZSParameters *p_EPZS = currSlice->p_EPZS;
    cur_mv = &point[*prednum].motion;
    scale_mv (cur_mv, p_EPZS->mv_scale[list][ref][ref - 1], &all_mv[ref - 1][blocktype][block_y][block_x], 8);

    //*prednum += ((cur_mv->mv_x | cur_mv->mv_y) != 0);
    *prednum += (*((int *) cur_mv) != 0);
    if (ref > 1)
    {
      cur_mv = &point[*prednum].motion;
      scale_mv (cur_mv, p_EPZS->mv_scale[list][ref][0], &all_mv[0][blocktype][block_y][block_x], 8);
      //*prednum += ((cur_mv->mv_x | cur_mv->mv_y) != 0);
      *prednum += (*((int *) cur_mv) != 0);
    }
  }

  cur_mv = &point[*prednum].motion;
  *cur_mv = all_mv[ref][1][block_y][block_x];

  //*prednum += ((cur_mv->mv_x | cur_mv->mv_y) != 0);
  *prednum += (*((int *) cur_mv) != 0);
}

/*!
************************************************************************
* \brief
*    EPZS Window Based Predictors
************************************************************************
*/
void
EPZSWindowPredictors (MotionVector * mv, EPZSStructure * predictor, int *prednum, EPZSStructure * windowPred)
{
  int pos;
  SPoint *wPoint = &windowPred->point[0];
  SPoint *pPoint = &predictor->point[(*prednum)];

  for (pos = 0; pos < windowPred->searchPoints; ++pos)
  {
    (pPoint++)->motion = add_MVs ((wPoint++)->motion, mv);
  }
  *prednum += windowPred->searchPoints;
}

/*!
***********************************************************************
* \brief
*    Spatial Predictors
*    AMT/HYC
***********************************************************************
*/
void
EPZS_spatial_memory_predictors (EPZSParameters * p_EPZS,  //!< EPZS Parameters
                          MEBlock * mv_block, //!< Motion estimation structure
                          int list, //!< Current list
                          int *prednum, //!< predictor position
                          int img_width)  //!< image width
{

  SPoint *point = p_EPZS->predictor->point;

  int blocktype = mv_block->blocktype - 1;

  int by = mv_block->block_y;

  int bs_x = (mv_block->blocksize_x >> 2);  // horizontal block size in 4-pel units
  int bs_y = (mv_block->blocksize_y >> 2);  // vertical block size in 4-pel units
  int pic_x = mv_block->pos_x2;

  int ref = mv_block->ref_idx;

#if EPZSREF
  MotionVector **prd_mv = p_EPZS->p_motion[list][ref][blocktype];
  MotionVector *cur_mv = &point[*prednum].motion;

  // Left Predictor
  if (pic_x > 0)
  {
    *cur_mv = prd_mv[by][pic_x - bs_x];
    *prednum += (*((int *) cur_mv) != 0);
    cur_mv = &point[*prednum].motion;
  }

  by = (by > 0) ? by - bs_y : 4 - bs_y;

  // Up predictor
  *cur_mv = prd_mv[by][pic_x];
  *prednum += (*((int *) cur_mv) != 0);

  // Up-Right predictor
  if (pic_x + bs_x < img_width)
  {
    cur_mv = &point[*prednum].motion;
    *cur_mv = prd_mv[by][pic_x + bs_x];
    *prednum += (*((int *) cur_mv) != 0);
  }
#else
  int mot_scale = p_EPZS->mv_scale[list][ref][0];

  MotionVector **prd_mv = p_EPZS->p_motion[list][blocktype];

  // Left Predictor
  point[*prednum].motion.mv_x = (pic_x > 0)
    ? (short) rshift_rnd_sf ((mot_scale * prd_mv[by][pic_x - bs_x].mv_x), 8)
    : 0;
  point[*prednum].motion.mv_y = (pic_x > 0)
    ? (short) rshift_rnd_sf ((mot_scale * prd_mv[by][pic_x - bs_x].mv_y), 8)
    : 0;
  *prednum += ((point[*prednum].motion.mv_x != 0) || (point[*prednum].motion.mv_y != 0));

  // Up predictor
  point[*prednum].motion.mv_x = (by > 0)
    ? (short) rshift_rnd_sf ((mot_scale * prd_mv[by - bs_y][pic_x].mv_x), 8)
    : (short) rshift_rnd_sf ((mot_scale * prd_mv[4 - bs_y][pic_x].mv_x), 8);
  point[*prednum].motion.mv_y = (by > 0)
    ? (short) rshift_rnd_sf ((mot_scale * prd_mv[by - bs_y][pic_x].mv_y), 8)
    : (short) rshift_rnd_sf ((mot_scale * prd_mv[4 - bs_y][pic_x].mv_y), 8);
  *prednum += ((point[*prednum].motion.mv_x != 0) || (point[*prednum].motion.mv_y != 0));

  // Up-Right predictor
  point[*prednum].motion.mv_x = (pic_x + bs_x < img_width)
    ? (by > 0)
    ? (short) rshift_rnd_sf ((mot_scale * prd_mv[by - bs_y][pic_x + bs_x].mv_x), 8)
    : (short) rshift_rnd_sf ((mot_scale * prd_mv[4 - bs_y][pic_x + bs_x].mv_x), 8)
    : 0;
  point[*prednum].motion.mv_y = (pic_x + bs_x < img_width)
    ? (by > 0)
    ? (short) rshift_rnd_sf ((mot_scale * prd_mv[by - bs_y][pic_x + bs_x].mv_y), 8)
    : (short) rshift_rnd_sf ((mot_scale * prd_mv[4 - bs_y][pic_x + bs_x].mv_y), 8)
    : 0;
  *prednum += ((point[*prednum].motion.mv_x != 0) || (point[*prednum].motion.mv_y != 0));
#endif
}

/*!
***********************************************************************
* \brief
*    Hierarchical Predictors
*    AMT
***********************************************************************
*/
void
EPZS_hierarchical_predictors (EPZSParameters * p_EPZS,  //!< EPZS Parameters
                          MEBlock * mv_block, //!< Motion estimation structure
                          int *prednum, //!< predictor position
                          StorablePicture *ref_picture,       //! <-- Current reference picture
                          Slice *currSlice)
{
  HMEInfo_t *pHMEInfo = currSlice->p_Vid->pHMEInfo;
  SPoint *point = p_EPZS->predictor->point;

  int list = mv_block->list;
  int ref  = mv_block->ref_idx;
  int bx   = (mv_block->pos_x >> pHMEInfo->HMEBlockSizeIdx);
  int by   = (mv_block->pos_y >> pHMEInfo->HMEBlockSizeIdx);

  MotionVector *cur_mv = &point[*prednum].motion;
  MotionVector **hme_mv;
  int i;

  for (i = 0; i < (int) currSlice->p_Dpb->ref_frames_in_buffer; i++)
  {
     if (ref_picture->poc == pHMEInfo->poc[0][list][i])
     {
       ref = i;
       break;
     }
  }

  hme_mv = pHMEInfo->p_hme_mv[0][list][ref];
  
  //printf("reference poc %d %lld\n", ref_picture->poc, pHMEInfo->poc[0][list][ref]);
  // co-located
  *cur_mv = hme_mv[by][bx];
  *prednum += (*((int *) cur_mv) != 0);
  cur_mv = &point[*prednum].motion;

  // All left predictors
  if (bx > 0)
  {
    // left predictor
    *cur_mv = hme_mv[by][bx - 1];
    *prednum += (*((int *) cur_mv) != 0);
    cur_mv = &point[*prednum].motion;
    // left above
    if (by > 0)
    {
      *cur_mv = hme_mv[by - 1][bx - 1];
      *prednum += (*((int *) cur_mv) != 0);
      cur_mv = &point[*prednum].motion;
    }
    // left below
    if (by + 1 < (ref_picture->size_y >> pHMEInfo->HMEBlockSizeIdx))
    {
      *cur_mv = hme_mv[by + 1][bx - 1];
      *prednum += (*((int *) cur_mv) != 0);
      cur_mv = &point[*prednum].motion;
    }
  }
  // All right predictors
  if (bx + 1 < (ref_picture->size_x >> pHMEInfo->HMEBlockSizeIdx))
  {
    // right predictor
    *cur_mv = hme_mv[by][bx + 1];
    *prednum += (*((int *) cur_mv) != 0);
    cur_mv = &point[*prednum].motion;
    // right above
    if (by > 0)
    {
      *cur_mv = hme_mv[by - 1][bx + 1];
      *prednum += (*((int *) cur_mv) != 0);
      cur_mv = &point[*prednum].motion;
    }
    // right below
    if (by + 1 < (ref_picture->size_y >> pHMEInfo->HMEBlockSizeIdx))
    {
      *cur_mv = hme_mv[by + 1][bx + 1];
      *prednum += (*((int *) cur_mv) != 0);
      cur_mv = &point[*prednum].motion;
    }
  }
  // above
  if (by > 0)
  {
    *cur_mv = hme_mv[by - 1][bx];
    *prednum += (*((int *) cur_mv) != 0);
    cur_mv = &point[*prednum].motion;
  }
  // below
  if (by + 1 < (ref_picture->size_y >> pHMEInfo->HMEBlockSizeIdx))
  {
    *cur_mv = hme_mv[by + 1][bx];
    *prednum += (*((int *) cur_mv) != 0);
    cur_mv = &point[*prednum].motion;
  }
}

/*!
*************************************************************************************
* \brief
*    Determine stop criterion for EPZS
*************************************************************************************
*/
distblk
EPZSDetermineStopCriterion (EPZSParameters * p_EPZS, distblk *prevSad, MEBlock * mv_block, distblk lambda_dist)
{
  int blocktype = mv_block->blocktype;
  int blockshape_x = (mv_block->blocksize_x >> 2);
  PixelPos *block = mv_block->block;
  distblk  sadA, sadB, sadC, stopCriterion;
  sadA = block[0].available ? prevSad[-blockshape_x] : DISTBLK_MAX;
  sadB = block[1].available ? prevSad[0] : DISTBLK_MAX;
  sadC = block[2].available ? prevSad[blockshape_x] : DISTBLK_MAX;

  stopCriterion = distblkmin (sadA, distblkmin (sadB, sadC));
  stopCriterion = distblkmax (stopCriterion, p_EPZS->minthres[blocktype]);
  stopCriterion = distblkmin (stopCriterion, p_EPZS->maxthres[blocktype] + lambda_dist);
  stopCriterion = (8 * distblkmax (p_EPZS->medthres[blocktype] + lambda_dist, stopCriterion) + 1 * p_EPZS->medthres[blocktype]) >> 3;

  return stopCriterion + lambda_dist;
}

/*!
***********************************************************************
* \brief
*    Report function for EPZS Fast ME
*    AMT/HYC
***********************************************************************
*/
void
EPZSOutputStats (InputParameters * p_Inp, FILE * stat, short stats_file)
{
  if (stats_file == 1)
  {
    fprintf (stat, " EPZS Pattern                 : %s\n", EPZS_PATTERN[p_Inp->EPZSPattern]);
    fprintf (stat, " EPZS Dual Pattern            : %s\n", EPZS_DUAL_PATTERN[p_Inp->EPZSDual]);
    fprintf (stat, " EPZS Fixed Predictors        : %s\n", EPZS_FIXED_PREDICTORS[p_Inp->EPZSFixed]);
    fprintf (stat, " EPZS Aggressive Predictors   : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSAggressiveWindow]);
#if (MVC_EXTENSION_ENABLE)
    if (p_Inp->num_of_views == 2)
    {
      fprintf (stat, " BL EPZS Temporal Predictors  : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSTemporal[0]]);
      fprintf (stat, " EL EPZS Temporal Predictors  : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSTemporal[1]]);
    }
    else
    {
      fprintf (stat, " EPZS Temporal Predictors     : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSTemporal[0]]);
    }
#else
    fprintf (stat, " EPZS Temporal Predictors     : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSTemporal]);
#endif
    fprintf (stat, " EPZS Spatial Predictors      : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSSpatialMem]);
#if (MVC_EXTENSION_ENABLE)
    if ( (p_Inp->num_of_views == 2) && (p_Inp->EnableEnhLayerEPZSScalers) )
    {
      fprintf (stat, " BL EPZS Threshold Multipliers     : (%d %d %d)\n", p_Inp->EPZSMedThresScale[0], p_Inp->EPZSMinThresScale[0], p_Inp->EPZSMaxThresScale[0]);
      fprintf (stat, " EL EPZS Threshold Multipliers     : (%d %d %d)\n", p_Inp->EPZSMedThresScale[1], p_Inp->EPZSMinThresScale[1], p_Inp->EPZSMaxThresScale[1]);
    }
    else
    {
      fprintf (stat, " EPZS Threshold Multipliers   : (%d %d %d)\n", p_Inp->EPZSMedThresScale[0], p_Inp->EPZSMinThresScale[0], p_Inp->EPZSMaxThresScale[0]);
    }
#else
    fprintf (stat, " EPZS Threshold Multipliers   : (%d %d %d)\n", p_Inp->EPZSMedThresScale, p_Inp->EPZSMinThresScale, p_Inp->EPZSMaxThresScale);
#endif
    fprintf (stat, " EPZS Subpel ME               : %s\n", EPZS_SUBPEL_METHOD[p_Inp->EPZSSubPelME]);
    fprintf (stat, " EPZS Subpel ME BiPred        : %s\n", EPZS_SUBPEL_METHOD[p_Inp->EPZSSubPelMEBiPred]);
  }
  else
  {
    fprintf (stat, " EPZS Pattern                      : %s\n", EPZS_PATTERN[p_Inp->EPZSPattern]);
    fprintf (stat, " EPZS Dual Pattern                 : %s\n", EPZS_DUAL_PATTERN[p_Inp->EPZSDual]);
    fprintf (stat, " EPZS Fixed Predictors             : %s\n", EPZS_FIXED_PREDICTORS[p_Inp->EPZSFixed]);
    fprintf (stat, " EPZS Aggressive Predictors        : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSAggressiveWindow]);

#if (MVC_EXTENSION_ENABLE)
    if (p_Inp->num_of_views == 2)
    {
      fprintf (stat, " BL EPZS Temporal Predictors       : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSTemporal[0]]);
      fprintf (stat, " EL EPZS Temporal Predictors       : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSTemporal[1]]);
    }
    else
    {
      fprintf (stat, " EPZS Temporal Predictors          : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSTemporal[0]]);
    }
#else
    fprintf (stat, " EPZS Temporal Predictors          : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSTemporal]);
#endif
    fprintf (stat, " EPZS Spatial Predictors           : %s\n", EPZS_OTHER_PREDICTORS[p_Inp->EPZSSpatialMem]);
#if (MVC_EXTENSION_ENABLE)
    if ( (p_Inp->num_of_views == 2) && (p_Inp->EnableEnhLayerEPZSScalers) )
    {
      fprintf (stat, " BL EPZS Threshold Multipliers     : (%d %d %d)\n", p_Inp->EPZSMedThresScale[0], p_Inp->EPZSMinThresScale[0], p_Inp->EPZSMaxThresScale[0]);
      fprintf (stat, " EL EPZS Threshold Multipliers     : (%d %d %d)\n", p_Inp->EPZSMedThresScale[1], p_Inp->EPZSMinThresScale[1], p_Inp->EPZSMaxThresScale[1]);
    }
    else
    {
      fprintf (stat, " EPZS Threshold Multipliers        : (%d %d %d)\n", p_Inp->EPZSMedThresScale[0], p_Inp->EPZSMinThresScale[0], p_Inp->EPZSMaxThresScale[0]);
    }
#else
    fprintf (stat, " EPZS Threshold Multipliers        : (%d %d %d)\n", p_Inp->EPZSMedThresScale, p_Inp->EPZSMinThresScale, p_Inp->EPZSMaxThresScale);
#endif
    fprintf (stat, " EPZS Subpel ME                    : %s\n", EPZS_SUBPEL_METHOD[p_Inp->EPZSSubPelME]);
    fprintf (stat, " EPZS Subpel ME BiPred             : %s\n", EPZS_SUBPEL_METHOD[p_Inp->EPZSSubPelMEBiPred]);
  }
}
