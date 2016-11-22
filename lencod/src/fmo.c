/*!
 *****************************************************************************
 *
 * \file fmo.c
 *
 * \brief
 *    Support for Flexible Macroblock Ordering for different Slice Group Modes: MBAmap handling
 *
 * \date
 *    16 June, 2002  Modified April 25, 2004
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 *    Dong Wang        Dong.Wang@bristol.ac.uk
 *
 *****************************************************************************/

/*!
 ****************************************************************************
 *   Notes by Dong Wang (April 25 2004)
 *
 *  Source codes are modified to support 7 slice group types (fmo modes).
 *  The functions for generating map are very similar to that in decoder, but have
 *  a little difference.
 *
 *  The MB map is calculated at the beginning of coding of each picture (frame or field).
 *
 *  'slice_group_change_cycle' in structure 'VideoParameters' is the syntax in the slice
 *  header. It's set to be 1 before the initialization of FMO in function code_a_picture().
 *  It can be changed every time if needed.
 *
 ****************************************************************************
 */

/*!
 *****************************************************************************
 *  How does a MBAmap look like?
 *
 *  An MBAmap is a one-diemnsional array of ints.  Each int
 *  represents an MB in scan order.  A zero or positive value represents
 *  a slice group ID.  Negative values are reserved for future extensions.
 *  The numbering range for the SliceGroupIDs is 0..7 as per JVT-C167.
 *
 *  This module contains a static variable MBAmap.  This is the MBAmap of the
 *  picture currently coded.  It can be accessed only through the access
 *  functions.
 *****************************************************************************
*/

//#define PRINT_FMO_MAPS  1


#include "global.h"

#include "fmo.h"


static void FmoGenerateType0MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps);
static void FmoGenerateType1MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps);
static void FmoGenerateType2MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps);
static void FmoGenerateType3MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps);
static void FmoGenerateType4MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps);
static void FmoGenerateType5MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps);
static void FmoGenerateType6MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps);


static int FmoGenerateMapUnitToSliceGroupMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps);
static int FmoGenerateMBAmap                 (VideoParameters * p_Vid, seq_parameter_set_rbsp_t* sps);


/*!
 ************************************************************************
 * \brief
 *    Generates p_Vid->MapUnitToSliceGroupMap 
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 *
 ************************************************************************
 */
static int FmoGenerateMapUnitToSliceGroupMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps)
{
  p_Vid->PicSizeInMapUnits = p_Vid->PicHeightInMapUnits * p_Vid->PicWidthInMbs;


  if (pps->slice_group_map_type == 6)
  {
    if ((pps->pic_size_in_map_units_minus1+1) != p_Vid->PicSizeInMapUnits)
    {
      error ("wrong pps->pic_size_in_map_units_minus1 for used SPS and FMO type 6", 500);
    }
  }

  // allocate memory for p_Vid->MapUnitToSliceGroupMap 
  if (p_Vid->MapUnitToSliceGroupMap)
    free (p_Vid->MapUnitToSliceGroupMap);

  if ((p_Vid->MapUnitToSliceGroupMap = malloc ((p_Vid->PicSizeInMapUnits) * sizeof (byte))) == NULL)
  {
    printf ("cannot allocated %d bytes for p_Vid->MapUnitToSliceGroupMap , exit\n", (int) ( p_Vid->PicSizeInMapUnits * sizeof (byte)));
    exit (-1);
  }

  if (pps->num_slice_groups_minus1 == 0)    // only one slice group
  {
    memset (p_Vid->MapUnitToSliceGroupMap, 0,  p_Vid->PicSizeInMapUnits * sizeof (byte));
    return 0;
  }

  switch (pps->slice_group_map_type)
  {
  case 0:
    FmoGenerateType0MapUnitMap (p_Vid, pps);
    break;
  case 1:
    FmoGenerateType1MapUnitMap (p_Vid, pps);
    break;
  case 2:
    FmoGenerateType2MapUnitMap (p_Vid, pps);
    break;
  case 3:
    FmoGenerateType3MapUnitMap (p_Vid, pps);
    break;
  case 4:
    FmoGenerateType4MapUnitMap (p_Vid, pps);
    break;
  case 5:
    FmoGenerateType5MapUnitMap (p_Vid, pps);
    break;
  case 6:
    FmoGenerateType6MapUnitMap (p_Vid, pps);
    break;
  default:
    printf ("Illegal slice_group_map_type %d , exit \n", pps->slice_group_map_type);
    exit (-1);
  }
  return 0;
}


/*!
 ************************************************************************
 * \brief
 *    Generates MBAmap from p_Vid->MapUnitToSliceGroupMap 
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param sps
 *    Sequence Parameter set to be used for map generation
 *
 ************************************************************************
 */
static int FmoGenerateMBAmap (VideoParameters * p_Vid, seq_parameter_set_rbsp_t* sps)
{
  unsigned i;

  // allocate memory for p_Vid->MBAmap
  if (p_Vid->MBAmap)
    free (p_Vid->MBAmap);


  if ((p_Vid->MBAmap = malloc ((p_Vid->PicSizeInMbs) * sizeof (byte))) == NULL)
  {
    printf ("cannot allocated %d bytes for p_Vid->MBAmap, exit\n", (int) ((p_Vid->PicSizeInMbs) * sizeof (byte)));
    exit (-1);
  }

  if ((sps->frame_mbs_only_flag) || p_Vid->field_picture)
  {
    for (i=0; i<p_Vid->PicSizeInMbs; i++)
    {
      p_Vid->MBAmap[i] = p_Vid->MapUnitToSliceGroupMap [i];
    }
  }
  else
    if (sps->mb_adaptive_frame_field_flag  &&  (! p_Vid->field_picture))
    {
      for (i=0; i<p_Vid->PicSizeInMbs; i++)
      {
        p_Vid->MBAmap[i] = p_Vid->MapUnitToSliceGroupMap [i >> 1];
      }
    }
    else
    {
      for (i=0; i<p_Vid->PicSizeInMbs; i++)
      {
        p_Vid->MBAmap[i] = p_Vid->MapUnitToSliceGroupMap [(i/(2*p_Vid->PicWidthInMbs))*p_Vid->PicWidthInMbs+(i%p_Vid->PicWidthInMbs)];
      }
    }
    return 0;
}


/*!
 ************************************************************************
 * \brief
 *    FMO initialization: Generates p_Vid->MapUnitToSliceGroupMap  and p_Vid->MBAmap.
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 * \param sps
 *    Sequence Parameter set to be used for map generation
 ************************************************************************
 */
int FmoInit(VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps, seq_parameter_set_rbsp_t * sps)
{

#ifdef PRINT_FMO_MAPS
  unsigned i,j;
  int bottom;
#endif

  int k;
  for (k = 0; k < MAXSLICEGROUPIDS; k++)
    p_Vid->FirstMBInSlice[k] = -1;

  FmoGenerateMapUnitToSliceGroupMap(p_Vid, pps);
  FmoGenerateMBAmap(p_Vid, sps);

#ifdef PRINT_FMO_MAPS
  printf("\n");
  printf("FMO Map (Units):\n");

  for (j=0; j<p_Vid->PicHeightInMapUnits; j++)
  {
    for (i=0; i<p_Vid->PicWidthInMbs; i++)
    {
      printf("%d ",p_Vid->MapUnitToSliceGroupMap [i+j*p_Vid->PicWidthInMbs]);
    }
    printf("\n");
  }
  printf("\n");

  if(sps->mb_adaptive_frame_field_flag==0)
  {
    printf("FMO Map (Mb):\n");
    for (j=0; j<(p_Vid->PicSizeInMbs/p_Vid->PicWidthInMbs); j++)
    {
      for (i=0; i<p_Vid->PicWidthInMbs; i++)
      {
        printf("%d ",p_Vid->MBAmap[i+j*p_Vid->PicWidthInMbs]);
      }
      printf("\n");
    }
    printf("\n");
  }
  else
  {
    printf("FMO Map (Mb in scan order for MBAFF):\n");
    for (j=0; j<(p_Vid->PicSizeInMbs/p_Vid->PicWidthInMbs); j++)
    {
      for (i=0; i<p_Vid->PicWidthInMbs; i++)
      {
        bottom=(j & 0x01);
        printf("%d ",p_Vid->MBAmap[(j-bottom)*p_Vid->PicWidthInMbs+i*2+bottom]);
      }
      printf("\n");

    }
    printf("\n");

  }

#endif

  return 0;
}


/*!
 ************************************************************************
 * \brief
 *    Free memory if allocated by FMO functions
 ************************************************************************
 */
void FmoUninit(VideoParameters *p_Vid)
{
  if (p_Vid->MBAmap)
  {
    free (p_Vid->MBAmap);
    p_Vid->MBAmap = NULL;
  }
  if (p_Vid->MapUnitToSliceGroupMap )
  {
    free (p_Vid->MapUnitToSliceGroupMap );
    p_Vid->MapUnitToSliceGroupMap  = NULL;
  }

}


/*!
 ************************************************************************
 * \brief
 *    Generate interleaved slice group map type MapUnit map (type 0)
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 ************************************************************************
 */
static void FmoGenerateType0MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps )
{
  unsigned iGroup, j;
  unsigned i = 0;
  
  do
  {
    for( iGroup = 0;
    (iGroup <= pps->num_slice_groups_minus1) && (i < p_Vid->PicSizeInMapUnits);
    i += pps->run_length_minus1[iGroup++] + 1)
    {
      for( j = 0; j <= pps->run_length_minus1[ iGroup ] && i + j < p_Vid->PicSizeInMapUnits; j++ )
        p_Vid->MapUnitToSliceGroupMap [i+j] = (byte)  iGroup;
    }
  }
  while( i < p_Vid->PicSizeInMapUnits );
}


/*!
 ************************************************************************
 * \brief
 *    Generate dispersed slice group map type MapUnit map (type 1)
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 ************************************************************************
 */
static void FmoGenerateType1MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps )
{
  unsigned i;
  for( i = 0; i < p_Vid->PicSizeInMapUnits; i++ )
  {
    p_Vid->MapUnitToSliceGroupMap [i] = (byte) (((i%p_Vid->PicWidthInMbs)+(((i/p_Vid->PicWidthInMbs)*(pps->num_slice_groups_minus1+1))>>1))
      %(pps->num_slice_groups_minus1+1));
  }
}

/*!
 ************************************************************************
 * \brief
 *    Generate foreground with left-over slice group map type MapUnit map (type 2)
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 ************************************************************************
 */
static void FmoGenerateType2MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps )
{
  int iGroup;
  unsigned i, x, y;
  unsigned yTopLeft, xTopLeft, yBottomRight, xBottomRight;

  for( i = 0; i < p_Vid->PicSizeInMapUnits; i++ )
    p_Vid->MapUnitToSliceGroupMap [ i ] = (byte) pps->num_slice_groups_minus1;

  for( iGroup = pps->num_slice_groups_minus1 - 1 ; iGroup >= 0; iGroup-- )
  {
    yTopLeft = pps->top_left[ iGroup ] / p_Vid->PicWidthInMbs;
    xTopLeft = pps->top_left[ iGroup ] % p_Vid->PicWidthInMbs;
    yBottomRight = pps->bottom_right[ iGroup ] / p_Vid->PicWidthInMbs;
    xBottomRight = pps->bottom_right[ iGroup ] % p_Vid->PicWidthInMbs;
    for( y = yTopLeft; y <= yBottomRight; y++ )
      for( x = xTopLeft; x <= xBottomRight; x++ )
        p_Vid->MapUnitToSliceGroupMap [ y * p_Vid->PicWidthInMbs + x ] = (byte) iGroup;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Generate box-out slice group map type MapUnit map (type 3)
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 ************************************************************************
 */
static void FmoGenerateType3MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps )
{
  unsigned i, k;
  int leftBound, topBound, rightBound, bottomBound;
  int x, y, xDir, yDir;
  int mapUnitVacant;

  unsigned mapUnitsInSliceGroup0 = imin((pps->slice_group_change_rate_minus1 + 1) * p_Vid->slice_group_change_cycle, p_Vid->PicSizeInMapUnits);

  for( i = 0; i < p_Vid->PicSizeInMapUnits; i++ )
    p_Vid->MapUnitToSliceGroupMap [ i ] = 2;

  x = ( p_Vid->PicWidthInMbs - pps->slice_group_change_direction_flag ) / 2;
  y = ( p_Vid->PicHeightInMapUnits - pps->slice_group_change_direction_flag ) / 2;

  leftBound   = x;
  topBound    = y;
  rightBound  = x;
  bottomBound = y;

  xDir =  pps->slice_group_change_direction_flag - 1;
  yDir =  pps->slice_group_change_direction_flag;

  for( k = 0; k < p_Vid->PicSizeInMapUnits; k += mapUnitVacant )
  {
    mapUnitVacant = ( p_Vid->MapUnitToSliceGroupMap [ y * p_Vid->PicWidthInMbs + x ]  ==  2 );
    if( mapUnitVacant )
      p_Vid->MapUnitToSliceGroupMap [ y * p_Vid->PicWidthInMbs + x ] = (byte) ( k >= mapUnitsInSliceGroup0 );

    if( xDir  ==  -1  &&  x  ==  leftBound )
    {
      leftBound = imax( leftBound - 1, 0 );
      x = leftBound;
      xDir = 0;
      yDir = 2 * pps->slice_group_change_direction_flag - 1;
    }
    else
      if( xDir  ==  1  &&  x  ==  rightBound )
      {
        rightBound = imin( rightBound + 1, (int)p_Vid->PicWidthInMbs - 1 );
        x = rightBound;
        xDir = 0;
        yDir = 1 - 2 * pps->slice_group_change_direction_flag;
      }
      else
        if( yDir  ==  -1  &&  y  ==  topBound )
        {
          topBound = imax( topBound - 1, 0 );
          y = topBound;
          xDir = 1 - 2 * pps->slice_group_change_direction_flag;
          yDir = 0;
        }
        else
          if( yDir  ==  1  &&  y  ==  bottomBound )
          {
            bottomBound = imin( bottomBound + 1, (int)p_Vid->PicHeightInMapUnits - 1 );
            y = bottomBound;
            xDir = 2 * pps->slice_group_change_direction_flag - 1;
            yDir = 0;
          }
          else
          {
            x = x + xDir;
            y = y + yDir;
          }
  }

}

/*!
 ************************************************************************
 * \brief
 *    Generate raster scan slice group map type MapUnit map (type 4)
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 ************************************************************************
 */
static void FmoGenerateType4MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps )
{

  unsigned mapUnitsInSliceGroup0 = imin((pps->slice_group_change_rate_minus1 + 1) * p_Vid->slice_group_change_cycle, p_Vid->PicSizeInMapUnits);
  unsigned sizeOfUpperLeftGroup = pps->slice_group_change_direction_flag ? ( p_Vid->PicSizeInMapUnits - mapUnitsInSliceGroup0 ) : mapUnitsInSliceGroup0;

  unsigned i;

  for( i = 0; i < p_Vid->PicSizeInMapUnits; i++ )
    if( i < sizeOfUpperLeftGroup )
      p_Vid->MapUnitToSliceGroupMap [ i ] = (byte) pps->slice_group_change_direction_flag;
    else
      p_Vid->MapUnitToSliceGroupMap [ i ] = (byte) (1 - pps->slice_group_change_direction_flag);

}

/*!
 ************************************************************************
 * \brief
 *    Generate wipe slice group map type MapUnit map (type 5)
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 ************************************************************************
*/
static void FmoGenerateType5MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps )
{

  unsigned mapUnitsInSliceGroup0 = imin((pps->slice_group_change_rate_minus1 + 1) * p_Vid->slice_group_change_cycle, p_Vid->PicSizeInMapUnits);
  unsigned sizeOfUpperLeftGroup = pps->slice_group_change_direction_flag ? ( p_Vid->PicSizeInMapUnits - mapUnitsInSliceGroup0 ) : mapUnitsInSliceGroup0;

  unsigned i,j, k = 0;

  for( j = 0; j < p_Vid->PicWidthInMbs; j++ )
    for( i = 0; i < p_Vid->PicHeightInMapUnits; i++ )
      if( k++ < sizeOfUpperLeftGroup )
        p_Vid->MapUnitToSliceGroupMap [ i * p_Vid->PicWidthInMbs + j ] = (byte) pps->slice_group_change_direction_flag;
      else
        p_Vid->MapUnitToSliceGroupMap [ i * p_Vid->PicWidthInMbs + j ] = (byte) (1 - pps->slice_group_change_direction_flag);

}

/*!
 ************************************************************************
 * \brief
 *    Generate explicit slice group map type MapUnit map (type 6)
 *
 * \param p_Vid
 *    Image Parameter to be used for map generation
 * \param pps
 *    Picture Parameter set to be used for map generation
 ************************************************************************
 */
static void FmoGenerateType6MapUnitMap (VideoParameters * p_Vid, pic_parameter_set_rbsp_t * pps )
{
  unsigned i;
  for (i=0; i<p_Vid->PicSizeInMapUnits; i++)
  {
    p_Vid->MapUnitToSliceGroupMap [i] = (byte) pps->slice_group_id[i];
  }
}

/*!
 ************************************************************************
 * \brief
 *    FmoStartPicture: initializes FMO at the begin of each new picture
 *
 * \par Input:
 *    None
 ************************************************************************
 */
int FmoStartPicture (VideoParameters *p_Vid)
{
  int i;

  assert (p_Vid->MBAmap != NULL);

  for (i=0; i<MAXSLICEGROUPIDS; i++)
    p_Vid->FirstMBInSlice[i] = FmoGetFirstMBOfSliceGroup (p_Vid, i);
  return 0;
}



/*!
 ************************************************************************
 * \brief
 *    FmoEndPicture: Ends the Scattered Slices Module (called once
 *    per picture).
 *
 * \par Input:
 *    None
 ************************************************************************
 */
int FmoEndPicture ()
{
  // Do nothing
  return 0;
}


/*!
 ************************************************************************
 * \brief
 *    FmoMB2Slice: Returns SliceID for a given MB
 *
 * \par Input:
 *    Macroblock Nr (in scan order)
 ************************************************************************
 */
int FmoMB2SliceGroup (VideoParameters *p_Vid, int mb)
{
  assert (mb < (int)p_Vid->PicSizeInMbs);
  assert (p_Vid->MBAmap != NULL);
  return p_Vid->MBAmap[mb];
}

/*!
 ************************************************************************
 * \brief
 *    FmoGetNextMBBr: Returns the MB-Nr (in scan order) of the next
 *    MB in the (FMO) Slice, -1 if the SliceGroup is finished
 *
 * \par Input:
 *    CurrentMbNr
 ************************************************************************
 */
int FmoGetNextMBNr (VideoParameters *p_Vid, int CurrentMbNr)
{

  int  SliceGroupID = FmoMB2SliceGroup (p_Vid, CurrentMbNr);

  while (++CurrentMbNr<(int)p_Vid->PicSizeInMbs &&  p_Vid->MBAmap[CurrentMbNr] != SliceGroupID)
    ;

  if (CurrentMbNr >= (int)p_Vid->PicSizeInMbs)
    return -1;    // No further MB in this slice (could be end of picture)
  else
    return CurrentMbNr;
}


/*!
 ************************************************************************
 * \brief
 *    FmoGetNextMBBr: Returns the MB-Nr (in scan order) of the next
 *    MB in the (FMO) Slice, -1 if the SliceGroup is finished
 *
 * \par Input:
 *    CurrentMbNr
 ************************************************************************
 */
int FmoGetPreviousMBNr (VideoParameters *p_Vid, int CurrentMbNr)
{

  int  SliceGroupID = FmoMB2SliceGroup (p_Vid, CurrentMbNr);
  CurrentMbNr--;
  while (CurrentMbNr>=0 &&  p_Vid->MBAmap[CurrentMbNr] != SliceGroupID)
    CurrentMbNr--;

  if (CurrentMbNr < 0)
    return -1;    // No previous MB in this slice
  else
    return CurrentMbNr;
}


/*!
 ************************************************************************
 * \brief
 *    FmoGetFirstMBOfSliceGroup: Returns the MB-Nr (in scan order) of the
 *    next first MB of the Slice group, -1 if no such MB exists
 *
 * \par Input:
 *    SliceGroupID: Id of SliceGroup
 ************************************************************************
 */
int FmoGetFirstMBOfSliceGroup (VideoParameters *p_Vid, int SliceGroupID)
{
  int i = 0;
  while ((i<(int)p_Vid->PicSizeInMbs) && (FmoMB2SliceGroup (p_Vid, i) != SliceGroupID))
    i++;

  if (i < (int)p_Vid->PicSizeInMbs)
    return i;
  else
    return -1;
}


/*!
 ************************************************************************
 * \brief
 *    FmoGetLastCodedMBOfSlice: Returns the MB-Nr (in scan order) of
 *    the last MB of the slice group
 *
 * \par Input:
 *    SliceGroupID
 * \par Return
 *    MB Nr in case of success (is always >= 0)
 *    -1 if the SliceGroup doesn't exist
 ************************************************************************
 */
int FmoGetLastCodedMBOfSliceGroup (VideoParameters *p_Vid, int SliceGroupID)
{
  int i;
  int LastMB = -1;

  for (i=0; i<(int)p_Vid->PicSizeInMbs; i++)
    if (FmoMB2SliceGroup (p_Vid, i) == SliceGroupID)
      LastMB = i;
  return LastMB;
}


void FmoSetLastMacroblockInSlice ( VideoParameters *p_Vid, int mb)
{
  // called by terminate_slice(), writes the last processed MB into the
  // FirstMBInSlice[MAXSLICEGROUPIDS] array.  FmoGetFirstMacroblockInSlice()
  // uses this info to identify the first uncoded MB in each slice group

  int currSliceGroup = FmoMB2SliceGroup (p_Vid, mb);
  assert (mb >= 0);
  mb = FmoGetNextMBNr (p_Vid, mb);   // The next (still uncoded) MB, or -1 if SG is finished
  p_Vid->FirstMBInSlice[currSliceGroup] = mb;
}

int FmoGetFirstMacroblockInSlice ( VideoParameters *p_Vid, int SliceGroup)
{
  return p_Vid->FirstMBInSlice[SliceGroup];
  // returns the first uncoded MB in each slice group, -1 if there is no
  // more to do in this slice group
}


int FmoSliceGroupCompletelyCoded( VideoParameters *p_Vid, int SliceGroupID)
{
  if (FmoGetFirstMacroblockInSlice (p_Vid, SliceGroupID) < 0)  // slice group completelty coded or not present
    return TRUE;
  else
    return FALSE;
}



