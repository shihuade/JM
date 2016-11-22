      
/*!
 ***********************************************************************
 * \file
 *    configfile.c
 * \brief
 *    Configuration handling.
 * \author
 *  Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Stephan Wenger           <stewe@cs.tu-berlin.de>
 * \note
 *    In the future this module should hide the Parameters and offer only
 *    Functions for their access.  Modules which make frequent use of some parameters
 *    (e.g. picture size in macroblocks) are free to buffer them on local variables.
 *    This will not only avoid global variable and make the code more readable, but also
 *    speed it up.  It will also greatly facilitate future enhancements such as the
 *    handling of different picture sizes in the same sequence.                         \n
 *                                                                                      \n
 *    For now, everything is just copied to the inp_par structure (gulp)
 *
 **************************************************************************************
 * \par Configuration File Format
 **************************************************************************************
 * Format is line oriented, maximum of one parameter per line                           \n
 *                                                                                      \n
 * Lines have the following format:                                                     \n
 * \<ParameterName\> = \<ParameterValue\> # Comments \\n                                    \n
 * Whitespace is space and \\t
 * \par
 * \<ParameterName\> are the predefined names for Parameters and are case sensitive.
 *   See configfile.h for the definition of those names and their mapping to
 *   cfgparams->values.
 * \par
 * \<ParameterValue\> are either integers [0..9]* or strings.
 *   Integers must fit into the wordlengths, signed values are generally assumed.
 *   Strings containing no whitespace characters can be used directly.  Strings containing
 *   whitespace characters are to be inclosed in double quotes ("string with whitespace")
 *   The double quote character is forbidden (may want to implement something smarter here).
 * \par
 * Any Parameters whose ParameterName is undefined lead to the termination of the program
 * with an error message.
 *
 * \par Known bug/Shortcoming:
 *    zero-length strings (i.e. to signal an non-existing file
 *    have to be coded as "".
 *
 * \par Rules for using command files
 *                                                                                      \n
 * All Parameters are initially taken from DEFAULTCONFIGFILENAME, defined in configfile.h.
 * If an -f \<config\> parameter is present in the command line then this file is used to
 * update the defaults of DEFAULTCONFIGFILENAME.  There can be more than one -f parameters
 * present.  If -p <ParameterName = ParameterValue> parameters are present then these
 * override the default and the additional config file's settings, and are themselves
 * overridden by future -p parameters.  There must be whitespace between -f and -p commands
 * and their respective parameters
 ***********************************************************************
 */

#define INCLUDED_BY_CONFIGFILE_C

#include <sys/stat.h>

#include "global.h"
#include "config_common.h"
#include "configfile.h"
#include "fmo.h"
#include "conformance.h"
#include "mc_prediction.h"
#include "mv_search.h"
#include "img_io.h"
#include "ratectl.h"

static void PatchInp                (VideoParameters *p_Vid, InputParameters *p_Inp);
static int  TestEncoderParams       (Mapping *Map, int bitdepth_qp_scale[3]);
static int  DisplayEncoderParams    (Mapping *Map);

static const int mb_width_cr[4] = {0,8, 8,16};
static const int mb_height_cr[4]= {0,8,16,16};

#define MAX_ITEMS_TO_PARSE  10000

#include "sei.h"
static void SetVUIScaleAndTicks(InputParameters *p_Inp, double frame_rate);

/*!
 ***********************************************************************
 * \brief
 *   Set JM VUI parameters
 ***********************************************************************
 */

static void set_jm_vui_params( InputParameters *p_Inp )
{
  SetVUIScaleAndTicks(p_Inp, 1.25 * p_Inp->output.frame_rate); 

  p_Inp->EnableVUISupport             = 1;

  p_Inp->VUI.nal_hrd_parameters_present_flag = 0;
  p_Inp->VUI.vcl_hrd_parameters_present_flag = 0;
  p_Inp->VUI.timing_info_present_flag = 1;   
  p_Inp->VUI.pic_struct_present_flag  = 1;
  p_Inp->VUI.fixed_frame_rate_flag    = 1;
}

/*!
 ***********************************************************************
 * \brief
 *   print help message and exit
 ***********************************************************************
 */
void JMHelpExit (void)
{
  fprintf( stderr, "\n   lencod [-h] [-d defenc.cfg] {[-f curenc1.cfg]...[-f curencN.cfg]}"
    " {[-p EncParam1=EncValue1]..[-p EncParamM=EncValueM]}\n\n"
    "## Parameters\n\n"

    "## Options\n"
    "   -h :  prints function usage\n"
    "   -d :  use <defenc.cfg> as default file for parameter initializations.\n"
    "         If not used then file defaults to encoder.cfg in local directory.\n"
    "   -f :  read <curencM.cfg> for reseting selected encoder parameters.\n"
    "         Multiple files could be used that set different parameters\n"
    "   -p :  Set parameter <EncParamM> to <EncValueM>.\n"
    "         See default encoder.cfg file for description of all parameters.\n\n"

    "## Supported video file formats\n"
    "   RAW:  .yuv.,rgb ->  P444 - Planar, 4:4:4 \n"
    "                       P422 - Planar, 4:2:2 \n"
    "                       P420 - Planar, 4:2:0  \n"
    "                       P400 - Planar, 4:0:0 \n"
    "                       I444 - Packed, 4:4:4 \n"
    "                       I422 - Packed, 4:2:2 \n"
    "                       I420 - Packed, 4:2:0 \n"
    "                       IYUV/YV12 - Planar, 4:2:0 \n"
    "                       IYU1 - Packed, 4:2:0 (UYYVYY) \n"
    "                       IYU2 - Packed, 4:4:4 (UYV) \n"
    "                       YUY2 - Packed, 4:2:2 (YUYV) \n"
    "                       YUV  - Packed, 4:4:4 (YUV) \n\n"

    "## Examples of usage:\n"
    "   lencod\n"
    "   lencod  -h\n"
    "   lencod  -d default.cfg\n"
    "   lencod  -f curenc1.cfg\n"
    "   lencod  -f curenc1.cfg -p InputFile=\"e:\\data\\container_qcif_30.yuv\" -p SourceWidth=176 -p SourceHeight=144\n"
    "   lencod  -f curenc1.cfg -p FramesToBeEncoded=30 -p QPISlice=28 -p QPPSlice=28 -p QPBSlice=30\n");

  exit(-1);
}

/*!
 ************************************************************************
 * \brief
 *    Reads Input File Size 
 *
 ************************************************************************
 */
int64 getVideoFileSize(int video_file)
{
   int64 fsize;   

   lseek(video_file, 0, SEEK_END); 
   fsize = tell((int) video_file); 
   lseek(video_file, 0, SEEK_SET); 

   return fsize;
}

/*!
 ************************************************************************
 * \brief
 *    Updates the number of frames to encode based on the file size
 *
 ************************************************************************
 */
void get_number_of_frames (InputParameters *p_Inp, VideoDataFile *input_file)
{
  int64 fsize = getVideoFileSize(input_file->f_num);

  int64 isize = (int64) p_Inp->source.size;
  int maxBitDepth = imax(p_Inp->source.bit_depth[0], p_Inp->source.bit_depth[1]);

  isize <<= (maxBitDepth > 8)? 1: 0;
  p_Inp->no_frames   = (int) (((fsize - p_Inp->infile_header)/ isize) - p_Inp->start_frame);
}

/*!
 ************************************************************************
 * \brief
 *    Updates images max values
 *
 ************************************************************************
 */
static void updateMaxValue(FrameFormat *format)
{
  format->max_value[0] = (1 << format->bit_depth[0]) - 1;
  format->max_value_sq[0] = format->max_value[0] * format->max_value[0];
  format->max_value[1] = (1 << format->bit_depth[1]) - 1;
  format->max_value_sq[1] = format->max_value[1] * format->max_value[1];
  format->max_value[2] = (1 << format->bit_depth[2]) - 1;
  format->max_value_sq[2] = format->max_value[2] * format->max_value[2];
}

/*!
 ************************************************************************
 * \brief
 *    Update output format parameters (resolution & bit-depth) given input
 *
 ************************************************************************
 */
static void updateOutFormat(InputParameters *p_Inp)
{
  FrameFormat *output = &p_Inp->output;
  FrameFormat *source = &p_Inp->source;
  output->yuv_format  = (ColorFormat) p_Inp->yuv_format;
  source->yuv_format  = (ColorFormat) p_Inp->yuv_format;

  if (p_Inp->src_resize == 0)
  {
    output->width[0]   = source->width[0];
    output->height[0]  = source->height[0];
  }

  if (p_Inp->yuv_format == YUV400) // reset bitdepth of chroma for 400 content
  {
    source->bit_depth[1] = 8;
    output->bit_depth[1] = 8;
    source->width[1]  = 0;
    source->width[2]  = 0;
    source->height[1] = 0;
    source->height[2] = 0;
    output->width[1]  = 0;
    output->width[2]  = 0;
    output->height[1] = 0;
    output->height[2] = 0;
  }
  else
  {
    source->width[1]  = (source->width[0]  * mb_width_cr [output->yuv_format]) >> 4;
    source->width[2]  = source->width[1];
    source->height[1] = (source->height[0] * mb_height_cr[output->yuv_format]) >> 4;
    source->height[2] = source->height[1];
    output->width[1]  = (output->width[0]  * mb_width_cr [output->yuv_format]) >> 4;
    output->width[2]  = output->width[1];
    output->height[1] = (output->height[0] * mb_height_cr[output->yuv_format]) >> 4;
    output->height[2] = output->height[1];
  }

  // source size
  source->size_cmp[0] = source->width[0] * source->height[0];
  source->size_cmp[1] = source->width[1] * source->height[1];
  source->size_cmp[2] = source->size_cmp[1];
  source->size        = source->size_cmp[0] + source->size_cmp[1] + source->size_cmp[2];
  source->mb_width    = source->width[0]  / MB_BLOCK_SIZE;
  source->mb_height   = source->height[0] / MB_BLOCK_SIZE;
  source->pic_unit_size_on_disk = (imax(source->bit_depth[0], source->bit_depth[1]) > 8) ? 16 : 8;
  source->pic_unit_size_shift3 = source->pic_unit_size_on_disk >> 3;


  // output size (excluding padding)
  output->size_cmp[0] = output->width[0] * output->height[0];
  output->size_cmp[1] = output->width[1] * output->height[1];
  output->size_cmp[2] = output->size_cmp[1];
  output->size        = output->size_cmp[0] + output->size_cmp[1] + output->size_cmp[2];
  output->mb_width    = output->width[0]  / MB_BLOCK_SIZE;
  output->mb_height   = output->height[0] / MB_BLOCK_SIZE;


  // both chroma components have the same bitdepth
  source->bit_depth[2] = source->bit_depth[1];
  output->bit_depth[2] = output->bit_depth[1];
  
  // if no bitdepth rescale ensure bitdepth is same
  if (p_Inp->src_BitDepthRescale == 0) 
  {    
    output->bit_depth[0] = source->bit_depth[0];
    output->bit_depth[1] = source->bit_depth[1];
    output->bit_depth[2] = source->bit_depth[2];
  }
  output->pic_unit_size_on_disk = (imax(output->bit_depth[0], output->bit_depth[1]) > 8) ? 16 : 8;
  output->pic_unit_size_shift3 = output->pic_unit_size_on_disk >> 3;
  
  if (p_Inp->enable_32_pulldown)
  {
    source->frame_rate  = source->frame_rate  * 5 / 4;
    p_Inp->idr_period   = p_Inp->idr_period   * 5 / 4;
    p_Inp->intra_period = p_Inp->intra_period * 5 / 4;
    p_Inp->no_frames    = p_Inp->no_frames    * 5 / 4;
  }

  output->frame_rate = source->frame_rate / (p_Inp->frame_skip + 1);
  output->color_model = source->color_model;
  output->pixel_format = source->pixel_format;

  updateMaxValue(source);
  updateMaxValue(output);
}


/*!
 ***********************************************************************
 * \brief
 *    Parse the command line parameters and read the config files.
 * \param p_Vid
 *    VideoParameters structure for encoding
 * \param p_Inp
 *    InputParameters structure as input configuration
 * \param ac
 *    number of command line parameters
 * \param av
 *    command line parameters
 ***********************************************************************
 */
void Configure (VideoParameters *p_Vid, InputParameters *p_Inp, int ac, char *av[])
{
  char *content = NULL;
  int CLcount, ContentLen, NumberParams;
  char *filename=DEFAULTCONFIGFILENAME;

  if (ac==2)
  {
    if (0 == strncmp (av[1], "-v", 2))
    {
      printf("JM-" VERSION "\n");
      exit(0);
    }
    if (0 == strncmp (av[1], "-V", 2))
    {
      printf("JM " JM ": compiled " __DATE__ " " __TIME__ "\n");
#if ( IMGTYPE == 0 )
      printf("support for more than 8 bits/pel disabled\n");
#endif
#if ( ENABLE_FIELD_CTX == 0 )
      printf("CABAC field coding disabled\n");
#endif
#if ( ENABLE_HIGH444_CTX == 0 )
      printf("CABAC High 4:4:4 profile coding disabled\n");
#endif
      exit(0);
    }

    if (0 == strncmp (av[1], "-h", 2))
    {
      JMHelpExit();
    }
  }

  memset (&cfgparams, 0, sizeof (InputParameters));
  //Set default parameters.
  printf ("Setting Default Parameters...\n");
  InitParams(Map);

  // Process default config file
  CLcount = 1;

  if (ac>=3)
  {
    if (0 == strncmp (av[1], "-d", 2))
    {
      filename=av[2];
      CLcount = 3;
    }
    if (0 == strncmp (av[1], "-h", 2))
    {
      JMHelpExit();
    }
  }
  printf ("Parsing Configfile %s", filename);
  content = GetConfigFileContent (filename);
  if (NULL==content)
    error (errortext, 300);
  ParseContent (p_Inp, Map, content, (int) strlen(content));
  printf ("\n");
  free (content);

  // Parse the command line

  while (CLcount < ac)
  {
    if (0 == strncmp (av[CLcount], "-h", 2))
    {
      JMHelpExit();
    }

    if (0 == strncasecmp (av[CLcount], "-f", 2))  // A file parameter?
    {
      content = GetConfigFileContent (av[CLcount+1]);
      if (NULL==content)
        error (errortext, 300);
      printf ("Parsing Configfile %s", av[CLcount+1]);
      ParseContent (p_Inp, Map, content, (int) strlen (content));
      printf ("\n");
      free (content);
      CLcount += 2;
    } 
    else
    {
      if (0 == strncasecmp (av[CLcount], "-p", 2))  // A config change?
      {
        // Collect all data until next parameter (starting with -<x> (x is any character)),
        // put it into content, and parse content.

        ++CLcount;
        ContentLen = 0;
        NumberParams = CLcount;

        // determine the necessary size for content
        while (NumberParams < ac && av[NumberParams][0] != '-')
          ContentLen += (int) strlen (av[NumberParams++]);        // Space for all the strings
        ContentLen += 1000;                     // Additional 1000 bytes for spaces and \0s


        if ((content = malloc (ContentLen))==NULL) no_mem_exit("Configure: content");;
        content[0] = '\0';

        // concatenate all parameters identified before

        while (CLcount < NumberParams)
        {
          char *source = &av[CLcount][0];
          char *destin = &content[(int) strlen (content)];

          while (*source != '\0')
          {
            if (*source == '=')  // The Parser expects whitespace before and after '='
            {
              *destin++=' '; *destin++='='; *destin++=' ';  // Hence make sure we add it
            } 
            else
              *destin++=*source;
            source++;
          }
          *destin = '\0';
          CLcount++;
        }
        printf ("Parsing command line string '%s'", content);
        ParseContent (p_Inp, Map, content, (int) strlen(content));
        free (content);
        printf ("\n");
      }
      else
      {
        snprintf (errortext, ET_SIZE, "Error in command line, ac %d, around string '%s', missing -f or -p parameters?", CLcount, av[CLcount]);
        error (errortext, 300);
      }
    }
  }
  printf ("\n");

#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->num_of_views > 1)
  {
    printf ("Parsing Second View Configfile %s", p_Inp->View1ConfigName );
    content = GetConfigFileContent ( p_Inp->View1ConfigName );
    if (NULL==content)
      error (errortext, 300);
    ParseContent (p_Inp, MapView1, content, (int) strlen(content));
    printf ("\n");
    free (content);
  }
#endif

  PatchInp(p_Vid, p_Inp);

  cfgparams = *p_Inp;

  if (p_Inp->DisplayEncParams)
    DisplayEncoderParams(Map);
}




/*!
 ***********************************************************************
 * \brief
 *    Validates encoding parameters.
 * \return
 *    -1 for error
 ***********************************************************************
 */
static int TestEncoderParams(Mapping *Map, int bitdepth_qp_scale[3])
{
  int i = 0;

  while (Map[i].TokenName != NULL)
  {
    if (Map[i].param_limits == 1)
    {
      if (Map[i].Type == 0)
      {
        if ( * (int *) (Map[i].Place) < (int) Map[i].min_limit || * (int *) (Map[i].Place) > (int) Map[i].max_limit )
        {
          snprintf(errortext, ET_SIZE, "Error in input parameter %s. Check configuration file. Value should be in [%d, %d] range.", Map[i].TokenName, (int) Map[i].min_limit,(int)Map[i].max_limit );
          error (errortext, 400);
        }

      }
      else if (Map[i].Type == 2)
      {
        if ( * (double *) (Map[i].Place) < Map[i].min_limit || * (double *) (Map[i].Place) > Map[i].max_limit )
        {
          snprintf(errortext, ET_SIZE, "Error in input parameter %s. Check configuration file. Value should be in [%.2f, %.2f] range.", Map[i].TokenName,Map[i].min_limit ,Map[i].max_limit );
          error (errortext, 400);
        }
      }
    }
    else if (Map[i].param_limits == 2)
    {
      if (Map[i].Type == 0)
      {
        if ( * (int *) (Map[i].Place) < (int) Map[i].min_limit )
        {
          snprintf(errortext, ET_SIZE, "Error in input parameter %s. Check configuration file. Value should not be smaller than %d.", Map[i].TokenName, (int) Map[i].min_limit);
          error (errortext, 400);
        }
      }
      else if (Map[i].Type == 2)
      {
        if ( * (double *) (Map[i].Place) < Map[i].min_limit )
        {
          snprintf(errortext, ET_SIZE, "Error in input parameter %s. Check configuration file. Value should not be smaller than %2.f.", Map[i].TokenName,Map[i].min_limit);
          error (errortext, 400);
        }
      }
    }
    else if (Map[i].param_limits == 3) // Only used for QPs
    {
      
      if (Map[i].Type == 0)
      {
        int cur_qp = * (int *) (Map[i].Place);
        int min_qp = (int) (Map[i].min_limit - bitdepth_qp_scale[0]);
        int max_qp = (int) Map[i].max_limit;
        
        if (( cur_qp < min_qp ) || ( cur_qp > max_qp ))
        {
          snprintf(errortext, ET_SIZE, "Error in input parameter %s. Check configuration file. Value should be in [%d, %d] range.", Map[i].TokenName, min_qp, max_qp );
          error (errortext, 400);
        }
      }
    }

    i++;
  }
  return -1;
}



/*!
 ***********************************************************************
 * \brief
 *    Outputs encoding parameters.
 * \return
 *    -1 for error
 ***********************************************************************
 */
static int DisplayEncoderParams(Mapping *Map)
{
  int i = 0;

  printf("******************************************************\n");
  printf("*               Encoder Parameters                   *\n");
  printf("******************************************************\n");
  while (Map[i].TokenName != NULL)
  {
    if (Map[i].Type == 0)
      printf("Parameter %s = %d\n",Map[i].TokenName,* (int *) (Map[i].Place));
    else if (Map[i].Type == 1)
      printf("Parameter %s = ""%s""\n",Map[i].TokenName,(char *)  (Map[i].Place));
    else if (Map[i].Type == 2)
      printf("Parameter %s = %.2f\n",Map[i].TokenName,* (double *) (Map[i].Place));
      i++;
  }
  printf("******************************************************\n");
  return -1;
}

/*!
 ************************************************************************
 * \brief
 *    read the slice group configuration file. Returns without action
 *    if type is not 0, 2 or 6
 ************************************************************************
 */
void read_slice_group_info(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  FILE * sgfile=NULL;
  int i;
  int ret;
  unsigned int PicSizeInMapUnits;

  if ((p_Inp->slice_group_map_type != 0) && (p_Inp->slice_group_map_type != 2) && (p_Inp->slice_group_map_type != 6))
  {
    // nothing to do
    return;
  }

  // do we have a file name (not only NULL character)
  if ((int) strlen (p_Inp->SliceGroupConfigFileName) <= 1)
    error ("No slice group config file name specified", 500);
    
  // open file
  sgfile = fopen(p_Inp->SliceGroupConfigFileName,"r");

  if ( NULL==sgfile )
  {
    snprintf(errortext, ET_SIZE, "Error opening slice group file %s", p_Inp->SliceGroupConfigFileName);
    error (errortext, 500);
  }

  switch (p_Inp->slice_group_map_type)
  {
  case 0:
    p_Inp->run_length_minus1=(int *)malloc(sizeof(int)*(p_Inp->num_slice_groups_minus1+1));
    if ( NULL==p_Inp->run_length_minus1 )
    {
      fclose(sgfile);
      no_mem_exit("read_slice_group_info: p_Inp->run_length_minus1");
    }

    // each line contains one 'run_length_minus1' value
    for(i=0; i <= p_Inp->num_slice_groups_minus1;i++)
    {
      ret  = fscanf(sgfile,"%d",(p_Inp->run_length_minus1+i));
      if ( 1!=ret )
      {
        fclose(sgfile);
        snprintf(errortext, ET_SIZE, "Error while reading slice group config file (line %d)", i+1);
        error (errortext, 500);
      }
      // scan remaining line
      ret = fscanf(sgfile,"%*[^\n]");
    }
    break;

  case 2:
    // determine expected frame size in map units
    PicSizeInMapUnits = (p_Inp->output.width[0] >> 4) * (p_Inp->output.height[0] >> 4);
    if (p_Inp->MbInterlace||p_Inp->PicInterlace) 
      PicSizeInMapUnits >>= 1;

    p_Inp->top_left     = (unsigned *)malloc(sizeof(unsigned)*p_Inp->num_slice_groups_minus1);
    p_Inp->bottom_right = (unsigned *)malloc(sizeof(unsigned)*p_Inp->num_slice_groups_minus1);
    
    if (NULL==p_Inp->top_left)
    {
      fclose(sgfile);
      no_mem_exit("PatchInp: p_Inp->top_left");
    }
    
    if (NULL==p_Inp->bottom_right)
    {
      fclose(sgfile);
      no_mem_exit("PatchInp: p_Inp->bottom_right");
    }

    // every two lines contain 'top_left' and 'bottom_right' value
    for(i=0;i<p_Inp->num_slice_groups_minus1;i++)
    {
      ret = fscanf(sgfile,"%ud",(p_Inp->top_left+i));
      if ( 1!=ret )
      {
        fclose(sgfile);
        snprintf(errortext, ET_SIZE, "Error while reading slice group config file (line %d)", 2*i +1);
        error (errortext, 500);
      }
      if (p_Inp->top_left[i] > PicSizeInMapUnits)
      {
        fprintf(stderr, "Warning: slice group # %d top_left exceeds picture size (will be clipped)\n", i);
      }
      // scan remaining line
      ret = fscanf(sgfile,"%*[^\n]");
      ret = fscanf(sgfile,"%ud",(p_Inp->bottom_right+i));
      if ( 1!=ret )
      {
        fclose(sgfile);
        snprintf(errortext, ET_SIZE, "Error while reading slice group config file (line %d)", 2*i + 2);
        error (errortext, 500);
      }
      if (p_Inp->bottom_right[i] > PicSizeInMapUnits)
      {
        fprintf(stderr, "Warning: slice group # %d bottom_right exceeds picture size (will be clipped)\n", i);
      }
      // scan remaining line
      ret = fscanf(sgfile,"%*[^\n]");
    }
    break;
  case 6:
    {
      int tmp;
      int frame_mb_only;
      int mb_width, mb_height, mapunit_height;

      frame_mb_only = !(p_Inp->PicInterlace || p_Inp->MbInterlace);
      mb_width  = (p_Inp->output.width[0] + p_Vid->auto_crop_right)>>4;
      mb_height = (p_Inp->output.height[0] + p_Vid->auto_crop_bottom)>>4;
      mapunit_height = mb_height / (2-frame_mb_only);

      p_Inp->slice_group_id=(byte * ) malloc(sizeof(byte)*mapunit_height*mb_width);
      if (NULL==p_Inp->slice_group_id)
      {
        fclose(sgfile);
        no_mem_exit("PatchInp: p_Inp->slice_group_id");
      }

      // each line contains slice_group_id for one Macroblock
      for (i=0;i<mapunit_height*mb_width;i++)
      {
        ret = fscanf(sgfile,"%d", &tmp);
        p_Inp->slice_group_id[i]= (byte) tmp;
        if ( 1!=ret )
        {
          fclose(sgfile);
          snprintf(errortext, ET_SIZE, "Error while reading slice group config file (line %d)", i + 1);
          error (errortext, 500);
        }
        if ( *(p_Inp->slice_group_id+i) > p_Inp->num_slice_groups_minus1 )
        {
          fclose(sgfile);
          snprintf(errortext, ET_SIZE, "Error while reading slice group config file: slice_group_id not allowed (line %d)", i + 1);
          error (errortext, 500);
        }
        // scan remaining line
        ret = fscanf(sgfile,"%*[^\n]");
      }
    }
    break;
  default:
    // we should not get here
    error ("Wrong slice group type while reading config file", 500);
    break;
  }

  // close file again
  fclose(sgfile);
}

/*!
 ***********************************************************************
 * \brief
 *    Checks the input parameters for consistency.
 ***********************************************************************
 */
static void PatchInp (VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int i;
  int bitdepth_qp_scale[3];

  if (p_Inp->src_BitDepthRescale)
  {
    bitdepth_qp_scale [0] = 6*(p_Inp->output.bit_depth[0] - 8);
    bitdepth_qp_scale [1] = 6*(p_Inp->output.bit_depth[1] - 8);
    bitdepth_qp_scale [2] = 6*(p_Inp->output.bit_depth[2] - 8);
  }
  else
  {
    bitdepth_qp_scale [0] = 6*(p_Inp->source.bit_depth[0] - 8);
    bitdepth_qp_scale [1] = 6*(p_Inp->source.bit_depth[1] - 8);
    bitdepth_qp_scale [2] = 6*(p_Inp->source.bit_depth[2] - 8);
  }

  TestEncoderParams(Map, bitdepth_qp_scale);

  if (p_Inp->source.frame_rate == 0.0)
    p_Inp->source.frame_rate = (double) INIT_FRAME_RATE;

  ParseVideoType(&p_Inp->input_file1);
  ParseFrameNoFormatFromString (&p_Inp->input_file1);

#if (MVC_EXTENSION_ENABLE)
  if ( p_Inp->num_of_views == 2 )
  {
    ParseVideoType(&p_Inp->input_file2);
    ParseFrameNoFormatFromString (&p_Inp->input_file2);
  }
#endif

    if(p_Inp->source.bit_depth[0] >8)
    {
      if(!IMGTYPE)
      {
        error("IMGTYPE should be 1 when bitdepth is greater than 8", 600);
      }
    }
  // Read resolution from file name
  if (p_Inp->source.width[0] == 0 || p_Inp->source.height[0] == 0)
  {
    if (ParseSizeFromString (&p_Inp->input_file1, &(p_Inp->source.width[0]), &(p_Inp->source.height[0]), &(p_Inp->source.frame_rate)) == 0)
    {
      snprintf(errortext, ET_SIZE, "File name does not contain resolution information.");    
      error (errortext, 500);
    }
  }

#if (!ENABLE_FIELD_CTX)
  if ( (p_Inp->PicInterlace || p_Inp->MbInterlace) && p_Inp->symbol_mode )
  {
    snprintf(errortext, ET_SIZE, "Recompile with ENABLE_FIELD_CTX set to one to enable interlaced coding with CABAC.");    
    error (errortext, 500);
  }
#endif

#if (!ENABLE_HIGH444_CTX)
  if ( (p_Inp->ProfileIDC == FREXT_Hi444 || p_Inp->ProfileIDC == NO_PROFILE )&& p_Inp->symbol_mode )
  {
    snprintf(errortext, ET_SIZE, "Recompile with ENABLE_HIGH444_CTX set to one to enable the High 4:4:4 Profile with CABAC.");    
    error (errortext, 500);
  }
#endif

  // Currently to simplify things, lets copy everything (overwrites yuv_format)
  p_Inp->input_file1.format = p_Inp->source;


  if (p_Inp->idr_period && p_Inp->intra_delay && p_Inp->idr_period <= p_Inp->intra_delay)
  {
    snprintf(errortext, ET_SIZE, " IntraDelay cannot be larger than or equal to IDRPeriod.");
    error (errortext, 500);
  }

  // some contraints for IntraPeriod, IDRPeriod, EnableIDRGOP, and NumberBFrames
  if ( p_Inp->idr_period > 0 && (p_Inp->NumberBFrames + 1 + p_Inp->intra_delay) > p_Inp->idr_period )
  {
    snprintf(errortext, ET_SIZE, " Setting NumberBFrames = IDRPeriod - 1 - IntraDelay.");
    p_Inp->NumberBFrames = p_Inp->idr_period - 1 - p_Inp->intra_delay;
    if ( p_Inp->NumberBFrames < 0 )
    {
      p_Inp->NumberBFrames = p_Inp->idr_period - 1;
      p_Inp->intra_delay = 0;
    }
    if ( p_Inp->HierarchicalCoding )
    {
      p_Inp->HierarchicalCoding = 2;
    }
  }
  if ( p_Inp->intra_period > 0 && (p_Inp->NumberBFrames + 1 + p_Inp->intra_delay) > p_Inp->intra_period )
  {
    snprintf(errortext, ET_SIZE, " Setting NumberBFrames = IntraPeriod - 1 - IntraDelay.");
    p_Inp->NumberBFrames = p_Inp->intra_period - 1 - p_Inp->intra_delay;
    if ( p_Inp->NumberBFrames < 0 )
    {
      p_Inp->NumberBFrames = p_Inp->intra_period - 1;
      p_Inp->intra_delay = 0;
    }
    if ( p_Inp->HierarchicalCoding )
    {
      p_Inp->HierarchicalCoding = 2;
    }
  }
  // Added to avoid encoding problems with List 0 being empty
  if (p_Inp->intra_delay && p_Inp->NumberBFrames > 0 && p_Inp->EnableOpenGOP)
    p_Inp->PocMemoryManagement = 1;

  // Let us set up p_Inp->jumpd from frame_skip and NumberBFrames
  p_Inp->jumpd = (p_Inp->NumberBFrames + 1) * (p_Inp->frame_skip + 1) - 1;


  updateOutFormat(p_Inp);
  // SEI/VUI 3:2 pulldown support
  if ( p_Inp->SEIVUI32Pulldown && p_Inp->enable_32_pulldown )
  {
    snprintf(errortext, ET_SIZE, "SEIVUI32Pulldown and Enable32Pulldown cannot be set at the same time.");
    error (errortext, 500);
  }
  if ( p_Inp->SEIVUI32Pulldown )
  {    
    set_jm_vui_params( p_Inp );
  }

  if (p_Inp->no_frames == -1)
  {
    OpenFiles(&p_Inp->input_file1);
    get_number_of_frames (p_Inp, &p_Inp->input_file1);
    CloseFiles(&p_Inp->input_file1);
  }

  if (p_Inp->no_frames < 1)
  {      
    snprintf(errortext, ET_SIZE, "Not enough frames to encode (%d)", p_Inp->no_frames);
    error (errortext, 500);
  }
  
  // Check IntraProfile settings
  if (p_Inp->IntraProfile)
  {
    if(p_Inp->NumberBFrames)
    {
      snprintf(errortext, ET_SIZE, "Use of B slices not supported with Intra Profiles. Set NumberBFrames to 0.");
      error (errortext, 400);
    }

    if (p_Inp->PocMemoryManagement)
    {
      printf("Warning: PocMemoryManagement not supported for Intra Profiles. Process Disabled.\n");
      p_Inp->PocMemoryManagement = 0;
    }
    if (p_Inp->ReferenceReorder)
    {
      printf("Warning: ReferenceReorder not supported for Intra Profiles. Process Disabled.\n");
      p_Inp->ReferenceReorder = 0;
    }    
  }

  // Direct Mode consistency check
  if(p_Inp->NumberBFrames && p_Inp->direct_spatial_mv_pred_flag != DIR_SPATIAL && p_Inp->direct_spatial_mv_pred_flag != DIR_TEMPORAL)
  {
    snprintf(errortext, ET_SIZE, "Unsupported direct mode=%d, use TEMPORAL=0 or SPATIAL=1", p_Inp->direct_spatial_mv_pred_flag);
    error (errortext, 400);
  }

  if (p_Inp->PicInterlace>0 || p_Inp->MbInterlace>0)
  {
    if (p_Inp->directInferenceFlag==0)
      printf("\nWarning: DirectInferenceFlag set to 1 due to interlace coding.");
    p_Inp->directInferenceFlag = 1;
  }

  if(p_Inp->PicInterlace > 0 || p_Inp->MbInterlace > 0)
  {
    printf("\nWarning: WPMCPredicions is set to 0 due to interlace coding.\n");
    p_Inp->WPMCPrecision = 0;
  }
#if TRACE
  if ((int) strlen (p_Inp->TraceFile) > 0 && (p_Enc->p_trace = fopen(p_Inp->TraceFile,"w"))==NULL)
  {
    snprintf(errortext, ET_SIZE, "Error open file %s", p_Inp->TraceFile);
    error (errortext, 500);
  }
#endif

  
  
  if ((p_Inp->slice_mode == 1)&&(p_Inp->MbInterlace != 0))
  {
    if ((p_Inp->slice_argument & 0x01)!=0)
    {
      fprintf ( stderr, "Warning: slice border within macroblock pair. ");
      if (p_Inp->slice_argument > 1)
      {
        p_Inp->slice_argument--;
      }
      else
      {
        p_Inp->slice_argument++;
      }
      fprintf ( stderr, "Using %d MBs per slice.\n", p_Inp->slice_argument);
    }
  }  

  if (p_Inp->WPMCPrecision && (p_Inp->RDPictureDecision != 1 || p_Inp->GenerateMultiplePPS != 1) )
  {
    snprintf(errortext, ET_SIZE, "WPMCPrecision requires both RDPictureDecision=1 and GenerateMultiplePPS=1.\n");
    error (errortext, 400);
  }
  p_Inp->num_ref_frames_org = p_Inp->num_ref_frames;
  p_Inp->P_List0_refs_org[0] = p_Inp->P_List0_refs[0];
  p_Inp->B_List0_refs_org[0] = p_Inp->B_List0_refs[0];
  p_Inp->B_List1_refs_org[0] = p_Inp->B_List1_refs[0];
  p_Inp->P_List0_refs_org[1] = p_Inp->P_List0_refs[1];
  p_Inp->B_List0_refs_org[1] = p_Inp->B_List0_refs[1];
  p_Inp->B_List1_refs_org[1] = p_Inp->B_List1_refs[1];

  if (p_Inp->WPMCPrecision && p_Inp->WPMCPrecFullRef && p_Inp->num_ref_frames < 16 )
  {
    p_Inp->num_ref_frames++;
    if ( p_Inp->P_List0_refs[0] )
    {
      p_Inp->P_List0_refs[0]++;
    }
    else
    {
      p_Inp->P_List0_refs[0] = p_Inp->num_ref_frames;
    }    
    if ( p_Inp->B_List0_refs[0] )
    {
      p_Inp->B_List0_refs[0]++;
    }
    else
    {
      p_Inp->B_List0_refs[0] = p_Inp->num_ref_frames;
    }
    if ( p_Inp->B_List1_refs[0] )
    {
      p_Inp->B_List1_refs[0]++;
    }
    else
    {
      p_Inp->B_List1_refs[0] = p_Inp->num_ref_frames;
    }
    if ( p_Inp->SepViewInterSearch )
    {
      if ( p_Inp->P_List0_refs[1] )
      {
        p_Inp->P_List0_refs[1]++;
      }
      else
      {
        p_Inp->P_List0_refs[1] = p_Inp->num_ref_frames;
      }
      if ( p_Inp->B_List0_refs[1] )
      {
        p_Inp->B_List0_refs[1]++;
      }
      else
      {
        p_Inp->B_List0_refs[1] = p_Inp->num_ref_frames;
      }
      if ( p_Inp->B_List1_refs[1] )
      {
        p_Inp->B_List1_refs[1]++;
      }
      else
      {
        p_Inp->B_List1_refs[1] = p_Inp->num_ref_frames;
      }
    }
  }
  else if ( p_Inp->WPMCPrecision && p_Inp->WPMCPrecFullRef )
  {
    snprintf(errortext, ET_SIZE, "WPMCPrecFullRef requires NumberReferenceFrames < 16.\n");
    error (errortext, 400);
  }

  if (p_Inp->PicInterlace) 
  {
    if (p_Inp->ReferenceReorder == 2)
    {
      snprintf(errortext, ET_SIZE, "ReferenceReorder = 2 are not supported with field encoding\n");
      error (errortext, 400);
    }
    if (p_Inp->PocMemoryManagement == 2)
    {
      snprintf(errortext, ET_SIZE, "PocMemoryManagement = 2 is not supported with field encoding\n");
      error (errortext, 400);
    }
  }

  if ( p_Inp->ReferenceReorder && p_Inp->MbInterlace )
  {
    snprintf(errortext, ET_SIZE, "ReferenceReorder is not supported with MBAFF\n");
    error (errortext, 400);
  }

  if ( p_Inp->SetFirstAsLongTerm && ( p_Inp->ReferenceReorder != 0 ) )
  {
    printf("SetFirstAsLongTerm is set. ReferenceReorder is not supported and therefore disabled. \n");
    p_Inp->ReferenceReorder = 0;
  }

  if (p_Inp->PocMemoryManagement && p_Inp->MbInterlace )
  {
    snprintf(errortext, ET_SIZE, "PocMemoryManagement is not supported with MBAFF\n");
    error (errortext, 400);
  }

  if(p_Inp->MbInterlace && p_Inp->RDPictureDecision && p_Inp->GenerateMultiplePPS)
  {
    snprintf(errortext, ET_SIZE, "RDPictureDecision+GenerateMultiplePPS not supported with MBAFF. RDPictureDecision therefore disabled\n");
    p_Inp->RDPictureDecision = 0;
  }

  if ((!p_Inp->rdopt)&&(p_Inp->MbInterlace==2))
  {
    snprintf(errortext, ET_SIZE, "MB AFF is not compatible with non-rd-optimized coding.");
    error (errortext, 500);
  }

  // Tian Dong: May 31, 2002
  // The number of frames in one sub-seq in enhanced layer should not exceed
  // the number of reference frame number.
  if ( p_Inp->NumFramesInELSubSeq > p_Inp->num_ref_frames || p_Inp->NumFramesInELSubSeq < 0 )
  {
    snprintf(errortext, ET_SIZE, "NumFramesInELSubSeq (%d) is out of range [0,%d).", p_Inp->NumFramesInELSubSeq, p_Inp->num_ref_frames);
    error (errortext, 500);
  }
  // Tian Dong: Enhanced GOP is not supported in bitstream mode. September, 2002
  if ( p_Inp->NumFramesInELSubSeq > 0 )
  {
    snprintf(errortext, ET_SIZE, "Enhanced GOP is not properly supported yet.");
    error (errortext, 500);
  }
  // Tian Dong (Sept 2002)
  // The AFF is not compatible with spare picture for the time being.
  if ((p_Inp->PicInterlace || p_Inp->MbInterlace) && p_Inp->SparePictureOption == TRUE)
  {
    snprintf(errortext, ET_SIZE, "AFF is not compatible with spare picture.");
    error (errortext, 500);
  }

  // Only the RTP mode is compatible with spare picture for the time being.
  if (p_Inp->of_mode != PAR_OF_RTP && p_Inp->SparePictureOption == TRUE)
  {
    snprintf(errortext, ET_SIZE, "Only RTP output mode is compatible with spare picture features.");
    error (errortext, 500);
  }

  if( (p_Inp->WeightedPrediction > 0 || p_Inp->WeightedBiprediction > 0) && (p_Inp->MbInterlace))
  {
    snprintf(errortext, ET_SIZE, "Weighted prediction coding is not supported for MB AFF currently.");
    error (errortext, 500);
  }
  if( (p_Inp->WeightedPrediction > 0 || p_Inp->WeightedBiprediction > 0) && (p_Inp->yuv_format==3))
  {
    snprintf(errortext, ET_SIZE, "Weighted prediction coding is not supported for 4:4:4 encoding.");
    error (errortext, 500);
  }
  if ( p_Inp->NumFramesInELSubSeq > 0 && p_Inp->WeightedPrediction > 0)
  {
    snprintf(errortext, ET_SIZE, "Enhanced GOP is not supported in weighted prediction coding mode yet.");
    error (errortext, 500);
  }

  // Rate control
  if(p_Inp->RCEnable)
  {
    if ( p_Inp->RCUpdateMode == RC_MODE_1 && 
      !( (p_Inp->intra_period == 1 || p_Inp->idr_period == 1 || p_Inp->BRefPictures == 2 ) && !p_Inp->NumberBFrames ) )
    {
      snprintf(errortext, ET_SIZE, "Use RCUpdateMode = 1 only for all intra or all B-slice coding.");
      error (errortext, 500);
    }
    if ( (p_Inp->RCUpdateMode == RC_MODE_0 || p_Inp->RCUpdateMode == RC_MODE_2 || p_Inp->RCUpdateMode == RC_MODE_3) && 
      (p_Inp->PReplaceBSlice && p_Inp->NumberBFrames ) )
    {
      snprintf(errortext, ET_SIZE, "PReplaceBSlice is not supported with RCUpdateMode=0,2,3.");
      error (errortext, 500);
    }

    if ( p_Inp->BRefPictures == 2 && p_Inp->intra_period == 0 && p_Inp->RCUpdateMode != RC_MODE_1 )
    {
      snprintf(errortext, ET_SIZE, "Use RCUpdateMode = 1 for all B-slice coding.");
      error (errortext, 500);
    }

    if ( p_Inp->HierarchicalCoding && p_Inp->RCUpdateMode != RC_MODE_2 && p_Inp->RCUpdateMode != RC_MODE_3 )
    {
      snprintf(errortext, ET_SIZE, "Use RCUpdateMode = 2 or 3 for hierarchical B-picture coding.");
      error (errortext, 500);
    }
    if ( (p_Inp->RCUpdateMode != RC_MODE_1) && (p_Inp->intra_period == 1) )
    {
      snprintf(errortext, ET_SIZE, "Use RCUpdateMode = 1 for all intra coding.");
      error (errortext, 500);
    }
  }

  if ((p_Inp->NumberBFrames)&&(p_Inp->BRefPictures)&&(p_Inp->idr_period)&&(p_Inp->pic_order_cnt_type!=0))
  {
    error("Stored B pictures combined with IDR pictures only supported in Picture Order Count type 0\n",-1000);
  }

  if( !p_Inp->direct_spatial_mv_pred_flag && p_Inp->num_ref_frames<2 && p_Inp->NumberBFrames >0)
    error("temporal direct needs at least 2 ref frames\n",-1000);

#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->SepViewInterSearch && p_Inp->SearchMode[1] == FAST_FULL_SEARCH && p_Inp->MEErrorMetric[F_PEL] > ERROR_SSE)
  {
    snprintf(errortext, ET_SIZE, "\nOnly SAD and SSE distortion computation supported with Fast Full Search.");
    error (errortext, 500);
  }
  else
#endif
  if (p_Inp->SearchMode[0] == FAST_FULL_SEARCH && p_Inp->MEErrorMetric[F_PEL] > ERROR_SSE)
  {
    snprintf(errortext, ET_SIZE, "\nOnly SAD and SSE distortion computation supported with Fast Full Search.");
    error (errortext, 500);
  }

  if (p_Inp->rdopt == 0)
  {
    if (p_Inp->DisableSubpelME[0] || p_Inp->DisableSubpelME[1])
    {
      if (p_Inp->MEErrorMetric[F_PEL] != p_Inp->ModeDecisionMetric)
      {
        snprintf(errortext, ET_SIZE, "\nLast refinement level (FPel) distortion not the same as Mode decision distortion.\nPlease update MEDistortionFPel (%d) and/or  MDDistortion(%d).", p_Inp->MEErrorMetric[F_PEL], p_Inp->ModeDecisionMetric);
        error (errortext, 500);
      }
    }
    else if (p_Inp->MEErrorMetric[Q_PEL] != p_Inp->ModeDecisionMetric)
    {
      snprintf(errortext, ET_SIZE, "\nLast refinement level (QPel) distortion not the same as Mode decision distortion.\nPlease update MEDistortionQPel (%d) and/or  MDDistortion(%d).", p_Inp->MEErrorMetric[Q_PEL], p_Inp->ModeDecisionMetric);
      error (errortext, 500);
    }
  }
  // frext
  if(p_Inp->Transform8x8Mode && p_Inp->sp_periodicity /*SP-frames*/)
  {
    snprintf(errortext, ET_SIZE, "\nThe new 8x8 mode is not implemented for sp-frames.");
    error (errortext, 500);
  }

  if (p_Inp->DisableIntra4x4 == 1 && p_Inp->DisableIntra16x16 == 1 && p_Inp->EnableIPCM == 0 && p_Inp->Transform8x8Mode == 0)
  {
    snprintf(errortext, ET_SIZE, "\nAt least one intra prediction mode needs to be enabled.");
    error (errortext, 500);
  }

  if (p_Inp->NumberBFrames && ((p_Inp->BiPredMotionEstimation) && (p_Inp->search_range[0] < p_Inp->BiPredMESearchRange[0])))
  {
    snprintf(errortext, ET_SIZE, "\nBiPredMESearchRange must be smaller or equal SearchRange.");
    error (errortext, 500);
  }
#if (MVC_EXTENSION_ENABLE)
  if ( p_Inp->SepViewInterSearch )
  {
    if (p_Inp->NumberBFrames && ((p_Inp->BiPredMotionEstimation) && (p_Inp->search_range[1] < p_Inp->BiPredMESearchRange[1])))
    {
      snprintf(errortext, ET_SIZE, "\nView1BiPredMESearchRange must be smaller or equal View1SearchRange.");
      error (errortext, 500);
    }
  }
#endif

  if (p_Inp->BiPredMotionEstimation)
  {
    p_Inp->BiPredMotionEstimation = 0;
    for (i = 0 ; i < 4; i++)
      p_Inp->BiPredMotionEstimation |= p_Inp->BiPredSearch[i];
  }
  else
  {
    for (i = 0 ; i < 4; i++)
      p_Inp->BiPredSearch[i] = 0;
  }

  // check consistency
  if ( p_Inp->ChromaMEEnable && !(p_Inp->ChromaMCBuffer) ) 
  {
    snprintf(errortext, ET_SIZE, "\nChromaMCBuffer must be set to 1 if ChromaMEEnable is set.");
    error (errortext, 500);
  }

  if ( p_Inp->ChromaMEEnable && p_Inp->yuv_format ==  YUV400) 
  {
    fprintf(stderr, "Warning: ChromaMEEnable cannot be used with monochrome color format, disabling ChromaMEEnable.\n");
    p_Inp->ChromaMEEnable = FALSE;
  }

  if ( (p_Inp->ChromaMCBuffer == 0) && (( p_Inp->yuv_format ==  YUV444) && (!p_Inp->separate_colour_plane_flag)) )
  {
    fprintf(stderr, "Warning: Enabling ChromaMCBuffer for 4:4:4 combined color coding.\n");
    p_Inp->ChromaMCBuffer = 1;
  }

  if (p_Inp->EnableOpenGOP && p_Inp->ReferenceReorder != 1)
  {
    printf("If OpenGOP is enabled than ReferenceReorder is set to 1. \n");
  }

  if (p_Inp->EnableOpenGOP)
    p_Inp->ReferenceReorder = 1;

  if (p_Inp->SearchMode[0] != EPZS 
#if (MVC_EXTENSION_ENABLE)
    && (!p_Inp->SepViewInterSearch || p_Inp->SearchMode[1] != EPZS )
#endif
    )
    p_Inp->EPZSSubPelGrid = 0;

  if (p_Inp->redundant_pic_flag)
  {
    if (p_Inp->PicInterlace || p_Inp->MbInterlace)
    {
      snprintf(errortext, ET_SIZE, "Redundant pictures cannot be used with interlaced tools.");
      error (errortext, 500);
    }
    if (p_Inp->RDPictureDecision)
    {
      snprintf(errortext, ET_SIZE, "Redundant pictures cannot be used with RDPictureDecision.");
      error (errortext, 500);
    }
    if (p_Inp->NumberBFrames)
    {
      snprintf(errortext, ET_SIZE, "Redundant pictures cannot be used with B frames.");
      error (errortext, 500);
    }
    if (p_Inp->PrimaryGOPLength < (1 << p_Inp->NumRedundantHierarchy))
    {
      snprintf(errortext, ET_SIZE, "PrimaryGOPLength must be equal or greater than 2^NumRedundantHierarchy.");
      error (errortext, 500);
    }
    if (p_Inp->num_ref_frames < p_Inp->PrimaryGOPLength)
    {
      snprintf(errortext, ET_SIZE, "NumberReferenceFrames must be greater than or equal to PrimaryGOPLength.");
      error (errortext, 500);
    }
  }

  if (p_Inp->num_ref_frames == 1 && p_Inp->NumberBFrames)
  {
    fprintf( stderr, "\nWarning: B slices used but only one reference allocated within reference buffer.\n");
    fprintf( stderr, "         Performance may be considerably compromised! \n");
    fprintf( stderr, "         2 or more references recommended for use with B slices.\n");
  }
  if ((p_Inp->HierarchicalCoding || p_Inp->BRefPictures) && p_Inp->NumberBFrames)
  {
    fprintf( stderr, "\nWarning: Hierarchical coding or Referenced B slices used.\n");
    fprintf( stderr, "         Make sure that you have allocated enough references\n");
    fprintf( stderr, "         in reference buffer to achieve best performance.\n");
  }

  if (p_Inp->FastMDEnable == 0)
  {
    p_Inp->FastIntraMD = 0;
    p_Inp->FastIntra16x16 = 0;
    p_Inp->FastIntra4x4 = 0;
    p_Inp->FastIntra8x8 = 0;
    p_Inp->FastIntraChroma = 0;
  }

  // Init RDOQuant
  if (p_Inp->UseRDOQuant == 1)
  {
    if (p_Inp->rdopt == 0)
    {
      snprintf(errortext, ET_SIZE, "RDO Quantization not supported with low complexity RDO.");
      error (errortext, 500);
    }

    if (p_Inp->MbInterlace != 0)
    {
      printf("RDO Quantization currently not supported with MBAFF. Option disabled.\n");
      p_Inp->UseRDOQuant = 0;
      p_Inp->RDOQ_QP_Num = 1;
      p_Inp->RDOQ_CP_MV = 0;
      p_Inp->RDOQ_CP_Mode = 0;
    }
    else
    {
      p_Inp->AdaptiveRounding = 0;
      printf("AdaptiveRounding is disabled when RDO Quantization is used\n");
      if (p_Inp->RDOQ_QP_Num < 2)
      {
        p_Inp->RDOQ_CP_MV = 0;
        p_Inp->RDOQ_CP_Mode = 0;
      }
    }
  }
  else
  {
    p_Inp->RDOQ_QP_Num = 1;
    p_Inp->RDOQ_CP_MV = 0;
    p_Inp->RDOQ_CP_Mode = 0;
  }

  if(p_Inp->num_slice_groups_minus1 > 0 && (p_Inp->GenerateMultiplePPS ==1 && p_Inp->RDPictureDecision == 1))
  {
    printf("Warning: Weighted Prediction may not function correctly for multiple slices\n"); 
  }

#if KEEP_B_SAME_LIST
  if ( p_Inp->BIdenticalList > 0 && p_Inp->ReferenceReorder != 1 )
  {
    snprintf(errortext, ET_SIZE, "Set two lists of B picture identical can only be used when ReferenceReorder is 1");
    error (errortext, 500);
  }
#endif

#if CRA
  if ( p_Inp->useCRA == 1 && p_Inp->intra_period == 0 )
  {
    printf("Warning: CRA can only be used for IntraPeriod > 0, set CRA to 0\n");
    p_Inp->useCRA = 0;
  }
  if ( p_Inp->useCRA == 1 && p_Inp->LowDelay == 1 )
  {
    snprintf(errortext, ET_SIZE, "CRA cannot be used when LowDelay is 1");
    error (errortext, 500);
  }
  if ( (p_Inp->useCRA == 1) && (p_Inp->PicInterlace > 0) )
  {
    snprintf(errortext, ET_SIZE, "HM5 like configuration options (CRA) cannot be used together with PicInterlace");
    error (errortext, 500);
  }
#endif

#if HM50_LIKE_MMCO
  if ( p_Inp->HM50RefStructure == 1 )
  {
    printf("Warning: HM50RefStructure can only be used for random access case with GOP size equal to 8 and HM-5.0 coding order\n");
    printf("         If it used for other coding structure, the performance may loss\n");
  }
  if ( p_Inp->HM50RefStructure == 1 && p_Inp->NumberBFrames != 7 )
  {
    printf("Warning: HM50RefStructure can only be used for random access case with NumberBFrames equal to 7, turn it off\n");
    p_Inp->HM50RefStructure = 0;
  }
  if ( p_Inp->HM50RefStructure == 1 && p_Inp->LowDelay == 1 )
  {
    printf("Warning: HM50RefStructure cannot be used for low delay, turn it off\n");
    p_Inp->HM50RefStructure = 0;
  }
  if ( (p_Inp->HM50RefStructure == 1) && (p_Inp->PicInterlace > 0) )
  {
    snprintf(errortext, ET_SIZE, "HM5 like configuration options (HM50RefStructure) cannot be used together with PicInterlace");
    error (errortext, 500);
  }
#endif

#if LD_REF_SETTING
  if ( p_Inp->LDRefSetting == 1 )
  {
    printf("Warning: LDRefSetting can only be used for low delay case with GOP size equal to 4 and 4 reference frames as HM-5.0 low delay\n");
    printf("         If other GOP size or number of reference frames are used, the performance may be loss with LDRefSetting\n");
  }
  if ( p_Inp->LDRefSetting == 1 && p_Inp->LowDelay == 0 )
  {
    snprintf(errortext, ET_SIZE, "LDRefSetting can only be used when LowDelay is 1");
    error (errortext, 500);
  }
  if ( (p_Inp->LDRefSetting == 1) && (p_Inp->PicInterlace > 0) )
  {
    snprintf(errortext, ET_SIZE, "HM5 like configuration options (LDRefSetting) cannot be used together with PicInterlace");
    error (errortext, 500);
  }
#endif
#if B0_MORE_REF
  if ( (p_Inp->BLevel0MoreRef == 1) && (p_Inp->PicInterlace > 0) )
  {
    snprintf(errortext, ET_SIZE, "HM5 like configuration options (BLevel0MoreRef) cannot be used together with PicInterlace");
    error (errortext, 500);
  }
#endif
#if KEEP_B_SAME_LIST
  if ( (p_Inp->BIdenticalList == 1) && (p_Inp->PicInterlace > 0) )
  {
    snprintf(errortext, ET_SIZE, "HM5 like configuration options (BIdenticalList) cannot be used together with PicInterlace");
    error (errortext, 500);
  }
#endif

  profile_check(p_Inp);

  if(!p_Inp->RDPictureDecision)
  {
    p_Inp->RDPictureMaxPassISlice = 1;
    p_Inp->RDPictureMaxPassPSlice = 1;
    p_Inp->RDPictureMaxPassBSlice = 1;
    p_Inp->RDPictureDeblocking    = 0;
    p_Inp->RDPictureDirectMode    = 0;
    p_Inp->RDPictureFrameQPPSlice = 0;
    p_Inp->RDPictureFrameQPBSlice = 0;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Set VUI time scale and number of clock ticks given input frame rate
 ************************************************************************
*/

static void SetVUIScaleAndTicks(InputParameters *p_Inp, double frame_rate)
{
  double frame_rate_integer = (double)ceil( frame_rate );  

  if ( frame_rate_integer != frame_rate )
  {    
    // frame rate is a floating-point number
    // check whether multiplying it with 1001/1000=1.001 brings it closer to frame_rate_integer
    double new_frame_rate = 1.001 * frame_rate;

    if ( dabs( new_frame_rate - frame_rate_integer ) < dabs( frame_rate - frame_rate_integer ) )
    {
      p_Inp->VUI.num_units_in_tick = 1001;
      p_Inp->VUI.time_scale        = (int)floor( frame_rate_integer * (1000 << 1) ); // two ticks per frame    
    }
    else
    {
      p_Inp->VUI.num_units_in_tick = 1000;
      p_Inp->VUI.time_scale        = (int)floor( frame_rate * (p_Inp->VUI.num_units_in_tick << 1) + 0.5 ); // two ticks per frame    
    }
  }
  else
  {
    // frame rate is an integer
    p_Inp->VUI.num_units_in_tick = 1000;
    p_Inp->VUI.time_scale        = (int)floor( frame_rate * (p_Inp->VUI.num_units_in_tick << 1) ); // two ticks per frame    
  }
}

