
/*!
 *************************************************************************************
 * \file q_matrix.c
 *
 * \brief
 *    read q_matrix parameters from input file: q_matrix.cfg
 *
 *************************************************************************************
 */

#include "global.h"
#include "memalloc.h"
#include "q_matrix.h"

extern char *GetConfigFileContent (char *Filename, int error_type);

#define MAX_ITEMS_TO_PARSE  1000

static const int quant_coef[6][4][4] = {
  {{13107, 8066,13107, 8066},{ 8066, 5243, 8066, 5243},{13107, 8066,13107, 8066},{ 8066, 5243, 8066, 5243}},
  {{11916, 7490,11916, 7490},{ 7490, 4660, 7490, 4660},{11916, 7490,11916, 7490},{ 7490, 4660, 7490, 4660}},
  {{10082, 6554,10082, 6554},{ 6554, 4194, 6554, 4194},{10082, 6554,10082, 6554},{ 6554, 4194, 6554, 4194}},
  {{ 9362, 5825, 9362, 5825},{ 5825, 3647, 5825, 3647},{ 9362, 5825, 9362, 5825},{ 5825, 3647, 5825, 3647}},
  {{ 8192, 5243, 8192, 5243},{ 5243, 3355, 5243, 3355},{ 8192, 5243, 8192, 5243},{ 5243, 3355, 5243, 3355}},
  {{ 7282, 4559, 7282, 4559},{ 4559, 2893, 4559, 2893},{ 7282, 4559, 7282, 4559},{ 4559, 2893, 4559, 2893}}
};

const int dequant_coef[6][4][4] = {
  {{10, 13, 10, 13},{ 13, 16, 13, 16},{10, 13, 10, 13},{ 13, 16, 13, 16}},
  {{11, 14, 11, 14},{ 14, 18, 14, 18},{11, 14, 11, 14},{ 14, 18, 14, 18}},
  {{13, 16, 13, 16},{ 16, 20, 16, 20},{13, 16, 13, 16},{ 16, 20, 16, 20}},
  {{14, 18, 14, 18},{ 18, 23, 18, 23},{14, 18, 14, 18},{ 18, 23, 18, 23}},
  {{16, 20, 16, 20},{ 20, 25, 20, 25},{16, 20, 16, 20},{ 20, 25, 20, 25}},
  {{18, 23, 18, 23},{ 23, 29, 23, 29},{18, 23, 18, 23},{ 23, 29, 23, 29}}
};

static const int quant_coef8[6][8][8] =
{
  {
    {13107, 12222,  16777,  12222,  13107,  12222,  16777,  12222},
    {12222, 11428,  15481,  11428,  12222,  11428,  15481,  11428},
    {16777, 15481,  20972,  15481,  16777,  15481,  20972,  15481},
    {12222, 11428,  15481,  11428,  12222,  11428,  15481,  11428},
    {13107, 12222,  16777,  12222,  13107,  12222,  16777,  12222},
    {12222, 11428,  15481,  11428,  12222,  11428,  15481,  11428},
    {16777, 15481,  20972,  15481,  16777,  15481,  20972,  15481},
    {12222, 11428,  15481,  11428,  12222,  11428,  15481,  11428}
  },
  {
    {11916, 11058,  14980,  11058,  11916,  11058,  14980,  11058},
    {11058, 10826,  14290,  10826,  11058,  10826,  14290,  10826},
    {14980, 14290,  19174,  14290,  14980,  14290,  19174,  14290},
    {11058, 10826,  14290,  10826,  11058,  10826,  14290,  10826},
    {11916, 11058,  14980,  11058,  11916,  11058,  14980,  11058},
    {11058, 10826,  14290,  10826,  11058,  10826,  14290,  10826},
    {14980, 14290,  19174,  14290,  14980,  14290,  19174,  14290},
    {11058, 10826,  14290,  10826,  11058,  10826,  14290,  10826}
  },
  {
    {10082, 9675, 12710,  9675, 10082,  9675, 12710,  9675},
    {9675,  8943, 11985,  8943, 9675, 8943, 11985,  8943},
    {12710, 11985,  15978,  11985,  12710,  11985,  15978,  11985},
    {9675,  8943, 11985,  8943, 9675, 8943, 11985,  8943},
    {10082, 9675, 12710,  9675, 10082,  9675, 12710,  9675},
    {9675,  8943, 11985,  8943, 9675, 8943, 11985,  8943},
    {12710, 11985,  15978,  11985,  12710,  11985,  15978,  11985},
    {9675,  8943, 11985,  8943, 9675, 8943, 11985,  8943}
  },
  {
    {9362,  8931, 11984,  8931, 9362, 8931, 11984,  8931},
    {8931,  8228, 11259,  8228, 8931, 8228, 11259,  8228},
    {11984, 11259,  14913,  11259,  11984,  11259,  14913,  11259},
    {8931,  8228, 11259,  8228, 8931, 8228, 11259,  8228},
    {9362,  8931, 11984,  8931, 9362, 8931, 11984,  8931},
    {8931,  8228, 11259,  8228, 8931, 8228, 11259,  8228},
    {11984, 11259,  14913,  11259,  11984,  11259,  14913,  11259},
    {8931,  8228, 11259,  8228, 8931, 8228, 11259,  8228}
  },
  {
    {8192,  7740, 10486,  7740, 8192, 7740, 10486,  7740},
    {7740,  7346, 9777, 7346, 7740, 7346, 9777, 7346},
    {10486, 9777, 13159,  9777, 10486,  9777, 13159,  9777},
    {7740,  7346, 9777, 7346, 7740, 7346, 9777, 7346},
    {8192,  7740, 10486,  7740, 8192, 7740, 10486,  7740},
    {7740,  7346, 9777, 7346, 7740, 7346, 9777, 7346},
    {10486, 9777, 13159,  9777, 10486,  9777, 13159,  9777},
    {7740,  7346, 9777, 7346, 7740, 7346, 9777, 7346}
  },
  {
    {7282,  6830, 9118, 6830, 7282, 6830, 9118, 6830},
    {6830,  6428, 8640, 6428, 6830, 6428, 8640, 6428},
    {9118,  8640, 11570,  8640, 9118, 8640, 11570,  8640},
    {6830,  6428, 8640, 6428, 6830, 6428, 8640, 6428},
    {7282,  6830, 9118, 6830, 7282, 6830, 9118, 6830},
    {6830,  6428, 8640, 6428, 6830, 6428, 8640, 6428},
    {9118,  8640, 11570,  8640, 9118, 8640, 11570,  8640},
    {6830,  6428, 8640, 6428, 6830, 6428, 8640, 6428}
  }
};



static const int dequant_coef8[6][8][8] =
{
  {
    {20,  19, 25, 19, 20, 19, 25, 19},
    {19,  18, 24, 18, 19, 18, 24, 18},
    {25,  24, 32, 24, 25, 24, 32, 24},
    {19,  18, 24, 18, 19, 18, 24, 18},
    {20,  19, 25, 19, 20, 19, 25, 19},
    {19,  18, 24, 18, 19, 18, 24, 18},
    {25,  24, 32, 24, 25, 24, 32, 24},
    {19,  18, 24, 18, 19, 18, 24, 18}
  },
  {
    {22,  21, 28, 21, 22, 21, 28, 21},
    {21,  19, 26, 19, 21, 19, 26, 19},
    {28,  26, 35, 26, 28, 26, 35, 26},
    {21,  19, 26, 19, 21, 19, 26, 19},
    {22,  21, 28, 21, 22, 21, 28, 21},
    {21,  19, 26, 19, 21, 19, 26, 19},
    {28,  26, 35, 26, 28, 26, 35, 26},
    {21,  19, 26, 19, 21, 19, 26, 19}
  },
  {
    {26,  24, 33, 24, 26, 24, 33, 24},
    {24,  23, 31, 23, 24, 23, 31, 23},
    {33,  31, 42, 31, 33, 31, 42, 31},
    {24,  23, 31, 23, 24, 23, 31, 23},
    {26,  24, 33, 24, 26, 24, 33, 24},
    {24,  23, 31, 23, 24, 23, 31, 23},
    {33,  31, 42, 31, 33, 31, 42, 31},
    {24,  23, 31, 23, 24, 23, 31, 23}
  },
  {
    {28,  26, 35, 26, 28, 26, 35, 26},
    {26,  25, 33, 25, 26, 25, 33, 25},
    {35,  33, 45, 33, 35, 33, 45, 33},
    {26,  25, 33, 25, 26, 25, 33, 25},
    {28,  26, 35, 26, 28, 26, 35, 26},
    {26,  25, 33, 25, 26, 25, 33, 25},
    {35,  33, 45, 33, 35, 33, 45, 33},
    {26,  25, 33, 25, 26, 25, 33, 25}
  },
  {
    {32,  30, 40, 30, 32, 30, 40, 30},
    {30,  28, 38, 28, 30, 28, 38, 28},
    {40,  38, 51, 38, 40, 38, 51, 38},
    {30,  28, 38, 28, 30, 28, 38, 28},
    {32,  30, 40, 30, 32, 30, 40, 30},
    {30,  28, 38, 28, 30, 28, 38, 28},
    {40,  38, 51, 38, 40, 38, 51, 38},
    {30,  28, 38, 28, 30, 28, 38, 28}
  },
  {
    {36,  34, 46, 34, 36, 34, 46, 34},
    {34,  32, 43, 32, 34, 32, 43, 32},
    {46,  43, 58, 43, 46, 43, 58, 43},
    {34,  32, 43, 32, 34, 32, 43, 32},
    {36,  34, 46, 34, 36, 34, 46, 34},
    {34,  32, 43, 32, 34, 32, 43, 32},
    {46,  43, 58, 43, 46, 43, 58, 43},
    {34,  32, 43, 32, 34, 32, 43, 32}
  }
};


int matrix4x4_check[6] = {0, 0, 0, 0, 0, 0};
int matrix8x8_check[6] = {0, 0, 0, 0, 0, 0};

static const char MatrixType4x4[6][20] =
{
  "INTRA4X4_LUMA",
  "INTRA4X4_CHROMAU",
  "INTRA4X4_CHROMAV",
  "INTER4X4_LUMA",
  "INTER4X4_CHROMAU",
  "INTER4X4_CHROMAV"
};

static const char MatrixType8x8[6][20] =
{
  "INTRA8X8_LUMA",
  "INTER8X8_LUMA",
  "INTRA8X8_CHROMAU",  // only for 4:4:4
  "INTER8X8_CHROMAU",  // only for 4:4:4
  "INTRA8X8_CHROMAV",  // only for 4:4:4
  "INTER8X8_CHROMAV"   // only for 4:4:4
};


static const short Quant_intra_default[16] =
{
 6,13,20,28,
13,20,28,32,
20,28,32,37,
28,32,37,42
};

static const short Quant_inter_default[16] =
{
10,14,20,24,
14,20,24,27,
20,24,27,30,
24,27,30,34
};

static const short Quant8_intra_default[64] =
{
 6,10,13,16,18,23,25,27,
10,11,16,18,23,25,27,29,
13,16,18,23,25,27,29,31,
16,18,23,25,27,29,31,33,
18,23,25,27,29,31,33,36,
23,25,27,29,31,33,36,38,
25,27,29,31,33,36,38,40,
27,29,31,33,36,38,40,42
};

static const short Quant8_inter_default[64] =
{
 9,13,15,17,19,21,22,24,
13,13,17,19,21,22,24,25,
15,17,19,21,22,24,25,27,
17,19,21,22,24,25,27,28,
19,21,22,24,25,27,28,30,
21,22,24,25,27,28,30,32,
22,24,25,27,28,30,32,33,
24,25,27,28,30,32,33,35
};


/*!
 ***********************************************************************
 * \brief
 *    Check the parameter name.
 * \param s
 *    parameter name string
 * \param type
 *    4x4 or 8x8 matrix type
 * \return
 *    the index number if the string is a valid parameter name,         \n
 *    -1 for error
 ***********************************************************************
 */
static int CheckParameterName (char *s, int *type)
{
  int i = 0;

  *type = 0;
  while ((MatrixType4x4[i] != NULL) && (i<6))
  {
    if (0==strcmp (MatrixType4x4[i], s))
      return i;
    else
      i++;
  }

  i = 0;
  *type = 1;
  while ((MatrixType8x8[i] != NULL) && (i<6))
  {
    if (0==strcmp (MatrixType8x8[i], s))
      return i;
    else
      i++;
  }

  return -1;
}

/*!
 ***********************************************************************
 * \brief
 *    Parse the Q matrix values read from cfg file.
 * \param p_Vid
 *    current video parameters
 * \param buf
 *    buffer to be parsed
 * \param bufsize
 *    buffer size of buffer
 ***********************************************************************
 */
static void ParseMatrix (VideoParameters *p_Vid, char *buf, int bufsize)
{
  ScaleParameters *p_QScale = p_Vid->p_QScale;
  char *items[MAX_ITEMS_TO_PARSE] = {NULL};
  int MapIdx;
  int item = 0;
  int InString = 0, InItem = 0;
  char *p = buf;
  char *bufend = &buf[bufsize];
  int IntContent;
  int i, j, range, type, cnt;
  short *ScalingList;

  while (p < bufend)
  {
    switch (*p)
    {
      case 13:
        p++;
        break;
      case '#':                 // Found comment
        *p = '\0';              // Replace '#' with '\0' in case of comment immediately following integer or string
        while (*p != '\n' && p < bufend)  // Skip till EOL or EOF, whichever comes first
          p++;
        InString = 0;
        InItem = 0;
        break;
      case '\n':
        InItem = 0;
        InString = 0;
        *p++='\0';
        break;
      case ' ':
      case '\t':              // Skip whitespace, leave state unchanged
        if (InString)
          p++;
        else
        {                     // Terminate non-strings once whitespace is found
          *p++ = '\0';
          InItem = 0;
        }
        break;

      case '"':               // Begin/End of String
        *p++ = '\0';
        if (!InString)
        {
          items[item++] = p;
          InItem = ~InItem;
        }
        else
          InItem = 0;
        InString = ~InString; // Toggle
        break;

      case ',':
        p++;
        InItem = 0;
        break;

      default:
        if (!InItem)
        {
          items[item++] = p;
          InItem = ~InItem;
        }
        p++;
    }
  }

  item--;

  for (i=0; i<item; i+=cnt)
  {
    cnt=0;
    if (0 > (MapIdx = CheckParameterName (items[i+cnt], &type)))
    {
      snprintf (errortext, ET_SIZE, " Parsing error in quantization matrix config file: Parameter Name '%s' not recognized.", items[i+cnt]);
      error (errortext, 300);
    }
    cnt++;
    if (strcmp ("=", items[i+cnt]))
    {
      snprintf (errortext, ET_SIZE, " Parsing error in quantization matrix config file: '=' expected as the second token in each item.");
      error (errortext, 300);
    }
    cnt++;

    if (!type) //4x4 Matrix
    {
      range = 16;
      ScalingList = p_QScale->ScalingList4x4input[MapIdx];
      matrix4x4_check[MapIdx] = 1; //to indicate matrix found in cfg file
    }
    else //8x8 matrix
    {
      range = 64;
      ScalingList = p_QScale->ScalingList8x8input[MapIdx];
      matrix8x8_check[MapIdx] = 1; //to indicate matrix found in cfg file
    }

    for(j=0; j<range; j++)
    {
      if (1 != sscanf (items[i+cnt+j], "%d", &IntContent))
      {
        snprintf (errortext, ET_SIZE, " Parsing error in quantization matrix file: Expected numerical value for Parameter of %s, found '%s'.", items[i], items[i+cnt+j]);
        error (errortext, 300);
      }

      ScalingList[j] = (short)IntContent; //save value in matrix
    }
    cnt+=j;
    printf (".");
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Check Q Matrix values. If invalid values found in matrix,
 *    whole matrix will be patch with default value 16.
 ***********************************************************************
 */
static void PatchMatrix(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  ScaleParameters *p_QScale = p_Vid->p_QScale;
  short *ScalingList;
  int i, cnt, fail;

  for(i=0; i<6; i++)
  {
    if(p_Inp->ScalingListPresentFlag[i])
    {
      ScalingList = p_QScale->ScalingList4x4input[i];
      if(matrix4x4_check[i])
      {
        fail=0;
        for(cnt=0; cnt<16; cnt++)
        {
          if(ScalingList[cnt]<0 || ScalingList[cnt]>255) // ScalingList[0]=0 to indicate use default matrix
          {
            fail=1;
            break;
          }
        }

        if(fail) //value of matrix exceed range
        {
          printf("\n%s value exceed range. (Value must be 1 to 255)\n", MatrixType4x4[i]);
          printf("Setting default values for this matrix.");
          if(i>2)
            memcpy(ScalingList, Quant_inter_default, sizeof(short)*16);
          else
            memcpy(ScalingList, Quant_intra_default, sizeof(short)*16);
        }
      }
      else //matrix not found, pad with default value
      {
        printf("\n%s matrix definition not found. Setting default values.", MatrixType4x4[i]);
        if(i>2)
          memcpy(ScalingList, Quant_inter_default, sizeof(short)*16);
        else
          memcpy(ScalingList, Quant_intra_default, sizeof(short)*16);
      }
    }

    if(p_Inp->ScalingListPresentFlag[i+6])
    {
      ScalingList = p_QScale->ScalingList8x8input[i];
      if(matrix8x8_check[i])
      {
        fail=0;
        for(cnt=0; cnt<64; cnt++)
        {
          if(ScalingList[cnt]<0 || ScalingList[cnt]>255) // ScalingList[0]=0 to indicate use default matrix
          {
            fail=1;
            break;
          }
        }

        if(fail) //value of matrix exceed range
        {
          printf("\n%s value exceed range. (Value must be 1 to 255)\n", MatrixType8x8[i]);
          printf("Setting default values for this matrix.");
          if(i==1 || i==3 || i==5)
            memcpy(ScalingList, Quant8_inter_default, sizeof(short)*64);
          else
            memcpy(ScalingList, Quant8_intra_default, sizeof(short)*64);
        }
      }
      else //matrix not found, pad with default value
      {
        printf("\n%s matrix definition not found. Setting default values.", MatrixType8x8[i]);
        if(i==1 || i==3 || i==5)
          memcpy(ScalingList, Quant8_inter_default, sizeof(short)*64);
        else
          memcpy(ScalingList, Quant8_intra_default, sizeof(short)*64);
      }
    }
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Allocate Q matrix arrays
 ***********************************************************************
 */
static void allocate_QMatrix (QuantParameters *p_Quant, InputParameters *p_Inp)
{
  int max_bitdepth = imax(p_Inp->output.bit_depth[0], p_Inp->output.bit_depth[1]);
  int max_qp = (3 + 6*(max_bitdepth));

  int bitdepth_qp_scale = 6*(p_Inp->output.bit_depth[0] - 8);
  int i;

  get_mem5Dquant(&p_Quant->q_params_4x4, 3, 2, max_qp + 1, 4, 4);
  get_mem5Dquant(&p_Quant->q_params_8x8, 3, 2, max_qp + 1, 8, 8);

  if ((p_Quant->qp_per_matrix = (int*)malloc((MAX_QP + 1 +  bitdepth_qp_scale)*sizeof(int))) == NULL)
    no_mem_exit("allocate_QMatrix: p_Quant->qp_per_matrix");
  if ((p_Quant->qp_rem_matrix = (int*)malloc((MAX_QP + 1 +  bitdepth_qp_scale)*sizeof(int))) == NULL)
    no_mem_exit("allocate_QMatrix: p_Quant->qp_per_matrix");

  for (i = 0; i < MAX_QP + bitdepth_qp_scale + 1; i++)
  {
    p_Quant->qp_per_matrix[i] = i / 6;
    p_Quant->qp_rem_matrix[i] = i % 6;
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Free Q matrix arrays
 ***********************************************************************
 */
void free_QMatrix (QuantParameters *p_Quant)
{
  free_mem5Dquant(p_Quant->q_params_4x4);
  free_mem5Dquant(p_Quant->q_params_8x8);

  free(p_Quant->qp_rem_matrix);
  free(p_Quant->qp_per_matrix);
}


/*!
 ***********************************************************************
 * \brief
 *    Initialise Q matrix values.
 ***********************************************************************
 */
void init_qmatrix (VideoParameters *p_Vid, InputParameters *p_Inp)
{
  QuantParameters *p_Quant = p_Vid->p_Quant;
  ScaleParameters *p_QScale = p_Vid->p_QScale;
  char *content;

  allocate_QMatrix (p_Quant, p_Inp);

  if(p_Inp->ScalingMatrixPresentFlag)
  {
    printf ("Parsing QMatrix file %s ", p_Inp->QmatrixFile);
    content = GetConfigFileContent(p_Inp->QmatrixFile, 0);
    if(content!='\0')
      ParseMatrix(p_Vid, content, (int) strlen (content));
    else
      printf("\nError: %s\nProceeding with default values for all matrices.", errortext);

    PatchMatrix(p_Vid, p_Inp);
    printf("\n");

    memset(p_QScale->UseDefaultScalingMatrix4x4Flag, 0, 6 * sizeof(short));
    memset(p_QScale->UseDefaultScalingMatrix8x8Flag, 0, 6 * sizeof(short));

    free(content);
  }
}

static void set_default_quant4x4(LevelQuantParams **q_params_4x4,  const int (*quant)[4], const int (*dequant)[4])
{
  int i, j;
  for(j=0; j<4; j++)
  {
    for(i=0; i<4; i++)
    {
      q_params_4x4[j][i].ScaleComp    = quant[j][i];
      q_params_4x4[j][i].InvScaleComp = dequant[j][i]<<4;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    For calculating the quantisation values at frame level
 *
 * \par Input:
 *    none
 *
 * \par Output:
 *    none
 ************************************************************************
 */
void CalculateQuant4x4Param(VideoParameters *p_Vid)
{
  QuantParameters *p_Quant  = p_Vid->p_Quant;
  ScaleParameters *p_QScale = p_Vid->p_QScale;

  pic_parameter_set_rbsp_t *active_pps = p_Vid->active_pps;
  seq_parameter_set_rbsp_t *active_sps = p_Vid->active_sps;

  int i, j, k, temp;
  int k_mod;
  int present[6];
  int no_q_matrix=FALSE;

  int max_bitdepth = imax(p_Vid->bitdepth_luma, p_Vid->bitdepth_chroma);
  int max_qp = (3 + 6*(max_bitdepth));


  if(!active_sps->seq_scaling_matrix_present_flag && !active_pps->pic_scaling_matrix_present_flag) //set to no q-matrix
    no_q_matrix=TRUE;
  else
  {
    memset(present, 0, 6 * sizeof(int));

    if(active_sps->seq_scaling_matrix_present_flag)
      for(i=0; i<6; i++)
        present[i] = active_sps->seq_scaling_list_present_flag[i];

    if(active_pps->pic_scaling_matrix_present_flag)
      for(i=0; i<6; i++)
      {
        if((i==0) || (i==3))
          present[i] |= active_pps->pic_scaling_list_present_flag[i];
        else
          present[i] = active_pps->pic_scaling_list_present_flag[i];
      }
  }

  if(no_q_matrix==TRUE)
  {
    for(k_mod = 0; k_mod <= max_qp; k_mod++)
    {
      k = k_mod % 6;
      set_default_quant4x4(p_Quant->q_params_4x4[0][0][k_mod],  quant_coef[k], dequant_coef[k]);
      set_default_quant4x4(p_Quant->q_params_4x4[0][1][k_mod],  quant_coef[k], dequant_coef[k]);
      set_default_quant4x4(p_Quant->q_params_4x4[1][0][k_mod],  quant_coef[k], dequant_coef[k]);
      set_default_quant4x4(p_Quant->q_params_4x4[1][1][k_mod],  quant_coef[k], dequant_coef[k]);
      set_default_quant4x4(p_Quant->q_params_4x4[2][0][k_mod],  quant_coef[k], dequant_coef[k]);
      set_default_quant4x4(p_Quant->q_params_4x4[2][1][k_mod],  quant_coef[k], dequant_coef[k]);
    }
  }
  else
  {
    for(k_mod = 0; k_mod <= max_qp; k_mod++)
    {
      k = k_mod % 6;
      for(j=0; j<4; j++)
      {
        for(i=0; i<4; i++)
        {
          temp = (j<<2)+i;
          if((!present[0]) || p_QScale->UseDefaultScalingMatrix4x4Flag[0])
          {
            p_Quant->q_params_4x4[0][1][k_mod][j][i].ScaleComp    = (quant_coef[k][j][i]<<4)/Quant_intra_default[temp];
            p_Quant->q_params_4x4[0][1][k_mod][j][i].InvScaleComp = dequant_coef[k][j][i]*Quant_intra_default[temp];
          }
          else
          {
            p_Quant->q_params_4x4[0][1][k_mod][j][i].ScaleComp    = (quant_coef[k][j][i]<<4)/p_QScale->ScalingList4x4[0][temp];
            p_Quant->q_params_4x4[0][1][k_mod][j][i].InvScaleComp = dequant_coef[k][j][i]*p_QScale->ScalingList4x4[0][temp];
          }

          if(!present[1])
          {
            p_Quant->q_params_4x4[1][1][k_mod][j][i].ScaleComp    = p_Quant->q_params_4x4[0][1][k_mod][j][i].ScaleComp;
            p_Quant->q_params_4x4[1][1][k_mod][j][i].InvScaleComp = p_Quant->q_params_4x4[0][1][k_mod][j][i].InvScaleComp;
          }
          else
          {
            p_Quant->q_params_4x4[1][1][k_mod][j][i].ScaleComp    = (quant_coef[k][j][i]<<4)/(p_QScale->UseDefaultScalingMatrix4x4Flag[1] ? Quant_intra_default[temp]:p_QScale->ScalingList4x4[1][temp]);
            p_Quant->q_params_4x4[1][1][k_mod][j][i].InvScaleComp = dequant_coef[k][j][i]*(p_QScale->UseDefaultScalingMatrix4x4Flag[1] ? Quant_intra_default[temp]:p_QScale->ScalingList4x4[1][temp]);
          }

          if(!present[2])
          {
            p_Quant->q_params_4x4[2][1][k_mod][j][i].ScaleComp    = p_Quant->q_params_4x4[1][1][k_mod][j][i].ScaleComp;
            p_Quant->q_params_4x4[2][1][k_mod][j][i].InvScaleComp = p_Quant->q_params_4x4[1][1][k_mod][j][i].InvScaleComp;
          }
          else
          {
            p_Quant->q_params_4x4[2][1][k_mod][j][i].ScaleComp    = (quant_coef[k][j][i]<<4)/(p_QScale->UseDefaultScalingMatrix4x4Flag[2] ? Quant_intra_default[temp]:p_QScale->ScalingList4x4[2][temp]);
            p_Quant->q_params_4x4[2][1][k_mod][j][i].InvScaleComp = dequant_coef[k][j][i]*(p_QScale->UseDefaultScalingMatrix4x4Flag[2] ? Quant_intra_default[temp]:p_QScale->ScalingList4x4[2][temp]);
          }

          if((!present[3]) || p_QScale->UseDefaultScalingMatrix4x4Flag[3])
          {
            p_Quant->q_params_4x4[0][0][k_mod][j][i].ScaleComp         = (quant_coef[k][j][i]<<4)/Quant_inter_default[temp];
            p_Quant->q_params_4x4[0][0][k_mod][j][i].InvScaleComp      = dequant_coef[k][j][i]*Quant_inter_default[temp];
          }
          else
          {
            p_Quant->q_params_4x4[0][0][k_mod][j][i].ScaleComp         = (quant_coef[k][j][i]<<4)/p_QScale->ScalingList4x4[3][temp];
            p_Quant->q_params_4x4[0][0][k_mod][j][i].InvScaleComp      = dequant_coef[k][j][i]*p_QScale->ScalingList4x4[3][temp];
          }

          if(!present[4])
          {
            p_Quant->q_params_4x4[1][0][k_mod][j][i].ScaleComp    = p_Quant->q_params_4x4[0][0][k_mod][j][i].ScaleComp;
            p_Quant->q_params_4x4[1][0][k_mod][j][i].InvScaleComp = p_Quant->q_params_4x4[0][0][k_mod][j][i].InvScaleComp;
          }
          else
          {
            p_Quant->q_params_4x4[1][0][k_mod][j][i].ScaleComp    = (quant_coef[k][j][i]<<4)/(p_QScale->UseDefaultScalingMatrix4x4Flag[4] ? Quant_inter_default[temp]:p_QScale->ScalingList4x4[4][temp]);
            p_Quant->q_params_4x4[1][0][k_mod][j][i].InvScaleComp = dequant_coef[k][j][i]*(p_QScale->UseDefaultScalingMatrix4x4Flag[4] ? Quant_inter_default[temp]:p_QScale->ScalingList4x4[4][temp]);
          }

          if(!present[5])
          {
            p_Quant->q_params_4x4[2][0][k_mod][j][i].ScaleComp    = p_Quant->q_params_4x4[1][0][k_mod][j][i].ScaleComp;
            p_Quant->q_params_4x4[2][0][k_mod][j][i].InvScaleComp = p_Quant->q_params_4x4[1][0][k_mod][j][i].InvScaleComp;
          }
          else
          {
            p_Quant->q_params_4x4[2][0][k_mod][j][i].ScaleComp    = (quant_coef[k][j][i]<<4)/(p_QScale->UseDefaultScalingMatrix4x4Flag[5] ? Quant_inter_default[temp]:p_QScale->ScalingList4x4[5][temp]);
            p_Quant->q_params_4x4[2][0][k_mod][j][i].InvScaleComp = dequant_coef[k][j][i]*(p_QScale->UseDefaultScalingMatrix4x4Flag[5] ? Quant_inter_default[temp]:p_QScale->ScalingList4x4[5][temp]);
          }
        }
      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Calculate the quantisation and inverse quantisation parameters
 *
 ************************************************************************
 */
void CalculateQuant8x8Param(VideoParameters *p_Vid)
{
  QuantParameters *p_Quant = p_Vid->p_Quant;
  ScaleParameters *p_QScale = p_Vid->p_QScale;

  pic_parameter_set_rbsp_t *active_pps = p_Vid->active_pps;
  seq_parameter_set_rbsp_t *active_sps = p_Vid->active_sps;

  int i, j, k, temp;
  int k_mod;
  int n_ScalingList8x8;
  int present[6];
  int no_q_matrix=FALSE;
  int max_bitdepth = imax(p_Vid->bitdepth_luma, p_Vid->bitdepth_chroma);
  int max_qp = (3 + 6*(max_bitdepth));


  // maximum number of valid 8x8 scaling lists
  n_ScalingList8x8 = ( active_sps->chroma_format_idc != 3 ) ? 2 : 6;

  if(!active_sps->seq_scaling_matrix_present_flag && !active_pps->pic_scaling_matrix_present_flag) //set to default matrix
    no_q_matrix=TRUE;
  else
  {
    memset(present, 0, sizeof(int)*n_ScalingList8x8);

    if(active_sps->seq_scaling_matrix_present_flag)
    {
      for(i=0; i<n_ScalingList8x8; i++)
        present[i] = active_sps->seq_scaling_list_present_flag[i+6];
    }

    if(active_pps->pic_scaling_matrix_present_flag)
    {
      for(i=0; i<n_ScalingList8x8; i++)
      {
        if( i==0 || i==1 )
          present[i] |= active_pps->pic_scaling_list_present_flag[i+6];
        else
          present[i] = active_pps->pic_scaling_list_present_flag[i+6];
      }
    }
  }

  if(no_q_matrix==TRUE)
  {
    for(k = 0; k <= max_qp; k++)
    {
      k_mod = k % 6;
      for(j=0; j<8; j++)
      {
        for(i=0; i<8; i++)
        {
          // intra Y
          p_Quant->q_params_8x8[0][1][k][j][i].ScaleComp      = quant_coef8[k_mod][j][i];
          p_Quant->q_params_8x8[0][1][k][j][i].InvScaleComp   = dequant_coef8[k_mod][j][i]<<4;

          // inter Y
          p_Quant->q_params_8x8[0][0][k][j][i].ScaleComp      = quant_coef8[k_mod][j][i];
          p_Quant->q_params_8x8[0][0][k][j][i].InvScaleComp   = dequant_coef8[k_mod][j][i]<<4;

          if( n_ScalingList8x8 > 2 )  // 4:4:4 case only
          {
            // intra U
            p_Quant->q_params_8x8[1][1][k][j][i].ScaleComp    = quant_coef8[k_mod][j][i];
            p_Quant->q_params_8x8[1][1][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]<<4;

            // intra V
            p_Quant->q_params_8x8[2][1][k][j][i].ScaleComp    = quant_coef8[k_mod][j][i];
            p_Quant->q_params_8x8[2][1][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]<<4;

            // inter U
            p_Quant->q_params_8x8[1][0][k][j][i].ScaleComp    = quant_coef8[k_mod][j][i];
            p_Quant->q_params_8x8[1][0][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]<<4;

            // inter V
            p_Quant->q_params_8x8[2][0][k][j][i].ScaleComp    = quant_coef8[k_mod][j][i];
            p_Quant->q_params_8x8[2][0][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]<<4;

          }
        }
      }
    }
  }
  else
  {
    for(k = 0; k <= max_qp; k++)
    {
      k_mod = k % 6;
      for(j=0; j<8; j++)
      {
        for(i=0; i<8; i++)
        {
          temp = (j<<3)+i;
          if((!present[0]) || p_QScale->UseDefaultScalingMatrix8x8Flag[0])
          {
            p_Quant->q_params_8x8[0][1][k][j][i].ScaleComp    = (quant_coef8[k_mod][j][i]<<4)/Quant8_intra_default[temp];
            p_Quant->q_params_8x8[0][1][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]*Quant8_intra_default[temp];
          }
          else
          {
            p_Quant->q_params_8x8[0][1][k][j][i].ScaleComp    = (quant_coef8[k_mod][j][i]<<4)/p_QScale->ScalingList8x8[0][temp];
            p_Quant->q_params_8x8[0][1][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]*p_QScale->ScalingList8x8[0][temp];
          }

          if((!present[1]) || p_QScale->UseDefaultScalingMatrix8x8Flag[1])
          {
            p_Quant->q_params_8x8[0][0][k][j][i].ScaleComp    = (quant_coef8[k_mod][j][i]<<4)/Quant8_inter_default[temp];
            p_Quant->q_params_8x8[0][0][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]*Quant8_inter_default[temp];
          }
          else
          {
            p_Quant->q_params_8x8[0][0][k][j][i].ScaleComp    = (quant_coef8[k_mod][j][i]<<4)/p_QScale->ScalingList8x8[1][temp];
            p_Quant->q_params_8x8[0][0][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]*p_QScale->ScalingList8x8[1][temp];
          }

          if( n_ScalingList8x8 > 2 )
          {

            // Intra U
            if(!present[2])
            {
              p_Quant->q_params_8x8[1][1][k][j][i].ScaleComp    = p_Quant->q_params_8x8[0][1][k][j][i].ScaleComp;
              p_Quant->q_params_8x8[1][1][k][j][i].InvScaleComp = p_Quant->q_params_8x8[0][1][k][j][i].InvScaleComp;
            }
            else
            {
              p_Quant->q_params_8x8[1][1][k][j][i].ScaleComp    = (quant_coef8[k_mod][j][i]<<4)/(p_QScale->UseDefaultScalingMatrix8x8Flag[2] ? Quant8_intra_default[temp]:p_QScale->ScalingList8x8[2][temp]);
              p_Quant->q_params_8x8[1][1][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]*(p_QScale->UseDefaultScalingMatrix8x8Flag[2] ? Quant8_intra_default[temp]:p_QScale->ScalingList8x8[2][temp]);
            }

            // Inter U
            if(!present[3])
            {
              p_Quant->q_params_8x8[1][0][k][j][i].ScaleComp    = p_Quant->q_params_8x8[0][0][k][j][i].ScaleComp;
              p_Quant->q_params_8x8[1][0][k][j][i].InvScaleComp = p_Quant->q_params_8x8[0][0][k][j][i].InvScaleComp;
            }
            else
            {
              p_Quant->q_params_8x8[1][0][k][j][i].ScaleComp    = (quant_coef8[k_mod][j][i]<<4)/(p_QScale->UseDefaultScalingMatrix8x8Flag[3] ? Quant8_inter_default[temp]:p_QScale->ScalingList8x8[3][temp]);
              p_Quant->q_params_8x8[1][0][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]*(p_QScale->UseDefaultScalingMatrix8x8Flag[3] ? Quant8_inter_default[temp]:p_QScale->ScalingList8x8[3][temp]);
            }

            // Intra V
            if(!present[4])
            {
              p_Quant->q_params_8x8[2][1][k][j][i].ScaleComp    = p_Quant->q_params_8x8[1][1][k][j][i].ScaleComp;
              p_Quant->q_params_8x8[2][1][k][j][i].InvScaleComp = p_Quant->q_params_8x8[1][1][k][j][i].InvScaleComp;
            }
            else
            {
              p_Quant->q_params_8x8[2][1][k][j][i].ScaleComp    = (quant_coef8[k_mod][j][i]<<4)/(p_QScale->UseDefaultScalingMatrix8x8Flag[4] ? Quant8_intra_default[temp]:p_QScale->ScalingList8x8[4][temp]);
              p_Quant->q_params_8x8[2][1][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i]*(p_QScale->UseDefaultScalingMatrix8x8Flag[4] ? Quant8_intra_default[temp]:p_QScale->ScalingList8x8[4][temp]);
            }

            // Inter V
            if(!present[5])
            {
              p_Quant->q_params_8x8[2][0][k][j][i].ScaleComp    = p_Quant->q_params_8x8[1][0][k][j][i].ScaleComp;
              p_Quant->q_params_8x8[2][0][k][j][i].InvScaleComp = p_Quant->q_params_8x8[1][0][k][j][i].InvScaleComp;
            }
            else
            {
              p_Quant->q_params_8x8[2][0][k][j][i].ScaleComp    = (quant_coef8[k_mod][j][i]<<4)/(p_QScale->UseDefaultScalingMatrix8x8Flag[5] ? Quant8_inter_default[temp]:p_QScale->ScalingList8x8[5][temp]);
              p_Quant->q_params_8x8[2][0][k][j][i].InvScaleComp = dequant_coef8[k_mod][j][i] * (p_QScale->UseDefaultScalingMatrix8x8Flag[5] ? Quant8_inter_default[temp]:p_QScale->ScalingList8x8[5][temp]);
            }
          }
        }
      }
    }
  }
}

