
/*!
 *************************************************************************************
 * \file q_offsets.c
 *
 * \brief
 *    read Quantization Offset matrix parameters from input file: q_OffsetMatrix.cfg
 *
 *************************************************************************************
 */

#include "global.h"
#include "memalloc.h"
#include "q_matrix.h"
#include "q_offsets.h"

extern char *GetConfigFileContent (char *Filename, int error_type);

#define MAX_ITEMS_TO_PARSE  2000

int offset4x4_check[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int offset8x8_check[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static const char OffsetType4x4[15][24] = {
  "INTRA4X4_LUMA_INTRA",
  "INTRA4X4_CHROMAU_INTRA",
  "INTRA4X4_CHROMAV_INTRA",
  "INTRA4X4_LUMA_INTERP",
  "INTRA4X4_CHROMAU_INTERP",
  "INTRA4X4_CHROMAV_INTERP",
  "INTRA4X4_LUMA_INTERB",
  "INTRA4X4_CHROMAU_INTERB",
  "INTRA4X4_CHROMAV_INTERB",
  "INTER4X4_LUMA_INTERP",
  "INTER4X4_CHROMAU_INTERP",
  "INTER4X4_CHROMAV_INTERP",
  "INTER4X4_LUMA_INTERB",
  "INTER4X4_CHROMAU_INTERB",
  "INTER4X4_CHROMAV_INTERB"
};

static const char OffsetType8x8[15][24] = {
  "INTRA8X8_LUMA_INTRA",
  "INTRA8X8_LUMA_INTERP",
  "INTRA8X8_LUMA_INTERB",
  "INTER8X8_LUMA_INTERP",
  "INTER8X8_LUMA_INTERB",
  "INTRA8X8_CHROMAU_INTRA",
  "INTRA8X8_CHROMAU_INTERP",
  "INTRA8X8_CHROMAU_INTERB",
  "INTER8X8_CHROMAU_INTERP",
  "INTER8X8_CHROMAU_INTERB",
  "INTRA8X8_CHROMAV_INTRA",
  "INTRA8X8_CHROMAV_INTERP",
  "INTRA8X8_CHROMAV_INTERB",
  "INTER8X8_CHROMAV_INTERP",
  "INTER8X8_CHROMAV_INTERB"
};

static const short Offset_intra_flat_intra[16] = {
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024
};

static const short Offset_intra_flat_chroma[16] = {
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024
};


static const short Offset_intra_flat_inter[16] = {
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
};

static const short Offset_inter_flat[16] = {
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024,
};

static const short Offset8_intra_flat_intra[64] = {
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
};

static const short Offset8_intra_flat_chroma[64] = {
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
};

static const short Offset8_intra_flat_inter[64] = {
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
};

static const short Offset8_inter_flat[64] = {
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
};


// Default offsets
static const short Offset_intra_default_intra[16] = {
  682, 682, 682, 682,
  682, 682, 682, 682,
  682, 682, 682, 682,
  682, 682, 682, 682
};

static const short Offset_intra_default_chroma[16] = {
  682, 682, 682, 682,
  682, 682, 682, 682,
  682, 682, 682, 682,
  682, 682, 682, 682
};


static const short Offset_intra_default_inter[16] = {
  342, 342, 342, 342,
  342, 342, 342, 342,
  342, 342, 342, 342,
  342, 342, 342, 342,
};

static const short Offset_inter_default[16] = {
  342, 342, 342, 342,
  342, 342, 342, 342,
  342, 342, 342, 342,
  342, 342, 342, 342,
};

static const short Offset8_intra_default_intra[64] = {
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682
};

static const short Offset8_intra_default_chroma[64] = {
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682
};

static const short Offset8_intra_default_inter[64] = {
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342
};

static const short Offset8_inter_default[64] = {
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342
};


// Function prototypes
void InitOffsetParam (QuantParameters *p_Quant, InputParameters *p_Inp);
/*!
 ***********************************************************************
 * \brief
 *    Allocate Q matrix arrays
 ***********************************************************************
 */
static void allocate_QOffsets (QuantParameters *p_Quant, InputParameters *p_Inp)
{
  int max_bitdepth = imax(p_Inp->output.bit_depth[0], p_Inp->output.bit_depth[1]);
  int max_qp = (3 + 6*(max_bitdepth));


  if (p_Inp->AdaptRoundingFixed)
  {
    get_mem3Dshort(&p_Quant->OffsetList4x4, 1, 25, 16);
    get_mem3Dshort(&p_Quant->OffsetList8x8, 1, 15, 64);    
  }
  else
  {
    get_mem3Dshort(&p_Quant->OffsetList4x4, max_qp + 1, 25, 16);
    get_mem3Dshort(&p_Quant->OffsetList8x8, max_qp + 1, 15, 64);    
  }

  get_mem2Dshort(&p_Quant->OffsetList4x4input, 25, 16);
  get_mem2Dshort(&p_Quant->OffsetList8x8input, 15, 64);
}

static inline void update_q_offset4x4(LevelQuantParams **q_params, short *offsetList, int q_bits)
{
  int i, j;
  short *p_list = &offsetList[0];
  for (j = 0; j < 4; j++)
  {
    for (i = 0; i < 4; i++)
    {          
      q_params[j][i].OffsetComp = (int) *p_list++ << q_bits;
    }
  }
}

static inline void update_q_offset8x8(LevelQuantParams **q_params, short *offsetList, int q_bits)
{
  int i, j;
  short *p_list;
  for (j = 0; j < 8; j++)
  {
    p_list = &offsetList[j << 3];
    for (i = 0; i < 8; i++)
    {          
      q_params[j][i].OffsetComp = (int) *p_list++ << q_bits;
    }
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Free Q matrix arrays
 ***********************************************************************
 */
void free_QOffsets (QuantParameters *p_Quant, InputParameters *p_Inp)
{
  free_mem3Dshort(p_Quant->OffsetList8x8);
  free_mem3Dshort(p_Quant->OffsetList4x4);    
  free_mem2Dshort(p_Quant->OffsetList8x8input);
  free_mem2Dshort(p_Quant->OffsetList4x4input);
}


/*!
 ***********************************************************************
 * \brief
 *    Check the parameter name.
 * \param s
 *    parameter name string
 * \param type
 *    4x4 or 8x8 offset matrix type
 * \return
 *    the index number if the string is a valid parameter name,         \n
 *    -1 for error
 ***********************************************************************
 */

int CheckOffsetParameterName (char *s, int *type)
{
  int i = 0;

  *type = 0;
  while ((OffsetType4x4[i] != NULL) && (i < 15))
  {
    if (0 == strcmp (OffsetType4x4[i], s))
      return i;
    else
      i++;
  }

  i = 0;
  *type = 1;
  while ((OffsetType8x8[i] != NULL) && (i < 15))
  {
    if (0 == strcmp (OffsetType8x8[i], s))
      return i;
    else
      i++;
  }

  return -1;
}

/*!
 ***********************************************************************
 * \brief
 *    Parse the Q Offset Matrix values read from cfg file.
 * \param p_Quant
 *    quantization parameters
 * \param buf
 *    buffer to be parsed
 * \param bufsize
 *    buffer size of buffer
 ***********************************************************************
 */
void ParseQOffsetMatrix (QuantParameters *p_Quant, char *buf, int bufsize)
{
  char *items[MAX_ITEMS_TO_PARSE] = {NULL}; 
  int MapIdx;
  int item = 0;
  int InString = 0, InItem = 0;
  char *p = buf;
  char *bufend = &buf[bufsize];
  int IntContent;
  int i, j, range, type, cnt;
  short *OffsetList;

  while (p < bufend)
  {
    switch (*p)
    {
      //case 13:
        //p++;
        //break;
      case '#':                 // Found comment
        *p = '\0';              // Replace '#' with '\0' in case of comment immediately following integer or string
        while ((*p != '\n' && *p != '\r') && p < bufend)  // Skip till EOL or EOF, whichever comes first
          p++;
        InString = 0;
        InItem = 0;
        break;
      case '\n':
      case '\r':
        InItem = 0;
        InString = 0;
      *p++ = '\0';
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

  for (i = 0; i < item; i += cnt)
  {
    cnt = 0;
    if (0 > (MapIdx = CheckOffsetParameterName (items[i + cnt], &type)))
    {
      snprintf (errortext, ET_SIZE,
        " Parsing error in quantization offset config file: Parameter Name '%s' not recognized.",
        items[i + cnt]);
      error (errortext, 300);
    }
    cnt++;
    if (strcmp ("=", items[i + cnt]))
    {
      snprintf (errortext, ET_SIZE,
        " Parsing error in quantization offset config file: '=' expected as the second token in each item.");
      error (errortext, 300);
    }
    cnt++;

    if (!type) //4x4 Matrix
    {
      range = 16;
      OffsetList = p_Quant->OffsetList4x4input[MapIdx];
      offset4x4_check[MapIdx] = 1; //to indicate matrix found in cfg file
    }
    else //8x8 matrix
    {
      range = 64;
      OffsetList = p_Quant->OffsetList8x8input[MapIdx];
      offset8x8_check[MapIdx] = 1; //to indicate matrix found in cfg file
    }

    for (j = 0; j < range; j++)
    {
      if (1 != sscanf (items[i + cnt + j], "%d", &IntContent))
      {
        snprintf (errortext, ET_SIZE,
          " Parsing error in quantization offset file: Expected numerical value for Parameter of %s, found '%s'.",
          items[i], items[i + cnt + j]);
        error (errortext, 300);
      }

      OffsetList[j] = (short) IntContent; //save value in matrix
    }
    cnt += j;
    printf (".");
  }
}


/*!
 ***********************************************************************
 * \brief
 *    Initialise Q offset matrix values.
 ***********************************************************************
 */
void init_qoffset (VideoParameters *p_Vid)
{
  char *content;
  InputParameters *p_Inp = p_Vid->p_Inp;

  allocate_QOffsets (p_Vid->p_Quant, p_Inp);

  if (p_Inp->OffsetMatrixPresentFlag)
  {
    printf ("Parsing Quantization Offset Matrix file %s ",
      p_Inp->QOffsetMatrixFile);
    content = GetConfigFileContent (p_Inp->QOffsetMatrixFile, 0);
    if (content != '\0')
      ParseQOffsetMatrix (p_Vid->p_Quant, content, (int) strlen (content));
    else
    {
      printf ("\nError: %s\nProceeding with default values for all matrices.", errortext);
      p_Inp->OffsetMatrixPresentFlag = 0;
    }

    printf ("\n");

    free (content);
  }
  //! Now set up all offset p_Inp. This process could be reused if we wish to re-init offsets
  InitOffsetParam (p_Vid->p_Quant, p_Inp);
}

/*!
 ************************************************************************
 * \brief
 *    Init quantization offset parameters
 *
 * \par Input:
 *    none
 *
 * \par Output:
 *    none
 ************************************************************************
 */

void InitOffsetParam (QuantParameters *p_Quant, InputParameters *p_Inp)
{
  int i, k;
  int max_qp_luma = (4 + 6*(p_Inp->output.bit_depth[0]));
  int max_qp_cr   = (4 + 6*(p_Inp->output.bit_depth[1]));

  for (i = 0; i < (p_Inp->AdaptRoundingFixed ? 1 : imax(max_qp_luma, max_qp_cr)); i++)
  {
    if (p_Inp->OffsetMatrixPresentFlag)
    {
      memcpy(&(p_Quant->OffsetList4x4[i][0][0]),&(p_Quant->OffsetList4x4input[0][0]), 400 * sizeof(short)); // 25 * 16
      memcpy(&(p_Quant->OffsetList8x8[i][0][0]),&(p_Quant->OffsetList8x8input[0][0]), 960 * sizeof(short)); // 15 * 64
    }
    else
    {
      if (p_Inp->OffsetMatrixFlat == 1)
      {
        // 0 (INTRA4X4_LUMA_INTRA)
        memcpy(&(p_Quant->OffsetList4x4[i][0][0]),&(Offset_intra_flat_intra[0]), 16 * sizeof(short));
        for (k = 1; k < 3; k++) // 1,2 (INTRA4X4_CHROMA_INTRA)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_intra_flat_chroma[0]),  16 * sizeof(short));
        for (k = 3; k < 9; k++) // 3,4,5,6,7,8 (INTRA4X4_LUMA/CHROMA_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_intra_flat_inter[0]),  16 * sizeof(short));
        for (k = 9; k < 25; k++) // 9,10,11,12,13,14 (INTER4X4)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_inter_flat[0]),  16 * sizeof(short));
  
        // 0 (INTRA8X8_LUMA_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][0][0]),&(Offset8_intra_flat_intra[0]), 64 * sizeof(short));
        for (k = 1; k < 3; k++)  // 1,2 (INTRA8X8_LUMA_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_flat_inter[0]),  64 * sizeof(short));
        for (k = 3; k < 5; k++)  // 3,4 (INTER8X8_LUMA_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_flat[0]),  64 * sizeof(short));
  
        // 5 (INTRA8X8_CHROMAU_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][5][0]),&(Offset8_intra_flat_chroma[0]), 64 * sizeof(short));
        for (k = 6; k < 8; k++)  // 6,7 (INTRA8X8_CHROMAU_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_flat_inter[0]),  64 * sizeof(short));
        for (k = 8; k < 10; k++)  // 8,9 (INTER8X8_CHROMAU_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_flat[0]),  64 * sizeof(short));

        // 10 (INTRA8X8_CHROMAV_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][10][0]),&(Offset8_intra_flat_chroma[0]), 64 * sizeof(short));
        for (k = 11; k < 13; k++)  // 11,12 (INTRA8X8_CHROMAV_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_flat_inter[0]),  64 * sizeof(short));
        for (k = 13; k < 15; k++)  // 8,9 (INTER8X8_CHROMAV_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_flat[0]),  64 * sizeof(short));
      }
      else if (p_Inp->OffsetMatrixFlat == 2)
      {
        // 0 (INTRA4X4_LUMA_INTRA)
        memcpy(&(p_Quant->OffsetList4x4[i][0][0]),&(Offset_intra_default_intra[0]), 16 * sizeof(short));
        for (k = 1; k < 3; k++) // 1,2 (INTRA4X4_CHROMA_INTRA)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_intra_flat_chroma[0]),  16 * sizeof(short));
        memcpy(&(p_Quant->OffsetList4x4[i][3][0]),&(Offset_intra_default_inter[0]),  16 * sizeof(short));
        for (k = 4; k < 6; k++) // 4,5 (INTRA4X4_CHROMA_INTERP)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_intra_flat_inter[0]),  16 * sizeof(short));
        memcpy(&(p_Quant->OffsetList4x4[i][6][0]),&(Offset_intra_default_inter[0]),  16 * sizeof(short));
        for (k = 7; k < 9; k++) // 7,8 (INTRA4X4_CHROMA_INTERB)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_intra_flat_inter[0]),  16 * sizeof(short));
        for (k = 9; k < 25; k++) // 9,10,11,12,13,14 (INTER4X4)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_inter_default[0]),  16 * sizeof(short));
  
        // 0 (INTRA8X8_LUMA_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][0][0]),&(Offset8_intra_default_intra[0]), 64 * sizeof(short));
        for (k = 1; k < 3; k++)  // 1,2 (INTRA8X8_LUMA_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_default_inter[0]),  64 * sizeof(short));
        for (k = 3; k < 5; k++)  // 3,4 (INTER8X8_LUMA_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_default[0]),  64 * sizeof(short));
  
        // 5 (INTRA8X8_CHROMAU_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][5][0]),&(Offset8_intra_flat_chroma[0]), 64 * sizeof(short));
        for (k = 6; k < 8; k++)  // 6,7 (INTRA8X8_CHROMAU_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_flat_inter[0]),  64 * sizeof(short));
        for (k = 8; k < 10; k++)  // 8,9 (INTER8X8_CHROMAU_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_default[0]),  64 * sizeof(short));

        // 10 (INTRA8X8_CHROMAV_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][10][0]),&(Offset8_intra_flat_chroma[0]), 64 * sizeof(short));
        for (k = 11; k < 13; k++)  // 11,12 (INTRA8X8_CHROMAV_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_flat_inter[0]),  64 * sizeof(short));
        for (k = 13; k < 15; k++)  // 8,9 (INTER8X8_CHROMAV_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_default[0]),  64 * sizeof(short));
      }
      else
      {
        // 0 (INTRA4X4_LUMA_INTRA)
        memcpy(&(p_Quant->OffsetList4x4[i][0][0]),&(Offset_intra_default_intra[0]), 16 * sizeof(short));
        for (k = 1; k < 3; k++) // 1,2 (INTRA4X4_CHROMA_INTRA)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_intra_default_chroma[0]),  16 * sizeof(short));
        for (k = 3; k < 9; k++) // 3,4,5,6,7,8 (INTRA4X4_LUMA/CHROMA_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_intra_default_inter[0]),  16 * sizeof(short));
        for (k = 9; k < 25; k++) // 9,10,11,12,13,14 (INTER4X4)
          memcpy(&(p_Quant->OffsetList4x4[i][k][0]),&(Offset_inter_default[0]),  16 * sizeof(short));
  
        // 0 (INTRA8X8_LUMA_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][0][0]),&(Offset8_intra_default_intra[0]), 64 * sizeof(short));
        for (k = 1; k < 3; k++)  // 1,2 (INTRA8X8_LUMA_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_default_inter[0]),  64 * sizeof(short));
        for (k = 3; k < 5; k++)  // 3,4 (INTER8X8_LUMA_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_default[0]),  64 * sizeof(short));
  
        // 5 (INTRA8X8_CHROMAU_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][5][0]),&(Offset8_intra_default_chroma[0]), 64 * sizeof(short));
        for (k = 6; k < 8; k++)  // 6,7 (INTRA8X8_CHROMAU_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_default_inter[0]),  64 * sizeof(short));
        for (k = 8; k < 10; k++)  // 8,9 (INTER8X8_CHROMAU_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_default[0]),  64 * sizeof(short));

        // 10 (INTRA8X8_CHROMAV_INTRA)
        memcpy(&(p_Quant->OffsetList8x8[i][10][0]),&(Offset8_intra_default_chroma[0]), 64 * sizeof(short));
        for (k = 11; k < 13; k++)  // 11,12 (INTRA8X8_CHROMAV_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_intra_default_inter[0]),  64 * sizeof(short));
        for (k = 13; k < 15; k++)  // 8,9 (INTER8X8_CHROMAV_INTERP/INTERB)
          memcpy(&(p_Quant->OffsetList8x8[i][k][0]),&(Offset8_inter_default[0]),  64 * sizeof(short));
      }
    }
  }  
}


/*!
 ************************************************************************
 * \brief
 *    Calculation of the quantization offset parameters at the frame level
 *
 * \par Input:
 *    none
 *
 * \par Output:
 *    none
 ************************************************************************
 */
void CalculateOffset4x4Param (VideoParameters *p_Vid)
{
  QuantParameters *p_Quant = p_Vid->p_Quant;
  int k;  
  int qp_per, qp;
  int img_type = ((p_Vid->type == SI_SLICE) ? I_SLICE : (p_Vid->type == SP_SLICE ? P_SLICE : p_Vid->type));

  int max_qp_scale = imax(p_Vid->bitdepth_luma_qp_scale, p_Vid->bitdepth_chroma_qp_scale);
  int max_qp = 51 + max_qp_scale;
  InputParameters *p_Inp = p_Vid->p_Inp;

  p_Vid->AdaptRndWeight   = p_Inp->AdaptRndWFactor  [p_Vid->nal_reference_idc != 0][img_type];
  p_Vid->AdaptRndCrWeight = p_Inp->AdaptRndCrWFactor[p_Vid->nal_reference_idc != 0][img_type];

  if (img_type == I_SLICE )
  {
    for (qp = 0; qp < max_qp + 1; qp++)
    {
      k = p_Quant->qp_per_matrix [qp];
      qp_per = Q_BITS + k - OffsetBits;
      k = p_Inp->AdaptRoundingFixed ? 0 : qp;

      // Intra4x4 luma
      update_q_offset4x4(p_Quant->q_params_4x4[0][1][qp], p_Quant->OffsetList4x4[k][ 0], qp_per);
      // Intra4x4 chroma u
      update_q_offset4x4(p_Quant->q_params_4x4[1][1][qp], p_Quant->OffsetList4x4[k][ 1], qp_per);
      // Intra4x4 chroma v
      update_q_offset4x4(p_Quant->q_params_4x4[2][1][qp], p_Quant->OffsetList4x4[k][ 2], qp_per);
    }
  }
  else if (img_type == B_SLICE)
  {
    for (qp = 0; qp < max_qp + 1; qp++)
    {
      k = p_Quant->qp_per_matrix [qp];
      qp_per = Q_BITS + k - OffsetBits;
      k = p_Inp->AdaptRoundingFixed ? 0 : qp;

      // Inter4x4 luma
      update_q_offset4x4(p_Quant->q_params_4x4[0][0][qp], p_Quant->OffsetList4x4[k][12], qp_per);
      // Intra4x4 luma
      update_q_offset4x4(p_Quant->q_params_4x4[0][1][qp], p_Quant->OffsetList4x4[k][ 6], qp_per);
      // Inter4x4 chroma u
      update_q_offset4x4(p_Quant->q_params_4x4[1][0][qp], p_Quant->OffsetList4x4[k][13], qp_per);
      // Intra4x4 chroma u
      update_q_offset4x4(p_Quant->q_params_4x4[1][1][qp], p_Quant->OffsetList4x4[k][ 7], qp_per);
      // Inter4x4 chroma v
      update_q_offset4x4(p_Quant->q_params_4x4[2][0][qp], p_Quant->OffsetList4x4[k][14], qp_per);      
      // Intra4x4 chroma v
      update_q_offset4x4(p_Quant->q_params_4x4[2][1][qp], p_Quant->OffsetList4x4[k][ 8], qp_per);
    }
  }
  else
  {
    for (qp = 0; qp < max_qp + 1; qp++)
    {
      k = p_Quant->qp_per_matrix [qp];
      qp_per = Q_BITS + k - OffsetBits;
      k = p_Inp->AdaptRoundingFixed ? 0 : qp;

      // Inter4x4 luma
      update_q_offset4x4(p_Quant->q_params_4x4[0][0][qp], p_Quant->OffsetList4x4[k][ 9], qp_per);
      // Intra4x4 luma
      update_q_offset4x4(p_Quant->q_params_4x4[0][1][qp], p_Quant->OffsetList4x4[k][ 3], qp_per);
      // Inter4x4 chroma u
      update_q_offset4x4(p_Quant->q_params_4x4[1][0][qp], p_Quant->OffsetList4x4[k][10], qp_per);
      // Intra4x4 chroma u
      update_q_offset4x4(p_Quant->q_params_4x4[1][1][qp], p_Quant->OffsetList4x4[k][ 4], qp_per);
      // Inter4x4 chroma v
      update_q_offset4x4(p_Quant->q_params_4x4[2][0][qp], p_Quant->OffsetList4x4[k][11], qp_per);      
      // Intra4x4 chroma v
      update_q_offset4x4(p_Quant->q_params_4x4[2][1][qp], p_Quant->OffsetList4x4[k][ 5], qp_per);
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Calculate the quantisation offset parameters
 *
 ************************************************************************
*/
void CalculateOffset8x8Param (VideoParameters *p_Vid)
{
  QuantParameters *p_Quant = p_Vid->p_Quant;
  int k;
  int q_bits, qp;

  int max_qp_scale = imax(p_Vid->bitdepth_luma_qp_scale, p_Vid->bitdepth_chroma_qp_scale);
  int max_qp = 51 + max_qp_scale;
  InputParameters *p_Inp = p_Vid->p_Inp;

  if (p_Vid->type == I_SLICE || p_Vid->type == SI_SLICE )
  {
    for (qp = 0; qp < max_qp + 1; qp++)
    {
      q_bits = Q_BITS_8 + p_Quant->qp_per_matrix[qp] - OffsetBits;
      k = p_Inp->AdaptRoundingFixed ? 0 : qp;
      // INTRA8X8
      update_q_offset8x8(p_Quant->q_params_8x8[0][1][qp], p_Quant->OffsetList8x8[k][0], q_bits);
      // INTRA8X8 CHROMAU
      update_q_offset8x8(p_Quant->q_params_8x8[1][1][qp], p_Quant->OffsetList8x8[k][5], q_bits);
      // INTRA8X8 CHROMAV
      update_q_offset8x8(p_Quant->q_params_8x8[2][1][qp], p_Quant->OffsetList8x8[k][10], q_bits);
    }
  }
  else if ((p_Vid->type == P_SLICE) || (p_Vid->type == SP_SLICE))
  {
    for (qp = 0; qp < max_qp + 1; qp++)
    {
      q_bits = Q_BITS_8 + p_Quant->qp_per_matrix[qp] - OffsetBits;
      k = p_Inp->AdaptRoundingFixed ? 0 : qp;

      // INTER8X8
      update_q_offset8x8(p_Quant->q_params_8x8[0][0][qp], p_Quant->OffsetList8x8[k][3], q_bits);
      // INTRA8X8
      update_q_offset8x8(p_Quant->q_params_8x8[0][1][qp], p_Quant->OffsetList8x8[k][1], q_bits);
      // INTER8X8 CHROMAU
      update_q_offset8x8(p_Quant->q_params_8x8[1][0][qp], p_Quant->OffsetList8x8[k][8], q_bits);
      // INTRA8X8 CHROMAU
      update_q_offset8x8(p_Quant->q_params_8x8[1][1][qp], p_Quant->OffsetList8x8[k][6], q_bits);
      // INTER8X8 CHROMAV
      update_q_offset8x8(p_Quant->q_params_8x8[2][0][qp], p_Quant->OffsetList8x8[k][13], q_bits);
      // INTRA8X8 CHROMAV
      update_q_offset8x8(p_Quant->q_params_8x8[2][1][qp], p_Quant->OffsetList8x8[k][11], q_bits);
    }
  }
  else
  {
    for (qp = 0; qp < max_qp + 1; qp++)
    {
      q_bits = Q_BITS_8 + p_Quant->qp_per_matrix[qp] - OffsetBits;
      k = p_Inp->AdaptRoundingFixed ? 0 : qp;

      // INTER8X8
      update_q_offset8x8(p_Quant->q_params_8x8[0][0][qp], p_Quant->OffsetList8x8[k][4], q_bits);
      // INTRA8X8
      update_q_offset8x8(p_Quant->q_params_8x8[0][1][qp], p_Quant->OffsetList8x8[k][2], q_bits);
      // INTER8X8 CHROMAU
      update_q_offset8x8(p_Quant->q_params_8x8[1][0][qp], p_Quant->OffsetList8x8[k][9], q_bits);
      // INTRA8X8 CHROMAU
      update_q_offset8x8(p_Quant->q_params_8x8[1][1][qp], p_Quant->OffsetList8x8[k][7], q_bits);
      // INTER8X8 CHROMAV
      update_q_offset8x8(p_Quant->q_params_8x8[2][0][qp], p_Quant->OffsetList8x8[k][14], q_bits);
      // INTRA8X8 CHROMAV
      update_q_offset8x8(p_Quant->q_params_8x8[2][1][qp], p_Quant->OffsetList8x8[k][12], q_bits);
    }
  }
}
