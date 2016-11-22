/*!
 *************************************************************************************
 * \file explicit_seq.c
 *
 * \brief
 *    Support for explicit coding order/structure support.
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Alexis Michael Tourapis                     <alexismt@ieee.org>
 *************************************************************************************
 */

#include "contributors.h"
#include "global.h"
#include "report.h"
#include "explicit_seq.h"

/*!
************************************************************************
* \brief
*    Read one Text Field
************************************************************************
*/
static int ReadTextField(FILE *expSeqFile, char* keyword)
{
  char readline [100];
  char word[64];  
  int err = -1;
  int read_line = 0; // read up to 5 lines to detect data
  do 
  {    
    read_line++;
    // let us read a full line of data
    if (NULL == fgets(readline, 100, expSeqFile))
      error ("error parsing explicit sequence file", 500);
    err = sscanf(readline,"%s",word);
    if (err == -1)
      break;
    if (strcasecmp(word, keyword) != 0)
      err = - 1;

    if (feof(expSeqFile)) // move to the end of the line
      break;
  }
  while (err == - 1 && read_line < 6);
  // Lets keep err disabled. This allows us to continue encoding even 
  // when input file is empty or reaches end.
  /*
  if (err != 1)
  {   
  printf("Error while reading text \"%s\" from input file.\n", keyword);
  report_stats_on_error();
  }  
  */
  return err;
}

/*!
************************************************************************
* \brief
*    Read one Field of Integer type
************************************************************************
*/
void ReadIntField(FILE *expSeqFile, char* format, char* keyword, int* value)
{
  char readline [100];
  char word[64];
  int err = -1;
  int read_line = 0; // read up to 5 lines to detect data

  do
  {
    read_line++;    
    if (NULL == fgets(readline, 100, expSeqFile))
      error ("error parsing explicit sequence file", 500);
    err = sscanf(readline, format, word, value);
    if (feof(expSeqFile))
      break;      
  }
  while (strcasecmp(word, keyword) != 0 && read_line < 6);

  if (err != 2 || strcasecmp(word, keyword) != 0)
  {
    printf("Error while reading %s.\n", keyword);
    report_stats_on_error();
  }
}

/*!
************************************************************************
* \brief
*    Read one Field of String type
************************************************************************
*/
void ReadCharField(FILE *expSeqFile, char* format, char* keyword, char* value)
{
  char readline [100];
  char word[64];
  int err = -1;
  int read_line = 0; // read up to 5 lines to detect data

  do
  {
    read_line++;
    if (NULL == fgets(readline, 100, expSeqFile))
      error ("error parsing explicit sequence file", 500);
    err = sscanf(readline, format, word, value);

    while (!feof(expSeqFile)) // move to the end of the line
      break;      
  }
  while (strcasecmp(word, keyword) != 0 && read_line < 6);

  if (err != 2 || strcasecmp(word, keyword) != 0)
  {
    printf("Error while reading %s.\n", keyword);
    report_stats_on_error();
  }
}


static void ParseSliceType(char *slice_type, ExpFrameInfo *info, int coding_index)
{
  if ( strcasecmp(slice_type, "P") == 0 )
  {   
    info->slice_type = P_SLICE;
  }
  else if ( strcasecmp(slice_type, "B")  == 0 )
  {
    info->slice_type = B_SLICE;
  }
  else if ( strcasecmp(slice_type, "I") == 0 )
  {
    info->slice_type = I_SLICE;
  }
  else if ( strcasecmp(slice_type, "SP") == 0 )
  {
    info->slice_type = SP_SLICE;
  }
  else if ( strcasecmp(slice_type, "SI") == 0 )
  {
    info->slice_type = SI_SLICE;
  }
  else
  {
    printf("ReadExplicitSeqFile : invalid slice type\n");
    report_stats_on_error();
  }

  if (coding_index == 0 && info->slice_type != I_SLICE)
  {
    printf("ReadExplicitSeqFile : First coded picture needs to be Intra.\n");
    report_stats_on_error();
  }
}

static void ParseReferenceIDC(int reference_idc, int coding_index)
{
  if ( reference_idc < NALU_PRIORITY_DISPOSABLE || reference_idc > NALU_PRIORITY_HIGHEST)
  {
    printf("ReadExplicitSeqFile : Invalid reference indicator \n");
    report_stats_on_error();
  }

  if (coding_index == 0 && reference_idc == NALU_PRIORITY_DISPOSABLE)
  {
    printf("ReadExplicitSeqFile : First coded picture needs to be a reference picture.\n");
    report_stats_on_error();
  }
}


static void ParseSeqNumber(int seq_number, ExpSeqInfo *seq_info, int coding_index)
{
  int i;
  for (i = 0; i < imin(coding_index, seq_info->no_frames); i++)
  {
    if (seq_info->info[i].seq_number == seq_number)
    {
      printf("ReadExplicitSeqFile : SeqNumber used for current frame already used. Terminating\n");
      report_stats_on_error();
    }
  }
}

/*!
************************************************************************
* \brief
*    Read Frame information
************************************************************************
*/
void ReadFrameData(FILE *expSeqFile, ExpSeqInfo *seq_info, int coding_index)
{
  Boolean slice_type_present = FALSE;
  Boolean seq_number_present = FALSE;
  char readline [100];
  char word[64], value[64];  
  int err = -1;
  ExpFrameInfo *info = &seq_info->info[coding_index % seq_info->no_frames];
  // Set some defaults
  info->reference_idc = NALU_PRIORITY_HIGHEST;  

  ReadTextField (expSeqFile, "{");  // Start bracket
  do
  {
    // read one line of data
    if (NULL == fgets(readline, 100, expSeqFile))
      error ("error parsing explicit sequence file", 500);
    // let us check if ending character reached or error
    err = sscanf(readline, "%s : %s", word, value);    
    if (err == 1) // Check ending characters
    {
      if (strcasecmp(word, "}") == 0)
        break;
      else if (strcasecmp(word, "{") == 0)
      {
        printf("Invalid \"{\" character found. Terminating\n");
        report_stats_on_error();
      }
    }
    else if (err == 2) // parse different parameters. Input method is flexible and does not fix input order.
    {
      if (strcasecmp(word, "SeqNumber") == 0)
      {
        info->seq_number = atoi(value);
        ParseSeqNumber(info->seq_number, seq_info, coding_index);
        seq_number_present = TRUE;
      }
      else if (strcasecmp(word, "SliceType") == 0)
      {
        ParseSliceType(value, info, coding_index);
        slice_type_present = TRUE;
      }
      else if (strcasecmp(word, "IDRPicture") == 0)
        info->is_idr = atoi(value);
      else if (strcasecmp(word, "Reference") == 0)
      {
        info->reference_idc = atoi(value);
        ParseReferenceIDC(info->reference_idc, coding_index);
      }      
    }
  }
  while (!feof(expSeqFile));

  if (slice_type_present == FALSE || seq_number_present == FALSE)
  {
    printf("Sequence info file does not contain all mandatory info (SeqNumber or SliceType). Terminating.\n");
    report_stats_on_error();
  }
}

/*!
 ************************************************************************
 * \brief
 *    Read one picture from explicit sequence information file
 ************************************************************************
 */
void ReadExplicitSeqFile(ExpSeqInfo *seq_info, FILE *expSFile, int coding_index)
{
  int  frm_header = ReadTextField(expSFile, "Frame");

  if (frm_header != -1)
  {
    ReadFrameData (expSFile, seq_info, coding_index);
  }
  else
  {
    printf("ReadExplicitSeqFile : No more data. \n");
    report_stats_on_error();
  }
}

/*!
 ************************************************************************
 * \brief
 *    Open explicit sequence information file
 ************************************************************************
 */
void OpenExplicitSeqFile(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int frm_count = 0;
  p_Vid->expSFile = fopen(p_Inp->ExplicitSeqFile, "r");
  if (p_Vid->expSFile == NULL)
  {
    printf("ERROR while opening the explicit sequence information file.\n");
    report_stats_on_error();
  }  
  
  if (ReadTextField(p_Vid->expSFile, "Sequence") == -1)
  {
    printf("Sequence info file is of invalid format. Terminating\n");
    report_stats_on_error();
  }
  else
  {
    ReadIntField  (p_Vid->expSFile, "%s : %d", "FrameCount", &frm_count);
    if (frm_count > 0)
    {
      p_Vid->expSeq = (ExpSeqInfo *) malloc(sizeof(ExpSeqInfo));
      p_Vid->expSeq->no_frames = frm_count;
      p_Vid->expSeq->info = (ExpFrameInfo *) calloc(frm_count, sizeof(ExpFrameInfo));
    }
    else
    {
      printf("Invalid FrameCount in Sequence info file. Terminating\n");
      report_stats_on_error();
    }
  }
}

/*!
************************************************************************
* \brief
*    Close explicit sequence information file
************************************************************************
*/
void CloseExplicitSeqFile(VideoParameters *p_Vid)
{
  if (p_Vid->expSFile != NULL)
    fclose(p_Vid->expSFile);
  if (p_Vid->expSeq != NULL)
  {
    free(p_Vid->expSeq->info);
    free(p_Vid->expSeq);
  }
}


