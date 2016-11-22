/*!
 *  \file
 *     report.c
 *  \brief
 *     Report related files()
 *  \author
 *   Main contributors (see contributors.h for copyright, address and affiliation details)
 *   - Karsten Suehring
 *   - Alexis Michael Tourapis         <alexismt@ieee.org>
 ***********************************************************************
 */

#include "contributors.h"

#include <time.h>
#include <math.h>

#include "global.h"
#include "context_ini.h"
#include "explicit_gop.h"
#include "filehandle.h"
#include "fmo.h"
#include "image.h"
#include "intrarefresh.h"
#include "leaky_bucket.h"
#include "me_epzs.h"
#include "me_epzs_int.h"
#include "output.h"
#include "parset.h"
#include "report.h"
#include "img_process_types.h"


static const char DistortionType[3][20] = {"SAD", "SSE", "Hadamard SAD"};

void   report_log_mode (VideoParameters *p_Vid, InputParameters *p_Inp, StatParameters *p_Stats);
/*!
 ************************************************************************
 * \brief
 *    Reports frame statistical data to a stats file
 ************************************************************************
 */
void report_frame_statistic(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  DistortionParams *p_Dist = p_Vid->p_Dist;
  FILE *p_stat_frm = NULL;  
  
  int i;
  char name[30];
  int bitcounter;
  StatParameters *cur_stats = &p_Vid->enc_picture->stats;

#ifndef WIN32
  time_t now;
  struct tm *l_time;
  char string[1000];
#else
  char timebuf[128];
#endif

  // write to log file
  if ((p_stat_frm = fopen("stat_frame.dat", "r")) == 0)            // check if file exists
  {
    if ((p_stat_frm = fopen("stat_frame.dat", "a")) == NULL)       // append new statistic at the end
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n", "stat_frame.dat.dat");
      error(errortext, 500);
    }
    else                                            // Create header for new log file
    {
      fprintf(p_stat_frm, " --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
      fprintf(p_stat_frm, "|            Encoder statistics. This file is generated during first encoding session, new sessions will be appended                                                                                                                                                                                                                                                                                                                                                              |\n");
      fprintf(p_stat_frm, " --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
    }
  }
  else
  {
    fclose (p_stat_frm);
    if ((p_stat_frm = fopen("stat_frame.dat", "a")) == NULL)       // File exists, just open for appending
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n", "stat_frame.dat.dat");
      error(errortext, 500);
    }
  }

  if (p_Vid->frame_statistic_start)
  {
    fprintf(p_stat_frm, "|     ver     | Date  | Time  |    Sequence                  |Frm | QP |P/MbInt|   Bits   |  SNRY  |  SNRU  |  SNRV  |  I4  |  I8  | I16  | IC0  | IC1  | IC2  | IC3  | PI4  | PI8  | PI16 |  P0  |  P1  |  P2  |  P3  | P1*4*| P1*8*| P2*4*| P2*8*| P3*4*| P3*8*|  P8  | P8:4 | P4*4*| P4*8*| P8:5 | P8:6 | P8:7 | BI4  | BI8  | BI16 |  B0  |  B1  |  B2  |  B3  | B0*4*| B0*8*| B1*4*| B1*8*| B2*4*| B2*8*| B3*4*| B3*8*|  B8  | B8:0 |B80*4*|B80*8*| B8:4 | B4*4*| B4*8*| B8:5 | B8:6 | B8:7 |\n");
    fprintf(p_stat_frm, " ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ \n");
  }

  //report
  fprintf(p_stat_frm, "|%4s/%s", VERSION, EXT_VERSION);

#ifdef WIN32
  _strdate( timebuf );
  fprintf(p_stat_frm, "| %1.5s |", timebuf);

  _strtime( timebuf);
  fprintf(p_stat_frm, " % 1.5s |", timebuf);
#else
  now = time ((time_t *) NULL); // Get the system time and put it into 'now' as 'calender time'
  time (&now);
  l_time = localtime (&now);
  strftime (string, sizeof string, "%d-%b-%Y", l_time);
  fprintf(p_stat_frm, "| %1.5s |", string );

  strftime (string, sizeof string, "%H:%M:%S", l_time);
  fprintf(p_stat_frm, " %1.5s |", string);
#endif

  for (i=0;i<30;i++)
    name[i] = p_Inp->input_file1.fname[i + imax(0,(int) (strlen(p_Inp->input_file1.fname)- 30))]; // write last part of path, max 30 chars

  fprintf(p_stat_frm, "%30.30s|", name);
  fprintf(p_stat_frm, "%3d |", p_Vid->frame_no);
  fprintf(p_stat_frm, "%3d |", p_Vid->qp);
  fprintf(p_stat_frm, "  %d/%d  |", p_Inp->PicInterlace, p_Inp->MbInterlace);

#if (MVC_EXTENSION_ENABLE)
  if (p_Vid->curr_frm_idx == 0 && p_Vid->frame_num == 0 && !p_Vid->view_id)
#else
  if (p_Vid->curr_frm_idx == 0 && p_Vid->frame_num == 0)
#endif
  {
    bitcounter = (int) p_Vid->p_Stats->bit_counter[I_SLICE];
  }
  else
  {
    bitcounter = (int) (p_Vid->p_Stats->bit_ctr_n - p_Vid->last_bit_ctr_n);
    p_Vid->last_bit_ctr_n = p_Vid->p_Stats->bit_ctr_n;
  }

  //report bitrate
  fprintf(p_stat_frm, " %9d|", bitcounter);

  //report snr's  
  fprintf(p_stat_frm, " %2.4f| %2.4f| %2.4f|", p_Dist->metric[PSNR].value[0], p_Dist->metric[PSNR].value[1], p_Dist->metric[PSNR].value[2]);

  //report modes
  //I-Modes
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[I_SLICE][I4MB ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[I_SLICE][I8MB ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[I_SLICE][I16MB]);

  //chroma intra mode
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->intra_chroma_mode[0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->intra_chroma_mode[1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->intra_chroma_mode[2]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->intra_chroma_mode[3]);

  //P-Modes
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][I4MB ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][I8MB ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][I16MB]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][0    ]);

  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][1    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][2    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][3    ]);
  
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[P_SLICE][1][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[P_SLICE][1][1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[P_SLICE][2][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[P_SLICE][2][1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[P_SLICE][3][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[P_SLICE][3][1]);

  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][P8x8 ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][4    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[P_SLICE][4][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[P_SLICE][4][1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][5    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][6    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[P_SLICE][7    ]);

  //B-Modes
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][I4MB ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][I8MB ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][I16MB]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][0    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][1    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][2    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][3    ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][0][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][0][1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][1][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][1][1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][2][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][2][1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][3][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][3][1]);

  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][P8x8]);
  fprintf(p_stat_frm, " %d|", (cur_stats->b8_mode_0_use [B_SLICE][0] + cur_stats->b8_mode_0_use [B_SLICE][1]));
  fprintf(p_stat_frm, " %5d|", cur_stats->b8_mode_0_use [B_SLICE][0]);
  fprintf(p_stat_frm, " %5d|", cur_stats->b8_mode_0_use [B_SLICE][1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][4   ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][4][0]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use_transform[B_SLICE][4][1]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][5   ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][6   ]);
  fprintf(p_stat_frm, " %5" FORMAT_OFF_T  "|", cur_stats->mode_use[B_SLICE][7   ]);

  fprintf(p_stat_frm, "\n");

  //save the last results
  p_Vid->frame_statistic_start = 0;
  fclose(p_stat_frm);
}


double report_slice_pred_stats(FILE *p_stat, StatParameters *p_Stats, int slice_type, double bit_use, char *slice_name)
{
  fprintf(p_stat,"\n ---------------------|----------------|-----------------|");
  fprintf(p_stat,"\n   %8s           |   Mode used    | MotionInfo bits |", slice_name);
  fprintf(p_stat,"\n ---------------------|----------------|-----------------|");
  fprintf(p_stat,"\n Mode  0  (copy)      |  %5" FORMAT_OFF_T  "         |    %8.2f     |", p_Stats->mode_use[slice_type][0   ], (double)p_Stats->bit_use_mode[slice_type][0   ] / bit_use);
  fprintf(p_stat,"\n Mode  1  (16x16)     |  %5" FORMAT_OFF_T  "         |    %8.2f     |", p_Stats->mode_use[slice_type][1   ], (double)p_Stats->bit_use_mode[slice_type][1   ] / bit_use);
  fprintf(p_stat,"\n Mode  2  (16x8)      |  %5" FORMAT_OFF_T  "         |    %8.2f     |", p_Stats->mode_use[slice_type][2   ], (double)p_Stats->bit_use_mode[slice_type][2   ] / bit_use);
  fprintf(p_stat,"\n Mode  3  (8x16)      |  %5" FORMAT_OFF_T  "         |    %8.2f     |", p_Stats->mode_use[slice_type][3   ], (double)p_Stats->bit_use_mode[slice_type][3   ] / bit_use);
  fprintf(p_stat,"\n Mode  4  (8x8)       |  %5" FORMAT_OFF_T  "         |    %8.2f     |", p_Stats->mode_use[slice_type][P8x8], (double)p_Stats->bit_use_mode[slice_type][P8x8] / bit_use);
  fprintf(p_stat,"\n Mode  5  intra 4x4   |  %5" FORMAT_OFF_T  "         |-----------------|", p_Stats->mode_use[slice_type][I4MB]);
  fprintf(p_stat,"\n Mode  6  intra 8x8   |  %5" FORMAT_OFF_T  "         |", p_Stats->mode_use[slice_type][I8MB]);
  fprintf(p_stat,"\n Mode  7+ intra 16x16 |  %5" FORMAT_OFF_T  "         |", p_Stats->mode_use[slice_type][I16MB]);
  fprintf(p_stat,"\n Mode     intra IPCM  |  %5" FORMAT_OFF_T  "         |", p_Stats->mode_use[slice_type][IPCM ]);

  return (double)(p_Stats->bit_use_mode[slice_type][0] + p_Stats->bit_use_mode[slice_type][1] + p_Stats->bit_use_mode[slice_type][2]
  + p_Stats->bit_use_mode[slice_type][3] + p_Stats->bit_use_mode[slice_type][P8x8]) / bit_use;
}

/*!
 ***********************************************************************
 * \brief
 *    Terminates and reports statistics on error.
 *
 ***********************************************************************
 */
void report_stats_on_error(void)
{
  free_encoder_memory(p_Enc->p_Vid, p_Enc->p_Inp);
  exit (-1);
}


void report_stats(VideoParameters *p_Vid, InputParameters *p_Inp, StatParameters *p_Stats, int64 bit_use[NUM_SLICE_TYPES][2])
{
  DistortionParams *p_Dist = p_Vid->p_Dist;
  FILE *p_stat;                    //!< status file for the last encoding session
  double mean_motion_info_bit_use[NUM_SLICE_TYPES] = {0.0};
  int i;

  if (strlen(p_Inp->StatsFile) == 0)
    strcpy (p_Inp->StatsFile,"stats.dat");

  if ((p_stat = fopen(p_Inp->StatsFile, "wt")) == 0)
  {
    snprintf(errortext, ET_SIZE, "Error open file %s", p_Inp->StatsFile);
    error(errortext, 500);
  }

  fprintf(p_stat," -------------------------------------------------------------- \n");
  fprintf(p_stat,"  This file contains statistics for the last encoded sequence   \n");
  fprintf(p_stat," -------------------------------------------------------------- \n");
  fprintf(p_stat,   " Sequence                     : %s\n", p_Inp->input_file1.fname);


  fprintf(p_stat,   " No.of coded pictures         : %4d\n", p_Stats->frame_counter);
  fprintf(p_stat,   " Freq. for encoded bitstream  : %4.0f\n", p_Inp->output.frame_rate);

  fprintf(p_stat,   " I Slice Bitrate(kb/s)        : %6.2f\n", p_Stats->bitrate_st[I_SLICE] / 1000.0);
  fprintf(p_stat,   " P Slice Bitrate(kb/s)        : %6.2f\n", p_Stats->bitrate_st[P_SLICE] / 1000.0);
  fprintf(p_stat,   " B Slice Bitrate(kb/s)        : %6.2f\n", p_Stats->bitrate_st[B_SLICE] / 1000.0);
  fprintf(p_stat,   " Total Bitrate(kb/s)          : %6.2f\n", p_Stats->bitrate / 1000.0);

  for (i = 0; i < 3; i++)
  {
    fprintf(p_stat," ME Level %1d Metric            : %s\n", i, DistortionType[p_Inp->MEErrorMetric[i]]);
  }
  fprintf(p_stat," Mode Decision Metric         : %s\n", DistortionType[p_Inp->ModeDecisionMetric]);

  switch ( p_Inp->ChromaMEEnable )
  {
  case 1:
    fprintf(p_stat," ME for components            : YCbCr\n");
    break;
  default:
    fprintf(p_stat," ME for components            : Y\n");
    break;
  }

  fprintf(p_stat,  " Image format                 : %dx%d\n", p_Inp->output.width[0], p_Inp->output.height[0]);

  if (p_Inp->intra_upd)
    fprintf(p_stat," Error robustness             : On\n");
  else
    fprintf(p_stat," Error robustness             : Off\n");

  fprintf(p_stat,  " Search range                 : %d\n", p_Inp->search_range[0]);
#if (MVC_EXTENSION_ENABLE)
  if ( p_Inp->SepViewInterSearch )
  {
    fprintf(p_stat,  " Search range (view 1)        : %d\n", p_Inp->search_range[1]);
  }
#endif

  fprintf(p_stat,   " Total number of references   : %d\n", p_Inp->num_ref_frames_org);
  fprintf(p_stat,   " References for P slices      : %d\n", p_Inp->P_List0_refs_org[0] ? p_Inp->P_List0_refs_org[0] : p_Inp->num_ref_frames_org);

  if (p_Stats->frame_ctr[B_SLICE]!=0)
  {
    fprintf(p_stat, " List0 refs for B slices      : %d\n", p_Inp->B_List0_refs_org[0] ? p_Inp->B_List0_refs_org[0] : p_Inp->num_ref_frames_org);
    fprintf(p_stat, " List1 refs for B slices      : %d\n", p_Inp->B_List1_refs_org[0] ? p_Inp->B_List1_refs_org[0] : p_Inp->num_ref_frames_org);
  }

#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->num_of_layers > 1 )
  {
    fprintf(p_stat,   " View 1 refs for P slices     : %d\n", p_Inp->P_List0_refs_org[1] ? p_Inp->P_List0_refs_org[1] : p_Inp->num_ref_frames_org);
    fprintf(p_stat,   " View 1 L0 refs for B slices  : %d\n", p_Inp->B_List0_refs_org[1] ? p_Inp->B_List0_refs_org[1] : p_Inp->num_ref_frames_org);
    fprintf(p_stat,   " View 1 L1 refs for B slices  : %d\n", p_Inp->B_List1_refs_org[1] ? p_Inp->B_List1_refs_org[1] : p_Inp->num_ref_frames_org);
  }
#endif

  fprintf(p_stat,   " Profile/Level IDC            : (%d,%d)\n", p_Inp->ProfileIDC, p_Inp->LevelIDC);
  if (p_Inp->symbol_mode == CAVLC)
    fprintf(p_stat,   " Entropy coding method        : CAVLC\n");
  else
    fprintf(p_stat,   " Entropy coding method        : CABAC\n");

  if (p_Inp->MbInterlace)
    fprintf(p_stat, " MB Field Coding : On \n");

  if (p_Inp->SearchMode[0] == EPZS || p_Inp->SearchMode[1] == EPZS)
  {
    EPZSOutputStats(p_Inp, p_stat, 1);
  }

  if (p_Inp->full_search == 2)
    fprintf(p_stat," Search range restrictions    : none\n");
  else if (p_Inp->full_search == 1)
    fprintf(p_stat," Search range restrictions    : older reference frames\n");
  else
    fprintf(p_stat," Search range restrictions    : smaller blocks and older reference frames\n");

  if (p_Inp->rdopt)
    fprintf(p_stat," RD-optimized mode decision   : used\n");
  else
    fprintf(p_stat," RD-optimized mode decision   : not used\n");

  fprintf(p_stat,"\n ---------------------|----------------|---------------|");
  fprintf(p_stat,"\n     Item             |     Intra      |   All frames  |");
  fprintf(p_stat,"\n ---------------------|----------------|---------------|");
  fprintf(p_stat,"\n SNR Y(dB)            |");
  fprintf(p_stat," %5.2f          |", p_Dist->metric[PSNR].avslice[I_SLICE][0]);
  fprintf(p_stat," %5.2f         |", p_Dist->metric[PSNR].average[0]);
  fprintf(p_stat,"\n SNR U/V (dB)         |");
  fprintf(p_stat," %5.2f/%5.2f    |", p_Dist->metric[PSNR].avslice[I_SLICE][1], p_Dist->metric[PSNR].avslice[I_SLICE][2]);
  fprintf(p_stat," %5.2f/%5.2f   |", p_Dist->metric[PSNR].average[1], p_Dist->metric[PSNR].average[2]);
  fprintf(p_stat,"\n ---------------------|----------------|---------------|");
  fprintf(p_stat,"\n");

  fprintf(p_stat,"\n ---------------------|----------------|---------------|---------------|");
  fprintf(p_stat,"\n     SNR              |        I       |       P       |       B       |");
  fprintf(p_stat,"\n ---------------------|----------------|---------------|---------------|");
  fprintf(p_stat,"\n SNR Y(dB)            |      %5.3f    |     %5.3f    |     %5.3f    |",
    p_Dist->metric[PSNR].avslice[I_SLICE][0], p_Dist->metric[PSNR].avslice[P_SLICE][0], p_Dist->metric[PSNR].avslice[B_SLICE][0]);
  fprintf(p_stat,"\n SNR U(dB)            |      %5.3f    |     %5.3f    |     %5.3f    |",
    p_Dist->metric[PSNR].avslice[I_SLICE][1], p_Dist->metric[PSNR].avslice[P_SLICE][1], p_Dist->metric[PSNR].avslice[B_SLICE][1]);
  fprintf(p_stat,"\n SNR V(dB)            |      %5.3f    |     %5.3f    |     %5.3f    |",
    p_Dist->metric[PSNR].avslice[I_SLICE][2], p_Dist->metric[PSNR].avslice[P_SLICE][2], p_Dist->metric[PSNR].avslice[B_SLICE][2]);
  fprintf(p_stat,"\n ---------------------|----------------|---------------|---------------|");  
  fprintf(p_stat,"\n");

  // QUANT.
  fprintf(p_stat,"\n ---------------------|----------------|---------------|---------------|");
  fprintf(p_stat,"\n     Ave Quant        |        I       |       P       |       B       |");
  fprintf(p_stat,"\n ---------------------|----------------|---------------|---------------|");
  fprintf(p_stat,"\n        QP            |      %5.3f    |     %5.3f    |     %5.3f    |",
    (float)p_Stats->quant[I_SLICE]/dmax(1.0,(float)p_Stats->num_macroblocks[I_SLICE]),
    (float)p_Stats->quant[P_SLICE]/dmax(1.0,(float)p_Stats->num_macroblocks[P_SLICE]),
    (float)p_Stats->quant[B_SLICE]/dmax(1.0,(float)p_Stats->num_macroblocks[B_SLICE]));
  fprintf(p_stat,"\n ---------------------|----------------|---------------|---------------|");  
  fprintf(p_stat,"\n");

  // MODE
  fprintf(p_stat,"\n ---------------------|----------------|");
  fprintf(p_stat,"\n   Intra              |   Mode used    |");
  fprintf(p_stat,"\n ---------------------|----------------|");
  fprintf(p_stat,"\n Mode 0  intra 4x4    |  %5" FORMAT_OFF_T  "         |", p_Stats->mode_use[I_SLICE][I4MB ]);
  fprintf(p_stat,"\n Mode 1  intra 8x8    |  %5" FORMAT_OFF_T  "         |", p_Stats->mode_use[I_SLICE][I8MB ]);
  fprintf(p_stat,"\n Mode 2+ intra 16x16  |  %5" FORMAT_OFF_T  "         |", p_Stats->mode_use[I_SLICE][I16MB]);
  fprintf(p_stat,"\n Mode    intra IPCM   |  %5" FORMAT_OFF_T  "         |", p_Stats->mode_use[I_SLICE][IPCM ]);

  // P slices
  if (p_Stats->frame_ctr[P_SLICE]!=0)
  {    
    mean_motion_info_bit_use[P_SLICE] = report_slice_pred_stats(p_stat, p_Stats, P_SLICE,(double) bit_use[P_SLICE][0], "P Slice ");
  }
  // B slices
  if (p_Stats->frame_ctr[B_SLICE]!=0)
  {
    mean_motion_info_bit_use[B_SLICE] = report_slice_pred_stats(p_stat, p_Stats, B_SLICE,(double) bit_use[B_SLICE][0], "B Slice ");
  }
  // SP slices
  if (p_Stats->frame_ctr[SP_SLICE]!=0)
  {    
    mean_motion_info_bit_use[SP_SLICE] = report_slice_pred_stats(p_stat, p_Stats, SP_SLICE,(double) bit_use[SP_SLICE][0], "SP Slice");
  }


  fprintf(p_stat,"\n ---------------------|----------------|");
  fprintf(p_stat,"\n");

  fprintf(p_stat,"\n ---------------------|----------------|----------------|----------------|----------------|");
  fprintf(p_stat,"\n  Bit usage:          |      Intra     |      Inter     |    B frame     |    SP frame    |");
  fprintf(p_stat,"\n ---------------------|----------------|----------------|----------------|----------------|");

  fprintf(p_stat,"\n Header               |");
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_header[I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_header[P_SLICE] / bit_use[P_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_header[B_SLICE] / bit_use[B_SLICE][0]);

  fprintf(p_stat,"\n Mode                 |");
  fprintf(p_stat," %10.2f     |", (float)p_Stats->bit_use_mb_type[I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float)p_Stats->bit_use_mb_type[P_SLICE] / bit_use[P_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float)p_Stats->bit_use_mb_type[B_SLICE] / bit_use[B_SLICE][0]);

  fprintf(p_stat,"\n Motion Info          |");
  fprintf(p_stat,"        ./.     |");
  fprintf(p_stat," %10.2f     |", mean_motion_info_bit_use[P_SLICE]);
  fprintf(p_stat," %10.2f     |", mean_motion_info_bit_use[B_SLICE]);

  fprintf(p_stat,"\n CBP Y/C              |");
  fprintf(p_stat," %10.2f     |", (float) p_Stats->tmp_bit_use_cbp[I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->tmp_bit_use_cbp[P_SLICE] / bit_use[P_SLICE][0]);   
  fprintf(p_stat," %10.2f     |", (float) p_Stats->tmp_bit_use_cbp[B_SLICE] / bit_use[B_SLICE][0]);

  // Print SP_SLICE
  fprintf(p_stat,"\n Coeffs. Y            |");
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[0][I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[0][P_SLICE] / bit_use[P_SLICE][0]);   
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[0][B_SLICE] / bit_use[B_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[0][SP_SLICE] / bit_use[SP_SLICE][0]);   

  fprintf(p_stat,"\n Coeffs. C            |");
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeffC[I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeffC[P_SLICE] / bit_use[P_SLICE][0]);   
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeffC[B_SLICE] / bit_use[B_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeffC[SP_SLICE] / bit_use[SP_SLICE][0]);   

  fprintf(p_stat,"\n Coeffs. CB           |");
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[1][I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[1][P_SLICE] / bit_use[P_SLICE][0]);   
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[1][B_SLICE] / bit_use[B_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[1][SP_SLICE] / bit_use[SP_SLICE][0]);   
  
  fprintf(p_stat,"\n Coeffs. CR           |");
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[2][I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[2][P_SLICE] / bit_use[P_SLICE][0]);   
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[2][B_SLICE] / bit_use[B_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_coeff[2][SP_SLICE] / bit_use[SP_SLICE][0]);   

  fprintf(p_stat,"\n Delta quant          |");
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_delta_quant[I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_delta_quant[P_SLICE] / bit_use[P_SLICE][0]);   
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_delta_quant[B_SLICE] / bit_use[B_SLICE][0]);

  fprintf(p_stat,"\n Stuffing Bits        |");
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_stuffing_bits[I_SLICE] / bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_stuffing_bits[P_SLICE] / bit_use[P_SLICE][0]);   
  fprintf(p_stat," %10.2f     |", (float) p_Stats->bit_use_stuffing_bits[B_SLICE] / bit_use[B_SLICE][0]);

  fprintf(p_stat,"\n ---------------------|----------------|----------------|----------------|");
  fprintf(p_stat,"\n average bits/frame   |");
  fprintf(p_stat," %10.2f     |", (float) bit_use[I_SLICE][1] / (float) bit_use[I_SLICE][0] );
  fprintf(p_stat," %10.2f     |", (float) bit_use[P_SLICE][1] / (float) bit_use[P_SLICE][0] );
  fprintf(p_stat," %10.2f     |", (float) bit_use[B_SLICE][1] / (float) bit_use[B_SLICE][0] );
  fprintf(p_stat,"\n ---------------------|----------------|----------------|----------------|");
  fprintf(p_stat,"\n");

  fclose(p_stat);
}


void report_log(VideoParameters *p_Vid, InputParameters *p_Inp, StatParameters *p_Stats)
{
  DistortionParams *p_Dist = p_Vid->p_Dist;
  FILE *p_log = p_Vid->p_log;
  char name[40];
  int i;
#ifndef WIN32
  time_t now;
  struct tm *l_time;
  char string[1000];
#else
  char timebuf[128];
#endif

  if ((p_log = fopen("log.dat", "r")) == 0)         // check if file exists
  {
    if ((p_log = fopen("log.dat", "a")) == NULL)    // append new statistic at the end
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n", "log.dat");
      error(errortext, 500);
    }
    else                                            // Create header for new log file
    {
      fprintf(p_log," ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ \n");
      fprintf(p_log,"|                          Encoder statistics. This file is generated during first encoding session, new sessions will be appended                                                                                                                                                                                 |\n");
      fprintf(p_log," ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ \n");
      fprintf(p_log,"|     ver     | Date  | Time  |               Sequence                 | #Img |P/MbInt| QPI| QPP| QPB| Format  |Iperiod| #B | FMES | Hdmd | S.R |#Ref | Freq |Coding|RD-opt|Intra upd|8x8Tr| SNRY 1| SNRU 1| SNRV 1| SNRY N| SNRU N| SNRV N|#Bitr I|#Bitr P|#Bitr B|#Bitr IPB|     Total Time   |      Me Time     |\n");
      fprintf(p_log," ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ \n");

    }
  }
  else
  {
    fclose (p_log);
    if ((p_log = fopen("log.dat", "a")) == NULL)         // File exists, just open for appending
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n", "log.dat");
      error(errortext, 500);
    }
  }
  fprintf(p_log,"|%5s/%-5s", VERSION, EXT_VERSION);

#ifdef WIN32
  _strdate( timebuf );
  fprintf(p_log,"| %1.5s |", timebuf );

  _strtime( timebuf);
  fprintf(p_log," % 1.5s |", timebuf);
#else
  now = time ((time_t *) NULL); // Get the system time and put it into 'now' as 'calender time'
  time (&now);
  l_time = localtime (&now);
  strftime (string, sizeof string, "%d-%b-%Y", l_time);
  fprintf(p_log,"| %1.5s |", string );

  strftime (string, sizeof string, "%H:%M:%S", l_time);
  fprintf(p_log," %1.5s |", string );
#endif

  for (i=0; i < 40; i++)
    name[i] = p_Inp->input_file1.fname[i + imax(0, ((int) strlen(p_Inp->input_file1.fname)) - 40)]; // write last part of path, max 40 chars
  fprintf(p_log,"%40.40s|",name);

  fprintf(p_log,"%5d |  %d/%d  |", p_Inp->no_frames, p_Inp->PicInterlace, p_Inp->MbInterlace);
  fprintf(p_log," %-3d| %-3d| %-3d|", p_Inp->qp[I_SLICE], p_Inp->qp[P_SLICE], p_Inp->qp[B_SLICE]);

  fprintf(p_log,"%4dx%-4d|", p_Inp->output.width[0], p_Inp->output.height[0]);
  fprintf(p_log,"  %3d  |%3d |", p_Inp->intra_period, p_Stats->NumberBFrames);


  switch( p_Inp->SearchMode[0] ) 
  {
  case UM_HEX:
    fprintf(p_log,"  HEX |");
    break;
  case UM_HEX_SIMPLE:
    fprintf(p_log," SHEX |");
    break;
  case EPZS:
    fprintf(p_log," EPZS |");
    break;
  case FAST_FULL_SEARCH:
    fprintf(p_log,"  FFS |");
    break;
  default:
    fprintf(p_log,"  FS  |");
    break;
  }
#if (MVC_EXTENSION_ENABLE)
  if ( p_Inp->SepViewInterSearch )
  {
    switch( p_Inp->SearchMode[1] ) 
    {
    case UM_HEX:
      fprintf(p_log,"  HEX |");
      break;
    case UM_HEX_SIMPLE:
      fprintf(p_log," SHEX |");
      break;
    case EPZS:
      fprintf(p_log," EPZS |");
      break;
    case FAST_FULL_SEARCH:
      fprintf(p_log,"  FFS |");
      break;
    default:
      fprintf(p_log,"  FS  |");
      break;
    }
  }
#endif

  fprintf(p_log,"  %1d%1d%1d |", p_Inp->MEErrorMetric[F_PEL], p_Inp->MEErrorMetric[H_PEL], p_Inp->MEErrorMetric[Q_PEL]);

#if (MVC_EXTENSION_ENABLE)
  if ( p_Inp->SepViewInterSearch )
  {
    fprintf(p_log," %3d (%3d) | %2d  |", p_Inp->search_range[0], p_Inp->search_range[1], p_Inp->num_ref_frames );
  }
  else
#endif
  fprintf(p_log," %3d | %2d  |", p_Inp->search_range[0], p_Inp->num_ref_frames );

  fprintf(p_log," %5.2f|", p_Vid->framerate);

  if (p_Inp->symbol_mode == CAVLC)
    fprintf(p_log," CAVLC|");
  else
    fprintf(p_log," CABAC|");

  fprintf(p_log,"   %d  |", p_Inp->rdopt);

  if (p_Inp->intra_upd == 1)
    fprintf(p_log,"   ON    |");
  else
    fprintf(p_log,"   OFF   |");

  fprintf(p_log,"  %d  |", p_Inp->Transform8x8Mode);

  fprintf(p_log,"%7.3f|%7.3f|%7.3f|", 
    p_Dist->metric[PSNR].avslice[I_SLICE][0],
    p_Dist->metric[PSNR].avslice[I_SLICE][1],
    p_Dist->metric[PSNR].avslice[I_SLICE][2]);
  fprintf(p_log,"%7.3f|%7.3f|%7.3f|", p_Dist->metric[PSNR].average[0],p_Dist->metric[PSNR].average[1],p_Dist->metric[PSNR].average[2]);
  /*
  fprintf(p_log,"%-5.3f|%-5.3f|%-5.3f|", p_Dist->metric[PSNR].avslice[I_SLICE][0], p_Dist->metric[PSNR].avslice[I_SLICE][1], p_Dist->metric[PSNR].avslice[I_SLICE][2]);
  fprintf(p_log,"%-5.3f|%-5.3f|%-5.3f|", p_Dist->metric[PSNR].avslice[P_SLICE][0], p_Dist->metric[PSNR].avslice[P_SLICE][1], p_Dist->metric[PSNR].avslice[P_SLICE][2]);
  fprintf(p_log,"%-5.3f|%-5.3f|%-5.3f|", p_Dist->metric[PSNR].avslice[B_SLICE][0], p_Dist->metric[PSNR].avslice[B_SLICE][1], p_Dist->metric[PSNR].avslice[B_SLICE][2]);
  */
  fprintf(p_log,"%7.0f|%7.0f|%7.0f|%9.0f|", p_Stats->bitrate_st[I_SLICE],p_Stats->bitrate_st[P_SLICE],p_Stats->bitrate_st[B_SLICE], p_Stats->bitrate);
  fprintf(p_log,"   %12d   |   %12d   |", (int)p_Vid->tot_time,(int)p_Vid->me_tot_time);


  fprintf(p_log,"\n");

  fclose(p_log);

  p_log = fopen("data.txt", "a");

  if ((p_Stats->NumberBFrames != 0) && (p_Stats->frame_ctr[B_SLICE] != 0)) // B picture used
  {
    fprintf(p_log, "%3d %2d %2d %2.2f %2.2f %2.2f %5" FORMAT_OFF_T  " "
      "%2.2f %2.2f %2.2f %5d "
      "%2.2f %2.2f %2.2f %5" FORMAT_OFF_T  " %5" FORMAT_OFF_T  " %.3f\n",
      p_Stats->frame_counter, p_Inp->qp[I_SLICE], p_Inp->qp[P_SLICE],
      p_Dist->metric[PSNR].avslice[I_SLICE][0],
      p_Dist->metric[PSNR].avslice[I_SLICE][1],
      p_Dist->metric[PSNR].avslice[I_SLICE][2],
      p_Stats->bit_counter[I_SLICE],
      0.0,
      0.0,
      0.0,
      0,
      p_Dist->metric[PSNR].average[0],
      p_Dist->metric[PSNR].average[1],
      p_Dist->metric[PSNR].average[2],
      (p_Stats->bit_counter[I_SLICE] + p_Stats->bit_ctr) / p_Stats->frame_counter,
      p_Stats->bit_counter[B_SLICE] / p_Stats->frame_ctr[B_SLICE],
      (double) 0.001 * p_Vid->tot_time / (p_Stats->frame_counter));
  }
  else
  {
    if (p_Inp->no_frames != 0)
      fprintf(p_log, "%3d %2d %2d %2.2f %2.2f %2.2f %5" FORMAT_OFF_T  " "
      "%2.2f %2.2f %2.2f %5d "
      "%2.2f %2.2f %2.2f %5" FORMAT_OFF_T  " %5d %.3f\n",
      p_Stats->frame_counter, p_Inp->qp[I_SLICE], p_Inp->qp[P_SLICE],
      p_Dist->metric[PSNR].avslice[I_SLICE][0],
      p_Dist->metric[PSNR].avslice[I_SLICE][1],
      p_Dist->metric[PSNR].avslice[I_SLICE][2],
      p_Stats->bit_counter[I_SLICE],
      0.0,
      0.0,
      0.0,
      0,
      p_Dist->metric[PSNR].average[0],
      p_Dist->metric[PSNR].average[1],
      p_Dist->metric[PSNR].average[2],
      (p_Stats->bit_counter[I_SLICE] + p_Stats->bit_ctr)/ p_Stats->frame_counter,
      0,
      (double)0.001 * p_Vid->tot_time / p_Stats->frame_counter);
  }

  fclose(p_log);
}

/*!
 ************************************************************************
 * \brief
 *    Reports the gathered information to appropriate outputs
 * \par Input:
 *    InputParameters *inp,                                            \n
 *    VideoParameters *p_Vid,                                            \n
 *    struct stat_par *p_Stats,                                          \n
 *
 * \par Output:
 *    None
 ************************************************************************
 */
void report( VideoParameters *p_Vid, InputParameters *p_Inp, StatParameters *p_Stats)
{
  DistortionParams *p_Dist = p_Vid->p_Dist;
  int64 bit_use[NUM_SLICE_TYPES][2];
  int i,j;
  int64 total_bits;
#if (MVC_EXTENSION_ENABLE)
  int64 total_bits_v[NUM_VIEWS];
#endif

  bit_use[ I_SLICE][0] = p_Stats->frame_ctr[I_SLICE];
  bit_use[ P_SLICE][0] = imax(p_Stats->frame_ctr[P_SLICE ] + p_Stats->frame_ctr[SP_SLICE], 1);
  bit_use[ B_SLICE][0] = imax(p_Stats->frame_ctr[B_SLICE ], 1);
  bit_use[SP_SLICE][0] = imax(p_Stats->frame_ctr[SP_SLICE], 1);

  // normalize time p_Stats
  p_Vid->tot_time    = timenorm(p_Vid->tot_time);
  p_Vid->me_tot_time = timenorm(p_Vid->me_tot_time);
  //  Accumulate bit usage for inter and intra frames
  for (j=0; j < NUM_SLICE_TYPES; j++)
  {
    bit_use[j][1] = 0;
  }

  for (j=0; j < NUM_SLICE_TYPES; j++)
  {
    for(i=0; i < MAXMODE; i++)
      bit_use[j][1] += p_Stats->bit_use_mode[j][i];

    bit_use[j][1] += p_Stats->bit_use_mb_type[j];
    bit_use[j][1] += p_Stats->bit_use_header[j];    
    bit_use[j][1] += p_Stats->tmp_bit_use_cbp[j];
    bit_use[j][1] += p_Stats->bit_use_coeffC[j];
    bit_use[j][1] += p_Stats->bit_use_coeff[0][j];   
    bit_use[j][1] += p_Stats->bit_use_coeff[1][j]; 
    bit_use[j][1] += p_Stats->bit_use_coeff[2][j]; 
    bit_use[j][1] += p_Stats->bit_use_delta_quant[j];
    bit_use[j][1] += p_Stats->bit_use_stuffing_bits[j];
  }

  //! Currently adding NVB bits on P rate. Maybe additional p_Stats info should be created instead and added in log file  
  p_Stats->bitrate_st[ I_SLICE] = (p_Stats->bit_counter[ I_SLICE]) * (float) (p_Inp->output.frame_rate) / (float) (p_Stats->frame_counter);
  p_Stats->bitrate_st[ P_SLICE] = (p_Stats->bit_counter[ P_SLICE]) * (float) (p_Inp->output.frame_rate) / (float) (p_Stats->frame_counter);
  p_Stats->bitrate_st[ B_SLICE] = (p_Stats->bit_counter[ B_SLICE]) * (float) (p_Inp->output.frame_rate) / (float) (p_Stats->frame_counter);
  p_Stats->bitrate_st[SP_SLICE] = (p_Stats->bit_counter[SP_SLICE]) * (float) (p_Inp->output.frame_rate) / (float) (p_Stats->frame_counter);

  switch (p_Inp->Verbose)
  {
  case 0:
  case 1:  
  default:
    fprintf(stdout,"------------------ Average data all frames  -----------------------------------\n\n");
    break;
  case 2:
    fprintf(stdout,"------------------------------------  Average data all frames  ---------------------------------\n\n");
    break;
  case 3:
    fprintf(stdout,"---------------------------------------  Average data all frames  -------------------------------------\n\n");
    break;
  }

  if (p_Inp->Verbose != 0)
  {
    DistMetric *snr     = &p_Dist->metric[PSNR    ];
    DistMetric *sse     = &p_Dist->metric[SSE     ];
    DistMetric *snr_rgb = &p_Dist->metric[PSNR_RGB];
    DistMetric *sse_rgb = &p_Dist->metric[SSE_RGB ];

    int  impix    = p_Inp->output.size_cmp[0];
    int  impix_cr = p_Inp->output.size_cmp[1];

    float csnr_y = psnr(p_Vid->max_imgpel_value_comp_sq[0], impix   , sse->average[0]);
    float csnr_u = psnr(p_Vid->max_imgpel_value_comp_sq[1], impix_cr, sse->average[1]);
    float csnr_v = psnr(p_Vid->max_imgpel_value_comp_sq[2], impix_cr, sse->average[2]);

#if (MVC_EXTENSION_ENABLE)
    DistMetric *snr_v[2] = {NULL, NULL};
    DistMetric *sse_v[2] = {NULL, NULL};
    float csnr_y_v[2] = {0.0, 0.0};
    float csnr_u_v[2] = {0.0, 0.0};
    float csnr_v_v[2] = {0.0, 0.0};

    if (p_Inp->num_of_views == 2)
    {
      snr_v[0] = &p_Dist->metric_v[0][PSNR];
      snr_v[1] = &p_Dist->metric_v[1][PSNR];
      sse_v[0] = &p_Dist->metric_v[0][SSE];
      sse_v[1] = &p_Dist->metric_v[1][SSE];
      csnr_y_v[0] = psnr(p_Vid->max_imgpel_value_comp_sq[0], impix   , sse_v[0]->average[0]);
      csnr_u_v[0] = psnr(p_Vid->max_imgpel_value_comp_sq[1], impix_cr, sse_v[0]->average[1]);
      csnr_v_v[0] = psnr(p_Vid->max_imgpel_value_comp_sq[2], impix_cr, sse_v[0]->average[2]);
      csnr_y_v[1] = psnr(p_Vid->max_imgpel_value_comp_sq[0], impix   , sse_v[1]->average[0]);
      csnr_u_v[1] = psnr(p_Vid->max_imgpel_value_comp_sq[1], impix_cr, sse_v[1]->average[1]);
      csnr_v_v[1] = psnr(p_Vid->max_imgpel_value_comp_sq[2], impix_cr, sse_v[1]->average[2]);
    }
#endif

    fprintf(stdout,  " Total encoding time for the seq.  : %7.3f sec (%3.2f fps)\n", (float) p_Vid->tot_time * 0.001, 1000.0 * (float) (p_Stats->frame_counter) / (float)p_Vid->tot_time);
    fprintf(stdout,  " Total ME time for sequence        : %7.3f sec \n\n", (float)p_Vid->me_tot_time * 0.001);

    fprintf(stdout," Y { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n", 
      snr->average[0], csnr_y, sse->average[0]/(float)impix);
    fprintf(stdout," U { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n",
      snr->average[1], csnr_u, sse->average[1]/(float)impix_cr);
    fprintf(stdout," V { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n",
      snr->average[2], csnr_v, sse->average[2]/(float)impix_cr);
#if (MVC_EXTENSION_ENABLE)
    if (p_Inp->num_of_views == 2)
    {
      fprintf( stdout, "\n" );
      fprintf(stdout," View0_Y { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n", 
        snr_v[0]->average[0], csnr_y_v[0], sse_v[0]->average[0]/(float)impix);
      fprintf(stdout," View0_U { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n",
        snr_v[0]->average[1], csnr_u_v[0], sse_v[0]->average[1]/(float)impix_cr);
      fprintf(stdout," View0_V { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n",
        snr_v[0]->average[2], csnr_v_v[0], sse_v[0]->average[2]/(float)impix_cr);
      fprintf( stdout, "\n" );
      fprintf(stdout," View1_Y { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n", 
        snr_v[1]->average[0], csnr_y_v[1], sse_v[1]->average[0]/(float)impix);
      fprintf(stdout," View1_U { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n",
        snr_v[1]->average[1], csnr_u_v[1], sse_v[1]->average[1]/(float)impix_cr);
      fprintf(stdout," View1_V { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n",
        snr_v[1]->average[2], csnr_v_v[1], sse_v[1]->average[2]/(float)impix_cr);
    }
#endif

    if(p_Inp->DistortionYUVtoRGB == 1)
    {
      float csnr_r = psnr(p_Vid->max_imgpel_value_comp_sq[0], impix, sse_rgb->average[0]);
      float csnr_g = psnr(p_Vid->max_imgpel_value_comp_sq[1], impix, sse_rgb->average[1]);
      float csnr_b = psnr(p_Vid->max_imgpel_value_comp_sq[2], impix, sse_rgb->average[2]);

      fprintf(stdout," R { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n", 
        snr_rgb->average[0], csnr_r, sse_rgb->average[0] / (float) impix);
      fprintf(stdout," G { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n",
        snr_rgb->average[1], csnr_g, sse_rgb->average[1] / (float) impix);
      fprintf(stdout," B { PSNR (dB), cSNR (dB), MSE }   : { %7.3f, %7.3f, %9.5f }\n",
        snr_rgb->average[2], csnr_b, sse_rgb->average[2] / (float) impix);
    }

    if (p_Inp->Distortion[SSIM] == 1)
    {
      if(p_Inp->DistortionYUVtoRGB == 1)
      {
        fprintf(stdout," SSIM {Y, R}                       : { %5.4f, %5.4f }\n", p_Dist->metric[SSIM].average[0], p_Dist->metric[SSIM_RGB].average[0]);
        fprintf(stdout," SSIM {U, G}                       : { %5.4f, %5.4f }\n", p_Dist->metric[SSIM].average[1], p_Dist->metric[SSIM_RGB].average[1]);
        fprintf(stdout," SSIM {V, B}                       : { %5.4f, %5.4f }\n", p_Dist->metric[SSIM].average[2], p_Dist->metric[SSIM_RGB].average[2]);
      }
      else
      {
        fprintf(stdout," Y SSIM                            : %5.4f\n", p_Dist->metric[SSIM].average[0]);
        fprintf(stdout," U SSIM                            : %5.4f\n", p_Dist->metric[SSIM].average[1]);
        fprintf(stdout," V SSIM                            : %5.4f\n", p_Dist->metric[SSIM].average[2]);
      }
    }
    if (p_Inp->Distortion[MS_SSIM] == 1)
    {
      if(p_Inp->DistortionYUVtoRGB == 1)
      {
        fprintf(stdout," MS-SSIM {Y, R}                    : { %5.4f, %5.4f }\n", p_Dist->metric[MS_SSIM].average[0], p_Dist->metric[MS_SSIM_RGB].average[0]);
        fprintf(stdout," MS-SSIM {U, G}                    : { %5.4f, %5.4f }\n", p_Dist->metric[MS_SSIM].average[1], p_Dist->metric[MS_SSIM_RGB].average[1]);
        fprintf(stdout," MS-SSIM {V, B}                    : { %5.4f, %5.4f }\n", p_Dist->metric[MS_SSIM].average[2], p_Dist->metric[MS_SSIM_RGB].average[2]);
      }
      else
      {
        fprintf(stdout," Y MS-SSIM                         : %5.4f\n", p_Dist->metric[MS_SSIM].average[0]);
        fprintf(stdout," U MS-SSIM                         : %5.4f\n", p_Dist->metric[MS_SSIM].average[1]);
        fprintf(stdout," V MS-SSIM                         : %5.4f\n", p_Dist->metric[MS_SSIM].average[2]);
      }
    }
    fprintf(stdout,"\n");
  }
  else
    fprintf(stdout,  " Total encoding time for the seq.  : %5.3f sec (%5.2f fps)\n\n", p_Vid->tot_time * 0.001, 1000.0 * (p_Stats->frame_counter) / p_Vid->tot_time);

  total_bits = p_Stats->bit_ctr_parametersets;
  total_bits += p_Stats->bit_ctr_filler_data;
  for (i = 0; i < NUM_SLICE_TYPES; i++)
    total_bits += p_Stats->bit_counter[i];
#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->num_of_views == 2)
  {
    for ( j = 0; j < NUM_VIEWS; j++ )
    {
      total_bits_v[j] = p_Stats->bit_ctr_parametersets_v[j];
      total_bits_v[j] += p_Stats->bit_ctr_filler_data_v[j];
      for (i = 0; i < NUM_SLICE_TYPES; i++)
        total_bits_v[j] += p_Stats->bit_counter_v[j][i];
    }
  }

  if (p_Inp->num_of_views == 2 && p_Vid->fld_flag)
  {
    if (p_Stats->frame_ctr[B_SLICE] != 0)
    {
      fprintf(stdout, " Total bits                        : %" FORMAT_OFF_T  " (I %" FORMAT_OFF_T  ", P %" FORMAT_OFF_T  ", B %" FORMAT_OFF_T  " NVB %d) \n",
        total_bits_v[0] + total_bits_v[1], 
        p_Stats->bit_counter_v[0][I_SLICE] + p_Stats->bit_counter_v[1][I_SLICE],
        p_Stats->bit_counter_v[0][P_SLICE] + p_Stats->bit_counter_v[1][P_SLICE], 
        p_Stats->bit_counter_v[0][B_SLICE] + p_Stats->bit_counter_v[1][B_SLICE], 
        p_Stats->bit_ctr_parametersets_v[0] + p_Stats->bit_ctr_parametersets_v[1]);
    }
    else
    {
      fprintf(stdout, " Total bits                        : %" FORMAT_OFF_T  " (I %" FORMAT_OFF_T  ", P %" FORMAT_OFF_T  ", NVB %d) \n",
        total_bits_v[0] + total_bits_v[1], 
        p_Stats->bit_counter_v[0][I_SLICE] + p_Stats->bit_counter_v[1][I_SLICE],
        p_Stats->bit_counter_v[0][P_SLICE] + p_Stats->bit_counter_v[1][P_SLICE], 
        p_Stats->bit_ctr_parametersets_v[0] + p_Stats->bit_ctr_parametersets_v[1]);
    }
  }
  else
#endif
  {
  if (p_Stats->frame_ctr[B_SLICE] != 0)
  {
    fprintf(stdout, " Total bits                        : %" FORMAT_OFF_T  " (I %" FORMAT_OFF_T  ", P %" FORMAT_OFF_T  ", B %" FORMAT_OFF_T  " NVB %d) \n",
      total_bits, p_Stats->bit_counter[I_SLICE], p_Stats->bit_counter[P_SLICE], p_Stats->bit_counter[B_SLICE], p_Stats->bit_ctr_parametersets);
  }
  else
  {
    fprintf(stdout, " Total bits                        : %" FORMAT_OFF_T  " (I %" FORMAT_OFF_T  ", P %" FORMAT_OFF_T  ", NVB %d) \n",
      total_bits, p_Stats->bit_counter[I_SLICE], p_Stats->bit_counter[P_SLICE], p_Stats->bit_ctr_parametersets);
  }
  }
#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->num_of_views == 2)
  {
    int view_id;

    for ( view_id = 0; view_id < NUM_VIEWS; view_id++ )
    {
      if (p_Stats->frame_ctr[B_SLICE] != 0)
      {
        fprintf(stdout, " View %1d Total-bits                 : %" FORMAT_OFF_T  " (I %" FORMAT_OFF_T  ", P %" FORMAT_OFF_T  ", B %" FORMAT_OFF_T  " NVB %d) \n",
          view_id, total_bits_v[view_id],  p_Stats->bit_counter_v[view_id][I_SLICE], p_Stats->bit_counter_v[view_id][P_SLICE], p_Stats->bit_counter_v[view_id][B_SLICE], p_Stats->bit_ctr_parametersets_v[view_id]);
      }
      else
      {
        fprintf(stdout, " View %1d Total-bits                 : %" FORMAT_OFF_T  " (I %" FORMAT_OFF_T  ", P %" FORMAT_OFF_T  ", NVB %d) \n",
          view_id, total_bits_v[view_id], p_Stats->bit_counter_v[view_id][I_SLICE], p_Stats->bit_counter_v[view_id][P_SLICE], p_Stats->bit_ctr_parametersets_v[view_id]);
      }
    }
  }

  if (p_Inp->num_of_views == 2)
  {
    p_Stats->bitrate= ((float) total_bits * (float) p_Inp->output.frame_rate) / ((float) (p_Stats->frame_counter>>1));
  }
  else
#endif
  p_Stats->bitrate= ((float) total_bits * (float) p_Inp->output.frame_rate) / ((float) (p_Stats->frame_counter));
  fprintf(stdout, " Bit rate (kbit/s)  @ %2.2f Hz     : %5.2f\n", p_Inp->output.frame_rate, p_Stats->bitrate / 1000.0);
#if (MVC_EXTENSION_ENABLE)
  if (p_Inp->num_of_views == 2)
  {
    p_Stats->bitrate_v[0] = ((float) total_bits_v[0] * (float) p_Inp->output.frame_rate) / ((float) (p_Stats->frame_counter >> 1));
    p_Stats->bitrate_v[1] = ((float) total_bits_v[1] * (float) p_Inp->output.frame_rate) / ((float) (p_Stats->frame_counter >> 1));
    fprintf(stdout, " View 0 BR (kbit/s)  @ %2.2f Hz    : %5.2f\n", p_Inp->output.frame_rate, p_Stats->bitrate_v[0] / 1000.0);
    fprintf(stdout, " View 1 BR (kbit/s)  @ %2.2f Hz    : %5.2f\n", p_Inp->output.frame_rate, p_Stats->bitrate_v[1] / 1000.0);
  }
#endif
  
  for (i = 0; i < 5; i++)
  {
    p_Stats->bit_ctr_emulation_prevention += p_Stats->bit_use_stuffing_bits[i];
  }
#if (MVC_EXTENSION_ENABLE)
  if ( p_Vid->num_of_layers == 2 )
  {
    for (i = 0; i < 5; i++)
    {
      p_Stats->bit_ctr_emulationprevention_v[p_Vid->dpb_layer_id] += p_Stats->bit_use_stuffing_bits[i];
    }
  }
#endif

  fprintf(stdout, " Bits to avoid Startcode Emulation : %" FORMAT_OFF_T  " \n", p_Stats->bit_ctr_emulation_prevention);
  fprintf(stdout, " Bits for parameter sets           : %d \n", p_Stats->bit_ctr_parametersets);
  fprintf(stdout, " Bits for filler data              : %" FORMAT_OFF_T  " \n\n", p_Stats->bit_ctr_filler_data);

  switch (p_Inp->Verbose)
  {
  case 0:
  case 1:
  default:
    fprintf(stdout,"-------------------------------------------------------------------------------\n");
    break;
  case 2:  
    fprintf(stdout,"------------------------------------------------------------------------------------------------\n");
    break;
  case 3:
    fprintf(stdout,"-------------------------------------------------------------------------------------------------------\n");
    break;
  }  
  fprintf(stdout,"Exit JM %s encoder ver %s ", JM, VERSION);
  fprintf(stdout,"\n");

  // status file
  report_stats(p_Vid, p_Inp, p_Stats, bit_use);

  // write to log file
  report_log(p_Vid, p_Inp, p_Stats);

  if (p_Inp->ReportFrameStats)
  {
    if ((p_Vid->p_log = fopen("stat_frame.dat", "a")) == NULL)       // append new statistic at the end
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n", "stat_frame.dat.dat");
      //    error(errortext, 500);
    }
    else
    {
      fprintf(p_Vid->p_log," --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
      fclose(p_Vid->p_log);
    }
    report_log_mode(p_Vid, p_Inp, p_Stats);
  }
}


/*!
 ************************************************************************
 * \brief
 *    Prints the header of the protocol.
 * \par Input:
 *    InputParameters *inp
 * \par Output:
 *    none
 ************************************************************************
 */
void information_init ( VideoParameters *p_Vid, InputParameters *p_Inp, StatParameters *p_Stats)
{
  int i;
  static const char yuv_types[4][10] = {"YUV 4:0:0", "YUV 4:2:0", "YUV 4:2:2", "YUV 4:4:4"};
  switch (p_Inp->Verbose)
  {
  case 0:
  case 1:
  default:
    printf("------------------------------- JM %4.4s %7.7s -------------------------------\n", VERSION, EXT_VERSION);
    break;
  case 2:
    printf("--------------------------------------- JM %4.4s %7.7s ----------------------------------------\n", VERSION, EXT_VERSION);
    break;
  case 3:
    printf("------------------------------------------ JM %4.4s %7.7s ------------------------------------------\n", VERSION, EXT_VERSION);
    break;
  }

  fprintf(stdout,  " Input YUV file                    : %s \n", p_Inp->input_file1.fname);
#if (MVC_EXTENSION_ENABLE)
  if(p_Inp->num_of_views==2)
    fprintf(stdout,  " Input YUV file 2                  : %s \n", p_Inp->input_file2.fname);
#endif

  fprintf(stdout,  " Output H.264 bitstream            : %s \n", p_Inp->outfile);
  if (p_Vid->p_dec != -1)
    fprintf(stdout,  " Output YUV file                   : %s \n", p_Inp->ReconFile);
#if (MVC_EXTENSION_ENABLE)
  if(p_Inp->num_of_views==2)
  {
    if (p_Vid->p_dec2 != -1)
      fprintf(stdout,  " Output YUV file 2                 : %s \n", p_Inp->ReconFile2);
  }
#endif
  fprintf(stdout,  " YUV Format                        : %s \n", &yuv_types[p_Vid->yuv_format][0]);//p_Vid->yuv_format==YUV422?"YUV 4:2:2":(p_Vid->yuv_format==YUV444)?"YUV 4:4:4":"YUV 4:2:0");
  fprintf(stdout,  " Frames to be encoded              : %d\n", p_Inp->no_frames);
  if (p_Inp->Verbose != 0)
  {
    fprintf(stdout,  " Freq. for encoded bitstream       : %3.2f\n", p_Inp->output.frame_rate);
    fprintf(stdout,  " PicInterlace / MbInterlace        : %d/%d\n", p_Inp->PicInterlace, p_Inp->MbInterlace);
    fprintf(stdout,  " Transform8x8Mode                  : %d\n", p_Inp->Transform8x8Mode);

    for (i=0; i<3; i++)
    {
      fprintf(stdout," ME Metric for Refinement Level %1d  : %s\n", i, DistortionType[p_Inp->MEErrorMetric[i]]);
    }
    fprintf(stdout,  " Mode Decision Metric              : %s\n", DistortionType[p_Inp->ModeDecisionMetric]);

    if( p_Inp->OnTheFlyFractMCP )
    {
      fprintf(stdout," On-the-fly interpolation mode     : OTF_L%d\n", p_Inp->OnTheFlyFractMCP );
    }

    switch ( p_Inp->ChromaMEEnable )
    {
    case 1:
      fprintf(stdout," Motion Estimation for components  : YCbCr\n");
      break;
    default:
      fprintf(stdout," Motion Estimation for components  : Y\n");
      break;
    }

    fprintf(stdout,  " Image format                      : %dx%d (%dx%d)\n", p_Inp->output.width[0], p_Inp->output.height[0], p_Vid->width,p_Vid->height);

    if (p_Inp->intra_upd)
      fprintf(stdout," Error robustness                  : On\n");
    else
      fprintf(stdout," Error robustness                  : Off\n");
    fprintf(stdout,  " Search range                      : %d\n", p_Inp->search_range[0]);
#if (MVC_EXTENSION_ENABLE)
    if ( p_Inp->SepViewInterSearch )
    {
      fprintf(stdout,  " Search range (view 1)             : %d\n", p_Inp->search_range[1]);
    }
#endif

    fprintf(stdout,  " Total number of references        : %d\n", p_Inp->num_ref_frames_org);
    fprintf(stdout,  " References for P slices           : %d\n", p_Inp->P_List0_refs_org[0] ? p_Inp->P_List0_refs_org[0] : p_Inp->num_ref_frames_org);
    fprintf(stdout,  " References for B slices (L0, L1)  : %d, %d\n", 
      p_Inp->B_List0_refs_org[0] ? p_Inp->B_List0_refs_org[0] : p_Inp->num_ref_frames_org, 
      p_Inp->B_List1_refs_org[0] ? p_Inp->B_List1_refs_org[0] : p_Inp->num_ref_frames_org);

    if ( p_Vid->num_of_layers > 1 )
    {
      fprintf(stdout,  " View 1 refs for P slices          : %d\n", p_Inp->P_List0_refs_org[1] ? p_Inp->P_List0_refs_org[1] : p_Inp->num_ref_frames_org);
      fprintf(stdout,  " View 1 refs for B slices (L0, L1) : %d, %d\n", 
        p_Inp->B_List0_refs_org[1] ? p_Inp->B_List0_refs_org[1] : p_Inp->num_ref_frames_org, 
        p_Inp->B_List1_refs_org[1] ? p_Inp->B_List1_refs_org[1] : p_Inp->num_ref_frames_org);
    }

    // Sequence Type
    fprintf(stdout,  " Sequence type                     :");
    if (p_Stats->NumberBFrames > 0 && p_Inp->HierarchicalCoding)
    {
      fprintf(stdout, " Hierarchy (QP: I %d, P %d, B %d) \n",
        p_Inp->qp[I_SLICE], p_Inp->qp[P_SLICE], p_Inp->qp[B_SLICE]);
    }
    else if (p_Stats->NumberBFrames > 0)
    {
      char seqtype[80];
      int i,j;

      strcpy (seqtype,"I");

      for (j=0; j < 2; j++)
      {
        for (i=0; i < p_Stats->NumberBFrames; i++)
        {
          if (p_Inp->BRefPictures)
            strncat(seqtype,"-RB", imax(0, (int) (79 - strlen(seqtype))));
          else
            strncat(seqtype,"-B", imax(0, (int) (79 - strlen(seqtype))));
        }
        strncat(seqtype,"-P", imax(0, (int) (79 - strlen(seqtype))));
      }
      if (p_Inp->BRefPictures)
        fprintf(stdout, " %s (QP: I %d, P %d, RB %d) \n", seqtype, p_Inp->qp[I_SLICE], p_Inp->qp[P_SLICE], iClip3(0, 51, p_Inp->qp[B_SLICE] + p_Inp->qpBRSOffset));
      else
        fprintf(stdout, " %s (QP: I %d, P %d, B %d) \n", seqtype, p_Inp->qp[I_SLICE], p_Inp->qp[P_SLICE], p_Inp->qp[B_SLICE]);
    }
    else if (p_Stats->NumberBFrames == 0 && (p_Inp->intra_period == 1 || p_Inp->idr_period == 1)) 
      fprintf(stdout, " IIII (QP: I %d) \n", p_Inp->qp[I_SLICE]);
    else if (p_Stats->NumberBFrames == 0 && p_Inp->sp_periodicity == 0) 
      fprintf(stdout, " IPPP (QP: I %d, P %d) \n", p_Inp->qp[I_SLICE], p_Inp->qp[P_SLICE]);
    else 
      fprintf(stdout, " I-P-P-SP-P (QP: I %d, P %d, SP (%d, %d)) \n",  p_Inp->qp[I_SLICE], p_Inp->qp[P_SLICE], p_Inp->qp[SP_SLICE], p_Inp->qpsp);

    // report on entropy coding  method
    if (p_Inp->symbol_mode == CAVLC)
      fprintf(stdout," Entropy coding method             : CAVLC\n");
    else
      fprintf(stdout," Entropy coding method             : CABAC\n");

    fprintf(stdout,  " Profile/Level IDC                 : (%d,%d)\n", p_Inp->ProfileIDC, p_Inp->LevelIDC);

    if (p_Inp->SearchMode[0] == UM_HEX)
      fprintf(stdout,  " Motion Estimation Scheme          : HEX\n");
    else if (p_Inp->SearchMode[0] == UM_HEX_SIMPLE)
      fprintf(stdout,  " Motion Estimation Scheme          : SHEX\n");
    else if (p_Inp->SearchMode[0] == EPZS)
    {
      fprintf(stdout,  " Motion Estimation Scheme          : EPZS\n");
      EPZSOutputStats(p_Inp, stdout, 0);
    }
    else if (p_Inp->SearchMode[0] == FAST_FULL_SEARCH)
      fprintf(stdout,  " Motion Estimation Scheme          : Fast Full Search\n");
    else
      fprintf(stdout,  " Motion Estimation Scheme          : Full Search\n");

#if (MVC_EXTENSION_ENABLE)
    if ( p_Inp->SepViewInterSearch )
    {
      if (p_Inp->SearchMode[1] == UM_HEX)
        fprintf(stdout,  " Motion Estimation Scheme          : HEX\n");
      else if (p_Inp->SearchMode[1] == UM_HEX_SIMPLE)
        fprintf(stdout,  " Motion Estimation Scheme          : SHEX\n");
      else if (p_Inp->SearchMode[1] == EPZS)
      {
        fprintf(stdout,  " Motion Estimation Scheme          : EPZS\n");
        EPZSOutputStats(p_Inp, stdout, 0);
      }
      else if (p_Inp->SearchMode[1] == FAST_FULL_SEARCH)
        fprintf(stdout,  " Motion Estimation Scheme          : Fast Full Search\n");
      else
        fprintf(stdout,  " Motion Estimation Scheme          : Full Search\n");
    }
#endif

    if (p_Inp->full_search == 2)
      fprintf(stdout," Search range restrictions         : none\n");
    else if (p_Inp->full_search == 1)
      fprintf(stdout," Search range restrictions         : older reference frames\n");
    else
      fprintf(stdout," Search range restrictions         : smaller blocks and older reference frames\n");

    if (p_Inp->rdopt)
      fprintf(stdout," RD-optimized mode decision        : used\n");
    else
      fprintf(stdout," RD-optimized mode decision        : not used\n");

    switch(p_Inp->partition_mode)
    {
    case PAR_DP_1:
      fprintf(stdout," Data Partitioning Mode            : 1 partition \n");
      break;
    case PAR_DP_3:
      fprintf(stdout," Data Partitioning Mode            : 3 partitions \n");
      break;
    default:
      fprintf(stdout," Data Partitioning Mode            : not supported\n");
      break;
    }

    switch(p_Inp->of_mode)
    {
    case PAR_OF_ANNEXB:
      fprintf(stdout," Output File Format                : H.264/AVC Annex B Byte Stream Format \n");
      break;
    case PAR_OF_RTP:
      fprintf(stdout," Output File Format                : RTP Packet File Format \n");
      break;
    default:
      fprintf(stdout," Output File Format                : not supported\n");
      break;
    }
  }


  switch (p_Inp->Verbose)
  {
  case 0:
  default:
    printf("-------------------------------------------------------------------------------\n");
    printf("\nEncoding. Please Wait.\n\n");
    break;    
  case 1:
#if (MVC_EXTENSION_ENABLE)
    if (p_Inp->num_of_views == 2)
    {
      printf("------------------------------------------------------------------------------------\n");
      printf("Frame     View Bit/pic    QP   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld Ref  \n");
      printf("------------------------------------------------------------------------------------\n");
    }
    else
#endif
    {
    printf("-------------------------------------------------------------------------------\n");
    printf("Frame     Bit/pic    QP   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld Ref  \n");
    printf("-------------------------------------------------------------------------------\n");
    }
    break;
  case 2:
#if (MVC_EXTENSION_ENABLE)
    if (p_Inp->num_of_views == 2)
    {
      if (p_Inp->Distortion[SSIM] == 1)
      {
        printf("----------------------------------------------------------------------------------------------------------------------------------\n");
        printf("Frame     View Bit/pic WP QP   QL   SnrY    SnrU    SnrV   SsimY   SsimU   SsimV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
        printf("----------------------------------------------------------------------------------------------------------------------------------\n");
      }
      else
      {
        printf("---------------------------------------------------------------------------------------------------------\n");
        printf("Frame     View Bit/pic WP QP   QL   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
        printf("---------------------------------------------------------------------------------------------------------\n");
      }
    }
    else
#endif
    {
    if (p_Inp->Distortion[SSIM] == 1)
    {
      printf("---------------------------------------------------------------------------------------------------------------------------\n");
      printf("Frame     Bit/pic WP QP   QL   SnrY    SnrU    SnrV   SsimY   SsimU   SsimV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
      printf("---------------------------------------------------------------------------------------------------------------------------\n");
    }
    else
    {
      printf("---------------------------------------------------------------------------------------------------\n");
      printf("Frame     Bit/pic WP QP   QL   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
      printf("---------------------------------------------------------------------------------------------------\n");
    }
    }
    break;
  case 3:
#if (MVC_EXTENSION_ENABLE)
    if (p_Inp->num_of_views == 2)
    {
      if (p_Inp->Distortion[SSIM] == 1)
      {
        printf("---------------------------------------------------------------------------------------------------------------------------------------\n");
        printf("Frame      View Bit/pic NVB WP QP   QL   SnrY    SnrU    SnrV   SsimY   SsimU   SsimV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
        printf("---------------------------------------------------------------------------------------------------------------------------------------\n");
      }
      else
      {
        printf("---------------------------------------------------------------------------------------------------------------\n");
        printf("Frame      View Bit/pic NVB WP QP   QL   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
        printf("---------------------------------------------------------------------------------------------------------------\n");
      }
    }
    else
#endif
    {
    if (p_Inp->Distortion[SSIM] == 1)
    {
      printf("--------------------------------------------------------------------------------------------------------------------------------\n");
      printf("Frame      Bit/pic NVB WP QP   QL   SnrY    SnrU    SnrV   SsimY   SsimU   SsimV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
      printf("--------------------------------------------------------------------------------------------------------------------------------\n");
    }
    else
    {
      printf("--------------------------------------------------------------------------------------------------------\n");
      printf("Frame      Bit/pic NVB WP QP   QL   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
      printf("--------------------------------------------------------------------------------------------------------\n");
    }
    }
    break;
  case 4:
#if (MVC_EXTENSION_ENABLE)
    if (p_Inp->num_of_views == 2)
    {
      if (p_Inp->Distortion[SSIM] == 1)
      {
        printf("-------------------------------------------------------------------------------------------------------------------------------------------------\n");
        printf("Frame     View Bit/pic   Filler NVB  WP QP   QL   SnrY    SnrU    SnrV   SsimY   SsimU   SsimV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
        printf("-------------------------------------------------------------------------------------------------------------------------------------------------\n");
      }
      else
      {
        printf("-------------------------------------------------------------------------------------------------------------------------\n");
        printf("Frame     View Bit/pic   Filler NVB  WP QP   QL   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
        printf("-------------------------------------------------------------------------------------------------------------------------\n");
      }
    }
    else
#endif
    {
    if (p_Inp->Distortion[SSIM] == 1)
    {
      printf("------------------------------------------------------------------------------------------------------------------------------------------\n");
      printf("Frame      Bit/pic   Filler NVB  WP QP   QL   SnrY    SnrU    SnrV   SsimY   SsimU   SsimV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
      printf("------------------------------------------------------------------------------------------------------------------------------------------\n");
    }
    else
    {
      printf("------------------------------------------------------------------------------------------------------------------\n");
      printf("Frame      Bit/pic   Filler NVB  WP QP   QL   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld   I D L0 L1 RDP Ref\n");
      printf("------------------------------------------------------------------------------------------------------------------\n");
    }
    }
    break;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Report mode distribution of the sequence to log_mode.dat
 ************************************************************************
 */
void report_log_mode(VideoParameters *p_Vid, InputParameters *p_Inp, StatParameters *p_Stats)
{
  FILE *p_stat;                    //!< status file for the last encoding session
  int i;
  char name[40];
#ifndef WIN32
  time_t now;
  struct tm *l_time;
  char string[1000];
#else
  char timebuf[128];
#endif

  if ((p_stat = fopen("log_mode.dat", "r")) == 0)         // check if file exists
  {
    if ((p_stat = fopen("log_mode.dat", "a")) == NULL)    // append new statistic at the end
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n", "log_mode.dat");
      error(errortext, 500);
    }
    else                                            // Create header for new log file
    {
      fprintf(p_stat, " ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
      fprintf(p_stat,"|                          Encoder statistics. This file is generated during first encoding session, new sessions will be appended                                                                                                                                                                                 |\n");
      fprintf(p_stat, " ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
      fprintf(p_stat, "|     ver     | Date  | Time  |    Sequence                  | QP |  I4  |  I8  | I16  | IC0  | IC1  | IC2  | IC3  | PI4  | PI8  | PI16 |  P0  |  P1  |  P2  |  P3  | P1*4*| P1*8*| P2*4*| P2*8*| P3*4*| P3*8*|  P8  | P8:4 | P4*4*| P4*8*| P8:5 | P8:6 | P8:7 | BI4  | BI8  | BI16 |  B0  |  B1  |  B2  |  B3  | B0*4*| B0*8*| B1*4*| B1*8*| B2*4*| B2*8*| B3*4*| B3*8*|  B8  | B8:0 |B80*4*|B80*8*| B8:4 | B4*4*| B4*8*| B8:5 | B8:6 | B8:7 |\n");
      fprintf(p_stat, " ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
    }
  }
  else
  {
    fclose (p_stat);
    if ((p_stat = fopen("log_mode.dat", "a")) == NULL)         // File exists, just open for appending
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n", "log_mode.dat");
      error(errortext, 500);
    }
  }

  //report
  fprintf(p_stat, "|%4s/%s", VERSION, EXT_VERSION);

#ifdef WIN32
  _strdate( timebuf );
  fprintf(p_stat, "| %1.5s |", timebuf);

  _strtime( timebuf);
  fprintf(p_stat, " % 1.5s |", timebuf);
#else
  now = time ((time_t *) NULL); // Get the system time and put it into 'now' as 'calender time'
  time (&now);
  l_time = localtime (&now);
  strftime (string, sizeof string, "%d-%b-%Y", l_time);
  fprintf(p_stat, "| %1.5s |", string );

  strftime (string, sizeof string, "%H:%M:%S", l_time);
  fprintf(p_stat, " %1.5s |", string);
#endif

  for (i=0;i<30;i++)
    name[i]=p_Inp->input_file1.fname[i + imax(0,(int) (strlen(p_Inp->input_file1.fname)- 30))]; // write last part of path, max 30 chars

  fprintf(p_stat, "%30.30s|", name);
  fprintf(p_stat, "%3d |", p_Vid->qp);

  //report modes
  //I-Modes
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[I_SLICE][I4MB ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[I_SLICE][I8MB ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[I_SLICE][I16MB]);

  //chroma intra mode
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->intra_chroma_mode[0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->intra_chroma_mode[1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->intra_chroma_mode[2]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->intra_chroma_mode[3]);

  //P-Modes
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][I4MB ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][I8MB ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][I16MB]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][0    ]);

  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][1    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][2    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][3    ]);
  
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[P_SLICE][1][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[P_SLICE][1][1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[P_SLICE][2][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[P_SLICE][2][1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[P_SLICE][3][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[P_SLICE][3][1]);

  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][P8x8 ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][4    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[P_SLICE][4][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[P_SLICE][4][1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][5    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][6    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[P_SLICE][7    ]);

  //B-Modes
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][I4MB ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][I8MB ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][I16MB]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][0    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][1    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][2    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][3    ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][0][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][0][1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][1][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][1][1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][2][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][2][1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][3][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][3][1]);

  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][P8x8]);
  fprintf(p_stat, " %d|", (p_Stats->b8_mode_0_use [B_SLICE][0]+p_Stats->b8_mode_0_use [B_SLICE][1]));
  fprintf(p_stat, " %5d|", p_Stats->b8_mode_0_use [B_SLICE][0]);
  fprintf(p_stat, " %5d|", p_Stats->b8_mode_0_use [B_SLICE][1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][4   ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][4][0]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use_transform[B_SLICE][4][1]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][5   ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][6   ]);
  fprintf(p_stat, " %5" FORMAT_OFF_T  "|", p_Stats->mode_use[B_SLICE][7   ]);

  fprintf(p_stat, "\n");
  fclose(p_stat);
}


