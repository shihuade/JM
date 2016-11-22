/*!
 ***************************************************************************
 * \file
 *    mode_decision.h
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Alexis Michael Tourapis         <alexismt@ieee.org>
 *
 * \date
 *    21. February 2005
 *
 * \brief
 *    Headerfile for mode decision
 **************************************************************************
 */

#ifndef _MODE_DECISION_H_
#define _MODE_DECISION_H_

//==== MODULE PARAMETERS ====
static const int  b8_mode_table[6]           = {0, 4, 5, 6, 7};                             // DO NOT CHANGE ORDER !!!
static const char mb_mode_table[10]          = {0, 1, 2, 3, P8x8, I16MB, I4MB, I8MB, IPCM, SI4MB}; // DO NOT CHANGE ORDER !!!
static const char mb_mode_table_updated[10]  = {P8x8, 3, 2, 1, 0, I16MB, I4MB, I8MB, IPCM, SI4MB}; // DO NOT CHANGE ORDER !!!
extern distblk  BIDPartitionCost                 (Macroblock *currMB, int, int, char[2], int);
extern distblk  BPredPartitionCost               (Macroblock *currMB, int, int, short, short, int, int);
extern void   StoreMV8x8                       (Slice *currSlice, int dir);
extern void   RestoreMV8x8                     (Slice *currSlice, int dir);
extern void   store_macroblock_parameters      (Macroblock *currMB, int);
extern void   StoreNewMotionVectorsBlock8x8    (Slice *currSlice, int, int, Info8x8 *);
extern void   assign_enc_picture_params        (Macroblock *currMB, int mode, Info8x8 *best, int block);
extern void   set_subblock8x8_info             (Block8x8Info*, int, int, RD_8x8DATA*);
extern void   set_block8x8_info                (Block8x8Info*, int, int, Info8x8 *best);
extern void   update_refresh_map               (Macroblock *currMB, int intra, int intra1);
extern void   get_bipred_cost                  (Macroblock *currMB, int mode, int block, int i, int j, Info8x8  *best, RD_PARAMS *enc_mb, distblk bmcost[5]);
extern short  I16Offset (int, short);
extern int    CheckReliabilityOfRef            (Macroblock *currMB, int block, int list_idx, int ref, int mode);
extern int    RDCost_for_macroblocks           (Macroblock *currMB, int, short);
extern int    mode_decision_for_I4x4_MB        (Macroblock *currMB, int, distblk*);
extern distblk  rdcost_for_8x8blocks           (Macroblock *currMB, RD_8x8DATA *dataTr, int*, int64*, int, int, short, Info8x8 *, distblk);

extern void rc_store_diff                  (int diff[16][16], imgpel **p_curImg, int cpix_x,imgpel **prediction);

extern void init_enc_mb_params             (Macroblock* currMB, RD_PARAMS *enc_mb, int intra);
extern void list_prediction_cost           (Macroblock *currMB, int list, int block, int mode, RD_PARAMS *enc_mb, distblk bmcost[5], char best_ref[2]);
extern void determine_prediction_list      (distblk [5], Info8x8 *, distblk *);
extern void compute_mode_RD_cost           (Macroblock *currMB, RD_PARAMS *enc_mb, short mode, short *inter_skip);

extern int transform_termination_control   (Macroblock* currMB, int mode);
extern int bslice_16x16_termination_control(InputParameters *p_Inp, Block8x8Info *b8x8info, int *ctr16x16, int mode, int bslice);

distblk distblkminarray ( distblk arr[], int size, int *minind );
extern int iminarray ( int arr[], int size, int *minind ); 

extern void update_lambda_costs(Macroblock *currMB, RD_PARAMS *enc_mb, int lambda_mf[3]);

extern void get_initial_mb16x16_cost(Macroblock *currMB);
extern void adjust_mb16x16_cost     (Macroblock *currMB, distblk cost);
extern void reset_adaptive_rounding(VideoParameters *p_Vid);

extern distblk distortionSSE   ( Macroblock *currMB );


#endif

