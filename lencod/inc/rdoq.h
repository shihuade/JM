
/*!
 ***************************************************************************
 * \file
 *    rdoq.h
 *
 * \brief
 *    Headerfile for trellis based mode decision
 *
 * \author
 *    Limin Liu                       <lliu@dolby.com>
 *    Alexis Michael Tourapis         <alexismt@ieee.org>                
 ***************************************************************************
 */


#ifndef _RDOQ_H_
#define _RDOQ_H_

#include <math.h>


#define SIGN_BITS    1

static const int estErr4x4[6][4][4] =
{
  {
    {25600, 27040, 25600, 27040}, 
    {27040, 25600, 27040, 25600}, 
    {25600, 27040, 25600, 27040}, 
    {27040, 25600, 27040, 25600} 
  },
  {
    {30976, 31360, 30976, 31360}, 
    {31360, 32400, 31360, 32400}, 
    {30976, 31360, 30976, 31360}, 
    {31360, 32400, 31360, 32400} 
  },
  {
    {43264, 40960, 43264, 40960}, 
    {40960, 40000, 40960, 40000}, 
    {43264, 40960, 43264, 40960}, 
    {40960, 40000, 40960, 40000} 
  },
  {
    {50176, 51840, 50176, 51840}, 
    {51840, 52900, 51840, 52900}, 
    {50176, 51840, 50176, 51840}, 
    {51840, 52900, 51840, 52900} 
  },
  {
    {65536, 64000, 65536, 64000}, 
    {64000, 62500, 64000, 62500}, 
    {65536, 64000, 65536, 64000}, 
    {64000, 62500, 64000, 62500} 
  },
  {
    {82944, 84640, 82944, 84640}, 
    {84640, 84100, 84640, 84100}, 
    {82944, 84640, 82944, 84640}, 
    {84640, 84100, 84640, 84100} 
  }
};

static const int estErr8x8[6][8][8]={
  {
    {6553600, 6677056, 6400000, 6677056, 6553600, 6677056, 6400000, 6677056}, 
    {6677056, 6765201, 6658560, 6765201, 6677056, 6765201, 6658560, 6765201}, 
    {6400000, 6658560, 6553600, 6658560, 6400000, 6658560, 6553600, 6658560}, 
    {6677056, 6765201, 6658560, 6765201, 6677056, 6765201, 6658560, 6765201}, 
    {6553600, 6677056, 6400000, 6677056, 6553600, 6677056, 6400000, 6677056}, 
    {6677056, 6765201, 6658560, 6765201, 6677056, 6765201, 6658560, 6765201}, 
    {6400000, 6658560, 6553600, 6658560, 6400000, 6658560, 6553600, 6658560}, 
    {6677056, 6765201, 6658560, 6765201, 6677056, 6765201, 6658560, 6765201} 
  },
  {
    {7929856, 8156736, 8028160, 8156736, 7929856, 8156736, 8028160, 8156736}, 
    {8156736, 7537770, 7814560, 7537770, 8156736, 7537770, 7814560, 7537770}, 
    {8028160, 7814560, 7840000, 7814560, 8028160, 7814560, 7840000, 7814560}, 
    {8156736, 7537770, 7814560, 7537770, 8156736, 7537770, 7814560, 7537770}, 
    {7929856, 8156736, 8028160, 8156736, 7929856, 8156736, 8028160, 8156736}, 
    {8156736, 7537770, 7814560, 7537770, 8156736, 7537770, 7814560, 7537770}, 
    {8028160, 7814560, 7840000, 7814560, 8028160, 7814560, 7840000, 7814560}, 
    {8156736, 7537770, 7814560, 7537770, 8156736, 7537770, 7814560, 7537770} 
  },
  {
    {11075584, 10653696, 11151360, 10653696, 11075584, 10653696, 11151360, 10653696}, 
    {10653696, 11045652, 11109160, 11045652, 10653696, 11045652, 11109160, 11045652}, 
    {11151360, 11109160, 11289600, 11109160, 11151360, 11109160, 11289600, 11109160}, 
    {10653696, 11045652, 11109160, 11045652, 10653696, 11045652, 11109160, 11045652}, 
    {11075584, 10653696, 11151360, 10653696, 11075584, 10653696, 11151360, 10653696}, 
    {10653696, 11045652, 11109160, 11045652, 10653696, 11045652, 11109160, 11045652}, 
    {11151360, 11109160, 11289600, 11109160, 11151360, 11109160, 11289600, 11109160}, 
    {10653696, 11045652, 11109160, 11045652, 10653696, 11045652, 11109160, 11045652} 
  },
  {
    {12845056, 12503296, 12544000, 12503296, 12845056, 12503296, 12544000, 12503296}, 
    {12503296, 13050156, 12588840, 13050156, 12503296, 13050156, 12588840, 13050156}, 
    {12544000, 12588840, 12960000, 12588840, 12544000, 12588840, 12960000, 12588840}, 
    {12503296, 13050156, 12588840, 13050156, 12503296, 13050156, 12588840, 13050156}, 
    {12845056, 12503296, 12544000, 12503296, 12845056, 12503296, 12544000, 12503296}, 
    {12503296, 13050156, 12588840, 13050156, 12503296, 13050156, 12588840, 13050156}, 
    {12544000, 12588840, 12960000, 12588840, 12544000, 12588840, 12960000, 12588840}, 
    {12503296, 13050156, 12588840, 13050156, 12503296, 13050156, 12588840, 13050156} 
  },
  {
    {16777216, 16646400, 16384000, 16646400, 16777216, 16646400, 16384000, 16646400}, 
    {16646400, 16370116, 16692640, 16370116, 16646400, 16370116, 16692640, 16370116}, 
    {16384000, 16692640, 16646400, 16692640, 16384000, 16692640, 16646400, 16692640}, 
    {16646400, 16370116, 16692640, 16370116, 16646400, 16370116, 16692640, 16370116}, 
    {16777216, 16646400, 16384000, 16646400, 16777216, 16646400, 16384000, 16646400}, 
    {16646400, 16370116, 16692640, 16370116, 16646400, 16370116, 16692640, 16370116}, 
    {16384000, 16692640, 16646400, 16692640, 16384000, 16692640, 16646400, 16692640}, 
    {16646400, 16370116, 16692640, 16370116, 16646400, 16370116, 16692640, 16370116} 
  },
  {
    {21233664, 21381376, 21667840, 21381376, 21233664, 21381376, 21667840, 21381376}, 
    {21381376, 21381376, 21374440, 21381376, 21381376, 21381376, 21374440, 21381376}, 
    {21667840, 21374440, 21529600, 21374440, 21667840, 21374440, 21529600, 21374440}, 
    {21381376, 21381376, 21374440, 21381376, 21381376, 21381376, 21374440, 21381376}, 
    {21233664, 21381376, 21667840, 21381376, 21233664, 21381376, 21667840, 21381376}, 
    {21381376, 21381376, 21374440, 21381376, 21381376, 21381376, 21374440, 21381376}, 
    {21667840, 21374440, 21529600, 21374440, 21667840, 21374440, 21529600, 21374440}, 
    {21381376, 21381376, 21374440, 21381376, 21381376, 21381376, 21374440, 21381376} 
  }
};


struct est_bits_cabac 
{
  int  significantBits[16][2];
  int  lastBits[16][2];
  int  greaterOneBits[2][5][2]; // c1 and c2
  int  greaterOneState[5];
  int  blockCbpBits[4][2]; // c1 and c2
};
typedef struct est_bits_cabac estBitsCabacStruct;

typedef struct level_data_struct
{
  int level[3];
  int  levelDouble;
  double  errLevel[3];
  int noLevels;
  int coeff_ctr;
  int pre_level;
  int sign;
} levelDataStruct;


extern void init_rdoq_slice(Slice *currSlice);

/*----------CAVLC related functions----------*/
extern void est_RunLevel_CAVLC(Macroblock *currMB, levelDataStruct *levelData, int *levelTrellis, int block_type, 
                        int b8, int b4, int coeff_num, double lambda);
extern int est_CAVLC_bits     (VideoParameters *p_Vid, int level_to_enc[16], int sign_to_enc[16], int nnz, int block_type);

/*----------CABAC related functions----------*/
extern void precalculate_unary_exp_golomb_level(VideoParameters *p_Vid);
extern int est_unary_exp_golomb_level_bits(unsigned int symbol, int bits0, int bits1);
extern int est_exp_golomb_encode_eq_prob  (unsigned int symbol);

extern void estRunLevel_CABAC               (Macroblock* currMB, int context);



extern int est_write_and_store_CBP_block_bit(Macroblock* currMB, int type);
extern void est_writeRunLevel_CABAC(Macroblock *currMB, levelDataStruct levelData[], int levelTabMin[], int type, double lambda, int kStart, 
                             int kStop, int noCoeff, int estCBP);
extern void init_trellis_data_4x4_CAVLC(Macroblock *currMB, int **tblock, int block_x, int qp_per, int qp_rem, 
                         LevelQuantParams **q_params_4x4, const byte *p_scan, 
                         levelDataStruct *dataLevel, int type);
extern int init_trellis_data_4x4_CABAC(Macroblock *currMB, int **tblock, 
                                       struct quant_methods *q_method, const byte *p_scan, 
                                       levelDataStruct *dataLevel, int* kStart, int* kStop, int type);
extern void init_trellis_data_8x8_CAVLC(Macroblock *currMB, int **tblock, int block_x, int qp_per, int qp_rem, 
                         LevelQuantParams **q_params_8x8, const byte *p_scan, 
                         levelDataStruct levelData[4][16]);
extern int init_trellis_data_8x8_CABAC(Macroblock *currMB, int **tblock, int block_x, int qp_per, int qp_rem, 
                         LevelQuantParams **q_params_8x8, const byte *p_scan, 
                         levelDataStruct *dataLevel, int* kStart, int* kStop);
extern void init_trellis_data_DC_CAVLC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, 
                         LevelQuantParams *q_params_4x4, const byte *p_scan, 
                         levelDataStruct *dataLevel);
extern int init_trellis_data_DC_CABAC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, 
                         LevelQuantParams *q_params_4x4, const byte *p_scan, 
                         levelDataStruct *dataLevel, int* kStart, int* kStop);
extern int init_trellis_data_DC_cr_CAVLC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, 
                         LevelQuantParams *q_params_4x4, const byte *p_scan, 
                         levelDataStruct *dataLevel);
extern int init_trellis_data_DC_cr_CABAC(Macroblock *currMB, int **tblock, int qp_per, int qp_rem, 
                         LevelQuantParams *q_params_4x4, const byte *p_scan, 
                         levelDataStruct *dataLevel, int* kStart, int* kStop);

extern void RDOQ_update_mode    (Slice *currSlice, RD_PARAMS *enc_mb);
extern void copy_rddata_trellis (Macroblock *currMB, RD_DATA *dest, RD_DATA *src);
extern void updateMV_mp         (Macroblock *currMB, distblk *m_cost, short ref, int list, int h, int v, int blocktype, int block8x8);
extern void trellis_coding      (Macroblock *currMB);
extern void get_dQP_table       (Slice *currSlice);

#endif  // _RDOQ_H_

