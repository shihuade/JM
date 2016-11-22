/*!
 **************************************************************************
 * \file defines.h
 *
 * \brief
 *    Header file containing some useful global definitions
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Detlev Marpe
 *     - Karsten Suehring
 *     - Alexis Michael Tourapis         <alexismt@ieee.org> 
 *   
 *
 * \date
 *    21. March 2001
 **************************************************************************
 */


#ifndef _DEFINES_H_
#define _DEFINES_H_

#if defined _DEBUG
# define TRACE           0      //!< 0:Trace off 1:Trace on 2:detailed CABAC context information
#else
# define TRACE           0      //!< 0:Trace off 1:Trace on 2:detailed CABAC context information
#endif

#define JM                  "19 (FRExt)"
#define VERSION             "19.0"
#define EXT_VERSION         "(FRExt)"

#define GET_METIME                1    //!< Enables or disables ME computation time
#define DUMP_DPB                  0    //!< Dump DPB info for debug purposes
#define PRINTREFLIST              0    //!< Print ref list info for debug purposes
#define IMGTYPE                   1    //!< Define imgpel size type. 0 implies byte (cannot handle >8 bit depths) and 1 implies unsigned short
#define ENABLE_FIELD_CTX          1    //!< Enables field context types for CABAC. If disabled, results in speedup for progressive content.
#define ENABLE_HIGH444_CTX        1    //!< Enables High 444 context types for CABAC. If disabled, results in speedup of non High444 profile encodings.
#define DEBUG_BITDEPTH            0    //!< Ensures that > 8 bit content have no values that would result in out of range results
#define ALLOW_GRAYSCALE           1    //!< Allow encoding in grayscale
#define ZEROSNR                   1    //!< PSNR computation method
#define USE_RND_COST              0    //!< Perform ME RD decision using a rounding estimate of the motion cost
#define JM_INT_DIVIDE             1
#define JM_MEM_DISTORTION         0
#define JCOST_CALC_SCALEUP        1    //!< 1: J = (D<<LAMBDA_ACCURACY_BITS)+Lambda*R; 0: J = D + ((Lambda*R+Rounding)>>LAMBDA_ACCURACY_BITS)
#define INTRA_RDCOSTCALC_ET       1    //!< Early termination 
#define INTRA_RDCOSTCALC_NNZ      1    //1: to recover block's nzn after rdcost calculation;
#define JCOST_OVERFLOWCHECK       0    //!<1: to check the J cost if it is overflow>
#define JM_PARALLEL_DEBLOCK       0    //!< Enables Parallel Deblocking
#define SIMULCAST_ENABLE          0

#define MVC_EXTENSION_ENABLE      1    //!< enable support for the Multiview High Profile
#define EOS_OUTPUT                0 

#define EPZSREF                   1

#define MAX_RC_MODE               3
#define RC_MAX_TEMPORAL_LEVELS    5

#define SSE_MEMORY_ALIGNMENT      16
#define MAX_NUM_DPB_LAYERS        2
//#define BEST_NZ_COEFF 1   // yuwen 2005.11.03 => for high complexity mode decision (CAVLC, #TotalCoeff)

// defines for creating similar coding structures like HM
#define OUTPUT_REF_LIST         0     //!< for debug purpose, output the reference picture list. do not support field

#define B0_MORE_REF             1     ///!< used for random access setting, treat POC(HM like poc, =JM poc/2)%(NumberOfBFrames+1)==0 as B0, i.e. use more references at temporal level 0
#define KEEP_B_SAME_LIST        1     ///!< keep B pictures using two identical reference picture lists
#define CRA                     1     ///!< used for random access setting, HM CRA like random access point
                                      ///!< using open GOP is not enough, for example, 32 is an Intra picture, when encoding 40, everything is OK.
                                      ///!< but when encoding 36, there is one reference picture (32) in list0; actually, there should be two (32, 40) as HM.
                                      ///!< setting number of reference frame to 1 at picture 40, using RPLR to let list1 use 32 as reference,
                                      ///!< and marking all pictures unused for reference except 32 to solve this problem
#define HM50_LIKE_MMCO          1     ///!< use the HM-5.0 like MMCO, keep the identical referencing structure as HM-5.0
#define LD_REF_SETTING          1     ///!< used for low delay setting, 1+X referencing structure, as proposed in JCTVC-F701


//AVC Profile IDC definitions
enum {
  NO_PROFILE     =  0,       //!< disable profile checking for experimental coding (enables FRExt, but disables MV)
  FREXT_CAVLC444 = 44,       //!< YUV 4:4:4/14 "CAVLC 4:4:4"
  BASELINE       = 66,       //!< YUV 4:2:0/8  "Baseline"
  MAIN           = 77,       //!< YUV 4:2:0/8  "Main"
  EXTENDED       = 88,       //!< YUV 4:2:0/8  "Extended"
  FREXT_HP       = 100,      //!< YUV 4:2:0/8  "High"
  FREXT_Hi10P    = 110,      //!< YUV 4:2:0/10 "High 10"
  FREXT_Hi422    = 122,      //!< YUV 4:2:2/10 "High 4:2:2"
  FREXT_Hi444    = 244,      //!< YUV 4:4:4/14 "High 4:4:4"
  MULTIVIEW_HIGH = 118,      //!< YUV 4:2:0/8  "Multiview High"
  STEREO_HIGH    = 128       //!< YUV 4:2:0/8  "Stereo High"
} ProfileIDC;

// Some typedefs used in the software
#include "types.h"

#define FILE_NAME_SIZE  255
#define INPUT_TEXT_SIZE 1024

#define CAVLC_LEVEL_LIMIT 2063

#if (ENABLE_HIGH444_CTX == 1)
# define NUM_BLOCK_TYPES 22  
#else
# define NUM_BLOCK_TYPES 10
#endif

#define _LEAKYBUCKET_

// ---------------------------------------------------------------------------------
// FLAGS and DEFINES for new chroma intra prediction, Dzung Hoang
// Threshold values to zero out quantized transform coefficients.
// Recommend that _CHROMA_COEFF_COST_ be low to improve chroma quality
#define _LUMA_COEFF_COST_       4 //!< threshold for luma coeffs
#define _CHROMA_COEFF_COST_     4 //!< threshold for chroma coeffs, used to be 7
#define _LUMA_MB_COEFF_COST_    5 //!< threshold for luma coeffs of inter Macroblocks
#define _LUMA_8x8_COEFF_COST_   5 //!< threshold for luma coeffs of 8x8 Inter Partition

//#define IMG_PAD_SIZE           20 //!< Number of pixels padded around the reference frame (>=4)
//#define IMG_PAD_SIZE_TIMES4    80 //!< Number of pixels padded around the reference frame in subpel units(>=16)
#define IMG_PAD_SIZE_X         32 //!< Number of pixels padded around the reference frame (>=4)
#define IMG_PAD_SIZE_Y         20 //!< Number of pixels padded around the reference frame (>=4)

#define MAX_VALUE       999999   //!< used for start value for some variables
#define INVALIDINDEX  (-135792468)

#define DUMMY   14
#define ET_SIZE 300      //!< size of error text buffer

#define  LAMBDA_ACCURACY_BITS         5
#define  LAMBDA_FACTOR(lambda)        ((int)((double)(1 << LAMBDA_ACCURACY_BITS) * lambda + 0.5))

#if (IMGTYPE == 0)
#define DISTBLK_MAX  INT_MAX
#else
#define DISTBLK_MAX  ((distblk) INT_MAX << LAMBDA_ACCURACY_BITS)
#endif

#define  MAXSLICEPERPICTURE           100
#define  MAX_REFERENCE_PICTURES        32

#define BLOCK_SHIFT            2
#define BLOCK_SIZE             4
#define BLOCK_SIZE_8x8         8
#define SMB_BLOCK_SIZE         8
#define BLOCK_PIXELS          16
#define MB_BLOCK_SIZE         16
#define MB_PIXELS            256 // MB_BLOCK_SIZE * MB_BLOCK_SIZE
#define MB_PIXELS_SHIFT        8 // log2(MB_BLOCK_SIZE * MB_BLOCK_SIZE)
#define MB_BLOCK_SHIFT         4
#define BLOCK_MULTIPLE         4 // (MB_BLOCK_SIZE/BLOCK_SIZE)
#define MB_BLOCK_PARTITIONS   16 // (BLOCK_MULTIPLE * BLOCK_MULTIPLE)
#define BLOCK_CONTEXT         64 // (4 * MB_BLOCK_PARTITIONS)
// These variables relate to the subpel accuracy supported by the software (1/4)
#define BLOCK_SIZE_SP      16  // BLOCK_SIZE << 2
#define BLOCK_SIZE_8x8_SP  32  // BLOCK_SIZE8x8 << 2

// wavelet based weighted PSNR wavelet levels
#define NUM_WAVELET_LEVEL 4

// RDOQ
#define MAX_PREC_COEFF    25



//  Available MB modes
enum {
  PSKIP        =  0,
  BSKIP_DIRECT =  0,
  P16x16       =  1,
  P16x8        =  2,
  P8x16        =  3,
  SMB8x8       =  4,
  SMB8x4       =  5,
  SMB4x8       =  6,
  SMB4x4       =  7,
  P8x8         =  8,
  I4MB         =  9,
  I16MB        = 10,
  IBLOCK       = 11,
  SI4MB        = 12,
  I8MB         = 13,
  IPCM         = 14,
  MAXMODE      = 15
} MBModeTypes;

// number of intra prediction modes
#define NO_INTRA_PMODE  9

// Direct Mode types
enum {
  DIR_TEMPORAL = 0, //!< Temporal Direct Mode
  DIR_SPATIAL  = 1 //!< Spatial Direct Mode
} DirectModes;

// CAVLC block types
enum {
  LUMA              =  0,
  LUMA_INTRA16x16DC =  1,
  LUMA_INTRA16x16AC =  2,
  CB                =  3,
  CB_INTRA16x16DC   =  4,
  CB_INTRA16x16AC   =  5,
  CR                =  8,
  CR_INTRA16x16DC   =  9,
  CR_INTRA16x16AC   = 10
} CAVLCBlockTypes;

// CABAC block types
enum {
  LUMA_16DC     =   0,
  LUMA_16AC     =   1,
  LUMA_8x8      =   2,
  LUMA_8x4      =   3,
  LUMA_4x8      =   4,
  LUMA_4x4      =   5,
  CHROMA_DC     =   6,
  CHROMA_AC     =   7,
  CHROMA_DC_2x4 =   8,
  CHROMA_DC_4x4 =   9,
  CB_16DC       =  10,
  CB_16AC       =  11,
  CB_8x8        =  12,
  CB_8x4        =  13,
  CB_4x8        =  14,
  CB_4x4        =  15,
  CR_16DC       =  16,
  CR_16AC       =  17,
  CR_8x8        =  18,
  CR_8x4        =  19,
  CR_4x8        =  20,
  CR_4x4        =  21
} CABACBlockTypes;

// Color components
enum {
  Y_COMP = 0,    // Y Component
  U_COMP = 1,    // U Component
  V_COMP = 2,    // V Component
  R_COMP = 3,    // R Component
  G_COMP = 4,    // G Component
  B_COMP = 5,    // B Component
  T_COMP = 6
} ColorComponent;


#define LEVEL_NUM         6
#define TOTRUN_NUM       15
#define RUNBEFORE_NUM     7
#define RUNBEFORE_NUM_M1  6

// Quantization parameter range
#define MIN_QP          0
#define MAX_QP          51
#define SHIFT_QP        12

// 4x4 intra prediction modes 
enum {
  VERT_PRED            = 0,
  HOR_PRED             = 1,
  DC_PRED              = 2,
  DIAG_DOWN_LEFT_PRED  = 3,
  DIAG_DOWN_RIGHT_PRED = 4,
  VERT_RIGHT_PRED      = 5,
  HOR_DOWN_PRED        = 6,
  VERT_LEFT_PRED       = 7,
  HOR_UP_PRED          = 8
} I4x4PredModes;

// 16x16 intra prediction modes
enum {
  VERT_PRED_16   = 0,
  HOR_PRED_16    = 1,
  DC_PRED_16     = 2,
  PLANE_16       = 3
} I16x16PredModes;

// 8x8 chroma intra prediction modes
enum {
  DC_PRED_8     =  0,
  HOR_PRED_8    =  1,
  VERT_PRED_8   =  2,
  PLANE_8       =  3
} I8x8PredModes;

#define INIT_FRAME_RATE 30
enum {
  EOS = 1,    //!< End Of Sequence
  SOP = 2,    //!< Start Of Picture
  SOS = 3     //!< Start Of Slice
};

// MV Prediction types
enum {
  MVPRED_MEDIAN   = 0,
  MVPRED_L        = 1,
  MVPRED_U        = 2,
  MVPRED_UR       = 3
} MVPredTypes;

#define MAX_SYMBOLS_PER_MB  1200  //!< Maximum number of different syntax elements for one MB
                                  // CAVLC needs more symbols per MB


#define MAX_PART_NR     3 /*!< Maximum number of different data partitions.
                               Some reasonable number which should reflect
                               what is currently defined in the SE2Partition map (elements.h) */

//Start code and Emulation Prevention need this to be defined in identical manner at encoder and decoder
#define ZEROBYTES_SHORTSTARTCODE 2 //indicates the number of zero bytes in the short start-code prefix

#define Q_BITS          15
#define DQ_BITS         6

#define Q_BITS_8        16
#define DQ_BITS_8       6

// Context Adaptive Lagrange Multiplier (CALM)
#define CALM_MF_FACTOR_THRESHOLD 512.0

#define MAX_PLANE       3

#define MAXSLICEGROUPIDS 8


#define NUM_MB_TYPE_CTX  11
#define NUM_B8_TYPE_CTX  9
#define NUM_MV_RES_CTX   10
#define NUM_REF_NO_CTX   6
#define NUM_DELTA_QP_CTX 4
#define NUM_MB_AFF_CTX   4

#define NUM_TRANSFORM_SIZE_CTX 3

#define NUM_IPR_CTX    2
#define NUM_CIPR_CTX   4
#define NUM_CBP_CTX    4
#define NUM_BCBP_CTX   4
#define NUM_MAP_CTX   15
#define NUM_LAST_CTX  15
#define NUM_ONE_CTX    5
#define NUM_ABS_CTX    5

enum // JLT : on-the-fly levels/modes
{
  OTF_L0 = 0, // Disable, interpolate & store all positions
  OTF_L1 = 1, // Store full pel & interpolated 1/2 pel positions; 1/4 pel positions interpolate on-the-fly
  OTF_L2 = 2  // Store only full pell positions; 1/2 & 1/4 pel positions interpolate on-the-fly  
} OTFMode;

enum
{
  OTF_ME = 0,
  OTF_MC = 1
} OTFFunction;

#if IMGTYPE
#if JM_MEM_DISTORTION
#pragma message("JM_MEM_DISTORTION must be zero!")
#endif
#endif


#endif

