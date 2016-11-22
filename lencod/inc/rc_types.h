
/*!
 ***************************************************************************
 * \file
 *    rc_types.h
 *
 * \author
 *    Zhengguo LI
 *    Athanasios Leontaris
 *
 * \date
 *    14 Jan 2003
 *    27 Apr 2009
 *
 * \brief
 *    Headerfile for basic rate control structures
 **************************************************************************
 */

#ifndef _RC_TYPES_H_
#define _RC_TYPES_H_

#define RC_MODEL_HISTORY 21

/* generic rate control variables */
typedef struct rc_generic
{
  // RC flags
  int   TopFieldFlag;
  int   FieldControl;
  int   FieldFrame;
  int   NoGranularFieldRC;
  // bits stats
  int   NumberofHeaderBits;
  int   NumberofTextureBits;
  int   NumberofBasicUnitHeaderBits;
  int   NumberofBasicUnitTextureBits;
  // frame stats
  int   NumberofGOP;
  int   NumberofCodedBFrame;  
  // MAD stats
  int64 TotalMADBasicUnit;
  int   *MADofMB;
  // buffer and budget
  int64 CurrentBufferFullness; //LIZG 25/10/2002
  int64 RemainingBits;
  // bit allocations for RC_MODE_3
  int   RCPSliceBits;
  int   RCISliceBits;
  int   RCBSliceBits[RC_MAX_TEMPORAL_LEVELS];
  int   temporal_levels;
  int   hierNb[RC_MAX_TEMPORAL_LEVELS];
  int   NPSlice;
  int   NISlice;
} RCGeneric;

typedef struct rc_quadratic
{
  float  bit_rate;
  float  frame_rate;
  float  PrevBitRate;           //LIZG  25/10/2002
  double GAMMAP;                //LIZG, JVT019r1
  double BETAP;                 //LIZG, JVT019r1
  double GOPTargetBufferLevel;
  double TargetBufferLevel;     //LIZG 25/10/2002
  double AveWp;
  double AveWb;
  int    MyInitialQp;
  int    PAverageQp;
  double PreviousPictureMAD;
  double MADPictureC1;
  double MADPictureC2;
  double PMADPictureC1;
  double PMADPictureC2;
  double PPictureMAD [RC_MODEL_HISTORY];
  double PictureMAD  [RC_MODEL_HISTORY];
  double ReferenceMAD[RC_MODEL_HISTORY];
  double m_rgQp      [RC_MODEL_HISTORY];
  double m_rgRp      [RC_MODEL_HISTORY];
  double Pm_rgQp     [RC_MODEL_HISTORY];
  double Pm_rgRp     [RC_MODEL_HISTORY];

  double m_X1;
  double m_X2;
  double Pm_X1;
  double Pm_X2;
  int    Pm_Qp;
  int    Pm_Hp;

  int    MADm_windowSize;
  int    m_windowSize;
  int    m_Qc;

  int    PPreHeader;
  int    PrevLastQP; // QP of the second-to-last coded frame in the primary layer
  int    CurrLastQP; // QP of the last coded frame in the primary layer
  int    NumberofBFrames;
  // basic unit layer rate control
  int    TotalFrameQP;
  int    NumberofBasicUnit;
  int    PAveHeaderBits1;
  int    PAveHeaderBits2;
  int    PAveHeaderBits3;
  int    PAveFrameQP;
  int    TotalNumberofBasicUnit;
  int    CodedBasicUnit;

  int    NumberofCodedPFrame;  
  int    TotalQpforPPicture;
  int    NumberofPPicture;

  double CurrentFrameMAD;
  double CurrentBUMAD;
  double TotalBUMAD;
  double PreviousFrameMAD;
  double PreviousWholeFrameMAD;

  int    DDquant;
  unsigned int    MBPerRow;
  int    QPLastPFrame;
  int    QPLastGOP;

  // adaptive field/frame coding
  int    FieldQPBuffer;
  int    FrameQPBuffer;
  int    FrameAveHeaderBits;
  int    FieldAveHeaderBits;
  double *BUPFMAD;
  double *BUCFMAD;
  double *FCBUCFMAD;
  double *FCBUPFMAD;

  Boolean GOPOverdue;
  int64   Pprev_bits;

  // rate control variables
  int    Xp, Xb;
  int    Target;
  int    TargetField;
  int    Np, Nb, bits_topfield;
  // HRD consideration
  int    UpperBound1, UpperBound2, LowerBound;
  double Wp, Wb; // complexity weights
  double DeltaP;
  int    TotalPFrame;
  int    PMaxQpChange;

  int    bitdepth_qp_scale; // support negative QPs (bitdepth > 8-bits per component)

  // simple encoder buffer simulation
  int    enc_buf_curr;

} RCQuadratic;

#endif

