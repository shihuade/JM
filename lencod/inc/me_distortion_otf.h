/*!
 ******************************************************************************************
 * \file
 *    me_distortion_otf.h
 *
 * \brief
 *    Headerfile for motion estimation distortion with subpel interpolation on-the-fly
 ******************************************************************************************
 */

#ifndef _ME_DISTORTION_OTF_H_
#define _ME_DISTORTION_OTF_H_

// SAD functions
extern distblk computeSAD_otf         (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);

// Weighted Prediction SAD functions
extern distblk computeSADWP_otf     (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);

// SATD
extern distblk computeSATD_otf       (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
extern distblk computeSATDWP_otf     (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);

// SSE
extern distblk computeSSE_otf        (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);
extern distblk computeSSEWP_otf      (StorablePicture *ref1, MEBlock*, distblk, MotionVector *);

// Bipred SAD
extern distblk computeBiPredSAD1_otf     (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
extern distblk computeBiPredSAD2_otf     (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);

// Bipred SATD
extern distblk computeBiPredSATD1_otf    (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
extern distblk computeBiPredSATD2_otf    (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);

// Bipred SSE
extern distblk computeBiPredSSE1_otf     (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);
extern distblk computeBiPredSSE2_otf     (StorablePicture *ref1, StorablePicture *ref2, MEBlock*, distblk, MotionVector *, MotionVector *);

#endif
