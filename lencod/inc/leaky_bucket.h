
/*!
 ***************************************************************************
 *
 * \file leaky_bucket.h
 *
 * \brief
 *    Header for Leaky Buffer parameters
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Shankar Regunathan                   <shanre@microsoft.com>
 **************************************************************************/

#ifndef _LEAKY_BUCKET_H_
#define _LEAKY_BUCKET_H_


/* Leaky Bucket Parameter Optimization */
#ifdef _LEAKYBUCKET_
extern int get_LeakyBucketRate(InputParameters *p_Inp, unsigned long NumberLeakyBuckets, unsigned long *Rmin);
extern void PutBigDoubleWord  (unsigned long dw, FILE *fp);
extern void write_buffer      (InputParameters *p_Inp, unsigned long NumberLeakyBuckets, unsigned long Rmin[], unsigned long Bmin[], unsigned long Fmin[]);
extern void Sort              (unsigned long NumberLeakyBuckets, unsigned long *Rmin);
extern void calc_buffer       (VideoParameters *p_Vid, InputParameters *p_Inp);
#endif

#endif

