/*!
 ***************************************************************************
 * \file
 *    lambda.h
 *
 * \date
 *    13 September 2009
 *
 * \brief
 *    Headerfile for lagrangian lambda related computations
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Alexis Michael Tourapis         <alexismt@ieee.org> 

 **************************************************************************
 */

#ifndef _LAMBDA_H_
#define _LAMBDA_H_

#include "global.h"

extern void get_implicit_lambda_p_slice (Slice *currSlice);
extern void get_implicit_lambda_b_slice (Slice *currSlice);
extern void get_implicit_lambda_i_slice (Slice *currSlice);
extern void get_implicit_lambda_sp_slice(Slice *currSlice);
extern void get_explicit_lambda         (Slice *currSlice);
extern void get_fixed_lambda            (Slice *currSlice);
extern void set_rdoq_lambda             (Slice *currSlice);

#endif
