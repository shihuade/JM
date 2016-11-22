
/*!
 *************************************************************************************
 * \file context_ini.h
 *
 * \brief
 *    CABAC context initializations
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Detlev Marpe
 *    - Heiko Schwarz
 **************************************************************************************
 */

#ifndef _CONTEXT_INI_
#define _CONTEXT_INI_

extern void  create_context_memory       (VideoParameters *p_Vid, InputParameters *p_Inp);
extern void  free_context_memory         (VideoParameters *p_Vid);
extern void  update_field_frame_contexts (VideoParameters *p_Vid, int);
extern void  SetCtxModelNumber           (Slice *currSlice);
extern void  init_contexts               (Slice *currSlice);
extern void  store_contexts              (Slice *currSlice);

#endif

