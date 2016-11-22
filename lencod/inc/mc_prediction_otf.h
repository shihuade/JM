#ifndef _MC_PREDICTION_OTF_H_
#define _MC_PREDICTION_OTF_H_

#include "global.h"

void luma_prediction_otf ( Macroblock* currMB, //!< Current Macroblock
                     int   block_x,      //!< relative horizontal block coordinate of block
                     int   block_y,      //!< relative vertical   block coordinate of block
                     int   block_size_x, //!< relative horizontal block coordinate of block
                     int   block_size_y, //!< relative vertical   block coordinate of block
                     int   p_dir,        //!< prediction direction (0=list0, 1=list1, 2=bipred)
                     int   list_mode[2], //!< list prediction mode (1-7, 0=DIRECT)
                     char  *ref_idx,     //!< reference pictures
                     short bipred_me     //!< use bi prediction mv (0=no bipred, 1 = use set 1, 2 = use set 2)
                     ) ;

void luma_prediction_bi_otf ( Macroblock* currMB, //!< Current Macroblock
                       int   block_x,      //!< relative horizontal block coordinate of 4x4 block
                       int   block_y,      //!< relative vertical   block coordinate of 4x4 block
                       int   block_size_x, //!< horizontal block size
                       int   block_size_y, //!< vertical   block size
                       int   l0_mode,      //!< list0 prediction mode (1-7, 0=DIRECT if l1_mode=0)
                       int   l1_mode,      //!< list1 prediction mode (1-7, 0=DIRECT if l0_mode=0)
                       short l0_ref_idx,   //!< reference frame for list0 prediction (-1: Intra4x4 pred. with l0_mode)
                       short l1_ref_idx,   //!< reference frame for list1 prediction 
                       int   list          //!< current list for prediction.
                       );

void chroma_prediction_otf ( Macroblock* currMB, // <-- Current Macroblock
                       int   uv,            // <-- colour component
                       int   block_x,       // <-- relative horizontal block coordinate of block
                       int   block_y,       // <-- relative vertical   block coordinate of block
                       int   block_size_x,  // <-- relative horizontal block coordinate of block
                       int   block_size_y,  // <-- relative vertical   block coordinate of block                        
                       int   p_dir,         // <-- prediction direction (0=list0, 1=list1, 2=bipred)
                       int   l0_mode,       // <-- list0  prediction mode (1-7, 0=DIRECT if l1_mode=0)
                       int   l1_mode,       // <-- list1 prediction mode (1-7, 0=DIRECT if l0_mode=0)
                       short l0_ref_idx,    // <-- reference frame for list0 prediction (if (<0) -> intra prediction)
                       short l1_ref_idx,    // <-- reference frame for list1 prediction 
                       short bipred_me      // <-- use bi prediction mv (0=no bipred, 1 = use set 1, 2 = use set 2)
                       )  ;

#endif

