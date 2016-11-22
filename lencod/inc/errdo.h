/*!
 **************************************************************************
 *  \file errdo.h
 *  \brief  Header file for error resilient RDO (name of file should change)
 *
 *  \author 
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Peshala Pahalawatta                     <ppaha@dolby.com>
 *    - Zhifeng Chen                            <zzchen@dolby.com>
 *    - Alexis Michael Tourapis                 <atour@ieee.org>
 *
 **************************************************************************
 */

#ifndef _ERRDO_H_
#define _ERRDO_H_


typedef enum
{
  LLN = 0,
  ROPE = 1,
  RMPC = 2,
} DE_Type;

struct distortion_estimation
{
  //To support slice data partitioing, residual and motion vector should use their own buffers (used if rdopt==3)
  int **  res_con_diff_Y;                 //!< for storing the residual concealment difference for component Y in rmpc algorithm
  int *** res_con_diff_UV;                //!< for storing the residual concealment difference for component UV in rmpc algorithm 
  int **  p_res_con_diff[MAX_PLANE];      //!< pointer array for accessing res_con_error_Y / res_con_error_UV[] 
  int **  MV_con_diff_Y;                  //!< for storing the MV concealment difference for component Y in rmpc algorithm
  int *** MV_con_diff_UV;                 //!< for storing the MV concealment difference for component UV in rmpc algorithm 
  int **  p_MV_con_diff[MAX_PLANE];       //!< pointer array for accessing MV_con_diff_Y / MV_con_diff_UV[] 
  byte ** error_sign_flag_Y;              //!< to indicate the sign of transmission error for component Y in rmpc algorithm
  byte ***error_sign_flag_UV;             //!< to indicate the sign of transmission error for component UV in rmpc algorithm
  byte ** p_error_sign_flag[MAX_PLANE];   //!< pointer array for accessing error_sign_flag_Y / error_sign_flag_UV[]
  int **  transmission_dist_Y;            //!< for storing the transmission distortion for component Y in rmpc algorithm
  int *** transmission_dist_UV;           //!< for storing the transmission distortion for component UV in rmpc algorithm 
  int **  p_transmission_dist[MAX_PLANE]; //!< pointer array for accessing transmission_dist_Y / transmission_dist_UV[] 
  int **  transmission_err_Y;             //!< for storing the transmission distortion for component Y in rmpc algorithm
  int *** transmission_err_UV;            //!< for storing the transmission distortion for component UV in rmpc algorithm 
  int **  p_transmission_err[MAX_PLANE];  //!< pointer array for accessing transmission_dist_Y / transmission_dist_UV[] 

  //Multiple Decoder Buffers (used if rdopt==3)
  imgpel ***  dec_imgY;              //!< Decoded Y component in multiple hypothetical decoders
  imgpel **** dec_imgUV;             //!< Decoded U and V components in multiple hypothetical decoders
  imgpel ***  p_dec_img[MAX_PLANE];  //!< pointer array for accessing decoded pictures in hypothetical decoders
  byte   ***  mb_error_map;          //!< Map of macroblock errors in hypothetical decoders.

  //(used if rdopt==3)
  imgpel **   first_moment_Y;             //!< for storing the estimated first moment for component Y in rope algorithm
  imgpel ***  first_moment_UV;            //!< for storing the estimated first moment for component Y in rope algorithm
  imgpel **   p_first_moment[MAX_PLANE];  //!< pointer array for accessing first_moment_Y / first_moment_UV[]
  uint16 **   second_moment_Y;            //!< for storing the estimated second moment for component Y in rope algorithm
  uint16 ***  second_moment_UV;           //!< for storing the estimated second moment for component Y in rope algorithm
  uint16 **   p_second_moment[MAX_PLANE]; //!< pointer array for accessing second_moment_Y / first_moment_UV[]
};


//! Info for the "decoders-in-the-encoder" used for rdoptimization with packet losses
struct decoders
{
  int    ***res_img;                 //!< Residual values for macroblock
  int    ***res_mb_best8x8;          //!< Residual values for the best b8x8 to be used in P8x8 mode decision

  //for RMPC algorithm
  int **RCD_bestY_mb;
  int ***RCD_bestY_b8x8;
  int **MVCD_bestY_mb;
  int ***MVCD_bestY_b8x8;
  byte **flag_bestY_mb;
  byte ***flag_bestY_b8x8;
  byte **flag_wo_res;
  byte ***flag_wo_res_bestY_b8x8;
  int **trans_dist_bestY_mb;
  int ***trans_dist_bestY_b8x8;
  int **trans_dist_wo_res;
  int ***trans_dist_wo_res_bestY_b8x8;
  int **trans_err_bestY_mb;
  int ***trans_err_bestY_b8x8;
  int **trans_err_wo_res;
  int ***trans_err_wo_res_bestY_b8x8;

  //for LLN algorithm
  imgpel ***dec_mbY_best;            //!< Best reconstructed macroblock pixel values
  imgpel ****dec_mbY_best8x8;        //!< Best reconstructed 8x8 mode pixel values
  imgpel ****dec_mb_pred_best8x8;    //!< Predicted pixel values for best 8x8 modes
  imgpel ***dec_mb_pred;             //!< Predicted pixel values for macroblock
  int rec_type;

  //for ROPE algorithm
  imgpel **first_moment_bestY_mb;
  imgpel ***first_moment_bestY_b8x8;
  imgpel ***first_moment_pred_bestY_b8x8;
  imgpel **first_moment_pred;
  uint16 **second_moment_bestY_mb;
  uint16 ***second_moment_bestY_b8x8;
  uint16 ***second_moment_pred_bestY_b8x8;
  uint16 **second_moment_pred;
};

typedef struct decoders Decoders;

//============= rate-distortion opt with packet losses ===========

extern void init_error_conceal      (VideoParameters *p_Vid, int concealment_type);
extern void errdo_compute_residue   (Macroblock *currMB, imgpel **imgY, int **res_img, imgpel **mb_pred, int b8block, int block_size);
extern int  allocate_errdo_mem      (VideoParameters *p_Vid, InputParameters *p_Inp);
extern void free_errdo_mem          (VideoParameters *p_Vid);

extern void init_distortion_estimation (VideoParameters *p_Vid, int de_algorithm);
extern void errdo_store_best_b8x8(Macroblock *currMB, int transform8x8, int block);
extern void errdo_get_best_b8x8(Macroblock *currMB, int transform8x8, int block);
extern void errdo_store_best_MB(Macroblock *currMB);
extern void errdo_get_best_MB(Macroblock *currMB);
extern void errdo_get_best_P8x8(Macroblock *currMB, int transform8x8);
extern void errdo_alloc_storable_picture(StorablePicture *s, VideoParameters *p_Vid, InputParameters *p_Inp, int size_x, int size_y, int size_x_cr, int size_y_cr);
extern void errdo_free_storable_picture(StorablePicture* p);
extern StorablePicture* find_nearest_ref_picture(DecodedPictureBuffer *p_Dpb, int poc);

#endif

