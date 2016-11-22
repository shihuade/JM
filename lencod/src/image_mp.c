
/*!
 *************************************************************************************
 * \file image.c
 *
 * \brief
 *    multiple-pass encoding of one frame
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Yan Ye                          <yye@dolby.com>
 *************************************************************************************
 */
#include "contributors.h"

#include "global.h"
#include "image.h"
#include "rc_quadratic.h"
#include "wp.h"
#include "pred_struct.h"
#include "slice.h"

#define DBG_IMAGE_MP  0

typedef enum
{
  REGULAR = 0,
  EXP_WP  = 1,
  IMP_WP  = 2,    // only applicable to B
  FRAME_TYPE = 2, // only applicable to P 
  FRAME_QP = 3,
  ALT_DIRECT = 4,
  ALT_ENTROPY = 5,
  DB_OFF = 6,
} FrameCodingMethod;

void frame_picture_mp_i_slice(VideoParameters *p_Vid, InputParameters *p_Inp);
void frame_picture_mp_p_slice(VideoParameters *p_Vid, InputParameters *p_Inp);
void frame_picture_mp_b_slice(VideoParameters *p_Vid, InputParameters *p_Inp);

void store_coding_info(VideoParameters *p_Vid, CodingInfo *coding_info)
{
  coding_info->type           = p_Vid->type;
  coding_info->intras         = p_Vid->intras;
  coding_info->sumFrameQP     = p_Vid->SumFrameQP;
  coding_info->num_ref_idx_l0 = p_Vid->num_ref_idx_l0_active; 
  coding_info->num_ref_idx_l1 = p_Vid->num_ref_idx_l1_active;
  coding_info->active_pps     = p_Vid->active_pps;
}

void store_coding_and_rc_info(VideoParameters *p_Vid, CodingInfo *coding_info)
{
  InputParameters *p_Inp = p_Vid->p_Inp;

  store_coding_info( p_Vid, coding_info );

  if ( p_Inp->RCEnable )
  {
    rc_save_state( p_Vid, p_Inp );
  }
}

void restore_coding_info(VideoParameters *p_Vid, CodingInfo *coding_info)
{
  p_Vid->type                  = coding_info->type;
  p_Vid->intras                = coding_info->intras;
  p_Vid->SumFrameQP            = coding_info->sumFrameQP;
  p_Vid->num_ref_idx_l0_active = coding_info->num_ref_idx_l0; 
  p_Vid->num_ref_idx_l1_active = coding_info->num_ref_idx_l1; 
  p_Vid->active_pps            = coding_info->active_pps;
}

void swap_frame_buffer(VideoParameters *p_Vid, int a, int b)
{
  StorablePicture *s; 
  Picture *p;

  p = p_Vid->frame_pic[a];
  p_Vid->frame_pic[a] = p_Vid->frame_pic[b];
  p_Vid->frame_pic[b] = p;

  s = p_Vid->enc_frame_picture[a];
  p_Vid->enc_frame_picture[a] = p_Vid->enc_frame_picture[b];
  p_Vid->enc_frame_picture[b] = s;
}


void frame_picture_mp_exit(VideoParameters *p_Vid, CodingInfo *coding_info)
{
  InputParameters *p_Inp = p_Vid->p_Inp;

  p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
  p_Vid->enc_picture=p_Vid->enc_frame_picture[0];
  p_Vid->p_frame_pic = p_Vid->frame_pic[0];
  restore_coding_info(p_Vid, coding_info); 

  if ( p_Inp->RCEnable )
  {
    rc_restore_state(p_Vid, p_Inp);
  }
}


void frame_picture_mp_p_slice(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int   rd_pass = 0;
  int   rd_qp = p_Vid->p_curr_frm_struct->qp;
  float rateRatio = 1.0F;
  int   wp_pass=0;
  int   frame_type_pass = 0;
  CodingInfo coding_info;
  FrameCodingMethod best_method = REGULAR; 
  int frame_type = P_SLICE; 
  int apply_wp = 0;
  int selection;

  frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
  store_coding_and_rc_info(p_Vid, &coding_info);

  if(p_Inp->WPIterMC)
    p_Vid->frameOffsetAvail = 1; 

#if (DBG_IMAGE_MP)
    printf("rd_pass = %d: %d (%.0f, %.0f, %.0f)\n", rd_pass, 
      p_Vid->frame_pic[0]->bits_per_picture, 
      p_Vid->frame_pic[0]->distortion.value[0], p_Vid->frame_pic[0]->distortion.value[1], p_Vid->frame_pic[0]->distortion.value[2]);
#endif

  rd_pass++;
  if(rd_pass >= p_Inp->RDPictureMaxPassPSlice)
  {
    frame_picture_mp_exit(p_Vid, &coding_info);
    return;
  }

  // for P_Slice, consider WP  
  wp_pass = 0;
  if (p_Inp->GenerateMultiplePPS)
  {
    Slice *dummy_slice = NULL;

    InitWP(p_Vid, p_Inp, 0);
    if ( p_Inp->WPMCPrecision )
      p_Vid->pWPX->curr_wp_rd_pass = p_Vid->pWPX->wp_rd_passes + 1;
    init_slice_lite(p_Vid, &dummy_slice, 0);

    if (p_Vid->TestWPPSlice(dummy_slice, 0) == 1)
    {
      // regular WP pass
      p_Vid->active_pps = p_Vid->PicParSet[1];
      if ( p_Inp->WPMCPrecision )
        p_Vid->pWPX->curr_wp_rd_pass->algorithm = WP_REGULAR;
      wp_pass = 1;
    }
    else if ( p_Inp->WPMCPrecision )
    {
      // WPMC pass
      p_Vid->active_pps = p_Vid->PicParSet[1];
      wp_pass = 1;
    }  

    // The way it is, the code would only reach here if prior conditional using 
    // generatemultiplepps is satisfied
    if(wp_pass)
    {
      p_Vid->write_macroblock = FALSE;
      p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
      frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
      selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
      printf("rd_pass = %d, selection = %d\n", rd_pass, selection);
#endif
#if (DBG_IMAGE_MP)
      printf("rd_pass = %d: %d (%.0f, %.0f, %.0f)\n", rd_pass, 
        p_Vid->frame_pic[rd_pass]->bits_per_picture, 
        p_Vid->frame_pic[rd_pass]->distortion.value[0], p_Vid->frame_pic[rd_pass]->distortion.value[1], p_Vid->frame_pic[rd_pass]->distortion.value[2]);
#endif

      if (selection)
      {
        swap_frame_buffer(p_Vid, 0, rd_pass); 
        store_coding_and_rc_info(p_Vid, &coding_info);
        best_method = EXP_WP;
        apply_wp = 1;
      }

      if(p_Inp->WPMethod == 0 || p_Inp->WPMCPrecision) 
      {
        wp_pass = 0;
        if ( p_Inp->WPMCPrecision )
          p_Vid->pWPX->curr_wp_rd_pass = p_Vid->pWPX->wp_rd_passes + 2;
        if (p_Inp->WPMethod == 0 && p_Vid->TestWPPSlice(dummy_slice, 1) == 1)
        {
          // regular WP pass
          p_Vid->active_pps = p_Vid->PicParSet[1];
          if ( p_Inp->WPMCPrecision )
            p_Vid->pWPX->curr_wp_rd_pass->algorithm = WP_REGULAR;
          wp_pass = 1;
        }
        else if ( p_Inp->WPMCPrecision )
        {
          // WPMC pass
          p_Vid->active_pps = p_Vid->PicParSet[1];
          wp_pass = 1;
        }

        if(wp_pass)
        {
          p_Vid->write_macroblock = FALSE;
          p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
          free_slice_list(p_Vid->frame_pic[rd_pass]);
          free_storable_picture(p_Vid, p_Vid->enc_frame_picture[rd_pass]);
          frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
          selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
          printf("rd_pass = %d, selection = %d\n", rd_pass, selection);
#endif
#if (DBG_IMAGE_MP)
          printf("rd_pass = %d: %d (%.0f, %.0f, %.0f)\n", rd_pass, 
            p_Vid->frame_pic[rd_pass]->bits_per_picture, 
            p_Vid->frame_pic[rd_pass]->distortion.value[0], p_Vid->frame_pic[rd_pass]->distortion.value[1], p_Vid->frame_pic[rd_pass]->distortion.value[2]);
#endif

          if (selection)
          {
            swap_frame_buffer(p_Vid, 0, rd_pass); 
            store_coding_and_rc_info(p_Vid, &coding_info);
            best_method = EXP_WP;
            apply_wp = 1;
          }
        }
      }

      rd_pass++;
      //free_slice(dummy_slice);

      if(rd_pass >= p_Inp->RDPictureMaxPassPSlice)
      {
        frame_picture_mp_exit(p_Vid, &coding_info);
        free_slice(dummy_slice);
        return;
      }
    }
    free_slice(dummy_slice);
  }


  // code as I? or maybe as B?
  frame_type_pass = 0;
  if(p_Inp->RDPSliceITest && (coding_info.intras * 100/p_Vid->FrameSizeInMbs) >= 75)
  {
    frame_type = I_SLICE; 
    set_slice_type(p_Vid, p_Inp, I_SLICE);
    populate_frame_slice_type( p_Inp, p_Vid->p_curr_frm_struct, I_SLICE, p_Vid->p_pred->max_num_slices );
    p_Vid->active_pps = p_Vid->PicParSet[0];
    frame_type_pass = 1;
  }
  else if (p_Inp->RDPSliceBTest && p_Vid->active_sps->profile_idc != BASELINE)
  // later need to add some automatic criterion to see if this (coding P as B) may be beneficial 
  {
    frame_type = B_SLICE; 
    set_slice_type(p_Vid, p_Inp, B_SLICE );
    populate_frame_slice_type( p_Inp, p_Vid->p_curr_frm_struct, B_SLICE, p_Vid->p_pred->max_num_slices );
    p_Vid->active_pps = p_Vid->PicParSet[0];
    frame_type_pass = 1;
  }

  if(frame_type_pass)
  {
    p_Vid->write_macroblock = FALSE;
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
    frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
    selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
  printf("rd_pass = %d, selection = %d\n", rd_pass, selection);
#endif

    if (selection)
    {
      swap_frame_buffer(p_Vid, 0, rd_pass); 
      store_coding_and_rc_info(p_Vid, &coding_info);
      best_method = FRAME_TYPE; 
    }
    // reset frame_type
    else 
      frame_type = P_SLICE;

    rd_pass++;
    if(rd_pass >= p_Inp->RDPictureMaxPassPSlice)
    {
      frame_picture_mp_exit(p_Vid, &coding_info);
      return;
    }
  }

  if(p_Vid->EvaluateDBOff)
  {
    // Perform DB off coding pass
    p_Vid->active_pps = (best_method == EXP_WP?p_Vid->PicParSet[1]:p_Vid->PicParSet[0]);
    if(frame_type != P_SLICE)
    {
      set_slice_type(p_Vid, p_Inp, frame_type);
      populate_frame_slice_type( p_Inp, p_Vid->p_curr_frm_struct, frame_type, p_Vid->p_pred->max_num_slices );
    }
    p_Vid->TurnDBOff = 1; 
    p_Vid->write_macroblock = FALSE;
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
    frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
    selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
  printf("DB OFF, rd_pass = %d, selection = %d\n", rd_pass, selection);
#endif

    if (selection)
    {
      swap_frame_buffer(p_Vid, 0, rd_pass); 
      store_coding_and_rc_info(p_Vid, &coding_info);
      best_method = DB_OFF; 
    }

    rd_pass++;
    if(rd_pass >= p_Inp->RDPictureMaxPassPSlice)
    {
      frame_picture_mp_exit(p_Vid, &coding_info);
      return;
    }
  }

  if(p_Inp->RDPictureFrameQPPSlice)
  {
    // frame QP pass
    p_Vid->active_pps = (apply_wp?p_Vid->PicParSet[1]:p_Vid->PicParSet[0]);
    if(frame_type != P_SLICE)
    {
      set_slice_type(p_Vid, p_Inp, frame_type);
      populate_frame_slice_type( p_Inp, p_Vid->p_curr_frm_struct, I_SLICE, p_Vid->p_pred->max_num_slices );
    }
    p_Vid->qp = (p_Vid->nal_reference_idc==0 ? rd_qp+1:rd_qp-1);
    p_Vid->qp = iClip3( p_Vid->RCMinQP, p_Vid->RCMaxQP, p_Vid->qp );
    if ( p_Inp->RCEnable )
    {
      rateRatio = p_Vid->nal_reference_idc ? 1.15F : 0.85F;
      rc_init_frame_rdpic( p_Vid, p_Inp, rateRatio );
    }
    p_Vid->TurnDBOff = 0;
    p_Vid->write_macroblock = FALSE;
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
    frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
    selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
    printf("rd_pass = %d, selection = %d\n", rd_pass, selection);
#endif

    if (selection)
    {
      swap_frame_buffer(p_Vid, 0, rd_pass); 
      store_coding_and_rc_info(p_Vid, &coding_info);
      best_method = FRAME_QP;
    }

    rd_pass++;
    if(rd_pass >= p_Inp->RDPictureMaxPassPSlice)
    {
      frame_picture_mp_exit(p_Vid, &coding_info);
      return;
    }
  }

  frame_picture_mp_exit(p_Vid, &coding_info);
}

void frame_picture_mp_i_slice(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int rd_pass = 0;
  int   qp = p_Vid->qp;
  float rateRatio = 1.0F;
  CodingInfo coding_info; 
  int selection;

  // initial pass encoding
  frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
  store_coding_and_rc_info(p_Vid, &coding_info);

  rd_pass++;
  if(rd_pass >= p_Inp->RDPictureMaxPassISlice)
  {
    frame_picture_mp_exit(p_Vid, &coding_info);
    return;
  }

  {
    // try QP-1
    qp = p_Vid->qp;
    p_Vid->qp = qp - 1;
    p_Vid->qp = iClip3( p_Vid->RCMinQP, p_Vid->RCMaxQP, p_Vid->qp );
    if ( p_Inp->RCEnable )
    {
      rateRatio = 1.15F;
      rc_init_frame_rdpic( p_Vid, p_Inp, rateRatio );
    }

    p_Vid->write_macroblock = FALSE;
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
    frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
    selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], qp);

    if (selection)
    {
      swap_frame_buffer(p_Vid, 0, rd_pass); 
      store_coding_and_rc_info(p_Vid, &coding_info);
    }

    rd_pass++;
    if(rd_pass >= p_Inp->RDPictureMaxPassISlice)
    {
      frame_picture_mp_exit(p_Vid, &coding_info);
      return;
    }

    // try QP+1
    p_Vid->qp    = (qp + 1);
    p_Vid->qp = iClip3( p_Vid->RCMinQP, p_Vid->RCMaxQP, p_Vid->qp );
    p_Vid->write_macroblock = FALSE;

    if ( p_Inp->RCEnable )
    {
      rateRatio = 1.0F;
      rc_init_frame_rdpic( p_Vid, p_Inp, rateRatio );
    }

    p_Vid->qp = iClip3( p_Vid->RCMinQP, p_Vid->RCMaxQP, p_Vid->qp );
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
    frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
    selection  = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], qp);

    if ( selection )
    {
      swap_frame_buffer(p_Vid, 0, rd_pass);
      store_coding_and_rc_info(p_Vid, &coding_info);
    }

    rd_pass++;
    if(rd_pass >= p_Inp->RDPictureMaxPassISlice)
    {
      frame_picture_mp_exit(p_Vid, &coding_info);
      return;
    }
  }

  frame_picture_mp_exit(p_Vid, &coding_info);
}

void frame_picture_mp_b_slice(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int   rd_pass = 0;
  int   rd_qp = p_Vid->p_curr_frm_struct->qp;
  float rateRatio = 1.0F;
  int   wp_pass = 0;
  CodingInfo coding_info;
  FrameCodingMethod best_method = REGULAR;
  int apply_wp = 0;
  int selection;
  Slice *dummy_slice = NULL;

#if (DBG_IMAGE_MP)
  printf("pass0_wp = %d\n", p_Vid->pass0_wp);
#endif  

  frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
  store_coding_and_rc_info(p_Vid, &coding_info);
  
  if(p_Inp->WPIterMC)
    p_Vid->frameOffsetAvail = 1; 

  rd_pass++;
  if(rd_pass >= p_Inp->RDPictureMaxPassBSlice)
  {
    frame_picture_mp_exit(p_Vid, &coding_info);
    return;
  }

  init_slice_lite(p_Vid, &dummy_slice, 0);

#if (DBG_IMAGE_MP)
    printf("rd_pass = %d: %d (%.0f, %.0f, %.0f)\n", rd_pass, 
      p_Vid->frame_pic[0]->bits_per_picture, 
      p_Vid->frame_pic[0]->distortion.value[0], p_Vid->frame_pic[0]->distortion.value[1], p_Vid->frame_pic[0]->distortion.value[2]);
#endif

  // for B_Slice, consider implicit WP 
  wp_pass = 0;
  {    
    if (p_Inp->GenerateMultiplePPS)
    {
      InitWP(p_Vid, p_Inp, 0);
      if ( p_Inp->WPMCPrecision )
        p_Vid->pWPX->curr_wp_rd_pass = p_Vid->pWPX->wp_rd_passes + 1;

      if (p_Vid->TestWPBSlice(dummy_slice, 1) == 1)
      {
        // regular WP pass
        p_Vid->active_pps = p_Vid->PicParSet[2];
        if(p_Inp->WPMCPrecision)
          p_Vid->pWPX->curr_wp_rd_pass->algorithm = WP_REGULAR;
        wp_pass = 1;
      }
    }

    if(wp_pass)
    {
      p_Vid->write_macroblock = FALSE;
      p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
      frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
      selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
      printf("IMP WP, rd_pass = %d, selection = %d\n", rd_pass, selection);
      printf("rd_pass = %d: %d (%.0f, %.0f, %.0f)\n", rd_pass, 
        p_Vid->frame_pic[rd_pass]->bits_per_picture, 
        p_Vid->frame_pic[rd_pass]->distortion.value[0], p_Vid->frame_pic[rd_pass]->distortion.value[1], p_Vid->frame_pic[rd_pass]->distortion.value[2]);
#endif

      if (selection)
      {
        swap_frame_buffer(p_Vid, 0, rd_pass); 
        store_coding_and_rc_info(p_Vid, &coding_info);
        best_method = IMP_WP; 
        apply_wp = IMP_WP;
      }

      rd_pass++;
      if(rd_pass >= p_Inp->RDPictureMaxPassBSlice)
      {
        frame_picture_mp_exit(p_Vid, &coding_info);
        free_slice(dummy_slice);
        return;
      }
    }

    // for B_Slice, consider explicit WP 
    wp_pass = 0;
    if (p_Inp->GenerateMultiplePPS)
    {
      InitWP(p_Vid, p_Inp, 0);
      if( p_Inp->WPMCPrecision )
        p_Vid->pWPX->curr_wp_rd_pass = p_Vid->pWPX->wp_rd_passes + 1;

      if (p_Vid->TestWPBSlice(dummy_slice, 0) == 1)
      {
        // regular WP pass
        p_Vid->active_pps = p_Vid->PicParSet[1];
        if( p_Inp->WPMCPrecision )
          p_Vid->pWPX->curr_wp_rd_pass->algorithm = WP_REGULAR;
        wp_pass = 1;
      }
      else if ( p_Inp->WPMCPrecision == 2 && (p_Inp->WPMCPrecBSlice == 2 || (p_Inp->WPMCPrecBSlice == 1 && p_Vid->nal_reference_idc) ) )
      {
        p_Vid->active_pps = p_Vid->PicParSet[1];
        wp_pass = 1;
      }
    }

    if(wp_pass)
    {
      p_Vid->write_macroblock = FALSE;
      p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
      frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
      selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
      printf("EXP WP, rd_pass = %d, selection = %d\n", rd_pass, selection);
      printf("rd_pass = %d: %d (%.0f, %.0f, %.0f)\n", rd_pass, 
        p_Vid->frame_pic[rd_pass]->bits_per_picture, 
        p_Vid->frame_pic[rd_pass]->distortion.value[0], p_Vid->frame_pic[rd_pass]->distortion.value[1], p_Vid->frame_pic[rd_pass]->distortion.value[2]);
#endif

      if (selection)
      {
        swap_frame_buffer(p_Vid, 0, rd_pass); 
        store_coding_and_rc_info(p_Vid, &coding_info);
        best_method = EXP_WP;
        apply_wp = EXP_WP;
      }

      rd_pass++;
      if(rd_pass >= p_Inp->RDPictureMaxPassBSlice)
      {
        frame_picture_mp_exit(p_Vid, &coding_info);
        free_slice(dummy_slice);
        return;
      }
    }
  }

  if(p_Inp->RDPictureFrameQPBSlice)
  {
    // frame QP pass
    p_Vid->active_pps = best_method == EXP_WP ? p_Vid->PicParSet[1]:best_method==IMP_WP?p_Vid->PicParSet[2]:p_Vid->PicParSet[0];
    //p_Vid->qp = (p_Vid->nal_reference_idc==0 ? rd_qp + 1 : rd_qp - 1);
    p_Vid->qp = (p_Vid->nal_reference_idc==0 ? rd_qp + 1 : rd_qp );
    p_Vid->qp = iClip3( p_Vid->RCMinQP, p_Vid->RCMaxQP, p_Vid->qp );
    if ( p_Inp->RCEnable )
    {
      rateRatio = p_Vid->nal_reference_idc ? 1.15F : 0.85F;
      rc_init_frame_rdpic( p_Vid, p_Inp, rateRatio );
    }

    p_Vid->write_macroblock = FALSE;
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
    frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
    selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
    printf("frame QP, rd_pass = %d, selection = %d \n", rd_pass, selection);
    printf("rd_pass = %d: %d (%.0f, %.0f, %.0f)\n", rd_pass, 
      p_Vid->frame_pic[rd_pass]->bits_per_picture, 
      p_Vid->frame_pic[rd_pass]->distortion.value[0], p_Vid->frame_pic[rd_pass]->distortion.value[1], p_Vid->frame_pic[rd_pass]->distortion.value[2]);
#endif

    if (selection)
    {
      swap_frame_buffer(p_Vid, 0, rd_pass); 
      store_coding_and_rc_info(p_Vid, &coding_info);
      best_method = FRAME_QP; 
    }

    rd_pass++;
    if(rd_pass >= p_Inp->RDPictureMaxPassBSlice)
    {
      frame_picture_mp_exit(p_Vid, &coding_info);
      free_slice(dummy_slice);
      return;
    }
  }

  // alternative direct mode pass
  if(p_Inp->RDPictureDirectMode)
  {
    // consider using the best coding "method" found so far
    p_Vid->active_pps = apply_wp == IMP_WP ? p_Vid->PicParSet[1]:apply_wp==EXP_WP?p_Vid->PicParSet[2]:p_Vid->PicParSet[0];
    if(best_method == FRAME_QP)
      p_Vid->qp = (p_Vid->nal_reference_idc==0 ? rd_qp+1:rd_qp-1);
    else 
      p_Vid->qp = rd_qp; 
    p_Vid->qp = iClip3( p_Vid->RCMinQP, p_Vid->RCMaxQP, p_Vid->qp );
    // flip direct mode type
    p_Vid->direct_spatial_mv_pred_flag = 1-p_Vid->direct_spatial_mv_pred_flag;
    p_Vid->write_macroblock = FALSE;
    p_Vid->p_curr_frm_struct->qp = p_Vid->qp;
    frame_picture (p_Vid, p_Vid->frame_pic[rd_pass], &p_Vid->imgData, rd_pass);
    selection = picture_coding_decision(p_Vid, p_Vid->frame_pic[0], p_Vid->frame_pic[rd_pass], rd_qp);
#if (DBG_IMAGE_MP)
    printf("alternate direct mode, rd_pass = %d, selection = %d\n", rd_pass, selection);
#endif

    if (selection)
    {
      swap_frame_buffer(p_Vid, 0, rd_pass); 
      store_coding_and_rc_info(p_Vid, &coding_info);
      best_method = ALT_DIRECT;
    }

    rd_pass++;
    if(rd_pass >= p_Inp->RDPictureMaxPassBSlice)
    {
      frame_picture_mp_exit(p_Vid, &coding_info);
      free_slice(dummy_slice);
      return;
    }
  }
  frame_picture_mp_exit(p_Vid, &coding_info);
  free_slice(dummy_slice);
  return;
}

void frame_picture_mp(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  p_Vid->EvaluateDBOff = 0;
  p_Vid->TurnDBOff = 0;
  if(p_Vid->type == I_SLICE)
  {
    frame_picture_mp_i_slice(p_Vid, p_Inp);
  }
  else if(p_Vid->type == P_SLICE)
  {
    frame_picture_mp_p_slice(p_Vid, p_Inp);
  }
  else if(p_Vid->type == B_SLICE)
  {
    frame_picture_mp_b_slice(p_Vid, p_Inp);
  }
  p_Vid->rd_pass = 0;
}

