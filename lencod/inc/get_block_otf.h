/*!
 *************************************************************************************
 * \file get_block_otf.h
 *
 * \brief
 *    get predicted (chroma|luma) block with subpel interpolation computes on-the-fly
 *
 *
 *************************************************************************************
 */


#ifndef _GET_BLOCK_OTF_H_
#define _GET_BLOCK_OTF_H_

void get_block_luma_otf_L2(  VideoParameters *p_Vid,  //!< video encoding parameters for current picture
                      imgpel*   mpred,         //!< array of prediction values (row by row)
                      int*   tmp_pred,         //!< array of temporary prediction values (row by row), used for some hal-pel interpolations
                      int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                      int    pic_pix_y,        //!< motion shifted vertical   coordinate of block
                      int    block_size_x,   //!< horizontal block size
                      int    block_size_y,   //!< vertical block size
                      StorablePicture *ref,    //!< reference picture list
                      int    pl                //!< plane
                    );

void get_block_luma_otf_L1(  VideoParameters *p_Vid,  //!< video encoding parameters for current picture
                      imgpel*   mpred,         //!< array of prediction values (row by row)
                      int*   tmp_pred,         //!< array of temporary prediction values (row by row), used for some hal-pel interpolations
                      int    pic_pix_x,        //!< motion shifted horizontal coordinate of block
                      int    pic_pix_y,        //!< motion shifted vertical   coordinate of block
                      int    block_size_x,   //!< horizontal block size
                      int    block_size_y,   //!< vertical block size
                      StorablePicture *ref,    //!< reference picture list
                      int    pl                //!< plane
                    ) ;


void get_block_chroma_otf_L2 ( VideoParameters *p_Vid, //!< video encoding parameters for current picture
                        imgpel* mpred,          //!< array to store prediction values
                        int*   tmp_pred,
                        int    pic_pix_x,          //!< motion shifted horizontal coordinate of block
                        int    pic_pix_y,          //!< motion shifted vertical  block
                        int    block_size_x, //!< horizontal block size
                        int    block_size_y, //!< vertical block size                                      
                        StorablePicture *ref,  //!< reference picture list
                        int    uv                //!< chroma component
                        ) ;

void me_get_block_chroma_otf_L1 ( VideoParameters *p_Vid, //!< video encoding parameters for current picture
                        imgpel* mpred,          //!< array to store prediction values
                        int*   tmp_pred,
                        int    pic_pix_x,          //!< motion shifted horizontal coordinate of block
                        int    pic_pix_y,          //!< motion shifted vertical  block
                        int    block_size_x, //!< horizontal block size
                        int    block_size_y, //!< vertical block size                                      
                        StorablePicture *ref,  //!< reference picture list
                        int    uv                //!< chroma component
                        ) ;

void mc_get_block_chroma_otf_L1 ( VideoParameters *p_Vid, //!< video encoding parameters for current picture
                        imgpel* mpred,          //!< array to store prediction values
                        int*   tmp_pred,
                        int    pic_pix_x,          //!< motion shifted horizontal coordinate of block
                        int    pic_pix_y,          //!< motion shifted vertical  block
                        int    block_size_x, //!< horizontal block size
                        int    block_size_y, //!< vertical block size                                      
                        StorablePicture *ref,  //!< reference picture list
                        int    uv                //!< chroma component
                        ) ;




//static inline int is_pel( int x, int y, int bx, int by ) 
//{
//  return ( (((x & bx)==0) && ((y & by)==0)) ? (1):(0) );
//}
//
//static inline int is_hpel( int x, int y, int bx, int by ) 
//{
//  return ( ((( (x & bx)==2)&&((y & by)==2)) || (((x & bx)==2)&&((y & by)==0)) || (((x & bx)==0)&&((y & by)==2)) ) ? (1):(0) );
//}
//
//static inline int is_qpel( int x, int y, int bx, int by ) 
//{
//  return ( !is_pel(x,y,bx,by) && !is_hpel(x,y,bx,by) );
//}

static inline int is_qpel( int x, int y, int bx, int by ) 
{
  return ( ((x&bx)%2) || ((y&by)%2) ) ? (1):(0) ;
}

#endif

