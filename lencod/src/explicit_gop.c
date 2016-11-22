/*!
 *************************************************************************************
 * \file explicit_gop.c
 *
 * \brief
 *    Code for explicit gop support and hierarchical coding.
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Alexis Michael Tourapis                     <alexismt@ieee.org>
 *************************************************************************************
 */

#include "contributors.h"

#include <ctype.h>
#include <limits.h>
#include "global.h"

#include "explicit_gop.h"
#include "image.h"
#include "nalucommon.h"
#include "report.h"

/*!
************************************************************************
* \brief
*    Initialization of GOP structure.
*
************************************************************************
*/
void init_gop_structure(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int max_gopsize = p_Inp->NumberBFrames;

  p_Vid->gop_structure = calloc(imax(10,max_gopsize), sizeof (GOP_DATA)); // +1 for reordering
  if (NULL==p_Vid->gop_structure)
    no_mem_exit("init_gop_structure: gop_structure");
}


/*!
************************************************************************
* \brief
*    Clear GOP structure
************************************************************************
*/
void clear_gop_structure(VideoParameters *p_Vid)
{
  if (p_Vid->gop_structure)
    free(p_Vid->gop_structure);
}


/*!
************************************************************************
* \brief
*    Interpret GOP struct from input parameters
************************************************************************
*/
void interpret_gop_structure(VideoParameters *p_Vid, InputParameters *p_Inp)
{
  int nLength = (int) strlen(p_Inp->ExplicitHierarchyFormat);
  int i =0, k, dqp, display_no;
  int slice_read =0, order_read = 0, stored_read = 0, qp_read =0;
  int coded_frame = 0;
  int tlyr, temporal_layer_read = 0; 

  if (nLength > 0)
  {

    for (i = 0; i < nLength ; i++)
    {
      //! First lets read slice type
      if (slice_read == 0)
      {
        switch (p_Inp->ExplicitHierarchyFormat[i])
        {
        case 'P':
        case 'p':
          p_Vid->gop_structure[coded_frame].slice_type = P_SLICE;
          break;
        case 'B':
        case 'b':
          p_Vid->gop_structure[coded_frame].slice_type = B_SLICE;
          break;
        case 'I':
        case 'i':
          p_Vid->gop_structure[coded_frame].slice_type = I_SLICE;
          break;
        default:
          snprintf(errortext, ET_SIZE, "Slice Type invalid in ExplicitHierarchyFormat param. Please check configuration file.");
          error (errortext, 400);
          break;
        }
        slice_read = 1;
      }
      else
      {
        //! Next is Display Order
        if (order_read == 0)
        {
          if (isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i))))
          {
            sscanf(p_Inp->ExplicitHierarchyFormat+i,"%d",&display_no);
            p_Vid->gop_structure[coded_frame].display_no = display_no;
            order_read = 1;
            if (display_no < 0 || display_no >= p_Inp->NumberBFrames)
            {
              snprintf(errortext, ET_SIZE, "Invalid Frame Order value. Frame position needs to be in [0,%d] range.",p_Inp->NumberBFrames - 1);
              error (errortext, 400);
            }
            for (k=0;k<coded_frame;k++)
            {
              if (p_Vid->gop_structure[k].display_no == display_no)
              {
                snprintf(errortext, ET_SIZE, "Frame Order value %d in frame %d already used for enhancement frame %d.",display_no,coded_frame,k);
                error (errortext, 400);
              }
            }
          }
          else
          {
            snprintf(errortext, ET_SIZE, "Slice Type needs to be followed by Display Order. Please check configuration file.");
            error (errortext, 400);
          }
        }
        else if (order_read == 1)
        {
          if (stored_read == 0 && !(isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i)))))
          {
            switch (p_Inp->ExplicitHierarchyFormat[i])
            {
            case 'E':
            case 'e':
              p_Vid->gop_structure[coded_frame].reference_idc = NALU_PRIORITY_DISPOSABLE;
              p_Vid->gop_structure[coded_frame].hierarchy_layer = 0;
              break;
            case 'R':
            case 'r':
              p_Vid->gop_structure[coded_frame].reference_idc= NALU_PRIORITY_LOW;
              p_Vid->gop_structure[coded_frame].hierarchy_layer = 1;
              break;
            default:
              snprintf(errortext, ET_SIZE, "Reference_IDC invalid in ExplicitHierarchyFormat param. Please check configuration file.");
              error (errortext, 400);
              break;
            }
            stored_read = 1;
          }
          else if (stored_read == 1 && qp_read == 0)
          {
            if (isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i))))
            {
              sscanf(p_Inp->ExplicitHierarchyFormat+i,"%d",&dqp);

              p_Vid->gop_structure[coded_frame].slice_qp_off = iClip3(-p_Vid->bitdepth_luma_qp_scale, 51, dqp);
              qp_read = 1;
            }
            else
            {
              snprintf(errortext, ET_SIZE, "Reference_IDC needs to be followed by QP. Please check configuration file.");
              error (errortext, 400);
            }
          }
          else if (stored_read == 1 && qp_read == 1 && temporal_layer_read == 0)
          {
            if (!(isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i)))))
            {
              if (p_Inp->ExplicitHierarchyFormat[i] == 't' || p_Inp->ExplicitHierarchyFormat[i] == 'T')
              {
                p_Vid->gop_structure[coded_frame].temporal_layer = 0; 
                if (isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i+1))))
                {
                  sscanf(p_Inp->ExplicitHierarchyFormat+i+1,"%d",&tlyr);

                  p_Vid->gop_structure[coded_frame].temporal_layer = tlyr; 
                  i++; // temporal layer number is specified
                }
              }
              else
              {
                i--; // temporal layer is optional and it is not specified
              }
            }
            temporal_layer_read = 1;
          }
          else if (stored_read == 1 && qp_read == 1 && temporal_layer_read == 1 && !(isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i)))) && (i < nLength - 3))

          {
            stored_read =0;
            qp_read=0;
            order_read=0;
            slice_read=0;
            temporal_layer_read=0;
            i--;
            coded_frame ++;
            if (coded_frame >= p_Inp->NumberBFrames )
            {
              snprintf(errortext, ET_SIZE, "Total number of frames in Enhancement GOP need to be fewer or equal to NumberBFrames parameter.");
              error (errortext, 400);
            }
          }
        }
      }
    }
  }
  else
  {
    snprintf(errortext, ET_SIZE, "ExplicitHierarchyFormat is empty. Please check configuration file.");
    error (errortext, 400);
  }

  p_Inp->NumberBFrames = coded_frame + 1;
}

