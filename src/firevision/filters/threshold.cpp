
/***************************************************************************
 *  threshold.cpp - Implementation for threshold filter, this filter will
 *                  luminance values below a given threshold to the given
 *                  min_replace value, values above a given max threshold
 *                  will be set to the max_replace value
 *
 *  Created: Tue Jun 07 14:30:10 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <filters/threshold.h>

#include <core/exception.h>

#include <cstddef>
#include <ippi.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterThreshold <filters/threshold.h>
 * Threshold filter
 */

/** Constructor.
 * @param min minimum value
 * @param min_replace values below min are replaced with this value
 * @param max maximum value
 * @param max_replace values above max are replaced with this value
 */
FilterThreshold::FilterThreshold(unsigned char min, unsigned char min_replace,
				 unsigned char max, unsigned char max_replace)
  : Filter("FilterThreshold")
{
  this->min = min;
  this->max = max;
  this->min_replace = min_replace;
  this->max_replace = max_replace;
}


/** Set new thresholds.
 * @param min minimum value
 * @param min_replace values below min are replaced with this value
 * @param max maximum value
 * @param max_replace values above max are replaced with this value
 */
void
FilterThreshold::set_thresholds(unsigned char min, unsigned char min_replace,
				unsigned char max, unsigned char max_replace)
{
  this->min = min;
  this->max = max;
  this->min_replace = min_replace;
  this->max_replace = max_replace;
}


void
FilterThreshold::apply()
{
  IppiSize size;
  size.width = src_roi[0]->width;
  size.height = src_roi[0]->height;

  IppStatus status;

  if ((dst == NULL) || (dst == src[0])) {
    // In-place
    status = ippiThreshold_GTVal_8u_C1IR( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
					  size, max, max_replace );
    if ( status == ippStsNoErr ) {
      status = ippiThreshold_LTVal_8u_C1IR( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
					    size, min, min_replace );
    }
  } else {
    //                                base + number of bytes to line y              + pixel bytes
    status = ippiThreshold_GTVal_8u_C1R( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
					 dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
					 size, max, max_replace );

    if ( status == ippStsNoErr ) {
      status = ippiThreshold_LTVal_8u_C1R( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
					   dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
					   size, min, min_replace );
    }
  }

  if ( status != ippStsNoErr ) {
    throw fawkes::Exception("Threshold filter failed with %i\n", status);
  }

}

} // end namespace firevision
