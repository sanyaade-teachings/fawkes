
/***************************************************************************
 *  vision_master.h - FireVision Vision Master
 *
 *  Created: Wed May 30 10:28:06 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_FVUTILS_BASE_VISION_MASTER_H_
#define __FIREVISION_FVUTILS_BASE_VISION_MASTER_H_

#include <fvutils/color/colorspaces.h>

class Camera;
namespace fawkes {
  class Thread;
}

class VisionMaster
{
 public:
  virtual ~VisionMaster();

  virtual Camera *  register_for_camera(const char *camera_string,
					fawkes::Thread *thread,
					colorspace_t cspace = YUV422_PLANAR) = 0;
  virtual void      unregister_thread(fawkes::Thread *thread)                = 0;
};



#endif
