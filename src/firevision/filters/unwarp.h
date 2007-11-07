
/***************************************************************************
 *  unwarp.h - Header of unwarp filter
 *
 *  Created: Mon Jul 25 11:20:49 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_FILTER_UNWARP_H_
#define __FIREVISION_FILTER_UNWARP_H_

#include <filters/filter.h>

class MirrorModel;

class FilterUnwarp : public Filter
{
 public:
  FilterUnwarp(MirrorModel *mm);

  virtual void apply();

 private:
  MirrorModel   *mm;
};

#endif

