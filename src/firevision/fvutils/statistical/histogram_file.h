
/***************************************************************************
 *  histogram_file.h - Histogram file
 *
 *  Created: Sat Mar 29 16:03:12 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __FIREVISION_FVUTILS_STATISTICAL_HISTOGRAM_FILE_H_
#define __FIREVISION_FVUTILS_STATISTICAL_HISTOGRAM_FILE_H_

#define FIREVISION_HISTOGRAM_MAGIC  0xFF04
#define FIREVISION_HISTOGRAM_CURVER 1

#include <fvutils/fileformat/fvfile.h>
#include <fvutils/base/roi.h>
#include <vector>
#include <map>

class HistogramBlock;

class HistogramFile : public FireVisionDataFile
{
 public:
  HistogramFile();
  ~HistogramFile();
  
  void add_histogram_block(HistogramBlock* block);

  /** Convenience typdef for a STL list of pointers to histogram blocks. */
  typedef std::list<HistogramBlock*> HistogramBlockList;  
  HistogramBlockList histogram_blocks();

  uint32_t get_value(hint_t object_type, 
		     uint16_t x, uint16_t y, uint16_t z);
  
  void set_value(hint_t object_type, 
		 uint16_t x, uint16_t y, uint16_t z,
		 uint32_t val);

 private:
  std::map<hint_t, HistogramBlock*> attached_histograms;
};

#endif /* __FIREVISION_FVUTILS_STATISTICAL_HISTOGRAM_FILE_H_ */