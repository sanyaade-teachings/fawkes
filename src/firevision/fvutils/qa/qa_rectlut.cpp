
/***************************************************************************
 *  qa_rectlut.h - QA for rectification LUT
 *
 *  Generated: Wed Oct 32 18:03:48 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include <fvutils/rectification/rectfile.h>
#include <fvutils/rectification/rectinfo_lut_block.h>

#include <cstdlib>
#include <iostream>

using namespace std;

#define WIDTH  640
#define HEIGHT 480

int
main(int argc, char **argv)
{
  srand(23423);

  const char *s = "qatest.rif";
  if ( argc > 1 ) {
    s = argv[1];
  }

  RectificationInfoFile *rif = new RectificationInfoFile(0xDEADBEEFDEADBEEF);

  RectificationLutInfoBlock *rlib = new RectificationLutInfoBlock(WIDTH, HEIGHT,
								  FIREVISION_RECTINFO_CAMERA_MAIN);

  RectificationLutInfoBlock *rlib2 = new RectificationLutInfoBlock(WIDTH, HEIGHT,
								   FIREVISION_RECTINFO_CAMERA_LEFT);

  /* Random alternative, harder to read though
  for ( int i = 0; i < 10; ++i ) {
    uint16_t x, y, to_x, to_y;
    x=1+(uint16_t)(1.f * WIDTH * rand() / (RAND_MAX + 1.f));
    y=1+(uint16_t)(1.f * HEIGHT * rand() / (RAND_MAX + 1.f));
    to_x=1+(uint16_t)(1.f * WIDTH * rand() / (RAND_MAX + 1.f));
    to_y=1+(uint16_t)(1.f * HEIGHT * rand() / (RAND_MAX + 1.f));

    printf("Mapping (%u, %u) to (%u, %u)\n", x, y, to_x, to_y);
    rlib->set_mapping(x, y, to_x, to_y);
  }
  */

  for ( int i = 0; i < 10; ++i ) {
    uint16_t x = i, y = i, to_x = i * 2, to_y = i * 2;
    printf("Mapping (%u, %u) to (%u, %u)\n", x, y, to_x, to_y);
    rlib->set_mapping(x, y, to_x, to_y);
  }

  for ( int i = 10; i < 20; ++i ) {
    uint16_t x = i, y = i, to_x = i * 2, to_y = i * 2;
    printf("Mapping2 (%u, %u) to (%u, %u)\n", x, y, to_x, to_y);
    rlib2->set_mapping(x, y, to_x, to_y);
  }

  rif->add_rectinfo_block(rlib);
  rif->add_rectinfo_block(rlib2);

  std::list<RectificationInfoBlock *> &blocks = rif->blocks();

  for (std::list<RectificationInfoBlock *>::iterator i = blocks.begin(); i != blocks.end(); ++i) {
    RectificationLutInfoBlock *rlib = dynamic_cast<RectificationLutInfoBlock *>(*i);
    if ( rlib == NULL ) {
      printf("Got rectification info block of unknown type");
      continue;
    }

    printf("LUT:  type: %u  camera: %u  size: %u\n",
	   rlib->type(), rlib->camera(), rlib->size());

    cout << "Looking for non-zero mappings" << endl;
    uint16_t x, y, to_x, to_y;
    for ( y = 0; y < HEIGHT; ++y) {
      for ( x = 0; x < WIDTH; ++x) {
	// Use evil (slow) method here, it's just for the QA...
	rlib->mapping(x, y, &to_x, &to_y);
	if ( (to_x != 0) || (to_y != 0) ) {
	  printf("(%u, %u) maps to (%u, %u)\n", x, y, to_x, to_y);
	}
      }
    }
  }

  cout << "Writing to " << s << endl;
  rif->write(s);

  rif->clear();

  cout << "Reading from " << s << endl;
  rif->read(s);

  blocks = rif->blocks();

  for (std::list<RectificationInfoBlock *>::iterator i = blocks.begin(); i != blocks.end(); ++i) {
    RectificationLutInfoBlock *rlib = dynamic_cast<RectificationLutInfoBlock *>(*i);
    if ( rlib == NULL ) {
      printf("Got rectification info block of unknown type");
      continue;

    }

    printf("LUT:  type: %u  camera: %u  size: %u\n",
	   rlib->type(), rlib->camera(), rlib->size());

    cout << "Looking for non-zero mappings" << endl;
    uint16_t x, y, to_x, to_y;
    for ( y = 0; y < HEIGHT; ++y) {
      for ( x = 0; x < WIDTH; ++x) {
	// Use evil (slow) method here, it's just for the QA...
	rlib->mapping(x, y, &to_x, &to_y);
	if ( (to_x != 0) || (to_y != 0) ) {
	  printf("(%u, %u) maps to (%u, %u)\n", x, y, to_x, to_y);
	}
      }
    }
  }

  delete rif;
  return 0;
}



/// @endcond