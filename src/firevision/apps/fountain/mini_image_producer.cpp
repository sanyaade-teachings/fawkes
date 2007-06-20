
/***************************************************************************
 *  mini_image.cpp - mini image producer
 *
 *  Generated: Tue May 17 09:21:41 2006 (Automatica 2006)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <apps/fountain/mini_image_producer.h>

#include <utils/system/console_colors.h>
#include <utils/logging/logger.h>
#include <fvutils/scalers/scaler.h>
#include <fvutils/ipc/shm_image.h>

/** @class MiniImageProducer <apps/fountain/mini_image_producer.h>
 * Mini image producer.
 * Uses a scaler to create small version of an image.
 */

/** Constructor.
 * @param orig_id original image ID
 * @param mini_id mini image ID
 * @param scaler Scaler
 * @param logger Logger
 */
MiniImageProducer::MiniImageProducer(const char *orig_id, const char *mini_id,
				     Scaler *scaler, Logger *logger)
{
  scale_factor = 0.25;

  this->scaler = scaler;
  scaler->set_scale_factor( scale_factor );

  logger->log_debug("MiniImageProducer", "Opening original image shmem segment for id %s", orig_id);
  orig_shmem = new SharedMemoryImageBuffer( orig_id );

  if ( ! orig_shmem->is_valid() ) {
    logger->log_error("MiniImageProducer", "Could not open original image");
    delete orig_shmem;
    orig_shmem = NULL;
    mini_shmem = NULL;
  } else {

    scaler->set_original_dimensions( orig_shmem->width(), orig_shmem->height() );

    logger->log_debug("MiniImageProducer", "Opening mini image shmem segment for id %s"
		                           ", w=%u, h=%u",
		      mini_id, scaler->needed_scaled_width(), scaler->needed_scaled_height());

    mini_shmem = new SharedMemoryImageBuffer( mini_id, YUV422_PLANAR,
					      scaler->needed_scaled_width(),
					      scaler->needed_scaled_height() );

    if ( ! mini_shmem->is_valid() ) {
      logger->log_error("MiniImageProducer", "Could not open mini image");
      delete orig_shmem;
      delete mini_shmem;
      orig_shmem = NULL;
      mini_shmem = NULL;
    }
  }
}


/** Destructor. */
MiniImageProducer::~MiniImageProducer()
{
  delete orig_shmem;
  delete mini_shmem;
}


/** Check if all data is valid.
 * @return true if shared memory images have been openened successfully and a scaler is
 * set, false otherwise
 */
bool
MiniImageProducer::isValid()
{
  return ( (orig_shmem != NULL) &&
	   (mini_shmem != NULL) &&
	   (scaler != NULL) );
}


/** Produce mini image. */
void
MiniImageProducer::produce()
{
  if ( orig_shmem == NULL ) {
    logger->log_warn("MiniImageProducer", "Original shmem image not opened");
    return;
  }
  if ( mini_shmem == NULL ) {
    logger->log_warn("MiniImageProducer", "Mini shmem image not opened");
    return;
  }

  scaler->set_scale_factor( scale_factor );
  scaler->set_original_dimensions( orig_shmem->width(), orig_shmem->height() );
  scaler->set_scaled_dimensions( mini_shmem->width(), mini_shmem->height() );
  scaler->set_original_buffer( orig_shmem->buffer() );
  scaler->set_scaled_buffer( mini_shmem->buffer() );
  scaler->scale();
}
