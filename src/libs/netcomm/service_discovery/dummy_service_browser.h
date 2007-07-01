
/***************************************************************************
 *  dummy_service_browser.h - browse services
 *
 *  Created: Fri Jun 29 15:24:15 2007 (on the flight to RoboCup 2007, Atlanta)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __NETCOMM_SERVICE_DISCOVERY_DUMMY_SERVICE_BROWSER_H_
#define __NETCOMM_SERVICE_DISCOVERY_DUMMY_SERVICE_BROWSER_H_

#include <netcomm/service_discovery/service_browser.h>

class DummyServiceBrowser : public ServiceBrowser
{
 public:
  DummyServiceBrowser();
  virtual ~DummyServiceBrowser();

  virtual void add_handler(const char *service_type, ServiceBrowseHandler *h);
  virtual void remove_handler(const char *service_type, ServiceBrowseHandler *h);
};


#endif