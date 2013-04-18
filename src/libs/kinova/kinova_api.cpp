
/***************************************************************************
 *  kinova_api.cpp - C# to C++ wrapper for Kinova API
 *
 *  Created: Tue Apr 02 16:32:12 2013
 *  Copyright  2013  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#include "kinova_api.h"

#include <mono/jit/jit.h>

#include <cstdio>

namespace Kinova
{

/** Constructor.
 * @param name Arbitrary name for this API, which will be used to identify the mono-domain.
 * @param path Full path to the directory containing all DLL files.
 */
JacoAPI::JacoAPI(const char* name, const char* path)
  : __name(name),
    __path(path),
    __domain(0)
{
}

/** Destructor. */
JacoAPI::~JacoAPI()
{
}

/** Initialize the API.
 * This creates the mono framework, loads all known assemblies and initializes them.
 * If this does not return any error, the API is fully loaded and all the connections
 * to mono are complete.
 *
 * @return Possible error. (ERROR_NONE == 0)
 */
KinovaMonoError_t
JacoAPI::init()
{
  //create the mono framework
  __domain = mono_jit_init(__name);
  if(!__domain)
    return MONO_ERROR_DOMAIN;

  //create the containers for the mono assembblies
  __assemblies.push_back(new API::Jaco::Assembly(__path));
  __assemblies.push_back(new DLL::SafeGate::Assembly(__path));

  //initialize the assemblies in the containers
  KinovaMonoError_t error;
  for( std::list<KinovaMonoAssembly*>::iterator it=__assemblies.begin(); it!=__assemblies.end(); ++it) {
    error = (*it)->init(__domain);
    if( error != MONO_ERROR_NONE )
      return error;
  }

  return MONO_ERROR_NONE;
}


} // namespace Kinova
