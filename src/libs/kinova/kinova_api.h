
/***************************************************************************
 *  kinova_api.h - C# to C++ wrapper for Kinova API
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

#ifndef KINOVA_API_H
#define KINOVA_API_H

#include "kinova_mono_types.h"

//include all assembly headers here
#include "kinova_dll_safegate.h"
#include "kinova_api_jaco.h"

#include <list>

namespace Kinova
{

/** This is the main API class.
 * Users that want to use this API, should create an instance of this class
 * and initialize its components.
 * That implies, that one only needs to include this header to have access
 * to all the provided API components. Of course one could manually load
 * the assemblies as this functionality is provided, but that would need
 * advanced knowledge and it would complicate the use of the API.
 */
class JacoAPI
{
 public:
  JacoAPI(const char* name, const char* path="");
  virtual ~JacoAPI();

  virtual KinovaMonoError_t init();

 private:
   const char*  __name;
   const char*  __path;
   MonoDomain*  __domain;
   std::list<KinovaMonoAssembly*> __assemblies;
};

} // namespace Kinova
#endif
