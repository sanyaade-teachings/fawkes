
/***************************************************************************
 *  kinova_mono_types.cpp
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

#include "kinova_mono_types.h"
//#include "kinova_dll_safegate.h"

#include <mono/jit/jit.h>
#include <mono/metadata/object.h>
#include <mono/metadata/assembly.h>

#include <cstdio>
#include <string>

namespace Kinova
{

/** A MonoObject wrapper, to allow forward declaration of an unnamed struct.
 * This means, that we need to use MyMonoObject in the headers and our classes,
 * and cast here and there with (MonoObject*) and (MyMonoObject*) when working
 * internally with the mono environment.
 */
struct MyMonoObject : public MonoObject {};

/* /================================\
 *         KinovaException
 * \================================/*/
/** Constructor.
 * @param msg Exception message
 */
KinovaException::KinovaException(const char* msg) throw()
{
  __msg = msg;
}

/** Constructor.
 * @param object The MonoObject holding the exception
 */
KinovaException::KinovaException(MyMonoObject* object) throw()
{
  __msg = mono_string_to_utf8(mono_object_to_string((MonoObject*)object, NULL));
}

/** Get exception message.
 * @return The Exception message.
 */
const char*
KinovaException::what() const throw()
{
  if ( __msg != NULL ) {
    return __msg;
  } else {
    return "Unknown Exception";
  }
}



/* /================================\
 *         KinovaMonoAssembly
 * \================================/*/
/** Constructor.
 * @param name The name of the assembly. Full name of the DLL file without the suffix.
 * @param path The full path to the assembly file.
 */
KinovaMonoAssembly::KinovaMonoAssembly(const char* name, const char* path)
  : __name(name)
{
  __path = path;
  __path.append(__name);
  __path.append(".dll");
}

/** Destructor. */
KinovaMonoAssembly::~KinovaMonoAssembly()
{
}

const char*
KinovaMonoAssembly::get_name() const
{
  return __name;
}

MonoDomain*
KinovaMonoAssembly::get_domain() const
{
  return __domain;
}

MonoAssembly*
KinovaMonoAssembly::get_assembly() const
{
  return __assembly;
}

MonoImage*
KinovaMonoAssembly::get_image() const
{
  return __image;
}


/** Init the assembly.
 * This procedure needs to be executed once for every assembly before it can be used.
 * @param domain The active mono-domain.
 * @return Possible error. (ERROR_NONE == 0)
 */
KinovaMonoError_t
KinovaMonoAssembly::init(MonoDomain* domain)
{
  if( __initialized )
    return MONO_ERROR_NONE;

  if(!domain)
    return MONO_ERROR_DOMAIN;

  // init assembly
  __domain = domain;
  __assembly = mono_domain_assembly_open(__domain, __path.c_str());
  if(!__assembly)
    return MONO_ERROR_ASSEMBLY;

  // get assembly image
  __image = mono_assembly_get_image(__assembly);
  if(!__image)
    return MONO_ERROR_IMAGE;

  // init assembly classes
  KinovaMonoError_t error = MONO_ERROR_NONE;
  for( std::list<init_func>::iterator it=__inits.begin(); it!=__inits.end() && error==MONO_ERROR_NONE; ++it) {
    error = (*it)(this);
  }

  if( error == MONO_ERROR_NONE )
    __initialized = true;

  return error;

  //return init_classes();
  //return MONO_ERROR_NONE;
}

} // namespace Kinova