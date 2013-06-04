
/***************************************************************************
 *  kinova_dll_data.cpp - C# to C++ wrapper for assembly Kinova.DLL.Data
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

#include "kinova_dll_data.h"

#include <cstdio>

#include <mono/jit/jit.h>
#include <mono/metadata/assembly.h>
#include <mono/metadata/object.h>
#include <mono/metadata/debug-helpers.h>

namespace Kinova { namespace DLL { namespace Data
{


/* /================================\
 *         Assembly
 * \================================/*/
/** Constructor.
 * @param path The full path to the directory containing the DLL file.
 */
Assembly::Assembly(const char* path)
  : KinovaMonoAssembly("Kinova.DLL.Data", path)
{
  __inits.push_back( KinovaMonoClass<CPosition>::init );
}

/** Desctructor. */
Assembly::~Assembly()
{
}

//*
/* /================================\
 *         CPosition
 * \================================/*/
MonoMethod*  CPosition::__m_Init = NULL;
/** Constructor. */
CPosition::CPosition()
{
  MonoObject* object = mono_object_new(__domain, __class);
  mono_runtime_object_init(object);
  if( !object )
    printf("ERROR!!!! could not create instance of CPosition \n");

  __object = (MyMonoObject*)object;
}

/** Desctructor. */
CPosition::~CPosition()
{
}

/** Initialize the CPosition class and its methods. Needs to be done once.
 * @param image The image of the assembly.
 * @param class_namespace The namespace which the class belongs to. Usually the name of the assembly.
 * @return Possible error. (ERROR_NONE == 0)
 */
KinovaMonoError_t
CPosition::init(MonoImage* image, const char* class_namespace)
{
  // get mono class for this class
  __class = mono_class_from_name(image, "Kinova.DLL.Data.Jaco.Diagnostic", "CPosition");
  if( !__class )
    return MONO_ERROR_CLASS;

  // get the methods of the class
  __m_Init = mono_class_get_method_from_name(__class, "Init", 0);
  if( !__m_Init )
    return MONO_ERROR_METHOD;

  return MONO_ERROR_NONE;
}

void
CPosition::Init()
{
  MonoObject* ex;
  mono_runtime_invoke(__m_Init, __object, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);
}

}}} // namespace Kinova::DLL::SafeGate
