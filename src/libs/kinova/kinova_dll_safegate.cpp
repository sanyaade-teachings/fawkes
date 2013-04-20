
/***************************************************************************
 *  kinova_dll_safegate.cpp - C# to C++ wrapper for assembly Kinova.DLL.SafeGate
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

#include "kinova_dll_safegate.h"

#include <cstdio>

#include <mono/jit/jit.h>
#include <mono/metadata/assembly.h>
#include <mono/metadata/object.h>
#include <mono/metadata/debug-helpers.h>

namespace Kinova { namespace DLL { namespace SafeGate
{


/* /================================\
 *         Assembly
 * \================================/*/
/** Constructor.
 * @param path The full path to the directory containing the DLL file.
 */
Assembly::Assembly(const char* path)
  : KinovaMonoAssembly("Kinova.DLL.SafeGate", path)
{
  __inits.push_back( KinovaMonoClass<Crypto>::init );
  __inits.push_back( KinovaMonoClass<CCypherMessage>::init );
}

/** Desctructor. */
Assembly::~Assembly()
{
}

//*
/* /================================\
 *         CCypherMessage
 * \================================/*/
/** Constructor. */
CCypherMessage::CCypherMessage()
{
  MonoObject* object = mono_object_new(__domain, __class);
  mono_runtime_object_init(object);
  if( !object )
    printf("ERROR!!!! could not create instance of CCypherMessage \n");

  __object = (MyMonoObject*)object;
}

/** Constructor.
 * This version might come in handy.
 * @param object A MonoObject pointer to the already existing CCypherMessage object.
 */
CCypherMessage::CCypherMessage(MyMonoObject* object)
{
  __object = object;
  if( !__object )
    printf("ERROR!!!! could not create instance of CCypherMessage \n");
}

/** Desctructor. */
CCypherMessage::~CCypherMessage()
{
}

/** Initialize the CCypherMessage class and its methods. Needs to be done once.
 * @param image The image of the assembly.
 * @param class_namespace The namespace which the class belongs to. Usually the name of the assembly.
 * @return Possible error. (ERROR_NONE == 0)
 */
KinovaMonoError_t
CCypherMessage::init(MonoImage* image, const char* class_namespace)
{
  // get mono class for this class
  __class = mono_class_from_name(image, class_namespace, "CCypherMessage");
  if( !__class )
    return MONO_ERROR_CLASS;

  return MONO_ERROR_NONE;
}



/* /================================\
 *         Class Crypto
 * \================================/*/
// init static members
Crypto*      Crypto::__instance = NULL;
MonoMethod*  Crypto::__m_GetInstance = NULL;
MonoMethod*  Crypto::__m_Encrypt = NULL;

/** Private constructor. */
Crypto::Crypto()
{
  MonoObject* ex;
  __object = (MyMonoObject*)mono_runtime_invoke(__m_GetInstance, NULL, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);

  if( !__object )
    printf("ERROR!!!! could not create instance of Crypto \n");
}

/** Private desctructor. */
/*
Crypto::~Crypto()
{
}
//*/


/** Initialize the Crypto class. Needs to be done once.
 * @param image The image of the assembly.
 * @param class_namespace The namespace which the class belongs to. Usually the name of the assembly.
 * @return Possible error. (ERROR_NONE == 0)
 */
//*
KinovaMonoError_t
Crypto::init(MonoImage* image, const char* class_namespace)
{
  // get mono class for this class
  __class = mono_class_from_name(image, class_namespace, "Crypto");
  if( !__class )
    return MONO_ERROR_CLASS;

  // get the methods of the class
  __m_GetInstance = mono_class_get_method_from_name(__class, "GetInstance", 0);
  __m_Encrypt = mono_class_get_method_from_name(__class, "Encrypt", 1);
  if( !__m_GetInstance || !__m_Encrypt )
    return MONO_ERROR_METHOD;

  return MONO_ERROR_NONE;
}
//*/

/** Get the single existing instance of this class.
 * @return Pointer to the singleton instance.
 */
Crypto*
Crypto::GetInstance()
{
  if( !__instance ) {
    __instance = new Crypto();
  }

  return __instance;
}

/** Get an encrypted CCyphermessage.
 * Missing apidoc in original API.
 * @param password The passwort, that should be encrypted.
 * @return Pointer to the CCypherMessage object.
 */
CCypherMessage*
Crypto::Encrypt(const char* password)
{
  MonoObject* ex;
  MonoObject* object;
  void *args[1];
  args[0] = mono_string_new(__domain, password);
  object = mono_runtime_invoke(__m_Encrypt, __object, args, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);

  if( !__object )
    throw KinovaException("Could not get Encrypt object");

  return new CCypherMessage((MyMonoObject*)object);
}

}}} // namespace Kinova::DLL::SafeGate
