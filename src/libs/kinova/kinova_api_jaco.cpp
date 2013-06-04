
/***************************************************************************
 *  kinova_api_jaco.cpp - C# to C++ wrapper for assembly Kinova.API.Jaco
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

#include "kinova_api_jaco.h"
#include "kinova_dll_safegate.h"
#include "kinova_dll_data.h"

#include <cstdio>

#include <mono/jit/jit.h>
#include <mono/metadata/assembly.h>
#include <mono/metadata/object.h>
#include <mono/metadata/debug-helpers.h>

namespace Kinova { namespace API { namespace Jaco
{


/* /================================\
 *         Assembly
 * \================================/*/
/** Constructor.
 * @param path The full path to the directory containing the DLL file.
 */
Assembly::Assembly(const char* path)
  : KinovaMonoAssembly("Kinova.API.Jaco", path)
{
  __inits.push_back( KinovaMonoClass<CJacoConfigurationManager>::init );
  __inits.push_back( KinovaMonoClass<CJacoDiagnosticDataManager>::init );
  __inits.push_back( KinovaMonoClass<CJacoDiagnosticManager>::init );
  __inits.push_back( KinovaMonoClass<CJacoArm>::init );
}

Assembly::~Assembly()
{
}



//*
/* /================================\
 *         Class CJacoConfigurationManager
 * \================================/*/
MonoMethod*  CJacoConfigurationManager::__m_ctor = NULL;
MonoMethod*  CJacoConfigurationManager::__m_GetHandPosition = NULL;

/** Constructor, taking a CCypherMessage (encrypted password).
 * This is the way the original API intends it to be used.
 * @param encPassword The encrypted password, enclosed in a CCypherMessage*.
 */
CJacoConfigurationManager::CJacoConfigurationManager(Kinova::DLL::SafeGate::CCypherMessage* encPassword)
{
  create(encPassword);
}

/** Constructor, taking a plain password.
 * This constructor is added for convenience.
 * @param password The non-encrypted password in plain text.
 */
CJacoConfigurationManager::CJacoConfigurationManager(const char* password)
{
  Kinova::DLL::SafeGate::Crypto* c = Kinova::DLL::SafeGate::Crypto::GetInstance();
  Kinova::DLL::SafeGate::CCypherMessage* encPassword = c->Encrypt(password);
  create(encPassword);
}

/** Constructor, taking an object.
 * @param object The actual object.
 */
CJacoConfigurationManager::CJacoConfigurationManager(MyMonoObject* object)
{
  __object = object;
  if( !__object )
    throw KinovaException("Could not create CJacoConfigurationManager, the MonoObject was NULL");
}

/** Destructor. */
CJacoConfigurationManager::~CJacoConfigurationManager()
{
}

/** Private method that creates the actual MonoObject for the Jaco-arm. */
void
CJacoConfigurationManager::create(Kinova::DLL::SafeGate::CCypherMessage* encPassword)
{
  MonoObject* ex;
  void *args[1];
  args[0] = encPassword->get_object();
  mono_runtime_invoke(__m_ctor, __object, args, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);
}

/** Initialize the CJacoConfigurationManager class and its methods. Needs to be done once.
 * @param image The image of the assembly.
 * @param class_namespace The namespace which the class belongs to. Usually the name of the assembly.
 * @return Possible error. (ERROR_NONE == 0)
 */
KinovaMonoError_t
CJacoConfigurationManager::init(MonoImage* image, const char* class_namespace)
{
  // get mono class for this class
  __class = mono_class_from_name(image, "Kinova.API.Jaco.Configurations", "CJacoConfigurationManager");
  if( !__class )
    return MONO_ERROR_CLASS;

  // get the methods of the class
  __m_ctor = mono_class_get_method_from_name(__class, ".ctor", 1 /*number of params*/);
  __m_GetHandPosition = mono_class_get_method_from_name(__class, "GetHandPosition", 0);
  if( !__m_ctor || !__m_GetHandPosition )
    return MONO_ERROR_METHOD;

  return MONO_ERROR_NONE;
}



//*
/* /================================\
 *         Class CJacoDiagnosticDataManager
 * \================================/*/
MonoMethod*  CJacoDiagnosticDataManager::__m_ctor = NULL;
MonoMethod*  CJacoDiagnosticDataManager::__m_GetPositionLogLiveFromJaco = NULL;

/** Constructor, taking a CCypherMessage (encrypted password).
 * This is the way the original API intends it to be used.
 * @param encPassword The encrypted password, enclosed in a CCypherMessage*.
 */
CJacoDiagnosticDataManager::CJacoDiagnosticDataManager(Kinova::DLL::SafeGate::CCypherMessage* encPassword)
{
  create(encPassword);
}

/** Constructor, taking a plain password.
 * This constructor is added for convenience.
 * @param password The non-encrypted password in plain text.
 */
CJacoDiagnosticDataManager::CJacoDiagnosticDataManager(const char* password)
{
  Kinova::DLL::SafeGate::Crypto* c = Kinova::DLL::SafeGate::Crypto::GetInstance();
  Kinova::DLL::SafeGate::CCypherMessage* encPassword = c->Encrypt(password);
  create(encPassword);
}

/** Constructor, taking an object.
 * @param object The actual object.
 */
CJacoDiagnosticDataManager::CJacoDiagnosticDataManager(MyMonoObject* object)
{
  __object = object;
  if( !__object )
    throw KinovaException("Could not create CJacoDiagnosticDataManager, the MonoObject was NULL");
}

/** Destructor. */
CJacoDiagnosticDataManager::~CJacoDiagnosticDataManager()
{
}

/** Private method that creates the actual MonoObject for the Jaco-arm. */
void
CJacoDiagnosticDataManager::create(Kinova::DLL::SafeGate::CCypherMessage* encPassword)
{
  MonoObject* ex;
  void *args[1];
  args[0] = encPassword->get_object();
  mono_runtime_invoke(__m_ctor, __object, args, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);
}

/** Initialize the CJacoDiagnosticDataManager class and its methods. Needs to be done once.
 * @param image The image of the assembly.
 * @param class_namespace The namespace which the class belongs to. Usually the name of the assembly.
 * @return Possible error. (ERROR_NONE == 0)
 */
KinovaMonoError_t
CJacoDiagnosticDataManager::init(MonoImage* image, const char* class_namespace)
{
  // get mono class for this class
  __class = mono_class_from_name(image, "Kinova.API.Jaco.Diagnostic", "CJacoDiagnosticDataManager");
  if( !__class )
    return MONO_ERROR_CLASS;

  // get the methods of the class
  __m_ctor = mono_class_get_method_from_name(__class, ".ctor", 1 /*number of params*/);
  __m_GetPositionLogLiveFromJaco = mono_class_get_method_from_name(__class, "GetPositionLogLiveFromJaco", 0);
  if( !__m_ctor || !__m_GetPositionLogLiveFromJaco )
    return MONO_ERROR_METHOD;

  return MONO_ERROR_NONE;
}


Kinova::DLL::Data::CPosition*
CJacoDiagnosticDataManager::GetPositionLogLiveFromJaco()
{

  MonoObject* ex;
  MonoObject* object;
  object = mono_runtime_invoke(__m_GetPositionLogLiveFromJaco, __object, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);

  Kinova::DLL::Data::CPosition* pos = new Kinova::DLL::Data::CPosition();
  pos->set_object((MyMonoObject*)object);

  return pos;
}


//*
/* /================================\
 *         Class CJacoDiagnosticManager
 * \================================/*/
MonoMethod*  CJacoDiagnosticManager::__m_ctor = NULL;
MonoProperty* CJacoDiagnosticManager::__p_DataManager = NULL;

/** Constructor, taking a CCypherMessage (encrypted password).
 * This is the way the original API intends it to be used.
 * @param encPassword The encrypted password, enclosed in a CCypherMessage*.
 */
CJacoDiagnosticManager::CJacoDiagnosticManager(Kinova::DLL::SafeGate::CCypherMessage* encPassword)
{
  create(encPassword);
}

/** Constructor, taking a plain password.
 * This constructor is added for convenience.
 * @param password The non-encrypted password in plain text.
 */
CJacoDiagnosticManager::CJacoDiagnosticManager(const char* password)
{
  Kinova::DLL::SafeGate::Crypto* c = Kinova::DLL::SafeGate::Crypto::GetInstance();
  Kinova::DLL::SafeGate::CCypherMessage* encPassword = c->Encrypt(password);
  create(encPassword);
}

/** Constructor, taking an object.
 * @param object The actual object.
 */
CJacoDiagnosticManager::CJacoDiagnosticManager(MyMonoObject* object)
{
  __object = object;
  if( !__object )
    throw KinovaException("Could not create CJacoDiagnosticManager, the MonoObject was NULL");
}

/** Destructor. */
CJacoDiagnosticManager::~CJacoDiagnosticManager()
{
}

/** Private method that creates the actual MonoObject for the Jaco-arm. */
void
CJacoDiagnosticManager::create(Kinova::DLL::SafeGate::CCypherMessage* encPassword)
{
  MonoObject* ex;
  void *args[1];
  args[0] = encPassword->get_object();
  mono_runtime_invoke(__m_ctor, __object, args, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);
}

/** Initialize the CJacoDiagnosticManager class and its methods. Needs to be done once.
 * @param image The image of the assembly.
 * @param class_namespace The namespace which the class belongs to. Usually the name of the assembly.
 * @return Possible error. (ERROR_NONE == 0)
 */
KinovaMonoError_t
CJacoDiagnosticManager::init(MonoImage* image, const char* class_namespace)
{
  // get mono class for this class
  __class = mono_class_from_name(image, "Kinova.API.Jaco.Diagnostic", "CJacoDiagnosticManager");
  if( !__class )
    return MONO_ERROR_CLASS;

  // get the methods of the class
  __m_ctor = mono_class_get_method_from_name(__class, ".ctor", 1 /*number of params*/);
  if( !__m_ctor )
    return MONO_ERROR_METHOD;

  // get methods of the class' properties
  __p_DataManager = mono_class_get_property_from_name(__class, "DataManager");
  if( !__p_DataManager )
    return MONO_ERROR_PROPERTY;

  return MONO_ERROR_NONE;
}


CJacoDiagnosticDataManager*
CJacoDiagnosticManager::DataManager()
{
  MonoObject* ex;
  MonoObject* object;
  object = mono_property_get_value(__p_DataManager, __object, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);

  if( !__o_DataManager )
    __o_DataManager = new CJacoDiagnosticDataManager((MyMonoObject*)object);
  else
    __o_DataManager->set_object((MyMonoObject*)object);

  return __o_DataManager;
}


//*
/* /================================\
 *         Class CJacoArm
 * \================================/*/
MonoMethod*  CJacoArm::__m_ctor = NULL;
MonoMethod*  CJacoArm::__m_CloseConnection = NULL;
MonoMethod*  CJacoArm::__m_GetAPIVersion = NULL;
MonoMethod*  CJacoArm::__m_JacoIsReady = NULL;
MonoMethod*  CJacoArm::__m_Finalize = NULL;
MonoProperty*  CJacoArm::__p_DiagnosticManager = NULL;

void (*CJacoArm::__CloseConnection)(MyMonoObject*) = NULL;

/** Constructor, taking a CCypherMessage (encrypted password).
 * This is the way the original API intends it to be used.
 * @param encPassword The encrypted password, enclosed in a CCypherMessage*.
 */
CJacoArm::CJacoArm(Kinova::DLL::SafeGate::CCypherMessage* encPassword)
{
  create(encPassword);
}

/** Constructor, taking a plain password.
 * This constructor is added for convenience.
 * @param password The non-encrypted password in plain text.
 */
CJacoArm::CJacoArm(const char* password)
{
  Kinova::DLL::SafeGate::Crypto* c = Kinova::DLL::SafeGate::Crypto::GetInstance();
  Kinova::DLL::SafeGate::CCypherMessage* encPassword = c->Encrypt(password);
  create(encPassword);
}

/** Destructor. */
CJacoArm::~CJacoArm()
{
}

/** Private method that creates the actual MonoObject for the Jaco-arm. */
void
CJacoArm::create(Kinova::DLL::SafeGate::CCypherMessage* encPassword)
{
  MonoObject* ex;
  void *args[1];
  args[0] = encPassword->get_object();
  mono_runtime_invoke(__m_ctor, __object, args, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);
}

/** Initialize the CJacoArm class and its methods. Needs to be done once.
 * @param image The image of the assembly.
 * @param class_namespace The namespace which the class belongs to. Usually the name of the assembly.
 * @return Possible error. (ERROR_NONE == 0)
 */
KinovaMonoError_t
CJacoArm::init(MonoImage* image, const char* class_namespace)
{
  // get mono class for this class
  __class = mono_class_from_name(image, class_namespace, "CJacoArm");
  if( !__class )
    return MONO_ERROR_CLASS;

  // get the methods of the class
  __m_ctor = mono_class_get_method_from_name(__class, ".ctor", 1 /*number of params*/);
  __m_CloseConnection = mono_class_get_method_from_name(__class, "CloseConnection", 0);
  __m_GetAPIVersion = mono_class_get_method_from_name(__class, "GetAPIVersion", 0);
  __m_JacoIsReady = mono_class_get_method_from_name(__class, "JacoIsReady", 0);
  __m_Finalize = mono_class_get_method_from_name(__class, "Finalize", 0);
  if( !__m_ctor || !__m_CloseConnection || !__m_GetAPIVersion || !__m_Finalize)
    return MONO_ERROR_METHOD;

  // get methods of the class' properties
  __p_DiagnosticManager = mono_class_get_property_from_name(__class, "DiagnosticManager");
  if( !__p_DiagnosticManager )
    return MONO_ERROR_PROPERTY;

  // EXPERIMENTAL; get function pointers to access the MonoMethods. Is supposed to be faster,
  // but I need to figure out how to pass arguments and how to catch exceptions with this.
  __CloseConnection = (void (*)(MyMonoObject*))mono_method_get_unmanaged_thunk(__m_CloseConnection);
  if( !__CloseConnection )
    return MONO_ERROR_METHOD;

  return MONO_ERROR_NONE;
}

/** Closes the USB Connection. */
void
CJacoArm::CloseConnection()
{
  MonoObject* ex;
  //__CloseConnection(__object);
  mono_runtime_invoke(__m_CloseConnection, __object, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);
}

/** Gets the API version.
 * @return The API version as a string.
 */
std::string
CJacoArm::GetAPIVersion()
{
  MonoObject* ex;
  MonoObject* object;
  object = mono_runtime_invoke(__m_GetAPIVersion, __object, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);

  return std::string(mono_string_to_utf8(mono_object_to_string(object, NULL)));
}

/** Ask if Jaco is ready to communicate.
 * @return True, if Jaco-arm is connected and ready to communicate.
 */
bool
CJacoArm::JacoIsReady()
{
  MonoObject* ex;
  MonoObject* object;
  object = mono_runtime_invoke(__m_JacoIsReady, __object, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);

  // need to unbox the return-value of the invoke, to be able to interpret it.
  MonoBoolean* mb = (MonoBoolean*)mono_object_unbox(object);
  if(*mb)
    return true;
  else
    return false;
}


void
CJacoArm::Finalize()
{
  MonoObject* ex;
  mono_runtime_invoke(__m_Finalize, __object, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);
}

CJacoDiagnosticManager*
CJacoArm::DiagnosticManager()
{
  MonoObject* ex;
  MonoObject* object;
  object = mono_property_get_value(__p_DiagnosticManager, __object, NULL, &ex);
  if( ex )
    throw KinovaException((MyMonoObject*)ex);

  if( !__o_DiagnosticManager )
    __o_DiagnosticManager = new CJacoDiagnosticManager((MyMonoObject*)object);
  else
    __o_DiagnosticManager->set_object((MyMonoObject*)object);

  return __o_DiagnosticManager;
}

//*/

}}} // namespace Kinova::API::Jaco
