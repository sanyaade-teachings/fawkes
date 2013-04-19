
/***************************************************************************
 *  kinova_api_jaco.h - C# to C++ wrapper for assembly Kinova.API.Jaco
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

#ifndef KINOVA_API_JACO_H
#define KINOVA_API_JACO_H

#include "kinova_mono_types.h"

#include <cstdio>
#include <string>

/* Ugly namespacing, but trying to resemble original C# API as much as possible.
 * This is too ugly though, and will most probably be removed/shortened in future versions.
 */
namespace Kinova { namespace DLL { namespace SafeGate {
  class CCypherMessage;
}}}

namespace Kinova { namespace API { namespace Jaco
{

/** The class representing this assembly. Note that it is just called "Assembly".
 * This is ok for now, as all assemblies define different namespaces (see comment
 * above on the namespace).
 * For now, this makes it easier to add new assemblies (basically copy-and-paste of this
 * declaration in the header)
 */
class Assembly : public KinovaMonoAssembly
{
 public:
  /// \brief Constructor.
  Assembly(const char* path="");

  /// \brief Desctructor.
  virtual ~Assembly();

 private:
  KinovaMonoError_t init_classes();
};



/** The CJacoArm class. See Kinova-API for further information.
 * This is the main class for working with the JacoArm.
 *
 * Again, we could save ourselves some code here by using class templates
 * for KinovaMonoClass. Will be done in futer versions.
 */
class CJacoArm : public KinovaMonoClass<CJacoArm>
{
 public:
  /// \brief Constructor, taking a CCypherMessage (encrypted password).
  CJacoArm(Kinova::DLL::SafeGate::CCypherMessage* encPassword);
  /// \brief Constructor, taking a plain password.
  CJacoArm(const char* password);

  /// \brief Destructor.
  ~CJacoArm();

  /// \brief Initializes connection to MonoMethods
  //static KinovaMonoError_t init(MonoDomain* domain, MonoAssembly* assembly, MonoImage* image);
  static KinovaMonoError_t init(MonoImage* image, const char* class_namespace);

  /// \brief Closes the USB Connection.
  void CloseConnection();

  /// \brief Gets the API version.
  std::string GetAPIVersion();

  /// \brief Ask if Jaco is ready to communicate.
  bool JacoIsReady();

 protected:
  /** Releases unmanaged resources and performs other cleanup operations
   *  before the CJacoArm is reclaimed by garbage collection. */
  virtual void Finalize();

 private:
  void create(Kinova::DLL::SafeGate::CCypherMessage* encPassword);
  static MonoMethod* __m_ctor;
  static MonoMethod* __m_CloseConnection;
  static MonoMethod* __m_GetAPIVersion;
  static MonoMethod* __m_JacoIsReady;
  static MonoMethod* __m_Finalize;

  static void (*__CloseConnection)(MyMonoObject* object);
  //CJacoConfigurationManager ConfigurationsManager;
  //CJacoControlManager       ControlManager;
  //CJacoDiagnosticManager    DiagnosticManager;

}; // class CJacoArm

}}} // namespace Kinova::API::Jaco
#endif
