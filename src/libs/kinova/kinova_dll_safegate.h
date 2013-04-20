
/***************************************************************************
 *  kinova_dll_safegate.h - C# to C++ wrapper for assembly Kinova.DLL.SafeGate
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

#ifndef KINOVA_DLL_SAFEGATE_H_
#define KINOVA_DLL_SAFEGATE_H_

#include "kinova_mono_types.h"

#include <cstdio>

/* Ugly namespacing, but trying to resemble original C# API as much as possible.
 * This is too ugly though, and will most probably be removed/shortened in future versions.
 */
namespace Kinova { namespace DLL { namespace SafeGate
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
  Assembly(const char* path="");
  virtual ~Assembly();
};



/** The CCypherMessage class. See Kinova-API for further information.
 * This is needed to properly create a CJacoArm object.
 *
 * Again, we could save ourselves some code here by using class templates
 * for KinovaMonoClass. Will be done in futer versions.
 */
class CCypherMessage : public KinovaMonoClass<CCypherMessage>
{
 public:
  CCypherMessage();
  CCypherMessage(MyMonoObject* object);
  ~CCypherMessage();

  static KinovaMonoError_t init(MonoImage* image, const char* class_namespace);
}; // class CCypherMessage



/** The Crypto class. See Kinova-API for further information.
 * This is needed to properly create a CCypherMessage with an encrypted password.
 *
 * This is declared as a singleton, therefor no public constructors/destructors. We
 * just provide public method, the rest is handled internally.
 *
 * Again, we could save ourselves some code here by using class templates
 * for KinovaMonoClass. Will be done in futer versions.
 */
class Crypto : public KinovaMonoClass<Crypto>
{
 public:
  static Crypto* GetInstance();
  CCypherMessage* Encrypt(const char* password);

  static KinovaMonoError_t init(MonoImage* image, const char* class_namespace);
 private:
  Crypto();
  //~Crypto();
  static Crypto*     __instance;
  static MonoMethod* __m_GetInstance;
  static MonoMethod* __m_Encrypt;
}; // class Crypto

}}} // namespace Kinova::DLL::SafeGate
#endif
