
/***************************************************************************
 *  kinova_dll_data.h - C# to C++ wrapper for assembly Kinova.DLL.Data
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

#ifndef KINOVA_DLL_DATA_H_
#define KINOVA_DLL_DATA_H_

#include "kinova_mono_types.h"

#include <cstdio>

/* Ugly namespacing, but trying to resemble original C# API as much as possible.
 * This is too ugly though, and will most probably be removed/shortened in future versions.
 */
namespace Kinova { namespace DLL { namespace Data
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

//*
/* /================================\
 *     Namespace Jaco.Diagnostic
 * \================================/*/
/** CPosition class. */
class CPosition : public KinovaMonoClass<CPosition>
{
 public:
  CPosition();
  ~CPosition();

  /// \brief Initializes connection to MonoMethods
  static KinovaMonoError_t init(MonoImage* image, const char* class_namespace);

  /// \brief All Initialization is done here.
  void Init();

 private:
  static MonoMethod* __m_Init;
};


//*
/* /================================\
 *     Namespace Util
 * \================================/*/

class CVectorEuler : public KinovaMonoClass<CVectorEuler>
{
 public:
  CVectorEuler();
  ~CVectorEuler();

  static KinovaMonoError_t init(MonoImage* image, const char* class_namespace);

  void Init();

  //properties in C#
  float* Position();
  void Position(float value[]);

 private:
  static MonoMethod* __m_Init;
  static MonoProperty* __p_Position;
  static MonoProperty* __p_Rotation;

};


}}} // namespace Kinova::DLL::Data
#endif
