
/***************************************************************************
 *  kinova_mono_types.h
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

#ifndef KINOVA_MONO_TYPES_H
#define KINOVA_MONO_TYPES_H

#include <string>
#include <exception>

// forward declarations. This way the user does not need to care about
// mono-2 libs/cflags
typedef struct _MonoDomain MonoDomain;
typedef struct _MonoAssembly MonoAssembly;
typedef struct _MonoImage MonoImage;
typedef struct _MonoClass MonoClass;
typedef struct _MonoMethod MonoMethod;
//MonoObject is an unnamed struct -_- see declaration below in Kinova namespace

namespace Kinova
{

/*As MonoObject is an unnamed struct, this is the only solution of forward declaration.
 * The cpp file then defines 'struct MyMonoObject : public MonoObject {};'
 * This requires some casting here and there, but it is for the sake of the user, who
 * does not need to worry about additional CLFAGS/LDFLAGS for the compiler, which would
 * be the case if we included <mono/metadata/object.h> here.
 */
/** A MonoObject wrapper, to allow forward declaration of an unnamed struct.*/
struct MyMonoObject;

/** Error types, can give a hint on where to look for the problem. */
typedef enum
{
  MONO_ERROR_NONE        = 0,
  MONO_ERROR_DOMAIN      = 1,
  MONO_ERROR_ASSEMBLY    = 2,
  MONO_ERROR_IMAGE       = 3,
  MONO_ERROR_CLASS       = 4,
  MONO_ERROR_METHOD      = 5,
  MONO_ERROR_OBJECT      = 6
} KinovaMonoError_t;


/** Default exception for Kinova API. We cannot distinguish the extensions
 * caught by the mono-framework yet. */
class KinovaException : public std::exception
{
 public:
  KinovaException(const char* msg = NULL) throw();
  KinovaException(MyMonoObject* object) throw();

  virtual const char* what() const throw();

 private:
  const char* __msg;
};


/** Representing a MonoAssembly */
class KinovaMonoAssembly
{
 public:
  KinovaMonoAssembly(const char* name, const char* path="");
  virtual ~KinovaMonoAssembly();

  virtual KinovaMonoError_t init(MonoDomain* d);

 protected:
  virtual KinovaMonoError_t init_classes();
  MonoDomain*   __domain;    /**< The active mono domain */
  MonoAssembly* __assembly;  /**< The actual mono assembly */
  MonoImage*    __image;     /**< The mono image of the assembly */

 private:
  const char*   __name;  /**< Name of the assembly. Should be the DLL-filename without suffix. */
  std::string   __path;  /**< Path to the assembly file */

  bool          __initialized; /**< Tracking if assembly was initialized */
};


/** Representing a Class.
 * Each provided class should be a child of this one.
 *
 * Probably better if declared as a template. This is just a first attempt though.*/
class KinovaMonoClass
{
 public:
  virtual ~KinovaMonoClass() {}

  /** Initialize the class. Needs to be done once per original API class.
   * @param domain The active mono-domain.
   * @param assembly The assembly this class belongs to.
   * @param image The image of the assembly.
   */
  static void init(MonoDomain* domain, MonoAssembly* assembly, MonoImage* image);

  MyMonoObject* get_object();

 protected:
  static MonoDomain* __domain;  /**< The active mono domain */
  static MonoClass*  __class;   /**< The actual class. Remember to declare static for child class! */
  MyMonoObject*      __object;  /**< The actual instance in the mono domain */
};


} // namespace Kinova
#endif
