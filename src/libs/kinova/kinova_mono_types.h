
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
#include <list>
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


class KinovaMonoAssembly;
typedef KinovaMonoError_t (*init_func)(KinovaMonoAssembly*);


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

  MonoDomain*   get_domain() const;
  MonoAssembly* get_assembly() const;
  MonoImage*    get_image() const;
  const char*   get_name() const;
  const char*   get_path() const;

 protected:
  MonoDomain*   __domain;    /**< The active mono domain */
  MonoAssembly* __assembly;  /**< The actual mono assembly */
  MonoImage*    __image;     /**< The mono image of the assembly */
  const char*   __name;      /**< Name of the assembly. Should be the DLL-filename without suffix. */
  std::list<init_func> __inits;  /**< List of the classes' static init functions. */

 private:
  std::string   __path;  /**< Path to the assembly file */
  bool          __initialized; /**< Tracking if assembly was initialized */
};



/** Representing a Class.
 * Each provided class should be a child of this one.
 *
 * Probably better if declared as a template. This is just a first attempt though.*/
template <typename T>
class KinovaMonoClass
{
 public:
  virtual ~KinovaMonoClass() {}

  static KinovaMonoError_t init(KinovaMonoAssembly* assembly);
  static KinovaMonoError_t init(MonoDomain* domain, MonoImage* image, const char* class_namespace);

  MyMonoObject* get_object();

 protected:
  static MonoDomain* __domain;  /**< The active mono domain */
  static MonoClass*  __class;   /**< The actual class. Remember to declare static for child class! */
  MyMonoObject*      __object;  /**< The actual instance in the mono domain */
};

// initialize static members
template <typename T> MonoDomain* KinovaMonoClass<T>::__domain( NULL );
template <typename T> MonoClass*  KinovaMonoClass<T>::__class( NULL );


/** Get the actual object in the mono-domain.
 * @return The actual object (MonoObject*) in the mono-domain.
 */
template <typename T>
MyMonoObject*
KinovaMonoClass<T>::get_object()
{
  return (MyMonoObject*)__object;
}

template <typename T>
KinovaMonoError_t
KinovaMonoClass<T>::init(KinovaMonoAssembly* assembly)
{
  return init(assembly->get_domain(),
              assembly->get_image(),
              assembly->get_name());
}

/** Initialize the class. Needs to be done once per original API class.
 * @param domain The active mono-domain.
 * @param assembly The assembly this class belongs to.
 * @param image The image of the assembly.
 * @param class_namespace The namespace which the class belongs to.
 * @return Possible error. (ERROR_NONE == 0)
 */
template <typename T>
KinovaMonoError_t
KinovaMonoClass<T>::init(MonoDomain* domain, MonoImage* image, const char* class_namespace)
{
  // set domain that we are working in
  __domain = domain;
  if( !__domain )
    return MONO_ERROR_DOMAIN;

  return T::init(image, class_namespace);
}


} // namespace Kinova
#endif
