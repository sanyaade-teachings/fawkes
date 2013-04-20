
/***************************************************************************
 *  qa_kinova_api.cpp - QA for Kinova API library
 *
 *  Created: Thu Apr 18 15:35:12 2013
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

// Do not include in api reference
///@cond QA

#include <logging/console.h>
#include <cstdio>
#include <iostream>
#include <vector>

#include <kinova/kinova_api.h>

using namespace fawkes;
using namespace std;

// nice ugly namespaces, to represent the structure of the original API.
// Hopefully gone in future releases.
using namespace Kinova::API::Jaco;
using namespace Kinova::DLL::SafeGate;

int
main(int argc, char **argv)
{
  const char* name="QaKinovaApi";

  ConsoleLogger* cl = new ConsoleLogger();
  cl->log_debug(name, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

  cl->log_debug(name, "Loading the API...");
  const char* api_name = "jaco_test";
  const char* api_path = "/home/kb/kinova/tutorial/dll/";
  Kinova::JacoAPI* jaco_api = new Kinova::JacoAPI(api_name, api_path);
  cl->log_debug(name, "...DONE.");

  cl->log_debug(name, "Initializing the API...");
  int ret = jaco_api->init();
  if( ret ) {
    cl->log_error(name, "...ERROR! ret:%i", ret);
    return 0;
  }
  cl->log_debug(name, "...DONE.");


  /* Try creating a CJacoArm object, i.e. connecting to the arm.
   * See how this resembles the way it is meant for the C# API, so one can use the Kinova-Jaco Tutorials
   * easily with this CPP API. Still, there are some differences (pointers), that need to be considered.
   */
  try {
    const char* password = "a_valid_password";
    cl->log_debug(name, "Try creating a CJacoArm object. Password: \"%s\" ", password);
    CCypherMessage* encPassw = Crypto::GetInstance()->Encrypt(password);
    CJacoArm* arm = new CJacoArm(encPassw);

    //We added another constructor for convenience, so it is also possible to simply do:
    // CJacoArm* arm = new CJacoArm("a_valid_password");

    cl->log_debug(name, "CJacoArm object created!");

    cl->log_debug(name, "Check if arm is ready to communicate...");
    if( arm->JacoIsReady() ) {
      cl->log_debug(name, "... YES!");
    } else {
      cl->log_debug(name, "... NO!");
    }

    cl->log_debug(name, "Closing the connection to the arm");
    arm->CloseConnection();

  } catch(Kinova::KinovaException &e) {
    cl->log_error(name, "Kinova Ex: %s", e.what());
  } catch(Exception &e) {
    cl->log_error(name, "Other Ex: %s", e.what());
  }

  cl->log_debug(name, "----------------------------------------------------------------------------------");
  return 0;
}


/// @endcond
