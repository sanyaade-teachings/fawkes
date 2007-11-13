
/***************************************************************************
 *  qa_bb_notify.cpp - BlackBoard notification QA
 *
 *  Created: Mon Nov 12 14:35:53 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */


/// @cond QA

#include <blackboard/blackboard.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>
#include <blackboard/event_listener.h>

#include <interfaces/test.h>

#include <core/exceptions/system.h>
#include <utils/logging/liblogger.h>

#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <vector>

using namespace std;

class QaBBEventListener : public BlackBoardEventListener
{
 public:
  QaBBEventListener()
  {
    bbel_add_interface_create_type("TestInterface");
  }

  virtual void bb_data_changed(Interface *interface) throw()
  {
    printf("BBEL: Data in interface %s has been modified\n", interface->uid());
  }

  virtual void bb_interface_created(const char *type, const char *id) throw()
  {
    printf("BBEL: Interface %s of type %s has been created\n", id, type);
  }

  virtual void bb_interface_writer_added(Interface *interface) throw()
  {
    printf("BBEL: Writer has been added to interface %s\n", interface->uid());
  }

  virtual void bb_interface_writer_removed(Interface *interface) throw()
  {
    printf("BBEL: Writer has been removed from interface %s\n", interface->uid());
  }

  virtual void bb_interface_reader_added(Interface *interface) throw()
  {
    printf("BBEL: Reader has been added to interface %s\n", interface->uid());
  }

  virtual void bb_interface_reader_removed(Interface *interface) throw()
  {
    printf("BBEL: Reader has been removed from interface %s\n", interface->uid());
  }

  virtual void add_interface(Interface *interface) throw()
  {
    printf("BBEL: Adding interface %s\n", interface->uid());
    bbel_add_data_interface(interface);
    bbel_add_reader_interface(interface);
    bbel_add_writer_interface(interface);
  }
};


int
main(int argc, char **argv)
{
  LibLogger::init();

  BlackBoard *bb = new BlackBoard();
  BlackBoardInterfaceManager *im = bb->interface_manager();

  QaBBEventListener qabbel;

  TestInterface *ti_writer_1;
  TestInterface *ti_writer_2;
  TestInterface *ti_writer_3;
  TestInterface *ti_writer_4;
  TestInterface *ti_writer_5;
  TestInterface *ti_writer_6;

  try {
    cout << "Opening interfaces.. " << flush;
    ti_writer_1 = im->open_for_writing<TestInterface>("SomeID 1");
    ti_writer_2 = im->open_for_writing<TestInterface>("SomeID 2");

    qabbel.add_interface(ti_writer_1);
    qabbel.add_interface(ti_writer_2);
    im->register_listener(&qabbel, BlackBoardInterfaceManager::BBEL_FLAG_ALL);

    ti_writer_3 = im->open_for_writing<TestInterface>("SomeID 3");
    ti_writer_4 = im->open_for_writing<TestInterface>("AnotherID 1");
    ti_writer_5 = im->open_for_writing<TestInterface>("AnotherID 2");
    ti_writer_6 = im->open_for_writing<TestInterface>("AnotherID 3");
    cout << "success" << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.print_trace();
    exit(1);
  }

  std::list<TestInterface *> *readers = im->open_all_of_type_for_reading<TestInterface>();
  for (std::list<TestInterface *>::iterator i = readers->begin(); i != readers->end(); ++i) {
    printf("Opened reader for interface %s of type %s\n", (*i)->id(), (*i)->type());
    im->close(*i);
  }
  delete readers;

  const char* prefix = "Another";
  readers = im->open_all_of_type_for_reading<TestInterface>(prefix);
#if __WORDSIZE == 64
  printf("Found %lu interfaces with prefix \"%s\"\n", readers->size(), prefix);
#else
  printf("Found %u interfaces with prefix \"%s\"\n", readers->size(), prefix);
#endif
  for (std::list<TestInterface *>::iterator i = readers->begin(); i != readers->end(); ++i) {
    printf("Opened reader for interface %s of type %s\n", (*i)->id(), (*i)->type());
    im->close(*i);
  }
  delete readers;

  printf("Removing writer one. This should print a warning.\n");
  im->close(ti_writer_1);
  im->unregister_listener(&qabbel);
  printf("Removing other writers. No warning should appear.\n");
  im->close(ti_writer_2);
  im->close(ti_writer_3);
  im->close(ti_writer_4);
  im->close(ti_writer_5);
  im->close(ti_writer_6);

  delete bb;
  LibLogger::finalize();
}


/// @endcond