
/***************************************************************************
 *  exec_thread.cpp - Fawkes Skiller: Execution Thread
 *
 *  Created: Mon Feb 18 10:30:17 2008
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "exec_thread.h"

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <utils/logging/component.h>
#ifdef SKILLER_TIMETRACKING
#  include <utils/time/tracker.h>
#endif

#include <lua/context.h>
#include <lua/interface_importer.h>

#include <interfaces/SkillerInterface.h>
#include <interfaces/SkillerDebugInterface.h>

#include <lua.hpp>
#include <tolua++.h>

#include <string>
#include <cstring>

using namespace std;
using namespace fawkes;

/** @class SkillerExecutionThread "exec_thread.h"
 * Skiller Execution Thread.
 * This thread runs and controls the Lua interpreter and passes data into the
 * execution engine.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
SkillerExecutionThread::SkillerExecutionThread()
  : Thread("SkillerExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL),
    BlackBoardInterfaceListener("SkillerExecutionThread")
{
  __error_written    = false;

  __lua = NULL;
}


/** Destructor. */
SkillerExecutionThread::~SkillerExecutionThread()
{
}

/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
SkillerExecutionThread::init_failure_cleanup()
{
  try {
    if ( __skiller_if ) blackboard->close(__skiller_if);
    if ( __skdbg_if )   blackboard->close(__skdbg_if);

    delete __lua_ifi;
    delete __clog;

  } catch (...) {
    // we really screwed up, can't do anything about it, ignore error, logger is
    // initialized since this method is only called from init() which is only called if
    // all aspects had been initialized successfully
    logger->log_error(name(), "Really screwed up while finalizing, aborting cleanup. "
		              "Fawkes is no longer in a clean state. Restart!");
  }
}


void
SkillerExecutionThread::init()
{
  __last_exclusive_controller = 0;
  __reader_just_left = false;
  __skdbg_what = "ACTIVE";
  __skdbg_graphdir = "TB";
  __skdbg_graphcolored = true;
  __clog = NULL;
  __sksf_pushed = false;

  try {
    __cfg_skillspace  = config->get_string("/skiller/skillspace");
    __cfg_watch_files = config->get_bool("/skiller/watch_files");
  } catch (Exception &e) {
    e.append("Insufficient configuration for Skiller");
    throw;
  }

  logger->log_debug("SkillerExecutionThread", "Skill space: %s", __cfg_skillspace.c_str());

  __clog = new ComponentLogger(logger, "SkillerLua");

  __lua = NULL;
  __lua_ifi = NULL;
  __skiller_if = NULL;
  __skdbg_if = NULL;

  std::string reading_prefix = "/skiller/interfaces/" + __cfg_skillspace + "/reading/";
  std::string writing_prefix = "/skiller/interfaces/" + __cfg_skillspace + "/writing/";

  try {
    __skiller_if = blackboard->open_for_writing<SkillerInterface>("Skiller");
    __skdbg_if   = blackboard->open_for_writing<SkillerDebugInterface>("Skiller");
    
    __lua  = new LuaContext(__cfg_watch_files);

    __lua_ifi = new LuaInterfaceImporter(__lua, blackboard, config, logger);
    __lua_ifi->open_reading_interfaces(reading_prefix);
    __lua_ifi->open_writing_interfaces(writing_prefix);
    __lua_ifi->add_interface("skdbg", __skdbg_if);
    __lua_ifi->add_interface("skiller", __skiller_if);

    __lua->add_package_dir(LUADIR);
    __lua->add_cpackage_dir(LUALIBDIR);

    __lua->add_package("fawkesutils");
    __lua->add_package("fawkesconfig");
    __lua->add_package("fawkesinterface");
    __lua->add_package("fawkesgeometry");

    __lua->set_string("SKILLSPACE", __cfg_skillspace.c_str());
    __lua->set_usertype("config", config, "Configuration", "fawkes");
    __lua->set_usertype("logger", __clog, "ComponentLogger", "fawkes");
    __lua->set_usertype("clock", clock, "Clock", "fawkes");

    __lua_ifi->push_interfaces();

    __lua->set_start_script(LUADIR"/skiller/start.lua");
  
    __skiller_if->set_skill_string("");
    __skiller_if->set_status(0, SkillerInterface::S_INACTIVE);
    __skiller_if->write();

    __skdbg_if->set_graph("");
    __skdbg_if->set_graph_fsm("ACTIVE");

  } catch (Exception &e) {
    init_failure_cleanup();
    throw;
  }

  // We want to know if our reader leaves and closes the interface
  bbil_add_reader_interface(__skiller_if);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_READER);

#ifdef SKILLER_TIMETRACKING
  __tt           = new TimeTracker();
  __ttc_total    = __tt->add_class("Total");
  __ttc_msgproc  = __tt->add_class("Message Processing");
  __ttc_luaprep  = __tt->add_class("Lua Preparation");
  __ttc_luaexec  = __tt->add_class("Lua Execution");
  __ttc_publish  = __tt->add_class("Publishing");
  __tt_loopcount = 0;
#endif
}


void
SkillerExecutionThread::finalize()
{
#ifdef SKILLER_TIMETRACKING
  delete __tt;
#endif
  delete __lua_ifi;

  blackboard->unregister_listener(this);
  blackboard->close(__skiller_if);
  blackboard->close(__skdbg_if);

  delete __lua;
  delete __clog;
}


void
SkillerExecutionThread::bb_interface_reader_removed(Interface *interface,
						  unsigned int instance_serial) throw()
{
  if ( instance_serial == __skiller_if->exclusive_controller() ) {
    logger->log_debug("SkillerExecutionThread", "Controlling interface instance was closed, "
		      "revoking exclusive control");

    __last_exclusive_controller = instance_serial;
    __reader_just_left = true;

    __skiller_if->set_exclusive_controller(0);
    __skiller_if->write();
  }
}



/** Determines the skill status and writes it to the BB.
 * This method assumes that it is called from within loop() and lua_mutex
 * is locked.
 */
void
SkillerExecutionThread::publish_skill_status()
{
  bool write = false;

  try {
    __lua->get_global("skillexec");
    __lua->get_field(-1, "status");
    __lua->pcall(0, __skiller_if->maxlenof_status());

    for (unsigned int i = __skiller_if->maxlenof_status(); i >= 1; --i) {
      SkillerInterface::SkillStatusEnum new_status =
	(SkillerInterface::SkillStatusEnum)__lua->to_integer(-1);
      __lua->pop(1);
      if (__skiller_if->status(i-1) != new_status) {
	__skiller_if->set_status(i-1, new_status);
	write = true;
      }
    }

    if (write) {
      __lua->get_global("skillexec");
      __lua->get_field(-1, "skillstring");
      __lua->pcall(0, 1);

      __skiller_if->set_skill_string(__lua->to_string(-1));
      __lua->pop(1);
    }

  } catch (Exception &e) {
    logger->log_error("SkillerExecutionThread", "Failed to retrieve skill status");
    logger->log_error("SkillerExecutionThread", e);
    try {
      for (unsigned int i = 0; i < __skiller_if->maxlenof_status(); ++i) {
	__skiller_if->set_status(i, SkillerInterface::S_FAILED);
      }
    } catch (Exception &e2) {
      logger->log_error("SkillerExecutionThread", "Failed to set FAILED as skill "
			"status value during error handling");
      logger->log_error("SkillerExecutionThread", e2);
    }
  }

  if (write) {
    __skiller_if->write();
  }
}


void
SkillerExecutionThread::publish_skdbg()
{
  /*
  try {
    __lua->do_string("skillenv.write_skiller_debug(interfaces.writing.skdbg, \"%s\", \"%s\", %s)",
		     __skdbg_what.c_str(), __skdbg_graphdir.c_str(),
		     __skdbg_graphcolored ? "true" : "false");
  } catch (Exception &e) {
    logger->log_warn("SkillerExecutionThread", "Error writing graph");
    logger->log_warn("SkillerExecutionThread", e);
  }
  */
}

void
SkillerExecutionThread::lua_loop_reset()
{
  try {
    __lua->do_string("skillenv.reset_loop()");
  } catch (Exception &e) {
    logger->log_warn("SkillerExecutionThread", "Lua Loop Reset failed");
    logger->log_warn("SkillerExecutionThread", e);
  }
}


void
SkillerExecutionThread::publish_error()
{
  /*
  try {
    __lua->do_string("skillenv.write_fsm_error(skillenv.get_skill_fsm(skillenv.get_active_skills()), interfaces.writing.skiller)");
  } catch (Exception &e) {
    logger->log_warn("SkillerExecutionThread", "Error writing error");
    logger->log_warn("SkillerExecutionThread", e);
    __skiller_if->set_error("Failed to set Lua error");
    __skiller_if->write();
  }
  */
}


void
SkillerExecutionThread::process_skdbg_messages()
{
  while ( ! __skdbg_if->msgq_empty() ) {
    if ( __skdbg_if->msgq_first_is<SkillerDebugInterface::SetGraphMessage>() ) {
      SkillerDebugInterface::SetGraphMessage *m = __skdbg_if->msgq_first<SkillerDebugInterface::SetGraphMessage>();
      logger->log_warn(name(), "Setting skiller debug what to: %s", m->graph_fsm());
      __skdbg_what = m->graph_fsm();
    } else if (__skdbg_if->msgq_first_is<SkillerDebugInterface::SetGraphDirectionMessage>() ) {
      SkillerDebugInterface::SetGraphDirectionMessage *m = __skdbg_if->msgq_first<SkillerDebugInterface::SetGraphDirectionMessage>();
      switch (m->graph_dir()) {
      case SkillerDebugInterface::GD_BOTTOM_TOP:  __skdbg_graphdir = "BT"; break;
      case SkillerDebugInterface::GD_LEFT_RIGHT:  __skdbg_graphdir = "LR"; break;
      case SkillerDebugInterface::GD_RIGHT_LEFT:  __skdbg_graphdir = "RL"; break;
      default:                                    __skdbg_graphdir = "TB"; break;
      }

    } else if (__skdbg_if->msgq_first_is<SkillerDebugInterface::SetGraphColoredMessage>() ) {
      SkillerDebugInterface::SetGraphColoredMessage *m = __skdbg_if->msgq_first<SkillerDebugInterface::SetGraphColoredMessage>();
      __skdbg_graphcolored = m->is_graph_colored();
    }

    __skdbg_if->msgq_pop();
  }
}


void
SkillerExecutionThread::loop()
{
#ifdef SKILLER_TIMETRACKING
  __tt->ping_start(__ttc_total);
#endif
#ifdef HAVE_INOTIFY
  __lua->process_fam_events();
#endif
  __lua_ifi->read();

  unsigned int excl_ctrl   = __skiller_if->exclusive_controller();
  bool write_skiller_if    = false;

#ifdef SKILLER_TIMETRACKING
  __tt->ping_start(__ttc_msgproc);
#endif
  process_skdbg_messages();

  while ( ! __skiller_if->msgq_empty() ) {
    if ( __skiller_if->msgq_first_is<SkillerInterface::AcquireControlMessage>() ) {
      Message *m = __skiller_if->msgq_first();
      if ( excl_ctrl == 0 ) {
	logger->log_debug("SkillerExecutionThread", "%s is new exclusive controller",
			  m->sender_thread_name());
	__skiller_if->set_exclusive_controller(m->sender_id());
	write_skiller_if = true;
	excl_ctrl = m->sender_id();
      } else {
	logger->log_warn("SkillerExecutionThread", "%s tried to acquire exclusive control, "
			 "but another controller exists already", m->sender_thread_name());
      }

    } else if ( __skiller_if->msgq_first_is<SkillerInterface::ReleaseControlMessage>() ) {
      Message *m = __skiller_if->msgq_first();
      if ( excl_ctrl == m->sender_id() ) {
	logger->log_debug("SkillerExecutionThread", "%s releases exclusive control",
			  m->sender_thread_name());
	
	__last_exclusive_controller = __skiller_if->exclusive_controller();
	__skiller_if->set_exclusive_controller(0);
	write_skiller_if = true;
	excl_ctrl = 0;
    } else {
	if ( !__reader_just_left || (m->sender_id() != __last_exclusive_controller)) {
	  logger->log_warn("SkillerExecutionThread", "%s tried to release exclusive control, "
			   "it's not the controller", m->sender_thread_name());
	}
      }

    } else if ( __skiller_if->msgq_first_is<SkillerInterface::ExecSkillMessage>() ) {
      SkillerInterface::ExecSkillMessage *m = __skiller_if->msgq_first<SkillerInterface::ExecSkillMessage>();

      if ( m->sender_id() == excl_ctrl ) {
	logger->log_debug(name(), "%s wants me to continuously execute '%s'",
			  m->sender_thread_name(), m->skill_string());

	try {
	  __lua->get_global("skillexec");
	  __lua->get_field(-1, "exec_skill");
	  __lua->push_string(m->skill_string());
	  __lua->pcall(1, 1);
	  unsigned int channel = __lua->to_integer(-1) - 1; // Lua is 1-based
	  __lua->pop(1);
	  __skiller_if->set_msgid(channel, m->id());
	  write_skiller_if = true;
	} catch (Exception &e) {
	  logger->log_error(name(), "Failed to start execution of '%s', "
			    "exception follows", m->skill_string());
	  logger->log_error(name(), e);
	}
      } else {
	logger->log_debug(name(), "%s tries to exec while not controller",
			  m->sender_thread_name());
      }

    } else if ( __skiller_if->msgq_first_is<SkillerInterface::StopExecMessage>() ) {
      SkillerInterface::StopExecMessage *m = __skiller_if->msgq_first<SkillerInterface::StopExecMessage>();

      if ( (m->sender_id() == excl_ctrl) ||
	   (__reader_just_left && (m->sender_id() == __last_exclusive_controller)) ) {

	logger->log_debug(name(), "Stopping execution of channel %u",
			  m->channel());	

	try {
	  __lua->get_global("skillexec");
	  __lua->get_field(-1, "stop_skill");
	  __lua->push_integer(m->channel());
	  __lua->pcall(1);
	  __skiller_if->set_msgid(m->channel(), 0);
	  //__skiller_if->set_status(m->channel(), SkillerInterface::S_INACTIVE);
	  write_skiller_if = true;
	} catch (Exception &e) {
	  logger->log_error(name(), "Failed to stop execution of channel %u",
			    m->channel());
	  logger->log_error(name(), e);
	}

      } else {
	logger->log_debug(name(), "%s tries to stop exec while not controller",
			  m->sender_thread_name());
      }


    } else if ( __skiller_if->msgq_first_is<SkillerInterface::StopAllMessage>() ) {
      SkillerInterface::StopAllMessage *m = __skiller_if->msgq_first<SkillerInterface::StopAllMessage>();

      if ( (m->sender_id() == excl_ctrl) ||
	   (__reader_just_left && (m->sender_id() == __last_exclusive_controller)) ) {

	logger->log_debug(name(), "Stopping execution of all channels");

	try {
	  __lua->get_global("skillexec");
	  __lua->get_field(-1, "stop_skills");
	  __lua->pcall();
	  for (unsigned int i = 0; i < __skiller_if->maxlenof_status(); ++i) {
	    __skiller_if->set_msgid(i, 0);
	    //__skiller_if->set_status(i, SkillerInterface::S_INACTIVE);
	  }
	  write_skiller_if = true;
	} catch (Exception &e) {
	  logger->log_error(name(), "Failed to stop execution of all channels");
	  logger->log_error(name(), e);
	}

      } else {
	logger->log_debug(name(), "%s tries to stop all while not controller",
			  m->sender_thread_name());
      }


    } else {
      logger->log_warn(name(), "Unhandled message of type %s in "
		       "skiller interface", __skiller_if->msgq_first()->type());
    }

    __skiller_if->msgq_pop();
  }

#ifdef SKILLER_TIMETRACKING
  __tt->ping_end(__ttc_msgproc);
#endif

  /*
  if ( __continuous_reset ) {
    logger->log_debug("SkillerExecutionThread", "Continuous reset forced");    try {
      if (__sksf_pushed) {
	__sksf_pushed = false;
	__lua->pop(1);			  // ---
      }
      __lua->do_string("skillenv.reset_all()");
    } catch (Exception &e) {
      logger->log_warn("SkillerExecutionThread", "Caught exception while resetting skills, ignored, output follows");
      logger->log_warn("SkillerExecutionThread", e);
    }

    __skiller_if->set_status(0, SkillerInterface::S_INACTIVE);
    __skiller_if->set_skill_string("");

    //We're not resetting, because this is information someone might need...
    //__skiller_if->set_error("");
    __error_written    = false;
    __continuous_reset = false;
    write_skiller_if   = true;
  }
  */

  if ( write_skiller_if )  __skiller_if->write();

  try {
    __lua->get_global("skillexec");
    __lua->get_field(-1, "loop");
    __lua->pcall();
  } catch (Exception &e) {
    logger->log_error(name(), "Skill loop execution failed, exception follows");
    logger->log_error(name(), e);
  }

#ifdef SKILLER_TIMETRACKING
    __tt->ping_start(__ttc_publish);
#endif
  publish_skill_status();
  publish_skdbg();
  lua_loop_reset();

  __reader_just_left = false;

  __lua_ifi->write();
#ifdef SKILLER_TIMETRACKING
  __tt->ping_end(__ttc_publish);
  __tt->ping_end(__ttc_total);
  if (++__tt_loopcount >= SKILLER_TT_MOD) {
    //logger->log_debug("Lua", "Stack size: %i", __lua->stack_size());
    __tt_loopcount = 0;
    __tt->print_to_stdout();
  }
#endif
}
