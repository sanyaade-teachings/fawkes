
/***************************************************************************
 *  clips_inifin.cpp - Fawkes CLIPSAspect initializer/finalizer
 *
 *  Created: Sat Jun 16 14:34:27 2012
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include <plugins/clips/aspect/clips_inifin.h>
#include <core/threading/thread_finalizer.h>
#include <logging/logger.h>
#include <clipsmm.h>

extern "C" {
#include <clips/clips.h>
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond INTERNALS

class CLIPSLogger
{
 public:
  CLIPSLogger(Logger *logger, const char *component = NULL)
  {
    logger_ = logger;
    if (component) {
      component_ = strdup(component);
    } else {
      component_ = NULL;
    }
  }

  ~CLIPSLogger()
  {
    if (component_) {
      free(component_);
    }
  }

  void log(const char *logical_name, const char *str)
  {
    if (strcmp(str, "\n") == 0) {
      if (strcmp(logical_name, "debug") == 0 || strcmp(logical_name, "logdebug") == 0 ||
	  strcmp(logical_name, WTRACE) == 0)
      {
	logger_->log_debug(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      } else if (strcmp(logical_name, "warn") == 0 || strcmp(logical_name, "logwarn") == 0 ||
		 strcmp(logical_name, WWARNING) == 0)
      {
	logger_->log_warn(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      } else if (strcmp(logical_name, "error") == 0 || strcmp(logical_name, "logerror") == 0 ||
		 strcmp(logical_name, WERROR) == 0)
      {
	logger_->log_error(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      } else {
	logger_->log_info(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      }

      buffer_.clear();
    } else {
      buffer_ += str;
    }
  }

 private:
  Logger *logger_;
  char *component_;
  std::string buffer_;
};

class CLIPSContextMaintainer {
 public:
  CLIPSContextMaintainer(Logger *logger, const char *log_component_name)
  {
    this->logger = new CLIPSLogger(logger, log_component_name);
  }

  ~CLIPSContextMaintainer()
  {
    delete logger;
  }

 public:
  CLIPSLogger *logger;
};


static int
log_router_query(void *env, char *logical_name)
{
  if (strcmp(logical_name, "l") == 0) return TRUE;
  if (strcmp(logical_name, "info") == 0) return TRUE;
  if (strcmp(logical_name, "debug") == 0) return TRUE;
  if (strcmp(logical_name, "warn") == 0) return TRUE;
  if (strcmp(logical_name, "error") == 0) return TRUE;
  if (strcmp(logical_name, "loginfo") == 0) return TRUE;
  if (strcmp(logical_name, "logdebug") == 0) return TRUE;
  if (strcmp(logical_name, "logwarn") == 0) return TRUE;
  if (strcmp(logical_name, "logerror") == 0) return TRUE;
  if (strcmp(logical_name, "stdout") == 0) return TRUE;
  if (strcmp(logical_name, WTRACE) == 0) return TRUE;
  if (strcmp(logical_name, WWARNING) == 0) return TRUE;
  if (strcmp(logical_name, WERROR) == 0) return TRUE;
  if (strcmp(logical_name, WDISPLAY) == 0) return TRUE;
  return FALSE;
}

static int
log_router_print(void *env, char *logical_name, char *str)
{
  void *rc = GetEnvironmentRouterContext(env);
  CLIPSLogger *logger = static_cast<CLIPSLogger *>(rc);
  logger->log(logical_name, str);
  return TRUE;
}

static int
log_router_exit(void *env, int exit_code)
{
  return TRUE;
}

/// @endcond

/** @class CLIPSAspectIniFin <plugins/clips/aspect/clips_inifin.h>
 * CLIPSAspect initializer/finalizer.
 * This initializer/finalizer will provide the CLIPS node handle to
 * threads with the CLIPSAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
CLIPSAspectIniFin::CLIPSAspectIniFin()
  : AspectIniFin("CLIPSAspect")
{
  logger_ = NULL;
}

/** Destructor. */
CLIPSAspectIniFin::~CLIPSAspectIniFin()
{
  logger_ = NULL;
}


LockPtr<CLIPS::Environment>
CLIPSAspectIniFin::new_env(const char *log_component_name)
{
  // CLIPS overwrites the SIGINT handler, restore it after
  // initializing the environment
  struct sigaction oldact;
  if (sigaction(SIGINT, NULL, &oldact) == 0) {
    LockPtr<CLIPS::Environment> clips(new CLIPS::Environment(),
				      /* recursive mutex */ true);

    CLIPSContextMaintainer *cm =
      new CLIPSContextMaintainer(logger_, log_component_name);

    void *env = clips->cobj();

    SetEnvironmentContext(env, cm);

    EnvAddRouterWithContext(env, (char *)"fawkeslog",
                            /* exclusive */ 30,
                            log_router_query,
                            log_router_print,
                            /* getc */   NULL,
                            /* ungetc */ NULL,
                            log_router_exit,
                            cm->logger);

    // restore old action
    sigaction(SIGINT, &oldact, NULL);

    return clips;
  } else {
    throw CannotInitializeThreadException("CLIPS: Unable to backup "
                                          "SIGINT sigaction for restoration.");
  }
}

void
CLIPSAspectIniFin::init(Thread *thread)
{
  CLIPSAspect *clips_thread;
  clips_thread = dynamic_cast<CLIPSAspect *>(thread);
  if (clips_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "CLIPSAspect, but RTTI says it "
					  "has not. ", thread->name());
  }
  
  LockPtr<CLIPS::Environment> clips;

  if (envs_.find(clips_thread->clips_env_name) != envs_.end()) {
    ClipsEnvData &envd = envs_[clips_thread->clips_env_name];

    if (clips_thread->CLIPSAspect_exclusive_) {
      throw CannotInitializeThreadException("Thread '%s' requires exclusive access "
					    "to already existing CLIPS environment '%s'",
					    thread->name(),
					    clips_thread->clips_env_name.c_str());
    }
    if (envd.exclusive) {
      throw CannotInitializeThreadException("Thread '%s' requires (shared) access "
					    "to already existing CLIPS environment '%s' "
					    "that has exclusive access set by thread '%s'",
					    thread->name(),
					    clips_thread->clips_env_name.c_str(),
					    envd.exclusive_holder.c_str());
    }
    clips = envd.env;
  } else if (clips_thread->CLIPSAspect_create_) {
    clips = new_env(clips_thread->get_CLIPSAspect_log_component_name());

    ClipsEnvData envd;
    envd.exclusive = clips_thread->CLIPSAspect_exclusive_;
    if (envd.exclusive) {
      envd.exclusive_holder = thread->name();
    }
    envd.env = clips;
    envs_[clips_thread->clips_env_name] = envd;

  } else {
    throw CannotInitializeThreadException("CLIPS environment '%s' does not exist and"
					  "creation has been disabled by thread '%s'",
					  clips_thread->clips_env_name.c_str(),
					  thread->name());
  }

  if (clips) {
    clips_thread->init_CLIPSAspect(clips);
  } else {
    throw CannotInitializeThreadException("Failed to initialize CLIPS environment '%s' "
					  "for thread '%s'",
					  clips_thread->clips_env_name.c_str(),
					  thread->name());
  }
}

void
CLIPSAspectIniFin::finalize(Thread *thread)
{
  CLIPSAspect *clips_thread;
  clips_thread = dynamic_cast<CLIPSAspect *>(thread);
  if (clips_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"CLIPSAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  void *env = clips_thread->clips->cobj();
  CLIPSContextMaintainer *cm =
    static_cast<CLIPSContextMaintainer *>(GetEnvironmentContext(env));

  EnvDeleteRouter(env, (char *)"fawkeslog");
  SetEnvironmentContext(env, NULL);
  delete cm;

  clips_thread->finalize_CLIPSAspect();

  if (envs_.find(clips_thread->clips_env_name) != envs_.end()) {
    ClipsEnvData &envd = envs_[clips_thread->clips_env_name];
    if (envd.env.refcount() == 1) { // only the env data references the environment
      //logger_->log_debug("CLIPSAspectIniFin", "Destroying environment %s",
      //		   clips_thread->clips_env_name.c_str());
      envs_.erase(clips_thread->clips_env_name);
    }
  }
}



/** Set the logger to use for logging (print to "l" output).
 * @param logger logger to use
 */
void
CLIPSAspectIniFin::set_logger(Logger *logger)
{
  logger_ = logger;
}

} // end namespace fawkes
