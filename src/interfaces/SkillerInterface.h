
/***************************************************************************
 *  SkillerInterface.h - Fawkes BlackBoard Interface - SkillerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008-2010  Tim Niemueller
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

#ifndef __INTERFACES_SKILLERINTERFACE_H_
#define __INTERFACES_SKILLERINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class SkillerInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(SkillerInterface)
 /// @endcond
 public:
  /* constants */

  /** 
	This determines the current status of skill execution.
       */
  typedef enum {
    S_INACTIVE /**< No skill is running. */,
    S_FINAL /**< The skill string has been successfully processed. */,
    S_RUNNING /**< The execution is still running. */,
    S_FAILED /**< The execution failed and cannot succeed anymore. */
  } SkillStatusEnum;
  const char * tostring_SkillStatusEnum(SkillStatusEnum value) const;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int exclusive_controller; /**< 
      Instance serial of the exclusive controller of the skiller. If this does not
      carry your instance serial your exec messages will be ignored. Aquire control with
      the AquireControlMessage. Make sure you release control before exiting.
     */
    unsigned int msgid[16]; /**< 
     */
    char skill_string[1024]; /**< 
      Combined string showing as much as possible of the currently
      executed skills string, at most the first 1023 bytes of it.
      Must be properly null-terminated. Shall be of the following
      format: S skill1(param) || S skill2() [||...]  where S is one of
      F, R, X (marking the skill state, where X is failure) and
      skill1(param) etc. are the skills of the respective
      sandboxes. || is used to separate the strings (representing
      concurrent execution). Inactive channels shall be omitted.
     */
    char error[128]; /**< 
      String describing the error. Can be set by a skill when it
      fails. Shall be of the form: N string | N string...
      where N denotes the skill channel and string the respective
      error string. Active channels which did not post an error
      message shall be omitted.
     */
    SkillStatusEnum status[16]; /**< 
      The status of the current skill execution for the appropriate channel.
     */
  } SkillerInterface_data_t;

  SkillerInterface_data_t *data;

 public:
  /* messages */
  class ExecSkillMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      char skill_string[1024]; /**< 
      Combined string showing as much as possible of the currently
      executed skills string, at most the first 1023 bytes of it.
      Must be properly null-terminated. Shall be of the following
      format: S skill1(param) || S skill2() [||...]  where S is one of
      F, R, X (marking the skill state, where X is failure) and
      skill1(param) etc. are the skills of the respective
      sandboxes. || is used to separate the strings (representing
      concurrent execution). Inactive channels shall be omitted.
     */
    } ExecSkillMessage_data_t;

    ExecSkillMessage_data_t *data;

   public:
    ExecSkillMessage(const char * ini_skill_string);
    ExecSkillMessage();
    ~ExecSkillMessage();

    ExecSkillMessage(const ExecSkillMessage *m);
    /* Methods */
    char * skill_string() const;
    void set_skill_string(const char * new_skill_string);
    size_t maxlenof_skill_string() const;
    virtual Message * clone() const;
  };

  class StopExecMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int channel; /**< Which channel to stop. */
    } StopExecMessage_data_t;

    StopExecMessage_data_t *data;

   public:
    StopExecMessage(const unsigned int ini_channel);
    StopExecMessage();
    ~StopExecMessage();

    StopExecMessage(const StopExecMessage *m);
    /* Methods */
    unsigned int channel() const;
    void set_channel(const unsigned int new_channel);
    size_t maxlenof_channel() const;
    virtual Message * clone() const;
  };

  class StopAllMessage : public Message
  {
   public:
    StopAllMessage();
    ~StopAllMessage();

    StopAllMessage(const StopAllMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class AcquireControlMessage : public Message
  {
   public:
    AcquireControlMessage();
    ~AcquireControlMessage();

    AcquireControlMessage(const AcquireControlMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class ReleaseControlMessage : public Message
  {
   public:
    ReleaseControlMessage();
    ~ReleaseControlMessage();

    ReleaseControlMessage(const ReleaseControlMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  SkillerInterface();
  ~SkillerInterface();

 public:
  /* Methods */
  char * skill_string() const;
  void set_skill_string(const char * new_skill_string);
  size_t maxlenof_skill_string() const;
  char * error() const;
  void set_error(const char * new_error);
  size_t maxlenof_error() const;
  unsigned int exclusive_controller() const;
  void set_exclusive_controller(const unsigned int new_exclusive_controller);
  size_t maxlenof_exclusive_controller() const;
  SkillStatusEnum * status() const;
  SkillStatusEnum status(unsigned int index) const;
  void set_status(unsigned int index, const SkillStatusEnum new_status);
  void set_status(const SkillStatusEnum * new_status);
  size_t maxlenof_status() const;
  unsigned int * msgid() const;
  unsigned int msgid(unsigned int index) const;
  void set_msgid(unsigned int index, const unsigned int new_msgid);
  void set_msgid(const unsigned int * new_msgid);
  size_t maxlenof_msgid() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
