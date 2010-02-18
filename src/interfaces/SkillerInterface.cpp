
/***************************************************************************
 *  SkillerInterface.cpp - Fawkes BlackBoard Interface - SkillerInterface
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

#include <interfaces/SkillerInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class SkillerInterface <interfaces/SkillerInterface.h>
 * SkillerInterface Fawkes BlackBoard Interface.
 * 
      The interface provides access to the skill execution runtime plugin.
      It provides basic status information about skiller and allows for
      calling skills via messages. It can also be used to manually restart
      the Lua interpreter if something is wedged.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
SkillerInterface::SkillerInterface() : Interface()
{
  data_size = sizeof(SkillerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (SkillerInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "skill_string", 1024, data->skill_string);
  add_fieldinfo(IFT_STRING, "error", 128, data->error);
  add_fieldinfo(IFT_UINT, "exclusive_controller", 1, &data->exclusive_controller);
  add_fieldinfo(IFT_ENUM, "status", 16, &data->status, "SkillStatusEnum");
  add_fieldinfo(IFT_UINT, "msgid", 16, &data->msgid);
  add_messageinfo("ExecSkillMessage");
  add_messageinfo("StopExecMessage");
  add_messageinfo("StopAllMessage");
  add_messageinfo("AcquireControlMessage");
  add_messageinfo("ReleaseControlMessage");
  unsigned char tmp_hash[] = {0xa7, 0xd4, 0x23, 0x4c, 0xc0, 0xf8, 0x99, 0x8e, 0x3f, 0x87, 0x97, 0x8c, 0xb6, 0xc0, 0x71, 0x4f};
  set_hash(tmp_hash);
}

/** Destructor */
SkillerInterface::~SkillerInterface()
{
  free(data_ptr);
}
/** Convert SkillStatusEnum constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
SkillerInterface::tostring_SkillStatusEnum(SkillStatusEnum value) const
{
  switch (value) {
  case S_INACTIVE: return "S_INACTIVE";
  case S_FINAL: return "S_FINAL";
  case S_RUNNING: return "S_RUNNING";
  case S_FAILED: return "S_FAILED";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get skill_string value.
 * 
      Combined string showing as much as possible of the currently
      executed skills string, at most the first 1023 bytes of it.
      Must be properly null-terminated. Shall be of the following
      format: S skill1(param) || S skill2() [||...]  where S is one of
      F, R, X (marking the skill state, where X is failure) and
      skill1(param) etc. are the skills of the respective
      sandboxes. || is used to separate the strings (representing
      concurrent execution). Inactive channels shall be omitted.
    
 * @return skill_string value
 */
char *
SkillerInterface::skill_string() const
{
  return data->skill_string;
}

/** Get maximum length of skill_string value.
 * @return length of skill_string value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_skill_string() const
{
  return 1024;
}

/** Set skill_string value.
 * 
      Combined string showing as much as possible of the currently
      executed skills string, at most the first 1023 bytes of it.
      Must be properly null-terminated. Shall be of the following
      format: S skill1(param) || S skill2() [||...]  where S is one of
      F, R, X (marking the skill state, where X is failure) and
      skill1(param) etc. are the skills of the respective
      sandboxes. || is used to separate the strings (representing
      concurrent execution). Inactive channels shall be omitted.
    
 * @param new_skill_string new skill_string value
 */
void
SkillerInterface::set_skill_string(const char * new_skill_string)
{
  strncpy(data->skill_string, new_skill_string, sizeof(data->skill_string));
}

/** Get error value.
 * 
      String describing the error. Can be set by a skill when it
      fails. Shall be of the form: N string | N string...
      where N denotes the skill channel and string the respective
      error string. Active channels which did not post an error
      message shall be omitted.
    
 * @return error value
 */
char *
SkillerInterface::error() const
{
  return data->error;
}

/** Get maximum length of error value.
 * @return length of error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_error() const
{
  return 128;
}

/** Set error value.
 * 
      String describing the error. Can be set by a skill when it
      fails. Shall be of the form: N string | N string...
      where N denotes the skill channel and string the respective
      error string. Active channels which did not post an error
      message shall be omitted.
    
 * @param new_error new error value
 */
void
SkillerInterface::set_error(const char * new_error)
{
  strncpy(data->error, new_error, sizeof(data->error));
}

/** Get exclusive_controller value.
 * 
      Instance serial of the exclusive controller of the skiller. If this does not
      carry your instance serial your exec messages will be ignored. Aquire control with
      the AquireControlMessage. Make sure you release control before exiting.
    
 * @return exclusive_controller value
 */
unsigned int
SkillerInterface::exclusive_controller() const
{
  return data->exclusive_controller;
}

/** Get maximum length of exclusive_controller value.
 * @return length of exclusive_controller value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_exclusive_controller() const
{
  return 1;
}

/** Set exclusive_controller value.
 * 
      Instance serial of the exclusive controller of the skiller. If this does not
      carry your instance serial your exec messages will be ignored. Aquire control with
      the AquireControlMessage. Make sure you release control before exiting.
    
 * @param new_exclusive_controller new exclusive_controller value
 */
void
SkillerInterface::set_exclusive_controller(const unsigned int new_exclusive_controller)
{
  data->exclusive_controller = new_exclusive_controller;
}

/** Get status value.
 * 
      The status of the current skill execution for the appropriate channel.
    
 * @return status value
 */
SkillerInterface::SkillStatusEnum *
SkillerInterface::status() const
{
  return data->status;
}

/** Get status value at given index.
 * 
      The status of the current skill execution for the appropriate channel.
    
 * @param index index of value
 * @return status value
 * @exception Exception thrown if index is out of bounds
 */
SkillerInterface::SkillStatusEnum
SkillerInterface::status(unsigned int index) const
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  return data->status[index];
}

/** Get maximum length of status value.
 * @return length of status value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_status() const
{
  return 16;
}

/** Set status value.
 * 
      The status of the current skill execution for the appropriate channel.
    
 * @param new_status new status value
 */
void
SkillerInterface::set_status(const SkillStatusEnum * new_status)
{
  memcpy(data->status, new_status, sizeof(SkillStatusEnum) * 16);
}

/** Set status value at given index.
 * 
      The status of the current skill execution for the appropriate channel.
    
 * @param new_status new status value
 * @param index index for of the value
 */
void
SkillerInterface::set_status(unsigned int index, const SkillStatusEnum new_status)
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  data->status[index] = new_status;
}
/** Get msgid value.
 * 
    
 * @return msgid value
 */
unsigned int *
SkillerInterface::msgid() const
{
  return data->msgid;
}

/** Get msgid value at given index.
 * 
    
 * @param index index of value
 * @return msgid value
 * @exception Exception thrown if index is out of bounds
 */
unsigned int
SkillerInterface::msgid(unsigned int index) const
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  return data->msgid[index];
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::maxlenof_msgid() const
{
  return 16;
}

/** Set msgid value.
 * 
    
 * @param new_msgid new msgid value
 */
void
SkillerInterface::set_msgid(const unsigned int * new_msgid)
{
  memcpy(data->msgid, new_msgid, sizeof(unsigned int) * 16);
}

/** Set msgid value at given index.
 * 
    
 * @param new_msgid new msgid value
 * @param index index for of the value
 */
void
SkillerInterface::set_msgid(unsigned int index, const unsigned int new_msgid)
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  data->msgid[index] = new_msgid;
}
/* =========== message create =========== */
Message *
SkillerInterface::create_message(const char *type) const
{
  if ( strncmp("ExecSkillMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ExecSkillMessage();
  } else if ( strncmp("StopExecMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopExecMessage();
  } else if ( strncmp("StopAllMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopAllMessage();
  } else if ( strncmp("AcquireControlMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new AcquireControlMessage();
  } else if ( strncmp("ReleaseControlMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ReleaseControlMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
SkillerInterface::copy_values(const Interface *other)
{
  const SkillerInterface *oi = dynamic_cast<const SkillerInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(SkillerInterface_data_t));
}

const char *
SkillerInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "SkillStatusEnum") == 0) {
    return tostring_SkillStatusEnum((SkillStatusEnum)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class SkillerInterface::ExecSkillMessage <interfaces/SkillerInterface.h>
 * ExecSkillMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_skill_string initial value for skill_string
 */
SkillerInterface::ExecSkillMessage::ExecSkillMessage(const char * ini_skill_string) : Message("ExecSkillMessage")
{
  data_size = sizeof(ExecSkillMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ExecSkillMessage_data_t *)data_ptr;
  strncpy(data->skill_string, ini_skill_string, 1024);
  add_fieldinfo(IFT_STRING, "skill_string", 1024, data->skill_string);
}
/** Constructor */
SkillerInterface::ExecSkillMessage::ExecSkillMessage() : Message("ExecSkillMessage")
{
  data_size = sizeof(ExecSkillMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ExecSkillMessage_data_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "skill_string", 1024, data->skill_string);
}

/** Destructor */
SkillerInterface::ExecSkillMessage::~ExecSkillMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::ExecSkillMessage::ExecSkillMessage(const ExecSkillMessage *m) : Message("ExecSkillMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ExecSkillMessage_data_t *)data_ptr;
}

/* Methods */
/** Get skill_string value.
 * 
      Combined string showing as much as possible of the currently
      executed skills string, at most the first 1023 bytes of it.
      Must be properly null-terminated. Shall be of the following
      format: S skill1(param) || S skill2() [||...]  where S is one of
      F, R, X (marking the skill state, where X is failure) and
      skill1(param) etc. are the skills of the respective
      sandboxes. || is used to separate the strings (representing
      concurrent execution). Inactive channels shall be omitted.
    
 * @return skill_string value
 */
char *
SkillerInterface::ExecSkillMessage::skill_string() const
{
  return data->skill_string;
}

/** Get maximum length of skill_string value.
 * @return length of skill_string value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::ExecSkillMessage::maxlenof_skill_string() const
{
  return 1024;
}

/** Set skill_string value.
 * 
      Combined string showing as much as possible of the currently
      executed skills string, at most the first 1023 bytes of it.
      Must be properly null-terminated. Shall be of the following
      format: S skill1(param) || S skill2() [||...]  where S is one of
      F, R, X (marking the skill state, where X is failure) and
      skill1(param) etc. are the skills of the respective
      sandboxes. || is used to separate the strings (representing
      concurrent execution). Inactive channels shall be omitted.
    
 * @param new_skill_string new skill_string value
 */
void
SkillerInterface::ExecSkillMessage::set_skill_string(const char * new_skill_string)
{
  strncpy(data->skill_string, new_skill_string, sizeof(data->skill_string));
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerInterface::ExecSkillMessage::clone() const
{
  return new SkillerInterface::ExecSkillMessage(this);
}
/** @class SkillerInterface::StopExecMessage <interfaces/SkillerInterface.h>
 * StopExecMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_channel initial value for channel
 */
SkillerInterface::StopExecMessage::StopExecMessage(const unsigned int ini_channel) : Message("StopExecMessage")
{
  data_size = sizeof(StopExecMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopExecMessage_data_t *)data_ptr;
  data->channel = ini_channel;
  add_fieldinfo(IFT_UINT, "channel", 1, &data->channel);
}
/** Constructor */
SkillerInterface::StopExecMessage::StopExecMessage() : Message("StopExecMessage")
{
  data_size = sizeof(StopExecMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopExecMessage_data_t *)data_ptr;
  add_fieldinfo(IFT_UINT, "channel", 1, &data->channel);
}

/** Destructor */
SkillerInterface::StopExecMessage::~StopExecMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::StopExecMessage::StopExecMessage(const StopExecMessage *m) : Message("StopExecMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopExecMessage_data_t *)data_ptr;
}

/* Methods */
/** Get channel value.
 * Which channel to stop.
 * @return channel value
 */
unsigned int
SkillerInterface::StopExecMessage::channel() const
{
  return data->channel;
}

/** Get maximum length of channel value.
 * @return length of channel value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SkillerInterface::StopExecMessage::maxlenof_channel() const
{
  return 1;
}

/** Set channel value.
 * Which channel to stop.
 * @param new_channel new channel value
 */
void
SkillerInterface::StopExecMessage::set_channel(const unsigned int new_channel)
{
  data->channel = new_channel;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerInterface::StopExecMessage::clone() const
{
  return new SkillerInterface::StopExecMessage(this);
}
/** @class SkillerInterface::StopAllMessage <interfaces/SkillerInterface.h>
 * StopAllMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::StopAllMessage::StopAllMessage() : Message("StopAllMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SkillerInterface::StopAllMessage::~StopAllMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::StopAllMessage::StopAllMessage(const StopAllMessage *m) : Message("StopAllMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerInterface::StopAllMessage::clone() const
{
  return new SkillerInterface::StopAllMessage(this);
}
/** @class SkillerInterface::AcquireControlMessage <interfaces/SkillerInterface.h>
 * AcquireControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::AcquireControlMessage::AcquireControlMessage() : Message("AcquireControlMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SkillerInterface::AcquireControlMessage::~AcquireControlMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::AcquireControlMessage::AcquireControlMessage(const AcquireControlMessage *m) : Message("AcquireControlMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerInterface::AcquireControlMessage::clone() const
{
  return new SkillerInterface::AcquireControlMessage(this);
}
/** @class SkillerInterface::ReleaseControlMessage <interfaces/SkillerInterface.h>
 * ReleaseControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
SkillerInterface::ReleaseControlMessage::ReleaseControlMessage() : Message("ReleaseControlMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
SkillerInterface::ReleaseControlMessage::~ReleaseControlMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
SkillerInterface::ReleaseControlMessage::ReleaseControlMessage(const ReleaseControlMessage *m) : Message("ReleaseControlMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SkillerInterface::ReleaseControlMessage::clone() const
{
  return new SkillerInterface::ReleaseControlMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
SkillerInterface::message_valid(const Message *message) const
{
  const ExecSkillMessage *m0 = dynamic_cast<const ExecSkillMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const StopExecMessage *m1 = dynamic_cast<const StopExecMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const StopAllMessage *m2 = dynamic_cast<const StopAllMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const AcquireControlMessage *m3 = dynamic_cast<const AcquireControlMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const ReleaseControlMessage *m4 = dynamic_cast<const ReleaseControlMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SkillerInterface)
/// @endcond


} // end namespace fawkes
