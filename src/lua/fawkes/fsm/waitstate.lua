
------------------------------------------------------------------------
--  waitstate.lua - FSM wait state to wait a specified number of seconds
--
--  Created: Wed Dec 31 14:56:12 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

require("fawkes.modinit")

--- This module provides a wait state for HSMs.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)
local jsmod = require("fawkes.fsm.jumpstate")
local JumpState = jsmod.JumpState
local oo = require("fawkes.ootools")

--- @class WaitState
-- State that waits a specified number of seconds.
-- @author Tim Niemueller
WaitState = {}

--- Create new jump state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function WaitState:new(o)
   assert(o, "WaitState requires a table as argument")
   local base = JumpState:new(o)
   local params = self:extract_params(o)
   
   local ws = oo.create_instance(self, params, base)

   ws:add_transition(o.next_state, WaitState.jumpcond_timeover, "Timeout")

   return ws
end

--- Extract parameters from table
-- Extracts parameters needed by this kind of state from a table
-- @param o the table to take the parameters from
function WaitState:extract_params(o)
   local s = {}
   assert(o.next_state,"WaitState " .. o.name .. " requires a next_state")

   s.next_state = o.next_state
   s.time_sec = o.time_sec

   return s
end

--- Check if time has passed.
-- @return true if time has passed, false otherwise
function WaitState:jumpcond_timeover()
   self.now = self.now or Time:new()
   self.now:stamp()
   return (self.now - self.endtime) >= 0
end

--- Initialize WaitState.
-- @param time_sec time in seconds the instance should wait, maybe nil if a time
-- has been passed to the constructor as time_sec.
function WaitState:do_init(time_sec, ...)
   local rv = { self:try_transitions(self.preconditions) }
   if next(rv) then return unpack(rv) end

   self:init(...)

   assert(time_sec or self.time_sec, "WaitState " .. self.fsm.name .. ":" .. self.name .. " requires a time in seconds passed to init or new")

   self.time_sec = time_sec or self.time_sec
   self.endtime = self.endtime or Time:new()
   self.endtime:stamp()
   self.endtime:add(self.time_sec)

   return self:try_transitions()
end

function WaitState:do_loop()
   self.now = self.now or Time:new()
   if self.labeltime then
      local remaining = self.endtime - self.now
      self.dotattr.label = string.format("%s (%0.2f/%0.2f)",
					 self.name, remaining, self.time_sec)
      self.fsm:mark_changed()
   end

   return JumpState.do_loop(self)
end

function WaitState:do_exit()
   if self.labeltime then
      self.dotattr.label = string.format("%s (0.00/%0.2f)", self.name, self.time_sec)
   end
   JumpState.do_exit(self)
end

function WaitState:reset()
   JumpState.reset(self)
   if self.labeltime and self.time_sec then
      self.dotattr.label = string.format("%s (%0.2f/%0.2f)",
					 self.name, self.time_sec, self.time_sec)
   end
end
