
------------------------------------------------------------------------
--  skillstate.lua - A Jumpstate to start and stop skill execution
--
--  Created: Mon May 10 15:42:37 2010
--  Copyright  2010  Patrick Podbregar [www.podbregar.com]
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

--- This module provides a skill state that starts or stops the execution
-- of skills
module(..., fawkes.modinit.module_init)
local jsmod = require("fawkes.fsm.jumpstate")
local JumpState = jsmod.JumpState
local oo = require("fawkes.ootools")
local SkillExecutor = require("luaagent.skillexecutor")

SkillState = {}

--- Creates a new skill state
-- @param o the table to create the state from
function SkillState:new(o)
   assert(type(o) == "table", "SkillState requires a table as argument")
   local base = JumpState:new(o)
   local params = self:extract_params(o)

   local ss = oo.create_instance(self, params, base)

   return ss
end

--- Extract parameters from table
-- Extracts parameters needed by this kind of state from a table
-- @param o the table to take the parameters from
function SkillState:extract_params(o)
   local s = {}
   s.start = o.start
   s.stop = o.stop
   return s
end

--- Execute init routines.
-- First checks all preconditions, stops all skills
-- in the stop list, calls the states init funtion and
-- then starts all skills in the start list
function SkillState:do_init()
   local rv = { self:try_transitions(self.preconditions) }
   if next(rv) then return unpack(rv) end
   self:stop_skills()
   self:init()
   self:start_skills()
   return self:try_transitions()
end

--- Execute exit routines
-- Calls the states exit function and terminates failed and
-- finished states
function SkillState:do_exit()
   self:exit()
   SkillExecutor.clean_up()
end

--- Stops all skills in the states stop list
function SkillState:stop_skills()
   if type(self.stop) == "table" then
      for _,s in ipairs(self.stop) do
	 SkillExecutor.stop(s)
      end
   elseif type(self.stop) == "string" and self.stop == "all" then
      SkillExecutor.stop_all()
   end
end

--- Starts all skills in stats start list
function SkillState:start_skills()
   if self.start then
      for _,s in ipairs(self.start) do
	 SkillExecutor.start(s)
      end
   end
end