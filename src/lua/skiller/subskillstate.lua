
------------------------------------------------------------------------
--  subskillstate.lua - A state to excute skills inside other skills
--
--  Created: Thu Jun 03 13:49:03 2010
--  Copyright  2010  Patrick Podbregar [www.podbregar.com]
--             2008-2009  Tim Niemueller [www.niemueller.de]
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

module(..., fawkes.modinit.module_init)
local jsmod = require("fawkes.fsm.jumpstate")
local JumpState = jsmod.JumpState
local oo = require("fawkes.ootools")
local skillstati = require("skiller.skillstati")

SubSkillState = {}

--- Creates new subskill state
-- @param o the table to create the state from
function SubSkillState:new(o)
   assert(type(o) == "table", "SubSkillState requires a table as argument")
   local base = JumpState:new(o)
   local params = self:extract_params(o)

   local sss = oo.create_instance(self, params, base)

   sss.skill_status = skillstati.S_Running

   sss.final_transition = 
      sss:add_transition(sss.final, sss.jumpcond_skill_final, 
			 sss.skill.name.."() succeeded")
   sss.failure_transition = 
      sss:add_transition(sss.failed, sss.jumpcond_skill_failed,
			 sss.skill.name.."() failed")

   return sss
end

--- Extract parameters from table
-- Extracts parameters needed by this kind of state from a table
-- @param o the table to take the parameters from
function SubSkillState:extract_params(o)
   local s = {}
   assert(o.skill, "SubSkillState needs a skill to execute")
   assert(o.final, "SubSkillState needs a final follow state")

   s.skill = o.skill
   s.baseargs = o.args or {}
   s.final = o.final
   s.failed = o.failed or "FAILED"

   return s
end

function SubSkillState:jumpcond_skill_final()
   return self.skill_status == skillstati.S_FINAL
end

function SubSkillState:jumpcond_skill_failed()
   if self.skill_status == skillstati.S_FAILED then
      local error = ""
      if self.skill then
	 error = self.skill.error
	 if self.skill.fsm then
	    error = self.skill.fsm.error
	 end
      end

      if error and error ~= "" then
	 self.fsm:set_error(self.name .. "() failed, " .. error)
      end
      return true
   else
      return false
   end
end

--- Execute init routines.
-- This resets the subskills that is executed in this state and then executes
-- the state's init() routine. Do not overwrite do_init(), rather implement init().
function SubSkillState:do_init()
   self.args = nil

   -- Try preconditions
   local rv = { self:try_transitions(self.preconditions) }
   if next(rv) then return unpack(rv) end

   self:skill_reset()
   self.skill_status = skillstati.S_RUNNING
   self:init()

   return self:try_transitions()
end

--- Execute loop.
function SubSkillState:do_loop()
   self:loop()
   
   if self.fsm.debug then
      local s = self.skill.name .. "{"
      local first = true
      for k,v in pairs(self.args or self.baseargs) do
	 s = s .. string.format("%s%s = %s", first and "" or ", ", k, tostring(v))
	 first = false
      end
      s = s .. "}"
      printf("%s: executing %s", self.name, s)
   end

   self.skill_status = self.skill(self.args or self.baseargs)

   return self:try_transitions()
end

function SubSkillState:reset()
   JumpState.reset(self)
   self:skill_reset()
   self.skill_status = skillstati.S_INACTIVE
end

function SubSkillState:skill_reset()
   self.skill.reset()
   self.args = nil
end
