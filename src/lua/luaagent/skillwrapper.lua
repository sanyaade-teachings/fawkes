
------------------------------------------------------------------------
--  skillwrapper.lua - a class to wrap skills and skill string creation
--
--  Created: Tue May 18 13:52:08 2010
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

module(..., fawkes.modinit.module_init)
local oo = require("fawkes.ootools")
local SkillExecutor = require("luaagent.skillexecutor")
local skillstati = require("skiller.skillstati")

SkillWrapper = {debug = true}
skiller = interfaces.reading.skiller

--- Creates a new skill wrapper
-- @param o the table to create the skill wrapper from
-- @return the newly creates skill wrapper
function SkillWrapper:new(o)
   assert(o, "SkillWrapper requires a table as argument")
   assert(type(o.name) == "string", "SkillWrapper requires "..
       "a name which must be a string")
   o.args = {}
   o.status = skillstati.S_INACTIVE
   o.debug = o.debug or self.debug
   return oo.create_instance(self, o)
end

--- Start execution of the wrapped skill
-- This function triggers the execution of the wrapped skill.
-- The arguments in the args parameter are only used if the
-- argument wasn't already set directly.
-- @param args the arguments for the skill (e.g {arg1=a, arg2=b})
-- @return the message id of the enqueued ExecSkillMessage
function SkillWrapper:start(args)
   if self.debug then
      print("SkillWrapper:start "..self.name)
   end
   local args = args or {}
   for k,v in pairs(args) do
      if not self.args[k] then
	 self.args[k] = v
      end
   end

   local skill_string = self:get_skill_string()
   local msg = skiller.ExecSkillMessage:new(skill_string)
   return skiller:msgq_enqueue_copy(msg)
end

--- Resets the skill wrapper
function SkillWrapper:reset()
   if self.debug then
      print("SkillWrapper:reset "..self.name)
   end
   self.fail = nil
   self.args = {}
end

--- Creates the skill string
-- @return the skill string
function SkillWrapper:get_skill_string()
   local params = ""
   if self.args then
      local subp = {}
      for k,v in pairs(self.args) do
	 if type(v) == "boolean" then
	    table.insert(subp, string.format("%s = %s", k, tostring(v)))
	 else
	    table.insert(subp, string.format("%s = %q", k, tostring(v)))
	 end
      end
      params = table.concat(subp, ", ")
   end
   return string.format("%s{%s}", self.name, params)
end

--- Manually sets the skills status to failed
function SkillWrapper:set_failed()
   self.fail = true
end

--- Creates a function that checks if the skill is running
-- The created function can be used a condition for a transition
function SkillWrapper:running()
   return function()
	     return SkillExecutor.get_status(self) == skillstati.S_RUNNING
	  end
end

--- Creates a function that checks if the skill is final
-- The created function can be used a condition for a transition
function SkillWrapper:final()
   return function()
	     return SkillExecutor.get_status(self) == skillstati.S_FINAL
	  end
end

--- Creates a function that checks if the skill has failed
-- The created function can be used a condition for a transition
function SkillWrapper:failed()
   return function()
	     return self.fail or SkillExecutor.get_status(self) == skillstati.S_FAILED
	  end
end
