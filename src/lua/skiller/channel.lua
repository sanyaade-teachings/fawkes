
----------------------------------------------------------------------------
--  channel.lua - Skill channel class
--
--  Created: Sun Feb 28 14:27:40 2010 (Olympic Winter Games 2010)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--
----------------------------------------------------------------------------

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
require("fawkes.logprint")

local skillstati = require("skiller.skillstati")

--- @class SkillChannel Skill channel
-- A skill channel is an independent unit of skill execution and monitoring.
-- Any number of parallel skill channels may exist.
-- @author Tim Niemueller
SkillChannel = {}

--- Constructor.
function SkillChannel:new()
   local o = {}
   setmetatable(o, self)
   self.__index = self

   o.skillstring = ""
   o.statusval = skillstati.S_INACTIVE

   return o
end


--- Execute iteration.
-- Execute one iteration of the skill
function SkillChannel:loop()
   assert(self.func, "No skill string set for execution")
   

   self.sandbox.__skill_status = { final={}, failed={}, running={} }
   self.func()

   if     #self.sandbox.__skill_status.failed > 0  then
      self.statusval = skillstati.S_FAILED
   elseif #self.sandbox.__skill_status.running > 0 then
      self.statusval = skillstati.S_RUNNING
   elseif #self.sandbox.__skill_status.final > 0   then
      self.statusval = skillstati.S_FINAL
   else self.statusval = skillstati.S_INACTIVE end
end


--- Execute a new string.
-- @param skillstring skill string to execute
function SkillChannel:exec(skillstring)
   assert(self.sandbox == nil, "Skill is already being executed.")
   self.skillstring = skillstring
   self.sandbox     = skillenv.gensandbox()
   self.sandbox.__skill_status = { final={}, failed={}, running={} }
   self.func        = loadstring(skillstring)
   self.statusval   = skillstati.S_RUNNING
   setfenv(self.func, self.sandbox)
end


--- Stop execution of the current skill.
-- This will also reset the internal sandbox and skill status.
function SkillChannel:stop()
   if self.statusval ~= skillstati.S_INACTIVE then
      skillenv.reset_skills_from_status(self.sandbox.__skill_status)
      skillenv.deactivate_skills(self.sandbox.dep_skills)
      self.skillstring = ""
      self.sandbox      = nil
      self.func         = nil
      self.statusval    = skillstati.S_INACTIVE
   end
end


--- Get the current status of the skill execution
-- @return skill status value
function SkillChannel:status()
   return self.statusval
end


--- Check if channel is inactive.
-- If the channel is inactive it may be re-assigned to a new skill string.
-- @return true if no skill is currently being executed, false otherwise
function SkillChannel:inactive()
   return self.statusval == skillstati.S_INACTIVE
end


--- Check if channel is final.
-- @return true if skill string execution is final, false otherwise
function SkillChannel:final()
   return self.statusval == skillstati.S_FINAL
end


--- Check if channel is failed.
-- @return true if skill string execution failed, false otherwise
function SkillChannel:final()
   return self.statusval == skillstati.S_FAILED
end


--- Check if channel is running.
-- @return true if skill string execution still running, false otherwise
function SkillChannel:running()
   return self.statusval == skillstati.S_RUNNING
end


--- Get skill string.
-- @return skill string currently executed in this channel
function SkillChannel:get_skillstring()
   local rv = ""
   if     self.statusval == skillstati.S_RUNNING then rv = "R "
   elseif self.statusval == skillstati.S_FINAL   then rv = "F "
   elseif self.statusval == skillstati.S_FAILED  then rv = "X "
   else rv = "I" end
   return rv .. self.skillstring
end

