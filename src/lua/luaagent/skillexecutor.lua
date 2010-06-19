
------------------------------------------------------------------------
--  skillexecutor.lua - A module to control the execution of skills
--
--  Created: Tue May 18 12:57:03 2010
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

--- This module provides functions to start and stop
-- the execution of skills
module(..., fawkes.modinit.module_init)

local skillstati = require("skiller.skillstati")

local debug = false

local skiller = interfaces.reading.skiller
local maxn_running_skills = skiller:maxlenof_msgid()
local running_skills = {}

--- Returns the skill channel a skill is running on
-- @param skill the skill wrapper to get the channel for
-- @return the channel the skill is running on (1 based)
function get_channel(skill)
   local skill_data = running_skills[skill]
   assert(skill_data, "Failed to get skill channel of ".. skill.name ..
	  ". Skill is not running.")
   if not skill_data.channel then
      for c_ch_num = 0, maxn_running_skills-1 do
	 if skiller:msgid(c_ch_num) == skill_data.msgid then
	    skill_data.channel = c_ch_num + 1
	    break
	 end
      end
   end

   return skill_data.channel
end

--- Stop skill execution
-- Stops the execution and resets a given skill
-- @param skill the skill wrapper for the skill we want to stop
function stop(skill)
   if debug then
      print("SkillExecutor: stop "..skill.name)
   end
   if running_skills[skill] then
      local ch_num = get_channel(skill)
      if ch_num then
	 local msg = skiller.StopExecMessage:new(ch_num)
	 skiller:msgq_enqueue_copy(msg)
      end

      skill:reset()
      running_skills[skill] = nil
    end
end

--- Stop the execution of all skills
-- Stops the execution and resets all running skills
function stop_all()
   if debug then
      print("SkillExecutor: stop all")
   end
   local msg = skiller.StopAllMessage:new()
   skiller:msgq_enqueue_copy(msg)

   for skill,_ in pairs(running_skills) do
      skill:reset()
   end
   running_skills = {}
end

--- Start the execution of a skill
-- This method takes a skill wrapper and the corresponding
-- arguments and starts its execution. If the skill was
-- already running it is restarted. If all skill channels
-- are currently occupied the skill's status is set to failed
-- and it is not enqueued.
-- @param skill_with_args a table which has for example the form
-- {skill, {arg1=a, arg2=b}}. "skill" must be a skill wrapper.
function start(skill_with_args)
   local skill = skill_with_args[1]
   local args = skill_with_args[2]

   if debug then
      print("SkillExecutor: start "..skill.name)
   end

   if running_skills[skill] then
      stop(skill)
   end

   msgid = skill:start(args)
   running_skills[skill] = {msgid = msgid}
end

--- Get the status of a skill
-- The function returns the current state of a skill
-- @param skill the queried skill
-- @return the status of the skill
function get_status(skill)
   local status = skillstati.S_INACTIVE
   if running_skills[skill] then
      local ch_num = get_channel(skill)
      if ch_num then
	 status =  skiller:status(ch_num-1)
      end
   end
   return status
end

--- Cleans up failed and final skills
function clean_up()
   if debug then
      print("SkillExecutor: cleanup")
   end
   for skill,_ in pairs(running_skills) do
      local status = get_status(skill)
      if status == skillstati.S_FINAL or status == skillstati.S_FAILED then
	 if debug then
	    print("  clean "..skill.name)
	 end
	 stop(skill)
      else
	 if debug then
	    print("  keep "..skill.name)
	 end
      end
   end
end
