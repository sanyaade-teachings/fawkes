
----------------------------------------------------------------------------
--  exec.lua - Skiller execution functions
--
--  Created: Sun Feb 28 00:11:39 2010 (Olympic Winter Games 2010)
--  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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
module(..., fawkes.modinit.register_all)
require("fawkes.logprint")

local channels = {}

--- Execute skill.
-- This method will execute the skill string in any available free
-- channel. An error is thrown if no free channel is available.
-- @param skill_string skill string to execute
-- @return channel number the skill is executed in
function exec_skill(skill_string)
end


--- Stop execution of a specific channel.
-- @param channel channel number of the skill to stop
function stop_skill(channel)
end

--- Stop execution of all skills.
function stop_skills()
end


--- Get status of all channels.
-- @return list of stati of all channels
function get_status()
end


--- Get combined skill string.
-- @return skill string concatenating strings of all active channels with "||"
function get_skillstring()
end


--- Set number of channels.
-- Must be called before executing any skills.
-- @param num_channels number of channels to initialize
function set_num_channels(num_channels)
   channels = {}
   for i = 1, num_channels do
      channels[i] = SkillChannel:new()
   end
end
