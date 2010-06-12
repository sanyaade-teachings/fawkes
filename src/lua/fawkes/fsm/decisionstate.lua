
------------------------------------------------------------------------
--  decisionstate.lua - FSM decision state
--
--  Created: Mon May 10 14:52:32 2010
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

--- This module provides an decision state that jumps to one of two
-- target states based on the positive or negative evaluation of
-- a jump condition
-- @author Patrick Podbregar
module(..., fawkes.modinit.module_init)
local oo = require("fawkes.ootools")
local jsmod = require("fawkes.fsm.jumpstate")
local JumpState = jsmod.JumpState

DecisionState = {}

function DecisionState:new(o)
   assert(o, "DecisionState requires a table as argument")
   local base = JumpState:new(o)
   local params = self:extract_params(o)
   
   local ds = oo.create_instance(self, params, base)

   ds:create_transitions_from_cond()

   return ds
end

--- Extract parameters from table
-- Extracts parameters needed by this kind of state from a table
-- @param o the table to take the parameters from
function DecisionState:extract_params(o)
   local s = {}
   assert(o.true_to, "The DecisionState " .. o.name .. " needs a state to jump to if the condition is true")
   assert(o.false_to, "The DecisionState " .. o.name .. " needs a state to jump to if the condition is false")
   assert(o.cond, "The DecisionState " .. o.name .. " needs a condition")

   s.true_to = o.true_to
   s.false_to = o.false_to
   s.cond = o.cond

   return s
end

--- Create transitions based on the decision
function DecisionState:create_transitions_from_cond()
   local cond = self.cond
   local positive, negative
   if type(cond) == "function" then
      print("cond is a function")
      positive = cond
      negative = function () return not cond() end
   elseif type(cond) == "string" then
      positive = cond
      negative = "not (" .. cond ..")"
   else
      error("Wrong type of condition in DecisionState " .. self.name)
   end

   self:add_transition(self.true_to, positive, "Decision positive")
   self:add_transition(self.false_to, negative, "Decision negatve")
end