
------------------------------------------------------------------------
--  ootools.lua - Tools for object oriented programming
--
--  Created: Tue May 11 13:47:29 2010
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

module(..., package.seeall)
--- This module provides some convenience functions for object oriented
-- programming
-- @author Patrick Podbregar

function create_instance(class, params, base)
   local instance = {}
   params = params or {}

   for k,v in pairs(class) do
      instance[k] = v
   end
   for k,v in pairs(params) do
      instance[k] = v
   end

   if base then
      setmetatable(instance, {__index = base})
   end
   
   return instance
end