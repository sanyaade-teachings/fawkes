
------------------------------------------------------------------------
--  set.lua - 
--
--  Created: Tue May 04 10:01:18 2010
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

module("fawkes.datastruct.set", fawkes.modinit.register_all)

--- Create a set from a list
-- @param list a list of items to creat the set from
-- @return the new set
function create(list)
   local set = {}
   for _,item in ipairs(list) do
      set[item] = true
   end
   return set
end

--- Insert an item into a set
-- @param set the set to insert the item into
-- @param item the item to insert into the set
-- @return true if the item was not already in the set,
-- false otherwise
function insert(set, item)
   if not set[item] then
      set[item] = true
      return true
   else
      return false
   end
end

--- Remove an item from the set
-- @param set the set to remove the item from
-- @param item the item to remove from the set
-- @return true if the item is in the set, false otherwise
function remove(set, item)
   if set[item] then
      set[item] = nil
      return true
   else
      return false
   end
end

--- Get the set items as list
-- @param set the set from which the list items are taken
-- @return a list containing the items of the set
function to_list(set)
   local list = {}
   for item,_ in pairs(set) do
      table.insert(list, item)
   end
   return list
end

--- The union of two sets
-- @param a the first set
-- @param b the second set
-- @return the union of a and b
function union(a,b)
   local union_set = {}
   for item,_ in pairs(a) do
      union_set[item] = true
   end
   for item,_ in pairs(b) do
      union_set[item] = true
   end
   return union_set
end

--- The difference of two sets
-- @param a the first set
-- @param b the second set
-- @return the difference (a\b)
function difference(a,b)
   local diff_set = {}
   for item,_ in pairs(a) do
      if not b[item] then
	 insert(diff_set,item)
      end
   end
   return diff_set
end

--- The intersection of a and b
-- @param a the first set
-- @param b the second set
-- @return the intersection of a and b
function intersection(a,b)
   local intersect_set = {}
   for item,_ in pairs(a) do
      if b[item] then
	 insert(intersect_set, item)
      end
   end
   return intersect_set
end

-- Print all items of a set
function to_string(set)
   item_list = {}
   for item,_ in pairs(set) do
      table.insert(item_list,tostring(item))
   end
   set_string = table.concat(item_list,", ")
   return "{"..set_string.."}"
end