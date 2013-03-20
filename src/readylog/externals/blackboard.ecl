
/***************************************************************************
 *  blackboard.ecl - Access the blackboard from ECLiPSe
 *
 *  Created: Wed Mar 09 17:10:54 2011
 *  Copyright  2011  Daniel Beck
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

%% module definition
:- module(blackboard).
:- export bb_connect_remote/1.
:- export bb_connect/0.
:- export bb_disconnect/0.
:- export bb_is_alive/0.
:- export bb_open_interface/3.
:- export bb_close_interface/1.
:- export bb_has_writer/1.
:- export bb_instance_serial/2.
:- export bb_read_interfaces/0.
:- export bb_write_interfaces/0.
:- export bb_read_interface/3.
:- export bb_write_interface/3.
:- export bb_send_message/3.
:- export bb_recv_messages/2.

%% load the external code from the shared object
:- ( exists("../../../lib") ->
       load("../../../lib/eclipse_externals.so")
   ;
       load("../../../../lib/eclipse_externals.so")
   ).

%% definition of external predicates
:- external(bb_connect_remote/1, p_connect_to_remote_blackboard).
:- external(bb_connect/0, p_connect_to_eclipse_blackboard).
:- external(bb_disconnect/0, p_disconnect_from_blackboard).
:- external(bb_is_alive/0, p_is_alive).
:- external(bb_is_connected/0, p_is_connected).
:- external(bb_open_interface/3, p_open_interface).
:- external(bb_close_interface/1, p_close_interface).
:- external(bb_has_writer/1, p_has_writer).
:- external(bb_instance_serial/2, p_instance_serial).
:- external(bb_read_interfaces/0, p_read_interfaces).
:- external(bb_write_interfaces/0, p_write_interfaces).
:- external(bb_read_interface/3, p_read_from_interface).
:- external(bb_write_interface/3, p_write_to_interface).
:- external(bb_send_message/3, p_send_message).
:- external(bb_recv_messages/2, p_recv_messages).

%% shortcuts
bb_open_interface_writing(Type, Id) :-
        bb_open_interface(w, Type, Id).

bb_open_interface_reading(Type, Id) :-
        bb_open_interface(r, Type, Id).

bb_ensure_connected(Host) :- bb_is_connected ; bb_connect(r, Host).
bb_ensure_connected :- bb_is_connected ; bb_connect(l,_)
