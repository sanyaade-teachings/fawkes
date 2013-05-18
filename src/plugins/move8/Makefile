
#*****************************************************************************
#         Makefile Build System for Fawkes: Simple plugin to make the Robotino drive an eight using the MotorInterface
#                            -------------------
#   Created on Sat May 18 02:21:24 2013
#   Copyright (C) 2013 by Frederik Zwilling
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
#The necessary libraries your plugin needs.
LIBS_move8 = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface MotorInterface RobotinoSensorInterface
#binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
OBJS_move8 = move8_plugin.o move8_thread.o

#names of the plugins you want to build
PLUGINS_all = $(PLUGINDIR)/move8.$(SOEXT)

OBJS_all = $(OBJS_move8)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_somewarning
warning_somewarning:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino Navigator plugin$(TNORMAL) (reason)"
endif

include $(BUILDSYSDIR)/base.mk
