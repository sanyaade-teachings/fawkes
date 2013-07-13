
#*****************************************************************************
#              Makefile Build System for Fawkes: Colli Plugin
#                            -------------------
#   Created on Sat Jul 13 12:00:00 2013
#   Copyright (C) 2013 by AllemaniACs
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk
ENABLE_VISUAL_DEBUGGING = 1

LIBS_colli = fawkescore fawkesutils fawkesaspects fawkesinterface fawkesblackboard fawkesbaseapp fawkesconfig MotorInterface Laser360Interface  Laser720Interface Position2DTrackInterface NavigatorInterface  fawkesgeometry fawkestf
OBJS_colli = colli_plugin.o colli_thread.o \
             drive_realization/quadratic_motor_instruct.o \
             drive_modes/select_drive_mode.o \
             drive_modes/stop_drive_mode.o \
             drive_modes/slow_forward_drive_mode.o  \
             drive_modes/slow_backward_drive_mode.o \
             drive_modes/slow_biward_drive_mode.o \
             drive_modes/medium_forward_drive_mode.o \
             drive_modes/medium_backward_drive_mode.o  \
             drive_modes/medium_biward_drive_mode.o \
             drive_modes/fast_forward_drive_mode.o  \
             drive_modes/fast_backward_drive_mode.o \
             drive_modes/fast_biward_drive_mode.o  \
             drive_modes/escape_drive_mode.o \
             search/og_laser.o -lm\
             search/astar.o \
             search/astar_search.o \
             search/ellipse.o \
             \
             robo-utils/occupancygrid/occupancygrid.o \
             robo-utils/roboshape.o \
             \
             robo-utils/rob/robo_laser.o \
             robo-utils/rob/robo_laserpoint.o
OBJS_all    = $(OBJS_colli)
ifeq ($(HAVE_TF),1)
  CFLAGS += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(ENABLE_VISUAL_DEBUGGING),1)
 ifeq ($(HAVE_ROS),1)
  ifeq ($(call ros-have-pkg,nav_msgs,geometry_msgs,visualization_msgs,interactive_markers),1)
   CFLAGS  += $(CFLAGS_ROS)
   LDFLAGS += $(LDFLAGS_ROS)
   CFLAGS += $(shell rospack export --lang=cpp --attrib=cflags geometry_msgs)
   CFLAGS += $(shell rospack export --lang=cpp --attrib=cflags visualization_msgs)
   CFLAGS += $(shell rospack export --lang=cpp --attrib=cflags interactive_markers)
   CFLAGS_visualization_thread  = -DHAVE_VISUAL_DEBUGGING $(CFLAGS_ROS) $(CFLAGS_TF) \
					$(CFLAGS_EIGEN3) $(call ros-pkg-cflags,nav_msgs,geometry_msgs,visualization_msgs) $(shell rospack export --lang=cpp --attrib=cflags geometry_msgs) \
                                   $(shell rospack export --lang=cpp --attrib=cflags visualization_msgs) \
			           $(shell rospack export --lang=cpp --attrib=cflags interactive_markers)
   CFLAGS += -DHAVE_VISUAL_DEBUGGING
   #CFLAGS += CFLAGS_visualization_thread
   LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,nav_msgs,geometry_msgs) $(shell rospack export --lang=cpp --attrib=cflags geometry_msgs)
   LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs) $(shell rospack export --lang=cpp --attrib=cflags visualization_msgs)
   LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,interactive_markers) $(shell rospack export --lang=cpp --attrib=cflags interactive_markers)
   LIBS_colli += fawkesrosaspect
   OBJS_colli += visualization_thread_base.o visualization_thread.o
   #OBJS_colli += navigation_thread_base.o navigation_thread.o
   else
    ENABLE_VISUAL_DEBUGGING = 0
    WARN_TARGETS += warning_nav_msgs
   endif
 else
  $(warning ros is not available)$
  ENABLE_VISUAL_DEBUGGING = 0
  WARN_TARGETS += warning_ros
 endif
endif
ifneq ($(ENABLE_VISUAL_DEBUGGING),1)
  $(warning HAVE: $(call boost-have-lib,thread)  CFLAGS: $(call boost-lib-cflags,thread)  LDFLAGS: $(call boost-lib-ldflags,thread))
  CFLAGS  += $(call boost-lib-cflags,thread)
  LDFLAGS += $(call boost-lib-ldflags,thread)
endif
PLUGINS_all = $(PLUGINDIR)/colli.$(SOEXT)
ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_ros warning_nav_msgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tf$(TNORMAL) (fawkestf not available)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling visual debugging$(TNORMAL) (ROS not available)"
warning_nav_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling visual debugging$(TNORMAL) (ROS package nav_msgs not available)"
endif
include $(BUILDSYSDIR)/base.mk




