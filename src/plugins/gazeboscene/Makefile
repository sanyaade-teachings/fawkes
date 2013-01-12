#*****************************************************************************
#    Makefile Build System for Fawkes: Gazebo Scene Reconstruction Plugin
#                            -------------------
#   Created on Mon Aug 27 09:45:40 2012
#   Author Bastian Klingen
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
include $(BASEDIR)/etc/buildsys/boost.mk
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk
include $(BASEDIR)/src/plugins/gazebo/gazebo.mk
include $(BASEDIR)/src/plugins/gazebo/protobuf.mk
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_gazeboscene = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		fawkesutils fawkeslogging fawkesmongodbaspect fawkesgazeboaspect fvutils fawkestf
OBJS_gazeboscene = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

CFLAGS  += $(CFLAGS_MONGODB) $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF) $(CFLAGS_TF) $(CFLAGS_PCL)
LDFLAGS += $(LDFLAGS_MONGODB) $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) $(LDFLAGS_TF) $(LDFLAGS_PCL) -lm $(call boost-libs-ldflags,system)

OBJS_all    = $(OBJS_gazeboscene)

REQUIRED_PCL_LIBS = sample_consensus segmentation filters surface registration

# use RRD
ifeq ($(USE_RRD),1)
  CFLAGS  += -DUSE_RRD
  LIBS_gazeboscene += fawkesrrdaspect
  WARN_TARGETS += info_rrd
endif

ifeq ($(HAVE_MONGODB)$(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(call boost-have-libs,system)$(HAVE_TF)$(HAVE_PCL),111111)
  PLUGINS_all = $(PLUGINDIR)/gazeboscene.$(SOEXT)
else
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(call boost-have-libs,system),1)
    WARN_TARGETS += warning_boost_system
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_gazebo warning_protobuf warning_boost_system warning_tf warning_pcl info_rrd
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting GazeboScene plugin$(TNORMAL) (mongodb[-devel] not installed)"
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting GazeboScene plugin$(TNORMAL) (Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting GazeboScene plugin$(TNORMAL) (protobuf[-devel] not installed)"
warning_boost_system:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting GazeboScene plugin$(TNORMAL) (boost-system not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting GazeboScene plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting GazeboScene plugin$(TNORMAL) (pcl[-devel] not installed)"
info_rrd:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TGREEN)Building GazeboScene plugin with RRD support$(TNORMAL)"

endif

include $(BUILDSYSDIR)/base.mk

