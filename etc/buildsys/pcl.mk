#*****************************************************************************
#                 Makefile Build System for Fawkes: PCL bits
#                            -------------------
#   Created on Mon Nov 07 01:06:16 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before pcl.mk)
endif

ifndef __buildsys_pcl_mk_
__buildsys_pcl_mk_ := 1

PCL_LIB_DIRS=/usr/lib64 /usr/lib /usr/lib32 \
             /usr/local/lib64 /usr/local/lib

ifneq ($(PKGCONFIG),)
  HAVE_PCL = $(if $(shell $(PKGCONFIG) --exists 'pcl_common'; echo $${?/1/}),1,0)
  HAVE_EIGEN3 = $(if $(shell $(PKGCONFIG) --exists 'eigen3'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_EIGEN3),1)
  CFLAGS_EIGEN3  = -DHAVE_EIGEN3 $(shell $(PKGCONFIG) --cflags 'eigen3') \
		   -DEIGEN_USE_NEW_STDVECTOR \
		   -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
  LDFLAGS_EIGEN3 = $(shell $(PKGCONFIG) --libs 'eigen3')
else
  HAVE_PCL = 0
endif

ifneq ($(wildcard $(SYSROOT)/usr/include/vtk/vtkVersion.h),)
  HAVE_VTK = 1
  CFLAGS_VTK = -I/usr/include/vtk
endif

ifneq ($(HAVE_PCL),1)
  # Give it another shot, name might contain version
  _PCL_ALTERNATE_NAME=$(shell $(PKGCONFIG) pkg-config --list-all | grep pcl_common | awk '{ print $$1 }')
  ifneq ($(_PCL_ALTERNATE_NAME),)
    PCL_VERSION_SUFFIX=$(patsubst pcl_common%,%,$(_PCL_ALTERNATE_NAME))
    HAVE_PCL=1
  endif
endif

ifeq ($(HAVE_PCL),1)
  # if we have ROS, by default use std_msgs and sensor_msgs from there, rather
  # than the types that come with PCL, because it'll crash miserably when pumping
  # data from Fawkes to ROS, it might cause problems the other way around
  # ifeq ($(HAVE_ROS),1)
  #   CFLAGS_PCL  = $(call ros-pkg-cflags,sensor_msgs) \
  # 		  $(call ros-pkg-cflags,std_msgs) \
  # 		  -DHAVE_ROS_STD_MSGS
  #   LDFLAGS_PCL = $(call ros-pkg-lflags,sensor_msgs) \
  # 		  $(call ros-pkg-lflags,std_msgs) \
  # 		  -DHAVE_ROS_SENSOR_MSGS
  # endif

  CFLAGS_PCL  += -DHAVE_PCL $(CFLAGS_EIGEN3) -Wno-unknown-pragmas -Wno-deprecated-declarations \
		 $(shell $(PKGCONFIG) --cflags 'pcl_common$(PCL_VERSION_SUFFIX)')
  LDFLAGS_PCL += $(LDFLAGS_EIGEN3) \
		 $(shell $(PKGCONFIG) --libs 'pcl_common$(PCL_VERSION_SUFFIX)')

  pcl-have-lib    = $(if $(shell $(PKGCONFIG) --exists 'pcl_$1$(PCL_VERSION_SUFFIX)'; echo $${?/1/}),1,0)
  pcl-lib-cflags  = $(shell $(PKGCONFIG) --cflags 'pcl_$1$(PCL_VERSION_SUFFIX)')
  pcl-lib-ldflags = $(shell $(PKGCONFIG) --libs 'pcl_$1$(PCL_VERSION_SUFFIX)')

  pcl-have-libs    = $(if $(strip $(subst 1,,$(foreach c,$1,$(call pcl-have-lib,$c)))),0,1)
  pcl-libs-cflags  = $(shell $(PKGCONFIG) --cflags $(foreach c,$1,'pcl_$c$(PCL_VERSION_SUFFIX)' ))
  pcl-libs-ldflags = $(shell $(PKGCONFIG) --libs $(foreach c,$1,'pcl_$c$(PCL_VERSION_SUFFIX)' ))

  pcl-missing-libs = $(strip $(foreach c,$1,$(if $(subst 1,,$(call pcl-have-lib,$c)),$c)))

else
  pcl-have-lib = 0
endif

endif # __buildsys_pcl_mk_

