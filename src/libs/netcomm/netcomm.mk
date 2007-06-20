#*****************************************************************************
#               Makefile Build System for Fawkes: NetComm Library
#                            -------------------
#   Created on Tue Nov 07 16:43:45 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

include $(BASEDIR)/etc/buildsys/config.mk
ifneq ($(PKGCONFIG),)
  HAVE_AVAHI     := $(if $(shell $(PKGCONFIG) --print-errors --errors-to-stdout --exists 'avahi-client'),0,1)
  HAVE_LIBCRYPTO := $(if $(shell $(PKGCONFIG) --print-errors --errors-to-stdout --exists 'libcrypto'),0,1)
endif
ifeq ($(HAVE_AVAHI),1)
  CFLAGS += -DHAVE_AVAHI
endif
ifeq ($(HAVE_LIBCRYPTO),1)
  CFLAGS += -DHAVE_LIBCRYPTO
endif
