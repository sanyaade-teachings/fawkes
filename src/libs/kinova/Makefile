#*****************************************************************************
#          Makefile for Fawkes Kinova API library
#                            -------------------
#   Created on Tue Apr 16 12:45:11 2013
#   Copyright (C) 2013 by Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

LIBS_libkinova = fawkescore
OBJS_libkinova = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))

OBJS_all = $(OBJS_libkinova)

ifeq ($(HAVE_MONO),1)
  CFLAGS  += $(CFLAGS_MONO)
  LDFLAGS += $(LDFLAGS_MONO)
  LIBS_all = $(LIBDIR)/libkinova.so
else
  WARN_TARGETS += warning_mono
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mono
warning_mono:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Kinova API library$(TNORMAL) (mono-2 not installed)"
endif

include $(BUILDSYSDIR)/base.mk
