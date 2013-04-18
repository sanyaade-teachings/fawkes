#*****************************************************************************
#        Makefile Build System for Fawkes: Kinova API library
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

ifneq ($(PKGCONFIG),)
  HAVE_MONO = $(if $(shell $(PKGCONFIG) --exists 'mono-2'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_MONO),1)
    CFLAGS_MONO  = $(shell $(PKGCONFIG) --cflags 'mono-2')
    LDFLAGS_MONO = $(shell $(PKGCONFIG) --libs 'mono-2')
  endif
endif
