
BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g -O2 -Wno-unused-but-set-variable -Wno-unused-variable

LIBS_qa_kinova_api = fawkeslogging kinova stdc++
OBJS_qa_kinova_api = qa_kinova_api.o

#CFLAGS  += $(shell $(PKGCONFIG) --cflags 'mono-2')
#LDFLAGS += $(shell $(PKGCONFIG) --libs 'mono-2')

OBJS_all = 	$(OBJS_qa_kinova_api)
BINS_all = 	$(BINDIR)/qa_kinova_api

include $(BUILDSYSDIR)/base.mk
