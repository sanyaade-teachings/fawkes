
BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_colliManual = stdc++ fawkescore fawkesutils fawkesinterface fawkesblackboard fawkesbaseapp fawkesconfig fawkestf fawkesgeometry NavigatorInterface 
OBJS_colliManual = colliManual.o -lm  
ifeq ($(HAVE_TF),1)
  CFLAGS += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
endif
OBJS_all    = $(OBJS_colliManual)	
BINS_all = $(BINDIR)/colliManual      

include $(BUILDSYSDIR)/base.mk
	
	

							
