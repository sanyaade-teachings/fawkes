
BASEDIR = ../../../

include $(BASEDIR)/etc/buildsys/config.mk	
	


LIBS_colli = fawkescore fawkesutils fawkesaspects fawkesinterface fawkesblackboard fawkesbaseapp fawkesconfig MotorInterface Laser360Interface  Laser720Interface Position2DTrackInterface NavigatorInterface  fawkesgeometry
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
             \
             robo-utils/occupancygrid/occupancygrid.o \
             robo-utils/roboshape.o \
             \
             robo-utils/rob/robo_laser.o \
             robo-utils/rob/robo_laserpoint.o \

OBJS_all    = $(OBJS_colli)      

PLUGINS_all = $(PLUGINDIR)/colli.$(SOEXT)
include $(BUILDSYSDIR)/base.mk
	
	

							
