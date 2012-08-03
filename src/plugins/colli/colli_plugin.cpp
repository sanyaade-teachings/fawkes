#include <core/plugin.h>

#include "colli_thread.h"
#ifdef HAVE_VISUAL_DEBUGGING
#include "visualization_thread.h"
#endif
using namespace fawkes;

class ColliPlugin : public fawkes::Plugin
{
  public:
    ColliPlugin(Configuration *config)
    : Plugin(config)
    {
     // thread_list.push_back(new ColliThread());
      ColliThread *collithr = new ColliThread();
      thread_list.push_back(collithr);
      #ifdef HAVE_VISUAL_DEBUGGING
      ColliVisualizationThread *visthr = new ColliVisualizationThread();
      collithr->set_visualization_thread(visthr);
      thread_list.push_back(visthr);
      #endif

    }
};

PLUGIN_DESCRIPTION("plugin for collision avoidance")
EXPORT_PLUGIN(ColliPlugin)
