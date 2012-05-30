#include <core/plugin.h>

#include "colli_thread.h"

using namespace fawkes;

class ColliPlugin : public fawkes::Plugin
{
  public:
    ColliPlugin(Configuration *config)
    : Plugin(config)
    {
      thread_list.push_back(new ColliThread());
    }
};

PLUGIN_DESCRIPTION("plugin for collision avoidance")
EXPORT_PLUGIN(ColliPlugin)
