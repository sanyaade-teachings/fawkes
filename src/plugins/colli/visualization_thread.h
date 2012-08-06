#ifndef __PLUGINS_COLLI_VISUALIZATION_THREAD_H_
#define __PLUGINS_COLLI_VISUALIZATION_THREAD_H_

#ifndef HAVE_VISUAL_DEBUGGING
#  error ColliVisualizationThread was disabled by build flags
#endif

#include "visualization_thread_base.h"

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/tf.h>
#include <aspect/configurable.h>
#include <plugins/ros/aspect/ros.h>
#include <core/threading/mutex_locker.h>

#include <vector>
namespace ros {
  class Publisher;
}

using namespace fawkes;

class ColliVisualizationThread
: public ColliVisualizationThreadBase,
  public fawkes::Thread,
  public fawkes::TransformAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ROSAspect
{
 public:
  ColliVisualizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void visualize(const std::string &frame_id,vector<HomPoint> &cells,HomPoint &m_RoboGridPos,HomPoint &m_LaserGridPos) throw();
  virtual void visualize_occ(vector<float > &data) throw();
 private:
  fawkes::Mutex mutex_;
  std::string frame_id_;
  ros::Publisher *vispub_;
  ros::Publisher *robpub_;
  ros::Publisher *laserpub_;
  ros::Publisher *gridpub_;
  vector<HomPoint > cells_;
  vector<float > data_;
  HomPoint robo_pos_;
  HomPoint laser_pos_;
}; 

#endif
