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
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <plugins/ros/aspect/ros.h>
#include <plugins/ros/aspect/ros_inifin.h>
#include <core/threading/mutex_locker.h>
#include <interfaces/NavigatorInterface.h>
#include <interfaces/MotorInterface.h>
#include <vector>

#include <ros/ros.h>
#include <ros/this_node.h>
#include <std_msgs/String.h>

namespace ros {
  class Publisher;
  class Subscriber;
}
#include <geometry_msgs/PoseStamped.h>

using namespace ros;
using namespace fawkes;
class ColliVisualizationThread
: public ColliVisualizationThreadBase,
  public fawkes::Thread,
  public fawkes::TransformAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ROSAspect
{
 public:
  ColliVisualizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void visualize(const std::string &frame_id,vector<HomPoint> &cells,HomPoint &m_RoboGridPos,HomPoint &m_LaserGridPos,vector<HomPoint> &laser_points,
                         vector< HomPoint > &plan,HomPoint &motor_des,int cell_width,int cell_height,HomPoint &target,
                         int grid_width, int grid_height, HomPoint &motor_real,HomPoint localTarget) throw();
  void visualize_grid_boundary();
  void visualize_path();
  void visualize_occ_cells();
  void visualize_laser_points();
  void visualize_real_motor();
  void visualize_des_motor();
  void visualize_local_target();
  HomPoint transform( HomPoint point );
  HomPoint transform_robo( HomPoint point );
  HomPoint transform_odom(HomPoint point);
  void callback( const geometry_msgs::PoseStamped::ConstPtr &msg);
 private:
  fawkes::Mutex mutex_;
  std::string frame_id_;
  ros::Publisher *vispub_;
  ros::Publisher *robpub_;
  ros::Publisher *laserpub_;
  ros::Publisher *gridpub_;
  ros::Publisher *laser_points_pub;
  ros::Publisher *pathpub_;
  ros::Publisher *targetpub_;
  ros::Publisher *tarfixpub_;
  ros::Publisher *target_real_pub_;
  ros::Publisher *target_local_pub_;

  ros::Publisher *rec1pub_;
  ros::Publisher *rec2pub_;
  ros::Publisher *rec3pub_;
  ros::Publisher *rec4pub_;

  ros::Subscriber *navsub_;
  ros::Publisher  *navpub_;

  vector<HomPoint > cells_;
  vector<HomPoint> laser_points_;
  vector<float > data_;
  vector< HomPoint > plan_;
  HomPoint robo_pos_;
  HomPoint laser_pos_;
  HomPoint target_pos_;
  HomPoint fix_target_;
  HomPoint motor_des_;
  HomPoint motor_real_;
  HomPoint local_target_;
  HomPoint rviz_target_;
  float cell_width_;
  float cell_height_;
  float grid_width_;
  float grid_height_;

  NavigatorInterface *m_navi;
  MotorInterface  *m_motor;
}; 

#endif
