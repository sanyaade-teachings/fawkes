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
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

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

  virtual void visualize(const std::string frame_id,vector<HomPoint> cells,vector<HomPoint > near_cells,vector<HomPoint > far_cells,vector<HomPoint > middle_cells,
                         HomPoint m_RoboGridPos,HomPoint m_LaserGridPos,vector<HomPoint> laser_points,
                         vector< HomPoint > plan,HomPoint motor_des,int cell_width,int cell_height,HomPoint target,
                         int grid_width, int grid_height, HomPoint motor_real,HomPoint localTarget,HomPoint target_odom,
                         vector<HomPoint > orig_laser_points,vector<HomPoint > search_occ,
                         vector<HomPoint > astar_found_occ,vector<HomPoint > free_cells,vector<HomPoint > seen_states,HomPoint modTarget) throw();

  void visualize_grid_boundary();
  void visualize_path();
  void visualize_path_cells();
  void visualize_occ_cells();
  void visualize_near_cells();
  void visualize_far_cells();
  void visualize_middle_cells();
  void visualize_laser_points();
  void visualize_real_motor();
  void visualize_des_motor();
  void visualize_local_target();
  void visualize_orig_laser_points();
  void visualize_search_occ();
  void visualize_found_astar_occ();
  void visualize_free_cells();
  void visualize_seen_states();
  void visualize_colli_params();
  void visualize_modified_target();
  HomPoint transform( HomPoint point );
  HomPoint transform_robo( HomPoint point );
  HomPoint transform_odom_to_base(HomPoint point);
  HomPoint transform_base_to_odom(HomPoint point);
  HomPoint transform_laser_to_base(HomPoint point);
  HomPoint transform_base_to_map(HomPoint point);
  HomPoint transform_map_to_base(HomPoint point);
  HomPoint transform_map_to_base(geometry_msgs::PoseStamped poseMsg);
  void visualize_target_odom();
  void callback( const geometry_msgs::PoseStamped::ConstPtr &msg);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
 private:
  fawkes::Mutex mutex_;
  std::string frame_id_;
  ros::Publisher *vispub_;
  ros::Publisher *robpub_;
  ros::Publisher *laserpub_;
  ros::Publisher *gridpub_;
  ros::Publisher *neargridpub_;
  ros::Publisher *fargridpub_;
  ros::Publisher *middlegridpub_;
  ros::Publisher *laser_points_pub;
  ros::Publisher *pathpub_;
  ros::Publisher *pathpubcells_;
  ros::Publisher *targetpub_;
  ros::Publisher *tarfixpub_;
  ros::Publisher *tarmpub_;
  ros::Publisher *target_real_pub_;
  ros::Publisher *target_local_pub_;
  ros::Publisher *target_odom_pub_;
  ros::Publisher *orig_laserpub_;
  ros::Publisher *soccpub_;
  ros::Publisher *found_occ_pub_;
  ros::Publisher *free_grid_pub_;
  ros::Publisher *states_pub_;

  ros::Publisher *rec1pub_;
  ros::Publisher *rec2pub_;
  ros::Publisher *rec3pub_;
  ros::Publisher *rec4pub_;

  ros::Subscriber *navsub_;
  ros::Subscriber *navsub_amcl_;
  ros::Publisher  *navpub_;
 
  ros::Publisher *drive_mode_pub_;
  interactive_markers::InteractiveMarkerServer *server;
  ros::Subscriber *drive_mode_sub_;

  vector<HomPoint > cells_;
  vector<HomPoint > near_cells_;
  vector<HomPoint > far_cells_;
  vector<HomPoint > middle_cells_;
  vector<HomPoint> laser_points_;
  vector<HomPoint > orig_laser_points_;
  vector<HomPoint > socc_;
  vector<float > data_;
  vector< HomPoint > plan_;
  vector<HomPoint > astar_found_occ_;
  vector<HomPoint > free_cells_;
  vector<HomPoint > seen_states_;

  HomPoint robo_pos_;
  HomPoint laser_pos_;
  HomPoint target_pos_;
  HomPoint fix_target_;
  HomPoint target_odom_;
  HomPoint motor_des_;
  HomPoint motor_real_;
  HomPoint local_target_;
  HomPoint rviz_target_;
  HomPoint rvizTarget;
  HomPoint modTarget_;
  float odomx_;;
  float odomy_;
  float odomori_;
  float cell_width_;
  float cell_height_;
  float grid_width_;
  float grid_height_;

  int has_feedback;
  int feedback_id;
  string pose_frame_id;
  NavigatorInterface *m_navi;
  NavigatorInterface *p_navi;
  MotorInterface  *m_motor;
  bool ref_obstacle;
  string naviface_id;
}; 

#endif
