
/***************************************************************************
 *  viaualization_thread.h - Visualization Thread
 *
 *  Created: Sat Jul 13 12:00:00 2013
 *  Copyright  2013  AllemaniACs
 *
 ****************************************************************************/

#ifndef __PLUGINS_COLLI_VISUALIZATION_THREAD_H_
#define __PLUGINS_COLLI_VISUALIZATION_THREAD_H_

#ifndef HAVE_VISUAL_DEBUGGING
#  error ColliVisualizationThread was disabled by build flags
#endif

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <aspect/tf.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <plugins/ros/aspect/ros.h>
#include <plugins/ros/aspect/ros_inifin.h>
#include <interfaces/NavigatorInterface.h>
#include <interfaces/MotorInterface.h>
#include <geometry/hom_point.h>

#include <ros/ros.h>
#include <ros/this_node.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <vector>
#include <string>

namespace ros {
  class Publisher;
  class Subscriber;
}

class ColliVisualizationThread
: public fawkes::Thread,
  public fawkes::TransformAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ROSAspect
{
 public:
  const static int max_counter_ = 100;

  ColliVisualizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void visualize(const std::string frame_id,
			 std::vector<fawkes::HomPoint> cells,
			 std::vector<fawkes::HomPoint> near_cells,
			 std::vector<fawkes::HomPoint> far_cells,
			 std::vector<fawkes::HomPoint> middle_cells,
                         fawkes::HomPoint m_RoboGridPos,
			 fawkes::HomPoint m_LaserGridPos,
			 std::vector<fawkes::HomPoint> laser_points,
                         std::vector<fawkes::HomPoint> plan,
			 fawkes::HomPoint motor_des,
			 int cell_width, int cell_height, fawkes::HomPoint target,
                         int grid_width, int grid_height, fawkes::HomPoint motor_real,
			 fawkes::HomPoint localTarget, fawkes::HomPoint target_odom,
                         std::vector<fawkes::HomPoint> orig_laser_points,
			 std::vector<fawkes::HomPoint> search_occ,
                         std::vector<fawkes::HomPoint> astar_found_occ,
			 std::vector<fawkes::HomPoint> free_cells,
			 std::vector<fawkes::HomPoint> seen_states,
			 fawkes::HomPoint modTarget) throw();

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
  void visualize_robot_icon();
  void visualize_modified_target();
  fawkes::HomPoint transform( fawkes::HomPoint point );
  fawkes::HomPoint transform_robo( fawkes::HomPoint point );
  fawkes::HomPoint transform_odom_to_base(fawkes::HomPoint point);
  fawkes::HomPoint transform_base_to_odom(fawkes::HomPoint point);
  fawkes::HomPoint transform_laser_to_base(fawkes::HomPoint point);
  fawkes::HomPoint transform_base_to_map(fawkes::HomPoint point);
  fawkes::HomPoint transform_map_to_base(fawkes::HomPoint point);
  fawkes::HomPoint transform_map_to_base(geometry_msgs::PoseStamped poseMsg);
  void visualize_target_odom();
  void callback( const geometry_msgs::PoseStamped::ConstPtr &msg);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void robotFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
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
  interactive_markers::InteractiveMarkerServer *server_robot;
  ros::Subscriber *robot_sub_;
  ros::Subscriber *drive_mode_sub_;

  std::vector<fawkes::HomPoint > cells_;
  std::vector<fawkes::HomPoint > near_cells_;
  std::vector<fawkes::HomPoint > far_cells_;
  std::vector<fawkes::HomPoint > middle_cells_;
  std::vector<fawkes::HomPoint> laser_points_;
  std::vector<fawkes::HomPoint > orig_laser_points_;
  std::vector<fawkes::HomPoint > socc_;
  std::vector<float > data_;
  std::vector< fawkes::HomPoint > plan_;
  std::vector<fawkes::HomPoint > astar_found_occ_;
  std::vector<fawkes::HomPoint > free_cells_;
  std::vector<fawkes::HomPoint > seen_states_;

  fawkes::HomPoint robo_pos_;
  fawkes::HomPoint laser_pos_;
  fawkes::HomPoint target_pos_;
  fawkes::HomPoint fix_target_;
  fawkes::HomPoint target_odom_;
  fawkes::HomPoint motor_des_;
  fawkes::HomPoint motor_real_;
  fawkes::HomPoint local_target_;
  fawkes::HomPoint rviz_target_;
  fawkes::HomPoint rvizTarget;
  fawkes::HomPoint modTarget_;
  float odomx_;;
  float odomy_;
  float odomori_;
  float cell_width_;
  float cell_height_;
  float grid_width_;
  float grid_height_;

  int has_feedback;
  int feedback_id;
  std::string pose_frame_id;
  fawkes::NavigatorInterface *m_navi;
  fawkes::NavigatorInterface *p_navi;
  fawkes::MotorInterface  *m_motor;
  bool ref_obstacle;
  std::string motor_iface_id;
  std::string naviface_id;
  geometry_msgs::Pose robot_marker_pose;
  bool frame_valid_;
  std::string fixed_frame_;
};

#endif
