#include "visualization_thread.h"

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/this_node.h>
#include <std_msgs/String.h>

ColliVisualizationThread::ColliVisualizationThread()
  : fawkes::Thread("ColliVisualizationThread", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
  has_feedback = 0;
  feedback_id = -1;
  pose_frame_id = "";
  fixed_frame_ = "/base_link";
}
//-------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::init()
{
  ref_obstacle = false;
  if(!config->exists("/plugins/colli/RefObstacle"))
  {
      cout << "Could not find: " << "RefObstacle" << endl;
  }
  else
  {
    ref_obstacle = config->get_bool("/plugins/colli/RefObstacle");
  }
 
  if (!config->exists("/plugins/colli/Navigator_interface_id") )
  {
    cout << "***** ERROR *****: Could not get: Navigator_interface_id"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    naviface_id = config->get_string("/plugins/colli/Navigator_interface_id");
  }
  if (!config->exists("/plugins/colli/Motor_interface_id") )
  {
    cout << "***** ERROR *****: Could not get: Motor_interface_id"
         << " --> ABORTING!" << endl << endl;
    return;
  }
  else
  {
    motor_iface_id = config->get_string("/plugins/colli/Motor_interface_id");
  }
  m_motor = blackboard->open_for_reading<MotorInterface>(motor_iface_id.c_str());
  m_navi = blackboard->open_for_reading<NavigatorInterface>(naviface_id.c_str());
  p_navi = blackboard->open_for_reading<NavigatorInterface>("Pathplan");
  navsub_ = new ros::Subscriber();
  *navsub_ = rosnode->subscribe("move_base_simple/goal", 10,&ColliVisualizationThread::callback,this);
  navsub_amcl_ = new ros::Subscriber();
  *navsub_amcl_ = rosnode->subscribe("amcl_pose", 10,&ColliVisualizationThread::callback,this);
  navpub_ = new ros::Publisher();
  *navpub_ = rosnode->advertise<nav_msgs::GridCells>("target_rviz", 1);

  vispub_ = new ros::Publisher();
  *vispub_ = rosnode->advertise<nav_msgs::GridCells>("grid_cells", 1);
  gridpub_ = new ros::Publisher();
  *gridpub_ = rosnode->advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
  robpub_ = new ros::Publisher();
  *robpub_ = rosnode->advertise<nav_msgs::GridCells>("robo_cell", 1);
  laserpub_ = new ros::Publisher();
  *laserpub_ = rosnode->advertise<nav_msgs::GridCells>("laser_cell", 1);
  laser_points_pub = new ros::Publisher();
  *laser_points_pub = rosnode->advertise<nav_msgs::GridCells>("laser_points", 1);
  targetpub_ = new ros::Publisher();
  *targetpub_ = rosnode->advertise<nav_msgs::Path>("target_desired", 1);
  target_real_pub_ = new ros::Publisher();
  *target_real_pub_ = rosnode->advertise<nav_msgs::Path>("target_real", 1);
  
  target_local_pub_ = new ros::Publisher();
  *target_local_pub_ = rosnode->advertise<nav_msgs::GridCells>("target_local", 1);
  
  target_odom_pub_ = new ros::Publisher();
  *target_odom_pub_ = rosnode->advertise<nav_msgs::GridCells>("target_transformed_odom", 1);

  tarfixpub_ = new ros::Publisher();
  *tarfixpub_ = rosnode->advertise<nav_msgs::GridCells>("target_fix_cell", 1);  

  tarmpub_ = new ros::Publisher();
  *tarmpub_ = rosnode->advertise<nav_msgs::GridCells>("modified_target_cell", 1);

  pathpub_ = new ros::Publisher();
  *pathpub_ = rosnode->advertise<nav_msgs::Path>("robo_path",1); 

  pathpubcells_ = new ros::Publisher();
  *pathpubcells_ = rosnode->advertise<nav_msgs::GridCells>("robo_path_cells",1);

  rec1pub_ = new ros::Publisher();
  *rec1pub_ = rosnode->advertise<nav_msgs::Path>("rec_path1",1);
  rec2pub_ = new ros::Publisher();
  *rec2pub_ = rosnode->advertise<nav_msgs::Path>("rec_path2",1);
  rec3pub_ = new ros::Publisher();
  *rec3pub_ = rosnode->advertise<nav_msgs::Path>("rec_path3",1);
  rec4pub_ = new ros::Publisher();
  *rec4pub_ = rosnode->advertise<nav_msgs::Path>("rec_path4",1);

  orig_laserpub_ = new ros::Publisher();
  *orig_laserpub_ = rosnode->advertise<nav_msgs::GridCells>("origin_laser_points", 1);
  
  neargridpub_ = new ros::Publisher(); 
  *neargridpub_ = rosnode->advertise<nav_msgs::GridCells>("near_occupancy_grid", 1);

  fargridpub_ = new ros::Publisher();
  *fargridpub_ = rosnode->advertise<nav_msgs::GridCells>("far_occupancy_grid", 1);

  middlegridpub_ = new ros::Publisher();
  *middlegridpub_ = rosnode->advertise<nav_msgs::GridCells>("middle_occupancy_grid", 1);

  soccpub_ = new ros::Publisher();
  *soccpub_ = rosnode->advertise<nav_msgs::GridCells>("search_occ_grid", 1);
  
  found_occ_pub_ = new ros::Publisher();
  *found_occ_pub_ = rosnode->advertise<nav_msgs::GridCells>("astar_found_occ_grid", 1);

  free_grid_pub_ = new ros::Publisher();
  *free_grid_pub_ = rosnode->advertise<nav_msgs::GridCells>("free_grid_cells", 1);

  states_pub_ = new ros::Publisher();
  *states_pub_ = rosnode->advertise<nav_msgs::GridCells>("seen_states_cells", 1);
  
  drive_mode_pub_ = new ros::Publisher();
  *drive_mode_pub_ = rosnode->advertise<visualization_msgs::InteractiveMarker>("drive_mode", 100);
 
  server = new interactive_markers::InteractiveMarkerServer("colli_params_marker");
  drive_mode_sub_ = new ros::Subscriber();
  *drive_mode_sub_ = rosnode->subscribe("colli_params_marker/feedback", 100,&ColliVisualizationThread::processFeedback,this);
  visualize_colli_params();

  server_robot = new interactive_markers::InteractiveMarkerServer("robot_marker");
  robot_sub_ = new ros::Subscriber();
  *robot_sub_ = rosnode->subscribe("robot_marker/feedback", 100,&ColliVisualizationThread::robotFeedback,this);  
  visualize_robot_icon();  
}
//-----------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::finalize()
{
  vispub_->shutdown();
  delete vispub_;
  robpub_->shutdown();
  delete robpub_;
  laserpub_->shutdown();
  delete laserpub_;
  laser_points_pub->shutdown();
  delete laser_points_pub;
  pathpub_->shutdown();
  delete pathpub_;
  targetpub_->shutdown();
  delete targetpub_;
  tarfixpub_->shutdown();
  delete tarfixpub_;
  rec1pub_->shutdown();
  delete rec1pub_;
  rec2pub_->shutdown();
  delete rec2pub_;
  rec3pub_->shutdown();
  delete rec3pub_;
  rec4pub_->shutdown();
  delete rec4pub_;
  target_real_pub_->shutdown();
  delete target_real_pub_;
  target_local_pub_->shutdown();
  delete target_local_pub_;
  drive_mode_pub_->shutdown();
  delete drive_mode_pub_;
}
//-------------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize(const std::string frame_id,vector<HomPoint > cells,vector<HomPoint > near_cells,vector<HomPoint > far_cells,vector<HomPoint > middle_cells,
                                         HomPoint m_RoboGridPos,HomPoint m_LaserGridPos,
                                         vector<HomPoint> laser_points,vector< HomPoint > plan, HomPoint motor_des, 
                                         int cell_width, int cell_height, HomPoint target, int grid_width, int grid_height, 
                                         HomPoint motor_real, HomPoint localTarget, HomPoint target_odom,
                                         vector<HomPoint > orig_laser_points,vector<HomPoint > search_occ,
                                         vector<HomPoint > astar_found_occ,vector<HomPoint > free_cells,vector<HomPoint > seen_states,HomPoint modTarget) throw()
{
  MutexLocker lock(&mutex_);
  frame_id_ = frame_id;
  cells_.clear();
  cells_ = cells;
  robo_pos_ = m_RoboGridPos;
  laser_pos_ = m_LaserGridPos;
  laser_points_.clear();
  laser_points_ = laser_points;
  target_odom_ = target_odom;
  plan_.clear();
  plan_ = plan;
  cell_width_ = (float)cell_width;
  cell_height_ = (float)cell_height;
  grid_width_ = grid_width;
  grid_height_ = grid_height;
  fix_target_ = target;
  motor_des_ = motor_des;
  motor_real_ = motor_real;
  local_target_ = localTarget;
  orig_laser_points_.clear();
  orig_laser_points_ = orig_laser_points;
  near_cells_.clear();
  near_cells_ = near_cells;
  far_cells_.clear();
  far_cells_ = far_cells;
  middle_cells_.clear();
  middle_cells_ = middle_cells;
  socc_.clear();
  socc_ = search_occ;
  astar_found_occ_.clear();
  astar_found_occ_ = astar_found_occ;
  free_cells_.clear();
  free_cells_ = free_cells;
  seen_states_.clear();
  seen_states_ = seen_states;
  modTarget_ = modTarget;
  wakeup();
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::callback( const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  HomPoint transPoint = transform(robo_pos_);
  
  geometry_msgs::PoseStamped poseMsg = *msg;
  pose_frame_id = poseMsg.header.frame_id;
  nav_msgs::GridCells targ;
  targ.header.frame_id = frame_id_;
  targ.header.stamp = ros::Time::now();
  targ.cell_width = 5*cell_width_/100.;
  targ.cell_height = 5*cell_height_/100.;
  geometry_msgs::Point ptarfix;
  ptarfix.x = poseMsg.pose.position.x;
  ptarfix.y = poseMsg.pose.position.y;
  ptarfix.z = poseMsg.pose.position.z;
  targ.cells.push_back(ptarfix);
  navpub_->publish(targ);
  HomPoint rvizTarget = transform_map_to_base(poseMsg);
  rviz_target_ = rvizTarget;
  m_navi->read(); 
  p_navi->read();
  if( p_navi->has_writer() )
  {
    NavigatorInterface::CartesianGotoMessage *nav_msg = new NavigatorInterface::CartesianGotoMessage();
    nav_msg->set_x(poseMsg.pose.position.x);
    nav_msg->set_y(poseMsg.pose.position.y);
    p_navi->msgq_enqueue(nav_msg);
  }
  else
  {
    NavigatorInterface::CartesianGotoMessage *nav_msg = new NavigatorInterface::CartesianGotoMessage();
    nav_msg->set_x(rvizTarget.x());
    nav_msg->set_y(rvizTarget.y());
    m_navi->msgq_enqueue(nav_msg);
  }
  
  if( m_navi->drive_mode() == NavigatorInterface::MovingNotAllowed ) 
  {
    NavigatorInterface::SetDriveModeMessage *drive_msg = new NavigatorInterface::SetDriveModeMessage();
    drive_msg->set_drive_mode(NavigatorInterface::SlowAllowBackward);
    m_navi->msgq_enqueue(drive_msg);
  }

  NavigatorInterface::SetMaxVelocityMessage *maxvel_msg = new NavigatorInterface::SetMaxVelocityMessage();
  maxvel_msg->set_max_velocity(1.5);
  m_navi->msgq_enqueue(maxvel_msg);

/* NavigatorInterface::SetSecurityDistanceMessage *seq_msg = new NavigatorInterface::SetSecurityDistanceMessage(); 
  if( ref_obstacle )
    seq_msg->set_security_distance(0);
  else
    seq_msg->set_security_distance(0.2);
  m_navi->msgq_enqueue(seq_msg);*/
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  has_feedback = 1000;
  cout << "In rviz feedback" << endl;
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
        << feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z );
  
  feedback_id = feedback->menu_entry_id;
  if(( feedback_id <= 8 ) && ( feedback_id > 0 ))
  {
    NavigatorInterface::SetDriveModeMessage *drive_msg = new NavigatorInterface::SetDriveModeMessage();
    switch( feedback_id )
    {
      case 1:
        drive_msg->set_drive_mode(NavigatorInterface::MovingNotAllowed);
        break;
      case 2:
        drive_msg->set_drive_mode(NavigatorInterface::CarefulForward);
        break;
      case 3:
        drive_msg->set_drive_mode(NavigatorInterface::SlowForward);
        break;
      case 4:
        drive_msg->set_drive_mode(NavigatorInterface::ModerateForward);
        break;
      case 5:
        drive_msg->set_drive_mode(NavigatorInterface::FastForward);
        break;
      case 6:
        drive_msg->set_drive_mode(NavigatorInterface::CarefulAllowBackward);
        break;
      case 7:
        drive_msg->set_drive_mode(NavigatorInterface::SlowAllowBackward);
        break;
      case 8:
        drive_msg->set_drive_mode(NavigatorInterface::ModerateAllowBackward);
        break;
          
    }
    m_navi->msgq_enqueue(drive_msg);
  }
  else if(( feedback_id <= 14 ) && ( feedback_id >= 12 ))
  {
    NavigatorInterface::SetMaxVelocityMessage *maxvel_msg = new NavigatorInterface::SetMaxVelocityMessage();
    switch( feedback_id )
    {
      case 12:
        maxvel_msg->set_max_velocity(1.0);
        break;
      case 13:
        maxvel_msg->set_max_velocity(1.5);
        break;
      case 14:
        maxvel_msg->set_max_velocity(2.0);
        break;
    }
    m_navi->msgq_enqueue(maxvel_msg);
  }
  else if(( feedback_id <= 18 ) && ( feedback_id >= 16 ))
  {
    NavigatorInterface::SetSecurityDistanceMessage *seq_msg = new NavigatorInterface::SetSecurityDistanceMessage();
    switch( feedback_id )
    {
      case 16:
        seq_msg->set_security_distance(0.0);
        break;
      case 17:
        seq_msg->set_security_distance(0.1);
        break;
      case 18:
        seq_msg->set_security_distance(0.2);
        break;
    }
    m_navi->msgq_enqueue(seq_msg);
  }
  else if( ( feedback_id <= 21 ) && ( feedback_id >= 20 ))
  {
    NavigatorInterface::SetEscapingMessage *esc_msg = new NavigatorInterface::SetEscapingMessage();
    switch( feedback_id )
    {
      case 20:
        esc_msg->set_escaping_enabled(true);
        break;
      case 21:
        esc_msg->set_escaping_enabled(false);
        break;      
    }
    m_navi->msgq_enqueue(esc_msg);
  }
  else if(feedback_id == 25)
  {
    MotorInterface::ResetOdometryMessage *odom_msg = new MotorInterface::ResetOdometryMessage();
    m_motor->msgq_enqueue(odom_msg);
  }
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::robotFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  robot_marker_pose.position.x = feedback->pose.position.x;
  robot_marker_pose.position.y = feedback->pose.position.y;
  robot_marker_pose.position.z = feedback->pose.position.z;
  robot_marker_pose.orientation.x = feedback->pose.orientation.x;
  robot_marker_pose.orientation.y = feedback->pose.orientation.y;
  robot_marker_pose.orientation.z = feedback->pose.orientation.z;
  MotorInterface::SetOdometryMessage *odom_msg = new MotorInterface::SetOdometryMessage(0,0,-robot_marker_pose.orientation.x);
  m_motor->msgq_enqueue(odom_msg);
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::loop()
{
  MutexLocker lock(&mutex_);
  static int counter = 0;
  counter ++;
  if( counter == max_counter_ )
  {
    visualize_colli_params();
    visualize_robot_icon();
    counter = 0;
  }
  nav_msgs::GridCells robs;
  robs.header.frame_id = frame_id_;
  robs.header.stamp = ros::Time::now();
  robs.cell_width = cell_width_/100.;
  robs.cell_height = cell_height_/100.;
  geometry_msgs::Point p;
  HomPoint transPoint = transform_robo(robo_pos_);
  p.x = transPoint.x();
  p.y = transPoint.y();
  p.z = 0.0;
  robs.cells.push_back(p);
  robpub_->publish(robs);
  logger->log_info(name(),"rviz target point is: %f,%f\n",rviz_target_.x(),rviz_target_.y());
  char *pose_frame_cstr = new char[pose_frame_id.length()+1];
  strcpy(pose_frame_cstr,pose_frame_id.c_str());
  logger->log_info(name(),"rviz target point frame id is:%s\n",pose_frame_cstr);
  logger->log_info(name(),"transformed rviz target point is: %f,%f\n",rvizTarget.x(),rvizTarget.y());
  logger->log_info(name(),"drive mode marker is:%d , feedback_id is:%d \n",has_feedback,feedback_id);
  logger->log_info(name(), "robot icon is now at : %f,%f,%f with orientation %f,%f,%f",robot_marker_pose.position.x,robot_marker_pose.position.y,robot_marker_pose.position.z,       
                            robot_marker_pose.orientation.x,robot_marker_pose.orientation.y,robot_marker_pose.orientation.z);
  
  nav_msgs::GridCells laserc;
  laserc.header.frame_id = frame_id_;
  laserc.header.stamp = ros::Time::now();
  laserc.cell_width = cell_width_/100.;
  laserc.cell_height = cell_height_/100.;
  geometry_msgs::Point p_laser;
  HomPoint laserTrans = transform(laser_pos_);
  p_laser.x = laserTrans.x();
  p_laser.y = laserTrans.y();
  p_laser.z = 0.0;
  laserc.cells.push_back(p);
  laserpub_->publish(laserc);

  nav_msgs::GridCells targf;
  targf.header.frame_id = frame_id_;
  targf.header.stamp = ros::Time::now();
  targf.cell_width = 2*cell_width_/100.;
  targf.cell_height = 2*cell_height_/100.;
  fix_target_ = transform_robo(fix_target_);
  geometry_msgs::Point ptarfix;
  ptarfix.x = fix_target_.x();
  ptarfix.y = -fix_target_.y();
  ptarfix.z = 0.0;
  targf.cells.push_back(ptarfix);
  tarfixpub_->publish(targf);

  visualize_search_occ();
  visualize_near_cells();
  visualize_far_cells();
  visualize_middle_cells(); 
  visualize_grid_boundary();
  visualize_path();
  visualize_path_cells();
  visualize_occ_cells();
  visualize_laser_points();
  visualize_des_motor();
  visualize_real_motor();
  visualize_local_target();
  visualize_target_odom();
  visualize_orig_laser_points();
  visualize_found_astar_occ();
  visualize_free_cells();
  visualize_seen_states();
  visualize_modified_target();
}
//------------------------------------------------------------------------------
HomPoint ColliVisualizationThread::transform( HomPoint point )
{
  float cell_transw = 100. /cell_width_;
  float cell_transh = 100. /cell_height_;
  float outx = point.x() / cell_transw - (laser_pos_.x() / cell_transw);
  float outy = point.y() / cell_transh - (laser_pos_.y() / cell_transh);
  HomPoint out(outx,outy);
  return out;
}
//---------------------------------------------------------------------------------
HomPoint ColliVisualizationThread::transform_robo( HomPoint point )
{
  float cell_transw = 100. /cell_width_;
  float cell_transh = 100. /cell_height_;
  float outx = point.x() / cell_transw - ((robo_pos_.x() )/ cell_transw);
  float outy = point.y() / cell_transh - ((robo_pos_.y() )/ cell_transh);
  HomPoint out(outx,outy);
  return out;
}
//-----------------------------------------------------------------------------------
HomPoint ColliVisualizationThread::transform_odom_to_base(HomPoint point)
{
  HomPoint res;
  try {
    tf::Stamped<tf::Point> target_odom(tf::Point(point.x(),-point.y(),0.),
                   fawkes::Time(0, 0), "/odom");
    tf::Stamped<tf::Point> baserel_target;
    tf_listener->transform_point("/base_link", target_odom, baserel_target);
    HomPoint target_trans(baserel_target.x(),baserel_target.y());
    res = target_trans;
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from odom");
    e.print_trace();
  }
  return res;
}
//-----------------------------------------------------------------------------------
HomPoint ColliVisualizationThread::transform_base_to_map(HomPoint point)
{
  HomPoint res;
  try {
    tf::Stamped<tf::Point> target_base(tf::Point(point.x(),point.y(),0.),
                   fawkes::Time(0, 0), "/base_link");
    tf::Stamped<tf::Point> maprel_target;
    tf_listener->transform_point("/map", target_base,maprel_target);
    HomPoint target_trans(maprel_target.x(),maprel_target.y());
    res = target_trans;
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from base_link to map");
    e.print_trace();
  }
  return res;
}
//---------------------------------------------------------------------------------
HomPoint ColliVisualizationThread::transform_map_to_base(HomPoint point)
{
  HomPoint res;
  try {
    tf::Stamped<tf::Point> target_map(tf::Point(point.x(),point.y(),0.),
                   fawkes::Time(0, 0), "/map");
    tf::Stamped<tf::Point> baserel_target;
    tf_listener->transform_point("/base_link", target_map,baserel_target);
    HomPoint target_trans(baserel_target.x(),baserel_target.y());
    res = target_trans;
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from map to base_link");
    e.print_trace();
  }
  return res;
}
//---------------------------------------------------------------------------------
HomPoint ColliVisualizationThread::transform_map_to_base(geometry_msgs::PoseStamped poseMsg)
{
  HomPoint res;
  try {
    tf::Stamped<tf::Point> target_map(tf::Point(poseMsg.pose.position.x,poseMsg.pose.position.y,0.),
                   fawkes::Time(0, 0), poseMsg.header.frame_id);
    tf::Stamped<tf::Point> baserel_target;
    tf_listener->transform_point("/base_link", target_map,baserel_target);
    HomPoint target_trans(baserel_target.x(),baserel_target.y());
    res = target_trans;
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from map to base_link");
    e.print_trace();
  }
  return res;
}
//---------------------------------------------------------------------------------
HomPoint ColliVisualizationThread::transform_base_to_odom(HomPoint point)
{
  HomPoint res;
  try {
    tf::Stamped<tf::Point> target_base(tf::Point(point.x(),point.y(),0.),fawkes::Time(0, 0), "/base_link");
    tf::Stamped<tf::Point> odomrel_target;
    tf_listener->transform_point("/odom", target_base, odomrel_target);
    HomPoint target_trans(odomrel_target.x(),-odomrel_target.y());
    res = target_trans;
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from baselink to odometry");
    e.print_trace();
  }
  return res;  
}
//-------------------------------------------------------------------------------------
HomPoint ColliVisualizationThread::transform_laser_to_base(HomPoint point)
{
  HomPoint res;
  try {
    tf::Stamped<tf::Point> target_laser(tf::Point(point.x(),point.y(),0.),fawkes::Time(0, 0), "/base_laser");
    tf::Stamped<tf::Point> base_target;
    tf_listener->transform_point("/base_link",target_laser,base_target);
    HomPoint target_trans(base_target.x(),base_target.y());
    res = target_trans;
  } catch (tf::TransformException &e) {
    logger->log_warn(name(),"can't transform from baselink to odometry");
    e.print_trace();
  }
  return res;
}
//-------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_colli_params()
{
  visualization_msgs::InteractiveMarker dmode;
  dmode.header.frame_id = fixed_frame_;
  dmode.header.stamp = ros::Time::now();
  dmode.name = "Colli Parameters Selection";
  dmode.description = "Colli Parameters Selection";
  dmode.pose.position.x = 3.5;
  dmode.pose.position.y = 0.0;
  dmode.pose.position.z = 0.0;
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 1.0;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.0;
  box_marker.color.a = 1.0;
  box_marker.lifetime = ros::Duration(120, 0);
  
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  box_control.name = "menu";
  box_control.description = "Colli Parameters Menu"; 
  box_control.markers.push_back( box_marker );
  dmode.controls.push_back( box_control );


  vector<string > drive_modes_vec;
  drive_modes_vec.push_back("MovingNotAllowed");
  drive_modes_vec.push_back("CarefulForward");
  drive_modes_vec.push_back("SlowForward"); 
  drive_modes_vec.push_back("ModerateForward");
  drive_modes_vec.push_back("FastForward");
  drive_modes_vec.push_back("CarefulAllowBackward");
  drive_modes_vec.push_back("SlowAllowBackward");
  drive_modes_vec.push_back("ModerateAllowBackward");
  drive_modes_vec.push_back("FastAllowBackward");
  drive_modes_vec.push_back("CarefulBackward");
  drive_modes_vec.push_back("SlowBackward");
  drive_modes_vec.push_back("ModerateBackward");
  drive_modes_vec.push_back("FastBackward");
  drive_modes_vec.push_back("ESCAPE");

  visualization_msgs::MenuEntry menuDrive;
  menuDrive.title = "Drive Modes->";
  menuDrive.command_type = visualization_msgs::MenuEntry::FEEDBACK;
  menuDrive.id = 10;
  menuDrive.parent_id = 0;
  dmode.menu_entries.push_back(menuDrive);


  visualization_msgs::MenuEntry menues[8]; 
  for( size_t i = 0; i < 8; i++ )
  {
    menues[i].title = drive_modes_vec[i];
    menues[i].command_type = visualization_msgs::MenuEntry::FEEDBACK;
    menues[i].id = i+1;
    menues[i].parent_id = menuDrive.id;
    dmode.menu_entries.push_back(menues[i]);
  }

  visualization_msgs::MenuEntry menuVel;
  menuVel.title = "Maximum Velocity->";
  menuVel.command_type = visualization_msgs::MenuEntry::FEEDBACK;
  menuVel.id = 11;
  menuVel.parent_id = 0;
  dmode.menu_entries.push_back(menuVel);

  visualization_msgs::MenuEntry menuesVel[3];
  float min_vel = 1.0;
  for( size_t i = 0; i < 3; i++ )
  {
    float cur_vel = min_vel + ((float) i )* (0.5);
    stringstream ss;
    ss << cur_vel;
    menuesVel[i].title = ss.str();
    menuesVel[i].command_type = visualization_msgs::MenuEntry::FEEDBACK;
    menuesVel[i].id = menuVel.id + i + 1;
    menuesVel[i].parent_id = menuVel.id;
    dmode.menu_entries.push_back(menuesVel[i]);
  }

  visualization_msgs::MenuEntry menuSecDis;
  menuSecDis.title = "Security Distance->";
  menuSecDis.command_type = visualization_msgs::MenuEntry::FEEDBACK;
  menuSecDis.id = 15;
  menuSecDis.parent_id = 0;
  dmode.menu_entries.push_back(menuSecDis);

  visualization_msgs::MenuEntry menuesSecDis[3];
  float min_dis = 0.0;
  for( size_t i = 0; i < 3; i++ )
  {
    float cur_dis = min_dis + ((float) i )* (0.1);
    stringstream ss;
    ss << cur_dis;
    menuesSecDis[i].title = ss.str();
    menuesSecDis[i].command_type = visualization_msgs::MenuEntry::FEEDBACK;
    menuesSecDis[i].id = menuSecDis.id + i + 1;
    menuesSecDis[i].parent_id = menuSecDis.id;
    dmode.menu_entries.push_back(menuesSecDis[i]);
  }

  visualization_msgs::MenuEntry menuEsc;
  menuEsc.title = "Escape Enable->";
  menuEsc.command_type = visualization_msgs::MenuEntry::FEEDBACK;
  menuEsc.id = 19;
  menuEsc.parent_id = 0;
  dmode.menu_entries.push_back(menuEsc);
  visualization_msgs::MenuEntry menuesEscMode[2];
  menuesEscMode[0].title = "true";
  menuesEscMode[0].command_type = visualization_msgs::MenuEntry::FEEDBACK;
  menuesEscMode[0].id = menuEsc.id + 1;
  menuesEscMode[0].parent_id = menuEsc.id;
  dmode.menu_entries.push_back(menuesEscMode[0]);
  menuesEscMode[1].title = "false";
  menuesEscMode[1].command_type = visualization_msgs::MenuEntry::FEEDBACK;
  menuesEscMode[1].id = menuEsc.id + 2;
  menuesEscMode[1].parent_id = menuEsc.id;
  dmode.menu_entries.push_back(menuesEscMode[1]);

  visualization_msgs::MenuEntry menuRes;
  menuRes.title = "Reset odom";
  menuRes.command_type = visualization_msgs::MenuEntry::FEEDBACK;
  menuRes.id = 25;
  menuRes.parent_id = 0;
  dmode.menu_entries.push_back(menuRes);

  server->insert(dmode);
  server->applyChanges();
}
//--------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_robot_icon()
{
  visualization_msgs::InteractiveMarker dmode;
  dmode.header.frame_id = fixed_frame_;
  dmode.header.stamp = ros::Time::now();
  dmode.name = "Robot";
  dmode.description = "Robot Icon";
  HomPoint transPoint = transform_robo(robo_pos_);
  dmode.pose.position.x = transPoint.x();
  dmode.pose.position.y = transPoint.y();
  dmode.pose.position.z = 0.0;
  visualization_msgs::Marker robot_marker;
  robot_marker.type = visualization_msgs::Marker::CYLINDER;
  robot_marker.scale.x = 0.2;
  robot_marker.scale.y = 0.2;
  robot_marker.scale.z = 0.2;
  robot_marker.color.r = 1.0;
  robot_marker.color.g = 0.0;
  robot_marker.color.b = 0.0;
  robot_marker.color.a = 1.0;
  robot_marker.lifetime = ros::Duration(120, 0);

  visualization_msgs::InteractiveMarkerControl robot_control;
  robot_control.always_visible = true;
  robot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  robot_control.name = "rotate";
  robot_control.description = "robot odometry rotate";
  robot_control.markers.push_back( robot_marker );
  dmode.controls.push_back( robot_control );

  server_robot->insert(dmode);
  server_robot->applyChanges();   
}
//---------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_modified_target()
{
  nav_msgs::GridCells targm;
  targm.header.frame_id = frame_id_;
  targm.header.stamp = ros::Time::now();
  targm.cell_width = 2*cell_width_/100.;
  targm.cell_height = 2*cell_height_/100.;
  modTarget_ = transform_robo(modTarget_);
  geometry_msgs::Point ptarm;
  ptarm.x = modTarget_.x();
  ptarm.y = -modTarget_.y();
  ptarm.z = 0.0;
  targm.cells.push_back(ptarm);
  tarmpub_->publish(targm);
}
//---------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_target_odom()
{
  HomPoint target_trans = transform_odom_to_base(target_odom_);
  nav_msgs::GridCells targl;
  targl.header.frame_id = frame_id_;
  targl.header.stamp = ros::Time::now();
  targl.cell_width = 2*cell_width_/100.;
  targl.cell_height = 2*cell_height_/100.;
  geometry_msgs::Point ptarloc;
  ptarloc.x = target_trans.x();
  ptarloc.y = target_trans.y();
  //ptarloc.y = -target_trans.y();
  ptarloc.z = 0.0;
  targl.cells.push_back(ptarloc);
  target_odom_pub_->publish(targl);
    
}
//-----------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_local_target()
{
  nav_msgs::GridCells targl;
  targl.header.frame_id = frame_id_;
  targl.header.stamp = ros::Time::now();
  targl.cell_width = 2*cell_width_/100.;
  targl.cell_height = 2*cell_height_/100.;
  //local_target_ = transform_odom_to_base(local_target_);
  geometry_msgs::Point ptarloc;
  ptarloc.x = local_target_.x();
  ptarloc.y = -local_target_.y();
  ptarloc.z = 0.0;
  targl.cells.push_back(ptarloc);
  target_local_pub_->publish(targl);
}
//----------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_real_motor()
{
  nav_msgs::Path targc;
  targc.header.frame_id = frame_id_;
  targc.header.stamp = ros::Time::now();
  geometry_msgs::Quaternion pathOri;
  pathOri.x = 0.0;
  pathOri.y = 0.0;
  pathOri.z = 0.0;
  pathOri.w = 0.0;
  geometry_msgs::PoseStamped pos;
  pos.header.stamp = ros::Time::now();
  pos.header.frame_id = frame_id_;
  geometry_msgs::Point pathPoint;
  HomPoint transPoint = transform(robo_pos_);
  pathPoint.x = transPoint.x();
  pathPoint.y = transPoint.y();
  pathPoint.z = 0.0;
  pos.pose.position = pathPoint;
  pos.pose.orientation = pathOri;
  targc.poses.push_back(pos);
  pathPoint.x += motor_real_.x();
  pathPoint.y += motor_real_.y();
  pathPoint.z = 0.0;
  pos.pose.position = pathPoint;
  pos.pose.orientation = pathOri;
  targc.poses.push_back(pos);
  target_real_pub_->publish(targc);
}
//-----------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_des_motor()
{
  nav_msgs::Path targc;
  targc.header.frame_id = frame_id_;
  targc.header.stamp = ros::Time::now();
  geometry_msgs::Quaternion pathOri;
  pathOri.x = 0.0;
  pathOri.y = 0.0;
  pathOri.z = 0.0;
  pathOri.w = 0.0;
  geometry_msgs::PoseStamped pos;
  pos.header.stamp = ros::Time::now();
  pos.header.frame_id = frame_id_;
  geometry_msgs::Point pathPoint;
  HomPoint transPoint = transform(robo_pos_);
  pathPoint.x = transPoint.x();
  pathPoint.y = transPoint.y();
  pathPoint.z = 0.0;
  pos.pose.position = pathPoint;
  pos.pose.orientation = pathOri;
  targc.poses.push_back(pos);
  pathPoint.x += motor_des_.x();
  pathPoint.y += motor_des_.y();
  pathPoint.z = 0.0;
  pos.pose.position = pathPoint;
  pos.pose.orientation = pathOri;
  targc.poses.push_back(pos);
  targetpub_->publish(targc);

}
//------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_laser_points()
{
  nav_msgs::GridCells lgs;
  lgs.header.frame_id = frame_id_;
  lgs.header.stamp = ros::Time::now();
  lgs.cell_width = cell_width_/100.;
  lgs.cell_height = cell_height_/100.;
  for( size_t l = 0; l < laser_points_.size(); l++ )
  {
    geometry_msgs::Point p;
    p.x = laser_points_[l].x();
    p.y = laser_points_[l].y();
    p.z = 0.0;
    lgs.cells.push_back(p);
  }
  laser_points_pub->publish(lgs);

}
//--------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_orig_laser_points()
{
  nav_msgs::GridCells lgs;
  lgs.header.frame_id = frame_id_;
  lgs.header.stamp = ros::Time::now();
  lgs.cell_width = cell_width_/100.;
  lgs.cell_height = cell_height_/100.;
  for( unsigned int l = 0; l < orig_laser_points_.size(); l++ )
  {
    //HomPoint base_point = orig_laser_points_[l];
    HomPoint base_point = transform_laser_to_base(orig_laser_points_[l]);
    geometry_msgs::Point p;
    p.x = base_point.x();
    p.y = base_point.y();
    p.z = 0.0;
    lgs.cells.push_back(p);
  }
  cout << endl;
  orig_laserpub_->publish(lgs);
}

//------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_occ_cells()
{
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < cells_.size(); i++ )
  {
    geometry_msgs::Point p;
    HomPoint transPoint = transform(cells_[i]);
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  vispub_->publish(gcs);
}
//----------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_near_cells()
{

  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < near_cells_.size(); i++ )
  {
    geometry_msgs::Point p;
    //HomPoint transPoint = transform_robo(near_cells_[i]);
    HomPoint transPoint = transform(near_cells_[i]);    
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  neargridpub_->publish(gcs);
}
//-------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_far_cells()
{
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < far_cells_.size(); i++ )
  {
    geometry_msgs::Point p;
    //HomPoint transPoint = transform_robo(far_cells_[i]);
    HomPoint transPoint = transform(far_cells_[i]);
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  fargridpub_->publish(gcs);
}
//--------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_middle_cells()
{
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < middle_cells_.size(); i++ )
  {
    geometry_msgs::Point p;
  //  HomPoint transPoint = transform_robo(middle_cells_[i]);
    HomPoint transPoint = transform(middle_cells_[i]);
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  middlegridpub_->publish(gcs);
}
//--------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_search_occ()
{
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < socc_.size(); i++ )
  {
    geometry_msgs::Point p;
    HomPoint transPoint = transform_robo(socc_[i]);
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  soccpub_->publish(gcs);
}

//-------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_found_astar_occ()
{
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < astar_found_occ_.size(); i++ )
  {
    geometry_msgs::Point p;
    //HomPoint transPoint = transform_robo(astar_found_occ_[i]);
    HomPoint transPoint = transform(astar_found_occ_[i]);
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  found_occ_pub_->publish(gcs);
}

//------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_free_cells()
{
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < free_cells_.size(); i++ )
  {
    geometry_msgs::Point p;
   // HomPoint transPoint = transform_robo(free_cells_[i]);
    HomPoint transPoint = transform(free_cells_[i]);
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  free_grid_pub_->publish(gcs);
}
//--------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_seen_states()
{
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < seen_states_.size(); i++ )
  {
    geometry_msgs::Point p;
   // HomPoint transPoint = transform_robo(seen_states_[i]);
    HomPoint transPoint = transform(seen_states_[i]);
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  states_pub_->publish(gcs);
}
//-------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_path()
{
  nav_msgs::Path robpath;
  robpath.header.frame_id = frame_id_;
  robpath.header.stamp = ros::Time::now();
  for( size_t j = 0; j < plan_.size(); j++ )
  {
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = frame_id_;
    geometry_msgs::Point pathPoint;
    HomPoint transPoint = transform_robo(plan_[j]);
    pathPoint.x = transPoint.x();
    pathPoint.y = -transPoint.y();
    pathPoint.z = 0.0;
    p.pose.position = pathPoint;
    geometry_msgs::Quaternion pathOri;
    pathOri.x = 0.0;
    pathOri.y = 0.0;
    pathOri.z = 0.0;
    pathOri.w = 0.0;
    p.pose.orientation = pathOri;
    robpath.poses.push_back(p);
  }
  pathpub_->publish(robpath);
}
//--------------------------------------------------------------------------
void ColliVisualizationThread::visualize_path_cells()
{
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now();
  gcs.cell_width = cell_width_/100.;
  gcs.cell_height = cell_height_/100.;
  for( size_t i = 0; i < plan_.size(); i++ )
  {
    geometry_msgs::Point p;
    HomPoint transPoint = transform_robo(plan_[i]);
    p.x = transPoint.x();
    p.y = -transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  pathpubcells_->publish(gcs);
 
}
//----------------------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_grid_boundary()
{
  HomPoint transPoint = transform(robo_pos_);
  float robo_posx = transPoint.x();
  float robo_posy = transPoint.y(); 
  nav_msgs::Path targc1;
  targc1.header.frame_id = frame_id_;
  targc1.header.stamp = ros::Time::now();
  geometry_msgs::Quaternion pathOri;
  pathOri.x = 0.0;
  pathOri.y = 0.0;
  pathOri.z = 0.0;
  pathOri.w = 0.0;
  geometry_msgs::PoseStamped pos;
  pos.header.stamp = ros::Time::now();
  pos.header.frame_id = frame_id_;
  geometry_msgs::Point pathPoint1;
  geometry_msgs::Point pathPoint2;
  pathPoint1.x = robo_posx - grid_width_ / 2.;
  pathPoint1.y = robo_posy - grid_height_ / 2.;
  pathPoint1.z = 0.0;
  pos.pose.position = pathPoint1;
  pos.pose.orientation = pathOri;
  targc1.poses.push_back(pos);
  pathPoint2.x = robo_posx + grid_width_ / 2.;
  pathPoint2.y = robo_posy - grid_height_ / 2.;
  pathPoint2.z = 0.0;
  pos.pose.position = pathPoint2;
  pos.pose.orientation = pathOri;
  targc1.poses.push_back(pos);
  rec1pub_->publish(targc1);

  geometry_msgs::Point pathPoint3;
  pathPoint3.x = robo_posx - grid_width_ / 2.;
  pathPoint3.y = robo_posy + grid_height_ / 2.;
  pathPoint3.z = 0.0;
  geometry_msgs::Point pathPoint4;
  pathPoint4.x = robo_posx + grid_width_ / 2.;
  pathPoint4.y = robo_posy + grid_height_ / 2.;
  pathPoint4.z = 0.0;

  nav_msgs::Path targc2;
  targc2.header.frame_id = frame_id_;
  targc2.header.stamp = ros::Time::now();
  nav_msgs::Path targc3;
  targc3.header.frame_id = frame_id_;
  targc3.header.stamp = ros::Time::now();
  nav_msgs::Path targc4;
  targc4.header.frame_id = frame_id_;
  targc4.header.stamp = ros::Time::now();

  pos.pose.position = pathPoint1;
  pos.pose.orientation = pathOri;
  targc2.poses.push_back(pos);
  pos.pose.position = pathPoint3;
  pos.pose.orientation = pathOri;
  targc2.poses.push_back(pos);
  rec2pub_->publish(targc2);  // ** between 1-3

  pos.pose.position = pathPoint2;
  pos.pose.orientation = pathOri;
  targc3.poses.push_back(pos);
  pos.pose.position = pathPoint4;
  pos.pose.orientation = pathOri;
  targc3.poses.push_back(pos);
  rec3pub_->publish(targc3);  // ** between 2-4

  pos.pose.position = pathPoint3;
  pos.pose.orientation = pathOri;
  targc4.poses.push_back(pos);
  pos.pose.position = pathPoint4;
  pos.pose.orientation = pathOri;
  targc4.poses.push_back(pos);
  rec4pub_->publish(targc4);  // ** between 1-3
}
