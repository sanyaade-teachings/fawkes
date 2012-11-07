#include "visualization_thread.h"

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/this_node.h>
#include <std_msgs/String.h>

ColliVisualizationThread::ColliVisualizationThread()
  : fawkes::Thread("ColliVisualizationThread", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
}


//-------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::init()
{ 
  m_motor = blackboard->open_for_reading<MotorInterface>("Motor Brutus");
  m_navi = blackboard->open_for_reading<NavigatorInterface>("NavigatorTarget");
  navsub_ = new ros::Subscriber();
//  *navsub_ = rosnode->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10,&ColliVisualizationThread::callback,this);
  *navsub_ = rosnode->subscribe("move_base_simple/goal", 10,&ColliVisualizationThread::callback,this);

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
 // *targetpub_ = rosnode->advertise<nav_msgs::GridCells>("target_cell", 1); 
  *targetpub_ = rosnode->advertise<nav_msgs::Path>("target_desired", 1);
  target_real_pub_ = new ros::Publisher();
  *target_real_pub_ = rosnode->advertise<nav_msgs::Path>("target_real", 1);
  
  target_local_pub_ = new ros::Publisher();
  *target_local_pub_ = rosnode->advertise<nav_msgs::GridCells>("target_local", 1);
   
  tarfixpub_ = new ros::Publisher();
  *tarfixpub_ = rosnode->advertise<nav_msgs::GridCells>("target_fix_cell", 1);  
  pathpub_ = new ros::Publisher();
  *pathpub_ = rosnode->advertise<nav_msgs::Path>("robo_path",1); 

  rec1pub_ = new ros::Publisher();
  *rec1pub_ = rosnode->advertise<nav_msgs::Path>("rec_path1",1);
  rec2pub_ = new ros::Publisher();
  *rec2pub_ = rosnode->advertise<nav_msgs::Path>("rec_path2",1);
  rec3pub_ = new ros::Publisher();
  *rec3pub_ = rosnode->advertise<nav_msgs::Path>("rec_path3",1);
  rec4pub_ = new ros::Publisher();
  *rec4pub_ = rosnode->advertise<nav_msgs::Path>("rec_path4",1);
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

}
//-------------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize(const std::string &frame_id,vector<HomPoint > &cells,HomPoint &m_RoboGridPos,HomPoint &m_LaserGridPos,
                                         vector<HomPoint> &laser_points,vector< HomPoint > &plan, HomPoint &motor_des, 
                                         int cell_width, int cell_height, HomPoint &target, int grid_width, int grid_height, HomPoint &motor_real, HomPoint localTarget) throw()
{
  MutexLocker lock(&mutex_);
  frame_id_ = frame_id;
  cells_.clear();
  cells_ = cells;
  robo_pos_ = m_RoboGridPos;
  laser_pos_ = m_LaserGridPos;
  laser_points_.clear();
  laser_points_ = laser_points;
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
  wakeup();
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::callback( const geometry_msgs::PoseStamped::ConstPtr &msg)
{
//  logger->log_info(name(),"message recieved");
  HomPoint transPoint = transform(robo_pos_);

  geometry_msgs::PoseStamped poseMsg = *msg;
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
  HomPoint rvizTarget(poseMsg.pose.position.x,poseMsg.pose.position.y,0);
 // rviz_target_ = rvizTarget;


 // float dist = sqrt(pow(rvizTarget.x(),2)+pow(rvizTarget.y(),2)); 
//  float phi = atan2(rvizTarget.y(),rvizTarget.x());


  m_motor->read();
  //float transx = m_motor->odometry_position_x() + rvizTarget.x();
  float transx = rvizTarget.x();
  //float transy = m_motor->odometry_position_y() - rvizTarget.y();
  float transy = rvizTarget.y();
  float tx = ( transx*cos( m_motor->odometry_orientation ()  ) + transy*sin( m_motor->odometry_orientation () ) );
  float ty = ( transy*cos( m_motor->odometry_orientation ()  ) - transx*sin( m_motor->odometry_orientation () ) );
  HomPoint base_target = HomPoint(tx+m_motor->odometry_position_x(),-ty+m_motor->odometry_position_y());
  //HomPoint base_target = transform_odom(HomPoint(tx,ty));
  rvizTarget = base_target;
  rviz_target_ = rvizTarget;
  //logger->log_info(name(),"odometry in visualization is: %f,%f",m_motor->odometry_position_x(),m_motor->odometry_position_y());

/*
  float x = m_motor->odometry_position_x();
  float y = m_motor->odometry_position_y();
  float ori = m_motor->odometry_orientation();
  float tx = rvizTarget.x(); 
  float ty = -rvizTarget.y();
 // float rel_x = cos(ori)*ty-sin(ori)*tx;   
 // float rel_y = sin(ori)*ty+cos(ori)*tx;

  float rel_x = cos(ori)*tx-sin(ori)*ty;   
  float rel_y = sin(ori)*tx+cos(ori)*ty;
  float target_x = x+rel_x*dist;
  float target_y = y+rel_y*dist;
  rvizTarget = HomPoint(target_x,target_y);
  rviz_target_ = rvizTarget;
 */
  NavigatorInterface::CartesianGotoMessage *nav_msg = new NavigatorInterface::CartesianGotoMessage();
  nav_msg->set_x(rvizTarget.x());
  nav_msg->set_y(rvizTarget.y());
  m_navi->msgq_enqueue(nav_msg);
/* 
  NavigatorInterface::PolarGotoMessage *nav_msg = new NavigatorInterface::PolarGotoMessage();
  nav_msg->set_dist(dist);
  nav_msg->set_phi(phi);
  m_navi->msgq_enqueue(nav_msg);
*/
  NavigatorInterface::SetDriveModeMessage *drive_msg = new NavigatorInterface::SetDriveModeMessage();
  drive_msg->set_drive_mode(NavigatorInterface::ModerateAllowBackward);
  m_navi->msgq_enqueue(drive_msg);
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::loop()
{
 // if( cells_.size() == 0 )
   // return;
  //m_motor->read();

  MutexLocker lock(&mutex_);

  nav_msgs::GridCells robs;
  robs.header.frame_id = frame_id_;
  robs.header.stamp = ros::Time::now();
  robs.cell_width = cell_width_/100.;
  robs.cell_height = cell_height_/100.;
  geometry_msgs::Point p;
  HomPoint transPoint = transform(robo_pos_);
  p.x = transPoint.x();
  p.y = transPoint.y();
  p.z = 0.0;
  robs.cells.push_back(p);
  robpub_->publish(robs);
//  logger->log_info(name(),"robot point is: %f,%f\n",p.x,p.y);
  logger->log_info(name(),"rviz target point is: %f,%f\n",rviz_target_.x(),rviz_target_.y());

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
  fix_target_ = transform(fix_target_);
  fix_target_ = transform_odom(fix_target_);
//  HomPoint temp(fix_target_.x()+(19.4/100./cell_width_),fix_target_.y());
//  fix_target_ = temp;
  geometry_msgs::Point ptarfix;
  ptarfix.x = fix_target_.x();
  ptarfix.y = fix_target_.y();
  ptarfix.z = 0.0;
  targf.cells.push_back(ptarfix);
  tarfixpub_->publish(targf);


  visualize_grid_boundary();
  visualize_path();
  visualize_occ_cells();
  visualize_laser_points();
  visualize_des_motor();
  visualize_real_motor();
  visualize_local_target();
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
HomPoint ColliVisualizationThread::transform_odom(HomPoint point)
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
void ColliVisualizationThread::visualize_local_target()
{
  nav_msgs::GridCells targl;
  targl.header.frame_id = frame_id_;
  targl.header.stamp = ros::Time::now();
  targl.cell_width = 2*cell_width_/100.;
  targl.cell_height = 2*cell_height_/100.;
  local_target_ = transform_odom(local_target_);
  geometry_msgs::Point ptarloc;
  ptarloc.x = local_target_.x();
  ptarloc.y = local_target_.y();
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
    p.y = transPoint.y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  vispub_->publish(gcs);
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
    HomPoint transPoint = transform(plan_[j]);
    transPoint = transform_odom(transPoint);
 //   HomPoint temp(transPoint.x()+(19.4/100./cell_width_),transPoint.y());
   // transPoint = temp;
    pathPoint.x = transPoint.x();
    pathPoint.y = transPoint.y();
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

















