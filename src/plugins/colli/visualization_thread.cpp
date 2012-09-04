#include "visualization_thread.h"

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

ColliVisualizationThread::ColliVisualizationThread()
  : fawkes::Thread("ColliVisualizationThread", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
}
//-------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::init()
{
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
}

//------------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize(const std::string &frame_id,vector<HomPoint > &cells,HomPoint &m_RoboGridPos,HomPoint &m_LaserGridPos,
                                         vector<HomPoint> &laser_points,vector< HomPoint > &plan, HomPoint &motor_des, 
                                         int cell_width, int cell_height, HomPoint &target, int grid_width, int grid_height, HomPoint &motor_real) throw()
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
  wakeup();
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::loop()
{
  if( cells_.size() == 0 )
    return;
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
  targf.cell_width = 5*cell_width_/100.;
  targf.cell_height = 5*cell_height_/100.;
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

















