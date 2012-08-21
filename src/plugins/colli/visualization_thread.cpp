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
  *targetpub_ = rosnode->advertise<nav_msgs::GridCells>("target_cell", 1); 
  
  pathpub_ = new ros::Publisher();
  *pathpub_ = rosnode->advertise<nav_msgs::Path>("robo_path",1); 
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
}

//------------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize(const std::string &frame_id,vector<HomPoint > &cells,HomPoint &m_RoboGridPos,HomPoint &m_LaserGridPos,vector<HomPoint> &laser_points,vector< HomPoint > &plan, HomPoint &m_TargetGridPos) throw()
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
  wakeup();
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::loop()
{
  //visualize_occ();
  if( cells_.size() == 0 )
    return;
  MutexLocker lock(&mutex_);
  nav_msgs::GridCells gcs;
  gcs.header.frame_id = frame_id_;
  gcs.header.stamp = ros::Time::now(); 
  gcs.cell_width = 0.05;
  gcs.cell_height = 0.05; 
  for( size_t i = 0; i < cells_.size(); i++ )
  {
    geometry_msgs::Point p;
    p.x = cells_[i].x() / 20. - (laser_pos_.x() / 20.);
    p.y = cells_[i].y() /20. - (laser_pos_.x() / 20.);
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  
  vispub_->publish(gcs);

  nav_msgs::GridCells robs;
  robs.header.frame_id = frame_id_;
  robs.header.stamp = ros::Time::now();
  robs.cell_width = 0.05;
  robs.cell_height = 0.05;
  geometry_msgs::Point p;
  p.x = robo_pos_.x() / 20. - (laser_pos_.x() / 20.);
  p.y = robo_pos_.y() / 20. - (laser_pos_.x() / 20.);
  p.z = 0.0;
  robs.cells.push_back(p);
  robpub_->publish(robs);

  nav_msgs::GridCells laserc;
  laserc.header.frame_id = frame_id_;
  laserc.header.stamp = ros::Time::now();
  laserc.cell_width = 0.05;
  laserc.cell_height = 0.05;
  geometry_msgs::Point p_laser;
  p_laser.x = laser_pos_.x() / 20. - (laser_pos_.x() / 20.);
  p_laser.y = laser_pos_.y() /20. -(laser_pos_.x() / 20.);
  p_laser.z = 0.0;
  laserc.cells.push_back(p);
  laserpub_->publish(laserc);

  nav_msgs::GridCells lgs;
  lgs.header.frame_id = frame_id_;
  lgs.header.stamp = ros::Time::now();
  lgs.cell_width = 0.05;
  lgs.cell_height = 0.05;
  for( size_t l = 0; l < laser_points_.size(); l++ )
  {
    geometry_msgs::Point p;
    p.x = laser_points_[l].x();
    p.y = laser_points_[l].y();
    p.z = 0.0;
    lgs.cells.push_back(p);
  }
  laser_points_pub->publish(lgs);

  nav_msgs::Path robpath;
  robpath.header.frame_id = frame_id_;
  robpath.header.stamp = ros::Time::now();
  for( size_t j = 0; j < plan_.size(); j++ )
  {
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = frame_id_;
    geometry_msgs::Point pathPoint;
    pathPoint.x = plan_[j].x() / 20. - (laser_pos_.x() / 20.);
    pathPoint.y = plan_[j].y() / 20. - (laser_pos_.x() / 20.);
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
  
  nav_msgs::GridCells targc;
  targc.header.frame_id = frame_id_;
  targc.header.stamp = ros::Time::now();
  targc.cell_width = 0.05;
  targc.cell_height = 0.05;
  geometry_msgs::Point ptarget;
  ptarget.x = target_pos_.x() / 20. - (laser_pos_.x() / 20.);
  ptarget.y = target_pos_.y() / 20. - (laser_pos_.x() / 20.);
  ptarget.z = 0.0;
  targc.cells.push_back(ptarget);
  targetpub_->publish(targc);
  /*nav_msgs::OccupancyGrid occg;
  occg.header.frame_id = frame_id_;
  occg.header.stamp = ros::Time::now();
  occg.info.width = 130;
  occg.info.height = 130;
  occg.info.origin.position.x = 0;
  occg.info.origin.position.y = 0;
  occg.info.origin.position.z = 0;
  for(size_t i = 0; i < data_.size(); i++ )
    occg.data.push_back(data_[i]);
  gridpub_->publish(occg);*/
}

//----------------------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize_occ(vector<float > &data ) throw()
{
  /*nav_msgs::OccupancyGrid occg;
  occg.header.frame_id = frame_id_;
  occg.header.stamp = ros::Time::now();
  occg.info.width = 130;
  occg.info.height = 130;
  occg.info.origin.position.x = 0;
  occg.info.origin.position.y = 0;
  occg.info.origin.position.z = 0;*/
  for(size_t i = 0; i < data.size(); i++ )
    data_.push_back(data[i]);
    //occg.data.push_back(data[i]);
  //gridpub_->publish(occg);
}

















