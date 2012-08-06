#include "visualization_thread.h"

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include<nav_msgs/OccupancyGrid.h>
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
}

//------------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize(const std::string &frame_id,vector<HomPoint > &cells,HomPoint &m_RoboGridPos,HomPoint &m_LaserGridPos) throw()
{
  MutexLocker lock(&mutex_);
  frame_id_ = frame_id;
  cells_.clear();
  cells_ = cells;
  robo_pos_ = m_RoboGridPos;
  laser_pos_ = m_LaserGridPos;
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
  gcs.cell_width = 5;
  gcs.cell_height = 5; 
  for( size_t i = 0; i < cells_.size(); i++ )
  {
    geometry_msgs::Point p;
    p.x = cells_[i].x();
    p.y = cells_[i].y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  
  vispub_->publish(gcs);

  nav_msgs::GridCells robs;
  robs.header.frame_id = frame_id_;
  robs.header.stamp = ros::Time::now();
  robs.cell_width = 5;
  robs.cell_height = 5;
  geometry_msgs::Point p;
  p.x = robo_pos_.x();
  p.y = robo_pos_.y();
  p.z = 0.0;
  robs.cells.push_back(p);
  robpub_->publish(robs);

  nav_msgs::GridCells laserc;
  laserc.header.frame_id = frame_id_;
  laserc.header.stamp = ros::Time::now();
  laserc.cell_width = 5;
  laserc.cell_height = 5;
  geometry_msgs::Point p_laser;
  p_laser.x = laser_pos_.x();
  p_laser.y = laser_pos_.y();
  p_laser.z = 0.0;
  laserc.cells.push_back(p);
  laserpub_->publish(laserc);

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

















