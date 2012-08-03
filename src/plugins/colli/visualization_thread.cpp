#include "visualization_thread.h"

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>

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
}
//-----------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::finalize()
{
  vispub_->shutdown();
  delete vispub_;
}

//------------------------------------------------------------------------------------------------------------------
void ColliVisualizationThread::visualize(const std::string &frame_id,vector<HomPoint > &cells) throw()
{
  MutexLocker lock(&mutex_);
  frame_id_ = frame_id;
  cells_.clear();
  cells_ = cells;
  wakeup();
}
//---------------------------------------------------------------------------------------------------
void ColliVisualizationThread::loop()
{
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
    p.x = cells_[i].x();
    p.y = cells_[i].y();
    p.z = 0.0;
    gcs.cells.push_back(p);
  }
  
  vispub_->publish(gcs);
}



















