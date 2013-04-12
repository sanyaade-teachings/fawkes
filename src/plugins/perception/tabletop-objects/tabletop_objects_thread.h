
/***************************************************************************
 *  tabletop_objects_thread.h - Thread to detect tabletop objects
 *
 *  Created: Fri Nov 04 23:54:19 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_PERCEPTION_TABLETOP_OBJECTS_THREAD_H_
#define __PLUGINS_PERCEPTION_TABLETOP_OBJECTS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>
#include <aspect/pointcloud.h>
#include <Eigen/StdVector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>

#include "cluster_colors.h"

namespace fawkes {
  class Position3DInterface;
  class SwitchInterface;
#ifdef USE_TIMETRACKER
  class TimeTracker;
#endif
}

#ifdef HAVE_VISUAL_DEBUGGING
class TabletopVisualizationThreadBase;
#endif

class TabletopObjectsThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect,
  public fawkes::PointCloudAspect
{
 public:
  TabletopObjectsThread();
  virtual ~TabletopObjectsThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

#ifdef HAVE_VISUAL_DEBUGGING
  void set_visualization_thread(TabletopVisualizationThreadBase *visthread);
#endif

 private:
  typedef pcl::PointXYZ PointType;
  typedef PointType RefPointType;
  typedef pcl::PointCloud<PointType> Cloud;
  typedef pcl::PointCloud<RefPointType> RefCloud;

  typedef pcl::PointXYZRGB ColorPointType;
  typedef pcl::PointCloud<ColorPointType> ColorCloud;
  typedef Cloud::Ptr CloudPtr;
  typedef Cloud::ConstPtr CloudConstPtr;
  typedef RefCloud::Ptr RefCloudPtr;


  typedef ColorCloud::Ptr ColorCloudPtr;
  typedef ColorCloud::ConstPtr ColorCloudConstPtr;

  typedef pcl::tracking::ParticleXYZRPY ParticleT;
  typedef pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> TrackerType;
//  typedef pcl::tracking::ParticleFilterOMPTracker<RefPointType, ParticleT> TrackerType;

 private:
  void set_position(fawkes::Position3DInterface *iface,
                    bool is_visible, const Eigen::Vector4f &centroid = Eigen::Vector4f(0, 0, 0, 0),
                    const Eigen::Quaternionf &rotation = Eigen::Quaternionf(1, 0, 0, 0));

  CloudPtr simplify_polygon(CloudPtr polygon, float sqr_dist_threshold);

  CloudPtr generate_table_model(const float length, const float width,
                                const float thickness, const float step, const float max_error);

  CloudPtr generate_table_model(const float length, const float width,
                                const float step, const float max_error = 0.01);

  bool is_polygon_edge_better(PointType &cb_br_p1p, PointType &cb_br_p2p, PointType &br_p1p, PointType &br_p2p);

  void extractSegmentCluster (const CloudConstPtr &cloud,
      const pcl::PointIndices segmented_indices,
      Cloud &result);

  std::vector<pcl::PointIndices> extract_object_clusters(
      CloudConstPtr input,
      pcl::IndicesPtr indices = pcl::IndicesPtr()); //TODO const?

  ColorCloud colorize_cluster(const Cloud &input_cloud, const std::vector<int> &cluster, uint color);

  ColorCloud colorize_cluster (const Cloud &input_cloud, uint color);

  unsigned int add_objects(CloudConstPtr input,
      CloudPtr tracking_cloud,
      ColorCloudPtr tmp_clusters,
      pcl::IndicesPtr indices = pcl::IndicesPtr());
  int find_new_indices(CloudConstPtr old_cloud, CloudConstPtr new_cloud, pcl::IndicesPtr indices);

  void reset_obj_ids();
  void reset_trackers();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::RefPtr<const pcl::PointCloud<PointType> > finput_;
  fawkes::RefPtr<pcl::PointCloud<ColorPointType> > fclusters_;
  CloudConstPtr input_;
  pcl::PointCloud<ColorPointType>::Ptr clusters_;

  pcl::VoxelGrid<PointType> grid_;
  pcl::SACSegmentation<PointType> seg_;

  std::vector<fawkes::Position3DInterface *> pos_ifs_;
  fawkes::Position3DInterface *table_pos_if_;

  fawkes::SwitchInterface *switch_if_;

  boost::shared_ptr<TrackerType> tracker_[MAX_CENTROIDS];
  std::queue<int> free_obj_ids_;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;

  float cfg_depth_filter_min_x_;
  float cfg_depth_filter_max_x_;
  float cfg_voxel_leaf_size_;
  unsigned int cfg_segm_max_iterations_;
  float cfg_segm_distance_threshold_;
  float cfg_segm_inlier_quota_;
  float cfg_max_z_angle_deviation_;
  float cfg_table_min_height_;
  float cfg_table_max_height_;
  float cfg_table_model_length_;
  float cfg_table_model_width_;
  float cfg_table_model_step_;
  float cfg_horizontal_va_;
  float cfg_vertical_va_;
  float cfg_cluster_tolerance_;
  unsigned int cfg_cluster_min_size_;
  unsigned int cfg_cluster_max_size_;
  std::string cfg_result_frame_;
  float cfg_tracking_maxdistance_;
  unsigned int cfg_tracking_particlenum_;
  float cfg_tracking_resample_likelihood_;
  float cfg_cluster_min_distance_;
  unsigned int cfg_rescan_objs_frequency_;
  float cfg_tracking_delta_;
  float cfg_tracking_eps_;
  float cfg_tracking_binsize_;
  unsigned int cfg_tracking_maxparticlenum_;
  float cfg_tracking_objthreshold_;
  unsigned int cfg_tracking_thread_nr_;
  bool first_run_;
  bool active_trackers[MAX_CENTROIDS];


  fawkes::RefPtr<Cloud> ftable_model_;
  CloudPtr table_model_;
  fawkes::RefPtr<Cloud> fsimplified_polygon_;
  CloudPtr simplified_polygon_;
  fawkes::RefPtr<Cloud> ftracking_cloud_;
  CloudPtr tracking_cloud_;
  fawkes::RefPtr<Cloud> funknown_objs_;
  CloudPtr unknown_objs_;

  unsigned int loop_count_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker  *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_full_loop_;
  unsigned int ttc_msgproc_;
  unsigned int ttc_voxelize_;
  unsigned int ttc_plane_;
  unsigned int ttc_extract_plane_;
  unsigned int ttc_plane_downsampling_;
  unsigned int ttc_cluster_plane_;
  unsigned int ttc_convex_hull_;
  unsigned int ttc_simplify_polygon_;
  unsigned int ttc_find_edge_;
  unsigned int ttc_transform_;
  unsigned int ttc_transform_model_;
  unsigned int ttc_extract_non_plane_;
  unsigned int ttc_polygon_filter_;
  unsigned int ttc_table_to_output_;
  unsigned int ttc_cluster_objects_;
  unsigned int ttc_visualization_;
#endif

#ifdef HAVE_VISUAL_DEBUGGING
  TabletopVisualizationThreadBase *visthread_;
#endif
};

#endif
