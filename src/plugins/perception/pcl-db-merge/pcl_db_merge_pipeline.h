
/***************************************************************************
 *  pcl_db_merge_pipeline.h - Restore and merge point clouds from MongoDB
 *                            Template class for variying point types
 *
 *  Created: Sat Dec 01 00:15:45 2012 (Freiburg)
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PERCEPTION_PCL_DB_MERGE_PCL_DB_MERGE_PIPELINE_H_
#define __PLUGINS_PERCEPTION_PCL_DB_MERGE_PCL_DB_MERGE_PIPELINE_H_

#include "mongodb_tf_transformer.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <logging/logger.h>
#include <pcl_utils/utils.h>
#include <pcl_utils/transforms.h>
#include <pcl_utils/storage_adapter.h>
#include <pcl_utils/comparisons.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#define USE_ALIGNMENT
#define USE_ICP_ALIGNMENT
// define USE_NDT_ALIGNMENT

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#ifdef USE_ICP_ALIGNMENT
#  include <pcl/registration/icp.h>
#endif
#ifdef USE_NDT_ALIGNMENT
#  if not defined(PCL_VERSION_COMPARE) || PCL_VERSION_COMPARE(<,1,7,0) 
#    error NDT alignment requires PCL 1.7.0 or higher
#  endif
#  include <pcl/registration/ndt.h>
#endif

#include <client/dbclient.h>
#include <client/gridfs.h>

static const uint8_t cluster_colors[12][3] =
  { {176, 0, 30}, {0, 0, 255}, {255, 90, 0}, {137, 82, 39}, {56, 23, 90}, {99, 0, 30},
    {255, 0, 0}, {0, 255, 0}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {27, 117, 196}};

template <typename PointType>
class PointCloudDBMergePipeline
{
 private:
  typedef pcl::PointCloud<PointType> Cloud;

  typedef pcl::PointXYZRGB ColorPointType;
  typedef pcl::PointCloud<ColorPointType> ColorCloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  typedef typename ColorCloud::Ptr ColorCloudPtr;
  typedef typename ColorCloud::ConstPtr ColorCloudConstPtr;

 public:
 PointCloudDBMergePipeline(mongo::DBClientBase *mongodb_client, std::string &database_name,
			   std::string &global_frame, ColorCloudPtr output,
			   fawkes::Logger *logger)
   : mongodb_client_(mongodb_client), database_name_(database_name),
    global_frame_(global_frame), output_(output), logger_(logger)
  {
    name_ = "PCL_DB_MergePL";
    mongodb_gridfs_ =
      new mongo::GridFS(*mongodb_client_, database_name);
      use_alignment_ = true;
#ifdef USE_TIMETRACKER
    tt_ = new fawkes::TimeTracker();
    tt_loopcount_ = 0;
    ttc_merge_              = tt_->add_class("Full Merge");
    ttc_retrieval_          = tt_->add_class("Retrieval");
    ttc_transform_global_   = tt_->add_class("Transform to Map");
    ttc_downsample_         = tt_->add_class("Downsampling");
    ttc_align_1_            = tt_->add_class("First ICP");
    ttc_transform_1_        = tt_->add_class("Apply 1st TF");
    ttc_remove_planes_      = tt_->add_class("Plane Removal");
    ttc_align_2_            = tt_->add_class("Second ICP");
    ttc_transform_final_    = tt_->add_class("Apply final TF");
    ttc_output_    = tt_->add_class("Output");
#endif
  }

  ~PointCloudDBMergePipeline()
  {
    delete mongodb_gridfs_;
  }

  bool
  applicable(std::vector<long long> &times, std::string &collection)
  {
    const unsigned int num_clouds = times.size();

    std::vector<sensor_msgs::PointField> pfields;
    pcl::for_each_type<typename pcl::traits::fieldList<PointType>::type>
      (pcl::detail::FieldAdder<PointType>(pfields));

    for (unsigned int i = 0; i < num_clouds; ++i) {
      std::auto_ptr<mongo::DBClientCursor> cursor =
	mongodb_client_->query(database_name_ + "." + collection,
			       QUERY("timestamp" << mongo::LTE << times[i])
			       .sort("timestamp", -1),
			       /* limit */ 1);

      if (cursor->more()) {
	mongo::BSONObj p = cursor->next();
	mongo::BSONObj pcldoc = p.getObjectField("pointcloud");
	std::vector<mongo::BSONElement> fields = pcldoc["field_info"].Array();

	for (unsigned int i = 0; i < pfields.size(); ++i) {
	  sensor_msgs::PointField &pf = pfields[i];

	  bool found = false;
	  for (unsigned int j = 0; j < fields.size(); ++j) {
	    if ((fields[j]["name"].String() == pf.name) &&
		(fields[j]["offset"].Int() == (int)pf.offset) &&
		(fields[j]["datatype"].Int() == pf.datatype) &&
		(fields[j]["count"].Int() == (int)pf.count) )
	    {
	      found = true;
	      break;
	    }
	  }
	  if (! found)  return false;
	}
      } else {
	return false;
      }
    }

    return true;
  }


  void
  merge(std::vector<long long> &times, std::string &collection)
  {
    TIMETRACK_START(ttc_merge_);
    const unsigned int num_clouds = times.size();

    std::vector<long long> actual_times(num_clouds);

    output_->points.clear();
    output_->height = 1;
    output_->width  = 0;
    output_->is_dense = false;

    size_t num_points = 0;

    CloudPtr pcls[num_clouds];
    CloudPtr non_transformed[num_clouds];
    CloudPtr non_aligned[num_clouds];
    CloudPtr non_aligned_downsampled[num_clouds];
    CloudPtr aligned_downsampled[num_clouds];
    CloudPtr aligned_downsampled_remplane[num_clouds];
    Eigen::Matrix4f transforms[num_clouds-1];

    for (unsigned int i = 0; i < num_clouds; ++i) {
      non_transformed[i] = CloudPtr(new Cloud());
      non_aligned[i] = CloudPtr(new Cloud());
      non_aligned_downsampled[i] = CloudPtr(new Cloud());
      aligned_downsampled[i] = CloudPtr(new Cloud());
      aligned_downsampled_remplane[i] = CloudPtr(new Cloud());
    }

    TIMETRACK_START(ttc_retrieval_);

    // retrieve point clouds
    for (unsigned int i = 0; i < num_clouds; ++i) {

      std::auto_ptr<mongo::DBClientCursor> cursor =
	mongodb_client_->query(database_name_ + "." + collection,
			       QUERY("timestamp" << mongo::LTE << times[i])
			       .sort("timestamp", -1),
			       /* limit */ 1);

      if (cursor->more()) {
	mongo::BSONObj p = cursor->next();
	mongo::BSONObj pcldoc = p.getObjectField("pointcloud");
	std::vector<mongo::BSONElement> fields = pcldoc["field_info"].Array();

	logger_->log_info(name_, "Restoring XYZ point cloud");
	// reconstruct point cloud

	CloudPtr lpcl(new Cloud());
	pcls[i] = lpcl;

	actual_times[i]       = p["timestamp"].Number();
	fawkes::Time actual_time((long)actual_times[i]);

	lpcl->header.frame_id = pcldoc["frame_id"].String();
	lpcl->is_dense        = (pcldoc["is_dense"].Int() != 0);
	lpcl->width           = pcldoc["width"].Int();
	lpcl->height          = pcldoc["height"].Int();
	fawkes::pcl_utils::set_time(lpcl, actual_time);
	lpcl->points.resize(pcldoc["num_points"].Int());

	read_gridfs_file(&lpcl->points[0], pcldoc.getFieldDotted("data.filename").String());

	num_points += lpcl->points.size();

	*non_transformed[i] = *lpcl;

      } else {
	logger_->log_warn(name_, "Cannot retrieve document for time %li", times[i]);
	return;
      }
    }

    TIMETRACK_INTER(ttc_retrieval_, ttc_transform_global_);

    for (unsigned int i = 0; i < num_clouds; ++i) {
      // retrieve transforms
      fawkes::tf::MongoDBTransformer
	transformer(mongodb_client_, database_name_);

      transformer.restore(/* start */  actual_times[i] - 1000,
			  /* end */    actual_times[i] + 1000);


      // transform point clouds to common frame
      fawkes::Time actual_time((long)actual_times[i]);
      try {
	fawkes::pcl_utils::transform_pointcloud(global_frame_, *pcls[i], transformer);
      } catch (fawkes::tf::TransformException &e) {
	logger_->log_warn(name_, "Failed to transform from %s to %s",
			 pcls[i]->header.frame_id.c_str(),
			 global_frame_.c_str());
	logger_->log_warn(name_, e);
      }
      *non_aligned[i] = *pcls[i];
    }

    // merge point clouds

    TIMETRACK_END(ttc_transform_global_);

    if (use_alignment_) {
      // align point clouds, use the first as target

      TIMETRACK_START(ttc_downsample_);

      // ### 1: ALIGN including table points

      // FILTER and DOWNSAMPLE

      pcl::PassThrough<PointType> pass;
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.6, 1.2);

      //pcl::ApproximateVoxelGrid<PointType> downsample;
      pcl::VoxelGrid<PointType> downsample;
      downsample.setLeafSize(0.01, 0.01, 0.01);

      CloudPtr filtered_z(new Cloud());

      for (unsigned int i = 0; i < num_clouds; ++i) {

	// downsample for efficient registration/Alignment
	pass.setInputCloud(pcls[i]);
	pass.filter(*filtered_z);

	downsample.setInputCloud(filtered_z);
	downsample.filter(*non_aligned_downsampled[i]);
	logger_->log_info(name_, "Filtered cloud %u contains %zu points",
			  i, non_aligned_downsampled[i]->points.size ());
      }
      TIMETRACK_INTER(ttc_downsample_, ttc_align_1_);

      // ALIGN using ICP including table
      for (unsigned int i = 1; i < num_clouds; ++i) {
	logger_->log_info(name_, "Aligning cloud %u to %u", i, 0);
	Eigen::Matrix4f transform;
	CloudConstPtr source, target;

	source = non_aligned_downsampled[i];
	target = non_aligned_downsampled[0];

#  ifdef USE_ICP_ALIGNMENT
	align_icp(source, target, transform);

#  elif defined(USE_NDT_ALIGNMENT)
	align_ndt(source, target);
#  endif

        transforms[i-1] = transform;
      }

      TIMETRACK_INTER(ttc_align_1_, ttc_transform_1_);

      // ### 2: ALIGN excluding table points

      *aligned_downsampled[0] = *non_aligned_downsampled[0];
      for (unsigned int i = 1; i < num_clouds; ++i) {
	pcl::transformPointCloud(*non_aligned_downsampled[i],
				 *aligned_downsampled[i], transforms[i-1]);
      }

      TIMETRACK_INTER(ttc_transform_1_, ttc_remove_planes_);

      for (unsigned int i = 0; i < num_clouds; ++i) {
	*aligned_downsampled_remplane[i] = *aligned_downsampled[i];
	remove_plane(aligned_downsampled_remplane[i]);
	logger_->log_info(name_, "Removed plane from cloud %u, %zu of %zu points remain",
			  i, aligned_downsampled_remplane[i]->points.size(),
			  aligned_downsampled[i]->points.size());
      }

      TIMETRACK_INTER(ttc_remove_planes_, ttc_align_2_);

      for (unsigned int i = 1; i < num_clouds; ++i) {
	Eigen::Matrix4f transform;
	CloudConstPtr source, target;

	source = aligned_downsampled_remplane[i];
	target = aligned_downsampled_remplane[0];
	
	align_icp(source, target, transform);

	Cloud tmp;
	pcl::transformPointCloud(*aligned_downsampled_remplane[i], tmp, transform);
	*aligned_downsampled_remplane[i] = tmp;

	transforms[i-1] *= transform;
      }

      TIMETRACK_INTER(ttc_align_2_, ttc_transform_final_);

      for (unsigned int i = 1; i < num_clouds; ++i) {
	Cloud tmp;
	pcl::transformPointCloud(*pcls[i], tmp, transforms[i-1]);
	*pcls[i] = tmp;
      }

      TIMETRACK_END(ttc_transform_final_);

    }

    TIMETRACK_END(ttc_merge_);
    TIMETRACK_START(ttc_output_);


#ifdef DEBUG_OUTPUT
    fawkes::Time now;

    merge_output(non_transformed, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(output_, now);
    usleep(1000000);

    merge_output(non_aligned, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(output_, now);
    usleep(1000000);

    merge_output(non_aligned_downsampled, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(output_, now);
    usleep(1000000);

    merge_output(aligned_downsampled, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(output_, now);
    usleep(1000000);

    merge_output(aligned_downsampled_remplane, num_clouds);
    now.stamp(); fawkes::pcl_utils::set_time(output_, now);
    usleep(1000000);
#endif

    merge_output(pcls, num_clouds);

    TIMETRACK_END(ttc_output_);

#ifdef USE_TIMETRACKER
    if (++tt_loopcount_ >= 5) {
      tt_loopcount_ = 0;
      tt_->print_to_stdout();
    }
#endif
  }

 private: // methods
  void read_gridfs_file(void *dataptr, std::string filename)
  {
    char *tmp = (char *)dataptr;
    mongo::GridFile file =
      mongodb_gridfs_->findFile(filename);
    if (! file.exists()) {
      logger_->log_warn(name_, "Grid file does not exist");
      return;
    }

    size_t bytes = 0;
    for (int c = 0; c < file.getNumChunks(); ++c) {
      mongo::GridFSChunk chunk = file.getChunk(c);
      int len = 0;
      const char *chunk_data = chunk.data(len);
      memcpy(tmp, chunk_data, len);
      tmp += len;
      bytes += len;
      //logger_->log_info(name_, "Read chunk %i of %i bytes", c, len);
    }
    //logger_->log_info(name_, "%zu bytes restored", bytes);
  }

  void remove_plane(CloudPtr &cloud)
  {
    pcl::SACSegmentation<PointType> tablesegm;
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    tablesegm.setOptimizeCoefficients(true);
    tablesegm.setModelType(pcl::SACMODEL_PLANE);
    tablesegm.setMethodType(pcl::SAC_RANSAC);
    tablesegm.setMaxIterations(1000);
    tablesegm.setDistanceThreshold(0.022);

    tablesegm.setInputCloud(cloud);
    tablesegm.segment(*inliers, *coeff);

    pcl::ExtractIndices<PointType> extract;
    Cloud extracted;
    extract.setNegative(true);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(extracted);
    *cloud = extracted;

    pcl::ConvexHull<PointType> convex_hull;
    convex_hull.setDimension(2);
    convex_hull.setInputCloud(cloud);
    CloudPtr hull(new Cloud());
    convex_hull.reconstruct(*hull);

    // Use only points above tables
    // Why coeff->values[3] > 0 ? ComparisonOps::GT : ComparisonOps::LT?
    // The model coefficients are in Hessian Normal Form, hence coeff[0..2] are
    // the normal vector. We need to distinguish the cases where the normal vector
    // points towards the origin (camera) or away from it. This can be checked
    // by calculating the distance towards the origin, which conveniently in
    // dist = N * x + p is just p which is coeff[3]. Therefore, if coeff[3] is
    // positive, the normal vector points towards the camera and we want all
    // points with positive distance from the table plane, otherwise it points
    // away from the origin and we want points with "negative distance".
    // We make use of the fact that we only have a boring RGB-D camera and
    // not an X-Ray...
    pcl::ComparisonOps::CompareOp op =
      coeff->values[3] < 0 ? pcl::ComparisonOps::GT : pcl::ComparisonOps::LT;
    typename fawkes::pcl_utils::PlaneDistanceComparison<PointType>::ConstPtr
      above_comp(new fawkes::pcl_utils::PlaneDistanceComparison<PointType>(coeff, op));
    typename pcl::ConditionAnd<PointType>::Ptr
      above_cond(new pcl::ConditionAnd<PointType>());
    above_cond->addComparison(above_comp);
    pcl::ConditionalRemoval<PointType> above_condrem(above_cond);
    above_condrem.setInputCloud(cloud);
    CloudPtr cloud_above(new Cloud());
    above_condrem.filter(*cloud_above);

    // Extract only points on the table plane
    pcl::PointIndices::Ptr polygon(new pcl::PointIndices());

    typename pcl::ConditionAnd<PointType>::Ptr
      polygon_cond(new pcl::ConditionAnd<PointType>());

    typename fawkes::pcl_utils::PolygonComparison<PointType>::ConstPtr
      inpoly_comp(new fawkes::pcl_utils::PolygonComparison<PointType>(*hull));
    polygon_cond->addComparison(inpoly_comp);

    // build the filter
    pcl::ConditionalRemoval<PointType> condrem(polygon_cond);
    condrem.setInputCloud(cloud_above);
    condrem.filter(*cloud);
  }


#ifdef USE_ICP_ALIGNMENT
  bool align_icp(CloudConstPtr source, CloudConstPtr target, Eigen::Matrix4f &transform)
  {
    Cloud final;

    //pcl::console::VERBOSITY_LEVEL old_level = pcl::console::getVerbosityLevel();
    //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputCloud(source);
    icp.setInputTarget(target);

    icp.setRANSACIterations(5000);

    // Set the max correspondence distance to 5cm
    // (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.1);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(200);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-7);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1e-7);

    logger_->log_info(name_, "Aligning");
    icp.align(final);
    logger_->log_info(name_, "Aligning done");
    //logger_->log_info(name_, "ICP %u -> %u did%s converge, score: %f",
    //	       icp.hasConverged() ? "" : " NOT", icp.getFitnessScore());
    transform = icp.getFinalTransformation();
    //score     = icp.getFitnessScore();
    //pcl::console::setVerbosityLevel(old_level);
    return icp.hasConverged();
  }
#endif

#ifdef USE_NDT_ALIGNMENT
  // untested
  bool align_ndt(CloudConstPtr source, CloudConstPtr target, Eigen::Matrix4f &transform)
  {
    Cloud final;

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setInputCloud(source);
    ndt.setInputTarget(target);

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(1.0);
      
    // Setting max number of registration iterations.
    ndt.setMaximumIterations(5);

    ndt.align(final);
    transform = ndt.getFinalTransformation();
    return ndt.hasConverged();
  }
#endif

  void merge_output(CloudPtr *clouds, unsigned int num_clouds)
  {
    size_t num_points = 0;
    for (unsigned int i = 0; i < num_clouds; ++i) {
      num_points += clouds[i]->points.size();
    }
    output_->points.resize(num_points);
    output_->height = 1;
    output_->width = num_points;
    size_t out_p = 0;
    for (unsigned int i = 0; i < num_clouds; ++i) {
      const CloudPtr &lpcl = clouds[i];
      const size_t cldn = lpcl->points.size();
      if (cldn == 0)  continue;

      for (size_t p = 0; p < cldn; ++p, ++out_p) {
	const PointType &ip      = lpcl->points[p];
	ColorPointType &op = output_->points[out_p];

	op.x = ip.x;
	op.y = ip.y;
	op.z = ip.z;

	op.r = cluster_colors[i][0];
	op.g = cluster_colors[i][1];
	op.b = cluster_colors[i][2];
      }
    }
  }


 private: // members
  const char *name_;

  mongo::DBClientBase *mongodb_client_;
  mongo::GridFS *mongodb_gridfs_;

  std::string database_name_;
  std::string global_frame_;
  ColorCloudPtr output_;

  fawkes::Logger *logger_;
#ifdef USE_TIMETRACKER
  fawkes::TimeTracker *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_merge_;
  unsigned int ttc_retrieval_;
  unsigned int ttc_transform_global_;
  unsigned int ttc_downsample_;
  unsigned int ttc_align_1_;
  unsigned int ttc_transform_1_;
  unsigned int ttc_remove_planes_;
  unsigned int ttc_align_2_;
  unsigned int ttc_transform_final_;
  unsigned int ttc_output_;
#endif

  bool use_alignment_;
};


#endif