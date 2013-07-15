#ifndef __PLUGINS_COLLI_VISUALIZATION_BASE_H_
#define __PLUGINS_COLLI_VISUALIZATION_BASE_H_


#ifndef HAVE_VISUAL_DEBUGGING
#  error ColliVisualizationThread was disabled by build flags
#endif

#include <utils/time/time.h>
//#include <Eigen/Core>
//#include <Eigen/StdVector>
#include <aspect/tf.h>
#include <geometry/hom_point.h>
#include <string.h>
#include<string>
#include <vector>

using namespace fawkes;
using namespace std;

class ColliVisualizationThreadBase
{
 public:

  virtual ~ColliVisualizationThreadBase();

  virtual void visualize(const std::string &frame_id,vector<HomPoint> &cells,HomPoint &m_RoboGridPos,HomPoint &m_LaserGridPos, vector<HomPoint> &laser_cells,
                         vector< HomPoint > &plan,HomPoint &motor_des,int cell_width, int cell_height,HomPoint &target,
                         int grid_width, int grid_height, HomPoint &motor_real,HomPoint localTarget) throw()=0;
};

#endif
