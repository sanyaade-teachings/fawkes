/***************************************************************************
 *  gazeboscene_thread.cpp - Gazebo Scene Reconstruction Thread
 *
 *  Created: Mon Aug 27 09:53:54 2012
 *  Author  Bastian Klingen
 *
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
#include "gazeboscene_thread.h"

#include <core/threading/mutex_locker.h>
#include <cstdlib>
#include <float.h>
#include <fvutils/writers/png.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// from MongoDB
#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>

// from Gazebo
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Image.hh>

using namespace mongo;
using namespace gazebo;
using namespace fawkes;
using namespace firevision;


/** @class GazeboSceneThread "gazeboscene_thread.h"
 *  Gazebo Scene Reconstruction Thread.
 *  This thread gets the previously recorded data from the MongoDB and
 *  packs it into gazebo::msgs to alter the world inside the simulator.
 *
 *  @author Bastian Klingen
 */

/** Constructor. */
GazeboSceneThread::GazeboSceneThread()
  : Thread("GazeboSceneThread", Thread::OPMODE_CONTINUOUS),
    TransformAspect(TransformAspect::BOTH, "scenereconstruction")
{
}


/** Destructor. */
GazeboSceneThread::~GazeboSceneThread()
{
}


/** Initialize.
 *  Initialize MongoDB and Gazebo.
 *  Register publishers and subscribers, get the database to use and
 *  set all needed variables. Send a message to the scenereconstruction
 *  GUI to inform it that the thread is available. Also send data to the
 *  Gazebo plugins to create a small buffer for robot/joint positions and
 *  objects to spawn.
 */
void
GazeboSceneThread::init()
{
  #ifdef USE_RRD
  __inmutex = new boost::mutex();
  __outmutex = new boost::mutex();

  {
    boost::mutex::scoped_lock inlock(*__inmutex);
    boost::mutex::scoped_lock outlock(*__outmutex);
    __rrdupdate = new Time(clock);
    __rrdupdate->stamp_systime();
    __outmsgcount_graph = NULL;
    __outmsgsize_graph = NULL;
    __inmsgcount_graph = NULL;
    __inmsgsize_graph = NULL;

    std::vector<RRDDataSource> rrds;
    rrds.push_back(RRDDataSource("GUI_Lasers", RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("GUI_Response", RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("GUI_Transform", RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("OI_Object",   RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("OI_Response",  RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("RC_Control",  RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("RC_Drawing", RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("Overall",  RRDDataSource::COUNTER));
    __outmsgcount_rrd = new RRDDefinition("outmsgcount", rrds);
    __outmsgcounts["GUI Lasers"] = 0;
    __outmsgcounts["GUI Response"] = 0;
    __outmsgcounts["GUI Transform"] = 0;
    __outmsgcounts["OI Object"] = 0;
    __outmsgcounts["OI Response"] = 0;
    __outmsgcounts["RC Control"] = 0;
    __outmsgcounts["RC Drawing"] = 0;
    __outmsgcounts["OVerall"] = 0;

    rrds.clear();
    rrds.push_back(RRDDataSource("GUI_Lasers", RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("GUI_Response", RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("GUI_Transform", RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("OI_Object",   RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("OI_Response",  RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("RC_Control",  RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("RC_Drawing", RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("Overall",  RRDDataSource::GAUGE));
    __outmsgsize_rrd = new RRDDefinition("outmsgsize", rrds);
    __outmsgsizes["GUI Lasers"] = 0;
    __outmsgsizes["GUI Response"] = 0;
    __outmsgsizes["GUI Transform"] = 0;
    __outmsgsizes["OI Object"] = 0;
    __outmsgsizes["OI Response"] = 0;
    __outmsgsizes["RC Control"] = 0;
    __outmsgsizes["RC Drawing"] = 0;
    __outmsgsizes["Overall"] = 0;

    rrds.clear();
    rrds.push_back(RRDDataSource("Control", RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("Lasers",  RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("Request", RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("TransformRequest", RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("WorldStatistics", RRDDataSource::COUNTER));
    rrds.push_back(RRDDataSource("Overall",  RRDDataSource::COUNTER));
    __inmsgcount_rrd = new RRDDefinition("inmsgcount", rrds);
    __inmsgcounts["Control"] = 0;
    __inmsgcounts["Lasers"] = 0;
    __inmsgcounts["Request"] = 0;
    __inmsgcounts["TransformRequest"] = 0;
    __inmsgcounts["WorldStatistics"] = 0;
    __inmsgcounts["Overall"] = 0;

    rrds.clear();
    rrds.push_back(RRDDataSource("Control", RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("Lasers",  RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("Request", RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("TransformRequest", RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("WorldStatistics", RRDDataSource::GAUGE));
    rrds.push_back(RRDDataSource("Overall",  RRDDataSource::GAUGE));
    __inmsgsize_rrd = new RRDDefinition("inmsgsize", rrds);
    __inmsgsizes["Control"] = 0;
    __inmsgsizes["Lasers"] = 0;
    __inmsgsizes["Request"] = 0;
    __inmsgsizes["TransformRequest"] = 0;
    __inmsgsizes["WorldStatistics"] = 0;
    __inmsgsizes["Overall"] = 0;

    try {
      rrd_manager->add_rrd(__inmsgcount_rrd);
      rrd_manager->add_rrd(__outmsgcount_rrd);
      rrd_manager->add_rrd(__inmsgsize_rrd);
      rrd_manager->add_rrd(__outmsgsize_rrd);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to add RRDs, ", "exception follows");
      logger->log_warn(name(), e);
      finalize();
      throw;
    }

    std::vector<RRDGraphDataDefinition> defs;
    std::vector<RRDGraphElement *> els;

    defs.push_back(RRDGraphDataDefinition("GUI_Lasers", RRDArchive::AVERAGE,
					  __outmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("GUI_Response", RRDArchive::AVERAGE,
					  __outmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("GUI_Transform", RRDArchive::AVERAGE,
					  __outmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("OI_Object", RRDArchive::AVERAGE,
					  __outmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("OI_Response", RRDArchive::AVERAGE,
					  __outmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("RC_Control", RRDArchive::AVERAGE,
					  __outmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("RC_Drawing", RRDArchive::AVERAGE,
					  __outmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("Overall", RRDArchive::AVERAGE,
					  __outmsgcount_rrd));
    
    els.push_back(new RRDGraphLine("GUI_Lasers", 1, "A52A2A", "GUI Lasers"));
    els.push_back(new RRDGraphGPrint("GUI_Lasers", RRDArchive::LAST,
				     "   Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("GUI_Lasers", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("GUI_Lasers", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("GUI_Response", 1, "800000", "GUI Response"));
    els.push_back(new RRDGraphGPrint("GUI_Response", RRDArchive::LAST,
				     " Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("GUI_Response", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("GUI_Response", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("GUI_Transform", 1, "8B4513", "GUI Transform"));
    els.push_back(new RRDGraphGPrint("GUI_Transform", RRDArchive::LAST,
				     "Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("GUI_Transform", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("GUI_Transform", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("OI_Object", 1, "4B0082", "OI  Object"));
    els.push_back(new RRDGraphGPrint("OI_Object", RRDArchive::LAST,
				     "   Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("OI_Object", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("OI_Object", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("OI_Response", 1, "0000CD", "OI  Response"));
    els.push_back(new RRDGraphGPrint("OI_Response", RRDArchive::LAST,
				     " Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("OI_Response", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("OI_Response", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("RC_Control", 1, "6B8E23", "RC  Control"));
    els.push_back(new RRDGraphGPrint("RC_Control", RRDArchive::LAST,
				     "  Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("RC_Control", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("RC_Control", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("RC_Drawing", 1, "808000", "RC  Drawing"));
    els.push_back(new RRDGraphGPrint("RC_Drawing", RRDArchive::LAST,
				     "  Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("RC_Drawing", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("RC_Drawing", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("Overall", 1, "DAA520", "Overall"));
    els.push_back(new RRDGraphGPrint("Overall", RRDArchive::LAST,
				     "      Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Overall", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Overall", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    __outmsgcount_graph = new RRDGraphDefinition("outmsgcount", __outmsgcount_rrd,
					        "Outgoing Gazebo Messages", "Amount",
					        defs, els);

    defs.clear();
    defs.push_back(RRDGraphDataDefinition("GUI_Lasers", RRDArchive::AVERAGE,
					  __outmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("GUI_Response", RRDArchive::AVERAGE,
					  __outmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("GUI_Transform", RRDArchive::AVERAGE,
					  __outmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("OI_Object", RRDArchive::AVERAGE,
					  __outmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("OI_Response", RRDArchive::AVERAGE,
					  __outmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("RC_Control", RRDArchive::AVERAGE,
					  __outmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("RC_Drawing", RRDArchive::AVERAGE,
					  __outmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("Overall", RRDArchive::AVERAGE,
					  __outmsgsize_rrd));
    
    __outmsgsize_graph = new RRDGraphDefinition("outmsgsize", __outmsgsize_rrd,
					        "Outgoing Gazebo Messages", "MB",
					        defs, els);

    defs.clear();
    els.clear();

    defs.push_back(RRDGraphDataDefinition("Control", RRDArchive::AVERAGE,
					  __inmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("Lasers", RRDArchive::AVERAGE,
					  __inmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("Request", RRDArchive::AVERAGE,
					  __inmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("TransformRequest", RRDArchive::AVERAGE,
					  __inmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("WorldStatistics", RRDArchive::AVERAGE,
					  __inmsgcount_rrd));
    defs.push_back(RRDGraphDataDefinition("Overall", RRDArchive::AVERAGE,
					  __inmsgcount_rrd));
    
    els.push_back(new RRDGraphLine("Control", 1, "800000", "Control"));
    els.push_back(new RRDGraphGPrint("Control", RRDArchive::LAST,
				     "  Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Control", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Control", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("Lasers", 1, "A52A2A", "Lasers"));
    els.push_back(new RRDGraphGPrint("Lasers", RRDArchive::LAST,
				     "   Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Lasers", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Lasers", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("Request", 1, "808000", "Request"));
    els.push_back(new RRDGraphGPrint("Request", RRDArchive::LAST,
				     "  Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Request", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Request", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("TransformRequest", 1, "8B4513", "Transform"));
    els.push_back(new RRDGraphGPrint("TransformRequest", RRDArchive::LAST,
				     "Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("TransformRequest", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("TransformRequest", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("WorldStatistics", 1, "D2691E", "WorldStat"));
    els.push_back(new RRDGraphGPrint("WorldStatistics", RRDArchive::LAST,
				     "Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("WorldStatistics", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("WorldStatistics", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    els.push_back(new RRDGraphLine("Overall", 1, "DAA520", "Overall"));
    els.push_back(new RRDGraphGPrint("Overall", RRDArchive::LAST,
				     "  Current\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Overall", RRDArchive::AVERAGE,
				     "Average\\:%8.2lf %s"));
    els.push_back(new RRDGraphGPrint("Overall", RRDArchive::MAX,
				     "Maximum\\:%8.2lf %s\\n"));

    __inmsgcount_graph = new RRDGraphDefinition("inmsgcount", __inmsgcount_rrd,
					        "Received Gazebo Messages", "Amount",
					        defs, els);

    defs.clear();
    defs.push_back(RRDGraphDataDefinition("Control", RRDArchive::AVERAGE,
					  __inmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("Lasers", RRDArchive::AVERAGE,
					  __inmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("Request", RRDArchive::AVERAGE,
					  __inmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("TransformRequest", RRDArchive::AVERAGE,
					  __inmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("WorldStatistics", RRDArchive::AVERAGE,
					  __inmsgsize_rrd));
    defs.push_back(RRDGraphDataDefinition("Overall", RRDArchive::AVERAGE,
					  __inmsgsize_rrd));

    __inmsgsize_graph = new RRDGraphDefinition("inmsgsize", __inmsgsize_rrd,
					        "Received Gazebo Messages", "MB",
					        defs, els);

    try {
      rrd_manager->add_graph(__inmsgcount_graph);
      rrd_manager->add_graph(__outmsgcount_graph);
      rrd_manager->add_graph(__inmsgsize_graph);
      rrd_manager->add_graph(__outmsgsize_graph);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to add graphs, ", "exception follows");
      logger->log_warn(name(), e);
      finalize();
      throw;
    }
  }
  #endif

  __database = "fflog";
  try {
    __database = config->get_string("/plugins/gazeboscene/database");
    logger->log_info(name(), "GazeboScene uses database: %s",
		       __database.c_str());
  }
  catch (Exception &eg) {
    logger->log_info(name(), "No database for GazeboScene configured, trying MongoLog's database: %s",
		       __database.c_str());
    try {
      __database = config->get_string("/plugins/mongolog/database");
    } catch (Exception &em) {
      logger->log_info(name(), "No database configured, reading from default database: %s",
		       __database.c_str());
    }
  }

  __dbbuffer = 500;
  try {
    __dbbuffer = (double) config->get_uint("/plugins/gazeboscene/buffer");
    logger->log_info(name(), "GazeboScene uses a buffer of %dms", __dbbuffer);
  }
  catch (Exception &e) {
    logger->log_info(name(), "No buffer for GazeboScene configured, defaulting to %dms", __dbbuffer);
  }

  __mongodb    = mongodb_client;
  __gazebo     = gazebonode;

  // define the zero time for the transforms
  __scenestart = new Time(clock);
  __scenestart->stamp_systime();
  __pause      = true;
  __pausestart = new Time();
  __pausestart->stamp_systime();
  __step       = true;
  __buffering  = false;

  std::list<std::string>::iterator iter;
  __collections = __mongodb->getCollectionNames(__database);
  __collections.sort();
  double starttime = DBL_MAX;
  double endtime = 0.0;

  BSONObj obj;
  BSONObj returnfields = fromjson("{timestamp:1}");
  BSONElement timestamp;
  for(iter = __collections.begin(); iter != __collections.end(); iter++) {
    // index the database
    if(*iter != __database+".system.indexes" && iter->find("GridFS") == std::string::npos) {
      __mongodb->ensureIndex(*iter, fromjson("{timestamp:1}"), true);
    }

    // determine the length of the scene
    if(iter->find("Position3DInterface") != std::string::npos || iter->find("KatanaInterface") != std::string::npos || iter->find("PanTiltInterface") != std::string::npos || iter->find("Localize") != std::string::npos) {
      obj = __mongodb->findOne(*iter, Query().sort("timestamp",1), &returnfields);
      if(!obj.isEmpty()) {
        timestamp = obj.getField("timestamp");
        if(starttime > timestamp.Number()) {
          starttime = timestamp.Number();
        }
      }
      obj = __mongodb->findOne(*iter, Query().sort("timestamp",-1), &returnfields);
      if(!obj.isEmpty()) {
        timestamp = obj.getField("timestamp");
        if(endtime < timestamp.Number())
          endtime = timestamp.Number();
      }
    }
    // create mongo::GridFS for the files collections
    else if(iter->find("GridFS") != std::string::npos && iter->find("files") != std::string::npos) {
      size_t dot1, dot2;
      std::string collectionname = __mongodb->nsGetCollection(*iter);
      dot1 = collectionname.find(".")+1;
      dot2 = collectionname.find(".", dot1);
      __mongogrids[collectionname.substr(dot1, dot2 - dot1)] = new GridFS(*__mongodb, __database, collectionname.substr(0, dot2));
    }

    // determine collections containing transforms
    if(iter->find("TransformInterface") != std::string::npos) {
      __tfcollections.push_back(*iter);
    }

    // determine collections containing lasers
    size_t lsr;
    if(((lsr = iter->find("Laser")) != std::string::npos) && iter->find("Interface.", lsr+5) != std::string::npos) {
      __lasercollections[*iter] = false;
    }

    // determine collections containing positions
    if(iter->find("Position3DInterface") != std::string::npos) {
      __poscollections.push_back(*iter);
      __querystarttimes[*iter] = 0.0;
    }
  }

  __dbtimeoffset = starttime;
  __dbcurtimeoffset = starttime;
  __dbscenelength = endtime - starttime;
  __dbbuffered = starttime;
  __lasttf = 0.0;

  logger->log_debug(name(), "Creating Gazebo publishers and waiting for connection");
  objectinstantiatorResponsePub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/ObjectInstantiator/Response");
  objectinstantiatorResponsePub->WaitForConnection();
  objectinstantiatorObjectPub = __gazebo->Advertise<msgs::Message_V>("~/SceneReconstruction/ObjectInstantiator/Object");
  objectinstantiatorObjectPub->WaitForConnection();
  robotcontrollerControlPub = __gazebo->Advertise<msgs::Message_V>("~/SceneReconstruction/RobotController/");
  robotcontrollerControlPub->WaitForConnection();
  scenereconstructionGuiPub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/GUI/MongoDB");
  scenereconstructionGuiPub->WaitForConnection();
  setupPub = __gazebo->Advertise<gazebo::msgs::SceneRobotController>("~/SceneReconstruction/RobotController/Init");
  setupPub->WaitForConnection();
  statusPub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/GUI/Availability/Response");
  statusPub->WaitForConnection();
  transformPub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/GUI/Response");
  transformPub->WaitForConnection();
  worldcontrolPub = __gazebo->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  worldcontrolPub->WaitForConnection();
  drawingPub = __gazebo->Advertise<msgs::Drawing>("~/SceneReconstruction/RobotController/Draw");
  drawingPub->WaitForConnection();
  timePub = __gazebo->Advertise<msgs::Double>("~/SceneReconstruction/GUI/Time");
  timePub->WaitForConnection();
  lasersPub = __gazebo->Advertise<msgs::Lasers>("~/SceneReconstruction/GUI/Lasers");
  lasersPub->WaitForConnection();
  logger->log_debug(name(), "Gazebo publishers created and connected");

  // feed initial transforms
  double start = __lasttf;
  double end = __dbcurtimeoffset + __dbbuffer;
  send_transforms(start, end);

  BSONObj bson;
  // get initial robot position & joint position
  bson = __mongodb->findOne(__database+".TransformInterface.TF_RCSoftX_Localize", Query().sort("timestamp",1));
  msgs::SceneRobotController robcon;
  std::vector<BSONElement> trans = bson.getField("translation").Array();
  std::vector<BSONElement> rota = bson.getField("rotation").Array();

  robcon.set_pos_x(trans[0].Double());
  robcon.set_pos_y(trans[1].Double());
  robcon.set_pos_z(trans[2].Double());
  robcon.set_ori_x(rota[0].Double());
  robcon.set_ori_y(rota[1].Double());
  robcon.set_ori_z(rota[2].Double());
  robcon.set_ori_w(rota[3].Double());

  // get and send initial joint positions
  std::vector<BSONElement> angles;
  bson = __mongodb->findOne(__database+".KatanaInterface.Katana", Query().sort("timestamp",1));
  angles = bson.getField("angles").Array();
  std::stringstream jointname;
  double jointangle;
  for(unsigned int i = 0; i < angles.size();  i++) {
    jointname.str("Katana_");
    jointname << i;
    jointangle = angles[i].Double();
    robcon.add_robot_name(jointname.str());
    robcon.add_robot_angle(jointangle);
  }

  // get and send initial joint positions
  bson = __mongodb->findOne(__database+".PanTiltInterface.PanTilt_RX28", Query().sort("timestamp",1));

  jointangle = bson.getField("pan").Double();
  robcon.add_robot_name("PanTilt_RX28_pan");
  robcon.add_robot_angle(jointangle);
  jointangle = bson.getField("tilt").Double();
  robcon.add_robot_name("PanTilt_RX28_tilt");
  robcon.add_robot_angle(jointangle);

  setupPub->Publish(robcon);

  frameworkRequestSub = __gazebo->Subscribe("~/SceneReconstruction/Framework/Request", &GazeboSceneThread::OnRequestMsg, this);
  transformRequestSub = __gazebo->Subscribe("~/SceneReconstruction/Framework/TransformRequest", &GazeboSceneThread::OnTransformRequestMsg, this);
  controlSub = __gazebo->Subscribe("~/SceneReconstruction/Framework/Control", &GazeboSceneThread::OnControlMsg, this);
  worldSub = __gazebo->Subscribe("~/world_stats", &GazeboSceneThread::OnWorldStatisticsMsg, this);
  laserSub = __gazebo->Subscribe("~/SceneReconstruction/Framework/Lasers", &GazeboSceneThread::OnLaserMsg, this);

  __available = false;
}


/** Finalize.
 *  cleans up everything
 */
void
GazeboSceneThread::finalize()
{
  objectinstantiatorResponsePub.reset();
  objectinstantiatorObjectPub.reset();
  robotcontrollerControlPub.reset();
  scenereconstructionGuiPub.reset();
  setupPub.reset();
  statusPub.reset();
  transformPub.reset();
  worldcontrolPub.reset();
  drawingPub.reset();
  timePub.reset();
  lasersPub.reset();
  if(frameworkRequestSub)
    frameworkRequestSub->Unsubscribe();
  frameworkRequestSub.reset();
  if(transformRequestSub)
  transformRequestSub->Unsubscribe();
  transformRequestSub.reset();
  if(controlSub)
  controlSub->Unsubscribe();
  controlSub.reset();
  if(laserSub)
  laserSub->Unsubscribe();
  laserSub.reset();

  #ifdef USE_RRD
  rrd_manager->remove_rrd(__inmsgsize_rrd);
  rrd_manager->remove_rrd(__inmsgcount_rrd);
  rrd_manager->remove_rrd(__outmsgsize_rrd);
  rrd_manager->remove_rrd(__outmsgcount_rrd);
  delete __inmsgsize_graph;
  delete __inmsgcount_graph;
  delete __outmsgsize_graph;
  delete __outmsgcount_graph;
  delete __inmsgsize_rrd;
  delete __inmsgcount_rrd;
  delete __outmsgsize_rrd;
  delete __outmsgcount_rrd;
  #endif
}


/** Loop.
 */
void
GazeboSceneThread::loop()
{
  if(!__available) {
    // send availability message in case the request was not received
    msgs::Response avail;
    avail.set_id(-1);
    avail.set_request("status");
    avail.set_response("Framework");  
    this->statusPub->Publish(avail);

    // send length of the scene to the GUI
    msgs::Double time;
    time.set_data(__dbscenelength);
    this->timePub->Publish(time);

    msgs::Response response;
    response.set_id(-1);
    response.set_request("collection_names");
    response.set_response("success");
    msgs::GzString_V src;
    response.set_type(src.GetTypeName());

    std::list<std::string>::iterator iter;
    for(iter = __collections.begin(); iter != __collections.end(); iter++) {
        src.add_data(__mongodb->nsGetCollection(*iter));
    }

    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    #ifdef USE_RRD
    {
      boost::mutex::scoped_lock lock(*__outmutex);
      __outmsgcounts["GUI Response"]++;
      __outmsgcounts["Overall"]++;
      __outmsgsizes["GUI Response"] += response.ByteSize();
      __outmsgsizes["Overall"] += response.ByteSize();
    }
    #endif

    scenereconstructionGuiPub->Publish(response);

    usleep(500000);
  }

  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock inlock(*__inmutex);
    boost::mutex::scoped_lock outlock(*__outmutex);
    Time now(clock);
    now.stamp_systime();
    if((now.in_sec() - __rrdupdate->in_sec()) >= 10.0) {
      try {
	      rrd_manager->add_data("outmsgcount", "N:%i:%i:%i:%i:%i:%i:%i:%i",
          __outmsgcounts["GUI Lasers"],
          __outmsgcounts["GUI Response"],
          __outmsgcounts["GUI Transform"],
          __outmsgcounts["OI Object"],
          __outmsgcounts["OI Response"],
          __outmsgcounts["RC Control"],
          __outmsgcounts["RC Drawing"],
          __outmsgcounts["Overall"]);
      } catch (Exception &e) {
	      logger->log_warn(name(), "Failed to update outmsgcount RRD, ", "exception follows");
	      logger->log_warn(name(), e);
      }
      try {
	      rrd_manager->add_data("outmsgsize", "N:%i:%i:%i:%i:%i:%i:%i:%i",
          __outmsgsizes["GUI Lasers"],
          __outmsgsizes["GUI Response"],
          __outmsgsizes["GUI Transform"],
          __outmsgsizes["OI Object"],
          __outmsgsizes["OI Response"],
          __outmsgsizes["RC Control"],
          __outmsgsizes["RC Drawing"],
          __outmsgsizes["Overall"]);
      } catch (Exception &e) {
	      logger->log_warn(name(), "Failed to update outmsgsize RRD, ", "exception follows");
	      logger->log_warn(name(), e);
      }
      try {
	      rrd_manager->add_data("inmsgcount", "N:%i:%i:%i:%i:%i:%i",
          __inmsgcounts["Control"],
          __inmsgcounts["Lasers"],
          __inmsgcounts["Request"],
          __inmsgcounts["TransformRequest"],
          __inmsgcounts["WorldStatistics"],
          __inmsgcounts["Overall"]);
      } catch (Exception &e) {
	      logger->log_warn(name(), "Failed to update inmsgcount RRD, ", "exception follows");
	      logger->log_warn(name(), e);
      }
      try {
	      rrd_manager->add_data("inmsgsize", "N:%i:%i:%i:%i:%i:%i",
          __inmsgsizes["Control"],
          __inmsgsizes["Lasers"],
          __inmsgsizes["Request"],
          __inmsgsizes["TransformRequest"],
          __inmsgsizes["WorldStatistics"],
          __inmsgsizes["Overall"]);
      } catch (Exception &e) {
	      logger->log_warn(name(), "Failed to update inmsgsize RRD, ", "exception follows");
	      logger->log_warn(name(), e);
      }
      __outmsgsizes["GUI Lasers"] = 0;
      __outmsgsizes["GUI Response"] = 0;
      __outmsgsizes["GUI Transform"] = 0;
      __outmsgsizes["OI Object"] = 0;
      __outmsgsizes["OI Response"] = 0;
      __outmsgsizes["RC Control"] = 0;
      __outmsgsizes["RC Drawing"] = 0;
      __outmsgsizes["Overall"] = 0;
      __inmsgsizes["Control"] = 0;
      __inmsgsizes["Lasers"] = 0;
      __inmsgsizes["Request"] = 0;
      __inmsgsizes["TransformRequest"] = 0;
      __inmsgsizes["WorldStatistics"] = 0;
      __inmsgsizes["Overall"] = 0;
      *__rrdupdate = now;
    }
  }
  #endif
}


/** OnWorldStatisticsMsg
 *  Callback for WorldStatistics messages to feed transforms, send object positions,
 *  joint positions and the robot position, draw lasers if enabled
 */
void
GazeboSceneThread::OnWorldStatisticsMsg(ConstWorldStatisticsPtr &_msg)
{
  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__inmutex);
    __inmsgcounts["WorldStatistics"]++;
    __inmsgcounts["Overall"]++;
    __inmsgsizes["WorldStatistics"] += _msg->ByteSize();
    __inmsgsizes["Overall"] += _msg->ByteSize();
  }
  #endif

  // only buffer if not paused or a step is triggered and if no buffering is currently done, else skip this message
  if(!__buffering && (!__pause || __step)) {
    __buffering = true;
    __dbcurrenttime = _msg->sim_time().sec()*1000 + _msg->sim_time().nsec()/1000000 +  __dbcurtimeoffset;
    double maxtime = __dbcurrenttime + __dbbuffer;

    // feed transforms
    if(maxtime >= __lasttf) {
      double start = __lasttf;
      __lasttf = send_transforms(start, maxtime);
    }

    // send objects, robotpositions and jointangles
    if(maxtime >= __dbbuffered) {
      // get and send object data
      double start = __dbbuffered;

      double tmpbuffered = buffer_objects(start, maxtime);
      if(__dbbuffered < tmpbuffered)
        __dbbuffered = tmpbuffered;

      // get and send robot positions
      tmpbuffered = buffer_positions(start, maxtime);
      if(__dbbuffered < tmpbuffered)
        __dbbuffered = tmpbuffered;

      // get and send joint positions
      tmpbuffered = buffer_joints(start,maxtime);
      if(__dbbuffered < tmpbuffered)
        __dbbuffered = tmpbuffered;
    }

    draw_lasers(__dbcurrenttime);

    __step = false;
    __buffering = false;
  }
}

void
GazeboSceneThread::OnControlMsg(ConstSceneFrameworkControlPtr &_msg)
{
  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__inmutex);
    __inmsgcounts["Control"]++;
    __inmsgcounts["Overall"]++;
    __inmsgsizes["Control"] += _msg->ByteSize();
    __inmsgsizes["Overall"] += _msg->ByteSize();
  }
  #endif

  if(_msg->has_pause()) {
    __pause = _msg->pause();
    if(!__pause) {
      // add the pause time to the scene start so calculations are still fine
      Time offset(clock);
      offset.stamp_systime();
      offset -= *__pausestart;
      *__scenestart += offset;
    }
    else {
      __pausestart->stamp_systime();
    }
  }

  if(_msg->has_step()) {
    __step = _msg->step();
  }

  if(_msg->has_change_offset()) {
    if(_msg->change_offset() && _msg->has_offset()) {
      __dbcurtimeoffset = __dbtimeoffset + _msg->offset();

      // adjust __scenestart for the new offset
      __scenestart->stamp_systime();

      // feed transforms of the previous 10 seconds
      tf_listener->clear();
      __lasttf = send_transforms(__dbcurtimeoffset - 10000, __dbcurtimeoffset + __dbbuffer);

      __dbbuffered = __dbcurtimeoffset;
      double start = __dbbuffered;
      double end = start + __dbbuffer;

      // get and send object data
      double tmpbuffered = buffer_objects(start, end);
      if(__dbbuffered < tmpbuffered)
        __dbbuffered = tmpbuffered;

      // get and send robot positions
      tmpbuffered = buffer_positions(start, end);
      if(__dbbuffered < tmpbuffered)
        __dbbuffered = tmpbuffered;

      // get and send joint positions
      tmpbuffered = buffer_joints(start,end);
      if(__dbbuffered < tmpbuffered)
        __dbbuffered = tmpbuffered;

      msgs::WorldControl wc;
      wc.set_step(true);
      worldcontrolPub->Publish(wc);
      __step = true;
    }
  }
}

void
GazeboSceneThread::OnLaserMsg(ConstLasersPtr &_msg)
{
  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__inmutex);
    __inmsgcounts["Lasers"]++;
    __inmsgcounts["Overall"]++;
    __inmsgsizes["Lasers"] += _msg->ByteSize();
    __inmsgsizes["Overall"] += _msg->ByteSize();
  }
  #endif

  if(_msg->has_update() && _msg->update()) {
    int i, v;
    i = _msg->interface_size();
    v = _msg->visible_size();
    if(i != v)
      return;

    for(int l=0; l<i; l++) {
      __lasercollections[_msg->interface(l)] = _msg->visible(l);
    }

    // force a step to update lasers
    __step = true;
  }

  // send current laser settings
  msgs::Lasers las;
  las.set_update(false);
  std::map<std::string, bool>::iterator iter;
  for(iter = __lasercollections.begin(); iter != __lasercollections.end(); iter++) {
    las.add_interface(iter->first);
    las.add_visible(iter->second);
  }

  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__outmutex);
    __outmsgcounts["GUI Lasers"]++;
    __outmsgcounts["Overall"]++;
    __outmsgsizes["GUI Lasers"] += las.ByteSize();
    __outmsgsizes["Overall"] += las.ByteSize();
  }
  #endif

  lasersPub->Publish(las);
}

void
GazeboSceneThread::OnRequestMsg(ConstRequestPtr &_msg)
{
  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__inmutex);
    __inmsgcounts["Request"]++;
    __inmsgcounts["Overall"]++;
    __inmsgsizes["Request"] += _msg->ByteSize();
    __inmsgsizes["Overall"] += _msg->ByteSize();
  }
  #endif

  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  if(_msg->request() == "available") {
    __available = true;
  }
  else if(_msg->request() == "collection_names") {
    msgs::GzString_V src;
    response.set_type(src.GetTypeName());

    std::list<std::string>::iterator iter;
    for(iter = __collections.begin(); iter != __collections.end(); iter++) {
        src.add_data(__mongodb->nsGetCollection(*iter));
    }

    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    #ifdef USE_RRD
    {
      boost::mutex::scoped_lock lock(*__outmutex);
      __outmsgcounts["GUI Response"]++;
      __outmsgcounts["Overall"]++;
      __outmsgsizes["GUI Response"] += response.ByteSize();
      __outmsgsizes["Overall"] += response.ByteSize();
    }
    #endif

    scenereconstructionGuiPub->Publish(response);
  }
  else if(_msg->request() == "select_collection") {
    if(_msg->has_data() && find(__collections.begin(), __collections.end(), __database+"."+_msg->data()) != __collections.end()) {
      __collection = _msg->data();
      msgs::GzString_V src;
      response.set_type(src.GetTypeName());
      std::stringstream conv;
      conv << __mongodb->count(__database+"."+__collection);
      double query_from = __dbcurrenttime - __dbbuffer;
      double query_to   = __dbcurrenttime + __dbbuffer;

      // workaround for deprecated auto_ptr
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      auto_ptr<DBClientCursor> cursor = __mongodb->query(__database+"."+__collection, QUERY("timestamp" << GTE << query_from << LTE << query_to).sort("timestamp"));
      #pragma GCC diagnostic warning "-Wdeprecated-declarations"
      BSONObj obj;
      while(cursor->more()) {
        obj = cursor->next();
        double t = obj.getField("timestamp").Number() - __dbtimeoffset;
        std::stringstream ts;
        ts << t;
        src.add_data(ts.str());
      }

      if(src.data_size() < 1) {
        obj = __mongodb->findOne(__database+"."+__collection, QUERY("timestamp" << LTE << query_to).sort("timestamp", -1));
        double t = obj.getField("timestamp").Number() - __dbtimeoffset;
        std::stringstream ts;
        ts << t;
        src.add_data(ts.str());
      }

      if(src.data_size() < 1) {
        obj = __mongodb->findOne(__database+"."+__collection, QUERY("timestamp" << GTE << query_from).sort("timestamp"));
        double t = obj.getField("timestamp").Number() - __dbtimeoffset;
        std::stringstream ts;
        ts << t;
        src.add_data(ts.str());
      }

      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);

      #ifdef USE_RRD
      {
        boost::mutex::scoped_lock lock(*__outmutex);
        __outmsgcounts["GUI Response"]++;
        __outmsgcounts["Overall"]++;
        __outmsgsizes["GUI Response"] += response.ByteSize();
        __outmsgsizes["Overall"] += response.ByteSize();
      }
      #endif

      scenereconstructionGuiPub->Publish(response);

    }
    else {
      response.set_response("failure");
      msgs::GzString src;
      response.set_type(src.GetTypeName());
      src.set_data("the given collection is unknown to the framework");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
    }

    #ifdef USE_RRD
    {
      boost::mutex::scoped_lock lock(*__outmutex);
      __outmsgcounts["GUI Response"]++;
      __outmsgcounts["Overall"]++;
      __outmsgsizes["GUI Response"] += response.ByteSize();
      __outmsgsizes["Overall"] += response.ByteSize();
    }
    #endif

    scenereconstructionGuiPub->Publish(response);
  }
  else if(_msg->request() == "select_document") {
    if(_msg->has_data()) {
      // workaround for deprecated auto_ptr
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      auto_ptr<DBClientCursor> cursor = __mongodb->query(__database+"."+__collection, QUERY("timestamp" << (_msg->dbl_data() + __dbtimeoffset)));
      #pragma GCC diagnostic warning "-Wdeprecated-declarations"
      if(cursor->more()) {
        BSONObj obj = cursor->next();
        msgs::GzString src;
        response.set_type(src.GetTypeName());
        src.set_data(obj.toString());
        std::string *serializedData = response.mutable_serialized_data();
        src.SerializeToString(serializedData);
      }
    }
    else {
      response.set_response("failure");
      msgs::GzString src;
      response.set_type(src.GetTypeName());
      src.set_data("the given database is unknown to the framework");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
    }

    scenereconstructionGuiPub->Publish(response);
  }
  else if(_msg->request() == "object_data") {
    if(_msg->has_data()) {
      msgs::Message_V src;
      response.set_type(src.GetTypeName());
      msgs::SceneDocument type;
      src.set_msgtype(type.GetTypeName());
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      auto_ptr<DBClientCursor> cursor;
      #pragma GCC diagnostic warning "-Wdeprecated-declarations"
      BSONObj bson;
      std::list<std::string>::iterator iter;
      std::string query = _msg->data();
      if(query == "")
        query = QUERY("timestamp" << LTE << __dbtimeoffset).toString();
      logger->log_debug(name(), ("processing query \""+query+"\" for all collections").c_str());
      for(iter = __collections.begin(); iter != __collections.end(); iter++) {
        if(*iter != __database+".system.indexes" && iter->find("GridFS") == std::string::npos) {
          cursor = __mongodb->query(*iter, fromjson(query));
          while(cursor->more()) {
            bson = cursor->next();
            msgs::SceneDocument doc;
            fill_document_message(bson, doc, *iter);
            std::string *msg = src.add_msgsdata();
            doc.SerializeToString(msg);
            
            // if current message exceeds 2 MB, send it and start a new message
            if(src.ByteSize() >= (2097152)) {
              response.set_response("part");
              std::string *serializedData = response.mutable_serialized_data();
              src.SerializeToString(serializedData);
              #ifdef USE_RRD
              {
                boost::mutex::scoped_lock lock(*__outmutex);
                __outmsgcounts["OI Response"]++;
                __outmsgcounts["Overall"]++;
                __outmsgsizes["OI Response"] += response.ByteSize();
                __outmsgsizes["Overall"] += response.ByteSize();
              }
              #endif

              objectinstantiatorResponsePub->Publish(response);
              src.clear_msgsdata();
            }
          }

          response.set_response("success");
        }
      }
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
      logger->log_debug(name(), ("processed query \""+query+"\" for all collections").c_str());
    }
    else {
      response.set_response("failure");
      msgs::GzString src;
      response.set_type(src.GetTypeName());
      src.set_data("no object reference given");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
    }

    #ifdef USE_RRD
    {
      boost::mutex::scoped_lock lock(*__outmutex);
      __outmsgcounts["OI Response"]++;
      __outmsgcounts["Overall"]++;
      __outmsgsizes["OI Response"] += response.ByteSize();
      __outmsgsizes["Overall"] += response.ByteSize();
    }
    #endif

    objectinstantiatorResponsePub->Publish(response);
  }
  else if(_msg->request() == "documents") {
    if(_msg->has_data()) {
      logger->log_debug(name(), ("requesting documents for collection: "+_msg->data()).c_str());
      msgs::Message_V src;
      response.set_type(src.GetTypeName());
      msgs::SceneDocument type;
      src.set_msgtype(type.GetTypeName());
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      auto_ptr<DBClientCursor> cursor;
      #pragma GCC diagnostic warning "-Wdeprecated-declarations"
      BSONObj bson;
      std::list<std::string>::iterator iter;
      iter = find(__collections.begin(), __collections.end(), __database+"."+_msg->data());
      if(iter != __collections.end()) {
        double start = __dbcurrenttime - 2*__dbbuffer;
        double end = __dbcurrenttime + 2*__dbbuffer;
        cursor = __mongodb->query(*iter, QUERY("timestamp" << GTE << start << LTE << end).sort("timestamp"));
        bool send = false;
        while(cursor->more()) {
          bson = cursor->next();
          msgs::SceneDocument doc;
          fill_document_message(bson, doc, *iter);
          std::string *msg = src.add_msgsdata();
          doc.SerializeToString(msg);

          // if current message exceeds 2 MB, send it and start a new message
          if(src.ByteSize() >= (2097152)) {
            response.set_response("part");
            std::string *serializedData = response.mutable_serialized_data();
            src.SerializeToString(serializedData);
            #ifdef USE_RRD
            {
              boost::mutex::scoped_lock lock(*__outmutex);
              __outmsgcounts["GUI Response"]++;
              __outmsgcounts["Overall"]++;
              __outmsgsizes["GUI Response"] += response.ByteSize();
              __outmsgsizes["Overall"] += response.ByteSize();
            }
            #endif

            scenereconstructionGuiPub->Publish(response);
            src.clear_msgsdata();
            send = true;
          }
        }
        response.set_response("success");
        
        if(!send && src.msgsdata_size() < 1) {
          logger->log_debug(name(), "no document inside timeframe, requesting freshest one");
          bson = __mongodb->findOne(*iter, QUERY("timestamp" << LTE << end).sort("timestamp", -1));
          msgs::SceneDocument doc;
          fill_document_message(bson, doc, *iter);
          std::string *msg = src.add_msgsdata();
          doc.SerializeToString(msg);
        }

        if(!send && src.msgsdata_size() < 1) {
          logger->log_debug(name(), "no freshest document found, requesting next one");
          bson = __mongodb->findOne(*iter, QUERY("timestamp" << GTE << start).sort("timestamp"));
          msgs::SceneDocument doc;
          fill_document_message(bson, doc, *iter);
          std::string *msg = src.add_msgsdata();
          doc.SerializeToString(msg);
        }

        std::string *serializedData = response.mutable_serialized_data();
        src.SerializeToString(serializedData);
  
        logger->log_debug(name(), "requesting documents done");
      }
      else {
        response.set_response("failure");
        msgs::GzString src;
        response.set_type(src.GetTypeName());
        src.set_data("data in documents request does not refer to a collection");
        std::string *serializedData = response.mutable_serialized_data();
        src.SerializeToString(serializedData);
      }
    }
    else {
      response.set_response("failure");
      msgs::GzString src;
      response.set_type(src.GetTypeName());
      src.set_data("missing data in documents request");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
    }

    #ifdef USE_RRD
    {
      boost::mutex::scoped_lock lock(*__outmutex);
      __outmsgcounts["GUI Response"]++;
      __outmsgcounts["Overall"]++;
      __outmsgsizes["GUI Response"] += response.ByteSize();
      __outmsgsizes["Overall"] += response.ByteSize();
    }
    #endif

    scenereconstructionGuiPub->Publish(response);
  }
  else {
    response.set_response("unknown");
    msgs::GzString src;
    response.set_type(src.GetTypeName());
    src.set_data("the given request is unknown to the framework");
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    #ifdef USE_RRD
    {
      boost::mutex::scoped_lock lock(*__outmutex);
      __outmsgcounts["GUI Response"]++;
      __outmsgcounts["Overall"]++;
      __outmsgsizes["GUI Response"] += response.ByteSize();
      __outmsgsizes["Overall"] += response.ByteSize();
    }
    #endif

    scenereconstructionGuiPub->Publish(response);
  }
}

void
GazeboSceneThread::OnTransformRequestMsg(ConstTransformRequestPtr &_msg)
{
  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__inmutex);
    __inmsgcounts["TransformRequest"]++;
    __inmsgcounts["Overall"]++;
    __inmsgsizes["TransformRequest"] += _msg->ByteSize();
    __inmsgsizes["Overall"] += _msg->ByteSize();
  }
  #endif

  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");

  if(_msg->request() == "transform_request") {
    msgs::Pose src;
    response.set_type(src.GetTypeName());

    // transform pose
    tf::Vector3 pos(_msg->pos_x(), _msg->pos_y(), _msg->pos_z());
    tf::Quaternion ori(_msg->ori_x(), _msg->ori_y(), _msg->ori_z(), _msg->ori_w());
    tf::Pose pose(ori, pos);
    Time tftime(clock);
    tftime.stamp_systime();
    tf::Stamped<tf::Pose> stamped_pose(pose, tftime, _msg->source_frame());
    tf::Stamped<tf::Pose> result_pose;
    if(tf_listener->can_transform(_msg->target_frame(), _msg->source_frame(), tftime)) {
      tf::StampedTransform transform;
      tf_listener->lookup_transform(_msg->target_frame(), _msg->source_frame(), tftime, transform);
      result_pose.set_data(transform * stamped_pose);
      src = Convert(result_pose);
    }
    else if(tf_listener->can_transform(_msg->source_frame(), _msg->target_frame(), tftime)) {
      tf::StampedTransform transform;
      tf_listener->lookup_transform(_msg->source_frame(), _msg->target_frame(), tftime, transform);
      result_pose.set_data(transform.inverseTimes(stamped_pose));
      src = Convert(result_pose);
    }
    else {
      gazebo::math::Pose invispose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      src = msgs::Convert(invispose);
    }

    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);

    #ifdef USE_RRD
    {
      boost::mutex::scoped_lock lock(*__outmutex);
      __outmsgcounts["GUI Transform"]++;
      __outmsgcounts["Overall"]++;
      __outmsgsizes["GUI Transform"] += response.ByteSize();
      __outmsgsizes["Overall"] += response.ByteSize();
    }
    #endif

    transformPub->Publish(response);
  }
}

msgs::Pose
GazeboSceneThread::Convert(tf::Stamped<tf::Pose> pose) {
  msgs::Pose result;
  result.set_name(pose.frame_id);
  msgs::Vector3d *pos = result.mutable_position();
  tf::Vector3 vec = pose.getOrigin();
  pos->set_x(vec.getX());
  pos->set_y(vec.getY());
  pos->set_z(vec.getZ());
  msgs::Quaternion *ori = result.mutable_orientation();
  tf::Quaternion qua = pose.getRotation();
  ori->set_x(qua.getX());
  ori->set_y(qua.getY());
  ori->set_z(qua.getZ());
  ori->set_w(qua.getW());
  return result;
}

double
GazeboSceneThread::send_transforms(double start, double end) {
  // increase buffering time for transforms
  end += __dbbuffer;
//  logger->log_debug(name(), "tranforms from: %.0f to: %.0f", start-__dbcurtimeoffset+__scenestart->in_msec(), end-__dbcurtimeoffset+__scenestart->in_msec());
  // workaround for deprecated auto_ptr
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto_ptr<DBClientCursor> cursor;
  #pragma GCC diagnostic warning "-Wdeprecated-declarations"
  BSONObj bson;
  double buffered = start;
  std::list<std::string>::iterator iter;
  for(iter = __tfcollections.begin(); iter != __tfcollections.end(); iter++) {
    cursor = __mongodb->query(*iter, QUERY("timestamp" << GTE << start << LT << end).sort("timestamp"));
    while(cursor->more()) {
      bson = cursor->next();
      std::vector<BSONElement> trans = bson.getField("translation").Array();
      std::vector<BSONElement> rota = bson.getField("rotation").Array();
      double rx, ry, rz, rw, tx, ty, tz;
      std::string frame, child_frame;
      long msec = bson.getField("timestamp").Long();
      msec -= (long) __dbcurtimeoffset;
      msec += __scenestart->in_msec();
      // date transform slightly to the future
      msec += 250;
      Time time(msec);
      rx = rota[0].Double();
      ry = rota[1].Double();
      rz = rota[2].Double();
      rw = rota[3].Double();
      tx = trans[0].Double();
      ty = trans[1].Double();
      tz = trans[2].Double();
      frame = bson.getField("frame").String();
      child_frame = bson.getField("child_frame").String();

      tf::Quaternion q(rx, ry, rz, rw);
      tf::assert_quaternion_valid(q);
      tf::Transform t(q, tf::Vector3(tx, ty, tz));
      tf::StampedTransform transform(t, time, frame, child_frame);
      tf_publisher->send_transform(transform);

      if(bson.getField("timestamp").Number() > buffered)
        buffered = bson.getField("timestamp").Number();
    }
  }

  return buffered;
}

double
GazeboSceneThread::buffer_objects(double start, double end) {
//  logger->log_debug(name(), "objects from: %.0f to: %.0f", start-__dbcurtimeoffset+__scenestart->in_msec(), end-__dbcurtimeoffset+__scenestart->in_msec());
  // workaround for deprecated auto_ptr
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto_ptr<DBClientCursor> cursor;
  #pragma GCC diagnostic warning "-Wdeprecated-declarations"
  BSONObj bson;
  double buffered = start;
  std::list<std::string>::iterator iter;
  msgs::Message_V objmsgs;
  msgs::SceneObject obj;
  objmsgs.set_msgtype(obj.GetTypeName());
  for(iter = __poscollections.begin(); iter != __poscollections.end(); iter++) {
    cursor = __mongodb->query(*iter, QUERY("timestamp" << GTE << start << LT << end).sort("timestamp"));
    while(cursor->more()) {
      bson = cursor->next();
      std::string objectname = __mongodb->nsGetCollection(*iter);
      objectname = objectname.substr(objectname.find(".")+1);
      obj.set_object(objectname);
      obj.set_visible(bson.getField("visibility_history").Int()>0);

      msgs::Pose *p = obj.mutable_pose();
      std::vector<BSONElement> trans = bson.getField("translation").Array();
      std::vector<BSONElement> rota = bson.getField("rotation").Array();
      

      tf::Vector3 pos(trans[0].Double(), trans[1].Double(), trans[2].Double());
      tf::Quaternion ori(rota[0].Double(), rota[1].Double(), rota[2].Double(), rota[3].Double());
      tf::Pose pose(ori, pos);
      long msec = bson.getField("timestamp").Long();
      msec -= (long) __dbcurtimeoffset;
      msec += __scenestart->in_msec();
      std::string frame = bson.getField("frame").String();
      obj.set_frame(frame);

      // only take into account if frame is valid
      if(tf_listener->frame_exists(frame)) {
        Time tftime(msec);
        tf::Stamped<tf::Pose> stamped_pose(pose, tftime, frame);
        tf::Stamped<tf::Pose> result_pose;

        if(tf_listener->can_transform("/map", frame, tftime)) {
          tf::StampedTransform transform;
          try {
            tf_listener->lookup_transform("/map", frame, tftime, transform);
            result_pose.set_data(transform * stamped_pose);
            *p = Convert(result_pose);
          } catch (Exception &e) {
            logger->log_warn(name(), "Unable to transform pose for object: \"%s\" at time: %li", objectname.c_str(), msec);
            logger->log_warn(name(), e);
            *p = Convert(stamped_pose);
          }
        }
        else if(tf_listener->can_transform(frame, "/map", tftime)) {
          tf::StampedTransform transform;
          try {
            tf_listener->lookup_transform(frame, "/map", tftime, transform);
            result_pose.set_data(transform.inverseTimes(stamped_pose));
            *p = Convert(result_pose);
          } catch (Exception &e) {
            logger->log_warn(name(), "Unable to transform pose for object: \"%s\" at time: %li", objectname.c_str(), msec);
            logger->log_warn(name(), e);
            *p = Convert(stamped_pose);
          }
        }
        else {
          logger->log_warn(name(), "Unable to transform pose for object: \"%s\" at time: %li", objectname.c_str(), msec);
          tf::StampedTransform transform;
          try {
            tf_listener->lookup_transform("/map", frame, tftime, transform);
          } catch (Exception &e) {
            logger->log_warn(name(), e);
          }
          try {
            tf_listener->lookup_transform(frame, "/map", tftime, transform);
          } catch (Exception &e) {
            logger->log_warn(name(), e);
          }
          *p = Convert(stamped_pose);
        }

        double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
        obj.set_time(time);

        std::string query = QUERY("timestamp" << GT << __querystarttimes[*iter] << LTE << bson.getField("timestamp").Number()).sort("timestamp").toString();
        obj.set_query(query);

        std::string *serializedData = objmsgs.add_msgsdata();
        obj.SerializeToString(serializedData);

        __querystarttimes[*iter] = bson.getField("timestamp").Number();
      }
      if(bson.getField("timestamp").Number() > buffered)
        buffered = bson.getField("timestamp").Number();
    }
  }

  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__outmutex);
    __outmsgcounts["OI Object"]++;
    __outmsgcounts["Overall"]++;
    __outmsgsizes["OI Object"] += objmsgs.ByteSize();
    __outmsgsizes["Overall"] += objmsgs.ByteSize();
  }
  #endif

  objectinstantiatorObjectPub->Publish(objmsgs);

  return buffered;
}

double
GazeboSceneThread::buffer_positions(double start, double end) {
//  logger->log_debug(name(), "positions from: %.0f to: %.0f", start-__dbcurtimeoffset+__scenestart->in_msec(), end-__dbcurtimeoffset+__scenestart->in_msec());
  // workaround for deprecated auto_ptr
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto_ptr<DBClientCursor> cursor;
  #pragma GCC diagnostic warning "-Wdeprecated-declarations"
  BSONObj bson;
  double buffered = start;
  msgs::Message_V robmsgs;
  std::vector<BSONElement> trans;
  std::vector<BSONElement> rota;

  cursor = __mongodb->query(__database+".TransformInterface.TF_RCSoftX_Localize", QUERY("timestamp" << GTE << start << LT << end));
  msgs::SceneRobot rob;
  robmsgs.set_msgtype(rob.GetTypeName());
  while(cursor->more()) {
    bson = cursor->next();
    trans = bson.getField("translation").Array();
    rota = bson.getField("rotation").Array();

    msgs::Pose *pose = rob.mutable_pose();
    msgs::Vector3d *pos = pose->mutable_position();
    msgs::Quaternion *ori = pose->mutable_orientation();
    pos->set_x(trans[0].Double());
    pos->set_y(trans[1].Double());
    pos->set_z(trans[2].Double());
    ori->set_x(rota[0].Double());
    ori->set_y(rota[1].Double());
    ori->set_z(rota[2].Double());
    ori->set_w(rota[3].Double());

    double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
    rob.set_controltime(time);

    std::string *serializedData = robmsgs.add_msgsdata();
    rob.SerializeToString(serializedData);

    if(bson.getField("timestamp").Number() > buffered)
      buffered = bson.getField("timestamp").Number();
  }

  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__outmutex);
    __outmsgcounts["RC Control"]++;
    __outmsgcounts["Overall"]++;
    __outmsgsizes["RC Control"] += robmsgs.ByteSize();
    __outmsgsizes["Overall"] += robmsgs.ByteSize();
  }
  #endif

  robotcontrollerControlPub->Publish(robmsgs);

  return buffered;
}

double
GazeboSceneThread::buffer_joints(double start, double end) {
//  logger->log_debug(name(), "joints from: %.0f to: %.0f", start-__dbcurtimeoffset+__scenestart->in_msec(), end-__dbcurtimeoffset+__scenestart->in_msec());
  // workaround for deprecated auto_ptr
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto_ptr<DBClientCursor> cursor;
  #pragma GCC diagnostic warning "-Wdeprecated-declarations"
  BSONObj bson;
  double buffered = start;
  msgs::Message_V jntmsgs;
  std::vector<BSONElement> angles;
  double jointangle;

  cursor = __mongodb->query(__database+".KatanaInterface.Katana", QUERY("timestamp" << GTE << start << LT << end));
  while(cursor->more()) {
    msgs::SceneJoint jnt;

    bson = cursor->next();
    angles = bson.getField("angles").Array();
    std::stringstream jointname;

    for(unsigned int i = 0; i < angles.size();  i++) {
      jointname.str("");
      jointname << "Katana_" << (int)i;
      jointangle = angles[i].Double();
      jnt.add_joint(jointname.str());
      jnt.add_angle(jointangle);
    }

    double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
    jnt.set_controltime(time);


    std::string *serializedData = jntmsgs.add_msgsdata();
    jnt.SerializeToString(serializedData);

    if(bson.getField("timestamp").Number() > buffered)
      buffered = bson.getField("timestamp").Number();
  }

  cursor = __mongodb->query(__database+".PanTiltInterface.PanTilt_RX28", QUERY("timestamp" << GTE << start << LT << end));
  while(cursor->more()) {
    msgs::SceneJoint jnt;

    bson = cursor->next();

    jointangle = bson.getField("pan").Double();
    jnt.add_joint("PanTilt_RX28_pan");
    jnt.add_angle(jointangle);
    jointangle = bson.getField("tilt").Double();
    jnt.add_joint("PanTilt_RX28_tilt");
    jnt.add_angle(jointangle);

    double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
    jnt.set_controltime(time);

    std::string *serializedData = jntmsgs.add_msgsdata();
    jnt.SerializeToString(serializedData);

    if(bson.getField("timestamp").Number() > buffered)
      buffered = bson.getField("timestamp").Number();
  }

  msgs::SceneJoint tmp;
  jntmsgs.set_msgtype(tmp.GetTypeName());

  #ifdef USE_RRD
  {
    boost::mutex::scoped_lock lock(*__outmutex);
    __outmsgcounts["RC Control"]++;
    __outmsgcounts["Overall"]++;
    __outmsgsizes["RC Control"] += jntmsgs.ByteSize();
    __outmsgsizes["Overall"] += jntmsgs.ByteSize();
  }
  #endif

  robotcontrollerControlPub->Publish(jntmsgs);

  return buffered;
}

void
GazeboSceneThread::draw_lasers(double time) {
  BSONObj obj;
  std::map<std::string, bool>::iterator iter;
  for(iter = __lasercollections.begin(); iter != __lasercollections.end(); iter++) {
    msgs::Drawing drw;
    drw.set_name(__mongodb->nsGetCollection(iter->first));
    drw.set_visible(iter->second);
    drw.set_material("SceneReconstruction/Laser");
    drw.set_mode(msgs::Drawing::LINE_LIST);

    if(iter->second) {
      // get latest stored values for the laser interface
      obj = __mongodb->findOne(iter->first, QUERY("timestamp" << LTE << time).sort("timestamp", -1));

      // set all points
      std::vector<BSONElement> distances;
      std::vector<BSONElement>::iterator dist;
      distances = obj.getField("distances").Array();
      // negate to achieve correct behavior
      bool clockwise = !(obj.getField("clockwise_angle").Bool());
      double angle = 0.0;
      double angle_step = 360.0 / distances.size();
      if(!clockwise)
        angle_step *= -1;

      std::string frame = obj.getField("frame").String();
      long msec = obj.getField("timestamp").Long();
      msec -= (long) __dbcurtimeoffset;
      msec += __scenestart->in_msec();
      Time tftime(msec);
      tf::StampedTransform transform;
      if(tf_listener->can_transform("/map", frame, tftime)) {
        tf_listener->lookup_transform("/map", frame, tftime, transform);
      }
      else if(tf_listener->can_transform(frame, "/map", tftime)) {
        tf_listener->lookup_transform(frame, "/map", tftime, transform);
        transform.set_data(transform.inverse());
      }

      for(dist = distances.begin(); dist != distances.end(); dist++) {
        msgs::Vector3d *p0 = drw.add_point()->mutable_position();
        msgs::Vector3d *p1 = drw.add_point()->mutable_position();

        gazebo::math::Vector3 mp0(0.0, 0.0, 0.00);
        gazebo::math::Vector3 mp1(dist->Double(), 0.0, 0.00);
        gazebo::math::Quaternion rot(0.0, 0.0, angle*M_PI/180);
        mp1 = rot.RotateVector(mp1);

        //transform laser coords
        tf::Vector3 pos1(mp0.x,mp0.y,mp0.z);
        tf::Quaternion ori1(0.0,0.0,0.0,1.0);
        tf::Pose pose1(ori1, pos1);
        tf::Vector3 pos2(mp1.x,mp1.y,mp1.z);
        tf::Quaternion ori2(0.0,0.0,0.0,1.0);
        tf::Pose pose2(ori2, pos2);
        tf::Stamped<tf::Pose> stamped_pose1(pose1, tftime, frame);
        tf::Stamped<tf::Pose> result_pose1;
        tf::Stamped<tf::Pose> stamped_pose2(pose2, tftime, frame);
        tf::Stamped<tf::Pose> result_pose2;
        result_pose1.set_data(transform * stamped_pose1);
        result_pose2.set_data(transform * stamped_pose2);
        tf::Vector3 vec = result_pose1.getOrigin();
        mp0.Set(vec.getX(), vec.getY(), vec.getZ());
        vec = result_pose2.getOrigin();
        mp1.Set(vec.getX(), vec.getY(), vec.getZ());

        *p0 = msgs::Convert(mp0);
        *p1 = msgs::Convert(mp1);

        angle += angle_step;

        // limit angle to values from interval [0.0, 360.0)
        double tmp_d = angle / 360.0;
        int tmp_i = static_cast<int>(tmp_d);
        if(tmp_i > tmp_d)
          tmp_i--;
        angle = angle - static_cast<double>(tmp_i)*360.0;
      }

    #ifdef USE_RRD
    {
      boost::mutex::scoped_lock lock(*__outmutex);
      __outmsgcounts["RC Drawing"]++;
      __outmsgcounts["Overall"]++;
      __outmsgsizes["RC Drawing"] += drw.ByteSize();
      __outmsgsizes["Overall"] += drw.ByteSize();
    }
    #endif

    drawingPub->Publish(drw);
    }
  }
}

void
GazeboSceneThread::fill_document_message(BSONObj &bson, msgs::SceneDocument &doc, std::string collection)
{
  std::string json;
  json = bson.toString();
  doc.set_document(json);
  doc.set_interface(__mongodb->nsGetCollection(collection));
  doc.set_timestamp(bson.getField("timestamp").Number() - __dbtimeoffset);
  // get image/pointcloud if available
  std::string gridname = __mongodb->nsGetCollection(collection);
  size_t dot1 = gridname.find(".");
  gridname = gridname.substr(0, dot1);

  // Image reconstruction and conversion using the PNGWriter
  if(collection.find("Image") != std::string::npos) {
    std::string filename = bson.getFieldDotted("image.data.filename").String();
    std::string colorspace = bson.getFieldDotted("image.colorspace").String();
    unsigned int width = (unsigned int) bson.getFieldDotted("image.width").Int();
    unsigned int height = (unsigned int) bson.getFieldDotted("image.height").Int();
    if(colorspace == "YUV422_PLANAR") {    // other formats not (yet) supported by PNGWriter
      // process image format
      GridFile file = __mongogrids[gridname]->findFile(filename);
      if(file.exists()) {
        std::stringstream str_buffer;
        file.write(str_buffer);
        size_t len = str_buffer.str().length();
        unsigned char* buffer = new unsigned char [len+1];
        strcpy(reinterpret_cast<char*>(buffer), str_buffer.str().c_str());
        PNGWriter png("png_temp_out.png", width, height);
        png.set_buffer(colorspace_by_name(colorspace.c_str()), buffer);
        png.write();
        common::Image img("png_temp_out.png");
        msgs::Set(doc.mutable_image(), img);
      }
    }
  }

  // PointCloud reconstruction and conversion
  // using code from Tim Niemueller's MongoDB PCL merge plugin
  else if (collection.find("PointCloud") != std::string::npos) {
    BSONObj pcldoc = bson.getObjectField("pointcloud");
    std::string filename = pcldoc.getFieldDotted("data.filename").String();
    GridFile file = __mongogrids[gridname]->findFile(filename);
    if(file.exists()) {
      msgs::Drawing *pcl = doc.mutable_pointcloud();
      pcl->set_name("pointcloud");
      pcl->set_visible("true");
      pcl->set_mode(msgs::Drawing::POINT_LIST);
      pcl->set_material("SceneReconstruction/PointCloud");

      long msec = bson.getField("timestamp").Long();
      msec -= (long) __dbcurtimeoffset;
      msec += __scenestart->in_msec();
      std::string frame = pcldoc["frame_id"].String();
      Time tftime(msec);
      tf::StampedTransform transform;
      if(tf_listener->can_transform("/map", frame, tftime)) {
        tf_listener->lookup_transform("/map", frame, tftime, transform);
      }
      else if(tf_listener->can_transform(frame, "/map", tftime)) {
        tf_listener->lookup_transform(frame, "/map", tftime, transform);
        transform.set_data(transform.inverse());
      }

      tf::Quaternion q = transform.getRotation();
      Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
      tf::Vector3 v = transform.getOrigin();
      Eigen::Vector3f origin(v.x(), v.y(), v.z());

      if(collection.find("xyzrgb") != std::string::npos) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr lpcl(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB> rpcl;
        lpcl->points.resize(pcldoc["num_points"].Int());

        size_t bytes = 0;
        char *tmp = (char *)&lpcl->points[0];
        for (int c = 0; c < file.getNumChunks(); ++c) {
          GridFSChunk chunk = file.getChunk(c);
          int len = 0;
          const char *chunk_data = chunk.data(len);
          memcpy(tmp, chunk_data, len);
          tmp += len;
          bytes += len;
        }

        pcl::transformPointCloud(*lpcl, rpcl, origin, rotation);

        for (unsigned int i = 0; i < rpcl.points.size(); ++i) {
          msgs::Drawing::Point *p = pcl->add_point();
          p->mutable_position()->set_x(rpcl.points[i].x);
          p->mutable_position()->set_y(rpcl.points[i].y);
          p->mutable_position()->set_z(rpcl.points[i].z);
          p->mutable_color()->set_r(rpcl.points[i].r);
          p->mutable_color()->set_g(rpcl.points[i].g);
          p->mutable_color()->set_b(rpcl.points[i].b);
        }
      }
      else if(collection.find("xyz") != std::string::npos) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr lpcl(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ> rpcl;
        lpcl->points.resize(pcldoc["num_points"].Int());
        size_t bytes = 0;
        char *tmp = (char *)&lpcl->points[0];
        for (int c = 0; c < file.getNumChunks(); ++c) {
          GridFSChunk chunk = file.getChunk(c);
          int len = 0;
          const char *chunk_data = chunk.data(len);
          memcpy(tmp, chunk_data, len);
          tmp += len;
          bytes += len;
        }

        pcl::transformPointCloud(*lpcl, rpcl, origin, rotation);

        for (unsigned int i = 0; i < rpcl.points.size(); ++i) {
          msgs::Drawing::Point *p = pcl->add_point();
          p->mutable_position()->set_x(rpcl.points[i].x);
          p->mutable_position()->set_y(rpcl.points[i].y);
          p->mutable_position()->set_z(rpcl.points[i].z);
        }
      }
    }
  }
}
