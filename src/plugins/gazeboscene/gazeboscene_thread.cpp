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

// from MongoDB
#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>

// from Gazebo
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.h>

using namespace mongo;
using namespace gazebo;
using namespace fawkes;


/** @class GazeboSceneThread "gazeboscene_thread.h"
 *  Gazebo Scene Reconstruction Thread.
 *  This thread gets the previously recorded data from the MongoDB and
 *  packs it into gazebo::msgs to alter the world inside the simulator.
 *
 *  @author Bastian Klingen
 */

/** Constructor. */
GazeboSceneThread::GazeboSceneThread()
  : Thread("GazeboSceneThread", Thread::OPMODE_WAITFORWAKEUP),
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
  __database = "fflog";
  try {
    __database = config->get_string("/plugins/gazeboscene/database");
    logger->log_info(name(), "GazeboScene uses database: %s",
		       __database.c_str());
  }
  catch (Exception &eg) {
    logger->log_info(name(), "No database for GazeboScene configured, trying MongoLog's database",
		       __database.c_str());
    try {
      __database = config->get_string("/plugins/mongolog/database");
    } catch (Exception &em) {
      logger->log_info(name(), "No database configured, reading from %s",
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

  std::list<std::string>::iterator iter;
  std::list<std::string> collections;
  collections = __mongodb->getCollectionNames(__database);
  double starttime = DBL_MAX;
  double endtime = 0.0;

  BSONObj obj;
  BSONObj returnfields = fromjson("{timestamp:1}");
  BSONElement timestamp;
  for(iter = collections.begin(); iter != collections.end(); iter++) {
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
      logger->log_debug(name(), "creating mongo::GridFS %s for %s", collectionname.substr(dot1, dot2 - dot1).c_str(), collectionname.substr(0, dot2).c_str());
      __mongogrids[collectionname.substr(dot1, dot2 - dot1)] = new GridFS(*__mongodb, __database, collectionname.substr(0, dot2));
    }

    // determine collections containing transforms
    if(iter->find("TransformInterface") != std::string::npos) {
      __tfcollections.push_back(*iter);
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

  objectinstantiatorResponsePub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/ObjectInstantiator/Response");
  objectinstantiatorObjectPub = __gazebo->Advertise<msgs::SceneObject_V>("~/SceneReconstruction/ObjectInstantiator/Object");
  robotcontrollerResponsePub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/RobotController/Response");
  robotcontrollerControlPub = __gazebo->Advertise<msgs::Message_V>("~/SceneReconstruction/RobotController/");
  scenereconstructionGuiPub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/GUI/MongoDB");
  setupPub = __gazebo->Advertise<gazebo::msgs::SceneRobotController>("~/SceneReconstruction/RobotController/Init");
  statusPub = __gazebo->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Availability/Response"));
  transformPub = __gazebo->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Response"));
  worldcontrolPub = __gazebo->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  timePub = __gazebo->Advertise<msgs::Double>(std::string("~/SceneReconstruction/GUI/Time"));

  // send availability message in case the request was not received
  msgs::Response response;
  response.set_id(-1);
  response.set_request("status");
  response.set_response("Framework");  
  this->statusPub->Publish(response);

  // send length of the scene to the GUI
  msgs::Double time;
  time.set_data(__dbscenelength);
  this->timePub->Publish(time);

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

  response.set_id(-1);
  response.set_request("collection_names");
  response.set_response("success");
  msgs::String_V src;
  response.set_type(src.GetTypeName());

  for(iter = collections.begin(); iter != collections.end(); iter++) {
      src.add_data(__mongodb->nsGetCollection(*iter));
  }

  std::string *serializedData = response.mutable_serialized_data();
  src.SerializeToString(serializedData);
  scenereconstructionGuiPub->Publish(response);

  frameworkRequestSub = __gazebo->Subscribe("~/SceneReconstruction/Framework/Request", &GazeboSceneThread::OnRequestMsg, this);
  transformRequestSub = __gazebo->Subscribe("~/SceneReconstruction/Framework/TransformRequest", &GazeboSceneThread::OnTransformRequestMsg, this);
  statusSub = __gazebo->Subscribe(std::string("~/SceneReconstruction/GUI/Availability/Request/Framework"), &GazeboSceneThread::OnStatusMsg, this);
  controlSub = __gazebo->Subscribe("~/SceneReconstruction/Framework/Control", &GazeboSceneThread::OnControlMsg, this);
  worldSub = __gazebo->Subscribe("~/world_stats", &GazeboSceneThread::OnWorldStatisticsMsg, this);
}


/** Finalize.
 *  cleans up everything
 */
void
GazeboSceneThread::finalize()
{
  objectinstantiatorResponsePub.reset();
  objectinstantiatorObjectPub.reset();
  robotcontrollerResponsePub.reset();
  robotcontrollerControlPub.reset();
  scenereconstructionGuiPub.reset();
  statusPub.reset();
  transformPub.reset();
  frameworkRequestSub->Unsubscribe();
  frameworkRequestSub.reset();
  statusSub->Unsubscribe();
  statusSub.reset();
}


/** Loop.
 *  Feed recorded transforms for the current scene time so the 
 *  "transform tree" can be build and used to convert coordinates.
 */
void
GazeboSceneThread::loop()
{
}

void
GazeboSceneThread::OnWorldStatisticsMsg(ConstWorldStatisticsPtr &_msg)
{
  if(!__pause) {
    // workaround for deprecated auto_ptr
    double maxtime = _msg->sim_time().sec()*1000 + _msg->sim_time().nsec()/1000000 + __dbcurtimeoffset + __dbbuffer;

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
  }
}

void
GazeboSceneThread::OnControlMsg(ConstSceneFrameworkControlPtr &_msg)
{
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

  if(_msg->has_change_offset()) {
    if(_msg->change_offset() && _msg->has_offset()) {
      __dbcurtimeoffset = __dbtimeoffset + _msg->offset();

      // adjust __scenestart for the new offset
      __scenestart->stamp_systime();

      // feed transforms of the previous 10 seconds
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
    }
  }
}

void
GazeboSceneThread::OnRequestMsg(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");

  if(_msg->request() == "collection_names") {
    msgs::String_V src;
    response.set_type(src.GetTypeName());

    std::list<std::string> collections;
    collections = __mongodb->getCollectionNames(__database);

    std::list<std::string>::iterator iter;
    for(iter = collections.begin(); iter != collections.end(); iter++) {
        src.add_data(__mongodb->nsGetCollection(*iter));
    }

    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);
    scenereconstructionGuiPub->Publish(response);
  }
  else if(_msg->request() == "select_collection") {
    std::list<std::string> collections;
    collections = __mongodb->getCollectionNames(__database);
    if(_msg->has_data() && find(collections.begin(), collections.end(), __database+"."+_msg->data()) != collections.end()) {
      __collection = _msg->data();
      msgs::String_V src;
      response.set_type(src.GetTypeName());
      std::stringstream conv;
      conv << __mongodb->count(__database+"."+__collection);
      src.add_data(conv.str());

      // workaround for deprecated auto_ptr
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      auto_ptr<DBClientCursor> cursor = __mongodb->query(__database+"."+__collection, Query("{}"), 1, 0);
      #pragma GCC diagnostic warning "-Wdeprecated-declarations"
      if(cursor->more()) {
        BSONObj obj = cursor->next();
        src.add_data(obj.toString());
      }

      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
      scenereconstructionGuiPub->Publish(response);

    }
    else {
      response.set_response("failure");
      msgs::String src;
      response.set_type(src.GetTypeName());
      src.set_data("the given collection is unknown to the framework");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
    }

    scenereconstructionGuiPub->Publish(response);
  }
  else if(_msg->request() == "select_object") {
    if(_msg->has_dbl_data()) {
      // workaround for deprecated auto_ptr
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      auto_ptr<DBClientCursor> cursor = __mongodb->query(__database+"."+__collection, Query("{}"), 1, (int)_msg->dbl_data());
      #pragma GCC diagnostic warning "-Wdeprecated-declarations"
      if(cursor->more()) {
        BSONObj obj = cursor->next();
        msgs::String src;
        response.set_type(src.GetTypeName());
        src.set_data(obj.toString());
        std::string *serializedData = response.mutable_serialized_data();
        src.SerializeToString(serializedData);
      }
    }
    else {
      response.set_response("failure");
      msgs::String src;
      response.set_type(src.GetTypeName());
      src.set_data("the given database is unknown to the framework");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
    }

    scenereconstructionGuiPub->Publish(response);
  }
  else if(_msg->request() == "object_data") {
    if(_msg->has_data()) {
      msgs::SceneObjectData src;
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      auto_ptr<DBClientCursor> cursor;
      #pragma GCC diagnostic warning "-Wdeprecated-declarations"
      BSONObj bson;
      std::list<std::string> collections;
      std::list<std::string>::iterator iter;
      collections = __mongodb->getCollectionNames(__database);
      for(iter = collections.begin(); iter != collections.end(); iter++) {
        if(*iter != __database+".system.indexes" && iter->find("GridFS") == std::string::npos) {
          cursor = __mongodb->query(*iter, fromjson(_msg->data()));
          while(cursor->more()) {
            bson = cursor->next();
            // TODO: fill SceneObjectData message
            //json = bson.toString();
            // get image if available
            //std::string gridname = __mongodb->nsGetCollection(*iter);
            //dot1 = gridname.find(".")+1;
            //dot2 = gridname.find(".", dot1);
            //gridname = gridname.substr(dot1, dot2 - dot1);
            //if(iter->find("PointCloud") != std::string::npos) {
            //  std::string filename = bson.getFieldDotted("pointcloud.data.filename").String();
            //  GridFile file = __mongogrids[gridname]->findFile(filename);
            // TODO: convert raw data to something useful
            //}
            //else if(iter->find("Image") != std::string::npos) {
            //  std::string filename = bson.getFieldDotted("image.data.filename").String();
            //  GridFile file = __mongogrids[gridname]->findFile(filename);
            // TODO: convert raw data to something useful
            //}
          }
        }
      }
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
    }
    else {
      response.set_response("failure");
      msgs::String src;
      response.set_type(src.GetTypeName());
      src.set_data("the given object reference is unknown to the framework");
      std::string *serializedData = response.mutable_serialized_data();
      src.SerializeToString(serializedData);
    }

    objectinstantiatorResponsePub->Publish(response);
  }
  else {
    response.set_response("unknown");
    msgs::String src;
    response.set_type(src.GetTypeName());
    src.set_data("the given request is unknown to the framework");
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);
    scenereconstructionGuiPub->Publish(response);
  }
}

void
GazeboSceneThread::OnTransformRequestMsg(ConstTransformRequestPtr &_msg)
{
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
    Time now(clock);
    now.stamp_systime();
    tf::Stamped<tf::Pose> stamped_pose(pose, now, _msg->source_frame());
    tf::Stamped<tf::Pose> result_pose;
//    tf_listener->transform_pose(_msg->target_frame(), stamped_pose, result_pose);

    src = Convert(stamped_pose);
    std::string *serializedData = response.mutable_serialized_data();
    src.SerializeToString(serializedData);
    transformPub->Publish(response);
  }
}

void
GazeboSceneThread::OnStatusMsg(ConstRequestPtr &_msg)
{
  if(_msg->request() == "status") {
    msgs::Response response;
    response.set_id(_msg->id());
    response.set_request(_msg->request());
    response.set_response("Framework");
    this->statusPub->Publish(response);

    msgs::Double time;
    time.set_data(__dbscenelength);
    this->timePub->Publish(time);
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
  // workaround for deprecated auto_ptr
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto_ptr<DBClientCursor> cursor;
  #pragma GCC diagnostic warning "-Wdeprecated-declarations"
  BSONObj bson;
  double buffered = start;
  std::list<std::string>::iterator iter;
  msgs::SceneObject_V objmsgs;
  for(iter = __poscollections.begin(); iter != __poscollections.end(); iter++) {
    cursor = __mongodb->query(*iter, QUERY("timestamp" << GTE << start << LT << end).sort("timestamp"));
    while(cursor->more()) {
      bson = cursor->next();
      std::string objectname = __mongodb->nsGetCollection(*iter);
      objectname = objectname.substr(objectname.find(".")+1);
      objmsgs.add_object(objectname);
      objmsgs.add_visible(bson.getField("visibility_history").Int()>0);

      msgs::Pose *p = objmsgs.add_pose();
      std::vector<BSONElement> trans = bson.getField("translation").Array();
      std::vector<BSONElement> rota = bson.getField("rotation").Array();
      

      tf::Vector3 pos(trans[0].Double(), trans[1].Double(), trans[2].Double());
      tf::Quaternion ori(rota[0].Double(), rota[1].Double(), rota[2].Double(), rota[3].Double());
      tf::Pose pose(ori, pos);
      long msec = bson.getField("timestamp").Long();
      msec -= (long) __dbcurtimeoffset;
      msec += __scenestart->in_msec();
      std::string frame = bson.getField("frame").String();
      objmsgs.add_frame(frame);
      Time tftime(msec);
      tf::Stamped<tf::Pose> stamped_pose(pose, tftime, frame);
      tf::Stamped<tf::Pose> result_pose;

      if(tf_listener->can_transform("/map", frame, tftime)) {
        tf::StampedTransform transform;
        tf_listener->lookup_transform("/map", frame, tftime, transform);
        result_pose.set_data(transform * stamped_pose);
        *p = Convert(result_pose);
      }
      else if(tf_listener->can_transform(frame, "/map", tftime)) {
        tf::StampedTransform transform;
        tf_listener->lookup_transform(frame, "/map", tftime, transform);
        result_pose.set_data(transform.inverseTimes(stamped_pose));
        *p = Convert(result_pose);
      }
      else {
        math::Pose invispose(0.0, 0.0, -10.0, 0.0, 0.0, 0.0);
        *p = msgs::Convert(invispose);
      }

      double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
      objmsgs.add_time(time);

      std::string query = QUERY("timestamp" << GT << __querystarttimes[*iter] << LTE << bson.getField("timestamp").Number()).sort("timestamp").toString();
      objmsgs.add_query(query);

      __querystarttimes[*iter] = bson.getField("timestamp").Number();
      if(bson.getField("timestamp").Number() > buffered)
        buffered = bson.getField("timestamp").Number();
    }
  }

  objectinstantiatorObjectPub->Publish(objmsgs);

  return buffered;
}

double
GazeboSceneThread::buffer_positions(double start, double end) {
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

    rob.set_pos_x(trans[0].Double());
    rob.set_pos_y(trans[1].Double());
    rob.set_pos_z(trans[2].Double());
    rob.set_ori_x(rota[0].Double());
    rob.set_ori_y(rota[1].Double());
    rob.set_ori_z(rota[2].Double());
    rob.set_ori_w(rota[3].Double());

    double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
    rob.set_controltime(time);

    std::string *serializedData = robmsgs.add_msgsdata();
    rob.SerializeToString(serializedData);

    if(bson.getField("timestamp").Number() > buffered)
      buffered = bson.getField("timestamp").Number();
  }

  robotcontrollerControlPub->Publish(robmsgs);

  return buffered;
}

double
GazeboSceneThread::buffer_joints(double start, double end) {
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

  robotcontrollerControlPub->Publish(jntmsgs);

  return buffered;
}
