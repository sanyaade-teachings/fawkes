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

  __dblookahead = 500.0;  // in ms
  try {
    __database = (double) config->get_int("/plugins/gazeboscene/lookahead");
    logger->log_info(name(), "GazeboScene uses a lookahead of %.0fms", __dblookahead);
  }
  catch (Exception &e) {
    logger->log_info(name(), "No lookahead for GazeboScene configured, defaulting to %.0fms", __dblookahead);
  }

  __mongodb    = mongodb_client;
  __gazebo     = gazebonode;

  // define the zero time for the transforms
  __scenestart = new Time(clock);
  __now = new Time(clock);
  __scenestart->stamp_systime();
  __now->stamp_systime();
  __lasttf = new Time();
  __lasttf->set_time(__scenestart);
  __pause      = true;
  __pausestart = new Time();
  __pausestart->stamp_systime();

  std::list<std::string> collections;
  collections = __mongodb->getCollectionNames(__database);
  double starttime = DBL_MAX;
  double endtime = 0.0;

  std::list<std::string>::iterator iter;
  BSONObj obj;
  BSONObj returnfields;
  returnfields = fromjson("{timestamp:1}");
  BSONElement timestamp;
  for(iter = collections.begin(); iter != collections.end(); iter++) {
    // index the database and determine the length of the scene
    if(*iter != __database+".system.indexes") {
      __mongodb->ensureIndex(*iter, fromjson("{timestamp:1}"), true);
      obj = __mongodb->findOne(*iter, Query().sort("timestamp",1), &returnfields);
      if(!obj.isEmpty()) {
        timestamp = obj.getField("timestamp");
        if(starttime > timestamp.Number()) 
          starttime = timestamp.Number();
      }
      obj = __mongodb->findOne(*iter, Query().sort("timestamp",-1), &returnfields);
      if(!obj.isEmpty()) {
        timestamp = obj.getField("timestamp");
        if(endtime < timestamp.Number())
          endtime = timestamp.Number();
      }
    }

    // determine collections containing transforms
    if(iter->find("TransformInterface") != std::string::npos) {
      __tfcollections.push_back(*iter);
    }
  }

  __dbtimeoffset = starttime;
  __dbcurtimeoffset = starttime;
  __dbscenelength = endtime - starttime;
  __dblookedahead = starttime;

  objectinstantiatorResponsePub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/ObjectInstantiator/Response");
  objectinstantiatorObjectPub = __gazebo->Advertise<msgs::SceneObject>("~/SceneReconstruction/ObjectInstantiator/Object");
  robotcontrollerResponsePub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/RobotController/Response");
  robotcontrollerControlPub = __gazebo->Advertise<msgs::Message_V>("~/SceneReconstruction/RobotController/");
  scenereconstructionGuiPub = __gazebo->Advertise<msgs::Response>("~/SceneReconstruction/GUI/MongoDB");
  setupPub = __gazebo->Advertise<gazebo::msgs::SceneRobotController>("~/SceneReconstruction/RobotController/Init");
  statusPub = __gazebo->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Availability/Response"));
  transformPub = __gazebo->Advertise<msgs::Response>(std::string("~/SceneReconstruction/GUI/Response"));
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
  // workaround for deprecated auto_ptr
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto_ptr<DBClientCursor> cursor;
  #pragma GCC diagnostic warning "-Wdeprecated-declarations"
  BSONObj bson;
  double start = __lasttf->in_msec() - __scenestart->in_msec() + __dbcurtimeoffset;
  double end = __now->in_msec() - __scenestart->in_msec() + __dbcurtimeoffset;

  // include transforms when __lasttf and __scenestart are equal while not publishing a transform twice
  if(start == __dbcurtimeoffset)
    start -= 0.1;

  for(iter = __tfcollections.begin(); iter != __tfcollections.end(); iter++) {
    cursor = __mongodb->query(*iter, QUERY("timestamp" << GT << start << LTE << end).sort("timestamp"));
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
    }
  }

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

  // TODO: get and set inital joint positions

  setupPub->Publish(robcon);

  // send commands and objects for the timeframe from 0 to __dblookahead
  double tmp = 0;
  double maxtime = __dbtimeoffset + __dblookahead;
  msgs::Message_V /* objmsgs, */ robmsgs /*, jntmsgs*/ ;
/*
  // get and send object data
  // TODO: set collection name(s)
  cursor = __mongodb->query(__database+".COLLECTION", QUERY("timestamp" << GTE << __dblookedahead << LT << maxtime).sort("timestamp");
  msgs::SceneObjectData obj;
  objmsgs.set_msgtype(obj.GetTypeName());
  while(cursor->more()) {
    bson = cursor->next();
    if(bson.getField("visible").Bool()) {
      // TODO: fill SceneObjectData message

//      obj.set_object_type();
//      obj.set_pos_x();
//      obj.set_pos_y();
//      obj.set_pos_z();
//      obj.set_ori_w();
//      obj.set_ori_x();
//      obj.set_ori_y();
//      obj.set_ori_z();
//      obj.set_frame();
//      obj.set_child_frame();
//      obj.set_objectids();
//      obj.set_name();

      double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
      obj.set_spawntime(time);

      std::string *serializedData = objmsgs.add_msgsdata();
      obj.SerializeToString(serializedData);
    }
  }

  objectinstantiatorObjectPub->Publish(objmsgs);

  if(bson.getField("timestamp").Number() > tmp)
    tmp = bson.getField("timestamp").Number();
*/
  // get and send robot positions
  cursor = __mongodb->query(__database+".TransformInterface.TF_RCSoftX_Localize", QUERY("timestamp" << GTE << __dblookedahead << LT << maxtime));
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
  }

  robotcontrollerControlPub->Publish(robmsgs);

  if(bson.getField("timestamp").Number() > tmp)
    tmp = bson.getField("timestamp").Number();
/*
  // get and send joint positions
  // TODO: set collection name(s)
  cursor = __mongodb->query(__database+".COLLECTION", QUERY("timestamp" << GTE << __dblookedahead << LT << maxtime));
  msgs::SceneJoint jnt;
  jntmsgs.set_msgtype(jnt.GetTypeName());
  while(cursor->more()) {
    bson = cursor->next();
    // TODO: fill SceneJoint message for all joints at the current time
//    std::string jointname;
//    jnt.add_joint(jointname);
//    double jointangle;
//    jnt.add_angle(jointangle);

    double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
    jnt.set_controltime(time);

    std::string *serializedData = jntmsgs.add_msgsdata();
    jnt.SerializeToString(serializedData);
  }

  robotcontrollerControlPub->Publish(jntmsgs);

  if(bson.getField("timestamp").Number() > tmp)
    tmp = bson.getField("timestamp").Number();
*/
  if(tmp > __dblookedahead)
    __dblookedahead = tmp;


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
  __now->stamp_systime();
  if(!__pause) {
    // workaround for deprecated auto_ptr
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    auto_ptr<DBClientCursor> cursor;
    #pragma GCC diagnostic warning "-Wdeprecated-declarations"
    std::list<std::string>::iterator iter;
    BSONObj bson;
    double start = __lasttf->in_msec() - __scenestart->in_msec() + __dbcurtimeoffset;
    double end = __now->in_msec() - __scenestart->in_msec() + __dbcurtimeoffset;

    // include transforms when __lasttf and __scenestart are equal while not publishing a transform twice
    if(start == __dbcurtimeoffset)
      start -= 0.1;

    for(iter = __tfcollections.begin(); iter != __tfcollections.end(); iter++) {
      cursor = __mongodb->query(*iter, QUERY("timestamp" << GT << start << LTE << end).sort("timestamp"));
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
      }
    }
  }
  __lasttf->set_time(__now);
}

void
GazeboSceneThread::OnWorldStatisticsMsg(ConstWorldStatisticsPtr &_msg)
{
  if(!__pause) {
    // workaround for deprecated auto_ptr
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    auto_ptr<DBClientCursor> cursor;
    #pragma GCC diagnostic warning "-Wdeprecated-declarations"
    BSONObj bson;
    double tmp = 0;
    double maxtime = _msg->sim_time().sec()*1000 + _msg->sim_time().nsec()/1000000 + __dbcurtimeoffset + __dblookahead;
    if(maxtime < __dblookedahead)
      return;

    msgs::Message_V /* objmsgs, */ robmsgs /*, jntmsgs */ ;
/*
    // TODO: set collection name(s)
    cursor = __mongodb->query(__database+".COLLECTION", QUERY("timestamp" << GTE << __dblookedahead << LT << maxtime).sort("timestamp");
    msgs::SceneObjectData obj;
    objmsgs.set_msgtype(obj.GetTypeName());
    while(cursor->more()) {
      bson = cursor->next();
      if(bson.getField("visible").Bool()) {
        // TODO: fill SceneObjectData message

//        obj.set_object_type();
//        obj.set_pos_x();
//        obj.set_pos_y();
//        obj.set_pos_z();
//        obj.set_ori_x();
//        obj.set_ori_y();
//        obj.set_ori_z();
//        obj.set_ori_w();
//        obj.set_frame();
//        obj.set_child_frame();
//        // TODO: collect objectids of images possibly related (timed around this object +- 0.5 secs?)
//        obj.set_objectids();
//        obj.set_name();

        double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
        obj.set_spawntime(time);

        std::string *serializedData = objmsgs.add_msgsdata();
        obj.SerializeToString(serializedData);
      }
    }

    robotcontrollerControlPub->Publish(objmsgs);

    if(bson.getField("timestamp").Number() > tmp)
      tmp = bson.getField("timestamp").Number();
*/    
    // get and send robot positions
    cursor = __mongodb->query(__database+".TransformInterface.TF_RCSoftX_Localize", QUERY("timestamp" << GTE << __dblookedahead << LT << maxtime));
    msgs::SceneRobot rob;
    robmsgs.set_msgtype(rob.GetTypeName());
    while(cursor->more()) {
      bson = cursor->next();
      std::vector<BSONElement> trans = bson.getField("translation").Array();
      std::vector<BSONElement> rota = bson.getField("rotation").Array();

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
    }

    robotcontrollerControlPub->Publish(robmsgs);

    if(bson.getField("timestamp").Number() > tmp)
      tmp = bson.getField("timestamp").Number();
/*
    // get and send joint positions
    // TODO: set collection name(s)
    cursor = __mongodb->query(__database+".COLLECTION", QUERY("timestamp" << GTE << __dblookedahead << LT << maxtime));
    msgs::SceneJoint jnt;
    jntmsgs.set_msgtype(jnt.GetTypeName());
    while(cursor->more()) {
      bson = cursor->next();
      // TODO: fill SceneJoint message for all joints at the current time
//      std::string jointname;
//      jnt.add_joint(jointname);
//      double jointangle;
//      jnt.add_angle(jointangle);

      double time = (bson.getField("timestamp").Number() - __dbcurtimeoffset) / 1000;
      jnt.set_controltime(time);

      std::string *serializedData = jntmsgs.add_msgsdata();
      jnt.SerializeToString(serializedData);
    }

    robotcontrollerControlPub->Publish(jntmsgs);

    if(bson.getField("timestamp").Number() > tmp)
      tmp = bson.getField("timestamp").Number();
*/
    if(tmp > __dblookedahead)
      __dblookedahead = tmp;
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
      *__lasttf += offset;
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
      __lasttf->set_time(__scenestart);

      // feed transforms of the previous 10 seconds
      // workaround for deprecated auto_ptr
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      auto_ptr<DBClientCursor> cursor;
      #pragma GCC diagnostic warning "-Wdeprecated-declarations"
      std::list<std::string>::iterator iter;
      BSONObj bson;
      double start = __dbcurtimeoffset - 10000;
      double end = __dbcurtimeoffset;
      __dblookedahead = __dbcurtimeoffset;
      for(iter = __tfcollections.begin(); iter != __tfcollections.end(); iter++) {
        cursor = __mongodb->query(*iter, QUERY("timestamp" << GT << start << LTE << end).sort("timestamp"));
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
        }
      }
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
      std::list<std::string> _objectids;
      std::list<std::string>::iterator _objectid;
      std::string objectids = _msg->data();
      std::size_t token_start = 0;
      std::size_t token_end   = objectids.find(";", token_start+1);
      while(token_end != std::string::npos) {
        _objectids.push_back(objectids.substr(token_start, token_end));
        token_start = token_end+1;
        token_end   = objectids.find(";", token_start+1);
      }

      msgs::SceneObjectData src;
      response.set_type(src.GetTypeName());
      src.set_objectids(objectids);
/*
      // TODO: fill SceneObjectData message
      for(_objectid = _objectids.begin(); _objectid != _objectids.end(); _objectid++) {
        msgs::SceneImages *sceneimg = src.add_images();
        sceneimg->set_objectid(*_objectid);
//        sceneimg->set_name();
        msgs::Image *img = sceneimg->mutable_image();
//        img->set_width();
//        img->set_height();
//        img->set_pixel_format();
//        img->set_step();
//        // TODO: convert raw image data to png format
//        img->set_data();
      }
*/
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

    tf::Vector3 pos(_msg->pos_x(), _msg->pos_y(), _msg->pos_z());
    tf::Quaternion ori(_msg->ori_x(), _msg->ori_y(), _msg->ori_z(), _msg->ori_w());
    tf::Pose pose(ori, pos);
    tf::Stamped<tf::Pose> stamped_pose(pose, *__now, _msg->source_frame());
    tf::Stamped<tf::Pose> result_pose;
    tf_listener->transform_pose(_msg->target_frame(), stamped_pose, result_pose);

    src = Convert(result_pose);
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
