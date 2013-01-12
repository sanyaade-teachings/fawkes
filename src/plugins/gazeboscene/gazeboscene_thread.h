
/***************************************************************************
 *  gazeboscene_thread.h - Gazebo Scene Reconstruction Thread
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

#ifndef __PLUGINS_GAZEBOSCENE_GAZEBOSCENE_THREAD_H_
#define __PLUGINS_GAZEBOSCENE_GAZEBOSCENE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>
#include <plugins/mongodb/aspect/mongodb.h>
#include <plugins/gazebo/aspect/gazebo.h>

#ifdef USE_RRD
#include <plugins/rrd/aspect/rrd.h>
#endif

#include <tf/types.h>

#include <string>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>

namespace mongo {
  class GridFS;
  class BSONObj;
}

class GazeboSceneThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect,
  public fawkes::MongoDBAspect,
  #ifdef USE_RRD
  public fawkes::RRDAspect,
  #endif
  public fawkes::GazeboAspect
{
 public:
  GazeboSceneThread();
  virtual ~GazeboSceneThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void OnTimeMsg(ConstDoublePtr&);
  void OnRequestMsg(ConstRequestPtr&);
  void OnTransformRequestMsg(ConstTransformRequestPtr&);
  void OnWorldStatisticsMsg(ConstWorldStatisticsPtr&);
  void OnControlMsg(ConstSceneFrameworkControlPtr&);
  void OnLaserMsg(ConstLasersPtr&);
  gazebo::msgs::Pose Convert(fawkes::tf::Stamped<fawkes::tf::Pose>);
  double send_transforms(double,double);
  double buffer_objects(double,double);
  double buffer_positions(double,double);
  double buffer_joints(double,double);
  void draw_lasers(double);
  void fill_document_message(mongo::BSONObj&, gazebo::msgs::SceneDocument&, std::string);

  mongo::DBClientBase                     *__mongodb;
  std::map< std::string, mongo::GridFS* >  __mongogrids;
  std::map< std::string, double >          __querystarttimes;
  gazebo::transport::NodePtr               __gazebo;
  std::string                              __database,
                                           __collection;
  std::list<std::string>                   __collections,
                                           __tfcollections,
                                           __poscollections;
  std::map< std::string, bool >            __lasercollections;
  fawkes::Time                            *__scenestart,
                                          *__pausestart;
  double                                   __dbtimeoffset,
                                           __dbcurtimeoffset,
                                           __dbscenelength,
                                           __dbbuffered,
                                           __dbcurrenttime,
                                           __lasttf;
  unsigned int                             __dbbuffer;
  bool                                     __pause,
                                           __step,
                                           __available,
                                           __buffering;
  
  

  gazebo::transport::PublisherPtr          objectinstantiatorResponsePub,
                                           objectinstantiatorObjectPub,
                                           robotcontrollerControlPub,
                                           scenereconstructionGuiPub,
                                           timePub,
                                           setupPub,
                                           transformPub,
                                           worldcontrolPub,
                                           drawingPub,
                                           lasersPub,
                                           statusPub;
  gazebo::transport::SubscriberPtr         frameworkRequestSub,
                                           transformRequestSub,
                                           controlSub,
                                           laserSub,
                                           worldSub;
  #ifdef USE_RRD
  boost::mutex                            *__outmutex;
  boost::mutex                            *__inmutex;
  fawkes::Time                            *__rrdupdate;
  fawkes::RRDDefinition                   *__inmsgcount_rrd;
  fawkes::RRDGraphDefinition              *__inmsgcount_graph;
  std::map<std::string, int>               __inmsgcounts;
  fawkes::RRDDefinition                   *__inmsgsize_rrd;
  fawkes::RRDGraphDefinition              *__inmsgsize_graph;
  std::map<std::string, int>               __inmsgsizes;
  fawkes::RRDDefinition                   *__outmsgcount_rrd;
  fawkes::RRDGraphDefinition              *__outmsgcount_graph;
  std::map<std::string, int>               __outmsgcounts;
  fawkes::RRDDefinition                   *__outmsgsize_rrd;
  fawkes::RRDGraphDefinition              *__outmsgsize_graph;
  std::map<std::string, int>               __outmsgsizes;
  #endif
};

#endif
