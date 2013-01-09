//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$             */
/*                                                                      */
/* Description: This is the occ-grid interface for colli_a*,            */
/*              the search algorithm searches on.                       */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This interface is mainly out of implementation reasons given   */
/*       here. It includes a occ-grid and the laserinterface, so no one */
/*       else has to care about.                                        */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _C_COLLI_LASEROCCUPANCY_GRID_H_
#define _C_COLLI_LASEROCCUPANCY_GRID_H_


//#include <utils/occupancygrid/occupancygrid.h>
//#include <utils/roboshape/roboshape.h>
//#include <utils/geometry/ellipse.h>
//#include <utils/geometry/circle.h>
//#include <utils/geometry/trig_table.h>
#include "../robo-utils/rob/robo_laser.h"
#include <logging/logger.h>
#include <config/config.h>
#include <interfaces/Laser360Interface.h>
#include <utils/math/angle.h>
#include <geometry/hom_point.h>

#include "../robo-utils/roboshape.h"
#include "../robo-utils/geometry/trig_table.h"
#include "../robo-utils/geometry/ellipse.h"
#include "ellipse_map.h"
#include "../robo-utils/occupancygrid/occupancygrid.h"
 
#include <stdio.h>
#include <iostream>

// also defined in ellipse.h
// should be deleted here
#ifndef _COLLI_CELL_CONSTANTS_
#define _COLLI_CELL_CONSTANTS_     1
#define _COLLI_CELL_OCCUPIED_   1000.0
#define _COLLI_CELL_NEAR_          4.0 // near an obstacle    | COST  6!
#define _COLLI_CELL_MIDDLE_        3.0 // rel.near an obstacle| COST  4!
#define _COLLI_CELL_FAR_           2.0 // far from an obstacle| COST  2!
#define _COLLI_CELL_FREE_          1.0 // default free value  | COST  1!
#endif

using namespace fawkes;
using namespace std;

/** CLaserOccupancyGrid.
 *  This OccGrid is derived by the Occupancy Grid originally from Andreas Strack,
 *    but modified for speed purposes.
 */
class CLaserOccupancyGrid : public OccupancyGrid 
{
public:

  // Constructor.
  CLaserOccupancyGrid( Logger* logger, Configuration *config, Laser * laser, int width = 150, int height = 150,
		       int cell_width = 5, int cell_height = 5 );
  

  // Destructor
  ~CLaserOccupancyGrid();
  

  /** Put the laser readings in the occupancy grid
   *  Also, every reading gets a radius according to the relative direction  
   *  of this reading to the robot.
   *  @param midX is the current point of the robot.
   *  @param midY is the current point of the robot.
   *  @param inc is the current constant to increase the obstacles.
   */
  void UpdateOccGrid( int midX, int midY, float inc, float vel, 
		      float xdiff, float ydiff, float oridiff );
  
  
  /** 
   *  Reset all old readings and forget about the world state!
   */
  void ResetOld( int max_age = -1 );


//private:
  float normalize_degree(float angle_deg);

  /** 
   *  Integrate historical readings to the current occgrid.
   */
  void IntegrateOldReadings( int midX, int midY, float inc, float vel, 
			     float xdiff, float ydiff, float oridiff );


  /** 
   *  Integrate the current readings to the current occgrid.
   */
  void IntegrateNewReadings( int midX, int midY, float inc, float vel );


  /** 
   *  Check if the current value is contained in the history.
   */
  bool Contained( float p_x, float p_y );


  /** 
   * Integrate a single ellipse
   * 
   * @param ellipse the ellipse that is to be integrated
   */
  void integrateObstacle( Ellipse ellipse );

  //  void integrateEllipseCell( int centerx, int centery,
  //			     int i, int j, float prob );

  // pointer to the laser
   Laser * m_pLaser;
  //Laser360Interface *m_pLaser;

  // my roboshape
  RoboShape * m_pRoboShape;

  // fast trigonometry table
  TrigTable * m_pTrigTable;

  // History
  std::vector< float > m_vOldReadings;

  // History concerned constants
  int m_MaxHistoryLength, m_MinHistoryLength, m_InitialHistorySize;

  // Trigonometry table its resolution
  int m_TrigTableResolution;
  
  // Laser concerned settings
  float m_MinimumLaserLength, m_EllipseDistance;

  int m_RobocupMode;

  CEllipseMap * ellipse_map;

  Logger* loggerGrid;

//  int m_CellWidth,m_CellHeight;
//  int m_Width,m_Height;

  inline float sqr( float x )
  {
    return (x*x);
  }


};

#endif
