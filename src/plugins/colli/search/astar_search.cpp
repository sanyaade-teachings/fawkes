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
/* $Id$           */
/*                                                                      */
/* Description: This is the interpretation class implementation for A*  */
/*              of Colli-A*                                             */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This class tries to translate the found plan to interpreteable */
/*       things for the rest of the program.                            */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_ASTARSEARCH_CPP_
#define _COLLI_ASTARSEARCH_CPP_


//#include <utils/configfile/configfile.h>
#include "astar_search.h"
#include <geometry/hom_point.h>

using namespace std;


// Constructor. Constructs the plan, initializes an A* Object and 
//  makes a reference to the OccupancyGrid.
CSearch::CSearch( Logger* logger, Configuration *config, CLaserOccupancyGrid * occGrid ) :
  CAbstractSearch( logger, occGrid )
{
  loggerAstar = logger;
  loggerAstar->log_info("CSearch","CSearch(Constructor): Entering \n");
  m_pAStar = new CAStar( logger, config, occGrid );
  
  /*string confFileName = "../cfg/robocup/colli.cfg";
  try 
    {
      ConfigFile * m_pConf = new ConfigFile( confFileName );
      
      m_RobocupMode = m_pConf->integer( "Colli_ROBOCUP_MODE" );


#ifdef _COLLI_VISUALIZE_
      m_pVis = 0;
      if ( m_pConf->integer( "ColliVis" ) == 1 )
	{
      	  m_pVis = new CVisualize( );
       	  int argc = 1;
       	  char * argv = "Colli-Vis";
      	  m_pVis->init_vwin( argc, &argv, 2*m_pOccGrid->getWidth(), 2*m_pOccGrid->getHeight() );
      	  m_pVis->draw_field();
      	  m_pVis->myXtAppMainLoop();
	}
#endif
      delete m_pConf;
    }
  catch (...)
    {
      cout << "***** ERROR *****: Could not open: " << confFileName 
	   << " --> ABORTING!" << endl << endl;
      exit( 0 );
    }*/
  if(!config->exists("/plugins/colli/Colli_ROBOCUP_MODE"))
  {
      cout << "***** ERROR *****: Could not open: Colli_ROBOCUP_MODE" 
           << " --> ABORTING!" << endl << endl;
      exit( 0 );
  }
  else
  {
     m_RobocupMode = config->get_int("/plugins/colli/Colli_ROBOCUP_MODE");
  }
  if (!config->exists("/plugins/colli/adjust_robo_pos") )
  {
    cout << "***** ERROR *****: Could not get: adjust_robo_pos" << endl;
    adjust_robopos = false;
  }
  else
  {
    adjust_robopos = config->get_bool("/plugins/colli/adjust_robo_pos");
  }

  if( !config->exists("/plugins/colli/Roboshape/WIDTH_X") )
     adjust_robopos = false;
  else
    robo_widthX = config->get_float("/plugins/colli/Roboshape/WIDTH_X") * 100.;

  if( !config->exists("/plugins/colli/Roboshape/WIDTH_Y") )
    adjust_robopos = false;
  else
    robo_widthY = config->get_float("/plugins/colli/Roboshape/WIDTH_Y") * 100.;
}



// Destructor
CSearch::~CSearch()
{
  delete m_pAStar;
}

std::vector< HomPoint >  CSearch::GetPlan()
{
  
  return m_vPlan;
}


// Perform an Update by searching in the occgrid for a plan
//   from robopos to targetpos
void CSearch::Update( int roboX, int roboY, int targetX, int targetY)
{
  m_UpdatedSuccessful = false;
  int min_roboX = roboX;
  int min_roboY = roboY;
  if( adjust_robopos )
  {
    int cell_size = robo_widthX;
    if( robo_widthY < cell_size )
      cell_size = robo_widthY;
    cell_size /= m_pOccGrid->getCellWidth();
    cell_size /= 2;

    float min_dis = sqrt(pow(roboX-targetX,2)+pow(roboY-targetY,2));
    for( int i = roboX - cell_size; i <= roboX + cell_size; i++ )
    {
      for( int j = roboY - cell_size; j <= roboY + cell_size; j++ )
      {
        if( ( i >= 0 ) && ( j >= 0 ) && ( i < m_pOccGrid->getWidth() ) && ( j < m_pOccGrid->getHeight()) )
        {
          float dis = sqrt(pow(i-targetX,2)+pow(j-targetY,2));
          if( dis < min_dis )
          {
            min_dis = dis;
            min_roboX = i;
            min_roboY = j;
          }
        }
      }
    }
  }
  // check, if a position is in an obstacle
  //m_RoboPosition = HomPoint(roboX, roboY);
  m_RoboPosition = HomPoint(min_roboX, min_roboY);
  m_LocalTarget     = HomPoint( min_roboX, min_roboY );
  m_LocalTrajectory = HomPoint( min_roboX, min_roboY );
  
  if( m_pOccGrid->getProb( targetX, targetY ) == _COLLI_CELL_OCCUPIED_ )
    {
      int stepX = 1;  // initializing to 1
      int stepY = 1;
      if ( roboX < targetX ) // if we search in the other direction, inverse it!
	stepX = -1;
      if ( roboY < targetY ) 
	stepY = -1;
      m_TargetPosition = m_pAStar->RemoveTargetFromObstacle( targetX, targetY,stepX, stepY );
    }
  else
    {
      m_TargetPosition = HomPoint( targetX, targetY );
    }
 
    m_pAStar->Solve( m_RoboPosition, m_TargetPosition, m_vPlan);
  
  if (m_vPlan.size() > 0)
    {
      m_UpdatedSuccessful = true;
      m_LocalTarget     = CalculateLocalTarget();
      m_LocalTarget     = AdjustWaypoint( m_LocalTarget );
      m_LocalTrajectory = CalculateLocalTrajectoryPoint();
    } 
}

// Return, if the previous called update performed successfully
bool CSearch::UpdatedSuccessful()
{
  return m_UpdatedSuccessful;
}

HomPoint CSearch::get_adjust_robo()
{
  return m_RoboPosition;
}

/* **************************************************************************** */
/* **************************************************************************** */
/* *********** P R I V A T E  -   S T U F F *********************************** */
/* **************************************************************************** */
/* **************************************************************************** */



HomPoint CSearch::CalculateLocalTarget()
{
  HomPoint target = m_RoboPosition;
  HomPoint prev   = m_RoboPosition;

  if (  m_vPlan.size() > 2 )
    {
      vector<HomPoint>::iterator it = m_vPlan.begin()+1;
      for ( ; it != m_vPlan.end()-1; ++it )
  	{
	  prev = target;
	  target = *it;

	  if ( m_RobocupMode != 1 ) // not robocup mode
	    {
	      if ( IsObstacleBetween( m_RoboPosition, *it, 5 ) &&
		   IsObstacleBetween( m_RoboPosition, *(it+1), 5 ) )
		return prev;
	    }
	  else // robocup mode
	    {
	      if ( IsObstacleBetween( m_RoboPosition, *it, 11 ) &&
		   IsObstacleBetween( m_RoboPosition, *(it+1), 11 ) )
		return prev;
	    }
   	}
      return HomPoint( *(m_vPlan.end()-1) );
    }
  else
    {
      // return the current position if there is no plan.
      return m_RoboPosition;
    }
}


HomPoint CSearch::AdjustWaypoint( const HomPoint &local_target )
{
  return local_target;
}



// forward and backward plans should no longer make a difference in
//   trajectory searching
HomPoint CSearch::CalculateLocalTrajectoryPoint( )
{
  int x = (int)m_RoboPosition.x();
  int y = (int)m_RoboPosition.y();

  int max_occ = 10;

  if ( m_RobocupMode == 1 )
    max_occ = 20; // speed is all ;-)


  if ( x < (int)m_LocalTarget.x() )
    {
      ++x;
      while ( ( x < (int)m_pOccGrid->getWidth() ) && 
	      ( x <= (int)m_LocalTarget.x() ) &&
	      !(IsObstacleBetween( HomPoint(x, y), m_LocalTarget, max_occ )) &&
	      !(IsObstacleBetween( m_RoboPosition, HomPoint(x, y), max_occ ) ) )
	++x;
      
      if ( x == m_LocalTarget.x() && y == m_LocalTarget.y() )
	return HomPoint( x, y );
      else
	return HomPoint( x-1, y );
    }
  else
    {
      --x;
      while ( ( x > 0 ) && 
	      ( x >= (int)m_LocalTarget.x() ) &&
	      !(IsObstacleBetween( HomPoint(x, y), m_LocalTarget, max_occ )) &&
	      !(IsObstacleBetween( m_RoboPosition, HomPoint(x, y), max_occ ) ) )
	--x;

      if ( (x == (int)m_LocalTarget.x()) && (y == (int)m_LocalTarget.y()) )
	return HomPoint( x, y );
      else
	return HomPoint( x+1, y );
    }
}


// checks per raytracing, if an obstacle is between two points.
bool CSearch::IsObstacleBetween( const HomPoint &a, const HomPoint &b, 
				 const int maxcount )
{
  if (a.x() == b.x() && a.y() == b.y() )
    return false;
  int count = 0;
  float prob = 0.0;

  register int _xDirInt, _yDirInt;
  register int _actXGrid = (int)a.x();
  int endXGrid = (int)b.x();
  int dX = abs(endXGrid - _actXGrid);
  ( endXGrid > _actXGrid ? _xDirInt = 1 : _xDirInt = -1 );
  register int _actYGrid = (int)a.y();
  int endYGrid = (int)b.y();
  ( endYGrid > _actYGrid ? _yDirInt = 1 : _yDirInt = -1 );
  int dY = abs(endYGrid - _actYGrid);
  
  // decide whether direction is more x or more y, and run the algorithm
  if (dX > dY) 
    {
      register int _P, _dPr, _dPru;
      _dPr  = dY<<1; // amount to increment decision if right is chosen (always)
      _dPru = _dPr - (dX<<1); // amount to increment decision if up is chosen
      _P    = _dPr - dX; // decision variable start value
      
      for ( ; (_actXGrid != endXGrid) && (_actYGrid != endYGrid); 
	    _actXGrid += _xDirInt )
	{
	  if (_actXGrid < 0 || _actXGrid > m_pOccGrid->getWidth() ||
	      _actYGrid < 0 || _actXGrid > m_pOccGrid->getHeight() )
	    return false;

	  prob = m_pOccGrid->getProb( _actXGrid, _actYGrid );

	  if ( prob == _COLLI_CELL_FREE_ )
	    ;
	  else if ( prob == _COLLI_CELL_OCCUPIED_ )
	    return true;
	  else if ( prob == _COLLI_CELL_FAR_ )
	    ++count;
	  else if ( prob == _COLLI_CELL_MIDDLE_ )
	    count += 2;
	  else if ( prob == _COLLI_CELL_NEAR_ )
	    count += 4;
	  else
	    loggerAstar->log_error("CSearch","AStar_Search Line 541: ERROR IN RAYTRACER!\n");
	  
	  if ( count > maxcount )
	    return true;

	  ( ( _P > 0 ) ? _actYGrid += _yDirInt, _P += _dPru : _P += _dPr );
	}
    }
  else 
    {
      register int _P, _dPr, _dPru;
      _dPr         = dX<<1; // amount to increment decision if right is chosen (always)
      _dPru        = _dPr - (dY<<1); // amount to increment decision if up is chosen
      _P           = _dPr - dY; // decision variable start value
      
      for ( ; (_actXGrid != endXGrid) && (_actYGrid != endYGrid); 
	    _actYGrid += _yDirInt ) 
	{
	  if (_actXGrid < 0 || _actXGrid > m_pOccGrid->getWidth() ||
	      _actYGrid < 0 || _actXGrid > m_pOccGrid->getHeight() )
	    return false;

	  prob = m_pOccGrid->getProb( _actXGrid, _actYGrid );

	  if ( prob == _COLLI_CELL_FREE_ )
	    ;
	  else if ( prob == _COLLI_CELL_OCCUPIED_ )
	    return true;
	  else if ( prob == _COLLI_CELL_FAR_ )
	    ++count;
	  else if ( prob == _COLLI_CELL_MIDDLE_ )
	    count += 2;
	  else if ( prob == _COLLI_CELL_NEAR_ )
	    count += 4;
	  else
	    loggerAstar->log_error("CSearch","AStar_Search Line 576: ERROR IN RAYTRACER!\n");
	  
	  if ( count > maxcount )
	    return true;

	  ( ( _P > 0 ) ? _actXGrid += _xDirInt, _P += _dPru : _P += _dPr );
	}
    }
  return false; // there is no obstacle between those two points.
}


#endif
