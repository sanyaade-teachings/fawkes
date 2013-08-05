/***************************************************************************
 *  og_laser.h - occ-grid interface for colli_a* search algorithm
 *
 *  Created: Sat Jul 13 18:06:21 2013
 *  Copyright  2002  Stefan Jacobs
 *             2012  Safoura Rezapour Lakani
 *             2013  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef _PLUGINS_COLLI_SEARCH_OG_LASER_H_
#define _PLUGINS_COLLI_SEARCH_OG_LASER_H_

//#include <utils/occupancygrid/occupancygrid.h>
#include "../robo-utils/occupancygrid/occupancygrid.h"

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

  class Logger;
  class Configuration;
//class Laser360Interface;
  class ColliEllipseMap;

  class Ellipse;
  class Laser;
  class RoboShape;
  class TrigTable;

  /** @class ColliLaserOccupancyGrid <plugins/colli/search/og_laser.h>
   * This OccGrid is derived by the Occupancy Grid originally from Andreas Strack,
   * but modified for speed purposes.
   *
   * This interface is mainly out of implementation reasons given
   * here. It includes a occ-grid and the laserinterface, so no one
   * else has to care about.
   *
   * @author Stefan Jacobs
   */
  class ColliLaserOccupancyGrid : public OccupancyGrid
  {
  public:

    // Constructor.
    ColliLaserOccupancyGrid(Logger* logger, Configuration *config, Laser * laser, int width = 150, int height = 150,
        int cell_width = 5, int cell_height = 5);

    // Destructor
    ~ColliLaserOccupancyGrid();

    /** Put the laser readings in the occupancy grid
     *  Also, every reading gets a radius according to the relative direction
     *  of this reading to the robot.
     *  @param midX is the current point of the robot.
     *  @param midY is the current point of the robot.
     *  @param inc is the current constant to increase the obstacles.
     */
    void
    UpdateOccGrid(int midX, int midY, float inc, float vel, float xdiff, float ydiff, float oridiff);

    /**
     *  Reset all old readings and forget about the world state!
     */
    void
    ResetOld(int max_age = -1);

//private:
    float
    normalize_degree(float angle_deg);

    /**
     *  Integrate historical readings to the current occgrid.
     */
    void
    IntegrateOldReadings(int midX, int midY, float inc, float vel, float xdiff, float ydiff, float oridiff);

    /**
     *  Integrate the current readings to the current occgrid.
     */
    void
    IntegrateNewReadings(int midX, int midY, float inc, float vel);

    /**
     *  Check if the current value is contained in the history.
     */
    bool
    Contained(float p_x, float p_y);

    /**
     * Integrate a single ellipse
     *
     * @param ellipse the ellipse that is to be integrated
     */
    void
    integrateObstacle(Ellipse ellipse);

    //  void integrateEllipseCell( int centerx, int centery,
    //           int i, int j, float prob );

    // pointer to the laser
    Laser * m_pLaser;
    //fawkes::Laser360Interface *m_pLaser;

    // my roboshape
    RoboShape * m_pRoboShape;

    // fast trigonometry table
    TrigTable * m_pTrigTable;

    // History
    std::vector<float> m_vOldReadings;

    // History concerned constants
    int m_MaxHistoryLength, m_MinHistoryLength, m_InitialHistorySize;

    // Trigonometry table its resolution
    int m_TrigTableResolution;

    // Laser concerned settings
    float m_MinimumLaserLength, m_EllipseDistance;

    int m_RobocupMode;

    float m_MaxCellExt;
    float m_MaxOldCellExt;

    ColliEllipseMap * ellipse_map;

    Logger* loggerGrid;
    bool ref_obstacle;
//  int m_CellWidth,m_CellHeight;
//  int m_Width,m_Height;

    inline float
    sqr(float x)
    {
      return (x * x);
    }

  };

} // end of namespace fawkes

#endif
