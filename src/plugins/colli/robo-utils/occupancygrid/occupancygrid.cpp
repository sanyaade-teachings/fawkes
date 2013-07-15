
//#include <utils/occupancygrid/occupancygrid.h>
#include <cstdlib>
#include <vector>

#include "occupancygrid.h"

OccupancyGrid::OccupancyGrid(int width, int height,
			       int cell_width, int cell_height)
{
  m_Width = width;
  m_Height = height;
  m_CellWidth = cell_width;
  m_CellHeight = cell_height;
  initGrid();
}


OccupancyGrid::~OccupancyGrid()
{
  m_OccupancyProb.clear();
}

void OccupancyGrid::setCellWidth(int width)
{
  m_CellWidth = width;
}

int OccupancyGrid::getCellWidth()
{
  return m_CellWidth;
}

void OccupancyGrid::setCellHeight(int height)
{
  m_CellHeight = height;
}

int OccupancyGrid::getCellHeight()
{
  return m_CellHeight;
}

void OccupancyGrid::setWidth(int width)
{
  m_Width = width;
  initGrid();
}

int OccupancyGrid::getWidth()
{
  return m_Width;
}

void OccupancyGrid::setHeight(int height)
{
  m_Height = height;
  initGrid();
}

int OccupancyGrid::getHeight()
{
  return m_Height;
}


void OccupancyGrid::setProb(int x, int y, Probability prob)
{
  if((x < m_Width) && (y < m_Height) 
     && ( (isProb(prob)) || (prob == 2.0) ) )
    {
      m_OccupancyProb[x][y] = prob;
    }
}

void OccupancyGrid::fill(Probability prob)
{
  if((isProb(prob)) || (prob == -1))
    {
      for(int x = 0; x < m_Width; x++)
	{
	  for(int y = 0; y < m_Height; y++)
	    {
	      m_OccupancyProb[x][y] = prob; 
	    }
	}
    }
}

//  Probability OccupancyGrid::getProb(int x, int y)
//  {
//    return m_OccupancyProb[x][y];
//  }


void OccupancyGrid::initGrid()
{
  m_OccupancyProb.clear();
  std::vector<Probability> row;
  row.resize(m_Height, 0.0);
  m_OccupancyProb.resize(m_Width, row);
  fill( 0.0 );
}
