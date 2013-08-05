/***************************************************************************
 *  ellipse.h -
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

#ifndef _PLUGINS_COLLI_ROBO_UTILS_GEOMETRY_ELLIPSE_H
#define _PLUGINS_COLLI_ROBO_UTILS_GEOMETRY_ELLIPSE_H

#include <utils/math/types.h>
#include <utils/math/angle.h>
#include <geometry/hom_point.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

  class Ellipse
  {
  public:
    Ellipse();

    Ellipse(const HomPoint & center, const float width, const float height, const float angle);
    ~Ellipse();

    const HomPoint &
    GetCenter() const;
    void
    SetCenter(const HomPoint & center);
    const float
    GetWidth() const;
    void
    SetWidth(const float width);
    const float
    GetHeight() const;
    void
    SetHeight(const float height);
    const float
    GetAngle() const;
    void
    SetAngle(const float angle);

    const HomPoint
    GetBorderPoint(float angle) const;

    const float
    GetSquaredDistanceTo(const HomPoint p1, const HomPoint p2) const;
    const float
    GetAngleTo(const HomPoint p1, const HomPoint p2) const;
    const bool
    IsInside(const HomPoint & p) const;

    HomPoint m_Center;
    float m_Width;
    float m_Height;
    float m_Angle;
  };

  inline
  Ellipse::Ellipse()
  {
    m_Center = HomPoint();
    m_Width = 2.0;
    m_Height = 2.0;
  }

  inline
  Ellipse::Ellipse(const HomPoint & center, const float width, const float height, const float angle)
  {
    m_Center = center;
    m_Width = width;
    m_Height = height;
    m_Angle = angle;
  }

  inline
  Ellipse::~Ellipse()
  {
  }

  inline const HomPoint &
  Ellipse::GetCenter() const
  {
    return m_Center;
  }

  inline void
  Ellipse::SetCenter(const HomPoint & center)
  {
    m_Center = center;
  }

  inline const float
  Ellipse::GetWidth() const
  {
    return m_Width;
  }

  inline void
  Ellipse::SetWidth(const float width)
  {
    m_Width = width;
  }

  inline const float
  Ellipse::GetHeight() const
  {
    return m_Height;
  }

  inline void
  Ellipse::SetHeight(const float height)
  {
    m_Height = height;
  }

  inline const float
  Ellipse::GetAngle() const
  {
    return m_Angle;
  }

  inline void
  Ellipse::SetAngle(const float angle)
  {
    m_Angle = angle;
  }

  inline const HomPoint
  Ellipse::GetBorderPoint(float angle) const
  {
    float xw = m_Width / 2.0;
    float yh = m_Height / 2.0;
    float x, y;

    angle = normalize_rad(angle - m_Angle);

    x = cos(angle) * xw + m_Center.x();
    y = sin(angle) * yh + m_Center.y();

    HomPoint p(x - m_Center.x(), y - m_Center.y());
    float x_tmp = x * cos(m_Angle) - y * sin(m_Angle);
    p.y() = p.x() * sin(m_Angle) + p.y() * cos(m_Angle);
    p.x() = x_tmp;

    p.x() += m_Center.x();
    p.y() += m_Center.y();

    return p;
  }

  inline const float
  Ellipse::GetSquaredDistanceTo(const HomPoint p1, const HomPoint p2) const
  {
    return (p2.x() - p1.x()) * (p2.x() - p1.x()) + (p2.y() - p1.y()) * (p2.y() - p1.y());
  }

  inline const float
  Ellipse::GetAngleTo(const HomPoint p1, const HomPoint p2) const
  {
    return atan2f(p2.y() - p1.y(), p2.x() - p1.x());
  }

  inline const bool
  Ellipse::IsInside(const HomPoint & p) const
  {
    return (GetSquaredDistanceTo(m_Center, p) <= GetSquaredDistanceTo(m_Center, GetBorderPoint(GetAngleTo(m_Center, p))));
  }

} // namespace fawkes

#endif
