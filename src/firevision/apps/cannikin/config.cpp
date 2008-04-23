
/***************************************************************************
 *  config.cpp - config helper - generated by genconfig 0.3
 *
 *  Config class generated: Mon Apr 21 19:49:47 2008
 *  Used template: ../../../../../src/modules/tools/genconfig/config.template.cpp
 *  Template created: Thu Apr 29 14:27:25 2004
 *  Copyright  2004  Tim Niemueller
 *  niemueller@i5.informatik.rwth-aachen.de
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
 
/// @cond RCSOFTX_CONFIG
 
#include "config.h"

#include <iostream>
#include <utils/utils.h>

using namespace std;

/** @class CannikinConfig "config.h"
 * Configration object.
 * Auto-generated configuration object.
 */

/** Constructor.
 * @param configFile config reader
 */
CannikinConfig::CannikinConfig(CConfigReader *configFile)
{

  string errmsg;

  m_pXMLConfigFile = configFile;
  m_sPrefix = "";

  if (m_pXMLConfigFile != NULL) {
    m_sPrefix = "<RCSoftConfigFile><Lowlevel><FireVision><Stereo><StereoGeneral>";

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<ClassifierSettings><ClassifierType>", ClassifierType, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<ClassifierSettings><ClassifierType>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<ClassifierSettings><ClassifierType>: " << toString(ClassifierType) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<ClassifierSettings><ShrinkerType>", ShrinkerType, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<ClassifierSettings><ShrinkerType>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<ClassifierSettings><ShrinkerType>: " << toString(ShrinkerType) << endl << flush;

    m_sPrefix = "<RCSoftConfigFile><Lowlevel><FireVision><Stereo><StereoGeneral><StereoModelSettings>";

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<ColorModelType>", ColorModel, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<ColorModelType>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<ColorModelType>: " << toString(ColorModel) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<ScanlineModelType>", ScanlineModel, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<ScanlineModelType>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<ScanlineModelType>: " << toString(ScanlineModel) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<ColorModel><LookupTable><ColormapDirectory>", ColormapDirectory, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<ColorModel><LookupTable><ColormapDirectory>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<ColorModel><LookupTable><ColormapDirectory>: " << toString(ColormapDirectory) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<ColorModel><LookupTable><ColormapFilestem>", ColormapFilestem, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<ColorModel><LookupTable><ColormapFilestem>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<ColorModel><LookupTable><ColormapFilestem>: " << toString(ColormapFilestem) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<ScanlineModel><GridModel><GridXOffset>", ScanlineGridXOffset, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<ScanlineModel><GridModel><GridXOffset>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<ScanlineModel><GridModel><GridXOffset>: " << toString(ScanlineGridXOffset) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<ScanlineModel><GridModel><GridYOffset>", ScanlineGridYOffset, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<ScanlineModel><GridModel><GridYOffset>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<ScanlineModel><GridModel><GridYOffset>: " << toString(ScanlineGridYOffset) << endl << flush;

    m_sPrefix = "<RCSoftConfigFile><Lowlevel><FireVision><Stereo><RobotStereoSettings hostname=\""+string( getenv("HOSTNAME") )+"\">";

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<CameraString>", CameraString, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<CameraString>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<CameraString>: " << toString(CameraString) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<CameraHeight>", CameraHeight, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<CameraHeight>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<CameraHeight>: " << toString(CameraHeight) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<CameraOffsetX>", CameraOffsetX, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<CameraOffsetX>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<CameraOffsetX>: " << toString(CameraOffsetX) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<CameraOffsetY>", CameraOffsetY, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<CameraOffsetY>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<CameraOffsetY>: " << toString(CameraOffsetY) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<CameraBearing>", CameraBearing, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<CameraBearing>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<CameraBearing>: " << toString(CameraBearing) << endl << flush;

    if (!m_pXMLConfigFile->GetNodeValue(m_sPrefix+"<CameraSlope>", CameraSlope, errmsg)) {
        cout << endl << "** XML ERROR while trying to fetch "+m_sPrefix+"<CameraSlope>" << endl << flush;
        cout << __FUNCTION__ << ": " << __FILE__ << "[" << __LINE__ << "]" << endl << flush;
        cout << "Error message from ConfigReader was: " << errmsg << endl;
        exit(1);
    }
    cout << m_sPrefix << "<CameraSlope>: " << toString(CameraSlope) << endl << flush;

  } else {
    cout << "*** XML ERROR: No XMLConfigReader given to Config Object. Did NOT read config!" << endl << flush;
    exit(2);
  }

}

/** Destructor. */
CannikinConfig::~CannikinConfig()
{
}

/// @endcond

