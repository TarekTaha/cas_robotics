/***************************************************************************
 *   Vision Classification Library                                         *
 *   Copyright (C) 2010 by:                                                *
 *      Tarek Taha, CAS-UTS  <tataha@cas.edu.au>                           *
 *      Dan Maynes-Aminzade  <monzy@cs.stanford.edu>                       *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02110-1301, USA.          *
 ***************************************************************************/
#ifndef PRECOMP_H
#define PRECOMP_H

#include <math.h>
#include <float.h>
#include <time.h>

// STL includes
#include <list>
#include <map>
#include <vector>
#include <algorithm>
#include <string>
using namespace std;

// Boost includes
#include "boost/filesystem.hpp"

// OpenCV Includes
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"

// Haar Training Includes
#include "_cvcommon.h"
#include "_cvhaartraining.h"
#include "cvhaartraining.h"
#include "cvclassifier.h"

// SIFT includes
#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"

// Gesture Tracking includes
#include "OneDollar.h"
typedef vector<OneDollarPoint> MotionTrack;

// Utility functions
CvScalar hsv2rgb(float hue);
bool DeleteDirectory(const char *path, bool useRecycleBin = true);
void SaveTrackToFile(MotionTrack mt, const char *filename);
MotionTrack ReadTrackFromFile (const char* filename);

// swatch of "nice" colors
#define COLOR_SWATCH_SIZE 16
extern CvScalar colorSwatch[];
extern char *filterNames[];

#endif
