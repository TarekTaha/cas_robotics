/***************************************************************************
 *   Vision Classification Library                                         *
 *   Copyright (C) 2010 by:                                                *
 *      Tarek Taha, CAS-UTS  <tataha@cas.edu.au>                           *
 *      Dan Maynes-Aminzade  <monzy@stanford.edu>                          *
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
#ifndef ONEDOLLAR_H
#define ONEDOLLAR_H
#include "constants.h"
#include <string>
#include <vector>
#include <list>
#include <map>
#include <vector>
#include <algorithm>
#include <string>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "cv.h"

using namespace std;

class OneDollarPoint
{
public:
    double m_x, m_y;
    OneDollarPoint(double x, double y)
    {
        m_x = x;
        m_y = y;
    }
};


class OneDollarRectangle
{
public:
    double m_x, m_y, m_width, m_height;
    OneDollarRectangle(double x, double y, double width, double height)
    {
        m_x = x;
        m_y = y;
        m_width = width;
        m_height = height;
    }
};

class Template
{
public:
    string m_name;
    vector<OneDollarPoint> m_points;

    Template(string name, vector<OneDollarPoint> points);
    Template(FILE *src);
    void WriteToFile(FILE *dst);
    int GetLength();
};

class Result
{
public:
    string m_name;
    double m_score;
    int m_index;
    Result(string name, double score, int index)
    {
        m_name = name;
        m_score = score;
        m_index = index;
    }
};

class Recognizer
{
public:
    vector<Template> m_templates;
    Recognizer();
    Result Recognize(vector<OneDollarPoint> points);
    Result BackRecognize(vector<OneDollarPoint> points);
    int addTemplate(string name, OneDollarPoint* points, int npoints);
    int addTemplate(string name, vector<OneDollarPoint> points);
    int addTemplate(string name, Template t);
    int DeleteUserTemplates();
};

vector<OneDollarPoint> Resample(vector<OneDollarPoint> points, int n);
vector<OneDollarPoint> RotateToZero(vector<OneDollarPoint> points);
vector<OneDollarPoint> RotateBy(vector<OneDollarPoint> points, double theta);
vector<OneDollarPoint> ScaleToSquare(vector<OneDollarPoint> points, double size);
vector<OneDollarPoint> TranslateToOrigin(vector<OneDollarPoint> points);
double DistanceAtBestAngle(vector<OneDollarPoint> points, Template T, double a, double b, double threshold);
double DistanceAtAngle(vector<OneDollarPoint> points, Template T, double theta);
OneDollarPoint Centroid(vector<OneDollarPoint> points);
OneDollarRectangle BoundingBox(vector<OneDollarPoint> points);
double PathDistance(vector<OneDollarPoint> pts1, vector<OneDollarPoint> pts2);
double PathLength(vector<OneDollarPoint> points);
double Distance(OneDollarPoint p1, OneDollarPoint p2);

#endif
