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
#include "precomp.h"

CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3] = {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;
    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

CvScalar colorSwatch[COLOR_SWATCH_SIZE] =
{
    CV_RGB(0xFF,0x66,0x00),
    CV_RGB(0xFF,0xE6,0x00),
    CV_RGB(0x99,0xFF,0x00),
    CV_RGB(0xFF,0x00,0x1A),
    CV_RGB(0x3D,0xB1,0xFF),
    CV_RGB(0xE6,0x00,0xFF),
    CV_RGB(0xFF,0x00,0x99),
    CV_RGB(0x66,0x00,0xFF),
    CV_RGB(0x00,0xFF,0xE6),
    CV_RGB(0xFF,0xAF,0x7A),
    CV_RGB(0x1A,0xFF,0x00),
    CV_RGB(0x71,0xCA,0xFF),
    CV_RGB(0x00,0x1A,0xFF),
    CV_RGB(0xFF,0x8B,0x3D),
    CV_RGB(0x00,0x99,0xFF),
    CV_RGB(0x00,0xFF,0x66)
};

char *filterNames[] = { (char*)"Color", (char*)"Shape", (char*)"Brightness", (char*)"SIFT", (char*)"Adaboost", (char*)"Motion", (char*)"Gesture" };

void SaveTrackToFile(MotionTrack mt, const char *filename)
{
    FILE *datafile = fopen(filename, "wb");
    int nPoints = (int) mt.size();
    fwrite(&nPoints, sizeof(int), 1, datafile);
    for (int i = 0; i<nPoints; i++)
    {
        fwrite(&(mt[i].m_x), sizeof(double), 1, datafile);
        fwrite(&(mt[i].m_y), sizeof(double), 1, datafile);
    }
    fclose(datafile);
}

MotionTrack ReadTrackFromFile(const char *filename)
{
    FILE *datafile = fopen(filename, "rb");
    MotionTrack mt;
    int nPoints = 0;
    fread(&nPoints, sizeof(int), 1, datafile);
    for (int i = 0; i<nPoints; i++)
    {
        OneDollarPoint pt(0,0);
        fread(&(pt.m_x), sizeof(double), 1, datafile);
        fread(&(pt.m_y), sizeof(double), 1, datafile);
        mt.push_back(pt);
    }
    fclose(datafile);
    return mt;
}

bool DeleteDirectory(const char *path, bool useRecycleBin)
{
    try
    {
        boost::filesystem::path path1(path);
        boost::filesystem::remove(path1);
        return true;
    }
    catch(exception &e)
    {
        printf("%s",e.what());
        return false;
    }
}
