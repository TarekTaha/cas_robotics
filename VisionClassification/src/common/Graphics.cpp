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
#include "Graphics.h"

Graphics::Graphics():
        rgbImage(NULL)
{
    cvInitFont( &base_font, CV_FONT_VECTOR0, 0.5, 0.5);
    strcpy(this->windowName,windowName);
    cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
}

Graphics::Graphics(const char *windowName):
        rgbImage(NULL)
{
    cvInitFont( &base_font, CV_FONT_VECTOR0, 0.5, 0.5);
    strcpy(this->windowName,windowName);
    cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
}

Graphics::Graphics(const char *windowName, IplImage *rgbImage)
{
    this->rgbImage = rgbImage;
    cvInitFont( &base_font, CV_FONT_VECTOR0, 0.5, 0.5);
    strcpy(this->windowName,windowName);
    cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
    cvShowImage (windowName, rgbImage);
}

Graphics::~Graphics()
{
    if(rgbImage)
        cvReleaseImage(&rgbImage);
    cvDestroyWindow(windowName);
}

void Graphics::drawImage(IplImage *img)
{
    cvShowImage(windowName,img);
}

void Graphics::drawArrow(IplImage *img, CvPoint center, double angleDegrees, double magnitude, CvScalar color, int thickness)
{
    if(rgbImage)
        cvReleaseImage(&rgbImage);
    rgbImage = img;

    CvPoint endpoint, arrowpoint;
    double angle = angleDegrees*CV_PI/180.0;
    endpoint.x = (int) (center.x + magnitude*cos(angle));
    endpoint.y = (int) (center.y + magnitude*sin(angle));

    /* Draw the main line of the arrow. */
    cvLine(img, center, endpoint, color, thickness, CV_AA, 0);
    /* Now draw the tips of the arrow, scaled to the size of the main part*/
    arrowpoint.x = (int) (endpoint.x + 12* cos(angle + 3*CV_PI/4));
    arrowpoint.y = (int) (endpoint.y + 12* sin(angle + 3*CV_PI/4));
    cvLine(img, arrowpoint, endpoint, color, thickness, CV_AA, 0);
    arrowpoint.x = (int) (endpoint.x + 12* cos(angle - 3*CV_PI/4));
    arrowpoint.y = (int) (endpoint.y + 12* sin(angle - 3*CV_PI/4));
    cvLine(img, arrowpoint, endpoint, color, thickness, CV_AA, 0);

    cvShowImage(windowName,rgbImage);
}

void Graphics::drawTrack(IplImage *img, MotionTrack mt,  CvScalar color, int thickness, float squareSize, int maxPointsToDraw)
{
    if(rgbImage)
        cvReleaseImage(&rgbImage);
    rgbImage = img;
    int nPoints = (int) mt.size();
    int startPoint = 0;
    if (maxPointsToDraw > 0)
    {
        if (nPoints > maxPointsToDraw)
        {
            startPoint = nPoints - maxPointsToDraw;
            nPoints = maxPointsToDraw;
        }
    }
    CvPoint *trackPoints = new CvPoint[nPoints];
    for (int i=0; i<nPoints; i++)
    {
        trackPoints[i].x = (int) (img->width/2.0 + (mt[i+startPoint].m_x*img->width)/squareSize);
        trackPoints[i].y = (int) (img->height/2.0 + (mt[i+startPoint].m_y*img->height)/squareSize);
    }
    cvPolyLine(img, &trackPoints, &nPoints, 1, 0, color, thickness, CV_AA);
    delete[] trackPoints;
    cvShowImage(windowName,rgbImage);
}

void Graphics::drawTrack(MotionTrack mt, float width, float height, float squareSize)
{
    if (!rgbImage)
        return;
    int nPoints = (int) mt.size();
    CvPoint *trackPoints = new CvPoint[nPoints];
    for (int i = 0; i<nPoints; i++)
    {
        trackPoints[i].x = (int)(width/2.0  + (mt[i].m_x*width)/squareSize);
        trackPoints[i].y = (int)(height/2.0 + (mt[i].m_y*height)/squareSize);
    }
    for (int i = 0; i<nPoints-1; i++)
    {
        CvScalar color = cvScalar((255*i)/nPoints, 100,255);
        cvDrawLine(rgbImage,trackPoints[i], trackPoints[i+1],color,1);
    }
    delete[] trackPoints;
}
