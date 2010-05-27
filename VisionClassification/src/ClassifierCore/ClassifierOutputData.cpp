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
#include "ClassifierOutputData.h"


ClassifierOutputData::ClassifierOutputData()
{
}

ClassifierOutputData::~ClassifierOutputData()
{
}

void ClassifierOutputData::addVariable(ClassifierOutputVariable var)
{
    if (hasVariable(var.getName())) 	// this variable already exists
    {
        setVariable(var);
    }
    else
    {
        data.push_back(var);
    }
}

void ClassifierOutputData::addVariable(string name, int value, bool state)
{
    ClassifierOutputVariable var(name, value, state);
    addVariable(var);
}

void ClassifierOutputData::addVariable(string name, float value, bool state)
{
    ClassifierOutputVariable var(name, value, state);
    addVariable(var);
}

void ClassifierOutputData::addVariable(string name, CvPoint value, bool state)
{
    ClassifierOutputVariable var(name, value, state);
    addVariable(var);
}

void ClassifierOutputData::addVariable(string name, string value, bool state)
{
    ClassifierOutputVariable var(name, value, state);
    addVariable(var);
}

void ClassifierOutputData::addVariable(string name, IplImage* value, bool state)
{
    ClassifierOutputVariable var(name, value, state);
    addVariable(var);
}

void ClassifierOutputData::addVariable(string name, CvSeq* value, bool state)
{
    ClassifierOutputVariable var(name, value, state);
    addVariable(var);
}

void ClassifierOutputData::addVariable(string name, vector<CvRect>* value, bool state)
{
    ClassifierOutputVariable var(name, value, state);
    addVariable(var);
}

ClassifierOutputVariable ClassifierOutputData::getVariable(string name)
{
    for (uint i=0; i<data.size(); i++)
    {
        ClassifierOutputVariable var = data[i];
        if (name.compare(var.getName()) == 0)
        {
            return var;
        }
    }
    ClassifierOutputVariable error;
    return error;
}

void ClassifierOutputData::setVariable(ClassifierOutputVariable newvar)
{
    string name = newvar.getName();
    for (uint i=0; i<data.size(); i++)
    {
        ClassifierOutputVariable var = data[i];
        if (name.compare(var.getName()) == 0)
        {
            assert(newvar.GetType() == var.GetType());
            // preserve the old state; this should only be changed with the SetVariableState function
            newvar.SetState(var.GetState());
            data[i] = newvar;
        }
    }
}

bool ClassifierOutputData::getVariableState(string name)
{
    for (uint i=0; i<data.size(); i++)
    {
        ClassifierOutputVariable var = data[i];
        if (name.compare(var.getName()) == 0)
        {
            return var.GetState();
        }
    }
    return false;
}

void ClassifierOutputData::setVariableState(string name, bool state)
{
    for (uint i=0; i<data.size(); i++)
    {
        ClassifierOutputVariable var = data[i];
        if (name.compare(var.getName()) == 0)
        {
            var.SetState(state);
            data[i] = var;
        }
    }
}

bool ClassifierOutputData::hasVariable(string name)
{
    for (uint i=0; i<data.size(); i++)
    {
        ClassifierOutputVariable var = data[i];
        if (name.compare(var.getName()) == 0)
        {
            return true;
        }
    }
    return false;
}

int ClassifierOutputData::getIntData(string name)
{
    ClassifierOutputVariable var = getVariable(name);
    return var.GetIntData();
}

float ClassifierOutputData::getFloatData(string name)
{
    ClassifierOutputVariable var = getVariable(name);
    return var.GetFloatData();
}

CvPoint ClassifierOutputData::getPointData(string name)
{
    ClassifierOutputVariable var = getVariable(name);
    return var.GetPointData();
}

string ClassifierOutputData::getStringData(string name)
{
    ClassifierOutputVariable var = getVariable(name);
    return var.GetStringData();
}

IplImage* ClassifierOutputData::getImageData(string name)
{
    ClassifierOutputVariable var = getVariable(name);
    return var.GetImageData();
}

CvSeq* ClassifierOutputData::getSequenceData(string name)
{
    ClassifierOutputVariable var = getVariable(name);
    return var.GetSequenceData();
}

vector<CvRect>* ClassifierOutputData::getBoundingBoxData(string name)
{
    ClassifierOutputVariable var = getVariable(name);
    return var.GetBoundingBoxData();
}


void ClassifierOutputData::setVariable(string name, int value)
{
    ClassifierOutputVariable var(name, value);
    setVariable(var);
}

void ClassifierOutputData::setVariable(string name, float value)
{
    ClassifierOutputVariable var(name, value);
    setVariable(var);
}

void ClassifierOutputData::setVariable(string name, CvPoint value)
{
    ClassifierOutputVariable var(name, value);
    setVariable(var);
}

void ClassifierOutputData::setVariable(string name, string value)
{
    ClassifierOutputVariable var(name, value);
    setVariable(var);
}

void ClassifierOutputData::setVariable(string name, IplImage* value)
{
    ClassifierOutputVariable var(name, value);
    setVariable(var);
}

void ClassifierOutputData::setVariable(string name, CvSeq* value)
{
    ClassifierOutputVariable var(name, value);
    setVariable(var);
}

void ClassifierOutputData::setVariable(string name, vector<CvRect>* value)
{
    ClassifierOutputVariable var(name, value);
    setVariable(var);
}

int ClassifierOutputData::numVariables()
{
    return data.size();
}

bool ClassifierOutputData::getStateOfIndex(int i)
{
    return data[i].GetState();
}

string ClassifierOutputData::getNameOfIndex(int i)
{
    return data[i].getName();
}

ClassifierVariableType ClassifierOutputData::getTypeOfIndex(int i)
{
    return data[i].GetType();
}

void ClassifierOutputData::mergeWith(ClassifierOutputData mergeData)
{
    for (uint i=0; i<mergeData.data.size(); i++)
    {
        ClassifierOutputVariable var = mergeData.data[i];
        if (this->hasVariable(var.getName()))
        {	// this variable is already present, so we'll overwrite it
            this->setVariable(var);
        }
        else
        {	// it's missing, so we'll add it
            this->addVariable(var);
        }
    }
}
