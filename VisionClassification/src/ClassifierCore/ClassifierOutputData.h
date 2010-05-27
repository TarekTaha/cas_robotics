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
#ifndef CLASSIFIER_OUTPUT_H
#define CLASSIFIER_OUTPUT_H

#include "precomp.h"

typedef enum ClassifierVariableType {
    CVAR_VOID = 0,
    CVAR_INT,
    CVAR_FLOAT,
    CVAR_POINT,
    CVAR_STRING,
    CVAR_IMAGE,
    CVAR_SEQ,
    CVAR_BBOXES
} ClassifierVariableType;

class ClassifierOutputVariable
{
public:
    ClassifierOutputVariable() {
        m_name = "ERROR";
        m_type = CVAR_VOID;
        m_state = false;
    }
    ClassifierOutputVariable(string name, int data, bool state=true) {
        m_name = name;
        m_type = CVAR_INT;
        m_intdata = data;
        m_state = state;
    }
    ClassifierOutputVariable(string name, float data, bool state=true) {
        m_name = name;
        m_type = CVAR_FLOAT;
        m_floatdata = data;
        m_state = state;
    }
    ClassifierOutputVariable(string name, CvPoint data, bool state=true) {
        m_name = name;
        m_type = CVAR_POINT;
        m_pointdata = data;
        m_state = state;
    }
    ClassifierOutputVariable(string name, string data, bool state=true) {
        m_name = name;
        m_type = CVAR_STRING;
        m_stringdata = data;
        m_state = state;
    }
    ClassifierOutputVariable(string name, IplImage *data, bool state=true) {
        m_name = name;
        m_type = CVAR_IMAGE;
        m_imagedata = data;
        m_state = state;
    }
    ClassifierOutputVariable(string name, CvSeq *data, bool state=true) {
        m_name = name;
        m_type = CVAR_SEQ;
        m_sequencedata = data;
        m_state = state;
    }
    ClassifierOutputVariable(string name, vector<CvRect> *data, bool state=true) {
        m_name = name;
        m_type = CVAR_BBOXES;
        m_bboxdata = data;
        m_state = state;
    }

    ~ClassifierOutputVariable() { }
    string getName() { return m_name; }
    ClassifierVariableType GetType() { return m_type; }
    bool GetState() { return m_state; }
    void SetState(bool state) { m_state = state; }

    int GetIntData() { assert(m_type==CVAR_INT);	return m_intdata; }
    float GetFloatData() { assert(m_type==CVAR_FLOAT);	return m_floatdata; }
    CvPoint GetPointData() { assert(m_type==CVAR_POINT);	return m_pointdata; }
    string GetStringData() { assert(m_type==CVAR_STRING);	return m_stringdata; }
    IplImage* GetImageData() { assert(m_type==CVAR_IMAGE);	return m_imagedata; }
    CvSeq* GetSequenceData() { assert(m_type==CVAR_SEQ);	return m_sequencedata; }
    vector<CvRect>* GetBoundingBoxData() { assert(m_type==CVAR_BBOXES);	return m_bboxdata; }

private:
    string m_name;
    ClassifierVariableType m_type;
    bool m_state;
    int m_intdata;
    float m_floatdata;
    CvPoint m_pointdata;
    string m_stringdata;
    IplImage *m_imagedata;
    CvSeq* m_sequencedata;
    vector<CvRect>* m_bboxdata;
};

class ClassifierOutputData
{
public:
	ClassifierOutputData();
	~ClassifierOutputData();

        void addVariable(ClassifierOutputVariable var);
        void addVariable(string name, int value, bool state=true);
        void addVariable(string name, float value, bool state=true);
        void addVariable(string name, CvPoint value, bool state=true);
        void addVariable(string name, string value, bool state=true);
        void addVariable(string name, IplImage* value, bool state=true);
        void addVariable(string name, CvSeq* value, bool state=true);
        void addVariable(string name, vector<CvRect>* value, bool state=true);

        void setVariable(ClassifierOutputVariable var);
        void setVariable(string name, int value);
        void setVariable(string name, float value);
        void setVariable(string name, CvPoint value);
        void setVariable(string name, string value);
        void setVariable(string name, IplImage* value);
        void setVariable(string name, CvSeq* value);
        void setVariable(string name, vector<CvRect>* value);

        void setVariableState(string name, bool state);
        bool getVariableState(string name);

        void mergeWith(ClassifierOutputData mergeData);

        ClassifierOutputVariable getVariable(string name);
        int getIntData(string name);
        float getFloatData(string name);
        CvPoint getPointData(string name);
        string getStringData(string name);
        IplImage* getImageData(string name);
        CvSeq* getSequenceData(string name);
        vector<CvRect>* getBoundingBoxData(string name);

        bool hasVariable(string name);
        int numVariables();
        bool getStateOfIndex(int index);
        string getNameOfIndex(int index);
        ClassifierVariableType getTypeOfIndex(int index);

	vector<ClassifierOutputVariable> data;

private:
};

#endif
