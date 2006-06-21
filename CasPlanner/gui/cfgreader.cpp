/***************************************************************************
 *   Copyright (C) 2006 by Waleed Kadous   *
 *   waleed@width   *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "cfgreader.h"

CfgReader::CfgReader(QStringList configFiles, RobotsView *in_rv, CommsMgr *in_cm):
    rv(in_rv),    
    cm(in_cm)
    {
    rv->setCommsMgr(cm); 
    for(int h=0; h < configFiles.size(); h++){
	ConfigFile *cf = new ConfigFile();
	int numSections; 
	cf->Load(configFiles[h].toLocal8Bit());
	numSections = cf->GetSectionCount(); 
	for(int i=0; i < numSections; i++){
	    QString sectionName = cf->GetSectionType(i);
	    if(sectionName == "gui"){
		rv->createNewTab(cf, i);  
	    }
	    if(sectionName == "comms"){
		cm->createNewComms(cf, i); 
	    }
	}
    }
}

void CfgReader::readRobot(QString fileName){
    ConfigFile *cf = new ConfigFile();
    int numSections; 
    cf->Load(fileName.toLocal8Bit());
    numSections = cf->GetSectionCount(); 
    for(int i=0; i < numSections; i++){
	QString sectionName = cf->GetSectionType(i);
	if(sectionName == "gui"){
	    rv->createNewTab(cf, i);  
	}
	if(sectionName == "comms"){
	    cm->createNewComms(cf, i); 
	}
    }
}

CfgReader::~CfgReader(){
    
}

