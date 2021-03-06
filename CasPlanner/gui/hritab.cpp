/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "hritab.h"
#include "playground.h"
#include "socialplanner.h"
#include "mapmanager.h"
#include "robotmanager.h"
#include "planningmanager.h"
#include "navigator.h"
#include "commmanager.h"
#include "IntentionRecognizer.h"

HriTab::HriTab(QWidget *parent,PlayGround * playG)
    : QWidget(parent),
    playGround(playG)
{
    ui.setupUi(this);
    connect(ui.toggleSpeech,  		SIGNAL(stateChanged (int)), this, SLOT(toggleVoiceNotification(int)));
    connect(ui.continiousStrategy,  SIGNAL(toggled(bool)), this, SLOT(toggleStrategy(bool)));
    connect(ui.minimalStrategy,  	SIGNAL(toggled(bool)), this, SLOT(toggleStrategy(bool)));    
}

HriTab::~HriTab()
{

}

void HriTab::toggleStrategy(bool)
{
    if(ui.continiousStrategy->isChecked())
    {
        if(playGround)
            if(playGround->robotPlatforms[0]->intentionRecognizer)
                playGround->robotPlatforms[0]->intentionRecognizer->setInteractionStrategy(CONTINIOUS_INPUT);
    }
    else
    {
        if(playGround)
        {
            if(playGround->robotPlatforms[0]->intentionRecognizer)
            {
                playGround->robotPlatforms[0]->intentionRecognizer->setInteractionStrategy(MINIMAL_INPUT);
            }
            else
                printf("\n Start Intention Recognizer First !!!");
        }
    }
}

void HriTab::toggleVoiceNotification(int state)
{
    if(playGround)
        if(playGround->robotPlatforms[0]->commManager)
            playGround->robotPlatforms[0]->commManager->setSpeechNotification(state);
}

