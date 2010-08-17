#ifndef MISSIONCONTROLTAB_H
#define MISSIONCONTROLTAB_H

#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

#include <QWidget>
#include <QVector>
#include "pathplanner.h"

class RobotManager;
class MapViewer;
class PlayGround;
class NavContainer;
class Pose;
class Map;
class QRadioButton;

namespace Ui {
    class MissionControlTab;
}

using namespace CasPlanner;

class MissionControlTab : public QWidget
{
    Q_OBJECT
public:
    explicit MissionControlTab(QWidget *parent,PlayGround *playGround_in);
    ~MissionControlTab();
public Q_SLOTS:
    void save();
    void setNavigation();
    void pathPlan();
    void loadMap();
    void pathFollow();
    void pathTraversed();
    void setStart(Pose);
    void setEnd(Pose);
    void setMap(Map * map);
    void startIntentionRecognition();
    void resetDestinationBelief();
    void updateSelectedRobot(bool);
    void pathFound(Node*);
private:
    Ui::MissionControlTab *ui;
    MapViewer  * mapViewer;
    PlayGround * playGround;
    NavContainer *navContainer;

    QVector <QRadioButton *> availableRobots;
    bool robotInitialization;
    static unsigned *image, *null;
    Node * path;
    static int width, height, components;

private Q_SLOTS:
    void on_generateSearchSpace_released();
};

#endif // MISSIONCONTROLTAB_H
