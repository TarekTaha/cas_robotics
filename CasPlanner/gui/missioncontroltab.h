#ifndef MISSIONCONTROLTAB_H
#define MISSIONCONTROLTAB_H

#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

#include <QWidget>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QPushButton>
#include <QHash>
#include "mapviewer.h"
#include "playground.h"
#include "node.h"
#include "configfile.h"
#include "logger.h"

class RobotManager;
class MapViewer;
class PlayGround;
class NavContainer;

using namespace CasPlanner;

namespace Ui {
    class MissionControlTab;
}

class MissionControlTab : public QWidget
{
    Q_OBJECT
public:
    explicit MissionControlTab(QWidget *parent,PlayGround *playGround_in);
    ~MissionControlTab();
public slots:
    void save();
    void setNavigation();
    void pathPlan();
    void loadMap();
    void generateSpace();
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
};

#endif // MISSIONCONTROLTAB_H
