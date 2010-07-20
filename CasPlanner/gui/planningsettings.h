#ifndef PLANNINGSETTINGS_H
#define PLANNINGSETTINGS_H

#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

#include <QWidget>
#include "playground.h"

class PlanningTab;
class RobotManager;
class PlayGround;

namespace Ui {
    class PlanningSettings;
}

class PlanningSettings : public QWidget
{
    Q_OBJECT

public:
    explicit PlanningSettings(QWidget *parent, PlayGround *playG);
    ~PlanningSettings();
public slots:
    void updateSelectedObject(double);
    void updateSelectedAvoidanceAlgo(bool);
    void updateCheckboxStates(int);
    void updateDisplaySettings(int);
private:
    void initialiseGUI();
    PlayGround   *playGround;
    RobotManager *currRobot;
    Ui::PlanningSettings *ui;
};

#endif // PLANNINGSETTINGS_H
