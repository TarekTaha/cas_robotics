#ifndef PLANNINGSETTINGS_H
#define PLANNINGSETTINGS_H

#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

#include <QWidget>

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
public Q_SLOTS:
    void updateSelectedObject(double);
    void updateSelectedAvoidanceAlgo(bool);
    void updateCheckboxStates(int);
    void updateDisplaySettings(int);
private:
    void initialiseGUI();
    void saveSettings();
    PlayGround   *playGround;
    RobotManager *currRobot;
    Ui::PlanningSettings *ui;
};

#endif // PLANNINGSETTINGS_H
