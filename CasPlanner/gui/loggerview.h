#ifndef LOGGERVIEW_H
#define LOGGERVIEW_H

#include <QWidget>
#include "logger.h"

namespace Ui
{
    class loggerview;
}

class loggerview : public QWidget
{
    Q_OBJECT

public:
    explicit loggerview(QWidget *parent = 0);
    ~loggerview();
public slots:
    void showMsg(int severityLvl,QString msg);
private:
    Ui::loggerview *ui;
};

#endif // LOGGERVIEW_H
