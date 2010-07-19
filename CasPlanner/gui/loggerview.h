#ifndef LOGGERVIEW_H
#define LOGGERVIEW_H

#include <QWidget>

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
    void logMsg(int severityLvl,QString msg);
private:
    Ui::loggerview *ui;
};

#endif // LOGGERVIEW_H
