#include "loggerview.h"
#include "ui_loggerview.h"

loggerview::loggerview(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::loggerview)
{
    ui->setupUi(this);
}

loggerview::~loggerview()
{
    delete ui;
}

void loggerview::logMsg(int severityLvl,QString msg)
{
    ui->textLogs->append(msg);
}
