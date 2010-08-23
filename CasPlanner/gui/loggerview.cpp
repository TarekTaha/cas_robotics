#include "loggerview.h"
#include "ui_loggerview.h"
#include "logger.h"
#include <iostream>

loggerview::loggerview(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::loggerview)
{
    ui->setupUi(this);
    ui->textLogs->setReadOnly(true);
    Logger &logger = Logger::getLogger();
    connect(&logger,SIGNAL(showMsg(int,QString)),this,SLOT(showMsg(int,QString)));
    ui->textLogs->setTextBackgroundColor(Qt::gray);
//    QPalette palette = ui->textLogs->palette();
//    palette.setColor(QPalette::Background, Qt::gray);
}

loggerview::~loggerview()
{
    delete ui;
}

void loggerview::showMsg(int severityLvl,QString msg)
{
    switch (severityLvl)
    {
    case Logger::Info :
        ui->textLogs->setTextColor(Qt::black);
        break;
    case Logger::Critical :
        ui->textLogs->setTextColor(Qt::red);
        break;
    case Logger::Warning :
        ui->textLogs->setTextColor(Qt::yellow);
        break;
    default:
        ui->textLogs->setTextColor(Qt::black);
    }
    ui->textLogs->append(msg);
}
