#include "loggerview.h"
#include "ui_loggerview.h"

loggerview::loggerview(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::loggerview)
{
    ui->setupUi(this);
    //Logger &logger = getLogger();
    //connect(&logger,SIGNAL(showMsg(int,QString)),this,SLOT(showMsg(int,QString)));
    //connect(&mrEmitter,SIGNAL(shoutLoud(int,QString)),this,SLOT(showMsg(int,QString)));
    //LOG(0,"TEST,TEST,TEST")
}

loggerview::~loggerview()
{
    delete ui;
}

void loggerview::showMsg(int severityLvl,QString msg)
{
    LOG(Logger::Info,"I recieved a msg")
    ui->textLogs->append(msg);
}
