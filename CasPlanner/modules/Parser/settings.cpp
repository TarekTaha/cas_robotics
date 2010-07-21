#include "settings.h"
#include "logger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>

CasPlannerSettings::CasPlannerSettings( QObject* parent )
{
    QCoreApplication::setOrganizationName("CasPlanner");
    QCoreApplication::setOrganizationDomain("cas-planner.sf.net");
    QCoreApplication::setApplicationName("CasPlanner");
}
