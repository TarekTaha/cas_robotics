#ifndef SETTINGS_H
#define SETTINGS_H

#include <QMap>
#include <QPoint>
#include <QSettings>
#include <QSize>
#include <QStringList>
#include <QMutex>
#include <QMutexLocker>
#include <QCoreApplication>

class Planning : public QSettings
{
public:
    Planning()
    {
        beginGroup( "Planning" );
    }
};


class DisplaySettings : public QSettings
{
public:
    DisplaySettings()
    {
        beginGroup( "Display" );
    }
};

class CasPlannerSettings : public QSettings
{
    Q_OBJECT   
 public:
    CasPlannerSettings( QObject* parent );
    void setCurrentUsername( QString username );

    QByteArray splitterState() const { return QSettings().value( "splitterState" ).toByteArray(); }
    void setSplitterState( QByteArray state ) { QSettings().setValue( "splitterState", state ); }

    int sidebarWidth() const { return QSettings().value( "sidebarWidth", 190 ).toInt(); }
    void setSidebarWidth( const int width ) { QSettings().setValue( "sidebarWidth", width ); }

    void setIsManualIpod( QString uid, bool b ) { DisplaySettings().setValue( uid + "/isManualIpod", b ); }
    bool isManualIpod( QString uid ) const { return DisplaySettings().value( uid + "/isManualIpod", false ).toBool(); }

    int  volume()     const { return QSettings().value( "volume", 50 ).toInt(); }
    void setVolume( int v ) { QSettings().setValue( "volume", v ); }

    QString browser()            const { return QSettings().value( "Browser" ).toString(); }
    void setBrowser( QString browser ) { QSettings().setValue( "Browser", browser ); }
};


namespace Settings
{
    inline CasPlannerSettings &settings()
    {
        static QMutex mutex;
        static CasPlannerSettings* settings = 0;
        QMutexLocker locker( &mutex );
        if (!settings)
        {
            settings = QCoreApplication::instance()->findChild<CasPlannerSettings*>( "CasPlanner-Settings-Instance" );
            if (!settings)
            {
                settings = new CasPlannerSettings( QCoreApplication::instance());
                settings->setObjectName( "CasPlanner-Settings-Instance" );
            }
        }
        return *settings;
    }
}

#endif
