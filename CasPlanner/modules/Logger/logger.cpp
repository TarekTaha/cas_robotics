/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/

#include "logger.h"
#include <QSysInfo>
#include <QFile>

using namespace std;

void loggingMsgHandler( QtMsgType   type, const char* msg )
{
#ifdef QT_NO_DEBUG
    // Release build, redirect to log file
    switch (type)
    {
        case QtDebugMsg:
            Logger::getLogger().loggingPatch( msg );
            break;
        case QtWarningMsg:
            LOG(Logger::Warning, msg);
            break;
        case QtCriticalMsg:
            LOG(Logger::Critical, msg);
            break;
        case QtFatalMsg:
            LOG(Logger::Critical, msg);
            Logger::getLogger().mDefaultMsgHandler(type, msg);
            break;
    }
#else
    // Debug build, use default handler
    QtMsgHandler& defHandler = Logger::getLogger().mDefaultMsgHandler;
    switch (type)
    {
        case QtDebugMsg:
            defHandler(type, msg);
            break;
        case QtWarningMsg:
        case QtCriticalMsg:
            defHandler(type, msg);
            #ifdef ASSERT_ON_QT_WARNINGS
                Q_ASSERT(!"Qt warning, might be a good idea to fix this");
            #endif
            break;
        case QtFatalMsg:
            defHandler(type, msg);
            break;
    }
#endif // QT_NO_DEBUG
}

static void defaultMsgHandler(QtMsgType   type, const char* msg )
{
    fprintf(stderr, "%s\n", msg);
    fflush(stderr);
    if (type == QtFatalMsg || (type == QtWarningMsg && (!qgetenv("QT_FATAL_WARNINGS").isNull())) )
    {
        #if defined(Q_OS_UNIX) && defined(QT_DEBUG)
            abort(); // trap; generates core dump
        #else
            exit(1);
        #endif
    }
}

void Logger::init( QString sFilename, bool bOverwrite)
{
    mFilePath = sFilename;
    // Trim file size if above 500k
    QFile qf( sFilename );
    if ( qf.size() > 500000 )
    {
        ifstream inFile( qPrintable(sFilename) );
        inFile.seekg( static_cast<streamoff>( qf.size() - 400000 ) );
        istreambuf_iterator<char> bufReader( inFile ), end;
        string sFile;
        sFile.reserve( 400005 );
        sFile.assign( bufReader, end );
        inFile.close();
        ofstream outFile( qPrintable(sFilename) );
        outFile << sFile << flush;
        outFile.close();
    }

    ios::openmode flags = ios::out;
    if (!bOverwrite)
    {
        flags |= ios::app;
    }
    mFileOut.open(qPrintable(sFilename), flags);

    if (!mFileOut)
    {
        qWarning() << "Could not open log file" << sFilename;
        return;
    }

    setLevel(Warning);

    // Print some initial startup info
    LOG(1, "************************************* STARTUP ********************************************")
    LOG(1, "*                                    CasPlanner v1.0")
    LOG(1, "******************************************************************************************")
    // Install message handler for dealing with qDebug output
    mDefaultMsgHandler = qInstallMsgHandler(loggingMsgHandler);
    if (mDefaultMsgHandler == 0)
    {
        mDefaultMsgHandler = defaultMsgHandler;
    }
}

void Logger::log( Severity level, string message, string function, int line)
{
    QMutexLocker loggerLock(&mMutex);
    if (mFileOut && level <= getLevel())
    {
        mFileOut<< QDateTime::currentDateTime().toUTC().toString( "dd/MM/yy hh:mm:ss" ) \
                                      <<" - "<< QString("%1").arg( (int)QThread::currentThreadId(), 10 ) \
                                      <<" - "<< function << '(' << line<< ") - L"<<level<<" - "<<message<<endl;
    }
    Q_EMIT showMsg(level,QString("%1").arg(message.c_str()));
}
/*
 Needed to show the actual line and function of the first call location
 */
void Logger::loggingPatch(const char* msg)
{
    QMutexLocker loggerLock(&mMutex);
    if (mFileOut)
    {
        mFileOut << msg << std::endl;
    }
    Q_EMIT showMsg(Debug,QString("%1").arg(msg));
}
