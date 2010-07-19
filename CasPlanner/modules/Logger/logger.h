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

#ifndef LOGGER_H
#define LOGGER_H

#include <QMutex>
#include <QThread>
#include <QString>
#include <QDebug>
#include <QtGlobal>

#include <string>
#include <fstream>
#include <sstream>
#include <ios>
#include <iomanip>

#include <ctime>
#include <cstdlib>

#define FUNCTION_NAME ( std::string( __FUNCTION__ ) )
#define LINE_NUMBER ( __LINE__ )

// Global LOG macro
#define LOG(level, msg)                                                      \
{                                                                            \
    std::ostringstream ss;                                                   \
    ss << msg;                                                               \
    Logger& lg = Logger::getLogger();                                        \
    lg.log( (Logger::Severity)level, ss.str(), FUNCTION_NAME, LINE_NUMBER ); \
}

#define LOGL(level, msg) LOG(level, msg << "\n")

inline std::ostream& operator<<(std::ostream& os, const QString& qs)
{
    os << qs.toAscii().data();
    return os;
}

class Logger : public QObject
{
    Q_OBJECT
public:   
    enum Severity
    {
        Critical = 1,
        Warning,
        Info,
        Debug
    };
    std::ofstream mFileOut;
    QMutex mMutex;
    QtMsgHandler mDefaultMsgHandler;    
    Logger() : mDefaultMsgHandler(NULL), mLevel(Warning) {}
    virtual ~Logger()
    {
        mFileOut.close();
    }    
    void init(QString sFilename, bool bOverwrite = true);
    static Logger& getLogger();
    void log(Severity level, std::string message, std::string function, int line );
    void setLevel( Severity level) { mLevel = level; }
    int  getLevel() { return mLevel; }
    QString getFilePath() const { return mFilePath; }
    static std::string
            GetTime()
    {
        time_t now;
        time(&now);
        struct tm* tmnow;
        tmnow = gmtime(&now);
        char acTmp[128];
        strftime(acTmp, 127, "%y%m%d %H:%M:%S", tmnow);
        std::string sTime(acTmp);
        return sTime;
    }
    void loggingPatch( const char* msg );
    signals:
        void logMsg(int severityLevel,QString msg);
private:
    Severity mLevel;
    QString mFilePath;
};

#include <QDebug>
#ifndef QT_NO_DEBUG

    #include <QTime>
    #include <QVariant>
    class QDebugBlock
    {
        mutable QString m_title; //because operator= requires a const parameter for some reason :(
        QTime m_time;
        static int &indents() { static int indent = 0; return indent; }
    public:
        QDebugBlock( QString title ) : m_title( title )
        {
            // use data() to prevent quotes being output by QDebugStream
            debug() << "BEGIN:" << title.toLatin1().data();
            indents()++;

            m_time.start();
        }

        QDebugBlock( const QDebugBlock& that )
        {
            *this = that;
        }

        QDebugBlock &operator=( const QDebugBlock& block )
        {
            m_title = block.m_title;
            m_time = block.m_time;

            block.m_title = "";

            return *this;
        }

        ~QDebugBlock()
        {
            if (!m_title.isEmpty())
            {
                indents()--;
                // use data() to prevent quotes being output by QDebugStream
                debug() << "END:  " << m_title.toLatin1().data() << "[elapsed:" << m_time.elapsed() << "ms]";
            }
        }

        static QDebug debug()
        {
            QDebug d( QtDebugMsg );
            int i = indents() * 2;
            while (i--)
                d.space();

            return d;
        }
        /**
         * so you can do: Q_DEBUG_BLOCK << somestring;
         */
        QDebugBlock &operator<<( const QVariant& v )
        {
            QString const s = v.toString();
            if (!s.isEmpty())
                debug() << s;

            return *this;
        }
    };

    #define Q_DEBUG_BLOCK QDebugBlock mxcl_block = QDebugBlock( __PRETTY_FUNCTION__ )
    #define qDebug() QDebugBlock::debug()
#else //Q_NO_DEBUG
    #include <QDateTime>   
    #define Q_DEBUG_BLOCK qDebug()
    class QDebugBlock { public: QDebugBlock( QString ) {} };    
    #define qDebug() qDebug() << QDateTime::currentDateTime().toUTC().toString( "dd/MM/yy hh:mm:ss" ) \
                              << '-' << QString("%1").arg( (int)QThread::currentThreadId(), 4 ) \
                              << '-' << __PRETTY_FUNCTION__ << '(' << __LINE__<< ") - L4 - "
#endif


#endif //LOGGER_H
