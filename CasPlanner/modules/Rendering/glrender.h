#ifndef GLRENDER_H
#define GLRENDER_H
#include <QtOpenGL> 

// Basic assumptions: Objects render themselves in an area that extends from (0,0) to (1,1) 
class GLRender: public QObject 
{
Q_OBJECT
    public: 
        GLRender(QGLWidget *in_w): w(in_w){}; 
        virtual void render()=0; 
        virtual ~GLRender()
        {
        }
    public slots:
        virtual void updateData()=0; 
    protected:
        QGLWidget *w; 
};

#endif
