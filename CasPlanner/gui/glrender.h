#ifndef GLRENDER_H
#define GLRENDER_H
#include <QtOpenGL> 

// Class that encapsulates something that knows how to render itself in OpenGL
// Basic assumptions: Objects render themselves in an area that extends from (0,0) to (1,1) 
// Look, this class is empty for now, but more functions could be added later. 
class GLRender: public QObject 
{
Q_OBJECT
    public: 
        GLRender(QGLWidget *in_w): w(in_w){}; 
        virtual void render()=0; 
        virtual ~GLRender(){
        }
                
    public slots:
        virtual void updateData()=0; 
    protected:
        QGLWidget *w; 
};

#endif
