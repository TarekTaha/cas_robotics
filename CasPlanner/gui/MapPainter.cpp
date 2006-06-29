#include "MapPainter.h"

MapPainter::MapPainter(QWidget *parent): QWidget(parent)
{
	if(!image.load("/home/BlackCoder/workspace/CasPlanner/resources/casareaicp.png", 0))
	{
		qWarning("Error Loading Image");
		exit(1);
	}
	color = qRgb(255, 255, 255);
	image.setColor(0, color);
	
}

void MapPainter::paintEvent(QPaintEvent *ev)
{
	qDebug("Paint");
	QPainter paint(this);
	paint.drawImage(0, 0, image, 0, 0,image.width(), image.height());
}

void MapPainter::mouseMoveEvent ( QMouseEvent * me )
{
	double x = me->x();
	double y = me->y();
	image.setPixel(x,y,color);
	update(x-10,y-10,20,20);
    //qDebug("Mouse Moving x: %f y: %f",x,y); 	
}

void MapPainter::mousePressEvent ( QMouseEvent * me )
{
	double x = me->x();
	double y = me->y();
	image.setPixel(x,y,0);
	update(x-10,y-10,20,20);
    //qDebug("Mouse pressed x: %f y: %f",x,y); 
}

MapPainter::~MapPainter()
{
}
