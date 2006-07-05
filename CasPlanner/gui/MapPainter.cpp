#include "MapPainter.h"

MapPainter::MapPainter(QWidget *parent): 
	QWidget(parent),
	start_initialized(false),
	end_initialized(false),
	drawPathEnabled(false),
	drawTreeEnabled(false),
	step(1)
{
	
	if(!image.load("/home/BlackCoder/workspace/CasPlanner/resources/casareaicp.png", 0))
	{
		qWarning("Error Loading Image");
		exit(1);
	}
	image.setColor(0, Qt::white);
}

Pose MapPainter::getStart()
{
	return this->start;	
}

Pose MapPainter::getEnd()
{
	return this->end;
}

QImage MapPainter::getImage()
{
	return this->image;
}

void MapPainter::drawPath(PathPlanner *p)
{
	this->planner = p;
	drawPathEnabled = true;
	update();
}

void MapPainter::paintEvent(QPaintEvent *ev)
{
	//qDebug("Paint");
	QPainter paint(this);
	paint.drawImage(0, 0, image, 0, 0,image.width(), image.height());
	paint.setBrush(Qt::cyan);
	paint.setPen(Qt::white);
	switch (step)
	{
		case 2:
			paint.drawLine(start.p,mouse_pos);
			break;
		case 4:
			paint.drawLine(end.p,mouse_pos);
			break;
	}
	if(start_initialized)
	{
		paint.drawArc(start.p.x()-5,start.p.y()-5,10,10,0,5760);
		paint.drawPie(start.p.x()-5,start.p.y()-5,10,10,(RTOD(start.phi)-10)*16,(RTOD(start.phi)+10)*16);
	}
	if(end_initialized)
	{
		paint.drawArc(end.p.x()-5,end.p.y()-5,10,10,0,5760);
		paint.drawPie(end.p.x()-5,end.p.y()-5,10,10,(RTOD(end.phi)-10)*16,(RTOD(end.phi)+10)*16);		
	}
	if(drawPathEnabled)
	{
		SearchSpaceNode * temp;
		QPointF p;
		// Draw All the Map
		paint.setPen(Qt::blue);
		for(int i=0;i<planner->map_width - 1;i++)
			for(int j=0;j<planner->map_height - 1 ;j++)
			{
				if(planner->map[i][j]==true)
					paint.drawPoint(i,j);
			}
		// Draw Search Space
		if(planner->search_space)
		{
			paint.setPen(Qt::red);
			temp = planner->search_space;
			while(temp!=NULL)
			{
				p = temp->location;
				planner->ConvertToPixel(&p);
				//qDebug("Pixel X:%f Y:%f",p.x(),p.y());
				paint.drawPoint(p);
				temp = temp->next;
			}		
		}
		// Draw Path if it exists
		if(planner->path)
		{
				paint.setPen(Qt::yellow);
				QPointF l_start,l_end,E,S;
			  	Node *p;
			  	p = planner->path;
				while(p != NULL && p->next!=NULL)
				{
					S =  p->pose.p;
					if (p->next)
					{
						l_start = p->pose.p; l_end = p->next->pose.p;
						planner->ConvertToPixel(&l_start); planner->ConvertToPixel(&l_end);
						paint.drawLine(l_start,l_end);
					}
					p = p->next;
				} 
		}
		if(drawTreeEnabled)
		{
			paint.setPen(Qt::white);
			QPointF l_start,l_end;
			for (unsigned int i =0; i<planner->tree.size();i++)
			{
				l_start = planner->tree[i].location;
				planner->ConvertToPixel(&l_start);
				//cout<<"\n Main X="<<tree[i].location.x<<" Y="<<tree[i].location.y;
				for(int j=0;j<planner->tree[i].children.size();j++)
				{
					l_end = planner->tree[i].children[j];
					planner->ConvertToPixel(&l_end);
					paint.drawLine(l_start,l_end);
				}
			}
		}		
	}
}

void MapPainter::mouseMoveEvent ( QMouseEvent * me )
{
	mouse_pos.setX(me->x());
	mouse_pos.setY(me->y());
	//image.setPixel(mouse_pos.x(),mouse_pos.y(),color);
	//update(mouse_pos.x()-25,mouse_pos.y()-25,50,50);
	update();
    //qDebug("Mouse Moving x: %f y: %f",x,y); 	
}

void MapPainter::mousePressEvent ( QMouseEvent * me )
{
	double x = me->x();
	double y = me->y();
	setMouseTracking(true);
	switch (step)
	{
		case 1:
			start.p.setX(x);
			start.p.setY(y);	
			step++;
			break;
		case 2:
			// Delta swapped becuase of image coordinate		
			start.phi = atan2(start.p.y()-y,x-start.p.x());
			qDebug("Start Angle =%f",RTOD(start.phi));
			start_initialized = true;
			step++;
			break;
		case 3:
			end.p.setX(x);
			end.p.setY(y);	
			step++;
			break;
		case 4:
			// Delta swapped becuase of image coordinate
			end.phi = atan2(end.p.y()-y,x-end.p.x());
			qDebug("End Angle =%f",RTOD(end.phi));		
			end_initialized = true;
			step++;
			break;
		default:
			step = 1;
			setMouseTracking(false);
			start_initialized = false;
			end_initialized   = false;
	}
}

MapPainter::~MapPainter()
{
}
