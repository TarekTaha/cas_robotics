#include "MapPainter.h"

MapPainter::MapPainter(QWidget *parent): 
	QWidget(parent),
	step(1),
	local_planner(NULL),
	global_planner(NULL),
	start_initialized(false),
	end_initialized(false),
	drawPathEnabled(false),
	drawTreeEnabled(false)
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

void   MapPainter::setPathEnabled(int set)
{
	if (set)	
		drawPathEnabled = true;
	else
		drawPathEnabled = false;
}

void MapPainter::drawPath(PathPlanner *p,Pose pose)
{
	this->local_planner = p;
	this->pose = pose;
	drawPathEnabled = true;
	update();
}

void MapPainter::drawPath(PathPlanner *p)
{
	this->global_planner = p;
	drawPathEnabled = true;
	update();
}

void MapPainter::paintEvent(QPaintEvent *)
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
		paint.drawArc(int(start.p.x()-5),int(start.p.y()-5),10,10,0,5760);
		paint.drawPie(int(start.p.x()-5),int(start.p.y()-5),10,10,(RTOD(start.phi)-10)*16,(RTOD(start.phi)+10)*16);
	}
	if(end_initialized)
	{
		paint.drawArc(int(end.p.x()-5),int(end.p.y()-5),10,10,0,5760);
		paint.drawPie(int(end.p.x()-5),int(end.p.y()-5),10,10,(RTOD(end.phi)-10)*16,(RTOD(end.phi)+10)*16);		
	}
	if(drawPathEnabled)
	{
		SearchSpaceNode * temp;
		QPointF p,start,end;
		if(local_planner)
		{
			Pose	relative_p;
			double res = local_planner->map->resolution;
			
			start = this->local_planner->start.p; 
			local_planner->ConvertToPixel(&start);
			end   = this->local_planner->end.p;   
			local_planner->ConvertToPixel(&end);
	
//			qDebug("Start X:%f Y:%f End X:%f Y:%f",start.x(),start.y(),end.x(),end.y());
			relative_p.p.setX( (( pose.p.x() + res*image.width() /2)/res) - local_planner->map->center.x() );
			relative_p.p.setY( ((-pose.p.y() + res*image.height()/2)/res) - local_planner->map->center.y() );		
			relative_p.phi = pose.phi;
			
			start.setX(start.x() + relative_p.p.x()); start.setY(start.y() + relative_p.p.y());
			end.setX    (end.x() + relative_p.p.x());     end.setY(end.y() + relative_p.p.y());		
	
			// Draw the expanded Local Map
			paint.setPen(Qt::blue);
			for(int i=0;i<local_planner->map->width;i++)
				for(int j=0;j<local_planner->map->height;j++)
				{
					if(local_planner->map->data[i][j] == true)
					{
						paint.drawPoint(int(i + relative_p.p.x()),int(j + relative_p.p.y()));
					}
				}
			// Draw Local Search Space
			paint.setPen(Qt::green);
			if(local_planner->search_space)
			{
				temp = local_planner->search_space;
				while(temp!=NULL)
				{
					p = temp->location;
					local_planner->ConvertToPixel(&p);
					p.setX(p.x() + relative_p.p.x());
					p.setY(p.y() + relative_p.p.y());				
					//qDebug("Pixel X:%f Y:%f",p.x(),p.y());
					paint.drawPoint(p);
					temp = temp->next;
				}		
			}
			paint.setBrush(Qt::green);				
			paint.drawPie(int(start.x()),int(start.y()),5,5,0,5760);
			paint.setBrush(Qt::red);
			paint.drawPie(int(end.x()),  int(end.y()),5,5,0,5760);
			// Draw Local Path if it exists
			paint.setPen(Qt::white);
			if(local_planner->path)
			{
					paint.setPen(Qt::yellow);
					QPointF l_start,l_end,E,S;
				  	Node *p;
				  	p = local_planner->path;
					while(p != NULL && p->next!=NULL)
					{
						S =  p->pose.p;
						if (p->next)
						{
							l_start = p->pose.p; l_end = p->next->pose.p;
							local_planner->ConvertToPixel(&l_start); local_planner->ConvertToPixel(&l_end);
							paint.drawLine(l_start + relative_p.p,l_end + relative_p.p);
						}
						p = p->next;
					} 
			}			
		return ;
		}
		if(global_planner)
		{
			// Draw the expanded Global Map
			paint.setPen(Qt::gray);
			for(int i=0;i<global_planner->map->width;i++)
				for(int j=0;j<global_planner->map->height;j++)
				{
					if(global_planner->map->data[i][j] == true)
					{
						paint.drawPoint(i,j);
					}
				}		
			// Draw Global Search Space
			paint.setPen(Qt::red);
			if(global_planner->search_space)
			{
				temp = global_planner->search_space;
				while(temp!=NULL)
				{
					p = temp->location;
					global_planner->ConvertToPixel(&p);
					p.setX(p.x());
					p.setY(p.y());				
					//qDebug("Pixel X:%f Y:%f",p.x(),p.y());
					paint.drawPoint(p);
					temp = temp->next;
				}
			}
			// Draw Global Path if it exists
			paint.setPen(Qt::yellow);
			if(global_planner->path)
			{
					QPointF l_start,l_end,E,S;
				  	Node *p;
				  	p = global_planner->path;
					while(p != NULL && p->next!=NULL)
					{
						S =  p->pose.p;
						if (p->next)
						{
							l_start = p->pose.p; l_end = p->next->pose.p;
							global_planner->ConvertToPixel(&l_start); global_planner->ConvertToPixel(&l_end);
							paint.drawLine(l_start,l_end);
						}
						p = p->next;
					} 
			}		
			// Draw Tree if requested
			paint.setPen(Qt::white);
			if(drawTreeEnabled)
			{
				QPointF l_start,l_end;
				for (unsigned int i =0; i<global_planner->tree.size();i++)
				{
					l_start = global_planner->tree[i].location;
					global_planner->ConvertToPixel(&l_start);
					//cout<<"\n Main X="<<tree[i].location.x<<" Y="<<tree[i].location.y;
					for(int j=0;j<global_planner->tree[i].children.size();j++)
					{
						l_end = global_planner->tree[i].children[j];
						global_planner->ConvertToPixel(&l_end);
						paint.drawLine(l_start,l_end);
					}
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
