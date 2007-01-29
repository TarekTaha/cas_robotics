#include "mapskeleton.h"

MapSkeleton::MapSkeleton(SSkelPtr & ssk):
sskel(ssk),
net(NULL)
{

}

MapSkeleton::~MapSkeleton()
{
	clear();
}

void MapSkeleton::clear()
{
    input.clear();
    output.clear();
}

void MapSkeleton::loadMap(QString fileName)
{
	std::ifstream in(qPrintable(fileName));
    if ( in )
    {
    	CGAL::set_ascii_mode(in);
      	input.clear();
      	defs::RegionPtr lRegion( new defs::Region() ) ;
      	int ccb_count ;
      	in >> ccb_count ;

     	for ( int i = 0 ; i < ccb_count ; ++ i )
      	{
        	defs::PolygonPtr lPoly( new Polygon() );
        	int v_count ;
        	in >> v_count ;
        	for ( int j = 0 ; j < v_count ; ++ j )
        	{
          		double x,y ;
          		in >> x >> y ;
          		lPoly->push_back( Point(x,y) ) ;
        	}
        	if ( lPoly->size() >= 3 )
        	{
          		CGAL::Orientation expected = ( i == 0 ? CGAL::COUNTERCLOCKWISE : CGAL::CLOCKWISE ) ;
          		if (  !CGAL::is_simple_2(lPoly->begin(),lPoly->end())
             	|| CGAL::orientation_2(lPoly->begin(),lPoly->end()) == expected
             		)
               		lRegion->push_back(lPoly);
          		else 
          			lRegion->push_back( PolygonPtr( new Polygon(lPoly->rbegin(),lPoly->rend()) ) ) ;
        	}
      }
      input.push_back(lRegion);
	  std::cout<<"\n Map Loaded Successfully";
    }
	else
	{
		std::cout<<"\n File Not Found";
	}
    output.clear();
}

SSkelPtr MapSkeleton::getSSkelPtr()
{
	return this->sskel;
}
//void MapSkeleton::Someting()
//{
//  	const Halfedge_const_handle null_halfedge ;
//  	const Vertex_const_handle   null_vertex ;		
//    if ( !this->sskel )
//      return ;
//    int watchdog_limit = sskel->size_of_halfedges();
//    	
//	for ( Face_const_iterator fit = sskel->faces_begin(), efit = sskel->faces_end(); fit != efit; ++ fit)
//    {
//      	Halfedge_const_handle hstart = fit->halfedge();
//     	Halfedge_const_handle he     = hstart ;
//      	int watchdog = watchdog_limit ;
//      	do
//      	{
//        	if ( he == null_halfedge )
//          		break ;
//        	if ( he->is_bisector() )
//        	{
//	          	bool lVertexOK      = he->vertex() != null_vertex ;
//	          	bool lOppositeOK    = he->opposite() != null_halfedge ;
//	          	bool lOppVertexOK   = lOppositeOK && he->opposite()->vertex() != null_vertex ;
//	          	bool lVertexHeOK    = lVertexOK && he->vertex()->halfedge() != null_halfedge ;
//	          	bool lOppVertexHeOK = lOppVertexOK && he->opposite()->vertex()->halfedge() != null_halfedge ;
//          		if ( lVertexOK && lOppVertexOK && lVertexHeOK && lOppVertexHeOK )
//          		{
////			    	he->is_inner_bisector()? glColor4f(0,0,1,1) : glColor4f(1,0,0,1);
////					glBegin(GL_LINES);
////						//if(he->opposite()->vertex()->is_skeleton())
////						glVertex2f(he->opposite()->vertex()->point().x(),he->opposite()->vertex()->point().y());
////		    			glVertex2f(he->vertex()->point().x(),he->vertex()->point().y());
////					glEnd();
////					drawProbHisto(QPointF(he->vertex()->point().x(),he->vertex()->point().y()),1.0);		
//          		}	
//        	}
//        	he = he->next();
//      	}
//      	while ( -- watchdog > 0 && he != hstart ) ;
//    }
//};
	
void MapSkeleton::generateInnerSkeleton()
{
	Vertex v,vOpp;
	if(net)
		delete net;
	net = new DBN();
    if ( input.size() > 0 )
	{
		defs::Region const& lRegion = *input.front();
      	SSkelBuilder builder ;
      	for( defs::Region::const_iterator bit = lRegion.begin(), ebit = lRegion.end() ; bit != ebit ; ++ bit )
      	{
      		builder.enter_contour((*bit)->begin(),(*bit)->end());
			//std::cout<<"\n Begin"<<(*bit)->begin()->x();
     	}
      	sskel = builder.construct_skeleton() ;
	  	const Halfedge_const_handle null_halfedge ;
	  	const Vertex_const_handle   null_vertex ;		
      	if ( !sskel )
      	{
      		std::cout<<"\n Something is WRONG";
      		return;
      	}
	    int watchdog_limit = sskel->size_of_halfedges();  	
		for ( Face_const_iterator fit = sskel->faces_begin(), efit = sskel->faces_end(); fit != efit; ++ fit)
	    {
	      	Halfedge_const_handle hstart = fit->halfedge();
	     	Halfedge_const_handle he     = hstart ;
	      	int watchdog = watchdog_limit ;
	      	do
	      	{
	        	if ( he == null_halfedge )
	          		break ;
	        	if ( he->is_bisector() )
	        	{
		          	bool lVertexOK      = he->vertex() != null_vertex ;
		          	bool lOppositeOK    = he->opposite() != null_halfedge ;
		          	bool lOppVertexOK   = lOppositeOK && he->opposite()->vertex() != null_vertex ;
		          	bool lVertexHeOK    = lVertexOK && he->vertex()->halfedge() != null_halfedge ;
		          	bool lOppVertexHeOK = lOppVertexOK && he->opposite()->vertex()->halfedge() != null_halfedge ;
	          		if ( lVertexOK && lOppVertexOK && lVertexHeOK && lOppVertexHeOK )
	          		{
	          			v.setLocation(he->vertex()->point().x(),he->vertex()->point().y());
	          			vOpp.setLocation(he->opposite()->vertex()->point().x(),he->opposite()->vertex()->point().y());
	          			std::cout<<"\nV x:"<<v.location.x()<<" y:"<<v.location.y();
	          			std::cout<<" Vopp x:"<<vOpp.location.x()<<" y:"<<vOpp.location.y();	          			
	       			    int i = verticies.indexOf(v);
						if (i == -1)
						{
          					verticies.push_back(v);
          					QString node_name= QString("node%1").arg(verticies.size());
          					net->AddNode(discrete^qPrintable(node_name), "right left up down");
          					verticies.push_back(vOpp);
          					node_name= QString("node%1").arg(verticies.size());
          					net->AddNode(discrete^qPrintable(node_name), "right left up down");          					
         					net->SetClique(qPrintable(QString("node%1 node%2").arg(verticies.size()-1).arg(verticies.size())));
						}
//				    	he->is_inner_bisector()? glColor4f(0,0,1,1) : glColor4f(1,0,0,1);
	          		}	
	        	}
	        	he = he->next();
	      	}
	      	while ( -- watchdog > 0 && he != hstart ) ;	      	
    	}
//	    float defaultProb = 1.0f / 4.0f;
//    	TokArr P = net->GetPTabular("node1^up node2");
//	    for(int i = 0; i < 4; i++)
//	    {
//	        if( P[i].FltValue() != defaultProb )
//	        {
//	        	PNL_THROW(pnl::CAlgorithmicException, "Setting or getting of tabular parameters for MRF is wrong");
//	        }
//	        else
//				std::cout<<"\nP"<<i<<":"<<P[i].FltValue();	        
//	    }
		std::cout<<"\nSkeleton Generated !!!!!"; 
//		std::cout<<"\n Number of Nodes=:"<<net->GetNumberOfNodes()<<" Verticies size=:"<<verticies.size(); 	
	}
	else
	{
		std::cout<<"\n Empty Input !!!!!\n";
	}
	fflush(stdout);
}
