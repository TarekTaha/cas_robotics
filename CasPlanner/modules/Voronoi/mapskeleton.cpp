#include "mapskeleton.h"

MapSkeleton::MapSkeleton(SSkelPtr & ssk):sskel(ssk)
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

//pnlw::MRF * MapSkeleton::GridMRFModel()
//{
////    node0 -- node1 -- node2
////      |        |        |
////    node3 -- node4 -- node5
////      |        |        |
////    node6 -- node7 -- node8
//    // node values
//    TokArr aChoice = "value0 value1";// possible values for nodes
//    // MRF creation
//    MRF *pMRF; 
//    pMRF = new MRF();
//    // nodes creation 
//    pMRF->AddNode(discrete ^ "node0 node1 node2 node3 node4 node5 node6 node7 node8", aChoice); 
//   
//    pMRF->SetClique("node0 node1"); 
//    pMRF->SetPTabular("node0 node1", "0.3 0.39 0.1 0.21");
//    pMRF->SetPTabular("node1 node2", "0.26 0.34 0.24 0.16");
//    pMRF->SetPTabular("node0 node3", "0.29 0.26 0.14 0.31");
//    pMRF->SetPTabular("node1 node4", "0.45 0.04 0.33 0.18");
//    pMRF->SetPTabular("node2 node5", "0.17 0.3 0.3 0.23");
//    pMRF->SetPTabular("node3 node4", "0.4 0.06 0.34 0.2");
//    pMRF->SetPTabular("node4 node5", "0.23 0.4 0.15 0.22");
//    pMRF->SetPTabular("node6 node3", "0.34 0.24 0.33 0.09");
//    pMRF->SetPTabular("node4 node7", "0.34 0.29 0.17 0.2");
//    pMRF->SetPTabular("node5 node8", "0.14 0.31 0.14 0.41");
//    pMRF->SetPTabular("node6 node7", "0.15 0.34 0.01 0.5");
//    pMRF->SetPTabular("node7 node8", "0.14 0.46 0.21 0.19");
//
//    return pMRF;
//}
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
	Vertex v;
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
	       			    int i = verticies.indexOf(v);
    						if (i == -1)
	          					verticies.push_back(v);
//				    	he->is_inner_bisector()? glColor4f(0,0,1,1) : glColor4f(1,0,0,1);
	          		}	
	        	}
	        	he = he->next();
	      	}
	      	while ( -- watchdog > 0 && he != hstart ) ;	      	
    	}
		std::cout<<"\n Skeleton Generated !!!!!\n";    	
	}
	else
	{
		std::cout<<"\n Empty Input !!!!!\n";
	}
	fflush(stdout);
}
