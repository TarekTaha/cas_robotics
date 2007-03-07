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
	if(net)
		delete net;    
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
std::ostream &operator<<(std::ostream &str, const TokArr &ta)
{
    return str << String(ta);
}

void MapSkeleton::testModel()
{
    // NodeA  NodeB
    //     \ /
    //    NodeC
    //     / \
    // NodeD  NodeE
    DBN *net;
    net = new DBN();
    net->AddNode(discrete^"NodeA NodeB NodeC NodeD NodeE", "up down right left");
                
    TokArr NodeA = net->GetPTabular("NodeA^up NodeC");
    TokArr NodeB = net->GetPTabular("NodeB^down NodeC");
    TokArr NodeC = net->GetPTabular("NodeC^right NodeD^down");
    TokArr NodeD = net->GetPTabular("NodeD^left NodeC");
            
    std::cout <<"NODE A:"<< NodeA << "\n";
    std::cout <<"NODE B:"<< NodeB << "\n";
    std::cout <<"NODE C:"<< NodeC << "\n";
    std::cout <<"NODE D:"<< NodeD << "\n";

//  The possible choices are: “pearl”, “jtree”, “gibbs”, or “naive” 
/*! For Gibbs Sampling Inference, the number of iterations must be specified. 
 *  The corresponding property name is "GibbsNumberOfIterations", and the default 
 *  number of iterations is 600. Gibbs Sampling Inference works by producing a 
 *  stream of evidence samples that approximate i.i.d. samples; however, it requires 
 *  an initialization period (a "burn-in") at the beginning to allow the samples to 
 *  converge to the correct distribution. After a sufficiently long burn-in period, 
 * 	samples will approximately be drawn from the correct distribution. The number of 
 *  samples to discard during the burn-in can be specified with the property 
 *  "GibbsThresholdIteration" (the default value is 10). You can also choose to generate 
 *  more than one independent stream of samples. The number of streams can be specified 
 *  with the property "GibbsNumberOfStreams" (1 stream by default).
 */
 	net->ClearEvidBuf();
	net->AddEvidToBuf("NodeC^up");

	net->SetProperty("EMMaxNumberOfIterations", "10");
	net->SetProperty("EMTolerance", "1e-4");
//	net->SetProperty("Learning", "bayes");
	net->SetProperty("Learning", "em");
	net->LearnParameters();
			 
//    net->SetProperty("Inference","jtree");
    net->SetProperty("Inference","naive");
//    net->SetProperty("Inference","pearl");
//    net->SetProperty("Inference","gibbs");
//    net->SetProperty("GibbsNumberOfIterations","1000");
//    net->SetProperty("GibbsNumberOfStreams","2");
//    net->SetProperty("GibbsThresholdIteration","100");
   
    TokArr nodeAJDP =  net->GetJPD("NodeA");
    std::cout <<"nodeAJDP:"<< nodeAJDP << "\n";
    
    TokArr nodeBJDP =  net->GetJPD("NodeB");
    std::cout <<"nodeBJDP:"<< nodeBJDP << "\n";
    
    TokArr nodeCJDP =  net->GetJPD("NodeC");
    std::cout <<"nodeCJDP:"<< nodeCJDP << "\n";
    
    TokArr nodeDJDP =  net->GetJPD("NodeD");
    std::cout <<"nodeDJDP:"<< nodeDJDP << "\n";
    
    TokArr nodeEJDP =  net->GetJPD("NodeE");
    std::cout <<"nodeEJDP:"<< nodeEJDP << "\n";
	delete net;
}

//void MapSkeleton::testModel()
//{
//    // NodeA  NodeB
//    //     \ /
//    //    NodeC
//    //     / \
//    // NodeD  NodeE
//    
//    MRF *net;
//    net = new MRF();
//    net->AddNode(discrete^"NodeA NodeB NodeC NodeD NodeE", "up down right left");
//
//    net->SetClique("NodeA NodeB");
//    net->SetClique("NodeA NodeC");
//    net->SetClique("NodeA NodeD");
//    net->SetClique("NodeB NodeC");
//    net->SetClique("NodeB NodeE");
//    net->SetClique("NodeE NodeC");
//    net->SetClique("NodeE NodeD");
//    net->SetClique("NodeD NodeC");
//
//    net->SetPTabular("NodeA NodeC", "0.2 0.2 0.2 0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2");
//    net->SetPTabular("NodeA NodeD", "0.2 0.2 0.2 0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2");
//    net->SetPTabular("NodeA NodeB", "0.2 0.2 0.2 0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2");
//    net->SetPTabular("NodeB NodeC", "0.2 0.2 0.2 0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2");
//    net->SetPTabular("NodeB NodeE", "0.2 0.2 0.2 0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2");
//    net->SetPTabular("NodeE NodeC", "0.2 0.2 0.2 0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2");
//    net->SetPTabular("NodeE NodeD", "0.2 0.2 0.2 0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2");    
//    net->SetPTabular("NodeC NodeD", "0.2 0.2 0.2 0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2");
//                    
//    TokArr NodeA = net->GetPTabular("NodeA^up NodeC");
//    TokArr NodeB = net->GetPTabular("NodeB^down NodeC");
//    TokArr NodeC = net->GetPTabular("NodeC^right NodeD^down");
//    TokArr NodeD = net->GetPTabular("NodeD^left NodeC");
//            
//    std::cout <<"NODE A:"<< NodeA << "\n";
//    std::cout <<"NODE B:"<< NodeB << "\n";
//    std::cout <<"NODE C:"<< NodeC << "\n";
//    std::cout <<"NODE D:"<< NodeD << "\n";
//
////  The possible choices are: “pearl”, “jtree”, “gibbs”, or “naive” 
///*! For Gibbs Sampling Inference, the number of iterations must be specified. 
// *  The corresponding property name is "GibbsNumberOfIterations", and the default 
// *  number of iterations is 600. Gibbs Sampling Inference works by producing a 
// *  stream of evidence samples that approximate i.i.d. samples; however, it requires 
// *  an initialization period (a "burn-in") at the beginning to allow the samples to 
// *  converge to the correct distribution. After a sufficiently long burn-in period, 
// * 	samples will approximately be drawn from the correct distribution. The number of 
// *  samples to discard during the burn-in can be specified with the property 
// *  "GibbsThresholdIteration" (the default value is 10). You can also choose to generate 
// *  more than one independent stream of samples. The number of streams can be specified 
// *  with the property "GibbsNumberOfStreams" (1 stream by default).
// */
// 	net->ClearEvidBuf();
////	net->AddEvidToBuf("NodeA^true");
////	net->AddEvidToBuf("NodeB^true NodeC^right");
////	net->AddEvidToBuf("NodeC^down");
//	net->AddEvidToBuf("NodeC^up");
////	net->AddEvidToBuf("NodeA^true");
//	
//	net->SetProperty("EMMaxNumberOfIterations", "10");
//	net->SetProperty("EMTolerance", "1e-4");
////	net->SetProperty("Learning", "bayes");	
//	net->SetProperty("Learning", "em");
//	net->LearnParameters();
//			 
////    net->SetProperty("Inference","jtree");
//    net->SetProperty("Inference","naive");
////    net->SetProperty("Inference","pearl");    
////    net->SetProperty("Inference","gibbs");
////    net->SetProperty("GibbsNumberOfIterations","1000");
////    net->SetProperty("GibbsNumberOfStreams","2");
////    net->SetProperty("GibbsThresholdIteration","100");
//   
//    TokArr nodeAJDP =  net->GetJPD("NodeA");
//    std::cout <<"nodeAJDP:"<< nodeAJDP << "\n";
//    
//    TokArr nodeBJDP =  net->GetJPD("NodeB");    	
//    std::cout <<"nodeBJDP:"<< nodeBJDP << "\n";
//    
//    TokArr nodeCJDP =  net->GetJPD("NodeC");    	
//    std::cout <<"nodeCJDP:"<< nodeCJDP << "\n"; 
//    
//    TokArr nodeDJDP =  net->GetJPD("NodeD");    	
//    std::cout <<"nodeDJDP:"<< nodeDJDP << "\n";  
//    
//    TokArr nodeEJDP =  net->GetJPD("NodeE");    	
//    std::cout <<"nodeEJDP:"<< nodeEJDP << "\n";              
//	delete net;      
//}

//void MapSkeleton::testModel()
//{
//    // NodeA  NodeB
//    //     \ /
//    //    NodeC
//    //     / \
//    // NodeD  NodeE
//    
//    BayesNet *net;
//    net = new BayesNet();
//    net->AddNode(discrete^"NodeA NodeB NodeC NodeD NodeE", "up down right left");
//    net->AddArc("NodeA", "NodeB NodeC NodeD");
//    net->AddArc("NodeB", "NodeC NodeE");
////    net->AddArc("NodeC", "NodeA NodeB NodeD NodeE");  
////    net->AddArc("NodeD", "NodeA NodeC NodeE");
////    net->AddArc("NodeE", "NodeB NodeC NodeD");
//
////    net->SetPTabular("NodeA NodeB NodeC NodeD NodeE", "0.9 0.1 0 0 0","NodeA^up");
//            
////    net->SetPTabular("NodeC^up"   , "0.2");
////    net->SetPTabular("NodeC^down" , "0.3");
////    net->SetPTabular("NodeC^right", "0.4");
////    net->SetPTabular("NodeC^left" , "0.1");
//
////    net->SetPTabular("NodeA^up NodeA^down ", "0.8 0.2","NodeC^up");
////    net->SetPTabular("NodeB^true NodeB^false", "0.8 0.2","NodeC^right");
////    net->SetPTabular("NodeD^true NodeD^false", "0.8 0.2","NodeC^left");
////    net->SetPTabular("NodeE^true NodeE^false", "0.8 0.2","NodeC^down");            
//
////    net->SetPTabular("NodeA^up NodeA^down NodeA^right NodeA^left", "0.2 0.2 0.2 0.4","NodeC^up");
////    net->SetPTabular("NodeA^up NodeA^down NodeA^right NodeA^left", "0.2 0.2 0.2 0.4","NodeC^down");
////    net->SetPTabular("NodeA^up NodeA^down NodeA^right NodeA^left", "0.2 0.2 0.2 0.4","NodeC^right");
////    net->SetPTabular("NodeA^up NodeA^down NodeA^right NodeA^left", "0.2 0.2 0.2 0.4","NodeC^left");
//        
//    TokArr NodeA = net->GetPTabular("NodeA");
//    TokArr NodeB = net->GetPTabular("NodeB");
//    TokArr NodeC = net->GetPTabular("NodeC");
//    TokArr NodeD = net->GetPTabular("NodeD");
//            
//    std::cout <<"NODE A:"<< NodeA << "\n";
//    std::cout <<"NODE B:"<< NodeB << "\n";
//    std::cout <<"NODE C:"<< NodeC << "\n";
//    std::cout <<"NODE D:"<< NodeD << "\n";
//
////  The possible choices are: “pearl”, “jtree”, “gibbs”, or “naive” 
///*! For Gibbs Sampling Inference, the number of iterations must be specified. 
// *  The corresponding property name is "GibbsNumberOfIterations", and the default 
// *  number of iterations is 600. Gibbs Sampling Inference works by producing a 
// *  stream of evidence samples that approximate i.i.d. samples; however, it requires 
// *  an initialization period (a "burn-in") at the beginning to allow the samples to 
// *  converge to the correct distribution. After a sufficiently long burn-in period, 
// * 	samples will approximately be drawn from the correct distribution. The number of 
// *  samples to discard during the burn-in can be specified with the property 
// *  "GibbsThresholdIteration" (the default value is 10). You can also choose to generate 
// *  more than one independent stream of samples. The number of streams can be specified 
// *  with the property "GibbsNumberOfStreams" (1 stream by default).
// */
//// 	net->ClearEvidBuf();
//////	net->AddEvidToBuf("NodeA^true");
//////	net->AddEvidToBuf("NodeB^true NodeC^right");
//////	net->AddEvidToBuf("NodeC^down");
////	net->AddEvidToBuf("NodeC^up");
//////	net->AddEvidToBuf("NodeA^true");
////	
////	net->SetProperty("EMMaxNumberOfIterations", "10");
////	net->SetProperty("EMTolerance", "1e-4");
//////	net->SetProperty("Learning", "bayes");	
////	net->SetProperty("Learning", "em");
////	net->LearnParameters();
////			 
//////    net->SetProperty("Inference","jtree");
////    net->SetProperty("Inference","naive");
//////    net->SetProperty("Inference","pearl");    
//////    net->SetProperty("Inference","gibbs");
//////    net->SetProperty("GibbsNumberOfIterations","1000");
//////    net->SetProperty("GibbsNumberOfStreams","2");
//////    net->SetProperty("GibbsThresholdIteration","100");
////   
////    TokArr nodeAJDP =  net->GetJPD("NodeA^true");
////    std::cout <<"nodeAJDP:"<< nodeAJDP << "\n";
////    
////    TokArr nodeBJDP =  net->GetJPD("NodeB^true");    	
////    std::cout <<"nodeBJDP:"<< nodeBJDP << "\n";
////    
////    TokArr nodeDJDP =  net->GetJPD("NodeD^true");    	
////    std::cout <<"nodeDJDP:"<< nodeDJDP << "\n";  
////    
////    TokArr nodeEJDP =  net->GetJPD("NodeE^true");    	
////    std::cout <<"nodeEJDP:"<< nodeEJDP << "\n";              
//	delete net;      
//}

void MapSkeleton::generateInnerSkeleton()
{
	Vertex v,vOpp;
	if(net)
		delete net;
	net = new BayesNet();
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
						//if (i == -1)
						{
          					verticies.push_back(v);
          					QString node_name= QString("node%1").arg(verticies.size());
          					net->AddNode(discrete^qPrintable(node_name), "right left up down");
          					verticies.push_back(vOpp);
          					node_name= QString("node%1").arg(verticies.size());
          					net->AddNode(discrete^qPrintable(node_name), "right left up down");          					
         					//net->SetClique(qPrintable(QString("node%1 node%2").arg(verticies.size()-1).arg(verticies.size())));
         					net->AddArc(qPrintable(QString("node%1").arg(verticies.size()-1)),qPrintable(QString("node%1").arg(verticies.size())));
						}
//				    	he->is_inner_bisector()? glColor4f(0,0,1,1) : glColor4f(1,0,0,1);
	          		}	
	        	}
	        	he = he->next();
	      	}
	      	while ( -- watchdog > 0 && he != hstart ) ;	      	
    	}
	    float defaultProb = 1.0f / 4.0f;
    	TokArr P = net->GetPTabular("node1","node2^down");
    	String PCloudyStr = String(P);
    	std::cout<<"\nProb "<<PCloudyStr;
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
