#include "mapskeleton.h"

MapSkeleton::MapSkeleton(SSkelPtr & ssk):sskel(ssk)
{

}

MapSkeleton::~MapSkeleton()
{

}

void MapSkeleton::clear()
{
    input.clear();
    output.clear();
}

void MapSkeleton::loadMap()
{

//	QFileDialog::getOpenFileName(QString::null, "Polygonal PolygonalRegion Files (*.poly)", this );
//    if ( s.isEmpty() )
//    	return;
// 	std::ifstream in("/home/BlackCoder/workspace/CasPlanner/modules/Voronoi/complex_5.poly");
//	std::ifstream in("/home/BlackCoder/workspace/CasPlanner/modules/Voronoi/map_polys.poly");
// 	std::ifstream in("/home/BlackCoder/workspace/CasPlanner/modules/Voronoi/many_holes.poly");
	std::ifstream in("/home/BlackCoder/workspace/CasPlanner/modules/Voronoi/alley_0.poly");
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

void MapSkeleton::generateInnerSkeleton()
{
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
      	sskel_valid = sskel ;
      	if ( !sskel_valid )
      		std::cout<<"\n Something is WRONG";
	  	else
			std::cout<<"\n Skeleton Generated !!!!!\n";
    }
	else
	{
		std::cout<<"\n Empty Input !!!!!\n";
	}
	fflush(stdout);
}
