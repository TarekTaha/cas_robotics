#ifndef MAPSKELETON_H_
#define MAPSKELETON_H_


#include <vector>
#include<string>
#include<iostream>
#include<sstream>
#include<fstream>
#include<iomanip>
#include<list>
#include<map>

#include <QString>
#include <CGAL/basic.h>
#include <CGAL/Unique_hash_map.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <boost/shared_ptr.hpp>
#include <CGAL/Straight_skeleton_builder_2.h>
#include <CGAL/Polygon_offset_builder_2.h>
#include <CGAL/compute_outer_frame_margin.h>

namespace defs
{
	//typedef CGAL::Simple_cartesian<double> K ;
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef CGAL::Polygon_2<K>            CGAL_Polygon ;
	typedef K::Point_2                    Point;
	typedef std::vector<Point>            Polygon;
	typedef boost::shared_ptr<Polygon>    PolygonPtr;
	typedef CGAL::Segment_2<K>            Segment;
	typedef std::vector<PolygonPtr>       Region ;
	typedef boost::shared_ptr<Region>     RegionPtr ;
	typedef std::vector<RegionPtr>        Regions ;
	typedef std::vector<double>           Doubles ;
	
	typedef CGAL::Straight_skeleton_2<K>                                SSkel;
	typedef CGAL::Straight_skeleton_builder_traits_2<K>                 SSkelBuilderTraits;
	typedef CGAL::Straight_skeleton_builder_2<SSkelBuilderTraits,SSkel> SSkelBuilder;
	typedef CGAL::Polygon_offset_builder_traits_2<K>                          OffsetBuilderTraits;
	typedef CGAL::Polygon_offset_builder_2<SSkel,OffsetBuilderTraits,Polygon> OffsetBuilder;
	typedef SSkel::Halfedge_iterator     Halfedge_iterator;
	typedef SSkel::Vertex_handle         Vertex_handle;
	typedef SSkel::Face_const_iterator   Face_const_iterator;
	typedef SSkel::Halfedge_const_handle Halfedge_const_handle ;
	typedef SSkel::Vertex_const_handle   Vertex_const_handle ;
	typedef boost::shared_ptr<SSkel> SSkelPtr ;

}

using namespace defs;

class MapSkeleton
{
public:
	MapSkeleton();
//	MapSkeleton(Map *);
	virtual ~MapSkeleton();
	bool convertGridToLineWithVoronoi(float minThreshold, float maxThreshold,
												  bool filterByCellValue, float valueToSet);
	void generateInnerSkeleton();
	void loadMap();
	void clear();
	void draw();
	SSkelPtr getSSkelPtr();
private:
    void something_changed()
    {
    	current_state++;
    };
//	Map * ogMap;			
    int current_state;
	SSkelPtr sskel;
	bool     sskel_valid ;
	Regions  input ;
	Regions  output ;
	Doubles  offsets ;
  	const Halfedge_const_handle null_halfedge ;
  	const Vertex_const_handle   null_vertex ;	
};

#endif /*MAPSKELETON_H_*/