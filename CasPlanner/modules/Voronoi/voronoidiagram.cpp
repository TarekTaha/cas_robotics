#include "voronoidiagram.h"

VoronoiDiagram::VoronoiDiagram()
{
}

VoronoiDiagram::~VoronoiDiagram()
{
}

Virtual_Voronoi_diagram_base_2 :: Object conflicts(const Site_2& s) const
{
	if ( Base::dual().dimension() < 2 ) 
	{
		return CGAL::make_object( (int)0 );
	}
	typedef std::vector<Delaunay_edge>          Edge_vector;
	typedef std::vector<Delaunay_face_handle>   Face_vector;
	typedef std::back_insert_iterator<Face_vector>   Face_output_iterator;
	typedef std::back_insert_iterator<Edge_vector>   Edge_output_iterator;
    Edge_vector evec;
	Face_vector fvec;
    Face_output_iterator fit(fvec);
	Edge_output_iterator eit(evec);
    Base::dual().get_conflicts_and_boundary(s, fit, eit);
    return CGAL::make_object( std::make_pair(fvec, evec) );
}

Virtual_Voronoi_diagram_base_2::Delaunay_edge opposite(const Delaunay_edge& e) const 
{
	int j = Base::dual().tds().mirror_index(e.first, e.second);
	Delaunay_face_handle n = e.first->neighbor(e.second);
	return Delaunay_edge(n, j);
}

template<class Query, class Iterator>
bool Virtual_Voronoi_diagram_base_2::find(const Query& q, Iterator first, Iterator beyond) const 
{
	for (Iterator it = first; it != beyond; ++it) 
	{
  		if ( q == *it ) { return true; }
	}
	return false;
}

Virtual_Voronoi_diagram_base_2::Object locate(const Point_2& q) const 
{
	if ( Base::number_of_faces() == 0 ) 
	{
		return CGAL::make_object(int(0));
	}
	typename Base::Adaptation_traits::Point_2 p(q.x(), q.y());
	Locate_result lr = Base::locate(p);
	return CGAL::make_object(lr);
}

Virtual_Voronoi_diagram_base_2::Object get_conflicts(const Point_2& q) const 
{
	return CGAL::make_object((int)0);
}

Virtual_Voronoi_diagram_base_2::Object get_conflicts(const Circle_2& c) const 
{
	return CGAL::make_object((int)0);
}

bool Virtual_Voronoi_diagram_base_2::is_valid() const 
{
	return Base::is_valid();
}

void Virtual_Voronoi_diagram_base_2::clear() 
{
	Base::clear();
}

//=========================================================================

//class VoronoiDiagram : public Virtual_Voronoi_diagram_base_2 <VD2,Voronoi_diagram_halfedge_2<VD2> >
//{
//	public:
//		VoronoiDiagram();
//		virtual ~VoronoiDiagram();
//	protected:
//		typedef Voronoi_diagram_halfedge_2<VD2>                   VD2_Halfedge;
//	  	typedef Virtual_Voronoi_diagram_base_2<VD2,VD2_Halfedge>  VBase;
//	
//	  	typedef VBase::Object   Object;
//	  	typedef VBase::Base     Base;
//	  	typedef VBase::Point_2  Point_2;
//	Base::Adaptation_traits::Site_2 to_site(const Point_2& p) const 
//	{
//		return Base::Adaptation_traits::Site_2(p.x(), p.y());
//	}
//	public:
//  		virtual void insert(const Point_2& p) 
//  		{
//	    	Base::insert( to_site(p) );
//	  	}
//	
//	  	virtual Object get_conflicts(const Point_2& q) const 
//	  	{
//	    	Base::Adaptation_traits::Point_2 p = to_site(q);
//	    	return conflicts( to_site(q) );
//	  	}
//	
//	  	virtual Object get_conflicts(const Circle_2& q) const 
//	  	{
//	    	return CGAL::make_object( (int)0 );
//	  	}
//	
//	  	virtual Object ptr() 
//	  	{ 
//	  		return CGAL::make_object(this); 
//	  	}	
//};