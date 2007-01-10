#ifndef VORONOIDIAGRAM_H_
#define VORONOIDIAGRAM_H_


#include <CGAL/basic.h>
#include <CGAL/iterator.h>
#include <CGAL/Apollonius_graph_constructions_C2.h>
#include <CGAL/Hyperbola_segment_2.h>
#include <CGAL/Hyperbola_ray_2.h>
#include <CGAL/Hyperbola_2.h>
#include <CGAL/Voronoi_diagram_2.h>

#include "typedefs.h"

#include <GL/glu.h>

template<class VDA>
class Voronoi_diagram_halfedge_2: public VDA::Halfedge
{
 	protected:
  		typedef VDA                                        Voronoi_diagram;
  		typedef typename Voronoi_diagram::Delaunay_graph   Delaunay_triangulation_2;
  		typedef typename Voronoi_diagram::Halfedge         Base;
  		typedef typename Base::Delaunay_edge               Delaunay_edge;
  		typedef typename Voronoi_diagram::Adaptation_traits::Site_2  Site_2;

	public:
  		Voronoi_diagram_halfedge_2() : Base() {}
  		Voronoi_diagram_halfedge_2(const Base& e) : Base(e), is_conflict(false) {}
  		Voronoi_diagram_halfedge_2(const Delaunay_edge& e, int inf, const Site_2& s)
    	: Base(), is_conflict(true), e_(e), inf_(inf), s_(s) {}
  		void draw() const//Qt_widget& qt_w) const
  		{
    		typedef typename Delaunay_triangulation_2::Geom_traits   Geom_traits;
    		typedef typename Geom_traits::Point_2                    Point_2;
    		typedef typename Geom_traits::Segment_2                  Segment_2;
    		typedef typename Geom_traits::Line_2                     Line_2;
    		if ( is_conflict ) 
    		{
      			if ( inf_ == 0 ) 
      			{
					typename Geom_traits::Construct_circumcenter_2 circumcenter;
					Point_2 c1 = circumcenter(e_.first->vertex(0)->point(),
				  	e_.first->vertex(1)->point(),
				  	e_.first->vertex(2)->point());
					int ccw_i = (e_.second + 1) % 3;
					int cw_i  = (e_.second + 2) % 3;
					Point_2 c2 = circumcenter(e_.first->vertex(ccw_i)->point(),
				  	e_.first->vertex(cw_i)->point(),
				  	s_);
					//qt_w << Segment_2(c1, c2);
      			} 
      			else 
      			{
					typename Geom_traits::Construct_circumcenter_2 circumcenter;
					typename Geom_traits::Construct_bisector_2     c_bis;
					typename Geom_traits::Construct_ray_2          c_ray;
					int ccw_i = (e_.second + 1) % 3;
					int cw_i  = (e_.second + 2) % 3;
					Point_2 c = circumcenter(e_.first->vertex(ccw_i)->point(),
				 	e_.first->vertex(cw_i)->point(),
				 	s_);
					Line_2 l = c_bis(e_.first->vertex(ccw_i)->point(),
			 		e_.first->vertex(cw_i)->point());
					//qt_w << c_ray(c, l);
      			}
      			return;
    		}
			if ( this->has_source() && this->has_target() ) 
			{
      			typename Geom_traits::Construct_circumcenter_2 circumcenter;
      			Point_2 c1 = circumcenter(this->down()->point(),
				this->up()->point(),
				this->left()->point());
				
      			Point_2 c2 = circumcenter(this->up()->point(),
				this->down()->point(),
				this->right()->point());
				
		      	//qt_w << Segment_2(c1, c2);
    		} 
    		else if ( this->has_source() && !this->has_target() ) 
    		{
      			typename Geom_traits::Construct_circumcenter_2 circumcenter;
      			typename Geom_traits::Construct_bisector_2     c_bis;
      			typename Geom_traits::Construct_ray_2          c_ray;
      			Point_2 c = circumcenter(this->down()->point(),
			    this->up()->point(),
			    this->left()->point());
			    
      			Line_2 l = c_bis(this->up()->point(), this->down()->point());
      			//qt_w << c_ray(c, l);
    		} 
    		else if ( !this->has_source() && this->has_target() ) 
    		{
      			typename Geom_traits::Construct_circumcenter_2 circumcenter;
      			typename Geom_traits::Construct_bisector_2     c_bis;
      			typename Geom_traits::Construct_ray_2          c_ray;
      			Point_2 c = circumcenter(this->up()->point(),
			    this->down()->point(),
			    this->right()->point());
			    
      			Line_2 l = c_bis(this->down()->point(), this->up()->point());
      			//qt_w << c_ray(c, l);
    		} 
    		else 
    		{
      			CGAL_assertion( !this->has_source() && !this->has_target() );
      			typename Geom_traits::Construct_bisector_2 c_bis;
      			//qt_w << c_bis(this->up()->point(),
		    	//this->down()->point());
    		}	
 		}
	private:
  		bool is_conflict;
  		Delaunay_edge e_;
  		int inf_;
  		Site_2 s_;
};

struct Virtual_Voronoi_diagram_2
{
	typedef CGAL::Object     Object;
  	typedef ::Rep GlobalRep;
  	typedef GlobalRep ::Point_2   Point_2;
  	typedef GlobalRep::Circle_2  Circle_2;
  
  	// virtual destructor
  	virtual ~Virtual_Voronoi_diagram_2() {}

  	// insert a site
  	virtual void insert(const Point_2&) = 0;
  	virtual void insert(const Circle_2&) = 0;

  	// remove a site
  	virtual void remove(const Object&) = 0;
	virtual Object locate(const Point_2&) const = 0;
  	virtual Object get_conflicts(const Point_2&) const = 0;
  	virtual Object get_conflicts(const Circle_2&) const = 0;
  	virtual Object ptr() = 0;
  	virtual bool is_valid() const = 0;
  	virtual void clear() = 0;
};

typedef Virtual_Voronoi_diagram_2   VVD2;

//=========================================================================

template<class VD, class Halfedge_with_draw_t>
class Virtual_Voronoi_diagram_base_2 : public VD, public Virtual_Voronoi_diagram_2
{
 	protected:
  		typedef Virtual_Voronoi_diagram_2    VBase;
  		typedef VD                           Base;
  		typedef typename VBase::Object       Object;
	  	typedef typename VBase::Point_2      Point_2;
	  	typedef typename VBase::Circle_2     Circle_2;
  		typedef typename Base::Halfedge                 Halfedge;
  		typedef typename Base::Halfedge_handle          Halfedge_handle;
  		typedef typename Base::Face_handle              Face_handle;
  		typedef typename Base::Ccb_halfedge_circulator  Ccb_halfedge_circulator;
  		typedef typename Base::Edge_iterator            Edge_iterator;
  		typedef typename Base::Site_iterator            Site_iterator;
  		typedef typename Base::Locate_result            Locate_result;
  		typedef Halfedge_with_draw_t                    Halfedge_with_draw;
  		typedef typename Base::Delaunay_graph           Delaunay_graph;
  		typedef typename Delaunay_graph::Edge           Delaunay_edge;
  		typedef typename Delaunay_graph::Vertex_handle  Delaunay_vertex_handle;
  		typedef typename Delaunay_graph::Face_handle    Delaunay_face_handle;
  		typedef typename Base::Adaptation_traits::Site_2   Site_2;
  		//typedef Triangulation_cw_ccw_2                  CW_CCW_2;
	  	Virtual_Voronoi_diagram_base_2() {}
  		virtual ~Virtual_Voronoi_diagram_base_2() {}
  		virtual void insert(const Point_2&) {}
  		virtual void insert(const Circle_2&) {}
		virtual void remove(const Object& o){};

  		virtual  Object conflicts(const Site_2& s) const
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
  		Delaunay_edge opposite(const Delaunay_edge& e) const
		{
			int j = Base::dual().tds().mirror_index(e.first, e.second);
			Delaunay_face_handle n = e.first->neighbor(e.second);
			return Delaunay_edge(n, j);
		}  		
		template<class Query, class Iterator>
  		bool find(const Query& q, Iterator first, Iterator beyond) const
		{
			for (Iterator it = first; it != beyond; ++it) 
			{
		  		if ( q == *it ) { return true; }
			}
			return false;
		}  		
		virtual void draw_conflicts(const Site_2& s, const Object& o) const 
		{
    		typedef std::vector<Delaunay_edge>           Edge_vector;
    		typedef std::vector<Delaunay_face_handle>    Face_vector;
    		typedef std::pair<Face_vector,Edge_vector>   result_type;
    		result_type res;
    		if ( !CGAL::assign(res, o) ) 
    		{ 
    			return; 
    		}
		    Face_vector fvec = res.first;
    		Edge_vector evec = res.second;
		
//		    widget << CGAL::YELLOW;
//    		unsigned int linewidth = widget.lineWidth();
//    		widget << CGAL::LineWidth(4);
			glPushMatrix();
			glLineWidth(2);
			glColor4f(1,0,0,1);	
			glBegin(GL_POINT);	
			
		    bool do_regular_draw = true;
    		if ( evec.size() == 2 ) 
    		{
      			Delaunay_edge e1 = evec[0];
      			Delaunay_edge e2 = evec[1];
      			if ( e1 == opposite(e2) ) 
      			{
					do_regular_draw = false;
					if ( !Base::dual().is_infinite(e1) ) 
					{
	  					Halfedge_with_draw ee(e1, 2, s);
//	  					widget << ee;
					}
      			}
    		}
			if ( do_regular_draw ) 
			{
				for (unsigned int i = 0; i < evec.size(); i++) 
				{
					if ( Base::dual().is_infinite(evec[i]) ) { continue; }
					Delaunay_edge opp = opposite(evec[i]);
					Halfedge_with_draw ee(opp, Base::dual().is_infinite(opp.first), s);
//					widget << ee;
      			}
    		}
    		typename Base::Adaptation_policy::Edge_rejector e_rejector = Base::adaptation_policy().edge_rejector_object();
    		for (unsigned int i = 0; i < fvec.size(); i++) 
    		{
      			for (int j = 0; j < 3; j++) 
      			{
					Delaunay_edge e(fvec[i], j);
					Delaunay_edge opp = opposite(e);
					if ( Base::dual().is_infinite(e) ) { continue; }
					if ( !find(e, evec.begin(), evec.end()) && !find(opp, evec.begin(), evec.end()) ) 
					{
	  					if ( !e_rejector(Base::dual(),e) ) 
	  					{
	    					Halfedge_with_draw ee(*Base::dual(e));
//	    					widget << ee;
	  					}
					}
      			}
    		}
		}
	public:
		void draw_edge(const Halfedge& e) const 
		{
			Halfedge_with_draw ee(e);
//		    widget << ee;
		}
		virtual void draw_feature(const Object& o) const 
		{
		    Locate_result lr;
		    //    if ( !CGAL::assign(lr, o) ) { return; }
		#if 1
		    const Locate_result* lrp0 = CGAL::object_cast<Locate_result>(&o);
			Locate_result* lrp = const_cast<Locate_result*>(lrp0);
		    if ( lrp == NULL ) 
		    {
		    	return; 
		    }
		#else
		    try 
		    {
		    	lr = CGAL::object_cast<Locate_result>(o);
		    } 
		    catch ( CGAL::Bad_object_cast ) 
		    {
		    	return;
		    }
		#endif
		    if ( Face_handle* f = boost::get<Face_handle>(lrp) ) 
		    {
		    	Ccb_halfedge_circulator ccb_start = (*f)->outer_ccb();
		      	Ccb_halfedge_circulator ccb = ccb_start;
		      	do 
		      	{
					draw_edge(*ccb);
					++ccb;
		      	} 
		      	while ( ccb != ccb_start );
		    }
		}
		
		virtual void draw_sites() const
		{
			printf("Here");fflush(stdout);			
			glPushMatrix();
			glLineWidth(2);
			glColor4f(1,0,0,1);	
			glBegin(GL_POINT);		
			for (Site_iterator sit = this->sites_begin();sit != this->sites_end(); ++sit) 
			{
//			  	float x = sit.x();
//			  	float y = sit.y();
//				glVertex2f(x,y);
		    }
			glEnd();
			glPopMatrix();	    	  
		};
		
		virtual void draw_diagram() const
		{
			Edge_iterator it;
		    for (it = this->edges_begin(); it != this->edges_end(); ++it) 
		    {
		    	draw_edge(*it);
		    }
		}
		virtual void draw_conflicts(const Point_2& p,const Object& o) const {}
		virtual void draw_conflicts(const Circle_2& c, const Object& o) const {}
		virtual Object locate(const Point_2& q) const
		{
			if ( Base::number_of_faces() == 0 ) 
			{
				return CGAL::make_object(int(0));
			}
			typename Base::Adaptation_traits::Point_2 p(q.x(), q.y());
			Locate_result lr = Base::locate(p);
			return CGAL::make_object(lr);
		}		
		virtual Object get_conflicts(const Point_2& q) const
		{
			return CGAL::make_object((int)0);
		}
		virtual Object get_conflicts(const Circle_2& c) const
		{
			return CGAL::make_object((int)0);
		}		
		virtual Object ptr() = 0;
		virtual bool is_valid() const
		{
			return Base::is_valid();
		}		
		virtual void clear()
		{
			Base::clear();
		}		
};

//=========================================================================

class VoronoiDiagram : public Virtual_Voronoi_diagram_base_2 <VD2,Voronoi_diagram_halfedge_2<VD2> >
{
	public:
		VoronoiDiagram();
		virtual ~VoronoiDiagram();
	protected:
		typedef Voronoi_diagram_halfedge_2<VD2>                   VD2_Halfedge;
	  	typedef Virtual_Voronoi_diagram_base_2<VD2,VD2_Halfedge>  VBase;
	
	  	typedef VBase::Object   Object;
	  	typedef VBase::Base     Base;
	  	typedef VBase::Point_2  Point_2;
	Base::Adaptation_traits::Site_2 to_site(const Point_2& p) const 
	{
		return Base::Adaptation_traits::Site_2(p.x(), p.y());
	}
	public:
  		virtual void insert(const Point_2& p) 
  		{
	    	Base::insert( to_site(p) );
	  	}
	
	  	virtual Object get_conflicts(const Point_2& q) const 
	  	{
	    	Base::Adaptation_traits::Point_2 p = to_site(q);
	    	return conflicts( to_site(q) );
	  	}
	
	  	virtual Object get_conflicts(const Circle_2& q) const 
	  	{
	    	return CGAL::make_object( (int)0 );
	  	}
	
	  	virtual Object ptr() 
	  	{ 
	  		return CGAL::make_object(this); 
	  	}	
};

#endif /*VORONOIDIAGRAM_H_*/
