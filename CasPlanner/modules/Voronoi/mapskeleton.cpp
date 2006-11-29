#include "mapskeleton.h"

MapSkeleton::MapSkeleton()
{
}

MapSkeleton::~MapSkeleton()
{
}

void MapSkeleton::clear()
{
    sskel = defs::SSkelPtr() ;
    input.clear();
    offsets.clear();
    output.clear();
}

void MapSkeleton::loadMap()
{
	QString s("alley_0.poly");//( QFileDialog::getOpenFileName(QString::null, "Polygonal PolygonalRegion Files (*.poly)", this ) );
    if ( s.isEmpty() )
      return;

    bool auto_create_offsets = true ;
    offsets.clear() ;

//    std::ifstream offsets_file(s + QString(".oft") );
	std::ifstream offsets_file("alley_0.poly.oft");
    if ( offsets_file )
    {
      CGAL::set_ascii_mode(offsets_file);

      while ( offsets_file )
      {
        double v ;
        offsets_file >> v;
        offsets.push_back(v);
      }
      auto_create_offsets = false ;
    }

    std::ifstream in("alley_0.poly");
    if ( in )
    {
      CGAL::set_ascii_mode(in);

      input.clear();

      defs::RegionPtr lRegion( new Region() ) ;

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
          if ( i == 0 )
          {
            CGAL::Bbox_2 lBbox = CGAL::bbox_2(lPoly->begin(),lPoly->end());
            double w = lBbox.xmax() - lBbox.xmin();
            double h = lBbox.ymax() - lBbox.ymin();
            double s = std::sqrt(w*w+h*h);
            double m = s * 0.01 ;
//            widget->set_window(lBbox.xmin()-m, lBbox.xmax()+m, lBbox.ymin()-m, lBbox.ymax()+m);
            if ( auto_create_offsets )
            {
              for ( int c = 1 ; c < 30 ; ++ c )
                offsets.push_back(c*m);
            }
          }
          CGAL::Orientation expected = ( i == 0 ? CGAL::COUNTERCLOCKWISE : CGAL::CLOCKWISE ) ;
          if (  !CGAL::is_simple_2(lPoly->begin(),lPoly->end())
             || CGAL::orientation_2(lPoly->begin(),lPoly->end()) == expected 
             )
               lRegion->push_back(lPoly);
          else lRegion->push_back( PolygonPtr( new Polygon(lPoly->rbegin(),lPoly->rend()) ) ) ;
        }
      }

      input.push_back(lRegion);
    }

    sskel = SSkelPtr() ;
    
    output.clear();
//    widget->redraw();
    something_changed();
}

SSkelPtr MapSkeleton::getSSkelPtr()
{
	return this->sskel;
}

void MapSkeleton::generateInnerSkeleton()
{
    if ( input.size() > 0 )
    {
      Region const& lRegion = *input.front();

      SSkelBuilder builder ;
      for( Region::const_iterator bit = lRegion.begin(), ebit = lRegion.end() ; bit != ebit ; ++ bit )
      {
        builder.enter_contour((*bit)->begin(),(*bit)->end());
      }
      sskel = builder.construct_skeleton() ;
      sskel_valid = sskel ;
      if ( !sskel_valid )
      	std::cout<<"\n Something is WRONG";
//        QMessageBox::critical( this, my_title_string,"Straight Skeleton construction failed." );
//      widget->redraw();
      something_changed();
    }	
}
