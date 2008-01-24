#ifndef LOCALMAPLIST
#define LOCALMAPLIST

#include "LocalMap.h"
#include <list>
#include <iostream.h>

using namespace std;

class LocalMapList{
  public:
    LocalMapList();
    list<LocalMap> lst;

    void add_local_map_first(LocalMap c);
    void add_local_map_last(LocalMap c);
    list<LocalMap>::iterator erase_element(list<LocalMap>::iterator it);
    void remove_first_local_map();
    void remove_all_local_maps();
    bool empty();
	
    list<LocalMap>::iterator begin();
    list<LocalMap>::iterator end();


    LocalMap& get_first_local_map();

    void print();
};

#endif
