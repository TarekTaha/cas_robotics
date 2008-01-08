#include "LocalMapList.h"

LocalMapList::LocalMapList(){}

list<LocalMap>::iterator LocalMapList::erase_element(list<LocalMap>::iterator it)
{
	return lst.erase(it);
}



void LocalMapList::add_local_map_first(LocalMap c){
  lst.push_front(c);
}

void LocalMapList::add_local_map_last(LocalMap c){
  lst.push_back(c);
}

void LocalMapList::remove_first_local_map(){
  lst.pop_front();
}

void LocalMapList::remove_all_local_maps(){
    lst.clear();
}

bool LocalMapList::empty(){
  return lst.empty();
}

LocalMap& LocalMapList::get_first_local_map(){
  return lst.front();
}

list<LocalMap>::iterator LocalMapList::begin(){
  return lst.begin();
}

list<LocalMap>::iterator LocalMapList::end(){
  return lst.end();
}

void LocalMapList::print(){
   list<LocalMap>::iterator it = lst.begin();
   while( it != lst.end()){
     cout <<"Local Map" << endl;
     cout << "trn(X)" << endl;
     trn(it->X).print();
     cout<< "P" << endl;
     it->P.print();
     it++;
   }
   cout << endl;
}

