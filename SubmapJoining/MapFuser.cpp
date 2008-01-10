#include "MapFuser.h"

MapFuser::MapFuser(){}

void MapFuser::fuse_map(LocalMap m){
	m.P.print();
	Matrix temp = m.P.get_sub_matrix(1,1,m.P.rows, 3);
    m.P.delete_rows(1, 3);
    m.P.delete_columns(1, 3);
    m.P.add_columns(3);
    m.P.add_rows(3);
    cout << endl << endl;


    m.P.set(1,m.P.columns - 2,temp);
    m.P.set(m.P.rows - 2, 1, trn(temp));
    m.P.print();
}

