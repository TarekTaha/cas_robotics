/* dx means that the sorting dependece on x arrays, if two values in ar1 is 
equal ar2 decides if x = 2
cx means that x arrays will be sorted in depending on the values in the other arrays
*/
#include <iostream.h>
#include <cmath>

using namespace std;

void quicksort_d1_c1(int* ar1, int* ar2, int top, int bottom);
void quicksort_d2_c1(int* ar1, int* ar2, double* ar3, int top, int bottom);
void sort_d2_c1(int* ar1, int* ar2, double* ar3, int size);
