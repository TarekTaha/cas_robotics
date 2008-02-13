/* dd1 means depends on one double array
 * di1 means depends on one integer array
 * if a sorting dependes on more than one array the second array is used if the first has two equal elements
 * cd1 means changes one double array but the sorting does not depend on it
*/
#include <iostream.h>
#include <cmath>

using namespace std;

void inc_quicksort_dd1_ci1(double* ar1, int* ar2, int top, int bottom);

void quicksort_di1_ci1(int* ar1, int* ar2, int top, int bottom);
void quicksort_dd1_ci1(double* ar1, int* ar2, int top, int bottom);
void quicksort_di2_cd1(int* ar1, int* ar2, double* ar3, int top, int bottom);
