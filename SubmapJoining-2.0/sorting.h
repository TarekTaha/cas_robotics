/* dd1 means depends on one double array
 * di1 means depends on one integer array
 * if a sorting dependes on more than one array the second array is used if the first has two equal elements
 * cd1 means changes one double array but the sorting does not depend on it
 * inc_ means sorting in increasing order
 * dec_ means sorting in decreasing order
*/

#ifndef SORTING
#define SORTING

#include <iostream.h>
#include <cmath>

using namespace std;

//extreme sort requires that the array to be sorted only includes all numbers between lower_value and upper_value and only one of each, the algorithm is linear.
void inc_extremesort_di1_ci1(int* ar1, int* ar2, int lower_value, int upper_value);

void dec_quicksort_dd1_ci1(double* ar1, int* ar2, int top, int bottom);
void inc_quicksort_di1_ci1(int* ar1, int* ar2, int top, int bottom);
void inc_quicksort_dd1_ci1(double* ar1, int* ar2, int top, int bottom);
void inc_quicksort_di2_cd1(int* ar1, int* ar2, double* ar3, int top, int bottom);

#endif
