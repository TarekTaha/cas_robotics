#ifndef PERMUTATION
#define PERMUTATION

#include <iostream.h>
#include "sorting.h"

using namespace std;

class Permutation{
	public:
		Permutation(int size = 0, int *p = 0);
		Permutation(const Permutation&);
		Permutation& operator=(const Permutation& perm);
		~Permutation();
		
		void print() const;
		int *p;
		int size;
};

Permutation inv(const Permutation&);
Permutation operator+(const Permutation&, const Permutation&);
Permutation no_reorder(int size);
Permutation sorting_permutation(const Permutation& perm);
Permutation sorting_permutation(int *array, int size);
Permutation sorting_permutation(double *array, int size);

#endif
