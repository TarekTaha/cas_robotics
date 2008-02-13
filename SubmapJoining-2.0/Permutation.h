#ifndef PERMUTATION
#define PERMUTATION

#include <iostream.h>

using namespace std;

class Permutation{
	public:
		Permutation(int size = 0, int *p = 0);
		Permutation(const Permutation&);
		Permutation& operator=(Permutation& perm);
		~Permutation();
		
		void print() const;
		int *p;
		int size;
};

Permutation inv(const Permutation&);
Permutation operator+(const Permutation&, const Permutation&);
Permutation no_reorder(int size);

#endif
