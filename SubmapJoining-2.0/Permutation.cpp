#include "Permutation.h"

Permutation::Permutation(int size, int *p){
	int *p_new = new int[size];
	this->size = size;
	if(p != 0){
		for(int i = 0; i < size; ++i){
			p_new[i] = p[i];
		}
	}
	//p = p_new;
	this->p = p_new;
}


Permutation::Permutation(const Permutation& perm){
	int *p_new = new int[perm.size];
	size = perm.size;
	for(int i = 0; i < size; ++i){
		p_new[i] = perm.p[i];
	}
	p = p_new;
}

Permutation::~Permutation(){
	delete[] p;
}

Permutation& Permutation::operator=(Permutation& perm){
	if(&perm != this){
		int *p_new = new int[perm.size];
		size = perm.size;
		for(int i = 0; i < size; ++i){
			p_new[i] = perm.p[i];
		}
		p = p_new;
	}
	return *this;
}

Permutation inv(const Permutation& perm){
	Permutation result(perm.size);
	for(int i = 0; i < perm.size; ++i){
		result.p[ perm.p[i]] = i;
	}
	return result;
}

void Permutation::print() const{
	for(int i = 0; i < size; ++i){
		cout << p[i] << " ";
	}
	cout << endl;
}

Permutation operator+(const Permutation& p1, const Permutation& p2){
	Permutation result(p1.size);
	for(int i = 0; i < p1.size; ++i){
		result.p[i] = p1.p[p2.p[i]];
	}
	return result;
}

Permutation no_reorder(int size){
	Permutation result(size);
	for(int i = 0; i < size; ++i){
		result.p[i] = i;
	}
	return result;
}

Permutation sorting_permutation(int *array, int size){
	Permutation result = no_reorder(size);
	
}