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
	delete[] p;
	p = p_new;
}

Permutation::~Permutation(){
	delete[] p;
}

Permutation& Permutation::operator=(const Permutation& perm){
	if(&perm != this){
		int *p_new = new int[perm.size];
		size = perm.size;
		for(int i = 0; i < size; ++i){
			p_new[i] = perm.p[i];
		}
		delete[] p;
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

Permutation sorting_permutation(const Permutation& perm){
	Permutation result = no_reorder(perm.size);
	Permutation temp(perm);
	quicksort_di1_ci1(temp.p, result.p, 0, temp.size - 1);
	return result;
}

Permutation sorting_permutation(int *array, int size){
	Permutation result = no_reorder(size);
	Permutation temp(size, array);
	quicksort_di1_ci1(temp.p, result.p, 0, temp.size - 1);
	return result;
}

Permutation sorting_permutation(int *array, int lower_value, int upper_value){
	int size = upper_value - lower_value + 1;
	Permutation result = no_reorder(size);
	Permutation temp(size, array);
	dec_extremesort_di1_ci1(temp.p, result.p, lower_value, upper_value);
	return result;
}

Permutation sorting_permutation(double *array, int size){
	Permutation result = no_reorder(size);
	double *temp = new double[size];
	for(int i = 0; i < size; ++i){
		temp[i] = array[i];
	}
	quicksort_dd1_ci1(temp, result.p, 0, size - 1);
	delete[] temp;
	return result;
}

void permute(int *array, const Permutation& p){
	int *temp_array = new int[p.size];
	for(int i = 0; i < p.size; ++i){
		temp_array[i] = array[p.p[i]];
	}
	for(int i = 0; i < p.size; ++i){
		array[i] = temp_array[i];
	}
	delete[] temp_array;
}
