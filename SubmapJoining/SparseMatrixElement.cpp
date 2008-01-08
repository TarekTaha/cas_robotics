#include "SparseMatrixElement.h"

SparseMatrixElement::SparseMatrixElement(int row, int col, double value, SparseMatrixElement *next_in_row){
	this->row = row;
	this->col = col;
	this->value = value;
	this->next_in_row = next_in_row;
}
