#ifndef MYARRAY_H
#define MYARRAY_H
#include <iostream>
using namespace std;
typedef struct Array
{
	double *mas;
	int rows;
	int columns;
	double get(int row, int column);
	double set(int row, int column, double value);
	void init(int cRows, int cColumns);
	void dispose();
	void toZeros();
	Array* multiply(Array* A, Array* B);
	void print();

} ;


#endif