#include "MyArray.h"




double Array::get(int row, int column)
{
	int a = 10, b = 0;
	if(row < 0 || row >= rows || column < 0 || column >= columns)
		return a / b;

	return mas[row*columns + column];
}


double Array::set(int row, int column, double value)
{
	int a = 10, b = 0;
	if(row < 0 || row >= rows || column < 0 || column >= columns)
		return a / b;
	mas[row*columns + column] = value;
	return value;
}

void Array::init(int cRows, int cColumns)
{
	mas = new double [cRows * cColumns];
	rows = cRows;
	columns = cColumns;
	toZeros();
}

void Array::dispose()
{
	if(mas != 0)
	{
		delete[] mas;
		mas = 0;
	}
	rows = 0;
	columns = 0;
}

void Array::toZeros()
{
	for (int i = 0; i < rows; ++i)
	{
		for(int j = 0 ; j < columns; ++j)
		{
			mas[i*columns + j] = 0;
		}
	}
}

Array* Array::multiply(Array* A, Array* B)
{
	int a = 10, b = 0;
	if( A->columns != B->rows)
	{
			a = a/b;
	}
	Array* result = new Array();
	result->init( A->rows, B->columns);
	for(int iR = 0; iR < result->rows; ++iR)
	{
		for(int iC = 0; iC < result->columns; ++iC)
		{
			double sum = 0;
			for(int k = 0; k < A->columns; ++k)
			{
				sum+= A->get(iR, k) * B->get(k, iC);
			}
			result->set(iR, iC, sum);
		}

	}
	return result;
}

void Array::print()
{
	cout << endl;
	for( int i = 0; i < rows; ++i)
	{
		for(int j = 0; j < columns;++j)
		{
			cout<< get(i, j) << " ";
		}
		cout << endl;
	}
}

