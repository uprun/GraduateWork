#include "MyPoint3D.h"


	MyPoint3D::MyPoint3D()
	{
		_x = 0;
		_y = 0;
		_z = 0;
	}
	MyPoint3D::MyPoint3D(double x, double y, double z)
	{
		_x = x;
		_y = y;
		_z = z;
	}
	MyPoint3D::MyPoint3D(const MyPoint3D & b)
	{
		_x = b._x;
		_y = b._y;
		_z = b._z;
	}
	
	

	MyPoint3D::~MyPoint3D()
	{
	}

	double MyPoint3D::scalarMultiplication(MyPoint3D &b)
	{
		double result = 0;
		result = _x * b._x + _y * b._y + _z * b._z;
		return result;
	}
	double MyPoint3D::normEuqlidianSquare()
	{
		double result = 0;
		result = _x * _x + _y * _y + _z * _z;
		return result;
	}

	double MyPoint3D::normEuqlidian()
	{
		double result = normEuqlidianSquare();
		result = sqrt(result);
		return result;
	}

	// result = this - b;
	MyPoint3D* MyPoint3D::subtract( MyPoint3D * b)
	{
		MyPoint3D* result = new MyPoint3D( *this );
		result->_x -= b->_x;
		result->_y -= b->_y;
		result->_z -= b->_z;
		return result;
	}

	double MyPoint3D::getX()
	{
		return _x;
	}
	double MyPoint3D::getY()
	{
		return _y;
	}
	double MyPoint3D::getZ()
	{
		return _z;
	}
	// static functions
	//result  a - b
	MyPoint3D* MyPoint3D::subtract(MyPoint3D *a, MyPoint3D* b)
	{
		return a->subtract(b);
	}
	

	MyPoint3D* MyPoint3D::add(MyPoint3D* b)
	{
		MyPoint3D *result = new MyPoint3D( *this );
		result->_x += b->_x;
		result->_y += b->_y;
		result->_z += b->_z;

		return result;
	}
	MyPoint3D* MyPoint3D::multiplicationByNumber(double alpha)
	{
		MyPoint3D *result = new MyPoint3D( *this );
		result->_x *= alpha;
		result->_y *= alpha;
		result->_z *= alpha;

		return result;
	}