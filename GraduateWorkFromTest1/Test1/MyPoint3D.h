#ifndef MY_POINT_3_D
#define MY_POINT_3_D
#include <math.h>

class MyPoint3D
{
private:
	double _x, _y, _z;
public:
	MyPoint3D();
	MyPoint3D(double x, double y, double z);
	MyPoint3D(const MyPoint3D & b);
	
	

	~MyPoint3D();

	double scalarMultiplication(MyPoint3D &b);
	double normEuqlidianSquare();

	double normEuqlidian();

	// result = this - b;
	MyPoint3D* subtract( MyPoint3D * b);
	MyPoint3D* add(MyPoint3D* b);
	MyPoint3D* multiplicationByNumber(double alpha);

	double getX();
	double getY();
	double getZ();
	// static functions
	//result  a - b
	static MyPoint3D* subtract(MyPoint3D *a, MyPoint3D* b);

	
};

#endif