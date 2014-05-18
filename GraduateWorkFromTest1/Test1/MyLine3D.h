#ifndef MY_LINE_3_D
#define MY_LINE_3_D

#define NULL 0

#include <math.h>
#include "MyPoint3D.h"



class MyLine3D
{
private:
	//double _a, _b, _c;
	MyPoint3D *_v; // vector of line
	//double _x0, _y0, _z0;
	MyPoint3D *_s; // start point
	
public:
	MyLine3D();

	MyLine3D(double x0, double y0, double z0, double a, double b, double c);

	MyLine3D(const MyLine3D &b);



	~MyLine3D();

	// in parameters result_time_1 and result_time_2 will be time on corresponding line
	int intersection(MyLine3D & line_2, double &result_time_1, double &result_time_2);

	MyPoint3D* getPointByTime(double time);

	static void runTest();
	static bool checkIfZero(double a, double eps);
};



#endif