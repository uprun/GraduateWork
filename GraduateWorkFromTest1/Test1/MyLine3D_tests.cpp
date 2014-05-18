#include "MyLine3D.h"
#include <iostream>

using namespace std;

void myAssert(int test_number, int  func_name())
{
	int res_test_1 = func_name();
	if(res_test_1 == 0)
	{
		cout<< "Test " << test_number << " passed OK!" << endl;
	}
	else
	{
		cout << "Test " << test_number << " failed." << endl;
	}
}


int run_test1()
{
	MyLine3D line_1(0, 0, 0, 1, 1, 0);
	MyLine3D line_2(3, 0, 0, 0, 1, 0);

	double time_1 = 0, time_2 = 0;

	int result = line_1.intersection(line_2,time_1, time_2);

	cout<< "Count of points = "  << result << endl;
	cout<< "Time_1 = " << time_1 << " time_2 = " << time_2 << endl;

	return 0;
}

int run_test2()
{
	double eps = 1e-10;
	MyLine3D line_1(0, 0, 0, 1, 1, 0);
	MyLine3D line_2(4, 0, 0, -1, 1, 0);

	double time_1 = 0, time_2 = 0;

	int result = line_1.intersection(line_2,time_1, time_2);

	cout<< "Count of points = "  << result << endl;
	cout<< "Time_1 = " << time_1 << " time_2 = " << time_2 << endl;

	bool failure = false;

	if(MyLine3D::checkIfZero(time_1 - 2, eps) == false)
	{
		cout<<"time_1 == " << time_1 << " and supposed to be : "  << 2 << endl;
		failure = true;
	}
	if(MyLine3D::checkIfZero(time_2 - 2, eps) == false)
	{
		cout<<"time_2 == " << time_2 << " and supposed to be : "  << 2 << endl;
		failure = true;
	}

	if(failure)
		return 1;

	return 0;
}

int run_test3()
{
	double eps = 1e-10;
	MyLine3D line_1(0, 0, 0, 1, 1, 0);
	MyLine3D line_2(0, 0, 0, 1, 1, 0);

	double time_1 = 0, time_2 = 0;

	int result = line_1.intersection(line_2,time_1, time_2);

	cout<< "Count of points = "  << result << endl;
	cout<< "Time_1 = " << time_1 << " time_2 = " << time_2 << endl;

	bool failure = false;

	if(result == 1)
	{
		cout<< "Result = " << result << " and supposed to be : " << 2 << endl;
		failure = true;
	}

	

	if(MyLine3D::checkIfZero(time_1 - 0, eps) == false)
	{
		cout<<"time_1 == " << time_1 << " and supposed to be : "  << 0 << endl;
		failure = true;
	}
	if(MyLine3D::checkIfZero(time_2 - 0, eps) == false)
	{
		cout<<"time_2 == " << time_2 << " and supposed to be : "  << 0 << endl;
		failure = true;
	}

	if(failure)
		return 1;

	return 0;
}

int run_test4()
{
	double eps = 1e-10;
	MyLine3D line_1(0, 0, 0, 1, 1, 0);
	MyLine3D line_2(1, 0, 0, 1, 1, 0);

	double time_1 = 0, time_2 = 0;

	int result = line_1.intersection(line_2,time_1, time_2);

	cout<< "Count of points = "  << result << endl;
	cout<< "Time_1 = " << time_1 << " time_2 = " << time_2 << endl;

	bool failure = false;

	if(result == 1)
	{
		cout<< "Result = " << result << " and supposed to be : " << 2 << endl;
		failure = true;
	}

	

	if(MyLine3D::checkIfZero(time_1 - 0, eps) == false)
	{
		cout<<"time_1 == " << time_1 << " and supposed to be : "  << 0 << endl;
		failure = true;
	}
	if(MyLine3D::checkIfZero(time_2 - (-0.5), eps) == false)
	{
		cout<<"time_2 == " << time_2 << " and supposed to be : "  << -0.5 << endl;
		failure = true;
	}

	if(failure)
		return 1;

	return 0;
}


int run_test5()
{
	double eps = 1e-10;
	MyLine3D line_1(0, 0, 0, 1, 1, 0);
	MyLine3D line_2(1, -1, 0, 1, -1, 0);

	double check_time_1, check_time_2, check_count_points ;
	check_time_1 = 0;
	check_time_2 = -1;
	check_count_points = 1;


	double time_1 = 0, time_2 = 0;



	int result = line_1.intersection(line_2,time_1, time_2);

	cout<< "Count of points = "  << result << endl;
	cout<< "Time_1 = " << time_1 << " time_2 = " << time_2 << endl;

	bool failure = false;

	if(result != check_count_points)
	{
		cout<< "Result count points = " << result << " and supposed to be : " << check_count_points << endl;
		failure = true;
	}

	

	if(MyLine3D::checkIfZero(time_1 - check_time_1, eps) == false)
	{
		cout<<"time_1 == " << time_1 << " and supposed to be : "  << check_time_1 << endl;
		failure = true;
	}
	if(MyLine3D::checkIfZero(time_2 - check_time_2, eps) == false)
	{
		cout<<"time_2 == " << time_2 << " and supposed to be : "  << check_time_2 << endl;
		failure = true;
	}

	if(failure)
		return 1;

	return 0;
}

int run_test6()
{
	double eps = 1e-10;
	MyLine3D line_1(0, 0, 0, 1, 1, 0);
	MyLine3D line_2(4, 0, 1, 0, 1, 0);

	double check_time_1, check_time_2, check_count_points ;
	check_time_1 = 4;
	check_time_2 = 4;
	check_count_points = 1;


	double time_1 = 0, time_2 = 0;



	int result = line_1.intersection(line_2,time_1, time_2);

	cout<< "Count of points = "  << result << endl;
	cout<< "Time_1 = " << time_1 << " time_2 = " << time_2 << endl;

	bool failure = false;

	if(result != check_count_points)
	{
		cout<< "Result count points = " << result << " and supposed to be : " << check_count_points << endl;
		failure = true;
	}

	

	if(MyLine3D::checkIfZero(time_1 - check_time_1, eps) == false)
	{
		cout<<"time_1 == " << time_1 << " and supposed to be : "  << check_time_1 << endl;
		failure = true;
	}
	if(MyLine3D::checkIfZero(time_2 - check_time_2, eps) == false)
	{
		cout<<"time_2 == " << time_2 << " and supposed to be : "  << check_time_2 << endl;
		failure = true;
	}

	if(failure)
		return 1;

	return 0;
}

int run_test7()
{
	double eps = 1e-10;
	MyLine3D line_1(2, 0, 0, 0, 1, 0);
	MyLine3D line_2(5, 0, 0, -1, 1, 0);

	double check_time_1, check_time_2, check_count_points ;
	check_time_1 = 3;
	check_time_2 = 3;
	check_count_points = 1;


	double time_1 = 0, time_2 = 0;



	int result = line_1.intersection(line_2,time_1, time_2);

	cout<< "Count of points = "  << result << endl;
	cout<< "Time_1 = " << time_1 << " time_2 = " << time_2 << endl;

	bool failure = false;

	if(result != check_count_points)
	{
		cout<< "Result count points = " << result << " and supposed to be : " << check_count_points << endl;
		failure = true;
	}

	

	if(MyLine3D::checkIfZero(time_1 - check_time_1, eps) == false)
	{
		cout<<"time_1 == " << time_1 << " and supposed to be : "  << check_time_1 << endl;
		failure = true;
	}
	if(MyLine3D::checkIfZero(time_2 - check_time_2, eps) == false)
	{
		cout<<"time_2 == " << time_2 << " and supposed to be : "  << check_time_2 << endl;
		failure = true;
	}

	if(failure)
		return 1;

	return 0;
}


void MyLine3D::runTest()
{
	myAssert(1, run_test1);
	myAssert(2, run_test2);
	myAssert(3, run_test3);
	myAssert(4, run_test4);
	myAssert(5, run_test5);
	myAssert(6, run_test6);
	myAssert(7, run_test7);
}

