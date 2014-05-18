#include "MyLine3D.h"

	MyLine3D::MyLine3D()
	{

		_v = new MyPoint3D(1, 1, 1);
		_s = new MyPoint3D(0, 0, 0);
	}

	MyLine3D::MyLine3D(double x0, double y0, double z0, double a, double b, double c)
	{
		_v = new MyPoint3D(a, b, c);
		_s = new MyPoint3D(x0, y0, z0);
	}

	MyLine3D::MyLine3D(const MyLine3D &b)
	{
		_v = new MyPoint3D( *(b._v));
		_s = new MyPoint3D( *(b._s));
	}



	MyLine3D::~MyLine3D()
	{
		if(_v != NULL )
			delete _v;
		if(_s != NULL)
			delete _s;
	}

	// return true if value of 'a' is close to zero
	// return false otherwise
	bool MyLine3D::checkIfZero(double a, double epsilon)
	{
		if(epsilon < 0)
			epsilon = - epsilon;
		if(a < - epsilon || a > epsilon)
		{
			return false;
		}
		else
		{
			return true;
		}

	}

	// in parameters result_time_1 and result_time_2 will be time on corresponding line
	// result = 1 if there is only one point of intersection and > 1 if there is innumerous points of intersection
	int MyLine3D::intersection(MyLine3D & line_2, double &result_time_1, double &result_time_2)
	{
		MyPoint3D *delta_s = (this->_s) -> subtract(line_2._s);
		MyPoint3D *v1 = new MyPoint3D( *(this->_v));
		MyPoint3D *v2 = new MyPoint3D( *(line_2._v));
 		double scal_mult_v2_v1 = v2->scalarMultiplication( *v1 );
		double eps = 1e-10;
		double norm_v1_square = v1->normEuqlidianSquare();
		double scal_mult_delta_s_v1 = delta_s->scalarMultiplication( *v1 );
		int result_count_of_points = 1;
		//scal_mult_v2_v1 > eps || scal_mult_v2_v1 < -eps
		if(checkIfZero(scal_mult_v2_v1, eps) == false) // != 0
		{
			double a1, b1, c1;
			double a2, b2, c2;
			a1 = v1->getX();
			b1 = v1->getY();
			c1 = v1->getZ();

			a2 = v2->getX();
			b2 = v2->getY();
			c2 = v2->getZ();

			double a_x, a_y, a_z, c_x, c_y, c_z;

			a_x = a1 * scal_mult_v2_v1 - a2 * norm_v1_square;
			c_x = (-a2 * scal_mult_delta_s_v1) + delta_s->getX() * scal_mult_v2_v1;

			a_y = b1 * scal_mult_v2_v1 - b2 * norm_v1_square;
			c_y = (-b2 * scal_mult_delta_s_v1) + delta_s->getY() * scal_mult_v2_v1;

			a_z = c1 * scal_mult_v2_v1 - c2 * norm_v1_square;
			c_z = (-c2 * scal_mult_delta_s_v1) + delta_s->getZ() * scal_mult_v2_v1;

			MyPoint3D* a = new MyPoint3D(a_x, a_y, a_z);
			MyPoint3D* c = new MyPoint3D(c_x, c_y, c_z);

			double norm_a_square = a->normEuqlidianSquare();
			double scal_mult_a_c = a->scalarMultiplication( *c );

			double alpha = norm_a_square / ( scal_mult_v2_v1 * scal_mult_v2_v1);

			double betta = ( 2 * scal_mult_a_c ) / (scal_mult_v2_v1 * scal_mult_v2_v1);

			if(checkIfZero(alpha, eps)) // alpha == 0;
			{
				result_time_1 = 0;
				if(checkIfZero(betta,eps)) // betta == 0
				{
					result_count_of_points = 2;
					result_time_1 = 1;
				}
			}
			else // alpha != 0;
			{
				result_time_1 = - betta / (2 * alpha);
			}

			result_time_2 = (result_time_1 * norm_v1_square + scal_mult_delta_s_v1) / scal_mult_v2_v1;
		



			delete a;
			delete c;

			
		}
		else // scal_mult_v2_v1 == 0
		{
			result_time_1 = - scal_mult_delta_s_v1 / norm_v1_square;
			double scal_mult_delta_s_v2 = delta_s->scalarMultiplication( *v2 );
			double norm_v2_square = v2->normEuqlidianSquare();
			result_time_2 = scal_mult_delta_s_v2 /  norm_v2_square;
		}
		
		delete delta_s;
		delete v1;
		delete v2;

		return result_count_of_points;


	}

	MyPoint3D* MyLine3D::getPointByTime(double time)
	{
		MyPoint3D*  add_value = this->_v->multiplicationByNumber(time);
		MyPoint3D* result = this->_s->add(add_value);
		delete add_value;
		return result;
	}

