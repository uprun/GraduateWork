#include "SupportCode.h"

double MyMax(double a, double b)
{
	if( a > b)
	{
		return a;
	}
	return b;
}

// y1 - x coordinate on screen
// y2 - y coordinate on screen
// k - inner calibration matrix
// r - rotation matrix
// t - transpose vector
MyLine3D* getLineWithTransformations(float y1, float y2, Array* k, Array* r, Array* t)
{
	float alpha_x, alpha_y, c_x, c_y;
	alpha_x = k->mas[0 * 4 + 0];
	alpha_y = k->mas[1 * 4 + 1];
	c_x = k->mas[0 * 4 + 2];
	c_y = k->mas[1 * 4 + 2];

	float x1, x2, x3;
	x3 = MyMax(alpha_x, alpha_y);
	x1 = ((y1 - c_x) * x3) / alpha_x;
	x2 = ((y2 - c_y) * x3) / alpha_y;
	
	float norm = x1*x1 + x2*x2 + x3*x3;
	norm = sqrtf(norm);
	x1 /= norm;
	x2 /= norm;
	x3 /= norm;


	Array* v =  new Array();
	v->init(4,1);
	Array* s = new Array();
	s->init(4,1);

	v->mas[0] = x1;
	v->mas[1] = x2;
	v->mas[2] = x3;

	
	s->mas[0] = 0;
	s->mas[1] = 0;
	s->mas[2] = 0;

	Array* v_1 = new Array();
	v_1->init(4,1);
	Array* s_1 = new Array();
	s_1->init(4,1);

	double sh_x, sh_y, sh_z;
	sh_x = t->get(0, 3);
	sh_y = t->get(1, 3);
	sh_z = t->get(2, 3);

	// translate two points
	//cvSub(v, t, v_1); // changed by Alexander Kryvonos 13.04.2014 at 15:22
	v_1->mas[0] = v->mas[0] + sh_x;
	v_1->mas[1] = v->mas[1] + sh_y;
	v_1->mas[2] = v->mas[2] + sh_z;
	//cvSub(s, t, s_1);
	s_1->mas[0] = s->mas[0] + sh_x;
	s_1->mas[1] = s->mas[1] + sh_y;
	s_1->mas[2] = s->mas[2] + sh_z;

	v->dispose();
	delete v;
	s->dispose();
	delete s;
	// rotate them
	//cvMatMul(r, v_1, v);
	v = r->multiply(r, v_1);

	//cvMatMul(r, s_1, s);
	s = r->multiply(r, s_1);

	// calculate new vector from s to v
	//cvSub(v, s, v_1);
	//cvCopy(v_1, v);
	v->mas[0] -= s->mas[0];
	v->mas[1] -= s->mas[1];
	v->mas[2] -= s->mas[2];

	MyLine3D* value = new MyLine3D(
		s->mas[0],
		s->mas[1],
		s->mas[2],
		v->mas[0],
		v->mas[1],
		v->mas[2]);


	/*cvReleaseMat(&v);
	cvReleaseMat(&v_1);
	cvReleaseMat(&s);
	cvReleaseMat(&s_1);*/

	s_1->dispose();
	delete s_1;
	v_1->dispose();
	delete v_1;
	s->dispose();
	delete s;
	v->dispose();
	delete v;


	return value;
}

MyLine3D* getJustLine(float y1, float y2, Array* k)
{
	float alpha_x, alpha_y, c_x, c_y;
	alpha_x = k->mas[0 * 4 + 0];
	alpha_y = k->mas[1 * 4 + 1];
	c_x = k->mas[0 * 4 + 2];
	c_y = k->mas[1 * 4 + 2];

	float x1, x2, x3;
	x3 = MyMax(alpha_x, alpha_y);
	x1 = ((y1 - c_x) * x3) / alpha_x;
	x2 = ((y2 - c_y) * x3) / alpha_y;

	float norm = x1*x1 + x2*x2 + x3*x3;
	norm = sqrtf(norm);
	x1 /= norm;
	x2 /= norm;
	x3 /= norm;

	MyLine3D* value = new MyLine3D(0, 0, 0, x1, x2, x3);
	return value;
}

Array* GenerateTranslate(double sh_x, double sh_y, double sh_z)
{
	double x = sh_x; //cos(alpha) * cos(gamma);
	double y = sh_y; //sin(alpha) * cos(gamma);
	double z = sh_z; //sin(gamma);
	Array* result = new Array();
	result->init(4, 4);
	for(int i = 0; i < 4; ++i)
	{
		result->set(i, i, 1);
	}
	result->set(0, 3, x);
	result->set(1, 3, y);
	result->set(2, 3, z);

	return result;
}

void GenerateTranslationCoordinates(double alpha, double gamma, double &sh_x, double &sh_y, double & sh_z)
{
	double x = cos(alpha) * cos(gamma);
	double y = sin(alpha) * cos(gamma);
	double z = sin(gamma);

	sh_x = x;
	sh_y = y;
	sh_z = z;
}

void ConvertTranslationCoordinatesToAngles(double &alpha, double &gamma, double sh_x, double sh_y, double sh_z)
{

	double norm = sh_x * sh_x + sh_y * sh_y + sh_z * sh_z;
	norm = sqrt( norm );

	sh_x /= norm;
	sh_y /= norm;
	sh_z /= norm;

	gamma = asin(sh_z);

	sh_x /= cos(gamma);
	sh_y /= cos(gamma);

	alpha = asin(sh_y);
	double myAccPi = acos( -1.0);
	if( alpha < 0 )
	{
		myAccPi = - myAccPi;
	}
	if( sh_x < 0 )
	{
		alpha = myAccPi - alpha;
	}

}

Array* GenerateTranslate(double alpha, double gamma)
{
	double x = cos(alpha) * cos(gamma);
	double y = sin(alpha) * cos(gamma);
	double z = sin(gamma);
	Array* result = new Array();
	result->init(4, 4);
	for(int i = 0; i < 4; ++i)
	{
		result->set(i, i, 1);
	}
	result->set(0, 3, x);
	result->set(1, 3, y);
	result->set(2, 3, z);

	return result;
}

Array* GenerateInner(double f, double cX, double cY)
{
	Array* result = new Array();
	result->init(4, 4);
	result->set(0, 0, f);
	result->set(1, 1, f);
	result->set(2, 2, 1);
	result->set(0, 2, cX);
	result->set(1, 2, cY);
	return result;
}

Array* GenerateRotateAroundZ(double rot)
{
	Array* result = new Array();
	result->init(4,4);
	double c, s;
	c = cos(rot);
	s = sin(rot);

	result->set(0,0, c);
	result->set(0,1, s);
	result->set(1,0, -s);
	result->set(1,1, c);
	result->set(2,2, 1);
	result->set(3,3, 1);

	return result;
}

Array* GenerateRotateAroundY(double rot)
{
	Array* result = new Array();
	result->init(4,4);
	double c, s;
	c = cos(rot);
	s = sin(rot);

	result->set(0,0, c);
	result->set(0,2, s);
	result->set(2,0, -s);
	result->set(2,2, c);
	result->set(1,1, 1);
	result->set(3,3, 1);

	return result;
}

Array* GenerateRotateAroundX(double rot)
{
	Array* result = new Array();
	result->init(4,4);
	double c, s;
	c = cos(rot);
	s = sin(rot);

	result->set(1,1, c);
	result->set(1,2, s);
	result->set(2,1, -s);
	result->set(2,2, c);
	result->set(0,0, 1);
	result->set(3,3, 1);

	return result;
}

Array* Generate3DHomogeneousPoint(double t, double f, double c_x, double c_y, double u_0, double v_0)
{
	double x, y, z;
	x = ((u_0 - c_x) * t) / f;
	y = ((v_0 - c_y) * t) / f;
	z =  t;
	Array* result = new Array();

	result->init(4, 1);
	result->set(0, 0, x);
	result->set(1, 0, y);
	result->set(2, 0, z);
	result->set(3, 0, 1);

	return result;
}