#ifndef SUPPORTCODE_H
#define SUPPORTCODE_H
#include "MyArray.h"
#include "MyLine3D.h"
#include <math.h>

double MyMax(double a, double b);

MyLine3D* getLineWithTransformations(float y1, float y2, Array* k, Array* r, Array* t);
MyLine3D* getJustLine(float y1, float y2, Array* k);

Array* Generate3DHomogeneousPoint(double t, double f, double c_x, double c_y, double u_0, double v_0);
Array* GenerateRotateAroundX(double rot);
Array* GenerateRotateAroundY(double rot);
Array* GenerateRotateAroundZ(double rot);
Array* GenerateInner(double f, double cX, double cY);
Array* GenerateTranslate(double alpha, double gamma);
Array* GenerateTranslate(double sh_x, double sh_y, double sh_z);
void GenerateTranslationCoordinates(double alpha, double gamma, double &sh_x, double &sh_y, double & sh_z);
void ConvertTranslationCoordinatesToAngles(double &alpha, double &gamma, double sh_x, double sh_y, double sh_z);
#endif