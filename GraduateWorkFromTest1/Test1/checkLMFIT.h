#ifndef CHECKLMFIT_H
#define CHECKLMFIT_H

#include "lmmin.h"
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>
#include "MyArray.h"
#include "MyLine3D.h"
#include "SupportCode.h"
#include "ModelEvaluators.h"
#include <opencv\cv.hpp>

using namespace std;
using namespace cv;

int mainLevenbergMarkvardt_LMFIT(double focus, const char* fileNameForPlyExport, Array& left_points, Array &right_points,
		Mat &leftImage, Mat & rightImage, Array& reconstruction_left_points, Array& reconstruction_right_points);
#endif