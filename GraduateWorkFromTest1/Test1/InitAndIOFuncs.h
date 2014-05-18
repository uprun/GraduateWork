#ifndef INITANDIOFUNCS_H
#define INITANDIOFUNCS_H
#define INFINITE 1e100

#include <opencv\cv.hpp>
#include <opencv\highgui.h>

#include <opencv2\nonfree\nonfree.hpp>
#include <opencv2\nonfree\features2d.hpp>

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <cmath>
//#include <math.h>
#include <algorithm>
#include <stdio.h>

#include "MyLine3D.h"
#include "MyArray.h"

using namespace std;
using namespace cv;
void WriteMatrixCoef(CvMat *matr);
void AddPointsToInnerCalibrate(vector<CvPoint2D32f> &qu,CvPoint2D32f* mas,int count);
void PrintAllPointsForInnerCalibrate(vector<CvPoint2D32f> &qu,int count_per_group);
void InitCvMatPointsParametersForInnerCallibration_part1(vector<CvPoint2D32f> &qu,int count_per_group,CvMat* &object_points,CvMat* &image_points,CvMat* &points_count,int szW,int szH);
void InitOtherCameraParametersForInnerCallibration_part2(int count_of_groups,CvMat* &cam_matr,CvMat* &dist_coefs,CvMat* &r_vecs,CvMat* &t_vecs);
struct ComparedIndexes
{
	float comparer_value;
	pair<int,int> comp_pair;
	int counterOfMatches;
	double medianOfComparedMatches;
	double medianOfSquaresComparedMatches;
	double degreesBetweenDeltaVector;
	ComparedIndexes(float comp_value,pair<int,int> &c_pair)
	{
		comparer_value=comp_value;
		comp_pair=c_pair;
		counterOfMatches = -1;
		medianOfComparedMatches = -1;
		medianOfSquaresComparedMatches = -1;
		degreesBetweenDeltaVector = 180;
	}
	ComparedIndexes(float comp_value,pair<int,int> &c_pair, int match_counter)
	{
		comparer_value=comp_value;
		comp_pair=c_pair;
		counterOfMatches = match_counter;
		medianOfComparedMatches = -1;
		medianOfSquaresComparedMatches = -1;
		degreesBetweenDeltaVector = 180;
	}
	ComparedIndexes(float comp_value,pair<int,int> &c_pair, int match_counter, double medianOfValues)
	{
		comparer_value=comp_value;
		comp_pair=c_pair;
		counterOfMatches = match_counter;
		medianOfComparedMatches = medianOfValues;
		medianOfSquaresComparedMatches = -1;
		degreesBetweenDeltaVector = 180;
	}
	ComparedIndexes(float comp_value,pair<int,int> &c_pair, int match_counter, double medianOfValues, double medianOfSquaresOfValues, double degreesAngleBetweenDeltaVector = 180)
	{
		comparer_value=comp_value;
		comp_pair=c_pair;
		counterOfMatches = match_counter;
		medianOfComparedMatches = medianOfValues;
		medianOfSquaresComparedMatches = medianOfSquaresOfValues;
		degreesBetweenDeltaVector = degreesAngleBetweenDeltaVector;
	}
	
	double Variance()
	{
		return medianOfSquaresComparedMatches - (medianOfComparedMatches * medianOfComparedMatches);
	}
};
int my_comparator_for_qsort(const void* A,const void* B);
bool my_comparator_for_stable_sort(const ComparedIndexes &A,const ComparedIndexes &B);

// try to open file if it fails - return false
// return true if it possible to open it
bool checkIfFileExist(const char* file_path);




#endif