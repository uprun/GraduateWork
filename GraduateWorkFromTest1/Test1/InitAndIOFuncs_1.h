#ifndef INITANDIOFUNCS_H
#define INITANDIOFUNCS_H
#include <opencv\cv.hpp>
#include <opencv\highgui.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
using namespace std;
using namespace cv;
void WriteMatrixCoef(CvMat *matr);
void AddPointsToInnerCalibrate(vector<CvPoint2D32f> &qu,CvPoint2D32f* mas,int count);
void PrintAllPointsForInnerCalibrate(vector<CvPoint2D32f> &qu,int count_per_group);
void InitCvMatPointsParametersForInnerCallibration_part1(vector<CvPoint2D32f> &qu,int count_per_group,CvMat* &object_points,CvMat* &image_points,CvMat* &points_count,int szW,int szH);
void InitOtherCameraParametersForInnerCallibration_part2(int count_of_groups,CvMat* &cam_matr,CvMat* &dist_coefs,CvMat* &r_vecs,CvMat* &t_vecs);

#endif