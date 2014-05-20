#ifndef SUPPORTFUNCTIONS_H
#define SUPPORTFUNCTIONS_H
#include "InitAndIOFuncs.h"
void MergeTwoImages(IplImage* one,IplImage* sec,IplImage* &res);

float SURFDescriptorsAbsoluteDifference(KeyPoint* pt1,float* desc1,KeyPoint* pt2,float* desc2,int descr_length, double meanAngleDiff = 0,
	double standardDeviationOfAngleDiff = 180, float maxRadiusOfMatching = 200);

void GetCorrespondingPointsForSURF(vector<KeyPoint>& left_seq,
						Mat& left_descr,
						vector<KeyPoint>& right_seq,
						Mat& right_descr,
						vector< ComparedIndexes > &LeftToRight_corr_pts,
						vector< ComparedIndexes > &RightToLeft_corr_pts
						);
void GetCorrespondingPointsForSURFWithFundamentalMatrix(
						vector<KeyPoint>& left_seq,
						Mat& left_descr,
						vector<KeyPoint>& right_seq,
						Mat& right_descr,
						vector< ComparedIndexes > &LeftToRight_corr_pts,
						vector< ComparedIndexes > &RightToLeft_corr_pts,
						CvMat* fundamentalMatrix,
						float ro_max,
						double meanAngleDiff = 0,
						double standardDeviationOfAngleDiff = 180,
						double delta_x = 0,
						double delta_y = 0,
						double maxDiffAngle = 180
						);
void PreparePointsForFindingFundamentalMatrix(CvMat* &ref_points1,
											CvMat* &ref_points2,
											CvMat* &ref_fund_matr,
											vector< ComparedIndexes > &left_right_corres,
											vector< ComparedIndexes > &right_left_corres,
											int count,
											vector<KeyPoint> &left_points_seq,
											vector<KeyPoint> &right_points_seq);


CvMat* getTranslationVectorFromMatrixTx(CvMat* matr_tx);

MyLine3D* getLineWithTransformations(float y1, float y2, CvMat* k, CvMat* r, CvMat* t);

MyLine3D* getLineWithTransformations(float y1, float y2, CvMat* k, CvMat* rt);

MyLine3D* getJustLine(float y1, float y2, CvMat* k);

void multiplicateByMinusOne(CvMat* m);

void getIntersections(CvMat* k, CvMat* r, CvMat* t, CvMat* points1, CvMat* points2, CvMat* times);

void getIntersections(CvMat* k, CvMat* rt, CvMat* points1, CvMat* points2, CvMat* times);

void analyzeIntersectionTimes(CvMat* times, int & good, int & total);

void getMiddleIntersectionPoints(CvMat* k, CvMat* r, CvMat* t, CvMat* points1, CvMat* points2, CvMat* times, CvMat* points_3d);

void getMiddleIntersectionPoints(CvMat* k, CvMat* rt, CvMat* points1, CvMat* points2, CvMat* times, CvMat* points_3d);

CvMat* combineMatrixAndVector(CvMat* r, CvMat* t);

CvMat* get001Vector();

CvMat* createHomogeneousVector(float x, float y, float z);

void SaveAcceptedCorresspondings(vector< ComparedIndexes > &left_right_corres,
											vector< ComparedIndexes > &right_left_corres,
											vector<KeyPoint> &left_points_seq,
											vector<KeyPoint> &right_points_seq,
											bool* acceptedLeftToRightCorrespondings,
											int sizeOfAccepptedLeftToRightCorrespondings);

void ConvertAcceptedCorresspondingsToMyArray(vector< ComparedIndexes > &left_right_corres,
											vector< ComparedIndexes > &right_left_corres,
											vector<KeyPoint> &left_points_seq,
											vector<KeyPoint> &right_points_seq,
											bool* acceptedLeftToRightCorrespondings,
											int sizeOfAccepptedLeftToRightCorrespondings,
											Array &left_points,
											Array &right_points
											);

double myAbsD(double a);
double distanceFromLine2D(double a, double b, double c, double x1, double y1);
void drawEpipolarLinesOnLeftAndRightImages(IplImage *mergedImages, CvPoint &leftPoint, CvPoint &rightPoint, CvMat* fundamentalMatrix);

bool getDistancesToCorrespondingEpipolarLines(CvPoint &leftPoint, CvPoint &rightPoint, CvMat* fundamentalMatrix,
	double &distanceForLeftPoint, double &distanceForRightPoint);
float getDifferenceBetweenAngles(double alpha, double betta);

void getAcceptedCorrespondingsForFindingModelParameters(
	vector< ComparedIndexes > &left_to_right_corresponding_points,
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	CvMat *fundamentalMatrix,
	bool* &acceptedLeftToRightCorrespondings,
	int &sizeOfAccepptedLeftToRightCorrespondings
	);

void getAcceptedCorrespondingsForReconstructionScene(
	vector< ComparedIndexes > &left_to_right_corresponding_points,
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	CvMat *fundamentalMatrix,
	bool* &acceptedForReconstructionLeftToRightCorrespondings,
	int &sizeOfAccepptedLeftToRightCorrespondings
	);

void getEpipolarLines(
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	CvMat *fundamentalMatrix,
	CvMat* &epipolar_linesForLeftPoints,
	CvMat* &epipolar_linesForRightPoints
	);

bool getDistancesToCorrespondingEpipolarLines(
	int indexInLeftPoints, 
	int indexInRightPoints, 
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	CvMat* epipolar_lines_for_left_points,
	CvMat* epipolar_lines_for_right_points,
	double &distanceForLeftPoint,
	double &distanceForRightPoint
	);

void openTwoImages(const char* left_image_file_path, const char* right_image_file_path, IplImage* &left_img, IplImage* &right_img  );
void extractFeaturesFromImage(IplImage* left_img, double min_hessian_value, IplImage* gray_img_left, vector<KeyPoint> &key_points_left, Mat &descriptors_left);
int myMin(int a, int b);
void filterPointsByDistanceFromEpipolarLines(CvMat* points1,
											 CvMat* points2,
											 int count_of_points,
											 CvMat* fundamentalMatrix,
											 double max_ro,
											 CvMat* &outPoints1,
											 CvMat* &outPoints2,
											 int &count_of_out_points
											 );

double getDegreesAngleBetweenVectors(KeyPoint* pointL, KeyPoint* pointR, double delta_x, double delta_y);

void findFundamentalMatrixAndCorrespondingPointsForReconstruction(
	vector< ComparedIndexes > &left_to_right_corresponding_points,
	vector< ComparedIndexes > &right_to_left_corresponding_points,
	CvMat *&fundamentalMatrix, 
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	Mat &descriptors_left,
	Mat &descriptors_right,
	IplImage* &left_img,
	IplImage* &right_img,
	IplImage* &gray_img_left,
	IplImage* &gray_img_right,
	Array &forReconstructionLeftPoints,
	Array &forReconstructionRightPoints,
	double min_hessian_value,
	double min_hessian_value_for_recosntruction_scene = 350);

#endif