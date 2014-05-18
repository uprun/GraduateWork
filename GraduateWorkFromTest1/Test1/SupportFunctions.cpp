#include "SupportFunctions.h"

void MergeTwoImages(IplImage* one,IplImage* sec,IplImage* &res)
{
	int width_part,height_part;
	width_part=one->width;
	height_part=one->height;
	if(res!=0)
	{
		cvReleaseImage(&res);
		res=cvCreateImage(cvSize(width_part<<1,height_part),one->depth,3);
	}
	else
	{
		res=cvCreateImage(cvSize(width_part<<1,height_part),one->depth,3);
	}
	cvSetImageROI(res,cvRect(0,0,width_part,height_part));
	cvCopy(one,res);
	cvSetImageROI(res,cvRect(width_part,0,width_part,height_part));
	cvCopy(sec,res);
	cvSetImageROI(res,cvRect(0,0,width_part<<1,height_part));
}

// angles should be in degrees
float getDifferenceBetweenAngles(double alpha, double betta)
{

	double angleDiff1 = myAbsD  (alpha - betta);

	if(angleDiff1 > 180)
	{
		angleDiff1 -= 360;
	}
	angleDiff1 = myAbsD(angleDiff1);

	return angleDiff1;
	
}

float SURFDescriptorsAbsoluteDifference(KeyPoint* pt1,float* desc1,KeyPoint* pt2,float* desc2,int descr_length, double meanAngleDiff,
	double standardDeviationOfAngleDiff, float maxRadiusOfMatching)
{
	//дескрипторы сфоримрованы инвариантно к масштабу
	
	/*if(pt1->octave!=pt2->octave)
		return INFINITE;*/
	double angleDiff1 = getDifferenceBetweenAngles( pt1->angle,  pt2->angle);

	double angleDiffFromMeanAngleDiff = getDifferenceBetweenAngles( angleDiff1, meanAngleDiff);
	
	if( angleDiffFromMeanAngleDiff > 3 * standardDeviationOfAngleDiff)
		return 200;
	float absX=0,absY=0;
	absX = pt1->pt.x - pt2->pt.x;
	absX *= absX;
	/*if(absX<0)
		absX=-absX;*/
	absY=pt1->pt.y - pt2->pt.y;
	/*if(absY<0)
		absY=-absY;*/
	absY *= absY;
	

	float _total_current_sum = absX + absY;

	maxRadiusOfMatching *= maxRadiusOfMatching;

	//if(_total_current_sum > maxRadiusOfMatching )
	//{
	//	return (_total_current_sum - maxRadiusOfMatching) * 100;
	//		//return INFINITE;
	//}
	

	float tot_sum=0;
	float buf;
	float buf_sum=0;
	for(int i=0;i<descr_length;i+=4)
	{
		buf_sum=0;
		buf=desc1[i]-desc2[i];
		buf_sum+=buf*buf;
		buf=desc1[i+1]-desc2[i+1];
		buf_sum+=buf*buf;
		buf=desc1[i+2]-desc2[i+2];
		buf_sum+=buf*buf;
		buf=desc1[i+3]-desc2[i+3];
		buf_sum+=buf*buf;
		buf_sum=sqrtf(buf_sum);
		tot_sum+=buf_sum;
	}

	return (tot_sum);
}



void GetCorrespondingPointsForSURF(vector<KeyPoint>& left_seq,
						Mat& left_descr,
						vector<KeyPoint>& right_seq,
						Mat& right_descr,
						vector< ComparedIndexes > &LeftToRight_corr_pts,
						vector< ComparedIndexes > &RightToLeft_corr_pts
						)
{
	LeftToRight_corr_pts.clear();
	RightToLeft_corr_pts.clear();
	int cur_descr_length=64;
	float* best_compare_value_for_right=new float[right_seq.size()];
	float* best_compare_value_for_left=new float[left_seq.size()];
	int* best_index_for_right=new int[right_seq.size()];
	int* best_index_for_left=new int[left_seq.size()];
	for(int i=0;i<right_seq.size();i++)
	{
		best_compare_value_for_right[i]=INFINITE;
		best_index_for_right[i]=-1;
	}
	for(int i=0;i<left_seq.size();i++)
	{
		best_compare_value_for_left[i]=INFINITE;
		best_index_for_left[i]=-1;
	}


	float compar_value_buf=0;
	float best_compare_value_left=0;
	int best_index_buf_left=-1;
	KeyPoint *left_point_ptr=0,*right_point_ptr=0;
	float *left_point_descriptor=0,*right_point_descriptor=0;
	for(int iL=0;iL<left_seq.size();iL++)
	{
		best_index_buf_left=-1;
		left_point_ptr=&(left_seq[iL]);
		left_point_descriptor = left_descr.ptr<float>(iL); //(left_descr.data + (iL* left_descr.cols));
		for(int iR=0;iR<right_seq.size();iR++)
		{
			right_point_ptr=&(right_seq[iR]);
			right_point_descriptor= right_descr.ptr<float>(iR);//(right_descr.data + (iR * right_descr.cols));
			compar_value_buf=SURFDescriptorsAbsoluteDifference(left_point_ptr,left_point_descriptor,right_point_ptr,right_point_descriptor,cur_descr_length);
			if(best_index_for_left[iL]<0)
			{
				best_index_for_left[iL]=iR;
				best_compare_value_for_left[iL]=compar_value_buf;
			}
			if(compar_value_buf<best_compare_value_for_left[iL])
			{
				best_index_for_left[iL]=iR;
				best_compare_value_for_left[iL]=compar_value_buf;
			}
			if(best_index_for_right[iR]<0)
			{
				best_index_for_right[iR]=iL;
				best_compare_value_for_right[iR]=compar_value_buf;
			}
			if(compar_value_buf<best_compare_value_for_right[iR])
			{
				best_index_for_right[iR]=iL;
				best_compare_value_for_right[iR]=compar_value_buf;
			}
		}
		//LeftToRight_corr_pts.push_back(pair<int,int>(iL,best_index_buf_left));
	}
	for(int iL=0;iL<left_seq.size();iL++)
	{
		// here first in pair is at first image, second in pair at second image
		LeftToRight_corr_pts.push_back(ComparedIndexes(best_compare_value_for_left[iL],pair<int,int>(iL,best_index_for_left[iL])));
	}
	for(int iR=0;iR<right_seq.size();iR++)
	{
		// here first in pair is at second image, second in pair at first image
		RightToLeft_corr_pts.push_back(ComparedIndexes(best_compare_value_for_right[iR],pair<int,int>(iR,best_index_for_right[iR])));
	}
	delete[] best_index_for_right;
	delete[] best_compare_value_for_right;
	delete[] best_index_for_left;
	delete[] best_compare_value_for_left;
	//qsort(LeftToRight_corr_pts.data(),LeftToRight_corr_pts.size(),sizeof(ComparedIndexes),my_comparator_for_qsort);
	//qsort(RightToLeft_corr_pts.data(),RightToLeft_corr_pts.size(),sizeof(ComparedIndexes),my_comparator_for_qsort);
	stable_sort(LeftToRight_corr_pts.begin(),LeftToRight_corr_pts.end(),my_comparator_for_stable_sort);
	stable_sort(RightToLeft_corr_pts.begin(),RightToLeft_corr_pts.end(),my_comparator_for_stable_sort);
}

void GetCorrespondingPointsForSURFWithFundamentalMatrix(
						vector<KeyPoint>& left_seq,
						Mat& left_descr,
						vector<KeyPoint>& right_seq,
						Mat& right_descr,
						vector< ComparedIndexes > &LeftToRight_corr_pts,
						vector< ComparedIndexes > &RightToLeft_corr_pts,
						CvMat* fundamentalMatrix,
						float ro_max,
						double meanAngleDiff,
						double standardDeviationOfAngleDiff,
						double delta_x,
						double delta_y,
						double maxDiffAngle
						)
{
	LeftToRight_corr_pts.clear();
	RightToLeft_corr_pts.clear();
	int cur_descr_length=64;
	float* best_compare_value_for_right=new float[right_seq.size()];
	float* best_compare_value_for_left=new float[left_seq.size()];
	double* sumsOfComparedValues_left=new double[left_seq.size()];
	double* sumsOfSquaresOfValues_left = new double[left_seq.size()];
	double* angleToDeltaVector = new double[left_seq.size()];
	int* best_index_for_right=new int[right_seq.size()];
	int* best_index_for_left=new int[left_seq.size()];
	int* countOfMatches = new int[left_seq.size()];
	for(int i=0;i<right_seq.size();i++)
	{
		best_compare_value_for_right[i]=INFINITE;
		best_index_for_right[i] = 0;
	}
	for(int i=0;i<left_seq.size();i++)
	{
		best_compare_value_for_left[i]=INFINITE;
		best_index_for_left[i] = 0;
		countOfMatches[i] = 0;
	}


	CvMat* epipolarLinesForLeftPoints, *epipolarLinesForRightPoints;

	getEpipolarLines(left_seq, right_seq, fundamentalMatrix, epipolarLinesForLeftPoints, epipolarLinesForRightPoints);

	float compar_value_buf=0;
	float best_compare_value_left=0;
	int best_index_buf_left=-1;
	KeyPoint *left_point_ptr=0,*right_point_ptr=0;
	float *left_point_descriptor=0,*right_point_descriptor=0;
	
	for(int iL=0;iL<left_seq.size();iL++)
	{
		best_index_buf_left=-1;
		left_point_ptr= &(left_seq[iL]);
		left_point_descriptor= left_descr.ptr<float>(iL); //(left_descr.data + (iL * left_descr.cols));


		int counterOfMatches = 0;
		
		double sumOfComparedValues = 0;
		double sumOfSquaresOfValues = 0;
		for(int iR=0;iR<right_seq.size();iR++)
		{
			double distanceToLeft = -1, distanceToRight = -1;
			bool checkIsPassed = false;

			right_point_ptr=&(right_seq[iR]);
			right_point_descriptor= right_descr.ptr<float>(iR);//(right_descr.data + (iR* right_descr.cols));
			checkIsPassed = getDistancesToCorrespondingEpipolarLines(
				iL,
				iR,
				left_seq,
				right_seq,
				epipolarLinesForLeftPoints,
				epipolarLinesForRightPoints,
				distanceToLeft,
				distanceToRight);
			if(!checkIsPassed)
			{
				continue;
			}

			if(distanceToLeft < 0 || distanceToLeft > ro_max)
			{
				continue;
			}
			if(distanceToRight < 0 || distanceToRight > ro_max)
			{
				continue;
			}
			

			
			compar_value_buf=SURFDescriptorsAbsoluteDifference(left_point_ptr,left_point_descriptor,
				right_point_ptr,right_point_descriptor,cur_descr_length,
				meanAngleDiff,
				standardDeviationOfAngleDiff);
			if(compar_value_buf > 1)
			{
				continue;
			}
			double degreesBetweenDelta = getDegreesAngleBetweenVectors(left_point_ptr, right_point_ptr, delta_x, delta_y);
			if(abs(degreesBetweenDelta) > maxDiffAngle)
			{
				continue;
			}
			++counterOfMatches;
			sumOfComparedValues += compar_value_buf;
			sumOfSquaresOfValues += compar_value_buf * compar_value_buf;
			if(best_index_for_left[iL]<0)
			{
				best_index_for_left[iL]=iR;
				best_compare_value_for_left[iL]=compar_value_buf;
			}
			if(compar_value_buf<best_compare_value_for_left[iL])
			{
				best_index_for_left[iL]=iR;
				best_compare_value_for_left[iL]=compar_value_buf;
				angleToDeltaVector[iL] = degreesBetweenDelta;
			}
			if(best_index_for_right[iR]<0)
			{
				best_index_for_right[iR]=iL;
				best_compare_value_for_right[iR]=compar_value_buf;
			}
			if(compar_value_buf<best_compare_value_for_right[iR])
			{
				best_index_for_right[iR]=iL;
				best_compare_value_for_right[iR]=compar_value_buf;
			}
		}
		if(counterOfMatches == 0)
		{
			countOfMatches[iL] = -1;
		}
		else
		{
			countOfMatches[iL] = counterOfMatches;
		}
		
		sumsOfComparedValues_left[iL] = sumOfComparedValues;
		sumsOfSquaresOfValues_left[iL] = sumOfSquaresOfValues;
		//LeftToRight_corr_pts.push_back(pair<int,int>(iL,best_index_buf_left));
	}
	for(int iL=0;iL<left_seq.size();iL++)
	{
		// here first in pair is at first image, second in pair at second image
		//if(best_index_for_left[iL] >= 0 )
		{
			LeftToRight_corr_pts.push_back(ComparedIndexes(best_compare_value_for_left[iL],pair<int,int>(iL,best_index_for_left[iL]),
				countOfMatches[iL],
				sumsOfComparedValues_left[iL] / (countOfMatches[iL]),
				sumsOfSquaresOfValues_left[iL] / (countOfMatches[iL]),
				angleToDeltaVector[iL]
				));
		}
	}
	for(int iR=0;iR<right_seq.size();iR++)
	{
		// here first in pair is at second image, second in pair at first image
		//if(best_index_for_right[iR] >= 0)
		{
			RightToLeft_corr_pts.push_back(ComparedIndexes(best_compare_value_for_right[iR],pair<int,int>(iR,best_index_for_right[iR])));
		}
	}

	cvReleaseMat(& epipolarLinesForLeftPoints);
	cvReleaseMat(& epipolarLinesForRightPoints);
	delete[] best_index_for_right;
	delete[] best_compare_value_for_right;
	delete[] best_index_for_left;
	delete[] best_compare_value_for_left;
	delete[] countOfMatches;
	delete[] sumsOfComparedValues_left;
	delete[] sumsOfSquaresOfValues_left;
	delete[] angleToDeltaVector;
	//qsort(LeftToRight_corr_pts.data(),LeftToRight_corr_pts.size(),sizeof(ComparedIndexes),my_comparator_for_qsort);
	//qsort(RightToLeft_corr_pts.data(),RightToLeft_corr_pts.size(),sizeof(ComparedIndexes),my_comparator_for_qsort);
	stable_sort(LeftToRight_corr_pts.begin(),LeftToRight_corr_pts.end(),my_comparator_for_stable_sort);
	stable_sort(RightToLeft_corr_pts.begin(),RightToLeft_corr_pts.end(),my_comparator_for_stable_sort);
}

void PreparePointsForFindingFundamentalMatrix(CvMat* &ref_points1,
											CvMat* &ref_points2,
											CvMat* &ref_fund_matr,
											vector< ComparedIndexes > &left_right_corres,
											vector< ComparedIndexes > &right_left_corres,
											int count,
											vector<KeyPoint> &left_points_seq,
											vector<KeyPoint> &right_points_seq)
{
	// at this time correspondings are token only from left_right_corres
	if(count<8)
		count=8;
	ref_points1=cvCreateMat(count,2,CV_32FC1);//points at first image
	ref_points2=cvCreateMat(count,2,CV_32FC1);//points at second image
	ref_fund_matr=cvCreateMat(3,3,CV_32FC1);//fundamental matrix
	float *inner_mas_left=ref_points1->data.fl,*inner_mas_right=ref_points2->data.fl;
	KeyPoint *left_point,*right_point;
	int left_point_index,right_point_index;
	for(int i=0;i<count;i++)
	{
		left_point_index=left_right_corres[i].comp_pair.first;
		right_point_index=left_right_corres[i].comp_pair.second;
		left_point = &(left_points_seq [left_point_index]);
		right_point = &(right_points_seq [right_point_index]);
		inner_mas_left[i*2+0]=left_point->pt.x;
		inner_mas_left[i*2+1]=left_point->pt.y;
		inner_mas_right[i*2+0]=right_point->pt.x;
		inner_mas_right[i*2+1]=right_point->pt.y;
	}
}

CvMat* getTranslationVectorFromMatrixTx(CvMat* matr_tx)
{
	CvMat* value = cvCreateMat(3,1,CV_32FC1);
	value->data.fl[0] = - matr_tx->data.fl[1*3 + 2];
	value->data.fl[1] = matr_tx->data.fl[0*3 + 2];
	value->data.fl[2] = - matr_tx->data.fl[0*3 + 1];

	return value;
}

// y1 - x coordinate on screen
// y2 - y coordinate on screen
// k - inner calibration matrix
// r - rotation matrix
// t - transpose vector
MyLine3D* getLineWithTransformations(float y1, float y2, CvMat* k, CvMat* r, CvMat* t)
{
	float alpha_x, alpha_y, c_x, c_y;
	alpha_x = k->data.fl[0 * 3 + 0];
	alpha_y = k->data.fl[1 * 3 + 1];
	c_x = k->data.fl[0 * 3 + 2];
	c_y = k->data.fl[1 * 3 + 2];

	float x1, x2, x3;
	x1 = (y1 - c_x) / alpha_x;
	x2 = (y2 - c_y) / alpha_y;
	x3 = 1;

	CvMat* v =  cvCreateMat(3,1,CV_32FC1);
	CvMat* s = cvCreateMat(3,1,CV_32FC1);

	v->data.fl[0] = x1;
	v->data.fl[1] = x2;
	v->data.fl[2] = x3;

	
	s->data.fl[0] = 0;
	s->data.fl[1] = 0;
	s->data.fl[2] = 0;

	CvMat* v_1 = cvCreateMat(3,1, CV_32FC1);
	CvMat* s_1 = cvCreateMat(3,1, CV_32FC1);

	// translate two points
	cvSub(v, t, v_1); // changed by Alexander Kryvonos 02.05.2013 at 1:21
	cvSub(s, t, s_1);

	// rotate them
	cvMatMul(r, v_1, v);
	cvMatMul(r, s_1, s);

	// calculate new vector from s to v
	cvSub(v, s, v_1);
	cvCopy(v_1, v);

	MyLine3D* value = new MyLine3D(
		s->data.fl[0],
		s->data.fl[1],
		s->data.fl[2],
		v->data.fl[0],
		v->data.fl[1],
		v->data.fl[2]);


	cvReleaseMat(&v);
	cvReleaseMat(&v_1);
	cvReleaseMat(&s);
	cvReleaseMat(&s_1);

	return value;
}

MyLine3D* getJustLine(float y1, float y2, CvMat* k)
{
	float alpha_x, alpha_y, c_x, c_y;
	alpha_x = k->data.fl[0 * 3 + 0];
	alpha_y = k->data.fl[1 * 3 + 1];
	c_x = k->data.fl[0 * 3 + 2];
	c_y = k->data.fl[1 * 3 + 2];

	float x1, x2, x3;
	x1 = (y1 - c_x) / alpha_x;
	x2 = (y2 - c_y) / alpha_y;
	//x1 = (y1 - c_x) * alpha_y;
	//x2 = (y2 - c_y) * alpha_x;
	x3 = 1;

	MyLine3D* value = new MyLine3D(0, 0, 0, x1, x2, x3);
	return value;
}

void multiplicateByMinusOne(CvMat* m)
{
	int rows =  m->rows;
	int colls =  m->cols;

	for(int i = 0; i < colls; i++ )
	{
		for(int j = 0; j < rows; j++)
		{
			float value = - (m->data.fl[j * colls + i]);
			m->data.fl[j * colls + i] =  value;
		}
	}
}

void getIntersections(CvMat* k, CvMat* r, CvMat* t, CvMat* points1, CvMat* points2, CvMat* times)
{
	int rows = points1->rows;
	for(int i = 0; i < rows; i++)
	{
		float x1, y1, x2, y2;
		x1 = points1 -> data.fl[i * 2 + 0];
		y1 = points1 -> data.fl[i * 2 + 1];

		x2 = points2 -> data.fl[i * 2 + 0];
		y2 = points2 -> data.fl[i * 2 + 1];

		MyLine3D * line_1, * line_2;
		line_1 = getLineWithTransformations(x1, y1, k, r, t);
		line_2 = getJustLine(x2, y2, k);
		double time_1 = 0, time_2 = 0;

		line_2->intersection(*line_1, time_2, time_1);
		times->data.db[i * 2 + 0] = time_1;
		times->data.db[i * 2 + 1] = time_2;

		delete line_1;
		delete line_2;
	}

}

void analyzeIntersectionTimes(CvMat* times, int & good, int & total)
{
	total = times->rows;
	good = 0;
	for(int i = 0; i < total; i++)
	{
		if(times->data.db[i * 2 + 0] > 0 && times->data.db[i * 2 + 1] > 0)
			good++;
	}

}

void getMiddleIntersectionPoints(CvMat* k, CvMat* r, CvMat* t, CvMat* points1, CvMat* points2, CvMat* times, CvMat* points_3d)
{
	int rows = points1->rows;
	for(int i = 0; i < rows; i++)
	{
		float x1, y1, x2, y2;
		x1 = points1 -> data.fl[i * 2 + 0];
		y1 = points1 -> data.fl[i * 2 + 1];

		x2 = points2 -> data.fl[i * 2 + 0];
		y2 = points2 -> data.fl[i * 2 + 1];

		MyLine3D * line_1, * line_2;
		line_1 = getLineWithTransformations(x1, y1, k, r, t);
		line_2 = getJustLine(x2, y2, k);
		double time_1 = 0, time_2 = 0;

		time_1 = times->data.db[i * 2 + 0];
		time_2 = times->data.db[i * 2 + 1]; 

		MyPoint3D* point_line_1, * point_line_2;

		point_line_1 = line_1->getPointByTime(time_1);
		point_line_2 = line_2->getPointByTime(time_2);

		MyPoint3D* diff = point_line_2->subtract(point_line_1);

		double ro_distance = diff->normEuqlidian();

		MyPoint3D* half_diff = diff->multiplicationByNumber( 0.5 );

		MyPoint3D* middle_point = point_line_1->add(half_diff);

		points_3d->data.db[i * 4 + 0] = middle_point->getX();
		points_3d->data.db[i * 4 + 1] = middle_point->getY();
		points_3d->data.db[i * 4 + 2] = middle_point->getZ();
		points_3d->data.db[i * 4 + 3] = ro_distance;

		delete point_line_1;
		delete point_line_2;
		delete diff;
		delete half_diff;
		delete middle_point;


		delete line_1;
		delete line_2;
	}
}

CvMat* combineMatrixAndVector(CvMat* r, CvMat* t)
{
	CvMat* res = cvCreateMat(3, 4, CV_32FC1);
	for(int i = 0; i < 3; i ++ )
	{
		for(int j = 0; j < 3; j ++)
			res->data.fl[i * 4 + j ] = r->data.fl[i * 3 + j];
	}

	for(int i = 0; i < 3; i ++)
	{
		res->data.fl[i * 4 + 3 ] = t->data.fl[i * 1 + 0];
	}
	return res;
}

CvMat* get001Vector()
{
	CvMat* res = cvCreateMat(3, 1, CV_32FC1);
	res->data.fl[0 * 1 + 0] = 0;
	res->data.fl[1 * 1 + 0] = 0;
	res->data.fl[2 * 1 + 0] = 1;

	return res;
}

CvMat* createHomogeneousVector(float x, float y, float z)
{
	CvMat * res = cvCreateMat(4, 1, CV_32FC1);

	res->data.fl[0 * 1 + 0 ] = x;
	res->data.fl[1 * 1 + 0 ] = y;
	res->data.fl[2 * 1 + 0 ] = z;
	res->data.fl[3 * 1 + 0 ] = 1;

	return res;
}

MyLine3D* getLineWithTransformations(float y1, float y2, CvMat* k, CvMat* rt)
{
	float alpha_x, alpha_y, c_x, c_y;
	alpha_x = k->data.fl[0 * 3 + 0];
	alpha_y = k->data.fl[1 * 3 + 1];
	c_x = k->data.fl[0 * 3 + 2];
	c_y = k->data.fl[1 * 3 + 2];

	float x1, x2, x3;
	x1 = (y1 - c_x) / alpha_x;
	x2 = (y2 - c_y) / alpha_y;
	x3 = 1;

	CvMat* v = cvCreateMat(3, 1, CV_32FC1);
	CvMat* s = cvCreateMat(3, 1, CV_32FC1);

	CvMat* v_1 = createHomogeneousVector(x1, x2, x3);
	CvMat* s_1 = createHomogeneousVector(0, 0, 0);

	CvMat* buf_matr = cvCreateMat(3, 1,CV_32FC1);

	// translation nad rotation of two points
	cvMatMul(rt, v_1, v);
	cvMatMul(rt, s_1, s);

	// caculating new vector from s to v
	cvSub(v, s, buf_matr);
	cvCopy(buf_matr, v);

	MyLine3D* value = new MyLine3D(
		s->data.fl[0],
		s->data.fl[1],
		s->data.fl[2],
		v->data.fl[0],
		v->data.fl[1],
		v->data.fl[2]);


	cvReleaseMat(&v);
	cvReleaseMat(&v_1);
	cvReleaseMat(&s);
	cvReleaseMat(&s_1);
	cvReleaseMat(&buf_matr);

	return value;
}

void getIntersections(CvMat* k, CvMat* rt, CvMat* points1, CvMat* points2, CvMat* times)
{

	int rows = points1->rows;
	for(int i = 0; i < rows; i++)
	{
		float x1, y1, x2, y2;
		x1 = points1 -> data.fl[i * 2 + 0];
		y1 = points1 -> data.fl[i * 2 + 1];

		x2 = points2 -> data.fl[i * 2 + 0];
		y2 = points2 -> data.fl[i * 2 + 1];

		MyLine3D * line_1, * line_2;
		line_1 = getLineWithTransformations(x1, y1, k, rt);
		line_2 = getJustLine(x2, y2, k);
		double time_1 = 0, time_2 = 0;

		line_2->intersection(*line_1, time_2, time_1);
		times->data.db[i * 2 + 0] = time_1;
		times->data.db[i * 2 + 1] = time_2;

		delete line_1;
		delete line_2;
	}
}

void getMiddleIntersectionPoints(CvMat* k, CvMat* rt, CvMat* points1, CvMat* points2, CvMat* times, CvMat* points_3d)
{
	int rows = points1->rows;
	for(int i = 0; i < rows; i++)
	{
		float x1, y1, x2, y2;
		x1 = points1 -> data.fl[i * 2 + 0];
		y1 = points1 -> data.fl[i * 2 + 1];

		x2 = points2 -> data.fl[i * 2 + 0];
		y2 = points2 -> data.fl[i * 2 + 1];

		MyLine3D * line_1, * line_2;
		line_1 = getLineWithTransformations(x1, y1, k, rt);
		line_2 = getJustLine(x2, y2, k);
		double time_1 = 0, time_2 = 0;

		time_1 = times->data.db[i * 2 + 0];
		time_2 = times->data.db[i * 2 + 1]; 

		MyPoint3D* point_line_1, * point_line_2;

		point_line_1 = line_1->getPointByTime(time_1);
		point_line_2 = line_2->getPointByTime(time_2);

		MyPoint3D* diff = point_line_2->subtract(point_line_1);

		double ro_distance = diff->normEuqlidian();

		MyPoint3D* half_diff = diff->multiplicationByNumber( 0.5 );

		MyPoint3D* middle_point = point_line_1->add(half_diff);

		points_3d->data.db[i * 4 + 0] = middle_point->getX();
		points_3d->data.db[i * 4 + 1] = middle_point->getY();
		points_3d->data.db[i * 4 + 2] = middle_point->getZ();
		points_3d->data.db[i * 4 + 3] = ro_distance;

		delete point_line_1;
		delete point_line_2;
		delete diff;
		delete half_diff;
		delete middle_point;


		delete line_1;
		delete line_2;
	}
}

void SaveAcceptedCorresspondings(vector< ComparedIndexes > &left_right_corres,
											vector< ComparedIndexes > &right_left_corres,
											vector<KeyPoint> &left_points_seq,
											vector<KeyPoint> &right_points_seq,
											bool* acceptedLeftToRightCorrespondings,
											int sizeOfAccepptedLeftToRightCorrespondings)
{
	int countOfAccepted = 0;
	for( int i = 0; i < sizeOfAccepptedLeftToRightCorrespondings; ++i)
	{
		if(acceptedLeftToRightCorrespondings[i])
		{
			++countOfAccepted;
		}
	}

	int count = countOfAccepted;
	CvMat* ref_points1=cvCreateMat(count,2,CV_32FC1);//points at first image
	CvMat* ref_points2=cvCreateMat(count,2,CV_32FC1);//points at second image
	float *inner_mas_left=ref_points1->data.fl,*inner_mas_right=ref_points2->data.fl;
	KeyPoint *left_point,*right_point;
	int left_point_index,right_point_index;
	int j = 0;
	for(int i=0; i < sizeOfAccepptedLeftToRightCorrespondings;i++)
	{
		if( acceptedLeftToRightCorrespondings[i])
		{
			left_point_index=left_right_corres[i].comp_pair.first;
			right_point_index=left_right_corres[i].comp_pair.second;
			left_point = &(left_points_seq[ left_point_index ]);
			right_point = &(right_points_seq[ right_point_index ]);
			inner_mas_left[j*2+0]=left_point->pt.x;
			inner_mas_left[j*2+1]=left_point->pt.y;
			inner_mas_right[j*2+0]=right_point->pt.x;
			inner_mas_right[j*2+1]=right_point->pt.y;
			j++;
		}
	}
	cvSave("image_accepted_points_left.txt", ref_points1, "leftImageAcceptedPoints");
	cvSave("image_accepted_points_right.txt", ref_points2, "rightImageAcceptedPoints");

	cvReleaseMat(& ref_points1);
	cvReleaseMat(& ref_points2);

}

void ConvertAcceptedCorresspondingsToMyArray(vector< ComparedIndexes > &left_right_corres,
											vector< ComparedIndexes > &right_left_corres,
											vector<KeyPoint> &left_points_seq,
											vector<KeyPoint> &right_points_seq,
											bool* acceptedLeftToRightCorrespondings,
											int sizeOfAccepptedLeftToRightCorrespondings,
											Array &left_points,
											Array &right_points
											)
{
	int countOfAccepted = 0;
	for( int i = 0; i < sizeOfAccepptedLeftToRightCorrespondings; ++i)
	{
		if(acceptedLeftToRightCorrespondings[i])
		{
			++countOfAccepted;
		}
	}

	int count = countOfAccepted;
	left_points.dispose();
	right_points.dispose();
	left_points.init(count, 2);
	right_points.init(count, 2);
	KeyPoint *left_point,*right_point;
	int left_point_index,right_point_index;
	int j = 0;
	for(int i=0; i < sizeOfAccepptedLeftToRightCorrespondings;i++)
	{
		if( acceptedLeftToRightCorrespondings[i])
		{
			left_point_index=left_right_corres[i].comp_pair.first;
			right_point_index=left_right_corres[i].comp_pair.second;
			left_point = &(left_points_seq[ left_point_index ]);
			right_point = &(right_points_seq[ right_point_index ]);
			//inner_mas_left[j*2+0]=left_point->pt.x;
			left_points.set(j, 0, left_point->pt.x);
			//inner_mas_left[j*2+1]=left_point->pt.y;
			left_points.set(j, 1, left_point->pt.y);
			//inner_mas_right[j*2+0]=right_point->pt.x;
			right_points.set(j, 0, right_point->pt.x);
			//inner_mas_right[j*2+1]=right_point->pt.y;
			right_points.set(j, 1, right_point->pt.y);
			j++;
		}
	}

}

void drawEpipolarLinesOnLeftAndRightImages(IplImage *mergedImages, CvPoint &leftPoint, CvPoint &rightPoint, CvMat* fundamentalMatrix)
{
	if(fundamentalMatrix == 0 )
	{
		return;
	}
	int width_part=mergedImages->width>>1;
	int count_of_points = 10;
	CvMat* points1=cvCreateMat(count_of_points,2,CV_32FC1);//points at first image
	CvMat* epipolar_lines = cvCreateMat(count_of_points, 3, CV_32FC1);

	points1->data.fl[0] = leftPoint.x;
	points1->data.fl[1] = leftPoint.y;
	// compute epipolar lines for surrent point
	cvComputeCorrespondEpilines(points1, 1, fundamentalMatrix,epipolar_lines);

	float x1, y1;
		
	float a,b,c;
	a = epipolar_lines->data.fl[ 0];
	b = epipolar_lines->data.fl[ 1];
	c = epipolar_lines->data.fl[ 2];

	float t = -c;
	float x0, y0;
	x0 = a * t;
	y0 = b * t;
	x1 = x0 + 10000 * ( -b );
	y1 = y0 + 10000 * ( a );
	x0 -= 10000 * ( -b );
	y0 -= 10000 * ( a );
	x0 += width_part;
	x1 += width_part;


	cvReleaseMat(& epipolar_lines);
	cvReleaseMat(& points1);

	cvLine(mergedImages, cvPoint(x0, y0), cvPoint(x1, y1), CV_RGB(0,0,255));

	{
		CvMat* points1=cvCreateMat(count_of_points,2,CV_32FC1);//points at second image
		CvMat* epipolar_lines = cvCreateMat(count_of_points, 3, CV_32FC1);

		points1->data.fl[0] = rightPoint.x;
		points1->data.fl[1] = rightPoint.y;
		// compute epipolar lines for current point
		cvComputeCorrespondEpilines(points1, 2, fundamentalMatrix, epipolar_lines);

		float x1, y1;
		
		float a,b,c;
		a = epipolar_lines->data.fl[ 0];
		b = epipolar_lines->data.fl[ 1];
		c = epipolar_lines->data.fl[ 2];

		float t = -c;
		float x0, y0;
		x0 = a * t;
		y0 = b * t;
		x1 = x0 + 10000 * ( -b );
		y1 = y0 + 10000 * ( a );
		x0 -= 10000 * ( -b );
		y0 -= 10000 * ( a );


		cvReleaseMat(& epipolar_lines);
		cvReleaseMat(& points1);

		cvLine(mergedImages, cvPoint(x0, y0), cvPoint(x1, y1), CV_RGB(255,0,255));
	}
}

double getDistanceToEpipolarLineOnRightImage(CvPoint &leftPoint, CvPoint &rightPoint, CvMat* fundamentalMatrix)
{

	if(fundamentalMatrix == 0 )
	{
		return -100;
	}

	int count_of_points = 10;
	CvMat* points1=cvCreateMat(count_of_points,2,CV_32FC1);//points at first image
	CvMat* epipolar_lines = cvCreateMat(count_of_points, 3, CV_32FC1);

	points1->data.fl[0] = leftPoint.x;
	points1->data.fl[1] = leftPoint.y;
	// compute epipolar lines for surrent point
	cvComputeCorrespondEpilines(points1, 1, fundamentalMatrix,epipolar_lines);

	float x1, y1;
		
	float a,b,c;
	a = epipolar_lines->data.fl[ 0];
	b = epipolar_lines->data.fl[ 1];
	c = epipolar_lines->data.fl[ 2];

	


	cvReleaseMat(& epipolar_lines);
	cvReleaseMat(& points1);

	return distanceFromLine2D(a, b, c, rightPoint.x, rightPoint.y);

}

bool getDistancesToCorrespondingEpipolarLines(CvPoint &leftPoint, CvPoint &rightPoint, CvMat* fundamentalMatrix,
	double &distanceForLeftPoint, double &distanceForRightPoint)
{
	distanceForLeftPoint = -1;
	distanceForRightPoint = -1;
	if(fundamentalMatrix == 0 )
	{
		return false;
	}

	int count_of_points = 10;
	CvMat* points1=cvCreateMat(count_of_points,2,CV_32FC1);//points at first image
	CvMat* epipolar_lines = cvCreateMat(count_of_points, 3, CV_32FC1);


	// calculate distance from right point to epipolar line
	points1->data.fl[0] = leftPoint.x;
	points1->data.fl[1] = leftPoint.y;
	// compute epipolar lines for surrent point
	cvComputeCorrespondEpilines(points1, 1, fundamentalMatrix,epipolar_lines);

	
		
	float a,b,c;
	a = epipolar_lines->data.fl[ 0];
	b = epipolar_lines->data.fl[ 1];
	c = epipolar_lines->data.fl[ 2];

	
	distanceForRightPoint = distanceFromLine2D(a, b, c, rightPoint.x, rightPoint.y);


	// calculate distance from left point to epipolar line
	points1->data.fl[0] = rightPoint.x;
	points1->data.fl[1] = rightPoint.y;
	
	// compute epipolar lines for surrent point
	cvComputeCorrespondEpilines(points1, 2, fundamentalMatrix,epipolar_lines);

	
		
	
	a = epipolar_lines->data.fl[ 0];
	b = epipolar_lines->data.fl[ 1];
	c = epipolar_lines->data.fl[ 2];

	
	distanceForLeftPoint = distanceFromLine2D(a, b, c, leftPoint.x, leftPoint.y);


	cvReleaseMat(& epipolar_lines);
	cvReleaseMat(& points1);

	return true;

}

bool getDistancesToCorrespondingEpipolarLines(
	int indexInLeftPoints, 
	int indexInRightPoints, 
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	CvMat* epipolar_lines_for_left_points,
	CvMat* epipolar_lines_for_right_points,
	double &distanceForLeftPoint,
	double &distanceForRightPoint
	)
{
	distanceForLeftPoint = -1;
	distanceForRightPoint = -1;

	CvPoint leftPoint = cvPoint(key_points_left[indexInLeftPoints].pt.x, key_points_left[indexInLeftPoints].pt.y);
	CvPoint rightPoint = cvPoint(key_points_right[indexInRightPoints].pt.x, key_points_right[indexInRightPoints].pt.y);
		
	float a,b,c;
	a = epipolar_lines_for_left_points->data.fl[indexInLeftPoints * 3 + 0];
	b = epipolar_lines_for_left_points->data.fl[indexInLeftPoints * 3 + 1];
	c = epipolar_lines_for_left_points->data.fl[indexInLeftPoints * 3 + 2];

	
	distanceForRightPoint = distanceFromLine2D(a, b, c, rightPoint.x, rightPoint.y);

	a = epipolar_lines_for_right_points->data.fl[indexInRightPoints * 3 + 0];
	b = epipolar_lines_for_right_points->data.fl[indexInRightPoints * 3 + 1];
	c = epipolar_lines_for_right_points->data.fl[indexInRightPoints * 3 + 2];

	
	distanceForLeftPoint = distanceFromLine2D(a, b, c, leftPoint.x, leftPoint.y);

	return true;

}

double myAbsD(double a)
{
	if(a<0)
		return -a;
	return a;
}

// a*a + b*b == 1
double distanceFromLine2D(double a, double b, double c, double x1, double y1)
{
	double t = -c - a * x1 - b * y1;
	return myAbsD(t);

}

void getAcceptedCorrespondingsForFindingModelParameters(
	vector< ComparedIndexes > &left_to_right_corresponding_points,
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	CvMat *fundamentalMatrix,
	bool* &acceptedLeftToRightCorrespondings,
	int &sizeOfAccepptedLeftToRightCorrespondings
	)
{
	KeyPoint *leftPoint,*rightPoint,*leftPoint2,*rightPoint2;
	
	int currentCount = 0;
	int maxCount = 20;
			
	sizeOfAccepptedLeftToRightCorrespondings = left_to_right_corresponding_points.size();
	acceptedLeftToRightCorrespondings = new bool[sizeOfAccepptedLeftToRightCorrespondings];
	CvMat* epipolarLinesForLeftPoints = NULL, *epipolarLinesForRightPoints = NULL;
	getEpipolarLines(key_points_left, key_points_right, fundamentalMatrix, epipolarLinesForLeftPoints, epipolarLinesForRightPoints);
	for(int i = 0; i < sizeOfAccepptedLeftToRightCorrespondings;++i)
	{
		acceptedLeftToRightCorrespondings[i] = false;

		leftPoint= &(key_points_left[ left_to_right_corresponding_points[i].comp_pair.first ]);
		rightPoint=&(key_points_right[ left_to_right_corresponding_points[i].comp_pair.second ]);
				
		double distanceForLeftPoint, distanceForRightPoint;
		bool checkFlag = getDistancesToCorrespondingEpipolarLines( 
			cvPoint(leftPoint->pt.x,leftPoint->pt.y),
			cvPoint(rightPoint->pt.x,rightPoint->pt.y),
			fundamentalMatrix,
			distanceForLeftPoint,
			distanceForRightPoint
			);
		if(checkFlag &&
			( distanceForLeftPoint >= 0 && distanceForLeftPoint <= 3) &&
			(distanceForRightPoint >= 0 && distanceForRightPoint <= 3)
			)
		{
			acceptedLeftToRightCorrespondings[i] = true;

			int countOfMatches = left_to_right_corresponding_points[i].counterOfMatches;

			double SD = sqrt( left_to_right_corresponding_points[i].Variance() ) ;
			double median = left_to_right_corresponding_points[i].medianOfComparedMatches;
			double compValue = left_to_right_corresponding_points[i].comparer_value;
			double mark_1_5 = median - 1.5 * SD - compValue;
			//if(compValue > 0.30)
			//{
			//	acceptedLeftToRightCorrespondings[i] = false;
			//			
			//}
			//else
			//{
			//	if(countOfMatches <= 2)
			//	{
			//		acceptedLeftToRightCorrespondings[i] = true;
			//		
			//	}
			//	else
			//	{
			//		//acceptedLeftToRightCorrespondings[i] = mark_1_5 >= 0;
			//		acceptedLeftToRightCorrespondings[i] = false;
			//	}
			//}

			if(countOfMatches <= 2 && compValue < 0.56 && currentCount <= maxCount)
			{
				acceptedLeftToRightCorrespondings[i] = true;
				++currentCount;
			}
			else
			{
				acceptedLeftToRightCorrespondings[i] = false;
			}

		}			
	}
	if(epipolarLinesForLeftPoints != NULL)
	{
		cvReleaseMat(& epipolarLinesForLeftPoints);
	}
	if(epipolarLinesForRightPoints != NULL)
	{
		cvReleaseMat(& epipolarLinesForRightPoints);
	}
	
}

void getEpipolarLines(
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	CvMat *fundamentalMatrix,
	CvMat* &epipolar_linesForLeftPoints,
	CvMat* &epipolar_linesForRightPoints
	)
{
	if(fundamentalMatrix == NULL)
	{
		return;
	}
	int countOfPointsLeft = key_points_left.size();
	int countOfPointsRight = key_points_right.size();

	if(countOfPointsLeft < 10)
	{
		countOfPointsLeft = 10;
	}
	if(countOfPointsRight < 10)
	{
		countOfPointsRight = 10;
	}
	CvMat* points1=cvCreateMat(countOfPointsLeft,2,CV_32FC1);//points at first image
	epipolar_linesForLeftPoints = cvCreateMat(countOfPointsLeft, 3, CV_32FC1);

	CvMat* points2 = cvCreateMat(countOfPointsRight, 2, CV_32FC1); // points on second image
	epipolar_linesForRightPoints = cvCreateMat(countOfPointsRight, 3, CV_32FC1);

	countOfPointsLeft = key_points_left.size();
	countOfPointsRight = key_points_right.size();

	KeyPoint *leftPoint,*rightPoint;
	for(int i = 0; i< countOfPointsLeft; ++i)
	{
		leftPoint= &(key_points_left[ i ]);
		points1->data.fl[i*2] = leftPoint->pt.x;
		points1->data.fl[i*2 + 1] = leftPoint->pt.y;
	}
	
	cvComputeCorrespondEpilines(points1, 1, fundamentalMatrix, epipolar_linesForLeftPoints);
	cvReleaseMat(& points1);

	
	for( int i = 0; i < countOfPointsRight; ++i)
	{
		rightPoint=&(key_points_right[ i ]);
		points2->data.fl[i*2] = rightPoint->pt.x;
		points2->data.fl[i*2 + 1] = rightPoint->pt.y;
	}
	cvComputeCorrespondEpilines(points2, 2, fundamentalMatrix, epipolar_linesForRightPoints);
	cvReleaseMat(& points2);

		

	
	// compute epipolar lines for surrent point
	
}

void getAcceptedCorrespondingsForReconstructionScene(
	vector< ComparedIndexes > &left_to_right_corresponding_points,
	vector<KeyPoint> &key_points_left,
	vector<KeyPoint> &key_points_right,
	CvMat *fundamentalMatrix,
	bool* &acceptedForReconstructionLeftToRightCorrespondings,
	int &sizeOfAccepptedLeftToRightCorrespondings
	)
{
	KeyPoint *leftPoint,*rightPoint,*leftPoint2,*rightPoint2;
			
			
	sizeOfAccepptedLeftToRightCorrespondings = left_to_right_corresponding_points.size();
	
	acceptedForReconstructionLeftToRightCorrespondings = new bool[sizeOfAccepptedLeftToRightCorrespondings];
	for(int i = 0; i < sizeOfAccepptedLeftToRightCorrespondings;++i)
	{
		
		acceptedForReconstructionLeftToRightCorrespondings[i] = false;

		leftPoint= &(key_points_left[ left_to_right_corresponding_points[i].comp_pair.first ]);
		rightPoint=&(key_points_right[ left_to_right_corresponding_points[i].comp_pair.second ]);
				
		double distanceForLeftPoint, distanceForRightPoint;
		bool checkFlag = getDistancesToCorrespondingEpipolarLines( 
			cvPoint(leftPoint->pt.x,leftPoint->pt.y),
			cvPoint(rightPoint->pt.x,rightPoint->pt.y),
			fundamentalMatrix,
			distanceForLeftPoint,
			distanceForRightPoint
			);
		if(checkFlag &&
			( distanceForLeftPoint >= 0 && distanceForLeftPoint <= 3) &&
			(distanceForRightPoint >= 0 && distanceForRightPoint <= 3)
			)
		{
			

			int countOfMatches = left_to_right_corresponding_points[i].counterOfMatches;

			double SD = sqrt( left_to_right_corresponding_points[i].Variance() ) ;
			double median = left_to_right_corresponding_points[i].medianOfComparedMatches;
			double compValue = left_to_right_corresponding_points[i].comparer_value;
			double mark_1_5 = median - 1.5 * SD - compValue;

			acceptedForReconstructionLeftToRightCorrespondings[i] = true;
			if(compValue > 0.62 && countOfMatches >= 6 && mark_1_5 < 0)
			{
				acceptedForReconstructionLeftToRightCorrespondings[i] = false;
			}
			/*else
			{
				if(countOfMatches >= 5)
				{
					acceptedForReconstructionLeftToRightCorrespondings[i] = mark_1_5 >= 0;
				}
			}*/
		}			
	}

	
}

void openTwoImages(const char* left_image_file_path, const char* right_image_file_path, IplImage* &left_img, IplImage* &right_img  )
{
	//open left picture
	if(checkIfFileExist(left_image_file_path))
	{
		if(left_img != 0)
		{
			cvReleaseImage(& left_img);
		}
		left_img = cvLoadImage(left_image_file_path, CV_LOAD_IMAGE_COLOR);
	}

	//open right picture
	if( checkIfFileExist(right_image_file_path))
	{
		if(right_img != 0)
		{
			cvReleaseImage(& right_img);
		}
		right_img = cvLoadImage(right_image_file_path, CV_LOAD_IMAGE_COLOR);
	}
}

void extractFeaturesFromImage(IplImage* _img, double min_hessian_value, IplImage* _gray_img, vector<KeyPoint> & _key_points, Mat & _descriptors)
{
	//proceed left
	_gray_img=cvCreateImage(cvSize((_img->width),(_img->height)),IPL_DEPTH_8U,1);
	cvCvtColor(_img, _gray_img,CV_RGB2GRAY);
	cv::Mat mat_gray_leftImage(_gray_img, true);
	cvReleaseImage(&_gray_img);
			

	SurfFeatureDetector detector( min_hessian_value, 5, 3 );
	_key_points.clear();
	detector.detect( mat_gray_leftImage, _key_points );
			

	SurfDescriptorExtractor extractor;
	_descriptors.release();
	extractor.compute( mat_gray_leftImage, _key_points, _descriptors );

	mat_gray_leftImage.release();
}

int myMin(int a, int b)
{
	if(a < b)
		return a;
	return b;
}

void filterPointsByDistanceFromEpipolarLines(CvMat* points1,
											 CvMat* points2,
											 int count_of_points,
											 CvMat* fundamentalMatrix,
											 double max_ro,
											 CvMat* &outPoints1,
											 CvMat* &outPoints2,
											 int &count_of_out_points
											 )
{
	CvMat* epipolar_lines = cvCreateMat(count_of_points, 3, CV_32FC1);
	cvComputeCorrespondEpilines(points1, 1, fundamentalMatrix,epipolar_lines);

	int count_of_good_points = 0;

	for(int i = 0; i < count_of_points; i++)
	{
		float x1, y1;
		x1 = points2->data.fl[i * 2 + 0];
		y1 = points2->data.fl[i * 2 + 1];
		float a,b,c;
		a = epipolar_lines->data.fl[i * 3 + 0];
		b = epipolar_lines->data.fl[i * 3 + 1];
		c = epipolar_lines->data.fl[i * 3 + 2];
		double ro = distanceFromLine2D(a, b, c, x1, y1);
		if(ro <= max_ro)
			count_of_good_points++;
	}
	count_of_out_points = count_of_good_points;
	int j = 0;
	if(count_of_out_points > 0)
	{
		outPoints1 = cvCreateMat(count_of_out_points, 2, CV_32FC1);
		outPoints2 = cvCreateMat(count_of_out_points, 2, CV_32FC1);
		for(int i = 0; i < count_of_points; i++)
		{
			float x1, y1, x0, y0;
			x0 = points1->data.fl[i * 2 + 0];
			y0 = points1->data.fl[i * 2 + 1];

			x1 = points2->data.fl[i * 2 + 0];
			y1 = points2->data.fl[i * 2 + 1];
			float a,b,c;
			a = epipolar_lines->data.fl[i * 3 + 0];
			b = epipolar_lines->data.fl[i * 3 + 1];
			c = epipolar_lines->data.fl[i * 3 + 2];
			double ro = distanceFromLine2D(a, b, c, x1, y1);
			if(ro <= max_ro)
			{
				outPoints1->data.fl[j * 2 + 0] = x0;
				outPoints1->data.fl[j * 2 + 1] = y0;

				outPoints2->data.fl[j * 2 + 0] = x1;
				outPoints2->data.fl[j * 2 + 1] = y1;
				j++;
			}
		}
	}

	cvReleaseMat(& epipolar_lines);


}

double getDegreesAngleBetweenVectors(KeyPoint* pointL, KeyPoint* pointR, double delta_x, double delta_y)
{
	
	double buf_x, buf_y;
	buf_x = pointR->pt.x - pointL->pt.x;
	buf_y = pointR->pt.y - pointL->pt.y;
					
	double ro_buf = sqrt(buf_x * buf_x + buf_y * buf_y);
	if(abs(ro_buf) < 1)
	{
		return 0;
	}
	buf_x /= ro_buf;
	buf_y /= ro_buf;

	double vectMul = buf_y * delta_x - buf_x * delta_y;
	double scalMul = buf_x * delta_x + buf_y * delta_y;
	double alpha = acos(scalMul);
	if(vectMul < 0)
	{
		alpha = - alpha;
	}
	double myAccPi = acos( -1.0);
	alpha = (alpha * 180) / myAccPi; 
	return alpha;
}

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
	double min_hessian_value)
{
	int count= myMin(30, left_to_right_corresponding_points.size());

			cout<< "Count of point to process = " << count << endl;

			CvMat *points1,*points2;
			if(fundamentalMatrix != 0)
			{
				cvReleaseMat(& fundamentalMatrix);
				fundamentalMatrix = 0;

			}
			PreparePointsForFindingFundamentalMatrix(points1,points2,fundamentalMatrix,
				left_to_right_corresponding_points,
				right_to_left_corresponding_points,
				count,
				key_points_left,
				key_points_right);
			int matrix_count=-1;
			CvMat* inliersAndOutliers = cvCreateMat(count, 1, CV_8UC1);
			matrix_count=cvFindFundamentalMat(points1,
				points2,fundamentalMatrix,
				CV_FM_RANSAC,3.0, 
				0.99999999995999999999,
				inliersAndOutliers);
			if(matrix_count>1)
				cout<<endl<<"There is more than one fundamental matrix!!!"<<endl;
			if(matrix_count==0)
				cout<<endl<<"No Fundamental matrix was find!!!"<<endl;
			if(matrix_count==1)
				cout<<"Fundamental matrix succesfully found:"<<endl;
			WriteMatrixCoef(fundamentalMatrix);

			int countOfPassedPoints = 0;
			int sizeOfAccepptedLeftToRightCorrespondings = left_to_right_corresponding_points.size();
			
			KeyPoint* leftPoint, * rightPoint;
			for(int i = 0; i < sizeOfAccepptedLeftToRightCorrespondings ;++i)
			{
				
				if( i >= inliersAndOutliers->rows)
				{
					continue;
				}
				bool inliersValue = inliersAndOutliers->data.i[i] > 0;
				/*if(!inliersValue)
				{
					continue;
				}*/
				leftPoint= &(key_points_left[ left_to_right_corresponding_points[i].comp_pair.first ]);
				rightPoint=&(key_points_right[ left_to_right_corresponding_points[i].comp_pair.second ]);
				
				double distanceForLeftPoint, distanceForRightPoint;
				bool checkFlag = getDistancesToCorrespondingEpipolarLines( 
					cvPoint(leftPoint->pt.x,leftPoint->pt.y),
					cvPoint(rightPoint->pt.x,rightPoint->pt.y),
					fundamentalMatrix,
					distanceForLeftPoint,
					distanceForRightPoint
					);
				if(checkFlag &&
					( distanceForLeftPoint >= 0 && distanceForLeftPoint <= 3) &&
					(distanceForRightPoint >= 0 && distanceForRightPoint <= 3)
					)
				{
					++countOfPassedPoints;
				}	
			}

			
			
			

			cout << "Count of passed points : " << countOfPassedPoints<< endl;

			bool need_filtration = false;
			if(need_filtration)
			{
				CvMat *outPoints1, *outPoints2; 
				int next_count = 0;
				filterPointsByDistanceFromEpipolarLines(points1,points2, count, fundamentalMatrix, 3.0, outPoints1,outPoints2, next_count);
				if(next_count > 0)
				{
					cvReleaseMat(& points1);
					cvReleaseMat(& points2);
					points1 = outPoints1;
					points2 = outPoints2;
					cout<< "Filtration of Points is completed. " << "New count of points = " << next_count << " / "<< count<< endl;
					count = next_count;
				}
			}
			cvReleaseMat(&points1);
			cvReleaseMat(&points2);

			int pointsProcessed = 0;
			double sumOfDiffAngles = 0;
			double sumOfSquaresOfAnglesDiff = 0;
			double delta_x = 0, delta_y = 0;
			for( int i = 0; i < left_to_right_corresponding_points.size() && i < inliersAndOutliers->rows; ++i)
			{
				KeyPoint* pointL, *pointR;
				int iL, iR;
				iL = left_to_right_corresponding_points[i].comp_pair.first;
				iR = left_to_right_corresponding_points[i].comp_pair.second;
				pointL = &key_points_left[iL];
				pointR = &key_points_right[iR];

				double distanceForLeftPoint, distanceForRightPoint;
				bool checkFlag = getDistancesToCorrespondingEpipolarLines( 
					cvPoint(pointL->pt.x,pointL->pt.y),
					cvPoint(pointR->pt.x,pointR->pt.y),
					fundamentalMatrix,
					distanceForLeftPoint,
					distanceForRightPoint
					);
				bool inliersValue = inliersAndOutliers->data.i[i] > 0;
				if(!inliersValue)
				{
					continue;
				}
				if(checkFlag &&
					( distanceForLeftPoint >= 0 && distanceForLeftPoint <= 3) &&
					(distanceForRightPoint >= 0 && distanceForRightPoint <= 3)
					)
				{
					double currentDiffAngle = getDifferenceBetweenAngles(pointL->angle, pointR->angle);
					cout<< "angle = "<< currentDiffAngle ;
					sumOfDiffAngles += currentDiffAngle;
					sumOfSquaresOfAnglesDiff += currentDiffAngle * currentDiffAngle;
					double buf_x, buf_y;
					buf_x = pointR->pt.x - pointL->pt.x;
					buf_y = pointR->pt.y - pointL->pt.y;
					cout<< " dx = " << buf_x << " dy = " << buf_y ;
					
					double ro_buf = sqrt(buf_x * buf_x + buf_y * buf_y);
					buf_x /= ro_buf;
					buf_y /= ro_buf;
					cout<< " x = " << buf_x << " y = " << buf_y << endl;
					delta_x += buf_x;
					delta_y += buf_y;
					pointsProcessed++;
				}
			}

			if( pointsProcessed != 0)
			{
				sumOfDiffAngles /= pointsProcessed;
				sumOfSquaresOfAnglesDiff /= pointsProcessed;
				delta_x /= pointsProcessed;
				delta_y /= pointsProcessed;
				double ro_delta = sqrt(delta_x * delta_x + delta_y * delta_y);
				delta_x /= ro_delta;
				delta_y /= ro_delta;
			}
			double variance = sumOfSquaresOfAnglesDiff - sumOfDiffAngles * sumOfDiffAngles;

			if(variance < 0.2)
			{
				variance = 0.2;
			}

			cout<< "Median angle diff: "<< sumOfDiffAngles<< endl;
			cout << "variance : " << variance << endl;
			cout<< "Standard deviation: " << sqrt(variance) << endl;

			cout<< "delta_x = " << delta_x << " delta_y = " << delta_y << endl;

			cvReleaseMat(&inliersAndOutliers);

			bool needToRecalculateFeaturePoints = true;
			if( needToRecalculateFeaturePoints)
			{
				cout << endl<<  "Start to find correspondings for reconstruction" << endl;
				left_to_right_corresponding_points.clear();
				right_to_left_corresponding_points.clear();

				SurfFeatureDetector detector( 200, 5, 3 );
				SurfDescriptorExtractor extractor;

				//proceed left
				gray_img_left=cvCreateImage(cvSize((left_img->width),(left_img->height)),IPL_DEPTH_8U,1);
				cvCvtColor(left_img,gray_img_left,CV_RGB2GRAY);
				cv::Mat mat_gray_leftImage(gray_img_left, true);
				cvReleaseImage(&gray_img_left);
			
				detector.detect( mat_gray_leftImage, key_points_left );
				extractor.compute( mat_gray_leftImage, key_points_left, descriptors_left );

				mat_gray_leftImage.release();
			
		
				//proceed right
				gray_img_right=cvCreateImage(cvSize((right_img->width),(right_img->height)),IPL_DEPTH_8U,1);
				cvCvtColor(right_img,gray_img_right,CV_RGB2GRAY);
				cv::Mat mat_gray_rightImage(gray_img_right, true);
				cvReleaseImage(&gray_img_right);

				detector.detect( mat_gray_rightImage, key_points_right );			
				extractor.compute( mat_gray_rightImage, key_points_right, descriptors_right );

				mat_gray_rightImage.release();

				GetCorrespondingPointsForSURFWithFundamentalMatrix(
					key_points_left, 
					descriptors_left,
					key_points_right,
					descriptors_right,
					left_to_right_corresponding_points,
					right_to_left_corresponding_points,
					fundamentalMatrix,
					3,
					sumOfDiffAngles,
					sqrt(variance),
					delta_x,
					delta_y,
					70
					);
				cout << endl<<  "Finished to find new correspondings" << endl;

				// select points for reconstruction

				int sizeOfAccepptedLeftToRightCorrespondings = left_to_right_corresponding_points.size();
				
				bool* acceptedForReconstructionLeftToRightCorrespondings = 0;

				getAcceptedCorrespondingsForReconstructionScene(left_to_right_corresponding_points,
					key_points_left,
					key_points_right,
					fundamentalMatrix,
					acceptedForReconstructionLeftToRightCorrespondings,
					sizeOfAccepptedLeftToRightCorrespondings);

				ConvertAcceptedCorresspondingsToMyArray(left_to_right_corresponding_points,
					right_to_left_corresponding_points,
					key_points_left,
					key_points_right,
					acceptedForReconstructionLeftToRightCorrespondings,
					sizeOfAccepptedLeftToRightCorrespondings,
					forReconstructionLeftPoints,
					forReconstructionRightPoints
					);

				delete[] acceptedForReconstructionLeftToRightCorrespondings;

				
			}

			cout << endl<<  "Start to find correspondings for model parameters" << endl;
			left_to_right_corresponding_points.clear();
			right_to_left_corresponding_points.clear();

			SurfFeatureDetector detector( min_hessian_value, 5, 3 );
			SurfDescriptorExtractor extractor;

			//proceed left
			gray_img_left=cvCreateImage(cvSize((left_img->width),(left_img->height)),IPL_DEPTH_8U,1);
			cvCvtColor(left_img,gray_img_left,CV_RGB2GRAY);
			cv::Mat mat_gray_leftImage(gray_img_left, true);
			cvReleaseImage(&gray_img_left);
			
			detector.detect( mat_gray_leftImage, key_points_left );
			extractor.compute( mat_gray_leftImage, key_points_left, descriptors_left );

			mat_gray_leftImage.release();
			
		
			//proceed right
			gray_img_right=cvCreateImage(cvSize((right_img->width),(right_img->height)),IPL_DEPTH_8U,1);
			cvCvtColor(right_img,gray_img_right,CV_RGB2GRAY);
			cv::Mat mat_gray_rightImage(gray_img_right, true);
			cvReleaseImage(&gray_img_right);

			detector.detect( mat_gray_rightImage, key_points_right );			
			extractor.compute( mat_gray_rightImage, key_points_right, descriptors_right );

			mat_gray_rightImage.release();

			GetCorrespondingPointsForSURFWithFundamentalMatrix(
				key_points_left, 
				descriptors_left,
				key_points_right,
				descriptors_right,
				left_to_right_corresponding_points,
				right_to_left_corresponding_points,
				fundamentalMatrix,
				3,
				sumOfDiffAngles,
				sqrt(variance),
				delta_x,
				delta_y,
				70
				);
			cout << endl<<  "Finished to find correspondings for model parameters" << endl;
}