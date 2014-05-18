#include "InitAndIOFuncs.h"
void WriteMatrixCoef(CvMat *matr)
{
	int columns=matr->cols;
	int rows=matr->rows;
	int step=matr->step;
	float *inner_mas=matr->data.fl;
	for(int ir=0;ir<rows;ir++)
	{
		for(int ic=0;ic<columns;ic++)
		{
			cout<<inner_mas[ir*columns+ic]<<"\t| ";
		}
		cout<<endl<<"__________________________________________"<<endl;
	}
	cout<<"***********************************************"<<endl;
}
void AddPointsToInnerCalibrate(vector<CvPoint2D32f> &qu,CvPoint2D32f* mas,int count)
{
	if(qu.size()<count*20)
	{
		for(int i=0;i<count;i++)
		{
			//qu.push(mas[i]);
			qu.push_back(mas[i]);

		}
	}
}
void PrintAllPointsForInnerCalibrate(vector<CvPoint2D32f> &qu,int count_per_group)
{
	cout<<qu.size()/count_per_group<<endl<<"----------------------------------------------------------------"<<endl;
	int inner_group_counter=0;
	CvPoint2D32f buf;
	int count=qu.size();
	for(int i=0;i<count;i++)
	{
		inner_group_counter++;
		if(inner_group_counter>=count_per_group)
			inner_group_counter-=count_per_group;
		
		//buf=qu.front();
		buf=qu[i];

		//qu.pop();
		cout<<buf.x<<" "<<buf.y<<" | ";
		if(inner_group_counter==0)
			cout<<endl<<"------------------------------------------------------------------------"<<endl;
	}
	if(inner_group_counter!=0)
	{
		cout<<endl<<"Some error in PrintAllPointsToInnerCaliibrate "<<endl;
	}
}
void InitCvMatPointsParametersForInnerCallibration_part1(vector<CvPoint2D32f> &q,
								int count_per_group,
								CvMat* &object_points,
								CvMat* &image_points,
								CvMat* &points_count,
								int szW,
								int szH)
{
	object_points=cvCreateMat(q.size(),3,CV_32FC1);//contains set of object model coordinates
	int  tot_indx=0;//index for each point of object model
	int max_tot_indx=q.size();//maximum count of points for object model
	int inner_index=0;//index to fill object_points->data
	float* inner_mas=object_points->data.fl;
	while(tot_indx<max_tot_indx)
	{
		for(int i=0;i<szH;i++)
			for(int j=0;j<szW;j++)
			{
				inner_mas[inner_index++]=j;//x
				inner_mas[inner_index++]=i;//y
				inner_mas[inner_index++]=0;//z
				tot_indx++;
			}
	}
	image_points=cvCreateMat(q.size(),2,CV_32FC1);//contains projection coordinates of object model at each camera view
	inner_mas=image_points->data.fl;
	inner_index=0;
	for(int i=0;i<max_tot_indx;i++)
	{
		inner_mas[inner_index++]=q[i].x;
		inner_mas[inner_index++]=q[i].y;
	}

	int write_res=szH*szW;
	points_count=cvCreateMat(q.size()/count_per_group,1,CV_32SC1);
	int *inner_mas3=points_count->data.i;
	int max_i=max_tot_indx/count_per_group;
	for(int i=0;i<max_i;i++)
	{
		inner_mas3[i]=write_res;
	}
}
void InitOtherCameraParametersForInnerCallibration_part2(int count_of_groups,CvMat* &cam_matr,
													CvMat* &dist_coefs,
													CvMat* &r_vecs,
													CvMat* &t_vecs)
{
	cam_matr=cvCreateMat(3,3,CV_32FC1);
	float* inner_cam_matr=cam_matr->data.fl;
	inner_cam_matr[0*3+0]=10.f;
	inner_cam_matr[1*3+1]=10.f;
	dist_coefs=cvCreateMat(5,1,CV_32FC1);
	r_vecs=cvCreateMat(count_of_groups,3,CV_32FC1);
	t_vecs=cvCreateMat(count_of_groups,3,CV_32FC1);

}

