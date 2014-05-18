#include "InitAndIOFuncs.h"

int chess_b_szW=7,chess_b_szH=7;//size's of inner angles at chessboard
int imgH=480,imgW=640;
int selectedLeftLine=0;
float min_hessian=0.0f;
bool re_draw=false;
void onTrackbarSlide1(int pos);
void MergeTwoImages(IplImage* one,IplImage* sec,IplImage* &res);
void onTrackbarSlideSelectLine(int pos);
vector< pair<int,int> > left_to_right_corresponding_points;
vector< pair<int,int> > right_to_left_corresponding_points;
void GetCorrespondingPointsForSURF(CvSeq* left_seq,CvSeq* left_descr,CvSeq* right_seq,CvSeq* right_descr,vector< pair<int,int> > &LeftToRight_corr_pts,vector< pair<int,int> > &RightToLeft_corr_pts);
int main()
{
	
	if(false)
	{
	CvMat *object_points_all=0;
	CvMat *image_points_all=0;
	CvMat *points_count_all=0;
	CvMat *camera_matr=0;
	CvMat *distor_coefs=0;
	CvMat *rotation_vecs=0;
	CvMat *transpose_vecs=0;
	vector<CvPoint2D32f> qu_calibr_points;
	IplImage* frameCam1;
	cvNamedWindow("WindowCam1",CV_WINDOW_KEEPRATIO);
	CvCapture *captureCam1=cvCreateCameraCapture(0);
	IplImage *quarterFrame;
	CvPoint2D32f *cornersFounded= new CvPoint2D32f[100];
	int cornersCount=0;
	int result_Found=0;
	// getting snapshots for inner camera calibration from video camera
	bool capture_flag=false;
	while(true)
	{
		frameCam1=cvQueryFrame(captureCam1);
		quarterFrame=cvCreateImage(cvSize((frameCam1->width),(frameCam1->height)),IPL_DEPTH_8U,3);
		
		cvCopy(frameCam1,quarterFrame);
		if(capture_flag)
		{
			result_Found=cvFindChessboardCorners(quarterFrame,cvSize(chess_b_szW,chess_b_szH),cornersFounded,&cornersCount);//,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS |CV_CALIB_CB_FAST_CHECK);
			cvDrawChessboardCorners(quarterFrame,cvSize(chess_b_szW,chess_b_szH),cornersFounded,cornersCount,result_Found);
			if(result_Found>0)
				AddPointsToInnerCalibrate(qu_calibr_points,cornersFounded,cornersCount);
			capture_flag=false;
			cvShowImage("WindowCam1",quarterFrame);
			if(result_Found>0)
				cvWaitKey(0);
		}
		char c=cvWaitKey(33);
		if(c==27)
			break;
		if(c==32 || c=='y' || c=='Y')
			capture_flag=true;
		cvShowImage("WindowCam1",quarterFrame);
		cvReleaseImage(&quarterFrame);
		
	}
	cvReleaseImage(&quarterFrame);
	
	cvReleaseCapture(&captureCam1);
	cvDestroyWindow("WindowCam1");
	
	PrintAllPointsForInnerCalibrate(qu_calibr_points,chess_b_szW*chess_b_szH);
	InitCvMatPointsParametersForInnerCallibration_part1(qu_calibr_points,chess_b_szW*chess_b_szH,object_points_all,image_points_all,points_count_all,chess_b_szW,chess_b_szH);
	InitOtherCameraParametersForInnerCallibration_part2(qu_calibr_points.size()/(chess_b_szW*chess_b_szH),camera_matr,distor_coefs,rotation_vecs,transpose_vecs);
	double calibration_error_result=cvCalibrateCamera2(object_points_all,
												image_points_all,
												points_count_all,
												cvSize(imgW,imgH),
												camera_matr,
												distor_coefs,
												rotation_vecs,
												transpose_vecs,
												CV_CALIB_FIX_PRINCIPAL_POINT|CV_CALIB_FIX_ASPECT_RATIO|CV_CALIB_ZERO_TANGENT_DIST
												);
	WriteMatrixCoef(camera_matr);
	WriteMatrixCoef(distor_coefs);
	cout<<"Total Error:"<<calibration_error_result<<endl;
	cout<<"Average Calibration Error :"<<(calibration_error_result)/qu_calibr_points.size()<<endl;
	IplImage *frame_cur;
	IplImage *undistor_image;
	cvNamedWindow("cameraUndistor",CV_WINDOW_KEEPRATIO);
	CvCapture *captureCam2=cvCreateCameraCapture(0);
	bool undist_flag=false;
	while(true)
	{
		frame_cur= cvQueryFrame(captureCam2);
		undistor_image=cvCreateImage(cvSize((frame_cur->width),(frame_cur->height)),IPL_DEPTH_8U,3);
		if(undist_flag)
		{
			cvUndistort2(frame_cur,undistor_image,camera_matr,distor_coefs);
		}
		else
		{
			cvCopy(frame_cur,undistor_image);
		}
		cvShowImage("cameraUndistor",undistor_image);
		char c=cvWaitKey(33);
		if(c==27)
			break;
		if(c=='u'||c=='U')
			undist_flag=!undist_flag;

		cvReleaseImage(&undistor_image);

	}
	cvReleaseImage(&undistor_image);
	cvReleaseCapture(&captureCam2);
	cvDestroyWindow("cameraUndistor");
	//using SURF
	CvCapture* capture_cam_3=cvCreateCameraCapture(0);
	cvNamedWindow("SURF from Cam",CV_WINDOW_KEEPRATIO);
	cvCreateTrackbar("Hessian Level","SURF from Cam",0,1000,onTrackbarSlide1);
	IplImage* buf_frame_3=0;
	IplImage* gray_copy=0;
	IplImage* buf_frame_3_copy=0;
	
	CvSeq *kp1,*descr1;
	CvMemStorage *storage=cvCreateMemStorage(0);
	
	CvSURFPoint *surf_pt;
	bool surf_flag=false;
	while(true)
	{
		buf_frame_3=cvQueryFrame(capture_cam_3);
		
		if(surf_flag)
		{
			surf_flag=false;
			gray_copy=cvCreateImage(cvSize((buf_frame_3->width),(buf_frame_3->height)),IPL_DEPTH_8U,1);
			buf_frame_3_copy=cvCreateImage(cvSize((buf_frame_3->width),(buf_frame_3->height)),IPL_DEPTH_8U,3);
			
			cvCvtColor(buf_frame_3,gray_copy,CV_RGB2GRAY);
			//cvSetImageROI(gray_copy,cvRect(280,200,40,40));
			cvExtractSURF(gray_copy,NULL,&kp1,&descr1,storage,cvSURFParams(0.0,0));
			cvReleaseImage(&gray_copy);
			re_draw=true;
			
			while(true)
			{
				if(re_draw)
				{
			
					cvCopy(buf_frame_3,buf_frame_3_copy);
					double pi=acos(-1.0);
					for(int i=0;i<kp1->total;i++)
					{
						surf_pt=(CvSURFPoint*)cvGetSeqElem(kp1,i);
						if(surf_pt->hessian<min_hessian)
							continue;
						int pt_x,pt_y;
						pt_x=(int)(surf_pt->pt.x);
						pt_y=(int)(surf_pt->pt.y);
						int sz=surf_pt->size;
						int rad_angle=(surf_pt->dir*pi)/180;
				
						cvCircle(buf_frame_3_copy,cvPoint(pt_x,pt_y),1/*sz*/,CV_RGB(0,255,0));
						cvLine(buf_frame_3_copy,cvPoint(pt_x,pt_y),cvPoint(pt_x+sz*cosl(rad_angle),pt_y-sz*sinl(rad_angle)),CV_RGB(0,0,255));
					}
					cvShowImage("SURF from Cam",buf_frame_3_copy);
					
				}
				char c=cvWaitKey(33);
				if(c==27)
				{
					
					
					break;
				}
			}
			cvReleaseImage(&buf_frame_3_copy);
		}
			
		cvShowImage("SURF from Cam",buf_frame_3);
		char ch=cvWaitKey(33);
		if(ch==27)
			break;
		if(ch==32)
			surf_flag=true;
		
	}
	if(gray_copy!=0)
		cvReleaseImage(&gray_copy);
	cvReleaseCapture(&capture_cam_3);
	cvDestroyWindow("SURF from Cam");
	}

	cvNamedWindow("First Snapshot",CV_WINDOW_KEEPRATIO);
	cvNamedWindow("Second Snapshot",CV_WINDOW_KEEPRATIO);
	cvNamedWindow("twoSnapshots",CV_WINDOW_KEEPRATIO);
	cvCreateTrackbar("Select LLine","twoSnapshots",0,1000,onTrackbarSlideSelectLine);
	CvCapture *capture_4;
	capture_4=cvCreateCameraCapture(0);	
	IplImage* left_img=0;
	IplImage* right_img=0;
	IplImage* cur_frame_buf=0;
	IplImage* gray_img_left=0;
	IplImage* gray_img_right=0;
	IplImage* merged_images=0;
	IplImage* merged_images_copy=0;
	CvSeq *key_points_left;
	CvSeq *descriptors_left; 
	CvSeq *key_points_right;
	CvSeq *descriptors_right;
	CvMemStorage *mem_stor=cvCreateMemStorage(0);
	while(true)
	{
		char ch=cvWaitKey(33);
		if(ch==27)
			break;
		cur_frame_buf=cvQueryFrame(capture_4);
		if(ch=='l'||ch=='L')
		{
			if(left_img==0)
				left_img=cvCreateImage(cvSize(cur_frame_buf->width,cur_frame_buf->height),IPL_DEPTH_8U,3);
			cvCopy(cur_frame_buf,left_img);
		}
		if(ch=='r'||ch=='R')
		{
			if(right_img==0)
				right_img=cvCreateImage(cvSize(cur_frame_buf->width,cur_frame_buf->height),IPL_DEPTH_8U,3);
			cvCopy(cur_frame_buf,right_img);
		}
		if(ch=='b'||ch=='B')
		{
			cvCopy(cur_frame_buf,left_img);
			cvCopy(cur_frame_buf,right_img);
		}
		if(ch=='q'||ch=='Q' && left_img!=0)
		{
			//proceed left
			gray_img_left=cvCreateImage(cvSize((left_img->width),(left_img->height)),IPL_DEPTH_8U,1);
			cvCvtColor(left_img,gray_img_left,CV_RGB2GRAY);
			cvExtractSURF(gray_img_left,NULL,&key_points_left,&descriptors_left,mem_stor,cvSURFParams(50/*101.0*/,0));
			cvReleaseImage(&gray_img_left);
			CvSURFPoint *surf_point_left;
			for(int i=0;i<key_points_left->total;i++)
			{
				surf_point_left=(CvSURFPoint*)cvGetSeqElem(key_points_left,i);
				int _x,_y;
				_x=(int)surf_point_left->pt.x;
				_y=(int)surf_point_left->pt.y;
				cvDrawCircle(left_img,cvPoint(_x,_y),1,CV_RGB(0,255,0));
			}
		}
		if(ch=='w'||ch=='W' && right_img!=0)
		{
			//proceed right
			gray_img_right=cvCreateImage(cvSize((right_img->width),(right_img->height)),IPL_DEPTH_8U,1);
			cvCvtColor(right_img,gray_img_right,CV_RGB2GRAY);
			cvExtractSURF(gray_img_right,NULL,&key_points_right,&descriptors_right,mem_stor,cvSURFParams(50/*101.0*/,0));
			cvReleaseImage(&gray_img_right);
			CvSURFPoint *surf_point_right;
			for(int i=0;i<key_points_right->total;i++)
			{
				surf_point_right=(CvSURFPoint*)cvGetSeqElem(key_points_right,i);
				int _x,_y;
				_x=(int)surf_point_right->pt.x;
				_y=(int)surf_point_right->pt.y;
				cvDrawCircle(right_img,cvPoint(_x,_y),1,CV_RGB(0,255,0));
			}
		}
		if(ch=='m'||ch=='M' && left_img!=0 && right_img!=0)
		{
			//merge two images in to bigger one
			MergeTwoImages(left_img,right_img,merged_images);
		}
		if(ch=='c'||ch=='C' && key_points_left!=0 && key_points_right!=0 && merged_images!=0)
		{
			//comparison of two images
			left_to_right_corresponding_points.clear();
			right_to_left_corresponding_points.clear();
			
			GetCorrespondingPointsForSURF(key_points_left,descriptors_left,key_points_right,descriptors_right,left_to_right_corresponding_points,right_to_left_corresponding_points);
			//drawing lines for corresponding points
			CvSURFPoint *leftPoint,*rightPoint;
			int width_part=merged_images->width>>1;
			/*for(int iL=0;iL<left_to_right_corresponding_points.size();iL++)
			{
				leftPoint=(CvSURFPoint*)cvGetSeqElem(key_points_left,left_to_right_corresponding_points[iL].first);
				rightPoint=(CvSURFPoint*)cvGetSeqElem(key_points_right,left_to_right_corresponding_points[iL].second);
				cvLine(merged_images,cvPoint(leftPoint->pt.x,leftPoint->pt.y),cvPoint(rightPoint->pt.x+width_part,rightPoint->pt.y),CV_RGB(255,0,0));
			}*/
			
			while(true)
			{
				merged_images_copy=cvCreateImage(cvSize(merged_images->width,merged_images->height),merged_images->depth,3);
				cvCopy(merged_images,merged_images_copy);
				int iL=selectedLeftLine;
				if(iL>=left_to_right_corresponding_points.size())
					iL=left_to_right_corresponding_points.size()-1;
				leftPoint=(CvSURFPoint*)cvGetSeqElem(key_points_left,left_to_right_corresponding_points[iL].first);
				rightPoint=(CvSURFPoint*)cvGetSeqElem(key_points_right,left_to_right_corresponding_points[iL].second);
				cvLine(merged_images_copy,cvPoint(leftPoint->pt.x,leftPoint->pt.y),cvPoint(rightPoint->pt.x+width_part,rightPoint->pt.y),CV_RGB(255,0,0));
				cvShowImage("twoSnapshots",merged_images_copy);
				cvReleaseImage(&merged_images_copy);
				char ch2=cvWaitKey(33);
				if(ch2==27)
					break;
			}
		}
		if(left_img!=0)
			cvShowImage("First Snapshot",left_img);
		if(right_img!=0)
			cvShowImage("Second Snapshot",right_img);
		if(merged_images!=0)
		{
			cvShowImage("twoSnapshots",merged_images);
		}
		
	}
	cvReleaseImage(&left_img);
	cvReleaseImage(&right_img);
	cvReleaseCapture(&capture_4);
	cvDestroyWindow("twoSnapshots");
	cvDestroyWindow("First Snapshot");
	cvDestroyWindow("Second Snapshot");
	return 0;
}
float SURFDescriptorsAbsoluteDifference(CvSURFPoint* pt1,float* desc1,CvSURFPoint* pt2,float* desc2,int descr_length)
{
	//предположим что дескриптор сформирован инвариантно к масштабу
	if(pt1->laplacian!=pt2->laplacian)
		return INFINITE;
	float tot_sum=0;
	float buf;
	for(int i=0;i<descr_length;i++)
	{
		buf=desc1[i]-desc2[i];
		if(buf<0)
			buf=-buf;
		tot_sum+=buf;
	}
	return tot_sum;
}
void onTrackbarSlide1(int pos)
{
	min_hessian=pos;
	re_draw=true;
}
void onTrackbarSlideSelectLine(int pos)
{
	selectedLeftLine=pos;
}
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
void GetCorrespondingPointsForSURF(CvSeq* left_seq,
						CvSeq* left_descr,
						CvSeq* right_seq,
						CvSeq* right_descr,
						vector< pair<int,int> > &LeftToRight_corr_pts,
						vector< pair<int,int> > &RightToLeft_corr_pts
						)
{
	int cur_descr_length=64;
	float* best_compare_value_for_right=new float[right_seq->total];
	int* best_index_for_right=new int[right_seq->total];
	float compar_value_buf=0;
	float best_compare_value_left=0;
	int best_index_buf_left=-1;
	CvSURFPoint *left_point_ptr=0,*right_point_ptr=0;
	float *left_point_descriptor=0,*right_point_descriptor=0;
	for(int iL=0;iL<left_seq->total;iL++)
	{
		best_index_buf_left=-1;
		left_point_ptr=(CvSURFPoint*)cvGetSeqElem(left_seq,iL);
		left_point_descriptor=(float*)cvGetSeqElem(left_descr,iL);
		for(int iR=0;iR<right_seq->total;iR++)
		{
			right_point_ptr=(CvSURFPoint*) cvGetSeqElem(right_seq,iR);
			right_point_descriptor=(float*) cvGetSeqElem(right_descr,iR);
			compar_value_buf=SURFDescriptorsAbsoluteDifference(left_point_ptr,left_point_descriptor,right_point_ptr,right_point_descriptor,cur_descr_length);
			if(best_index_buf_left<0)
			{
				best_index_buf_left=iR;
				best_compare_value_left=compar_value_buf;
			}
			if(compar_value_buf<best_compare_value_left)
			{
				best_index_buf_left=iR;
				best_compare_value_left=compar_value_buf;
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
		LeftToRight_corr_pts.push_back(pair<int,int>(iL,best_index_buf_left));
	}
	for(int iR=0;iR<right_seq->total;iR++)
	{
		RightToLeft_corr_pts.push_back(pair<int,int>(iR,best_index_for_right[iR]));
	}
	delete[] best_index_for_right;
	delete[] best_compare_value_for_right;
}