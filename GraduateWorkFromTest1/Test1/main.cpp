#include "InitAndIOFuncs.h"
#include "SupportFunctions.h"
#include "checkLMFIT.h"

int chess_b_szW=7,chess_b_szH=7;//size's of inner angles at chessboard
int imgH=480,imgW=640;
int selectedLeftLine=0;
float min_hessian=0.0f;
bool re_draw=false;
void onTrackbarSlide1(int pos);
void onTrackbarSlideSelectLine(int pos);



vector< ComparedIndexes > left_to_right_corresponding_points;
vector< ComparedIndexes > right_to_left_corresponding_points;




void filterPointsByDistanceFromEpipolarLines(CvMat* points1,
											 CvMat* points2,
											 int count_of_points,
											 CvMat* fundamentalMatrix,
											 double max_ro,
											 CvMat* &outPoints1,
											 CvMat* &outPoints2,
											 int &count_of_out_points
											 );





bool run_tests_only = false;



int main()
{

	if(run_tests_only)
	{
		MyLine3D::runTest();
		return 0;
	}

	//CvMat *camera_inner_calibration_matrix; 
	bool show_surf_example=false;
	bool show_calibration_from_camera_and_undistortion=false;
	if(show_calibration_from_camera_and_undistortion)
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
		//camera_inner_calibration_matrix=cvCreateMat(3,3,CV_32FC1);
		//cvCopy(camera_matr,camera_inner_calibration_matrix);
		cvSave("camera_calibration_inner.txt",camera_matr,"camera_inner_calibration_matrix");
		cvSave("camera_calibration_dist.txt",distor_coefs,"distor_coefs","coeficients of distortions");
		cout<<"Total Error:"<<calibration_error_result<<endl;
		cout<<"Average Calibration Error :"<<(calibration_error_result)/qu_calibr_points.size()<<endl;
	//undistortion example
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
	}//ending undistortion_example
	
	if(show_surf_example)
	{
		//using SURF
		
		initModule_nonfree();// added at 16.04.2013
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
	}//ending SURF_example

	CvFont my_font=cvFont(1,1);
	cvInitFont(&my_font,CV_FONT_HERSHEY_SIMPLEX,1.0,1.0);

	cvNamedWindow("twoSnapshots",CV_WINDOW_KEEPRATIO);
	cvCreateTrackbar("Select LLine","twoSnapshots",0,1000,onTrackbarSlideSelectLine);
	CvCapture *capture_4 = 0;
	
	IplImage* left_img=0;
	IplImage* right_img=0;
	IplImage* cur_frame_buf=0;
	IplImage* gray_img_left=0;
	IplImage* gray_img_right=0;
	IplImage* merged_images=0;
	IplImage* merged_images_copy=0;
	CvMat *fundamentalMatrix = 0;
	vector<KeyPoint> key_points_left;
	Mat descriptors_left; 
	vector<KeyPoint> key_points_right;
	Mat descriptors_right;
	//CvMemStorage *mem_stor=cvCreateMemStorage(0);*/
	float min_hessian_value=1001.0f;

	char* left_image_file_path = "camera_picture_left.png";
	char* right_image_file_path = "camera_picture_right.png";

	Array left_points, right_points;
	left_points.init(1,1);
	right_points.init(1,1);
	Array forReconstructionLeftPoints, forReconstructionRightPoints;
	forReconstructionLeftPoints.init(1,1);
	forReconstructionRightPoints.init(1,1);

	

	while(true)
	{
		char ch=cvWaitKey(33);
		if(ch==27)
			break;
		// open left and right images
		if(ch == 'o' || ch == 'O')
		{
			openTwoImages(left_image_file_path, right_image_file_path, left_img, right_img );
			MergeTwoImages(left_img,right_img,merged_images);
		}
		// save both left and right images from camera
		if(ch == 's' || ch == 'S')
		{
			if( left_img != 0 )
				cvSaveImage(left_image_file_path, left_img);
			if( right_img != 0)
				cvSaveImage(right_image_file_path, right_img);
		}

		if(ch=='l'||ch=='L')
		{
			if(capture_4 == 0)
			{
				capture_4=cvCreateCameraCapture(0);	
			}
			
			cur_frame_buf=cvQueryFrame(capture_4);
			if(left_img==0)
				left_img=cvCreateImage(cvSize(cur_frame_buf->width,cur_frame_buf->height),IPL_DEPTH_8U,3);
			cvCopy(cur_frame_buf,left_img);

			if(right_img == 0)
			{
				right_img=cvCreateImage(cvSize(cur_frame_buf->width,cur_frame_buf->height),IPL_DEPTH_8U,3);
				cvCopy(cur_frame_buf,right_img);
			}

			MergeTwoImages(left_img,right_img,merged_images);
		}
		if(ch=='r'||ch=='R')
		{
			if(capture_4 == 0)
			{
				capture_4=cvCreateCameraCapture(0);	
			}
			cur_frame_buf=cvQueryFrame(capture_4);
			if(right_img==0)
				right_img=cvCreateImage(cvSize(cur_frame_buf->width,cur_frame_buf->height),IPL_DEPTH_8U,3);
			cvCopy(cur_frame_buf,right_img);

			if(left_img == 0)
			{
				left_img=cvCreateImage(cvSize(cur_frame_buf->width,cur_frame_buf->height),IPL_DEPTH_8U,3);
				cvCopy(cur_frame_buf,left_img);
			}
			MergeTwoImages(left_img,right_img,merged_images);
		}
		if(ch=='b'||ch=='B')
		{
			if(capture_4 == 0)
			{
				capture_4=cvCreateCameraCapture(0);	
			}
			cur_frame_buf=cvQueryFrame(capture_4);
			cvCopy(cur_frame_buf,left_img);
			cvCopy(cur_frame_buf,right_img);
		}
		if(ch=='q'||ch=='Q' && left_img!=0)
		{
			//proceed left
			extractFeaturesFromImage(left_img, min_hessian_value, gray_img_left, key_points_left, descriptors_left);

		}
		if(ch=='w'||ch=='W' && right_img!=0)
		{
			//proceed right
			extractFeaturesFromImage(right_img, min_hessian_value, gray_img_right, key_points_right, descriptors_right);			

		}
		if(ch=='m'||ch=='M' && left_img!=0 && right_img!=0)
		{
			//merge two images in to bigger one
			MergeTwoImages(left_img,right_img,merged_images);
		}
		if(ch=='c'||ch=='C' && merged_images!=0)
		{
			//comparison of two images
			if(fundamentalMatrix != 0)
			{
				cvReleaseMat(& fundamentalMatrix);
				fundamentalMatrix = 0;
			}
			left_to_right_corresponding_points.clear();
			right_to_left_corresponding_points.clear();
			
			GetCorrespondingPointsForSURF(key_points_left,descriptors_left,key_points_right,descriptors_right,left_to_right_corresponding_points,right_to_left_corresponding_points);
		}

		if(ch == 'E' || ch == 'e')
		{
			//drawing lines for corresponding points
			KeyPoint *leftPoint,*rightPoint,*leftPoint2,*rightPoint2;
			int width_part=merged_images->width>>1;
			/*for(int iL=0;iL<left_to_right_corresponding_points.size();iL++)
			{
				leftPoint=(CvSURFPoint*)cvGetSeqElem(key_points_left,left_to_right_corresponding_points[iL].first);
				rightPoint=(CvSURFPoint*)cvGetSeqElem(key_points_right,left_to_right_corresponding_points[iL].second);
				cvLine(merged_images,cvPoint(leftPoint->pt.x,leftPoint->pt.y),cvPoint(rightPoint->pt.x+width_part,rightPoint->pt.y),CV_RGB(255,0,0));
			}*/
			
			int sizeOfAccepptedLeftToRightCorrespondings = left_to_right_corresponding_points.size();
			bool* acceptedLeftToRightCorrespondings = 0;
			getAcceptedCorrespondingsForFindingModelParameters(left_to_right_corresponding_points,
				key_points_left,
				key_points_right,
				fundamentalMatrix,
				acceptedLeftToRightCorrespondings,
				sizeOfAccepptedLeftToRightCorrespondings);

			
			while(true)
			{
				merged_images_copy=cvCreateImage(cvSize(merged_images->width,merged_images->height),merged_images->depth,3);
				cvCopy(merged_images,merged_images_copy);
				int iL=selectedLeftLine;
				int iR=iL;
				if(iL>=left_to_right_corresponding_points.size())
					iL=left_to_right_corresponding_points.size()-1;
				if(iR>=right_to_left_corresponding_points.size())
					iR=right_to_left_corresponding_points.size()-1;
				char str[100]={0};
				if(iL >= 0 )
				{
					bool isLeftToRightLineIsAccepted = acceptedLeftToRightCorrespondings[iL];
				
					// difference value
					sprintf(str,"%f",left_to_right_corresponding_points[iL].comparer_value);
					cvPutText(merged_images_copy,str,cvPoint(0,merged_images_copy->height-40),&my_font,CV_RGB(0,255,0));
					// count of Matches
					sprintf(str,"%d",left_to_right_corresponding_points[iL].counterOfMatches);
					cvPutText(merged_images_copy,str,cvPoint(200,merged_images_copy->height-40),&my_font,CV_RGB(255,255,0));
					// median of compared values
					sprintf(str,"%lf",left_to_right_corresponding_points[iL].medianOfComparedMatches);
					cvPutText(merged_images_copy,str,cvPoint(250,merged_images_copy->height-40),&my_font,CV_RGB(255,0,0));

					// Variance of compared values
					sprintf(str,"V=%lf",left_to_right_corresponding_points[iL].Variance());
					cvPutText(merged_images_copy,str,cvPoint(0,merged_images_copy->height-80),&my_font,CV_RGB(0,255,0));

					// Standard deviation of compared values
					sprintf(str,"SD=%lf",sqrt( left_to_right_corresponding_points[iL].Variance() ));
					cvPutText(merged_images_copy,str,cvPoint(250,merged_images_copy->height-80),&my_font,CV_RGB(0,255,0));

					double SD = sqrt( left_to_right_corresponding_points[iL].Variance() ) ;
					double median = left_to_right_corresponding_points[iL].medianOfComparedMatches;
					double compValue = left_to_right_corresponding_points[iL].comparer_value;
					double mark_1_5 = median - 1.5 * SD - compValue;

					// Mark 1.5
					sprintf(str,"m1.5=%lf", mark_1_5);
					cvPutText(merged_images_copy,str,cvPoint(0,merged_images_copy->height-120),&my_font,CV_RGB(0,255,0));

					sprintf(str,"angle=%lf", left_to_right_corresponding_points[iL].degreesBetweenDeltaVector);
					cvPutText(merged_images_copy,str,cvPoint(0,merged_images_copy->height-150),&my_font,CV_RGB(0,255,0));

					

					leftPoint= &(key_points_left[ left_to_right_corresponding_points[iL].comp_pair.first ]);
					rightPoint=&(key_points_right[ left_to_right_corresponding_points[iL].comp_pair.second ]);
				
					cvLine(merged_images_copy,cvPoint(leftPoint->pt.x,leftPoint->pt.y),cvPoint(rightPoint->pt.x+width_part,rightPoint->pt.y),CV_RGB(0,255,0));

					drawEpipolarLinesOnLeftAndRightImages(merged_images_copy, cvPoint(leftPoint->pt.x,leftPoint->pt.y),
						cvPoint(rightPoint->pt.x,rightPoint->pt.y), fundamentalMatrix);

					CvScalar color = CV_RGB(255, 0, 0);
					if(isLeftToRightLineIsAccepted)
					{
						color = CV_RGB(0,255,0);
					}

					cvCircle(merged_images_copy, cvPoint(leftPoint->pt.x,leftPoint->pt.y), 5, color);
					cvCircle(merged_images_copy, cvPoint(rightPoint->pt.x+width_part,rightPoint->pt.y), 5, color);
				}
				//cvLine(merged_images_copy,cvPoint(leftPoint->pt.x,leftPoint->pt.y),cvPoint(rightPoint->pt.x,rightPoint->pt.y),CV_RGB(255,0,255));
				if(iR >= 0 )
				{
					sprintf(str,"%f",right_to_left_corresponding_points[iR].comparer_value);
					cvPutText(merged_images_copy,str,cvPoint(width_part,merged_images_copy->height-40),&my_font,CV_RGB(255,0,0));
					rightPoint2= &(key_points_right [right_to_left_corresponding_points[iR].comp_pair.first]);
					leftPoint2= &(key_points_left [right_to_left_corresponding_points[iR].comp_pair.second]);
					cvLine(merged_images_copy,cvPoint(leftPoint2->pt.x,leftPoint2->pt.y),cvPoint(rightPoint2->pt.x+width_part,rightPoint2->pt.y),CV_RGB(255,0,0));
				}
				//cvLine(merged_images_copy,cvPoint(leftPoint2->pt.x+width_part,leftPoint2->pt.y),cvPoint(rightPoint2->pt.x+width_part,rightPoint2->pt.y),CV_RGB(255,0,255));
				
				cvShowImage("twoSnapshots",merged_images_copy);
				cvReleaseImage(&merged_images_copy);
				char ch2=cvWaitKey(33);
				if(ch2==27)
					break;
				if(ch2=='z' && selectedLeftLine>0)
				{
					selectedLeftLine--;
				}
				if(ch2=='x' && selectedLeftLine<1000)
				{
					selectedLeftLine++;
				}
				if( ch2 == 'a' || ch2 == 'A')
				{
					acceptedLeftToRightCorrespondings[selectedLeftLine] = true;
				}
				if( ch2 == 'd' || ch2 == 'D')
				{
					acceptedLeftToRightCorrespondings[selectedLeftLine] = false;
				}
			}//end of while(true)

			SaveAcceptedCorresspondings(
					left_to_right_corresponding_points,
					right_to_left_corresponding_points,
					key_points_left,
					key_points_right,
					acceptedLeftToRightCorrespondings,
					sizeOfAccepptedLeftToRightCorrespondings
					);
			ConvertAcceptedCorresspondingsToMyArray(left_to_right_corresponding_points,
					right_to_left_corresponding_points,
					key_points_left,
					key_points_right,
					acceptedLeftToRightCorrespondings,
					sizeOfAccepptedLeftToRightCorrespondings,
					left_points,
					right_points
					);


			delete[] acceptedLeftToRightCorrespondings;
		}
		if( ch == 'T' || ch == 't')
		{
			clock_t startTime = clock();

			openTwoImages(left_image_file_path, right_image_file_path, left_img, right_img );
			// proceed left
			extractFeaturesFromImage(left_img, min_hessian_value, gray_img_left, key_points_left, descriptors_left);
			//proceed right
			extractFeaturesFromImage(right_img, min_hessian_value, gray_img_right, key_points_right, descriptors_right);	
			//comparison of two images
			if(fundamentalMatrix != 0)
			{
				cvReleaseMat(& fundamentalMatrix);
				fundamentalMatrix = 0;
			}
			left_to_right_corresponding_points.clear();
			right_to_left_corresponding_points.clear();
			
			GetCorrespondingPointsForSURF(key_points_left,descriptors_left,key_points_right,descriptors_right,left_to_right_corresponding_points,right_to_left_corresponding_points);

			// searching fundamental matrix and corresponding points
			findFundamentalMatrixAndCorrespondingPointsForReconstruction(
				left_to_right_corresponding_points,
				right_to_left_corresponding_points,
				fundamentalMatrix,
				key_points_left,
				key_points_right,
				descriptors_left,
				descriptors_right,
				left_img,
				right_img,
				gray_img_left,
				gray_img_right,
				forReconstructionLeftPoints,
				forReconstructionRightPoints,
				min_hessian_value);
			// selecting points for finding model parameters

			int sizeOfAccepptedLeftToRightCorrespondings = left_to_right_corresponding_points.size();
			bool* acceptedLeftToRightCorrespondings = 0;
			getAcceptedCorrespondingsForFindingModelParameters(left_to_right_corresponding_points,
				key_points_left,
				key_points_right,
				fundamentalMatrix,
				acceptedLeftToRightCorrespondings,
				sizeOfAccepptedLeftToRightCorrespondings);

			ConvertAcceptedCorresspondingsToMyArray(left_to_right_corresponding_points,
					right_to_left_corresponding_points,
					key_points_left,
					key_points_right,
					acceptedLeftToRightCorrespondings,
					sizeOfAccepptedLeftToRightCorrespondings,
					left_points,
					right_points
					);

			delete[] acceptedLeftToRightCorrespondings;

			// start process of determination parameters of model and reconstruction of scene
			cv::Mat mat_left_img(left_img, true);
			cv::Mat mat_right_img(right_img, true);
			mainLevenbergMarkvardt_LMFIT(100, "currentPLYExportFile", left_points, right_points, 
				mat_left_img, mat_right_img,
				forReconstructionLeftPoints, forReconstructionRightPoints);
			mat_left_img.release();
			mat_right_img.release();


			cout << "Code execution time: "<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." << endl;
		}
		if( ch == 'I' || ch == 'i')
		{	

			//-- Step 3: Matching descriptor vectors using FLANN matcher
			FlannBasedMatcher matcher;
			std::vector< DMatch > matches;
			matcher.match( descriptors_left, descriptors_right, matches );

			//double max_dist = 0; double min_dist = 100;

			////-- Quick calculation of max and min distances between keypoints
			//for( int i = 0; i < descriptors_left.rows; i++ )
			//{ double dist = matches[i].distance;
			//	if( dist < min_dist ) min_dist = dist;
			//	if( dist > max_dist ) max_dist = dist;
			//}

			//printf("-- Max dist : %f \n", max_dist );
			//printf("-- Min dist : %f \n", min_dist );

			//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
			//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
			//-- small)
			//-- PS.- radiusMatch can also be used here.
			//std::vector< DMatch > good_matches;
			
			left_to_right_corresponding_points.clear();
			right_to_left_corresponding_points.clear();

			for( int i = 0; i < descriptors_left.rows; i++ )
			{ 
				//if( matches[i].distance <= max(2*min_dist, 0.02) )
				{
					//good_matches.push_back( matches[i]); 
					left_to_right_corresponding_points.push_back( ComparedIndexes(matches[i].distance, pair<int, int> (i, matches[i].trainIdx)) );
				}
			}
			
			cout<< "Count of good matches :" << left_to_right_corresponding_points.size() << endl;

			stable_sort(left_to_right_corresponding_points.begin(),left_to_right_corresponding_points.end(),my_comparator_for_stable_sort);
		}

		//if( ch == 'K' || ch == 'k')
		//{
		//	CvSURFPoint *leftPoint;
		//	//proceed left
		//	gray_img_left=cvCreateImage(cvSize((left_img->width),(left_img->height)),IPL_DEPTH_8U,1);
		//	cvCvtColor(left_img,gray_img_left,CV_RGB2GRAY);
		//	cvExtractSURF(gray_img_left,NULL,&key_points_left,&descriptors_left,mem_stor,cvSURFParams(min_hessian_value,0));

		//	cv::Mat mat_gray_leftImage(gray_img_left, true);
		//	cvReleaseImage(&gray_img_left);
		//	// proceed right
		//	gray_img_right=cvCreateImage(cvSize((right_img->width),(right_img->height)),IPL_DEPTH_8U,1);
		//	cvCvtColor(right_img,gray_img_right,CV_RGB2GRAY);
		//	cv::Mat mat_gray_rightImage(gray_img_right, true);
		//	cvReleaseImage(&gray_img_right);
		//	vector<Point2f> LK_left_points;
		//	vector<Point2f> LK_right_points;

		//	LK_right_points.resize(key_points_left->total);

		//	for( int i = 0; i < key_points_left->total; i++)
		//	{
		//		leftPoint=(CvSURFPoint*)cvGetSeqElem(key_points_left, i);
		//		LK_left_points.push_back(Point2f( leftPoint->pt.x, leftPoint->pt.y));
		//	}
		//	
		//	vector<uchar> status;
  //          vector<float> err;

		//	cv::calcOpticalFlowPyrLK(
		//		mat_gray_leftImage,
		//		mat_gray_rightImage, 
		//		LK_left_points,
		//		LK_right_points, 
		//		status,
		//		err);
		//	int width_part=merged_images->width>>1;
		//	
		//	float minErr = err[0];

		//	for(int k = 0; k < err.size(); k++)
		//	{
		//		if(status[k] && err[k] < minErr) 
		//		{
		//			minErr = err[k];
		//		}
		//	}

		//	cout<< "Lucass Kanade min error: " << minErr<< endl;

		//	int i = 0;
		//	merged_images_copy=cvCreateImage(cvSize(merged_images->width,merged_images->height),merged_images->depth,3);
		//	cvCopy(merged_images,merged_images_copy);
		//	for(; i < LK_left_points.size(); ++i)
		//	{
		//		if(err[i] < 5 * minErr && status[i])
		//		{
		//			cvLine(merged_images_copy,cvPoint(LK_left_points[i].x,LK_left_points[i].y),cvPoint(LK_right_points[i].x+width_part,LK_right_points[i].y),
		//					CV_RGB(100 + (( i *3) % 155), 100+ ((i*7)%155), 100+ ((i*13)%155)));
		//		}
		//	}

		//	cvShowImage("twoSnapshots",merged_images_copy);
		//		
		//	while(true)
		//	{

		//		char ch2=cvWaitKey(33);
		//		if(ch2==27)
		//			break;
		//		
		//	}
		//	
		//	cvReleaseImage(&merged_images_copy);

		//	status.clear();
		//	err.clear();
		//	LK_left_points.clear();
		//	LK_right_points.clear();
		//	mat_gray_leftImage.release();
		//	mat_gray_rightImage.release();
		//}

		if( ch == 'F' || ch == 'f')
		{
			findFundamentalMatrixAndCorrespondingPointsForReconstruction(
				left_to_right_corresponding_points,
				right_to_left_corresponding_points,
				fundamentalMatrix,
				key_points_left,
				key_points_right,
				descriptors_left,
				descriptors_right,
				left_img,
				right_img,
				gray_img_left,
				gray_img_right,
				forReconstructionLeftPoints,
				forReconstructionRightPoints,
				min_hessian_value);


		}
		if( ch == 'P' || ch == 'p')
		{
			cv::Mat mat_left_img(left_img, true);
			cv::Mat mat_right_img(right_img, true);
			mainLevenbergMarkvardt_LMFIT(350, "currentPLYExportFile", left_points, right_points, 
				mat_left_img, mat_right_img,
				forReconstructionLeftPoints, forReconstructionRightPoints);
			mat_left_img.release();
			mat_right_img.release();
		}
		if(merged_images!=0)
		{
			cvShowImage("twoSnapshots",merged_images);
		}
		
	}
	cvReleaseImage(&left_img);
	cvReleaseImage(&right_img);
	if( capture_4 != 0)
	{
		cvReleaseCapture(&capture_4);
	}
	
	cvDestroyWindow("twoSnapshots");

	
	
	//finding fundamental matrix
	int count= myMin(30, left_to_right_corresponding_points.size());

	cout<< "Count of point to process = " << count << endl;

	if( fundamentalMatrix != 0)
	{
		cvReleaseMat(& fundamentalMatrix);
		fundamentalMatrix = 0;
	}

	CvMat *points1,*points2;
	PreparePointsForFindingFundamentalMatrix(points1,points2,fundamentalMatrix,
		left_to_right_corresponding_points,
		right_to_left_corresponding_points,
		count,
		key_points_left,
		key_points_right);
	int matrix_count=-1;
	matrix_count=cvFindFundamentalMat(points1,
		points2,fundamentalMatrix,
		CV_FM_RANSAC,3.0);
	if(matrix_count>1)
		cout<<endl<<"There is more than one fundamental matrix!!!"<<endl;
	if(matrix_count==0)
		cout<<endl<<"No Fundamental matrix was find!!!"<<endl;
	if(matrix_count==1)
		cout<<"Fundamental matrix succesfully found:"<<endl;

	WriteMatrixCoef(fundamentalMatrix);

	bool need_filtration = true;
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

	
	CvMat *U,*W,*V,*essential_matrix;
	essential_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *inner_calib_matr_loaded;
	CvMat *trasposed_inner_calib_matr=cvCreateMat(3,3,CV_32FC1);
	inner_calib_matr_loaded=(CvMat*)cvLoad("camera_calibration_inner.txt");

	cout<<"Inner calibration matrix:"<<endl;
	WriteMatrixCoef(inner_calib_matr_loaded);

	cvTranspose(inner_calib_matr_loaded,trasposed_inner_calib_matr);
	CvMat* buffer_matr=cvCreateMat(3,3,CV_32FC1);
	CvMat* buffer_matr_Vt=cvCreateMat(3,3,CV_32FC1);
	CvMat* buffer_essential_matr=cvCreateMat(3,3,CV_32FC1);
	cvMatMul(trasposed_inner_calib_matr,fundamentalMatrix,buffer_matr);
	cvMatMul(buffer_matr,inner_calib_matr_loaded,essential_matrix);//Founded essential matrix
	
	cout<<endl<<"Essential matrix: "<<endl;
	WriteMatrixCoef(essential_matrix);
	
	//SVD of essential_matrix
	U=cvCreateMat(3,3,CV_32FC1);
	W=cvCreateMat(3,3,CV_32FC1);
	V=cvCreateMat(3,3,CV_32FC1);
	//comment : essential_matrix=U*W*(V')
	cvSVD(essential_matrix,W,U,V);

	cout<<"U:"<<endl;
	WriteMatrixCoef(U);

	cout<<"V:"<<endl;
	WriteMatrixCoef(V);

	cout<<"W:"<<endl;
	WriteMatrixCoef(W);

	cvTranspose(V,buffer_matr_Vt);
	cvMatMul(U,W,buffer_matr);
	cout<<"Buffer_matr:"<<endl;
	WriteMatrixCoef(buffer_matr);

	cvMatMul(buffer_matr,buffer_matr_Vt,buffer_essential_matr);
	
	cout<<"Restored Essential Matrix From SVD:"<<endl;
	WriteMatrixCoef(buffer_essential_matr);


	CvMat *S,*R,*E,*Z;
	CvMat *R_v2,*Et;
	S = cvCreateMat(3,3,CV_32FC1);
	R = cvCreateMat(3,3,CV_32FC1);
	R_v2 = cvCreateMat(3,3,CV_32FC1);

	E = cvCreateMat(3,3,CV_32FC1);
	Et = cvCreateMat(3,3,CV_32FC1);
	Z = cvCreateMat(3,3,CV_32FC1);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			E->data.fl[i*3+j]=0.0f;
			Z->data.fl[i*3+j]=0.0f;
		}
		Z->data.fl[0*3+1]=-1.0f;
		Z->data.fl[1*3+0]=1.0f;
		E->data.fl[0*3+1]=1.0f;
		E->data.fl[1*3+0]=-1.0f;
		E->data.fl[2*3+2]=1.0f;
	cout<<"Z:"<<endl;
	WriteMatrixCoef(Z);
	cout<<"E:"<<endl;
	WriteMatrixCoef(E);
	CvMat *Vt=cvCreateMat(3,3,CV_32FC1);
	CvMat *BufMatr=cvCreateMat(3,3,CV_32FC1); 
	cvTranspose(V,Vt);

	cvTranspose(E,Et);

	cvMatMul(V,Z,S);
	cvMatMul(S,Vt,BufMatr);
	cvCopy(BufMatr,S);//Found Shift Matrix; EssentialMatrix=R*S
	
	cout<<"S:"<<endl;
	WriteMatrixCoef(S);

	CvMat * translation_vector = getTranslationVectorFromMatrixTx(S);

	

	cvMatMul(U,E,R);
	cvMatMul(R,Vt,BufMatr);
	cvCopy(BufMatr,R);

	cvMatMul(U, Et, R_v2);
	cvMatMul(R_v2, Vt, BufMatr);
	cvCopy(BufMatr, R_v2);

		
	cout<<"R:"<<endl;
	WriteMatrixCoef(R);

	cout<<"R version 2:"<<endl;
	WriteMatrixCoef(R_v2);

	CvMat* translation_vector_2 = cvCreateMat(3, 1, CV_32FC1);
	cvCopy(translation_vector, translation_vector_2);

	multiplicateByMinusOne(translation_vector_2);

	cout<<"Translation vector v1:"<<endl;

	WriteMatrixCoef(translation_vector);

	
	cout<<"Translation vector v2:"<<endl;

	WriteMatrixCoef(translation_vector_2);




	

	CvMat *times_1, *times_2, *times_3, *times_4;

	int good, total;

	int current_good = 0;
	int current_index = -1;

	times_1 = cvCreateMat(count, 2, CV_64FC1);

	getIntersections( 
		inner_calib_matr_loaded,
		R,
		translation_vector,
		points1,
		points2,
		times_1);


	analyzeIntersectionTimes(times_1, good, total);
	
	cout << " Combination " << 1 << "(R_v1, t_1) Result =" << good << " / " << total << endl;
	if(good > current_good)
	{
		current_good = good;
		current_index = 1;
	}
	cout.flush();

	times_2 = cvCreateMat(count, 2, CV_64FC1);

	getIntersections( 
		inner_calib_matr_loaded,
		R,
		translation_vector_2,
		points1,
		points2,
		times_2);

	
	analyzeIntersectionTimes(times_2, good, total);
	
	cout << " Combination " << 2 << "(R_v1, t_2) Result =" << good << " / " << total << endl;

	if(good > current_good)
	{
		current_good = good;
		current_index = 2;
	}
	cout.flush();

	times_3 = cvCreateMat(count, 2, CV_64FC1);

	getIntersections( 
		inner_calib_matr_loaded,
		R_v2,
		translation_vector,
		points1,
		points2,
		times_3);

	
	analyzeIntersectionTimes(times_3, good, total);
	
	cout << " Combination " << 3 << "(R_v2, t_1) Result =" << good << " / " << total << endl;

	if(good > current_good)
	{
		current_good = good;
		current_index = 3;
	}
	cout.flush();

	times_4 = cvCreateMat(count, 2, CV_64FC1);

	getIntersections( 
		inner_calib_matr_loaded,
		R_v2,
		translation_vector_2,
		points1,
		points2,
		times_4);

	
	analyzeIntersectionTimes(times_4, good, total);
	
	cout << " Combination " << 4 << " (R_v2, t_2) Result =" << good << " / " << total << endl;

	if(good > current_good)
	{
		current_good = good;
		current_index = 4;
	}
	cout.flush();

	cout<< "Current selected variant is :" << current_index << endl;

	CvMat* middle_points = cvCreateMat(count, 4, CV_64FC1);

	if(current_index == 1)
	{
		cvSave("currentInnerCalibrationMatrix.txt", inner_calib_matr_loaded, "inner_callibration_matrix");
		cvSave("currentRotationMatrix.txt", R , "current_rotation_matrix");
		cvSave("currentTransposeVector.txt", translation_vector , "current_transpose_vector");

		getMiddleIntersectionPoints(inner_calib_matr_loaded, R, translation_vector, points1, points2, times_1, middle_points);
		cvSave("currentMiddlePoints.txt", middle_points , "current_middle_points");
	}
	if(current_index == 2)
	{
		cvSave("currentInnerCalibrationMatrix.txt", inner_calib_matr_loaded, "inner_callibration_matrix");
		cvSave("currentRotationMatrix.txt", R , "current_rotation_matrix");
		cvSave("currentTransposeVector.txt", translation_vector_2 , "current_transpose_vector");

		getMiddleIntersectionPoints(inner_calib_matr_loaded, R, translation_vector_2, points1, points2, times_2, middle_points);
		cvSave("currentMiddlePoints.txt", middle_points , "current_middle_points");
	}
	if(current_index == 3)
	{	
		cvSave("currentInnerCalibrationMatrix.txt", inner_calib_matr_loaded, "inner_callibration_matrix");
		cvSave("currentRotationMatrix.txt", R_v2 , "current_rotation_matrix");
		cvSave("currentTransposeVector.txt", translation_vector , "current_transpose_vector");

		getMiddleIntersectionPoints(inner_calib_matr_loaded, R_v2, translation_vector, points1, points2, times_3, middle_points);
		cvSave("currentMiddlePoints.txt", middle_points , "current_middle_points");
	}
	if(current_index == 4)
	{
		cvSave("currentInnerCalibrationMatrix.txt", inner_calib_matr_loaded, "inner_callibration_matrix");
		cvSave("currentRotationMatrix.txt", R_v2 , "current_rotation_matrix");
		cvSave("currentTransposeVector.txt", translation_vector_2 , "current_transpose_vector");

		getMiddleIntersectionPoints(inner_calib_matr_loaded, R_v2, translation_vector_2, points1, points2, times_4, middle_points);
		cvSave("currentMiddlePoints.txt", middle_points , "current_middle_points");
	}





	// another variant

	current_good = 0;
	current_index = -1;

	CvMat* vector001 = get001Vector();
	CvMat* additional_column = cvCreateMat(3, 1, CV_32FC1);
	// U * (0, 0, 1)'
	cvMatMul(U, vector001, additional_column);

	CvMat* minus_additional_column = cvCreateMat(3, 1, CV_32FC1);

	cvCopy(additional_column, minus_additional_column);

	multiplicateByMinusOne(minus_additional_column);


	CvMat *matrix_1, *matrix_2, *matrix_3, *matrix_4;
	matrix_1 = combineMatrixAndVector(R, additional_column);
	matrix_2 = combineMatrixAndVector(R, minus_additional_column);
	matrix_3 = combineMatrixAndVector(R_v2, additional_column);
	matrix_4 = combineMatrixAndVector(R_v2, minus_additional_column);

	cout<< endl << "Another variant:" << endl;

	cout<< "Additional column :" << endl;
	WriteMatrixCoef(additional_column);

	cout<< "Minus additional column:" << endl;
	WriteMatrixCoef(minus_additional_column);

	cout<< "Roation matrix 1:" << endl;
	WriteMatrixCoef(R);

	cout<< "Rotation matrix 2:" << endl;
	WriteMatrixCoef(R_v2);

	cout<< "Matrix 1:" <<endl;
	WriteMatrixCoef(matrix_1);

	cout<< "Matrix 2:" <<endl;
	WriteMatrixCoef(matrix_2);

	cout<< "Matrix 3:" <<endl;
	WriteMatrixCoef(matrix_3);

	cout<< "Matrix 4:" <<endl;
	WriteMatrixCoef(matrix_4);

	getIntersections( 
		inner_calib_matr_loaded,
		matrix_1,
		points1,
		points2,
		times_1);


	analyzeIntersectionTimes(times_1, good, total);
	
	cout << " Combination " << 1 << "(Matrix 1) Result =" << good << " / " << total << endl;
	if(good > current_good)
	{
		current_good = good;
		current_index = 1;
	}
	cout.flush();

	

	getIntersections( 
		inner_calib_matr_loaded,
		matrix_2,
		points1,
		points2,
		times_2);

	
	analyzeIntersectionTimes(times_2, good, total);
	
	cout << " Combination " << 2 << "(Matrix 2) Result =" << good << " / " << total << endl;

	if(good > current_good)
	{
		current_good = good;
		current_index = 2;
	}
	cout.flush();

	

	getIntersections( 
		inner_calib_matr_loaded,
		matrix_3,
		points1,
		points2,
		times_3);

	
	analyzeIntersectionTimes(times_3, good, total);
	
	cout << " Combination " << 3 << "(Matrix 3) Result =" << good << " / " << total << endl;

	if(good > current_good)
	{
		current_good = good;
		current_index = 3;
	}
	cout.flush();

	

	getIntersections( 
		inner_calib_matr_loaded,
		matrix_4,
		points1,
		points2,
		times_4);

	
	analyzeIntersectionTimes(times_4, good, total);
	
	cout << " Combination " << 4 << " (Matrix 4) Result =" << good << " / " << total << endl;

	if(good > current_good)
	{
		current_good = good;
		current_index = 4;
	}
	cout.flush();

	cout<< "Current selected variant is :" << current_index << endl;

	

	if(current_index == 1)
	{
		
		cvSave("currentMatrix.txt", matrix_1 , "current_rotation_matrix");
		

		getMiddleIntersectionPoints(inner_calib_matr_loaded, matrix_1, points1, points2, times_1, middle_points);
		cvSave("currentMiddlePoints_next.txt", middle_points , "current_middle_points");
	}
	if(current_index == 2)
	{
		cvSave("currentMatrix.txt", matrix_2 , "current_rotation_matrix");
		

		getMiddleIntersectionPoints(inner_calib_matr_loaded, matrix_2, points1, points2, times_2, middle_points);
		cvSave("currentMiddlePoints_next.txt", middle_points , "current_middle_points");
	}
	if(current_index == 3)
	{	
		cvSave("currentMatrix.txt", matrix_3 , "current_rotation_matrix");
		

		getMiddleIntersectionPoints(inner_calib_matr_loaded, matrix_3, points1, points2, times_3, middle_points);
		cvSave("currentMiddlePoints_next.txt", middle_points , "current_middle_points");
	}
	if(current_index == 4)
	{
		cvSave("currentMatrix.txt", matrix_4 , "current_rotation_matrix");
		

		getMiddleIntersectionPoints(inner_calib_matr_loaded, matrix_4, points1, points2, times_4, middle_points);
		cvSave("currentMiddlePoints_next.txt", middle_points , "current_middle_points");
	}



	cvReleaseMat(& times_1);
	cvReleaseMat(& times_2);
	cvReleaseMat(& times_3);
	cvReleaseMat(& times_4);

	cvReleaseMat(& middle_points);

	if(fundamentalMatrix != 0 )
	{
		cvReleaseMat( & fundamentalMatrix);
	}
	
	cout.flush();
	cout<< " All combinations are processed." << endl;
	cout.flush();

	

	return 0;
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





