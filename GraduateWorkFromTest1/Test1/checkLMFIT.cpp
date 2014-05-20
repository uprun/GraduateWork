#include "checkLMFIT.h"

	

	void parseYAML(const char* fileName, double* &mas, int &rows, int &columns);

	void parseYAML(const char* fileName, Array &buf)
	{
		parseYAML(fileName, buf.mas, buf.rows, buf.columns);
	}

	const double accPi = acos(-1.0);
	

	
	
	int passed = 0;

	void testFromInitialData( double alpha, double gamma, double focus, double* &parResult, int &n_par, bool &isPassed, Array& left_points, Array &right_points,
		double imageWidth,
		double imageHeight,
		ModelEvaluatorType modelType,
		double &fnorm,
		double &fmark
		)
	{
		/*Array left_points, right_points;
		parseYAML("image_accepted_points_left.txt", left_points);
		parseYAML("image_accepted_points_right.txt", right_points);*/
        /* parameter vector */
        n_par = -1;
		int m_dat =  min( 30, left_points.rows);
		switch(modelType)
		{
		case PAR_6_PROJ:
			n_par = 6;
			break;
		case PAR_7_PROJ:
			n_par = 7;
			break;
		case PAR_6_AND_LINE_TIMES_PROJ:
			n_par = 6 + m_dat;
			break;
		case PAR_7_AND_LINE_TIMES_PROJ:
			n_par = 7 + m_dat;
			break;
		}

        double* par = new double[n_par];
		if(modelType == PAR_6_PROJ || modelType == PAR_6_AND_LINE_TIMES_PROJ)
		{
			par[0] = alpha;
			par[1] = gamma;
			par[2] = focus;
			par[3] = 0;
			par[4] = 0;
			par[5] = 0;

		}
		if(modelType == PAR_6_AND_LINE_TIMES_PROJ)
		{
			for(int i = 6; i < n_par; ++i)
			{
				par[i] = 3;
			}
		}
		if(modelType == PAR_7_PROJ || modelType == PAR_7_AND_LINE_TIMES_PROJ)
		{
			double sh_x, sh_y, sh_z;

			GenerateTranslationCoordinates(alpha, gamma, sh_x, sh_y, sh_z);
			par[0] = sh_x;
			par[1] = sh_y;
			par[2] = sh_z;
			par[3] = focus;
			par[4] = 0;
			par[5] = 0;
			par[6] = 0;
			if(modelType == PAR_7_AND_LINE_TIMES_PROJ)
			{
				for(int i = 7; i < n_par; ++i)
				{
					par[i] = 3;
				}
			}
		}
		 /* arbitrary starting value */

        /* data points */
		

		data_struct data = { &left_points, &right_points, imageWidth / 2, imageHeight / 2, n_par};

        /* auxiliary parameters */
        lm_status_struct status;
        lm_control_struct control = lm_control_double;
		control.patience = 200;
		
        //control.verbosity = 3;

        /* perform the fit */
        //printf( "Fitting:\n" );
		
		
		switch(modelType)
		{
		case PAR_6_PROJ: case PAR_7_PROJ:
			lmmin( n_par, par, m_dat * 2, (const void*) &data, evaluateModelWithPojectionDifferences,
               &control, &status );
			break;	
		case PAR_6_AND_LINE_TIMES_PROJ:
			lmmin( n_par, par, m_dat * 2, (const void*) &data, evaluateModelWithPojectionDifferencesPointsOnLines,
               &control, &status );
			break;
		case PAR_7_AND_LINE_TIMES_PROJ:
			lmmin( n_par, par, m_dat * 2, (const void*) &data, evaluateModelPar7_WithProjectionDifferencesPointsOnLines,
               &control, &status );
			break;
		}

		

        /* print results */
        /*printf( "\nResults:\n" );
        printf( "status after %d function evaluations:\n  %s\n",
                status.nfev, lm_infmsg[status.outcome] );*/

        //printf("obtained parameters:\n");
        /*int i;
        for ( i=0; i<n_par; ++i )
        printf("  par[%i] = %12g\n", i, par[i]);*/
        //printf("obtained norm:\n  %12g\n", status.fnorm );

        //printf("fitting data as follows:\n");

		int userBreak = 0;
		int mark = 0;
		if(parResult != 0)
		{
			delete[] parResult;
		}
		parResult = new double[n_par];
		for(int i = 0 ; i < n_par; ++i)
		{
			parResult[i] = par[i];
		}

		switch(modelType)
		{
		
		case PAR_6_AND_LINE_TIMES_PROJ:
			data.n_par = 6;
			
			break;
		case PAR_7_AND_LINE_TIMES_PROJ:
			data.n_par = 7;
			break;
		}
		mark = getMarkForModel(par, m_dat, &data, &userBreak);
		fmark = ((double)mark) / m_dat;
		fnorm = status.fnorm;
		//printf("obtained mark:\n  %12g  =(%d / %d)\n", fmark, mark, m_dat );

		isPassed = false;
		
		cout<< "fnorm: "<< status.fnorm << " fmark: " << fmark << endl ;
		if( (status.fnorm <= 25 * m_dat) && (fmark >= 0.7))
		{
			++passed;
			if( modelType == PAR_6_PROJ || modelType == PAR_6_AND_LINE_TIMES_PROJ)
			{
				cout <<" alpha: " << par[0] << " gamma: "<< par[1] << endl ;
				cout << " f: " << par[2] << endl << " r_x: " << par[3] << endl << " r_y: " << par[4] << endl << " r_z: " << par[5] << endl;
				if(modelType == PAR_6_AND_LINE_TIMES_PROJ)
				{
					cout<< "Line Times:"<<endl;
					for(int k = 6; k < n_par; k++)
					{
						cout <<"t["<< k - 6 <<"]= " << par[k] << "; ";
					}
					cout << endl;
				}
			}
			
			if( modelType == PAR_7_PROJ || modelType == PAR_7_AND_LINE_TIMES_PROJ)
			{
				cout <<" shift x: " << par[0] << " shift y: "<< par[1] << " shift z: "<< par[2] << endl ;
				cout << " f: " << par[3] << endl << " r_x: " << par[4] << endl << " r_y: " << par[5] << endl << " r_z: " << par[6] << endl;
				if(modelType == PAR_7_AND_LINE_TIMES_PROJ)
				{
					cout<< "Line Times:"<<endl;
					for(int k = 7; k < n_par; k++)
					{
						cout <<"t["<< k - 7 <<"]= " << par[k] << "; ";
					}
					cout << endl;
				}
			}
			isPassed = true;	
		}

		//left_points.dispose();
		//right_points.dispose();
		delete[] par;

	}

	void exportReconstructedDotsOfModelToPLY(const char* fileName, data_struct& dataToReconstruct, double *par,
		Mat &leftImage, Mat &rightImage, ModelEvaluatorType modelType);
	void exportReconstructedDotsOfModelToPLY(const char* fileName, data_struct& dataToReconstruct, double *par, 
		Mat &leftImage, Mat &rightImage, ModelEvaluatorType modelType)
	 {
        /* for readability, explicit type conversion */
		


        data_struct *D;
		D = &dataToReconstruct;

		FILE* writePLY = fopen(fileName, "w");
		fprintf(writePLY, "ply\n");
		fprintf(writePLY, "format ascii 1.0\n");
		fprintf(writePLY, "comment author: Alexander Kryvonos\n");
		fprintf(writePLY, "comment object: reconstructed scene with color\n");
		

		double sh_x, sh_y, sh_z, f, r_x, r_y, r_z, alpha, gamma;
		int n_par = D->n_par;
		if( modelType == PAR_7_PROJ || modelType == PAR_7_AND_LINE_TIMES_PROJ)
		{
			sh_x = par[0];
			sh_y = par[1];
			sh_z = par[2];


			f = par[3];
			r_x = par[4];
			r_y = par[5];
			r_z = par[6];
		}
		if( modelType == PAR_6_PROJ || modelType == PAR_6_AND_LINE_TIMES_PROJ)
		{
			alpha = par[0];
			gamma = par[1];
			f = par[2];
			r_x = par[3];
			r_y = par[4];
			r_z = par[5];
		
		}
		

		Array *translate, *rot_around_x, *rot_around_y, *rot_around_z, *inner;

		if( modelType == PAR_6_PROJ || modelType == PAR_6_AND_LINE_TIMES_PROJ)
		{
			translate = GenerateTranslate(alpha, gamma);
		}
		if( modelType == PAR_7_PROJ || modelType == PAR_7_AND_LINE_TIMES_PROJ)
		{
			translate = GenerateTranslate(sh_x, sh_y, sh_z);
		}
		
		rot_around_x = GenerateRotateAroundX(r_x);
		rot_around_y = GenerateRotateAroundY(r_y);
		rot_around_z = GenerateRotateAroundZ(r_z);
		inner = GenerateInner(f, D->c_x, D->c_y);

		/*translate->print();
		rot_around_x->print();
		rot_around_y->print();
		rot_around_z->print();
		inner->print();*/

		Array *buf, *result;
		Array *rotationResult, *rotationResult2;

		result = rot_around_z->multiply(rot_around_z, translate);

		buf = rot_around_z->multiply(rot_around_y, result);
		result->dispose();
		delete result;
		result = rot_around_z->multiply(rot_around_x, buf);
		buf->dispose();
		delete buf;
		buf = rot_around_z->multiply(inner, result);
		result->dispose();
		delete result;
		
		result = buf; // result = inner*Rx*Ry*Rz*Tr

		rotationResult = rot_around_y->multiply(rot_around_y, rot_around_z);
		rotationResult2 = rotationResult->multiply(rot_around_x, rotationResult);
		rotationResult->dispose();
		delete rotationResult;
		rotationResult = rotationResult2;

		int mark = 0;

		// calculate count of points with positive z coordinate on both cameras
		for(int i = 0; i < D->leftPoints->rows; ++i)
		{
			
			
			int points_index = i ;

			double u_0 = D->leftPoints->get(points_index, 0); // x on first image
			double v_0 = D->leftPoints->get(points_index, 1); // y on first image
			double u_1 = D->rightPoints->get(points_index, 0); // x on second image
			double v_1 = D->rightPoints->get(points_index, 1); // y on second image

			
				
			MyLine3D * line_1, * line_2;
			line_1 = getLineWithTransformations(u_0, v_0, inner, rotationResult, translate);
			line_2 = getJustLine(u_1, v_1, inner);

			double time_1, time_2;

			int intersectionResult = line_2->intersection(*line_1, time_2, time_1);


			if(time_1 > 0 && time_2 > 0 && intersectionResult == 1)
			{
				mark++;
			}
			delete line_1;
			delete line_2;	
		}

		cout << "Filtered count of points : "<< mark << " / " << D->leftPoints->rows <<endl;

		fprintf(writePLY, "element vertex %d\n", mark);
		fprintf(writePLY, "property double x\n");
		fprintf(writePLY, "property double y\n");
		fprintf(writePLY, "property double z\n");
		fprintf(writePLY, "property uchar red\n");
		fprintf(writePLY, "property uchar green\n");
		fprintf(writePLY, "property uchar blue\n");
		fprintf(writePLY, "end_header\n");

		for(int i = 0; i < D->leftPoints->rows; ++i)
		{
			
			
			int points_index = i ;

			double u_0 = D->leftPoints->get(points_index, 0); // x on first image
			double v_0 = D->leftPoints->get(points_index, 1); // y on first image
			double u_1 = D->rightPoints->get(points_index, 0); // x on second image
			double v_1 = D->rightPoints->get(points_index, 1); // y on second image

			
				
			MyLine3D * line_1, * line_2;
			line_1 = getLineWithTransformations(u_0, v_0, inner, rotationResult, translate);
			line_2 = getJustLine(u_1, v_1, inner);

			double time_1, time_2;

			int intersectionResult = line_2->intersection(*line_1, time_2, time_1);


			if(time_1 > 0 && time_2 > 0 && intersectionResult == 1)
			{
				mark++;
			}
			else
			{
				continue;
			}

			int r = 0,g = 0,b = 0 , divideByCount = 0;
			for( int x = 0; x < 3; ++x)
			{
				for( int y = 0; y < 3;++y)
				{
					int curX, curY ;
					curX = (int)floor(u_0 - 1.5 + x + 0.5);
					curY = (int)floor(v_0 - 1.5 + y + 0.5);
					if((curX < 0 )|| (curX >= leftImage.cols) )
					{
						continue;
					}
					if( (curY < 0) || (curY >= leftImage.rows))
					{
						continue;
					}
					++divideByCount;
					Vec3b curPixel = leftImage.at<Vec3b>(curY, curX);
					b += curPixel[0];
					g += curPixel[1];
					r += curPixel[2];
				}
			}

			for( int x = 0; x < 3; ++x)
			{
				for( int y = 0; y < 3;++y)
				{
					int curX, curY ;
					curX = (int)floor(u_1 - 1.5 + x + 0.5);
					curY = (int)floor(v_1 - 1.5 + y + 0.5);
					if((curX < 0 )|| (curX >= rightImage.cols) )
					{
						continue;
					}
					if( (curY < 0) || (curY >= rightImage.rows))
					{
						continue;
					}
					++divideByCount;
					Vec3b curPixel = rightImage.at<Vec3b>(curY, curX);
					b += curPixel[0];
					g += curPixel[1];
					r += curPixel[2];
				}
			}

			r /= divideByCount;
			g /= divideByCount;
			b /= divideByCount;
			/*int curX, curY ;
			curX = (int)floor(u_1 + 0.5 );
			curY = (int)floor(v_1 + 0.5);
			Vec3b curPixel = rightImage.at<Vec3b>(curY, curX);
			
			b = curPixel[0];
			g = curPixel[1];
			r = curPixel[2];*/

			{
				MyPoint3D* point_line_1, * point_line_2;

				point_line_1 = line_1->getPointByTime(time_1);
				point_line_2 = line_2->getPointByTime(time_2);

				MyPoint3D* diff = point_line_2->subtract(point_line_1);

				double ro_distance_square = diff->normEuqlidianSquare();

				MyPoint3D* half_diff = diff->multiplicationByNumber( 0.5 );
				
				// get middle point in second camera coordinate system
				MyPoint3D* middle_point = point_line_1->add(half_diff);
				
				fprintf(writePLY, "%lf %lf %lf %d %d %d\n", middle_point->getX(), middle_point->getY(), middle_point->getZ(),
					r, g, b);
				
				
				delete point_line_1;
				delete point_line_2;
				delete diff;
				delete half_diff;
				delete middle_point;
			}


			delete line_1;
			delete line_2;			

		}


		rotationResult->dispose();
		delete rotationResult;

		result->dispose();
		delete result;

		translate->dispose();
		rot_around_x->dispose();
		rot_around_y->dispose();
		rot_around_z->dispose();
		inner->dispose();
		delete translate;
		delete rot_around_x;
		delete rot_around_y;
		delete rot_around_z;
		delete inner;
		fclose(writePLY);

    }
   
    int mainLevenbergMarkvardt_LMFIT(double focus, const char* fileNameForPlyExport, Array& left_points, Array &right_points,
		Mat &leftImage, Mat & rightImage, Array& reconstruction_left_points, Array& reconstruction_right_points)
    {
		
		double leftAlpha = -accPi, rightAlpha = accPi;
		double leftGamma = -accPi / 2, rightGamma = accPi / 2;

		double step = 0.6;
		int n_par = -1;
		double *paramsFromTest = 0;
		//double focus = 1;

		cout<< "focus:"<<  focus << endl;
		char* str = new char[1000];

		strcpy(str, fileNameForPlyExport);
		strcat(str, ".ply\0");

		cout << endl  << "File name is accepted. Result file name is: " << str << endl;
		bool isPassed = false;

		int alphaSize = (int)( ceil((rightAlpha - leftAlpha) / step) );
		int gammaSize = (int)( ceil((rightGamma - leftGamma) / step) );

		int *coordinatesOfValidModels = new int[alphaSize * gammaSize];
		int *movesOfModels = new int[alphaSize * gammaSize];
		double *map_fnorm = new double [alphaSize * gammaSize];
		pair<double, double> *initial_params = new pair<double, double>[alphaSize * gammaSize];

		for( int i = 0 ; i < alphaSize; ++i)
		{
			for( int j = 0; j < gammaSize; ++j)
			{
				coordinatesOfValidModels[i * gammaSize + j] = 0;
				movesOfModels[i * gammaSize + j] = 0;
				map_fnorm[i * gammaSize + j] = 0;
			}
		}
		int row, column;
		row = 0;
		
		ModelEvaluatorType modelType = PAR_7_AND_LINE_TIMES_PROJ;

		for(double a = leftAlpha; a <= rightAlpha; a += step, row++)
		{
			column = 0;
			for( double g = leftGamma; g <= rightGamma; g += step, column++)
			{
				cout << "========================="<< endl;
				initial_params[row* gammaSize + column] = pair<double, double>(a, g);
				cout << "start alpha: " << row << " start gamma: " << column << endl;
				double fmark, fnorm ;
				testFromInitialData(a, g, focus, paramsFromTest, n_par, isPassed, left_points, right_points, leftImage.cols ,leftImage.rows, modelType,
					fnorm, fmark);
				double testAlpha, testGamma;
				if( modelType == PAR_6_PROJ || modelType == PAR_6_AND_LINE_TIMES_PROJ)
				{
					testAlpha = paramsFromTest[0];
					testGamma = paramsFromTest[1];
				}
				if( modelType == PAR_7_PROJ || modelType == PAR_7_AND_LINE_TIMES_PROJ)
				{
					ConvertTranslationCoordinatesToAngles(testAlpha, testGamma,
						paramsFromTest[0],
						paramsFromTest[1],
						paramsFromTest[2]);
				}
				
				int xAlpha =(int) ( floor(((testAlpha - leftAlpha) / step) + 0.5) );
				int xGamma = (int) ( floor(((testGamma - leftGamma) / step) + 0.5));

				xAlpha = (xAlpha % alphaSize + alphaSize) % alphaSize;
				xGamma = (xGamma % gammaSize + gammaSize) % gammaSize;
				int currentFinishValue= xAlpha* gammaSize + xGamma;
				if(isPassed)
				{
					
					cout<< "xAlpha: "<< xAlpha << " xGamma: " << xGamma<<endl;
					coordinatesOfValidModels[xAlpha* gammaSize + xGamma]++;
					movesOfModels[row* gammaSize + column] = currentFinishValue;
					map_fnorm[row* gammaSize + column] += fnorm;
				}
				else
				{
					//coordinatesOfValidModels[xAlpha* gammaSize + xGamma]--;
					movesOfModels[row* gammaSize + column] = - currentFinishValue;
				}
				
			}
		}
		cout<<"focus: "<< focus<< endl;
		cout<< "map of results:"<< endl;
		for( int i = 0 ; i < alphaSize; ++i)
		{
			for( int j = 0; j < gammaSize; ++j)
			{
				cout<<std::setw(6)<< coordinatesOfValidModels[i* gammaSize + j];
			}
			cout<< endl;
		}
		cout<< "map of norm values: " << endl;
		for( int i = 0 ; i < alphaSize; ++i)
		{
			for( int j = 0; j < gammaSize; ++j)
			{
				int total_count = coordinatesOfValidModels[i* gammaSize + j];
				if(total_count > 0)
				{
					cout<<std::setw(6) << fixed << setprecision(3) << map_fnorm[i* gammaSize + j] /  total_count ;
				}
				else
				{
					cout<<std::setw(6) << "=" ;
				}
				
			}
			cout<< endl;
		}
		cout<<"map of moves:" << endl;

		for( int i = 0 ; i < alphaSize; ++i)
		{
			for( int j = 0; j < gammaSize; ++j)
			{
				int current_value = movesOfModels[i* gammaSize + j];
				cout<<std::setw(6) << current_value / gammaSize << "|" << current_value % gammaSize;
			}
			cout<< endl;
		}
		cout << "Passed : "<< passed<< endl;
		cout << "Starting selecting optimal initial parameters:" << endl;

		int maxI = 0, maxJ = 0;
		for( int i = 0 ; i < alphaSize; ++i)
		{
			for( int j = 0; j < gammaSize; ++j)
			{
				int currentValue = coordinatesOfValidModels[i* gammaSize + j] ;
				int currentMaxValue = coordinatesOfValidModels[maxI * gammaSize + maxJ] ;
				if(currentValue > currentMaxValue)
				{
					maxI = i;
					maxJ = j;
				}
			}
			
		}

		cout<< "Selected coordinates are: row : " << maxI << " column: "<< maxJ << endl;
		cout << "Starting to find initial parameters:  " << endl;
		int currentFinishValue= maxI* gammaSize + maxJ;
		bool resultIsFound = false;
		int curI = 0, curJ = 0;

		for( int i = 0 ; i < alphaSize; ++i)
		{
			for( int j = 0; j < gammaSize; ++j)
			{
				int currentValue = movesOfModels[i* gammaSize + j] ;
				
				if(currentValue == currentFinishValue)
				{
					int dist = abs(maxI - i) + abs(maxJ - j);
					int distCur = abs(maxI - curI) + abs(maxJ - curJ);
					if( dist < distCur)
					{
						curI = i;
						curJ = j;
					}
				}
			}
			
		}

		maxI = curI;
		maxJ = curJ;

		cout<< "Selected initial parameters are: row : " << maxI << " column: "<< maxJ << endl;
		pair<double, double> selected_initial_params = initial_params[maxI * gammaSize + maxJ];
		double alpha = selected_initial_params.first; //leftAlpha + step * (maxI);
		double gamma = selected_initial_params.second; //leftGamma + step * (maxJ);
		cout << "Alpha: " << alpha << " Gamma: " << gamma << endl;
		double f_norm, f_mark;
		testFromInitialData(alpha, gamma, focus, paramsFromTest, n_par, isPassed, left_points,
			right_points, leftImage.cols ,leftImage.rows, modelType, f_norm, f_mark);

		double testAlpha, testGamma;
		if( modelType == PAR_6_PROJ || modelType == PAR_6_AND_LINE_TIMES_PROJ)
		{
			testAlpha = paramsFromTest[0];
			testGamma = paramsFromTest[1];
		}
		if( modelType == PAR_7_PROJ || modelType == PAR_7_AND_LINE_TIMES_PROJ)
		{
			ConvertTranslationCoordinatesToAngles(testAlpha, testGamma,
				paramsFromTest[0],
				paramsFromTest[1],
				paramsFromTest[2]);
		}
				
		int xAlpha =(int) ( floor(((testAlpha - leftAlpha) / step) + 0.5) );
		int xGamma = (int) ( floor(((testGamma - leftGamma) / step) + 0.5));

		xAlpha = (xAlpha % alphaSize + alphaSize) % alphaSize;
		xGamma = (xGamma % gammaSize + gammaSize) % gammaSize;
		int finishValue= xAlpha* gammaSize + xGamma;
		if( finishValue != currentFinishValue)
		{
			cout << "Error/Warning:: Determined finish value and actual finish value are not equal" << endl;
		}
		cout<< "xAlpha: "<< xAlpha << " xGamma: " << xGamma<<endl;
		for(int i = 0; i < n_par; ++i)
			{
				cout << "par[" << i << "] = " << paramsFromTest[i] << endl;
			}
		if(isPassed)
		{
			
			cout <<endl << "Successfully finished to determine parameters!!!"<< endl;
			
		}
		else
		{
			cout << endl << "ERROR: Algorithm failed to finish with valid parameters." << endl;
		}

        /* parameter vector */
        /* number of parameters in model function f */
		data_struct data = { &reconstruction_left_points, &reconstruction_right_points, leftImage.cols / 2.0, leftImage.rows / 2.0, n_par};
		cout << "Starting reconstruction and export to *.PLY file" << endl;
		exportReconstructedDotsOfModelToPLY(str, data, paramsFromTest, leftImage, rightImage, modelType);
		cout << "Reconstruction and export are successfully finished. Result file name is :"<< endl;
		cout << str << endl;

		delete[] paramsFromTest;
		delete[] coordinatesOfValidModels;
		delete[] movesOfModels;
		delete[] initial_params;
		delete[] map_fnorm;
        

        return 0;
    }



	void parseYAML(const char* fileName, double* &mas, int &rows, int &columns)
	{
		

		FILE* handler = fopen(fileName, "r");

		char str[255];
		rows = 0;
		columns = 0;
		fscanf(handler, "%s", &str);// YAML
		fscanf(handler, "%s", &str); // name: 
		fscanf(handler, "%s", &str); // !!opencv-matrix
		fscanf(handler, "%s", &str); // rows:
		fscanf(handler, "%d", &rows); // <number of rows>
		fscanf(handler, "%s", &str); // cols:
		fscanf(handler, "%d", &columns); // cols: <number of columns>
		fscanf(handler, "%s", &str); // dt:
		fscanf(handler, "%s", &str); // f

		fscanf(handler, "%s", &str); // data:
		fscanf(handler, "%s", &str); // [

		int totalCount = rows * columns;

		mas = new double[totalCount];
		for( int i = 0; i < totalCount; ++i)
		{
			double buf = 0;
			fscanf(handler, "%lf", &buf); // array data
			fscanf(handler, "%s", &str);// separator or ]
			mas[i] = buf;
		}
		

		fclose( handler);
	}

