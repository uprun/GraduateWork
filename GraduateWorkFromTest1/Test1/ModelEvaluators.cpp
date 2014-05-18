#include "ModelEvaluators.h"

void evaluateModelWith6ParamsMinimizeLineDistance( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak )
    {
        /* for readability, explicit type conversion */
        data_struct *D;
        D = (data_struct*)data;

		double sh_x, sh_y, sh_z, f, r_x, r_y, r_z, alpha, gamma;
		/*sh_x = par[0];
		sh_y = par[1];
		sh_z = par[2];


		f = par[3];
		r_x = par[4];
		r_y = par[5];
		r_z = par[6];*/

		alpha = par[0];
		gamma = par[1];
		f = par[2];
		r_x = par[3];
		r_y = par[4];
		r_z = par[5];
		int n_par = D->n_par;

		// clean fvec
		for(int i = 0; i < m_dat; ++i)
		{
			fvec[i] = 0;
		}

		// check requirements and bounds for variables
		bool needToReturn = false;

		

		if(f < 0)
		{
			/*for(int i = 0; i < m_dat; ++i)
			{
				fvec[i] = f * f * penalty;
			}
			needToReturn = true;*/
			*userbreak = -1;
			needToReturn = true;
		}

		

		if(needToReturn)
		{
			return;
		}

		Array *translate, *rot_around_x, *rot_around_y, *rot_around_z, *inner;

		translate = GenerateTranslate(alpha, gamma);
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

		for(int i = 0; i < m_dat; ++i)
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

			

			if( intersectionResult > 1)
			{
				fvec[i] = penalty;
			}
			else
			{
				MyPoint3D* point_line_1, * point_line_2;

				point_line_1 = line_1->getPointByTime(time_1);
				point_line_2 = line_2->getPointByTime(time_2);

				MyPoint3D* diff = point_line_2->subtract(point_line_1);

				double ro_distance_square = diff->normEuqlidianSquare();

				MyPoint3D* half_diff = diff->multiplicationByNumber( 0.5 );
				
				// get middle point in second camera coordinate system
				MyPoint3D* middle_point = point_line_1->add(half_diff);

				fvec[i] = ro_distance_square;

				//double z = middle_point->getZ();

				//if(z <= 0.1)
				//{
				//	fvec[i] = (z - 0.1) * (z - 0.1) ;
				//}
				//else
				//{
				//	Array* currentPoint = new Array();
				//	currentPoint->init(4, 1);
				//	currentPoint->set(0, 0, middle_point->getX());
				//	currentPoint->set(1, 0, middle_point->getY());
				//	currentPoint->set(2, 0, z);
				//	// get projection of middle point on second camera
				//	Array* projectionResult = inner->multiply(inner, currentPoint);

				//	double x, y;

				//	x = projectionResult->get(0, 0);
				//	y = projectionResult->get(1, 0);
				//	x /= z;
				//	y /= z;

				//	double deltaX = x - u_1;
				//	double deltaY = y - v_1;

				//	double distanceSquare = deltaX * deltaX + deltaY * deltaY;

				//	fvec[i] = distanceSquare ;


				//	currentPoint->dispose();
				//	delete currentPoint;
				//	projectionResult->dispose();
				//	delete projectionResult;
				//}
				
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


        
    }


	void evaluateModelWith6Params( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak )
    {
        /* for readability, explicit type conversion */
        data_struct *D;
        D = (data_struct*)data;

		double sh_x, sh_y, sh_z, f, r_x, r_y, r_z, alpha, gamma;
		/*sh_x = par[0];
		sh_y = par[1];
		sh_z = par[2];


		f = par[3];
		r_x = par[4];
		r_y = par[5];
		r_z = par[6];*/

		alpha = par[0];
		gamma = par[1];
		f = par[2];
		r_x = par[3];
		r_y = par[4];
		r_z = par[5];
		int n_par = D->n_par;

		// clean fvec
		for(int i = 0; i < m_dat; ++i)
		{
			fvec[i] = 0;
		}

		// check requirements and bounds for variables
		bool needToReturn = false;

		

		if(f < 0)
		{
			/*for(int i = 0; i < m_dat; ++i)
			{
				fvec[i] = f * f * penalty;
			}
			needToReturn = true;*/
			*userbreak = -1;
			needToReturn = true;
		}

		

		if(needToReturn)
		{
			return;
		}

		Array *translate, *rot_around_x, *rot_around_y, *rot_around_z, *inner;

		translate = GenerateTranslate(alpha, gamma);
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

		for(int i = 0; i < m_dat; ++i)
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

			

			if( intersectionResult > 1)
			{
				fvec[i] = penalty;
			}
			else
			{
				MyPoint3D* point_line_1, * point_line_2;

				point_line_1 = line_1->getPointByTime(time_1);
				point_line_2 = line_2->getPointByTime(time_2);

				MyPoint3D* diff = point_line_2->subtract(point_line_1);

				double ro_distance_square = diff->normEuqlidianSquare();

				MyPoint3D* half_diff = diff->multiplicationByNumber( 0.5 );
				
				// get middle point in second camera coordinate system
				MyPoint3D* middle_point = point_line_1->add(half_diff);

				//fvec[i] = ro_distance_square;

				double z = middle_point->getZ();

				if(z <= 0.1)
				{
					fvec[i] = (z - 0.1) * (z - 0.1) ;
				}
				else
				{
					Array* currentPoint = new Array();
					currentPoint->init(4, 1);
					currentPoint->set(0, 0, middle_point->getX());
					currentPoint->set(1, 0, middle_point->getY());
					currentPoint->set(2, 0, z);
					// get projection of middle point on second camera
					Array* projectionResult = inner->multiply(inner, currentPoint);

					double x, y;

					x = projectionResult->get(0, 0);
					y = projectionResult->get(1, 0);
					x /= z;
					y /= z;

					double deltaX = x - u_1;
					double deltaY = y - v_1;

					double distanceSquare = deltaX * deltaX + deltaY * deltaY;

					fvec[i] = distanceSquare ;


					currentPoint->dispose();
					delete currentPoint;
					projectionResult->dispose();
					delete projectionResult;
				}
				
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


        
    }

	void evaluateModelWithPojectionDifferences( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak )
    {
        /* for readability, explicit type conversion */
        data_struct *D;
        D = (data_struct*)data;

		double sh_x, sh_y, sh_z, f, r_x, r_y, r_z, alpha, gamma;
		int n_par = D->n_par;
		if( n_par == 7)
		{
			sh_x = par[0];
			sh_y = par[1];
			sh_z = par[2];


			f = par[3];
			r_x = par[4];
			r_y = par[5];
			r_z = par[6];
		}
		if( n_par == 6)
		{
			alpha = par[0];
			gamma = par[1];
			f = par[2];
			r_x = par[3];
			r_y = par[4];
			r_z = par[5];
		}
		
		

		// clean fvec
		for(int i = 0; i < m_dat; ++i)
		{
			fvec[i] = 0;
		}

		// check requirements and bounds for variables
		bool needToReturn = false;

		

		if(f < 0)
		{
			/*for(int i = 0; i < m_dat; ++i)
			{
				fvec[i] = f * f * penalty;
			}
			needToReturn = true;*/
			*userbreak = -1;
			needToReturn = true;
		}

		

		if(needToReturn)
		{
			return;
		}

		Array *translate, *rot_around_x, *rot_around_y, *rot_around_z, *inner;

		if( n_par == 6)
		{
			translate = GenerateTranslate(alpha, gamma);
		}
		if( n_par == 7 )
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

		int upI = m_dat / 2;

		for(int i = 0; i < upI; ++i)
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

			

			if( intersectionResult > 1)
			{
				fvec[i] = penalty;
			}
			else
			{
				MyPoint3D* point_line_1, * point_line_2;

				point_line_1 = line_1->getPointByTime(time_1);
				point_line_2 = line_2->getPointByTime(time_2);

				MyPoint3D* diff = point_line_2->subtract(point_line_1);

				double ro_distance_square = diff->normEuqlidianSquare();

				MyPoint3D* half_diff = diff->multiplicationByNumber( 0.5 );
				
				// get middle point in second camera coordinate system
				MyPoint3D* middle_point = point_line_1->add(half_diff);

				//fvec[i] = ro_distance_square;

				double z = middle_point->getZ();

				if(z <= 0.1)
				{
					fvec[2*i] = (z - 0.1) * (z - 0.1) ;
					fvec[2*i+1] = (z - 0.1) * (z - 0.1) ;
				}
				else
				{
					Array* currentPoint = new Array();
					currentPoint->init(4, 1);
					currentPoint->set(0, 0, middle_point->getX());
					currentPoint->set(1, 0, middle_point->getY());
					currentPoint->set(2, 0, z);
					// get projection of middle point on second camera
					Array* projectionResult = inner->multiply(inner, currentPoint);

					double x, y;

					x = projectionResult->get(0, 0);
					y = projectionResult->get(1, 0);
					x /= z;
					y /= z;

					double deltaX = x - u_1;
					double deltaY = y - v_1;

					//double distanceSquare = deltaX * deltaX + deltaY * deltaY;

					fvec[i*2] = deltaX;
					fvec[i*2 + 1] = deltaY;


					currentPoint->dispose();
					delete currentPoint;
					projectionResult->dispose();
					delete projectionResult;
				}
				
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


        
    }

	void evaluateModelWithPojectionDifferencesçPointsOnLines( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak )
    {
        /* for readability, explicit type conversion */
        data_struct *D;
        D = (data_struct*)data;

		double sh_x, sh_y, sh_z, f, r_x, r_y, r_z, alpha, gamma;
		int n_par = D->n_par;
		
		{
			alpha = par[0];
			gamma = par[1];
			f = par[2];
			r_x = par[3];
			r_y = par[4];
			r_z = par[5];
		}
		
		

		// clean fvec
		for(int i = 0; i < m_dat; ++i)
		{
			fvec[i] = 0;
		}

		// check requirements and bounds for variables
		bool needToReturn = false;

		

		if(f < 0)
		{
			/*for(int i = 0; i < m_dat; ++i)
			{
				fvec[i] = f * f * penalty;
			}
			needToReturn = true;*/
			*userbreak = -1;
			needToReturn = true;
		}

		

		if(needToReturn)
		{
			return;
		}

		Array *translate, *rot_around_x, *rot_around_y, *rot_around_z, *inner;

		
		translate = GenerateTranslate(alpha, gamma);
		
		
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

		int upI = m_dat / 2;

		for(int i = 0; i < upI; ++i)
		{
			
			
			int points_index = i ;

			double u_0 = D->leftPoints->get(points_index, 0); // x on first image
			double v_0 = D->leftPoints->get(points_index, 1); // y on first image
			double u_1 = D->rightPoints->get(points_index, 0); // x on second image
			double v_1 = D->rightPoints->get(points_index, 1); // y on second image

			
				
			MyLine3D * line_1;
			line_1 = getLineWithTransformations(u_0, v_0, inner, rotationResult, translate);

			double time_1 = par[i+6];

			

			

			
			{
				MyPoint3D* point_line_1;

				point_line_1 = line_1->getPointByTime(time_1);
				double z = point_line_1->getZ();

				if(z <= 0.1)
				{
					fvec[2*i] = (z - 0.1) * (z - 0.1) * penalty + penalty;
					fvec[2*i+1] = (z - 0.1) * (z - 0.1) * penalty + penalty;
				}
				else
				{
					Array* currentPoint = new Array();
					currentPoint->init(4, 1);
					currentPoint->set(0, 0, point_line_1->getX());
					currentPoint->set(1, 0, point_line_1->getY());
					currentPoint->set(2, 0, z);
					// get projection of point_line_1 on second camera
					Array* projectionResult = inner->multiply(inner, currentPoint);

					double x, y;

					x = projectionResult->get(0, 0);
					y = projectionResult->get(1, 0);
					x /= z;
					y /= z;

					double deltaX = x - u_1;
					double deltaY = y - v_1;

					

					fvec[i*2] = deltaX;
					fvec[i*2 + 1] = deltaY;


					currentPoint->dispose();
					delete currentPoint;
					projectionResult->dispose();
					delete projectionResult;
				}
				
				delete point_line_1;
				
			}


			delete line_1;			

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


        
    }

	


	int getMarkForModel( const double *par, int m_dat,
        const void *data, int *userbreak )
    {
        /* for readability, explicit type conversion */
        data_struct *D;
        D = (data_struct*)data;

		double sh_x, sh_y, sh_z, f, r_x, r_y, r_z, alpha, gamma;

		int n_par = D->n_par;

		if( n_par == 7)
		{
			sh_x = par[0];
			sh_y = par[1];
			sh_z = par[2];


			f = par[3];
			r_x = par[4];
			r_y = par[5];
			r_z = par[6];
		}
		if( n_par == 6)
		{
			alpha = par[0];
			gamma = par[1];
			f = par[2];
			r_x = par[3];
			r_y = par[4];
			r_z = par[5];
		}
		

		
		


		// check requirements and bounds for variables
		bool needToReturn = false;

		

		if(f < 0)
		{
			/*for(int i = 0; i < m_dat; ++i)
			{
				fvec[i] = f * f * penalty;
			}
			needToReturn = true;*/
			*userbreak = -1;
			needToReturn = true;
		}

		

		if(needToReturn)
		{
			return 0;
		}

		Array *translate, *rot_around_x, *rot_around_y, *rot_around_z, *inner;

		if( n_par == 6)
		{
			translate = GenerateTranslate(alpha, gamma);
		}
		if( n_par == 7)
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

		for(int i = 0; i < m_dat; ++i)
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


			if(time_1 > 0 && time_2 > 0)
			{
				mark++;
			}
			

			if( intersectionResult > 1)
			{
			//	fvec[i] = penalty;
			}
			else
			{
				MyPoint3D* point_line_1, * point_line_2;

				point_line_1 = line_1->getPointByTime(time_1);
				point_line_2 = line_2->getPointByTime(time_2);

				MyPoint3D* diff = point_line_2->subtract(point_line_1);

				double ro_distance_square = diff->normEuqlidianSquare();

				MyPoint3D* half_diff = diff->multiplicationByNumber( 0.5 );
				
				// get middle point in second camera coordinate system
				MyPoint3D* middle_point = point_line_1->add(half_diff);

				//fvec[i] = ro_distance_square;

				//double z = middle_point->getZ();

				//if(z <= 0.1)
				//{
				//	fvec[i] = (z - 0.1) * (z - 0.1) ;
				//}
				//else
				//{
				//	Array* currentPoint = new Array();
				//	currentPoint->init(4, 1);
				//	currentPoint->set(0, 0, middle_point->getX());
				//	currentPoint->set(1, 0, middle_point->getY());
				//	currentPoint->set(2, 0, z);
				//	// get projection of middle point on second camera
				//	Array* projectionResult = inner->multiply(inner, currentPoint);

				//	double x, y;

				//	x = projectionResult->get(0, 0);
				//	y = projectionResult->get(1, 0);
				//	x /= z;
				//	y /= z;

				//	double deltaX = x - u_1;
				//	double deltaY = y - v_1;

				//	double distanceSquare = deltaX * deltaX + deltaY * deltaY;

				//	fvec[i] = distanceSquare ;


				//	currentPoint->dispose();
				//	delete currentPoint;
				//	projectionResult->dispose();
				//	delete projectionResult;
				//}
				
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


        return mark;
    }