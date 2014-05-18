#ifndef MODELEVALUATORS_H
#define MODELEVALUATORS_H
#include <math.h>
#include "MyArray.h"
#include "MyLine3D.h"
#include "SupportCode.h"
const double penalty = 100000;

enum ModelEvaluatorType {PAR_6_PROJ, PAR_7_PROJ, PAR_6_AND_LINE_TIMES_PROJ, PAR_7_AND_LINE_TIMES_PROJ, PAR_7_AND_POINTS_IN_SPACE_PROJ};

/* data structure to transmit data arays and fit model */
    typedef struct {
		Array *leftPoints, *rightPoints;
		int c_x, c_y, n_par;
    } data_struct;

int getMarkForModel( const double *par, int m_dat,
        const void *data, int *userbreak );
void evaluateModelWithPojectionDifferences( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak );
void evaluateModelWith6Params( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak );
void evaluateModelWith6ParamsMinimizeLineDistance( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak );

void evaluateModelWithPojectionDifferencesPointsOnLines( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak );

void evaluateModelPar7_WithProjectionDifferencesPointsOnLines( const double *par, int m_dat,
        const void *data, double *fvec, int *userbreak );

#endif