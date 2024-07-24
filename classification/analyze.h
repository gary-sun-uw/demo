#ifndef ANALYZE_H
#define ANALYZE_H


#include "../commons/data_formats.h"
#include "classify.h"
#include "../commons/vec3d.h"
#include "stdint.h"

typedef struct AnalysisResult {
	ActionType classification;
} AnalysisResult;

typedef enum AnalysisStatus {
	ANALYSIS_OK
} AnalysisStatus;

#define ANALYSIS_FUNCTOR_NAME_LENGTH 21

typedef struct AnalysisFunctor {
	uint8_t debug;
	char name[ANALYSIS_FUNCTOR_NAME_LENGTH];
} AnalysisFunctor;

void init_analysis_functor(AnalysisFunctor *functor, uint8_t debug, char name[ANALYSIS_FUNCTOR_NAME_LENGTH]);

/* Analyze and classify motion data. */
AnalysisStatus analyze(
		BMI2SensData data[], 		/* Raw data. */
		uint16_t data_length, 		/* Length of data. */
		BMI2SensData g, 			/* Stable gravity recording. */
		uint8_t debug, 				/* Debug flag. Will print all intermediate results.*/
		AnalysisResult *out);		/* Output. */


#endif