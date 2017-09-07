#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "debug/data_logging.h"

#define EMG_MAX_MUSCLES_NUMBER 10
#define EMG_MAX_JOINTS_NUMBER 10
#define EMG_MAX_SAMPLES_NUMBER 5000


typedef struct _EMGModelData EMGModelData;
typedef EMGModelData* EMGModel;

typedef struct _EMGSamplingData
{
  double muscleSignalsList[ EMG_MAX_MUSCLES_NUMBER ][ EMG_MAX_SAMPLES_NUMBER ];
  double measuredAnglesList[ EMG_MAX_JOINTS_NUMBER ][ EMG_MAX_SAMPLES_NUMBER ];
  double setpointAnglesList[ EMG_MAX_JOINTS_NUMBER ][ EMG_MAX_SAMPLES_NUMBER ];
  double externalTorquesList[ EMG_MAX_JOINTS_NUMBER ][ EMG_MAX_SAMPLES_NUMBER ];
  double sampleTimesList[ EMG_MAX_SAMPLES_NUMBER ];
  size_t samplesCount;
}
EMGSamplingData;


EMGModel EMGProcessing_InitModel( const char* );
void EMGProcessing_EndModel( EMGModel );
const char** EMGProcessing_GetMuscleNames( EMGModel );
//void EMGProcessing_GetMuscleForces( EMGModel, double* );
size_t EMGProcessing_GetMusclesCount( EMGModel );
void EMGProcessing_GetJointTorques( EMGModel, double* );
void EMGProcessing_GetJointStiffnesses( EMGModel, double* );
const char** EMGProcessing_GetJointNames( EMGModel );
size_t EMGProcessing_GetJointsCount( EMGModel );
void EMGProcessing_RunStep( EMGModel, double*, double*, double* );
void EMGProcessing_FitParameters( EMGModel, EMGSamplingData* );


#endif // EMG_PROCESSING_H 
