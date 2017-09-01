#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "debug/data_logging.h"

enum EMGProcessingPhase { EMG_PROCESSING_MEASUREMENT, EMG_PROCESSING_CALIBRATION, EMG_PROCESSING_OFFSET, EMG_PROCESSING_SAMPLING, EMG_PROCESSING_PHASES_NUMBER };

#define EMG_JOINT_MAX_MUSCLES_NUMBER 10
#define EMG_MAX_SAMPLES_NUMBER 5000


typedef struct _EMGJointData EMGJointData;
typedef EMGJointData* EMGJoint;

typedef struct _EMGJointSamplingData
{
  double muscleSignalsList[ EMG_JOINT_MAX_MUSCLES_NUMBER ][ EMG_MAX_SAMPLES_NUMBER ];
  double measuredAnglesList[ EMG_MAX_SAMPLES_NUMBER ];
  double setpointAnglesList[ EMG_MAX_SAMPLES_NUMBER ];
  double externalTorquesList[ EMG_MAX_SAMPLES_NUMBER ];
  double sampleTimesList[ EMG_MAX_SAMPLES_NUMBER ];
  size_t musclesCount, samplesCount;
  Log protocolLog;
  bool isLogging;
}
EMGJointSamplingData;

typedef EMGJointSamplingData* EMGJointSampler;


EMGJoint EMGProcessing_Init( const char* );
void EMGProcessing_End( EMGJoint );
void EMGProcessing_GetMuscleSignals( EMGJoint, double* );
void EMGProcessing_GetMuscleTorques( EMGJoint, double*, double, double, double* );
double EMGProcessing_GetTorque( EMGJoint, double* );
double EMGProcessing_GetStiffness( EMGJoint, double* );
void EMGProcessing_SetProcessingPhase( EMGJoint, enum EMGProcessingPhase );
size_t EMGProcessing_GetMusclesCount( EMGJoint );
void EMGProcessing_FitParameters( EMGJoint, EMGJointSampler );


#endif // EMG_PROCESSING_H 
