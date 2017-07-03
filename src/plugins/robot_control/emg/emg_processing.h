#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "namespaces.h"

#include "utils/debug/data_logging.h"

enum EMGProcessingPhase { EMG_PROCESSING_MEASUREMENT, EMG_PROCESSING_CALIBRATION, EMG_PROCESSING_OFFSET, EMG_PROCESSING_SAMPLING, EMG_PROCESSING_PHASES_NUMBER };
//enum EMGMuscleGain { MUSCLE_GAIN_ACTIVATION, MUSCLE_GAIN_LENGTH, MUSCLE_GAIN_ARM, MUSCLE_GAIN_PENATION, MUSCLE_GAIN_FORCE, MUSCLE_GAINS_NUMBER };

#define EMG_JOINT_INVALID_ID (unsigned long) -1       ///< Joint identifier to be returned on initialization errors
typedef unsigned long JointID;

#define EMG_JOINT_MAX_MUSCLES_NUMBER 10
#define EMG_MAX_SAMPLES_NUMBER 5000


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


#define EMG_PROCESSING_FUNCTIONS( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( JointID, Namespace, InitJoint, const char* ) \
        INIT_FUNCTION( void, Namespace, EndJoint, JointID ) \
        INIT_FUNCTION( void, Namespace, GetJointMuscleSignals, JointID, double* ) \
        INIT_FUNCTION( void, Namespace, GetJointMuscleTorques, JointID, double*, double, double, double* ) \
        INIT_FUNCTION( double, Namespace, GetJointTorque, JointID, double* ) \
        INIT_FUNCTION( double, Namespace, GetJointStiffness, JointID, double* ) \
        INIT_FUNCTION( void, Namespace, SetJointProcessingPhase, JointID, enum EMGProcessingPhase ) \
        INIT_FUNCTION( size_t, Namespace, GetJointMusclesCount, JointID ) \
        INIT_FUNCTION( void, Namespace, FitJointParameters, JointID, EMGJointSampler )

DECLARE_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESSING_FUNCTIONS );


#endif // EMG_PROCESSING_H 
