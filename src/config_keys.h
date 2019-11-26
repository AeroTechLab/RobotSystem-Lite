#ifndef CONFIG_KEYS_H
#define CONFIG_KEYS_H

#define KEY_CONFIG                "config"
#define KEY_MODULES               "plugins"
#define KEY_SENSOR                "sensor"
#define KEY_SENSORS               KEY_SENSOR "s"
#define KEY_MOTOR                 "motor"
#define KEY_MOTORS                KEY_MOTOR "s"
#define KEY_ACTUATOR              "actuator"
#define KEY_ACTUATORS             KEY_ACTUATOR "s"
#define KEY_ROBOT                 "robot"
#define KEY_ROBOTS                KEY_ROBOT "s"
#define KEY_ID                    "id"
#define KEY_JOINTS                "joints"
#define KEY_AXES                  "axes"
#define KEY_SIGNAL_IO             "signal_io"
#define KEY_ROBOT_CONTROL         "robot_control"
#define KEY_CONTROLLER            "controller"
#define KEY_TIME_STEP             "time_step"
#define KEY_INTERFACE             "interface"
#define KEY_TYPE                  "type"
#define KEY_CHANNEL               "channel"
#define KEY_VARIABLE              "variable"
#define KEY_DEVIATION             "deviation"
#define KEY_LIMIT                 "limit"
#define KEY_REFERENCE             "reference"
#define KEY_INPUT                 "input"
#define KEY_INPUTS                KEY_INPUT "s"
#define KEY_OUTPUT                "output"
#define KEY_OUTPUTS               KEY_OUTPUT "s"
#define KEY_EXTRA                 "extra"
#define KEY_EXTRA_INPUTS          KEY_EXTRA "_" KEY_INPUTS
#define KEY_EXTRA_OUTPUTS         KEY_EXTRA "_" KEY_OUTPUTS
#define KEY_SIGNAL_PROCESSING     "signal_processing"
#define KEY_FREQUENCY             "frequency"
#define KEY_MIN_FREQUENCY         "min_" KEY_FREQUENCY
#define KEY_MAX_FREQUENCY         "max_" KEY_FREQUENCY
#define KEY_RECTIFIED             "rectified"
#define KEY_NORMALIZED            "normalized"
#define KEY_LOG                   "log"
#define KEY_LOGS                  KEY_LOG "s"
#define KEY_FILE                  "to_file"
#define KEY_PRECISION             "precision"

#endif // CONFIG_KEYS_H
