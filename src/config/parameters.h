#ifndef PARAMETERS_H_
#define PARAMETERS_H_

//#define PID_GAIN_P 0.90f
//#define PID_GAIN_I 0.025f
//#define PID_GAIN_D 7.2f

#define PID_GAIN_P 1.6f
#define PID_GAIN_I 0.004f
#define PID_GAIN_D 10.0f

#define PID_GAIN_YAW_P 1.6f
#define PID_GAIN_YAW_I 0.002f
#define PID_GAIN_YAW_D 0.0f

#define PID_MAX_I_ERROR 150.0f
#define PID_MAX_OUTPUT  400.0f

#define MOTOR_OUTPUT_MAX  2000
#define MOTOR_OUTPUT_MIN  1050

#define MAIN_LOOP_MAX_TIME 2500 // MS

#define RECEIVER_LEVEL_ADJUST_GAIN 15.0f
#define SET_POINT_DIV             3.0f

#define FILTER_COMP_GAIN_GYRO_RATE  (1.0f / 43.0f)
#define FILTER_COMP_GAIN_ACC  0.3f

#endif