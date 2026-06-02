#ifndef PID_H
#define PID_H

#include <stdbool.h>

// Structure, that holds PID parameters and the current controller state.
typedef struct PIDControllerInfoStruct {

    // Controller parameters
    float Kp;   // Controller gain
    float Ti;   // Reset time (0 -> no I part)
    float Td;   // Derivative time (0 -> no D part)
    float Tf;   // Filter time constant (0 -> no filter)
    bool Arw;   // true: anti-windup active, false: anti-windup inactive
    bool IPartActivePrev; // Boolean indicating whether the I part was active in the previous compute step to only recompute the coefficients for the controll algorithm when actually needed.
    bool FilterActivePrev; // Boolean indicating whether the filter was active in the previous compute step to only recompute the coefficients for the filter algorithm when actually needed.

    // Coefficients for the control algorithm
	float C[3];

    // Coefficients for filter algorithm
    float Cf[2];

	// Lower and upper bounds
	float mMin;
    float mMax;

	// Sample time
	float Ts;

    // Buffer for control deviation (filtered and unfiltered): e[0] is the most recent value, e[1] the one from the previous step, ...
	float e[3];
    float eFil[3];

    // Buffer actuation value: m[0] is the most recent value, m[1] the one from the previous step, ...
	float m[3]; // Controller output

	// Current P, I, and D part
	float P;
	float I;
	float D;

} PIDControllerInfo;

void pid_init(PIDControllerInfo* PidInfo);
bool pid_para_set(PIDControllerInfo* PidInfo, float Kp, float Ti, float Td, float Tf, float Ts);
bool pid_limits_set(PIDControllerInfo* PidInfo, float Mmin, float Mmax);
void pid_arw_set(PIDControllerInfo* PidInfo, bool Arw);
void pid_execute(PIDControllerInfo* PidInfo, float e, float* m);

void pid_update_coeff(PIDControllerInfo* PidInfo, bool ForceCalculation);
bool pid_i_part_active(PIDControllerInfo* PidInfo);
bool pid_filter_active(PIDControllerInfo* PidInfo);

#endif