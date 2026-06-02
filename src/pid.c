#include "pid.h"

#include <limits.h>
#include<float.h>

/** Initializes a PID controller
 *
 *  @param PidInfo points at die PIDControllerInfo structure
 *
 */
void pid_init(PIDControllerInfo* PidInfo)
{
    int i;

    PidInfo->Kp = 0.0f;
    PidInfo->Ti = 0.0f;
    PidInfo->Td = 0.0f;
    PidInfo->Tf = 0.0f;
    PidInfo->Arw = false;
    PidInfo->Ts = 1.0f;
    PidInfo->mMin = -FLT_MAX;
    PidInfo->mMax = FLT_MAX;

    PidInfo->IPartActivePrev = true;
    PidInfo->FilterActivePrev = true;

    for (i = 0; i < 3; i++) {
        PidInfo->C[i] = 0.0f;
        PidInfo->e[i] = 0.0f;
        PidInfo->eFil[i] = 0.0f;
        PidInfo->m[i] = 0.0f;
    }

    PidInfo->Cf[0] = 0.0f;
    PidInfo->Cf[1] = 0.0f;

    PidInfo->P = 0.0f;
    PidInfo->I = 0.0f;
    PidInfo->D = 0.0f;
}


/** Sets the parameters of the pid controller
 * 
 *  This function sets the controller parameters of the pid and calculates the coefficient for further computation.
 *  Furthermore, the function returns false if an invalid input is detected:
 *  - the sample time is invalid 
 *  - the filter time constant is invalid with respect to sample time (Ts < Tf, if Tf != 0.0f)
 *  - coefficient computation for PD controller-Berechnung auch für einen PD-Regler zulassen
 *  - the other time parameters are invalid (
 *
 *  If all inputs are valid:
 * 
 *  - Kp > 0
 *  - Ti >= 0 (Ti = 0: integral part of the controller is inactive)
 *  - Td >= 0 (Td = 0: differential part of the controller is inactive)
 *  - Tf >= 0 (Tf = 0: the filter is inactive)
 *  - Ts > 0
 *  - Tf > Ts if Tf = 0 (when the filter is active)
 *
 *  @param PidInfo Pointer to the PIDControllerInfo structure
 *  @param Kp Controller gain
 *  @param Ti Reset time in seconds
 *  @param Td Derivative time in seconds
 *  @param Tf Filter time constant in seconds
 *  @param Ts Sample time in seconds
 *
 *  @returns true, if parameters were set, false otherwise.
 */
bool pid_para_set(PIDControllerInfo* PidInfo, float Kp, float Ti, float Td, float Tf, float Ts)
{
    if (Kp <= 0.0f || Ti < 0.0f || Td < 0.0f || Tf < 0.0f || Ts <= 0.0f || (Ts >= Tf && Tf != 0.0f))
    {
        return false;
    }

    // Set parameters
    PidInfo->Kp = Kp;
    PidInfo->Ti = Ti;
    PidInfo->Td = Td;
    PidInfo->Tf = Tf;
    PidInfo->Ts = Ts;

    pid_util_update_coeff(PidInfo, true);

    return true;
}


/** Sets the upper and lower bound of the actuating value (output of controller)
 *
 *  @param PidInfo Pointer to the PIDControllerInfo structure
 *  @param mMin Lower bound
 *  @param mMax Upper bound
 */
bool pid_limits_set(PIDControllerInfo* PidInfo, float mMin, float mMax)
{
    if (mMin >= mMax) return false;

    PidInfo->mMin = mMin;
    PidInfo->mMax = mMax;

    return true;
}


/** Activates or deactivates the anti-windup mechanism of the PID controller
 *
 *  @param PidInfo Pointer to the PIDControllerInfo structure
 *  @param Arw true, if ARW is supposed to be active, false otherwise
 */
void pid_arw_set(PIDControllerInfo* PidInfo, bool Arw)
{
    PidInfo->Arw = Arw;
}


/** Performs a compute step of the PID controller.
 *
 *  @param PidInfo Pointer to the PIDControllerInfo structure
 *  @param e The current control deviation
 *  @param m Pointer to a float variable, which the computed actuation value will be written to
 */
void pid_execute(PIDControllerInfo* PidInfo, float e, float* m)
{
    // Shift arrays m and e by one to free the first index
    int i;
    for (i = 2; i > 0; i--)
    {
        PidInfo->m[i] = PidInfo->m[i - 1];
        PidInfo->e[i] = PidInfo->e[i - 1];
        PidInfo->eFil[i] = PidInfo->eFil[i - 1];
    }

    // Set current values for e and eFil
    PidInfo->eFil[0] = PidInfo->Cf[0] * PidInfo->e[1] + PidInfo->Cf[1] * PidInfo->eFil[1];
    PidInfo->e[0] = e;

    // Update coefficients for filter and PID controller
    pid_util_update_coeff(PidInfo, false);

    // Determine if filter and i-part are active
    bool IPartActive = pid_util_i_part_active(PidInfo);
    bool FilterActive = pid_util_filter_active(PidInfo);

    // Use filtered or measured e-values depending on filter usage
    float (*eCalc)[3] = FilterActive ? PidInfo->eFil : PidInfo->e;

    // Calculate controller output and write to actuation value history for next execution step
    PidInfo->P = PidInfo->Kp * (*eCalc)[0];
    PidInfo->I = IPartActive ? PidInfo->I + PidInfo->Kp * PidInfo->Ts / (2.0f * PidInfo->Ti) * ((*eCalc)[0] + (*eCalc)[1]) : PidInfo->I;
    PidInfo->D = PidInfo->Kp * PidInfo->Td * ((*eCalc)[0] - (*eCalc)[1]) / PidInfo->Ts;

    PidInfo->m[0] = PidInfo->m[1] + PidInfo->C[0] * (*eCalc)[0] + PidInfo->C[1] * (*eCalc)[1] + PidInfo->C[2] * (*eCalc)[2];

    // Limit output to control circuit using given bounds
    *m = pid_util_max(pid_util_min(PidInfo->m[0], PidInfo->mMax), PidInfo->mMin);
}

/** Updates the control and filter algorithm's coefficients when needed.
* 
* @param PidInfo Pointer to the PIDControllerInfo structure
* @param ForceCalculation true: force recomputing coefficients, false: recompute only if something has changed
*/
void pid_util_update_coeff(PIDControllerInfo* PidInfo, bool ForceCalculation)
{
    bool IPartActive = pid_util_i_part_active(PidInfo);
    bool FilterActive = pid_util_filter_active(PidInfo);

    // Check if IPartActive or FilterActive has changed to only recompute coefficients when needed, unless calculation is forced
    if (PidInfo->IPartActivePrev == IPartActive && PidInfo->FilterActivePrev == FilterActive  && (ForceCalculation == false))
    {
        PidInfo->IPartActivePrev = IPartActive;
        PidInfo->FilterActivePrev = FilterActive;
        return;
    }

    PidInfo->IPartActivePrev = IPartActive;
    PidInfo->FilterActivePrev = FilterActive;

    // Compute coefficients with / without Ki depending on whether IPartActive
    float Ki = IPartActive ? PidInfo->Kp * PidInfo->Ts / (2.0f * PidInfo->Ti) : 0.0f;
    float Kd = PidInfo->Kp * PidInfo->Td / PidInfo->Ts;

    PidInfo->C[0] = PidInfo->Kp + Ki + Kd;
    PidInfo->C[1] = -PidInfo->Kp + Ki - 2.0f * Kd;
    PidInfo->C[2] = Kd;

    // Compute filter coefficients
    PidInfo->Cf[0] = PidInfo->Tf == 0.0f ? 0.0f : PidInfo->Ts / PidInfo->Tf;
    PidInfo->Cf[1] = PidInfo->Tf == 0.0f ? 0.0f : 1.0f - PidInfo->Ts / PidInfo->Tf;
}

/**
* 
* @param PidInfo Pointer to the PIDControllerInfo structure
* 
* @returns whether the I part should be computed or not depending on whether ARW is used and bounds are exceeded.
*/
bool pid_util_i_part_active(PIDControllerInfo* PidInfo)
{
    // !(Ti==0 || ARW active && bounds exceeded)
    return !(PidInfo->Ti == 0.0f || PidInfo->Arw && (PidInfo->m[0] >= PidInfo->mMax || PidInfo->m[0] <= PidInfo->mMin));
}

/**
* 
* @param PidInfo Pointer to the PIDControllerInfo structure
* 
* @returns whether the filter is active depending on whether Tf is 0 or not.
*/
bool pid_util_filter_active(PIDControllerInfo* PidInfo)
{
    // Tf != 0 ?
    return PidInfo->Tf != 0.0f;
}

/**
* @param a first number for comparison.
* @param b second number for comparison.
* 
* @returns a: if a > b, b: otherwise
*/
float pid_util_max(float a, float b)
{
    return a > b ? a : b;
}


/**
* @param a first number for comparison.
* @param b second number for comparison.
*
* @returns a: if a < b, b: otherwise
*/
float pid_util_min(float a, float b)
{
    return a < b ? a : b;
}