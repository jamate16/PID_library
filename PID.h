#pragma once

#include <cmath>

#include "pico/stdlib.h"

enum class PIDType {
    P_pole_placement_closed_loop_plant,
    PI_pole_placement_1st_order_plant,
    PID_pole_placement_2nd_order_plant,
    P_manual_tunning,
    PD_manual_tunning,
    PI_manual_tunning,
    PID_manual_tunning
};

class PIDController {
public: // Public attributes
    PIDType PID_type;

    float kp_bias; /// Bias for P-controller
    float kp; /// Proportional term gain.
    float ki; /// Integral term gain.
    float kd; /// Derivative term gain (careful, usually is not needed because the signal from usually encoders has zero to no noise).
    float N; /// Derivative filter coefficient. Use only when derivative term gain is different from zero. \todo implement PID with filter on the derivative term

private: // Private attributes
    float last_error;
    float last_output;
    float last_integral;

public: // Public methods

    PIDController(PIDType type, float kp, float kp_bias=0);
    PIDController(PIDType type, float kp, float kd);
    /**
     * @brief This function creates a new PIDController object that automatically calculates the controller 
     * constants based on the given first-order motor model and desired response parameters.
     * The input to the model of the motor has to be Duty cycle of a PWM signal.
     * 
     * @param type For calling this constructor the type has to be PIDType::PI_pole_placement_1st_order_plant.
     * @param K Gain of the first order system representing the motor.
     * @param tau [s]. Time constant of the first order system representing the motor.
     * @param tss [s]. Desired settling time for the controlled system.
     * @param OS [%]. Desired overshoot for the controlled system.
     */
    PIDController(PIDType type, float K, float tau, float tss, float OS);

    /**
     * @brief This function creates a new PIDController object that automatically calculates the controller 
     * constants based on the given second-order motor model and desired response parameters.
     * The input to the model of the motor has to be Duty cycle of a PWM signal.
     * 
     * @param type For calling this constructor the type has to be PIDType::PID_pole_placement_2nd_order_plant.
     * @param K Gain of the second order system representing the motor.
     * @param zeta Dampening ratio of the second order system representing the motor.
     * @param wn Undamped natural frequency of the second order system representing the motor.
     * @param tss [s]. Desired settling time for the controlled system.
     * @param OS [%]. Desired overshoot for the controlled system.
     */
    PIDController(PIDType type, float K, float zeta, float wn, float tss, float OS);

    /**
     * @brief 
     * 
     * @param error (sp-pv). Difference between desired motor output and actual motor output.
     * @return DC [%]. Duty cycle of the PWM. The magnitude dictates strength of movement, the sign dictates the direction.
     */
    float calculateControl(float error);

    float getError();

private: // Private methods

    float pCalculateControl(float error);
    float piCalculateControl(float error, float delta_time);
    float pidPpCalculateControl(float error, float delta_time);
    float pidMCalculateControl(float error, float delta_time);

};
