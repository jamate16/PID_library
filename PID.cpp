#include <cmath>

#include "PID.h"

PIDController::PIDController(PIDType type, float kp, float kp_bias) 
    : kp {kp}, kp_bias {kp_bias} {} /// \todo implement pole placement part based on control theory

PIDController::PIDController(PIDType type, float K, float tau, float tss, float OS) {
    OS /= 100; // Overshoot is passed in percentage, need it as a decimal
    float zeta = std::sqrt((std::pow(std::log(OS), 2)) / (std::pow(M_PI, 2) + std::pow(std::log(OS), 2)));
    float wn = 4 / (zeta*tss); // 2% criterion. 98% of steady state value

    // Controller constants based on pole placement
    kp = (2*zeta*wn*tau - 1) / K;
    ki = tau*std::pow(wn, 2) / K;

    // q0 = -kp;
    // q1 = kp+ki*Ts;
}

float PIDController::calculateControl(float error) {
    static absolute_time_t last_time {get_absolute_time()};
    absolute_time_t current_time = get_absolute_time();
    float delta_time = absolute_time_diff_us(current_time, last_time) * 1e-6f;

    float output {100};
    switch (PID_type) {
        case PIDType::P_pole_placement_closed_loop_plant:
            output = pCalculateControl(error);
            break;
        case PIDType::PI_pole_placement_1st_order_plant:
            output = piCalculateControl(error, delta_time);
            break;
        case PIDType::PID_pole_placement_2nd_order_plant:
            output = pidPpCalculateControl(error, delta_time);
            break;
        case PIDType::PID_manual_tunning:
            output = pidMCalculateControl(error, delta_time);
            break;
    }

    last_output = output;
    last_error = error;
    last_time = current_time;
    return output;
}

float PIDController::pCalculateControl(float error)
{
    float output = kp*error + 15*(error - last_error) / 20e-3;

    return output; /// \todo Implement clamping 'n stuff on the control signal
}

float PIDController::piCalculateControl(float error, float delta_time)
{
    /// \todo Separate each term in order to implement anti-windup.
    // Controller transfer function: U(s)/E(s) = kp + ki/s
    static float q0 = -kp;
    static float q1 = kp + ki*delta_time;

    float output = q1*error + q0*last_error + last_output;

    return output;
}

float PIDController::pidPpCalculateControl(float error, float delta_time) {
    return 0.0f;
}

float PIDController::pidMCalculateControl(float error, float delta_time) {
    return 0.0f;
}

float PIDController::getError() {
    return last_error;
}
