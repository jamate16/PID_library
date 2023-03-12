#include "PID.h"

PIDController::PIDController(float max_output) : MAX_OUTPUT {max_output} {
    sp_threshold = 0; // Value by default

    last_error = 0;
    last_integral = 0;
    last_derivative = 0;
    last_output = 0;
}

// [kp, kd, ki, N, kp_bias, K, tau, zeta, wn, tss, OS]
// P_manual_tunning params array  = [kp]
// PD_manual_tunning params array = [kp, kd]
// PI_manual_tunning params array = [kp, ki]
// PI_pole_placement_1st_order_plant = [K, tau, tss, OS]

void PIDController::setGains(PIDType type, float *params) {
    switch (type) {
        case PIDType::P_manual_tunning:
            kp = params[0];
            ki = 0;
            kd = 0;
            break;
        case PIDType::PD_manual_tunning:
            kp = params[0];
            ki = 0;
            kd = params[1];
            break;
        case PIDType::PI_manual_tunning:
            kp = params[0];
            ki = params[1];
            kd = 0;
            break;
        case PIDType::PID_manual_tunning:
            kp = params[0];
            ki = params[1];
            kd = params[2];
            break;
        case PIDType::P_pole_placement_closed_loop_plant:

            break;
        case PIDType::PI_pole_placement_1st_order_plant:
            params[3] /= 100; // Overshoot is passed in percentage, need it as a decimal
            float zeta = std::sqrt((std::pow(std::log(params[3]), 2)) / (std::pow(M_PI, 2) + std::pow(std::log(params[3]), 2)));
            float wn = 4 / (zeta*params[2]); // 2% criterion. 98% of steady state value

            // Controller constants based on pole placement
            kp = (2*zeta*wn*params[1] - 1) / params[0];
            ki = params[1]*std::pow(wn, 2) / params[0];
            break;
    }
}

float PIDController::calculateControl(float error) {
    static absolute_time_t last_time {get_absolute_time()};
    absolute_time_t current_time = get_absolute_time();
    float delta_time = absolute_time_diff_us(current_time, last_time) * 1e-6f; // [s]

    // Guard clause, in case the user defines a band around the set point and it's achieved don't do anything
    if (at_sp = ((-sp_threshold < error) && (error < sp_threshold))) return last_output;

    // Handy calculations to make the code below more readable
    float error_sum = error + last_error;
    float error_subtraction = error - last_error;

    float proportional_term = kp * error;
    float integral_term = ki * .5f * delta_time * error_sum + last_integral; // Tustin approximation of integral term
    // float derivative_term = kd / (.5f * delta_time) * error_subtraction - last_derivative; // Tustin approximation of derivative term
    float derivative_term = kd * N / (.5f * N * delta_time + 1) * error_subtraction - (.5f * N * delta_time - 1) / (.5f * N * delta_time + 1) * last_derivative; // Tustin approximation of derivative term with filter

    // Clamp integral term. This acts as anti-windup, but in the case of a big disturbance the proportional term can be large too. So it's necesary to clamp the output as well. This way once the disturbance is removed the output isn't some biiiig value.
    integral_term = clamp(integral_term);

    float output = proportional_term + integral_term + derivative_term;

    // Clamp output.
    output = clamp(output);

    // Save values for next pass.
    last_output = output;
    last_error = error;
    last_integral = integral_term;
    last_derivative = derivative_term;
    last_time = current_time;

    return output;
}

inline float PIDController::clamp(float value) {
    if (value > MAX_OUTPUT) return MAX_OUTPUT;
    if (value < -MAX_OUTPUT) return -MAX_OUTPUT;

    return value;
}

void PIDController::changeSPThreshold(float threshold) {
    sp_threshold = threshold;
}

float PIDController::getError() {
    return last_error;
}
