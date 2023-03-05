#pragma once

class PIDController {
public: // Public attributes
    

private: // Private attributes
    float last_error;
    float last_output;
    float last_integral;

public: // Public methods
    PIDController();

    calculateControl(float error);
};
