#pragma once

class PID{
    public:
    PID(double Kp, double Ki, double Kd, double I_aloc = 0.1);

    double update(double Position);

    void set_target(double target);

    private:
    double Kp;
    double Ki;
    double Kd;
    double I_alloc;
    double target;
    double last_pos;
    double ierror;
};