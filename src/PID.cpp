#include "PID.hpp"

double max(double a, double b){
    return a > b ? a : b;
}

double min(double a, double b){
    return a < b ? a : b;
}

PID::PID(double Kp, double Ki, double Kd, double I_alloc){
    this->Kd = Kd;
    this->Ki = Ki;
    this->Kp = Kp;
    this->I_alloc = I_alloc;
    this->last_pos = 0;
    this->target = 0;
    this->ierror = 0;
}

void PID::set_target(double target){
    this->target = target;
}

double PID::update(double Position){
    double error = target - Position;
    double derror = (Position - last_pos);
    ierror += error;
    if(ierror * error < 0){
        ierror = 0;
    }
    last_pos = Position;

    return min(127, max(-127, Kp * error + min(127 * I_alloc, max(-127 * I_alloc, Ki * ierror)) - Kd * derror));
}