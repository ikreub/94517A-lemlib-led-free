#pragma once
#include "api.h"
#include <vector>

struct Path{
    double height;
    double velocity;
};

struct Motor{
    double Ra;
    double KT;
    double KE;
    double dband;
};

class lift_mpc{
    public:
    lift_mpc(pros::Rotation& sensor, pros::MotorGroup& motors, double Lift_len, double trans_len, double End_mass, Motor motor_consts, double dt = 0.02);

    std::vector<Path> PathGen(double Start, double End, double max_speed);

    void mpc_control();

    void enable();
    void disable();

    bool get_state();

    void init();
    void set_target(double target, double speed_percent = 1);

    double heightToAngle(double height);

    double MMT_inertia(double pos);

    double Vbase(double vel_t, double vel_c);

    double eprop(double pos_t, double pos_c);

    private:
    std::size_t Index;
    double Lift_len;
    double trans_len;
    double End_mass;

    double last_pos;

    pros::Rotation& rot_sen;
    pros::MotorGroup& motors;

    bool mpc_active;

    double dt;

    std::vector<Path> path;

    Motor mconsts;
};