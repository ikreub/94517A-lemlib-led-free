#include "pathgen.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <algorithm>
#include <iterator>
#include <iostream>

const double radtopos = 18000 / M_PI;

lift_mpc::lift_mpc(pros::Rotation& sensor, pros::MotorGroup& motors, double Lift_len, double trans_len, double End_mass, Motor motor_consts, double dt) : rot_sen(sensor), motors(motors){
    this->Lift_len = Lift_len;
    this->trans_len = trans_len;
    this->End_mass = End_mass / 386.22;
    Index = 0;
    last_pos = 0;
    mpc_active = true;
    path = {{.height = -.48, .velocity = 0}};
    this->dt = dt;
    mconsts = motor_consts;
}

void lift_mpc::init(){
    motors.move(-127);
    pros::delay(200);
    motors.move(-60);
    while(fabs(rot_sen.get_position() - last_pos) >= 20){
        last_pos = rot_sen.get_position();
        pros::delay(20);
    }
    rot_sen.set_position(-.48 * radtopos);
    motors.move(0);
}

void lift_mpc::enable(){
    mpc_active = true;
}

void lift_mpc::disable(){
    mpc_active = false;
}

double lift_mpc::heightToAngle(double height){
    return asin((height - trans_len) / (2 * Lift_len));
}

std::vector<Path> lift_mpc::PathGen(double Start, double End, double max_speed){
    // 1. Double check for NaN or extreme bounds from heightToAngle
    if (std::isnan(End) || std::isinf(End)) {
        std::cout << "[MPC ERROR]: Target angle is NaN! Check lift geometry constants." << std::endl;
        return {{Start, 0.0}}; 
    }

    double dir = (Start <= End ? 1: -1);
    double pf = End;
    double vf = 0;
    std::vector<Path> result = {};

    while(dir * pf > dir * Start){
        result.push_back({pf, vf});
        pf = pf - vf * dt;
        vf = std::clamp((vf * mconsts.Ra * MMT_inertia(pf) + dir * mconsts.KT * dt * 12) / (1 - mconsts.KE * mconsts.KT * dt), -max_speed, max_speed);
        
        if (result.size() > 250) { 
            std::cout << "[MPC WARNING]: Maximum velocity profile hit array ceiling limit." << std::endl;
            break; 
        }
    }

    std::reverse(result.begin(), result.end());

    return result;
}

void lift_mpc::set_target(double target, double max_speed){
    bool was_active = mpc_active;
    mpc_active = false;
    std::cout << "\nSetting New Path | target: " << target << std::endl;
    max_speed = std::clamp(max_speed, 0.0, 1.0);
    
    path = PathGen(rot_sen.get_position() / radtopos, heightToAngle(target), max_speed * 1.1 * M_PI);
    Index = 0;

    for (int i = 0; i < path.size(); ++i){
        std::cout << "Index: " << i << " | position: " << path[i].height << " | velocity: " << path[i].velocity << "\n";
    }
    std::cout << "\n Resume mpc";
    
    mpc_active = was_active;
}

double lift_mpc::MMT_inertia(double pos){
    return Lift_len * Lift_len * (0.00001476 * Lift_len + 0.00001476 * trans_len + 0.00011808 + 2 * (0.00001476 * Lift_len + 2 * End_mass) * cos(pos) * cos(pos)) / 11211.19;
}

std::vector<double> dif(std::vector<Path> path, double pos){
    std::vector<double> p = {};
    for (int i = 0; i < path.size(); ++i){
        p.push_back(fabs(pos - path[i].height));
    }

    return p;
}

double sign(double x){
    return x < 0 ? -1 : x > 0 ? 1 : 0;
}

double lift_mpc::Vbase(double vel_t, double vel_c){
    return std::clamp((mconsts.KE * vel_c + sign(vel_t) * sqrt(mconsts.KE * mconsts.KE * vel_t * vel_t + fabs(4 * vel_t * mconsts.Ra))) / 2, -12.0, 12.0);
}

double lift_mpc::eprop(double pos_t, double pos_c){
    return (pos_t - pos_c) * sign(fmax(0, fabs(pos_t - pos_c) - mconsts.dband / 2));
}

void lift_mpc::mpc_control(){
    double Kp = 10;
    double Kv = 2.3;

    while(true){
        double C_pos = rot_sen.get_position() / radtopos;

        double C_vel = rot_sen.get_velocity() / radtopos;
        if(mpc_active){

            std::vector<double> difs = dif(path, C_pos);
            auto search_start = difs.begin() + std::min((int)Index, (int)difs.size() - 1);
            auto min_val = std::min_element(search_start, difs.end());

            Index = std::min((int)std::distance(difs.begin(), min_val), (int)(path.size() - 1));

            double t_vel = path[Index].velocity;
            double t_pos = path[Index].height;
            std::cout << "index: " << Index << " | t_vel: " << t_vel << " | t_pos: " << t_pos << std::endl;

            double V = std::clamp(Vbase(t_vel, C_vel) + Kp * eprop(t_pos, C_pos) + Kv * (t_vel - C_vel),-12.0,12.0);

            std::cout << "Voltage: " << V << std::endl;
            motors.move(V * 127 / 12);
        }
        pros::delay(1000 * dt);
    }
}

bool lift_mpc::get_state(){
    return mpc_active;
}