#pragma once
#include "../include/lemlib/api.hpp"
#include "api.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/rotation.hpp"
#include "PID.hpp"
#include <numeric>
#include <vector>
#include <functional>
#include "pathgen.hpp"

class piston{
    public:
    inline piston(pros::adi::DigitalOut& piston, bool reversed = false) : Piston(piston), reversed(reversed) {
        state = false;
    }
    inline void toggle(){
        state = !state;
        Piston.set_value(state ^ reversed); 
    }
    inline void set(bool state){
        this->state = state;
        Piston.set_value(state ^ reversed);
    }
    inline const bool get(){
        return state;
    }
    private:
    pros::adi::DigitalOut& Piston;
    bool state;
    bool reversed;

};

inline pros::MotorGroup left_motors({-19, -17}, pros::MotorGearset::blue);
inline pros::MotorGroup right_motors({16, 14}, pros::MotorGearset::blue); 

inline lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.5, lemlib::Omniwheel::NEW_325, 480, 2);

inline pros::IMU imu(9);
inline pros::Rotation vert(-4);
inline pros::Rotation horiz(-11);

inline pros::Motor intake(2);

inline pros::Distance left(6);
inline pros::Distance back_left(1);
inline pros::Distance right (5);
inline pros::Distance back_right(21);
inline pros::Distance claw_dist(10);

inline lemlib::TrackingWheel Vert(&vert, lemlib::Omniwheel::NEW_2, .3);
inline lemlib::TrackingWheel Horiz(&horiz, lemlib::Omniwheel::NEW_2, 3);

inline lemlib::OdomSensors sensors(&Vert, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &Horiz, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
inline lemlib::ControllerSettings lateral_controller(13.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              70, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              4, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
inline lemlib::ControllerSettings angular_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              60, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              80, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              200, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

inline lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors 
);

inline pros::Rotation lift_tracker(-13);

//void run_auton(int auton_number);

//inline std::vector<std::function<void(void)>> autons;

inline pros::Mutex screen_mutex;

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

inline pros::Motor left_lift(3);
inline pros::Motor right_lift(-15);

inline pros::MotorGroup lift({3, -15});
inline pros::Motor claw_right(-8);
inline pros::Motor claw_left(7);

inline lift_mpc Lift(lift_tracker, lift, 12.5, 9.5, 3, {.Ra = 3.8, .KT = 24 / (1.1 * M_PI), .KE = 12 / (1.1 * M_PI), .dband = 0.169});

inline pros::adi::DigitalOut claw('A');
inline piston Claw(claw);
inline pros::adi::DigitalOut wrist('B');
inline piston Wrist(wrist);

inline PID lift_pid(2.515642, 0.134392, 8.281429, 0.1);
//8 17

inline double lift_target = 360;
inline int i = 0;

inline int p = 0;
inline int p2 = 0;

inline double voltage = 0;

inline void wait_until_claw(int timeout){
    double time = 0;
    while(Claw.get() == true){
        pros::delay(20);
        time += 20;
        if(time > timeout){
            Claw.set(false);
        }
    }
}

inline void wait_until_dsen(int timeout, int dist = 20){
    while(claw_dist.get_distance() > dist){
        pros::delay(20);
        timeout -= 20;
        if(timeout <= 0){
            break;
        }
    }
}


inline void wait_until_button(){
    while(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        pros::delay(50);
    }
}

inline void move_lift(){
    double fvoltage = voltage;
    if(lift_tracker.get_position() < -1750){
        p = 0;
        p2 = 0;
        if(fvoltage == 0){
            fvoltage = -10;
        }
    }else if(lift_tracker.get_position() > 4688){
        p=1;
        if(fvoltage  == 0){
            fvoltage = 20;
        }
        if(abs(lift_tracker.get_velocity()) <= 200 & fvoltage  < 0 & Claw.get() == true){
            i++;
            p2 = 1;
            if(i > 12){
                p2 = 2;
                Claw.set(false);
                i = 0;
            }
        }else{
            i = 0;
            p2 = 0;
        }
    }else{
        p = 2;
        if(abs(lift_tracker.get_velocity()) <= 300 * fvoltage / 127 & fvoltage  < 0 & Claw.get() == true){
            i++;
            p2 = 1;
            if(i > 12){
                p2 = 2;
                Claw.set(false);
                i = 0;
            }
        }else{
            i = 0;
            p2 = 0;
        }
    }
    if(claw_dist.get_distance() < 30){
        fvoltage += 20;
    }
    lift.move(fvoltage);
}

const double d = 0.7 * 127;

inline double dir = 1;

inline double error = 0;
inline double Perror = 0;

inline double sign(double x){
    if(x >= 0){
        return 1;
    }else if(x < 0){
        return -1;
    }else{
        return 0;
    }
}

inline std::vector<double>  lift_poses = {};
inline std::vector<double>  lift_times = {};

inline double t = 0;

inline double Ppose = 360;
inline double Pvelocity = 0;

// inline void lift_control(){
//     lift_target = 300;
//     double Current_p = lift_tracker.get_position() / 100.0;
//     if(i <= 40){
//         t+=0.05;
//         // i++;
//         // pros::lcd::print(7, "Lift Target: %f : %f", lift_target, i);
//         double Current_p = lift_tracker.get_position() / 100.0;
//         double velocity = (Current_p - Ppose);
//         Perror = error; 
//         error = lift_target - Current_p;
        
//         if(error * Perror < 0){
//             dir = -dir;
//         }
//         if(velocity * Pvelocity < 0){
//             lift_poses.push_back(fabs(lift_target - Ppose));
//             lift_times.push_back(t);
//             t = 0;
//             i++;
//         }
//         left_lift.move(dir*d);
//         right_lift.move(dir*d);


//         Ppose = Current_p;
//         Pvelocity = velocity;
//     }else{
//         left_lift.move(0);
//         right_lift.move(0);
//         double ku = std::accumulate(lift_poses.begin() + 5, lift_poses.end(), 0.0) / (lift_poses.size() - 5);
//         double Ku = 4 * d / (3.14159 * ku);
//         double Pu = 2 * std::accumulate(lift_times.begin() + 5, lift_times.end(), 0.0) / (lift_times.size() - 5);

//         double Kp = 0.2 * Ku;
//         double Ki = 0.05 * Kp / (0.5 * Pu);
//         double Kd = 0.33 * Kp * Pu / 0.05;


//         pros::lcd::print(6, "%f : %f", Ku, Pu);
//         pros::lcd::print(7, "PID: %f : %f : %f", Kp, Ki, Kd);
        
//     }
// }


inline void lift_control(){
    // double Current_p = lift_tracker.get_position() / 100.0;
    // double output = lift_pid.update(Current_p);
    // left_lift.move(-output);
    // right_lift.move(-output);
}

inline void cintake(double voltage, bool same = false){
    claw_left.move(voltage);
    claw_right.move(voltage * (same ? -1 : 1));
}

