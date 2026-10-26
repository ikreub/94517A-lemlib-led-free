#include "main.h"
#include "autons.hpp"
#include "pros/misc.h"
#include "subsystems.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

int page = 0; // page number for the brain screen

int output;

const int MAX_PAGES = 3;

void on_left_button() {
    page--;
    for (int i = 0; i < 8; i++){
        pros::lcd::clear_line(i);
    }
    if (page < 0) {
        page = MAX_PAGES;
    }
}

void on_right_button() {
    page++;
    for (int i = 0; i < 8; i++){
        pros::lcd::clear_line(i);
    }
    if (page > MAX_PAGES) {
        page = 0;
    }
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    pros::lcd::register_btn0_cb(on_left_button); // register left button callback
    pros::lcd::register_btn2_cb(on_right_button); // register right button callback
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    controller.rumble(".");
    Lift.init();
    Lift.disable();
    pros::delay(500);
    controller.rumble(".");
    pros::Task screen_task([&]() {
        while (true) {
            move_lift();
            // print robot location to the brain screenlift_control();
            switch (page)
            {
            case 0:
                pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
                pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
                pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
                break;
            case 1:
                pros::lcd::print(0, "safe left", nullptr);
                break;
            case 2:
                pros::lcd::print(0,"safe right", nullptr);
                break;
            case 3:
                pros::lcd::print(0, "skills", nullptr);
            // delay to save resources
            }
            switch(p){
                case 0:
                pros::lcd::print(6, "greater than 350");
                break;
                case 1:
                pros::lcd::print(6, "less than 260");
                break;
                case 2:
                pros::lcd::print(6, "no motor power");
                break;
            }
            switch(p2){
                case 0:
                pros::lcd::print(7, "auto claw off");
                break;
                case 1:
                pros::lcd::print(7, "claw resistance");
                break;
                case 2:
                pros::lcd::print(7, "claw released");
                break;
            }
            if(!Lift.get_state()) lift_control();
            pros::delay(50);
        }
    });
    pros::Task mpc([&]{Lift.mpc_control();});
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    lift.move(-10);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    switch (page) {
    case 0:
        lift_test();
        break;
    case 1:
        left2_2();
        break;
    case 2:
        right2_2();
        break;
    case 3:
        skills();
        break;
    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    bool auto_clamp = false;
    int ac_timer = 0;
    Wrist.set(false);
    // loop forever
    while (true) {
        // get left y and right x positions
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move(127);
            claw_right.move(127);
            claw_left.move(127);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move(-127);
            claw_right.move(-127);
            claw_left.move(-127);
        }else{
            intake.move(0);
            claw_right.move(0);
            claw_left.move(0);
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            voltage = 127;
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            voltage = -127;
        }else{
            voltage = 0;
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            Claw.toggle();
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            Wrist.set(true);
            intake.move(127);
            claw_right.move(127);
            claw_left.move(127);
        }else{
            Wrist.set(false);
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            auto_clamp = true;
            ac_timer = 0;
        }

        if(auto_clamp){
            cintake(127);
            if(ac_timer < 4500 || Claw.get() == false){
                if(claw_dist.get_distance() < 50){
                    Claw.set(true);
                    ac_timer = 4500;
                }else{
                    Claw.set(false);
                }
            }
            ac_timer += 20;
            if(ac_timer >= 5000){
                auto_clamp = false;
            }
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) & controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            autonomous();
        }

        if(lift_tracker.get_position() > 3688){
            rightY = .9 * rightY;
        }else if(lift_tracker.get_position() > 4688){
            rightY = .7 * rightY;
        }

        // move the robot
        chassis.arcade(rightY, leftX);

        // delay to save resources
        pros::delay(20);
    }
}