/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2024, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
#include "lemlib/api.hpp"
//#include "okapi/api.hpp"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>

// drivetrain motor configuration
// left side motors (ports 1, 2, 3)
inline pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGearset::blue);
// right side motors (ports 4, 8, 10)
inline pros::MotorGroup right_motors({4, 8, 10}, pros::MotorGearset::blue);

// intake/outtake motor configuration
inline pros::Motor front_bottom(7, pros::MotorGearset::blue);  // Front bottom motor
inline pros::Motor middle(5, pros::MotorGearset::green);        // Middle motor
inline pros::Motor back_top(6, pros::MotorGearset::green);      // Back top motor

// pneumatics
inline pros::adi::DigitalOut gutter('C');       // gutter/match loader pneumatic on port C
inline pros::adi::DigitalOut pneumatic_d('D');  // pneumatic on port D

// sensors
inline pros::Optical color_sensor(20);  // color sensor on port 20 for ball detection
inline pros::Imu imu(13);               // IMU on port 13
inline pros::adi::Encoder horizontal_encoder('A', 'B', false);  // horizontal tracking wheel (X-axis)
inline pros::adi::Encoder vertical_encoder('G', 'H', false);    // vertical tracking wheel (Y-axis)

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// horizontal tracking wheel (perpendicular, measures X/strafe)
// offset: -0.5" (0.5" to the left of center)
inline lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_325, -0.5);

// vertical tracking wheel (parallel, measures Y/forward-back)  
// offset: 0.5" (0.5" to the right of center)
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_325, 0.5);

// drivetrain configuration
inline lemlib::Drivetrain drivetrain(
    &left_motors,                // left motor group
    &right_motors,               // right motor group
    11,                          // track width (inches)
    lemlib::Omniwheel::NEW_325,  // wheel type
    600,                         // gear ratio (RPM)
    2                            // horizontal drift (for omni wheels)
);

// lateral PID controller (for moving forward/backward)
inline lemlib::ControllerSettings lateral_controller(
    10,   // kP
    0,    // kI
    3,    // kD
    3,    // anti-windup
    1,    // small error range (inches)
    100,  // small error timeout (ms)
    3,    // large error range (inches)
    500,  // large error timeout (ms)
    20    // maximum acceleration (slew)
);

// angular PID controller (for turning)
inline lemlib::ControllerSettings angular_controller(
    2,    // kP
    0,    // kI
    10,   // kD
    3,    // anti-windup
    1,    // small error range (degrees)
    100,  // small error timeout (ms)
    3,    // large error range (degrees)
    500,  // large error timeout (ms)
    0     // maximum acceleration (slew)
);

// odometry configuration
inline lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,    // vertical tracking wheel
    nullptr,                     // second vertical tracking wheel (none)
    &horizontal_tracking_wheel,  // horizontal tracking wheel
    nullptr,                     // second horizontal tracking wheel (none)
    &imu                         // inertial sensor
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

#endif

#endif  // _PROS_MAIN_H_
