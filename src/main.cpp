#include "main.h"

// intake control functions
void intake_store() {
    front_bottom.move(-127);  // Counter-clockwise to intake
    
    // Check if color sensor detects a ball
    if (color_sensor.get_proximity() > 100) {
        middle.move(65);  // if theres a ball, move it up
    } else {
        middle.move(0);   // if theres no ball, stop middle
    }
    back_top.move(0);     // never run top motor during storage
}

void outtake_top() {
    front_bottom.move(-127);
    middle.move(127);
    back_top.move(127);
}

void outtake_middle() {
    front_bottom.move(-127);
    middle.move(127);
    back_top.move(-127);
}

void outtake_bottom() {
    front_bottom.move(127);
    middle.move(-127);
    back_top.move(0);
}

void stop_intake() {
    front_bottom.move(0);
    middle.move(0);
    back_top.move(0);
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Calibrating...");

	pros::lcd::register_btn1_cb(on_center_button);
	
	// calibrate sensors
	chassis.calibrate();
	
	// wait for IMU to finish calibrating
	pros::delay(2000);
	
	pros::lcd::set_text(1, "Ready!");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
void autonomous() {}

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
	// pneumatic states
	bool gutter_state = false;
	bool pneumatic_d_state = false;

	while (true) {
		// get joystick values
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		// drive the robot (arcade drive)
		chassis.arcade(leftY, rightX, false, 0.6);
		
		// pneumatic controls
		// right arrow: toggle gutter (match loader)
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			gutter_state = !gutter_state;
			gutter.set_value(gutter_state);
		}
		
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			pneumatic_d_state = !pneumatic_d_state;
			pneumatic_d.set_value(pneumatic_d_state);
		}
		
		// align to nearest 0 or 180 degrees (faster turn)
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			float current_heading = chassis.getPose().theta;
			while (current_heading < 0) current_heading += 360;
			while (current_heading >= 360) current_heading -= 360;
			
			// Find nearest between 0 and 180
			float diff_to_0 = std::abs(current_heading - 0);
			if (diff_to_0 > 180) diff_to_0 = 360 - diff_to_0;
			float diff_to_180 = std::abs(current_heading - 180);
			if (diff_to_180 > 180) diff_to_180 = 360 - diff_to_180;
			
			float nearest = (diff_to_0 < diff_to_180) ? 0 : 180;
			
			chassis.turnToHeading(nearest, 500, {.maxSpeed = 127}, false);
		}
		
		// reverse direction toggle (press A to flip front)
		int forward = leftY;
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			forward = -forward;
		}
		
		// intake/outtake controls
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake_store();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			outtake_top();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			outtake_middle();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			outtake_bottom();
		} else {
			stop_intake();
		}
		
		// brake mode control - hold when disconnected
		if (!controller.is_connected()) {
			chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
		} else {
			chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
		}
		
		pros::delay(20);  // Run for 20 ms then update
	}
}