#include "vex.h"
#include "autons.h"
#include "robot-config.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  // chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  // chassis.set_heading_constants(6, .4, 0, 1, 0);
  // chassis.set_turn_constants(6, 0.2, 0, 1.5, 5); //(3, 0.3, 0, 1, 5);
  // chassis.set_swing_constants(12, .3, .001, 2, 15);

  // // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  // chassis.set_drive_exit_conditions(1.5, 300, 5000);
  // chassis.set_turn_exit_conditions(1, 300, 3000);
  // chassis.set_swing_exit_conditions(1, 300, 3000);

  chassis.set_drive_constants(10, 0.4, 0, 1.3, 0);//0.4kd
  chassis.set_heading_constants(3, 0.5, 0, 0.00, 0);//(3, 1.1, 0, 0.01, 0);
  chassis.set_turn_constants(12, .4, .03, 3.5, 10);//3.18
  chassis.set_swing_constants(12, 0.3, .001, 2, 15); //(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 100, 5000);
  chassis.set_turn_exit_conditions(1, 100, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  // chassis.heading_max_voltage = 10;
  // chassis.drive_max_voltage = 8;
  // chassis.drive_settle_error = 3;
  // chassis.boomerang_lead = .5;
  // chassis.drive_min_voltage = 0;

  chassis.heading_max_voltage = 3;//
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = 1.5;//1.1
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  // chassis.drive_distance(6);
  // chassis.drive_distance(12);
  // chassis.drive_distance(18);
  // chassis.drive_distance(-36);
}

void tunepid(){
  // Add debug messages
    // float turn = 8.3;

    // trial for pid
    //******************************************************
    Brain.Screen.clearScreen();
    // Calibrate inertial first
    Brain.Screen.printAt(5, 20, "Calibrating...");
    //chassis.Gyro.calibrate();
    Inertial.calibrate();
    wait(1000, msec);
    // Test current PID values
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "PID Tune Test");
    Brain.Screen.printAt(5, 40, "Starting angle: %.1f", chassis.get_absolute_heading());
    wait(2000, msec);
    // Try to turn exactly 90 degrees
    chassis.set_turn_exit_conditions(5, 150, 2000);
    chassis.turn_to_angle(90);
    wait(2000, msec);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    // Show results
    double final_angle = chassis.get_absolute_heading();
    double error = 90.0 - final_angle;
    Brain.Screen.printAt(5, 60, "Final angle: %.1f", final_angle);
    Brain.Screen.printAt(5, 80, "Error: %.1f degrees", error);
    Brain.Screen.printAt(5, 100, "Target was: 90.0" );

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    // Controller1.Screen.print("Final: %.1f", final_angle);
    // Controller1.Screen.print("Final: %.1f", final_angle);
    // Controller1.Screen.setCursor(2, 1);
    // double a = 90-91;         // Testing purpose
    Controller1.Screen.print("Final angle: %.1f", final_angle);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("Error: %.1f degrees", error);
    // Wait so you can read results
    wait(1000, msec);
    // chassis.turn_to_angle(180);
    // double angle = chassis.get_absolute_heading();
    // Brain.Screen.printAt(5, 120, "Final angle: %.1f", angle);
    // Brain.Screen.printAt(5, 80, "Error: %.1f degrees", error);
    // Brain.Screen.printAt(5, 100, "Target was: 180.0 + turn offset = 180");
    // wait(1000, msec);
}

void drive_pid_test(){
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5, 20, "Calibrating...");
  Inertial.calibrate();
  wait(1000, msec);
  
  // Test current PID values
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5, 20, "Drive PID Test");
  Brain.Screen.printAt(5, 40, "Starting position");
  
  // Print to controller
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Drive Test Start");
  
  wait(2000, msec);
  
  // Reset motor positions to track distance
  LeftDrive.resetPosition();
  RightDrive.resetPosition();
  
  float target_distance = -10.0; // Drive 24 inches forward
  
  // Record starting positions
  float start_left = chassis.get_left_position_in();
  float start_right = chassis.get_right_position_in();
  
  Brain.Screen.printAt(5, 60, "Driving %.1f inches...", target_distance);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Driving %.1f in", target_distance);
  
  // Perform drive
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.drive_distance(target_distance);
  
  wait(500, msec);
  LeftDrive.stop(hold);
  RightDrive.stop(hold);
  
  // Calculate results
  float final_left = chassis.get_left_position_in();
  float final_right = chassis.get_right_position_in();
  float actual_distance = ((final_left - start_left) + (final_right - start_right)) / 2.0;
  float error = target_distance - actual_distance;
  
  // Show results on Brain
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5, 20, "Drive PID Test Results");
  Brain.Screen.printAt(5, 40, "Target: %.1f inches", target_distance);
  Brain.Screen.printAt(5, 60, "Actual: %.1f inches", actual_distance);
  Brain.Screen.printAt(5, 80, "Error: %.1f inches", error);
  Brain.Screen.printAt(5, 100, "Left: %.1f  Right: %.1f", final_left - start_left, final_right - start_right);
  
  if(fabs(error) < 0.5) {
    Brain.Screen.printAt(5, 120, "EXCELLENT!");
  } else if(fabs(error) < 1.5) {
    Brain.Screen.printAt(5, 120, "GOOD");
  } else {
    Brain.Screen.printAt(5, 120, "NEEDS TUNING");
  }
  
  // Show results on Controller
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Actual: %.1f in", actual_distance);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Error: %.1f in", error);
  Controller1.Screen.setCursor(3, 1);
  
  if(fabs(error) < 0.5) {
    Controller1.Screen.print("EXCELLENT!");
    // Controller1.rumble(".");
  } else {
    Controller1.Screen.print("Tune PID");
    // Controller1.rumble("-");
  }
  
  wait(3000, msec);
  
  // // Test driving backwards
  // Brain.Screen.clearScreen();
  // Brain.Screen.printAt(5, 20, "Testing backward drive...");
  // Controller1.Screen.clearScreen();
  // Controller1.Screen.setCursor(1, 1);
  // Controller1.Screen.print("Backward test");
  
  // wait(1000, msec);
  
  // LeftDrive.resetPosition();
  // RightDrive.resetPosition();
  
  // chassis.drive_distance(-target_distance);
  // wait(500, msec);
  
  // float back_actual = (chassis.get_left_position_in() + chassis.get_right_position_in()) / 2.0;
  // float back_error = -target_distance - back_actual;
  
  // Brain.Screen.printAt(5, 40, "Backward Error: %.1f in", back_error);
  // Controller1.Screen.setCursor(2, 1);
  // Controller1.Screen.print("Back Err: %.1f", back_error);
  
  // Brain.Screen.printAt(5, 60, "TEST COMPLETE!");
  // Controller1.rumble("..");
  
  // wait(3000, msec);
}
/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(45);
  chassis.drive_distance(-5);
  // chassis.right_swing_to_angle(-90);
  chassis.turn_to_angle(90);
  chassis.drive_distance(24);
  chassis.drive_distance(-10);
  chassis.turn_to_angle(0);
}

/**
 * Tests position tracking by driving in a square using coordinates
 */

void odom_square_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  
  // Drive in a 24x24 inch square
  chassis.drive_to_point(24, 0);    // Go right
  chassis.drive_to_point(24, 24);   // Go forward 
  chassis.drive_to_point(0, 24);    // Go left
  chassis.drive_to_point(0, 0);     // Return home
  chassis.turn_to_angle(0);         // Face original direction
}

void tuneodom(){
  odom_constants();
  chassis.set_coordinates(0,22,0);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5,20, "X: %.1f", chassis.get_X_position());
  Brain.Screen.printAt(5,40, "Y: %.1f", chassis.get_Y_position());
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("X: %.1f", chassis.get_X_position());
  Controller1.Screen.print("Y: %.1f", chassis.get_Y_position());
  wait(500, msec);
  chassis.turn_to_point(-22,22);
  wait(500, msec);
  chassis.drive_to_point(-22,22);
  LeftDrive.stop(hold);
  RightDrive.stop(hold);
  Brain.Screen.printAt(5,80, "X: %.1f", chassis.get_X_position());
  Brain.Screen.printAt(5,100, "Y: %.1f", chassis.get_Y_position());
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X: %.1f", chassis.get_X_position());
  Controller1.Screen.print("Y: %.1f", chassis.get_Y_position());
  wait(2000, msec);
  chassis.drive_to_point(0,22);
  LeftDrive.stop(hold);
  RightDrive.stop(hold);
  wait(500, msec);
  chassis.turn_to_angle(0);
  wait(500, msec);
  Brain.Screen.printAt(5,140, "X: %.1f", chassis.get_X_position());
  Brain.Screen.printAt(5,160, "Y: %,1f", chassis.get_Y_position());
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("X: %.1f", chassis.get_X_position());
  Controller1.Screen.print("Y: %.1f", chassis.get_Y_position());
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %.2f inches", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %.2f inches", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %.1f degrees", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %.2f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %.2f", chassis.get_SidewaysTracker_position());
    Brain.Screen.printAt(5,120, "Push robot around to test!");
    Brain.Screen.printAt(5,140, "Press Brain button to exit");
    
    if(Brain.Screen.pressing()) {
      break;
    }
    wait(100, msec);
  }
}

void randomtest(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  double initial = chassis.get_absolute_heading();
  Controller1.Screen.print("Initial: %.1f", initial);
  chassis.turn_to_angle(90);
  LeftDrive.stop(hold);
  RightDrive.stop(hold);
  double final = chassis.get_absolute_heading();
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Final: %.1f", final);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Diff: %.1f", final-initial);
  wait(5000,msec);
}
// void odom_test(){
//   chassis.set_coordinates(0, 0, 0);
//   while(1){
//     Brain.Screen.clearScreen();
//     Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
//     Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
//     Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
//     Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
//     Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
//     //task::sleep(20);
//   }
// }

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

/**
 * Advanced demonstration of ODOM capabilities
 * Shows off smooth curved paths and precise positioning
 */

void advanced_odom_demo(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  
  // Start intake while moving
  IntakeMotor.spin(forward, 75, percent);
  
  // Complex path demonstration
  chassis.drive_to_point(18, 12);      // Diagonal movement
  chassis.turn_to_point(36, 0);        // Turn to face new target
  chassis.drive_to_point(36, 0);       // Drive to target
  
  // Stop intake and reverse
  IntakeMotor.stop();
  wait(500, msec);
  IntakeMotor.spin(reverse, 50, percent);
  
  // Return via different path
  chassis.drive_to_point(18, 18);      // Go to corner
  chassis.turn_to_angle(225);          // Face back toward start
  chassis.drive_to_point(0, 0);        // Return home
  chassis.turn_to_angle(0);            // Face original direction
  
  IntakeMotor.stop();
}

/**
 * Custom autonomous routine template
 * Replace this with your actual competition autonomous
 */

void custom_auton(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  
  // Example competition autonomous routine
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(50, 100, "Running Custom Auton!");
  
  // Start intake
  IntakeMotor.spin(forward, 100, percent);
  
  // Drive forward to collect first object
  chassis.drive_distance(24);
  wait(1000, msec);  // Wait to collect
  
  // Turn and drive to scoring position  
  chassis.turn_to_angle(90);
  chassis.drive_distance(18);
  
  // Score (reverse intake)
  IntakeMotor.spin(reverse, 100, percent);
  wait(1500, msec);
  IntakeMotor.stop();
  
  // Return to start (using coordinates)
  chassis.drive_to_point(0, 0);
  chassis.turn_to_angle(0);
  
  Brain.Screen.printAt(50, 120, "Auton Complete!");
}

void ScoreLongGoals(){
  ScoreMotor.spin(forward);
  wait(1000,msec);
  ScoreMotor.stop();
}

void ScoreMiddleGoal(){
  ScoreMotor.spin(reverse);
  wait(1000,msec);
  ScoreMotor.stop();
}

void RightSide_HalfAWP(){
  chassis.set_coordinates(0, 0, 0);
  odom_constants();
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 1");
  wait(1000,msec);
  IntakeMotor.spin(reverse, 100, pct);
  //////////////////////////////////
  chassis.set_drive_exit_conditions(1.5, 100, 1050);//// 1101
  chassis.drive_distance(30);
  chassis.set_turn_exit_conditions(1, 100, 270);/////////300
  IntakeTilt.set(true);
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 2");
  wait(1000,msec);
  //////////////////////////////////
  chassis.turn_to_angle(270);
  // chassis.turn_to_point(-22,27);
  chassis.set_turn_exit_conditions(1, 100, 300);/////////
  IntakeMotor.spin(reverse, 100, pct);
  // IntakeMotor.spin(reverse, 20, pct);
  wait(100,msec);//200
  chassis.drive_to_point(-22,27);
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 3");
  wait(1000,msec);
  //////////////////////////////////
  chassis.set_drive_exit_conditions(1.5, 100, 600);
  chassis.drive_distance(10,90);
  chassis.drive_with_voltage(1, 1);//////////////////////
  wait(650,msec); //tal vez cambiar para no agarrar azul
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 4");
  wait(1000,msec);
  //////////////////////////////////
  chassis.set_drive_exit_conditions(1.5, 100, 700);/////////1000
  chassis.drive_distance(-35,90);
  IntakeTilt.set(false);
  chassis.drive_with_voltage(-2, -2);
  IntakeMotor.spin(reverse, 100, pct);
  ScoreLongGoals();
  wait(1400,msec);//1100
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 5");
  wait(1000,msec);
  //////////////////////////////////
  chassis.drive_with_voltage(12, 12);
  wait(150,msec); //200
  chassis.drive_with_voltage(12, -6);
  wait(200,msec);//300
  IntakeMotor.spin(forward, 100, pct);
  // IntakeMotor.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(1.5, 100, 800);//800
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 6");
  wait(1000,msec);
  //////////////////////////////////
  chassis.turn_to_point(-25,6);///-25,6
  chassis.drive_to_point(-25,6);
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 7");
  wait(1000,msec);
  //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  wait(1000,msec);          
  IntakeMotor.spin(reverse);        // Score
  wait(300,msec);  
  ////////////////////////////////////////////////////////////////////////////////
  chassis.set_drive_exit_conditions(1.5, 100, 5000);
  chassis.drive_to_point(-25,-41);
  chassis.set_turn_exit_conditions(1, 100, 200);///250
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 8");
  wait(1000,msec);
  //////////////////////////////////
  chassis.turn_to_angle(130);
  chassis.set_drive_exit_conditions(1.5, 100, 450);/////////////////////
  chassis.set_turn_exit_conditions(1, 100, 3000);
  chassis.drive_distance(-30,130);//////////////////////////////////////-24
  //////////////////////////////////
  wait(1000,msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Step 9");
  wait(1000,msec);
  //////////////////////////////////
  //MatchLoad.set(false);
  // CenterGoal.set(true);
  // Intake1.spin(forward, 100, pct);
  // Intake2.spin(reverse, 100, pct);//
  wait(1300,msec);//900
  chassis.drive_with_voltage(0, 0);
}