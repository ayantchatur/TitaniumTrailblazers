#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
// brain  Brain;
// digital_out Descore(Brain.ThreeWirePort.C);
// //digital_out IntakeTilt(Brain.ThreeWirePort.A);
// digital_out MatchLoader(Brain.ThreeWirePort.H);

// controller Controller1 = controller(primary);

// // Sensors
// inertial Inertial = inertial(PORT10);
// rotation ForwardTracker = rotation(PORT21, false);

// // LeftDrive
// motor LeftDriveMotorA = motor(PORT6, ratio6_1, true);
// motor LeftDriveMotorB = motor(PORT3, ratio6_1, true);
// motor LeftDriveMotorC = motor(PORT5, ratio6_1, true);
// motor_group LeftDrive = motor_group(LeftDriveMotorA, LeftDriveMotorB, LeftDriveMotorC);

// //RightDrive
// motor RightDriveMotorA = motor(PORT18, ratio6_1, true);
// motor RightDriveMotorB = motor(PORT19, ratio6_1, true);
// motor RightDriveMotorC = motor(PORT16, ratio6_1, true);
// motor_group RightDrive = motor_group(RightDriveMotorA, RightDriveMotorB, RightDriveMotorC);

// // Other motors
// motor IntakeMotor = motor(PORT2, ratio6_1, false);
// motor ScoreMotor = motor(PORT19, ratio18_1, true);

// // Pneumatics
// digital_out ScoreMiddle = digital_out(Brain.ThreeWirePort.A);
// digital_out IntakeTilt = digital_out(Brain.ThreeWirePort.H);



void vexcodeInit( void ) {
  // nothing to initialize
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  chassis.Gyro.calibrate();
  Inertial.calibrate();
  //Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (chassis.Gyro.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(5, msec);
  Brain.Screen.clearScreen();
  LeftDrive.setVelocity(30, percent);
  RightDrive.setVelocity(30,percent);
  IntakeMotor.setVelocity(100,percent);
}