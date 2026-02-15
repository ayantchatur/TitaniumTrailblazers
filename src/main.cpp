#include "vex.h"
#include "autons.h"
#include "robot-config.h"

using namespace vex;
competition Competition;

brain Brain;
controller Controller1 = controller(primary);

// Sensors
inertial Inertial = inertial(PORT7);
//rotation ForwardTracker = rotation(PORT12, false);

// LeftDrive
motor LeftDriveMotorA = motor(PORT20, ratio6_1, false);
motor LeftDriveMotorB = motor(PORT19, ratio6_1, true);
motor LeftDriveMotorC = motor(PORT21, ratio6_1, true);
motor_group LeftDrive = motor_group(LeftDriveMotorA, LeftDriveMotorB, LeftDriveMotorC);

//RightDrive
motor RightDriveMotorA = motor(PORT11, ratio6_1, false);
motor RightDriveMotorB = motor(PORT10, ratio6_1, true);
motor RightDriveMotorC = motor(PORT9, ratio6_1, false);
motor_group RightDrive = motor_group(RightDriveMotorA, RightDriveMotorB, RightDriveMotorC);

// Other motors
motor IntakeMotor = motor(PORT15, ratio6_1, false);
motor RampMotor = motor(PORT3, ratio6_1, false);
motor ScoreMotor = motor(PORT17, ratio18_1, true);

// Pneumatics
 digital_out Descore(Brain.ThreeWirePort.C);
//  digital_out IntakeTilt(Brain.ThreeWirePort.A);
digital_out MatchLoader(Brain.ThreeWirePort.H);

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_ODOM,
//TANK_TWO_ROTATION, 
//ZERO_TRACKER_NO_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LeftDrive),

//Right Motors:
motor_group(RightDrive),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT7,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
353.04,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
0,     0,

//LB:      //RB: 
0,     0,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT12,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0, //-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
0,

//Sideways tracker diameter (reverse to make the direction switch):
0,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
0

);

int current_auton_selection = 0;
bool auto_started = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0 - ODOM");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading:");
    Brain.Screen.printAt(5, 100, "%.1f degrees", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Position: X=%.1f Y=%.1f", chassis.get_X_position(), chassis.get_Y_position());
    Brain.Screen.printAt(5, 140, "Selected Auton:");
    // Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    // Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    // Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 160, "Basic Drive Test");
        break;
      case 1:
        Brain.Screen.printAt(5, 160, "Turn Test");
        break;
      case 2:
        Brain.Screen.printAt(5, 160, "Swing Test");
        break;
      case 3:
        Brain.Screen.printAt(5, 160, "Full Movement Test");
        break;
      case 4:
        Brain.Screen.printAt(5, 160, "Position Test (Square)");
        break;
      case 5:
        Brain.Screen.printAt(5, 160, "Advanced Odom Test");
        break;
      case 6:
        Brain.Screen.printAt(5, 160, "Live Position Display");
        break;
      case 7:
        Brain.Screen.printAt(5, 160, "Custom Auton");
        break;
      // case 0:
      //   Brain.Screen.printAt(5, 140, "Auton 1");
      //   break;
      // case 1:
      //   Brain.Screen.printAt(5, 140, "Auton 2");
      //   break;
      // case 2:
      //   Brain.Screen.printAt(5, 140, "Auton 3");
      //   break;
      // case 3:
      //   Brain.Screen.printAt(5, 140, "Auton 4");
      //   break;
      // case 4:
      //   Brain.Screen.printAt(5, 140, "Auton 5");
      //   break;
      // case 5:
      //   Brain.Screen.printAt(5, 140, "Auton 6");
      //   break;
      // case 6:
      //   Brain.Screen.printAt(5, 140, "Auton 7");
      //   break;
      // case 7:
      //   Brain.Screen.printAt(5, 140, "Auton 8");
      //   break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    wait(50, msec);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;

  //RightSide_HalfAWP();
  // Run this for tuning PID
  //tunepid();
  // drive_pid_test();
  //tuneodom();
  // odom_test();
  // randomtest();
  // full_test();
  // LeftSide();
  leftauton();

  // switch(current_auton_selection){ 
  //   case 0:
  //     drive_test();
  //     break;
  //   case 1:         
  //     turn_test();
  //     break;
  //   case 2:
  //     swing_test();
  //     break;
  //   case 3:
  //     full_test();
  //     break;
  //   case 4:
  //     odom_square_test();
  //     break;
  //   case 5:
  //     tank_odom_test();
  //     break;
  //   case 6:
  //     odom_test();
  //     break;
  //   case 7:
  //     custom_auton();
  //     break;
    // case 0:
    //   drive_test();
    //   break;
    // case 1:         
    //   drive_test();
    //   break;
    // case 2:
    //   turn_test();
    //   break;
    // case 3:
    //   swing_test();
    //   break;
    // case 4:
    //   full_test();
    //   break;
    // case 5:
    //   odom_test();
    //   break;
    // case 6:
    //   tank_odom_test();
    //   break;
    // case 7:
    //   holonomic_odom_test();
    //   break;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

bool ScoreMiddleState = false;
bool ScoreMiddleLastPress = false;
bool matchLoaderState = false;
bool matchLoaderLastPress = false;
bool descoreState = false;
bool descoreLastPress = false;

void usercontrol(void) {
  // User control code here, inside the loop
  chassis.set_coordinates(0, 0, 0);
  while (1) {
    
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    IntakeMotor.setVelocity(100,pct);
    RampMotor.setVelocity(100,pct);
    ScoreMotor.setVelocity(100,pct);
    chassis.drive_max_voltage = 12;
    
    while(true)
    {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("Current Position: %f",chassis.get_absolute_heading());
      // if(abs(Controller1.Axis3.position()) > 30)
      // {
      //   LeftDrive.spin(vex::reverse,Controller1.Axis3.position(),percent);
      // }
      // else
      // {
      //   LeftDrive.stop();
      // }
  
      // if(abs(Controller1.Axis2.position()) > 30)
      // {
      //   RightDrive.spin(vex::reverse,Controller1.Axis2.position(),percent);
      // }
      // else
      // {
      //   RightDrive.stop();
      // }

      // Intake controls
      if(Controller1.ButtonR1.pressing()) {
        IntakeMotor.spin(forward);
        //RampMotor.spin(reverse);
        //ScoreMotor.spin(reverse);
      } else if(Controller1.ButtonR2.pressing()) {
        IntakeMotor.spin(reverse);
        //RampMotor.spin(forward);
        //ScoreMotor.spin(forward);
      } else {
        IntakeMotor.stop();
        //RampMotor.stop();
        //ScoreMotor.stop();
      }

      if(Controller1.ButtonA.pressing()){
        waitUntil(!Controller1.ButtonA.pressing());
        DrivePIDTest();
      }

      if(Controller1.ButtonUp.pressing()){
        waitUntil(!Controller1.ButtonUp.pressing());
        TurnPIDTest();
      }

      if(Controller1.ButtonL1.pressing()) {
        ScoreMotor.spin(reverse);
      } else if(Controller1.ButtonL2.pressing()) {
        ScoreMotor.spin(forward);
      } else {
        ScoreMotor.stop();
      }

      // Pneumatic Control: Intake Tilt (match loading)
      bool matchLoaderCurrentPress = Controller1.ButtonB.pressing();
      if (matchLoaderCurrentPress && !matchLoaderLastPress) {
        matchLoaderState = !matchLoaderState;
        MatchLoader.set(matchLoaderState);
      }
      matchLoaderLastPress = matchLoaderCurrentPress;

      // // Pneumatic Toggle: Middle Goal Scoring
      // bool ScoreMiddleCurrentPress = Controller1.ButtonB.pressing();
      // if (ScoreMiddleCurrentPress && !ScoreMiddleLastPress) {
      //   ScoreMiddleState = !ScoreMiddleState;
      //   ScoreMiddle.set(ScoreMiddleState);
      // }
      // ScoreMiddleLastPress = ScoreMiddleCurrentPress;

      // Pneumatic Toggle: Descore
      bool descoreCurrentPress = Controller1.ButtonX.pressing();
      if (descoreCurrentPress && !descoreLastPress) {
        descoreState = !descoreState;
        Descore.set(descoreState);
      }
      descoreLastPress = descoreCurrentPress;

      // if (Controller1.ButtonY.pressing()) {
      //   piddrivetest();
      //   waitUntil(!Controller1.ButtonY.pressing());
      // }
      // // Pneumatic Control: Descore
      // bool DescoreCurrentPress = Controller1.ButtonX.pressing();
      // if (DescoreCurrentPress && !DescoreLastPress) {
      //   DescoreState = !DescoreState;
      //   Descore.set(DescoreState);
      // }
      // DescoreLastPress = DescoreCurrentPress;

      // // Pneumatic Toggle: Matchload
      // bool MatchloadCurrentPress = Controller1.ButtonB.pressing();
      // if (MatchloadCurrentPress && !MatchloadLastPress) {
      //   MatchloadState = !MatchloadState;
      //   Matchload.set(MatchloadState);
      // }
      // MatchloadLastPress = MatchloadCurrentPress;

// ScoreMiddle.set(true);
      //Replace this line with 
      //chassis.control_tank(); //for tank drive 
      //or 
      //chassis.control_holonomic(); //for holo drive.
      chassis.control_arcade();
        
    
    
      // Display position on controller screen (press Up arrow)
      if(Controller1.ButtonUp.pressing()) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("X:%.1f Y:%.1f H:%.1f",
        chassis.get_X_position(), 
        chassis.get_Y_position(), 
        chassis.get_absolute_heading());
        Controller1.Screen.newLine();
        Controller1.Screen.print("We are Charlie Kirk");
      }

      wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
    }
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}