#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
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
  chassis.heading_max_voltage = 3;//
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = 1.5;//1.1
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */
void matchloadTask() 
{
  wait(1000,msec);           // Espera 800 milisegundos
  MatchLoad.set(true);        // Activa el pistón
  wait(300,msec);  
  MatchLoad.set(false);
}

void matchloadTask2() 
{
  wait(700,msec);           // Espera 800 milisegundos
  MatchLoad.set(true);        // Activa el pistón
  wait(300,msec);  
  MatchLoad.set(false);
}
void matchloadTask3() 
{
  wait(200,msec);           // Espera 800 milisegundos
  MatchLoad.set(true);        // Activa el pistón
  wait(300,msec);  
  MatchLoad.set(false);
}


void SoloAWPSignature()
{
  chassis.set_coordinates(0, 0, 0);
  odom_constants();
  chassis.set_drive_exit_conditions(1.5, 100, 1050);//// 1101
  chassis.drive_distance(31,0);
  chassis.set_turn_exit_conditions(1, 100, 270);/////////300
  MatchLoad.set(true);
  chassis.turn_to_angle(90);
  chassis.set_turn_exit_conditions(1, 100, 300);/////////
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  wait(100,msec);//200
  chassis.set_drive_exit_conditions(1.5, 100, 600);
  chassis.drive_distance(10,90);
  chassis.drive_with_voltage(1, 1);//////////////////////
  wait(650,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 700);/////////1000
  chassis.drive_distance(-35,90);
  MatchLoad.set(false);
  chassis.drive_with_voltage(-2, -2);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1100,msec);//2000
  chassis.drive_with_voltage(12, 12);
  wait(150,msec); //200
  chassis.drive_with_voltage(12, -6);
  wait(200,msec);//300
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(1.5, 100, 800);//800
  chassis.turn_to_point(-25,6);///-25,6
  chassis.drive_to_point(-25,6);
  thread matchloadThread(matchloadTask);
  chassis.set_drive_exit_conditions(1.5, 100, 5000);
  chassis.drive_to_point(-25,-41);
  chassis.set_turn_exit_conditions(1, 100, 200);///250
  chassis.turn_to_angle(130);
  chassis.set_drive_exit_conditions(1.5, 100, 450);/////////////////////
  chassis.set_turn_exit_conditions(1, 100, 3000);
  chassis.drive_distance(-30,130);//////////////////////////////////////-24
   MatchLoad.set(false);
  CenterGoal.set(true);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 100, pct);//
  wait(900,msec);//2000
  ScoreMotor.spin(reverse, 30, pct);//
  CenterGoal.set(false);
  chassis.set_drive_exit_conditions(1.5, 100, 1300);///5000
  chassis.drive_distance(54,135);
  MatchLoad.set(true);
  chassis.set_turn_exit_conditions(1, 100, 300);// nuevo
  chassis.turn_to_angle(90);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(1.5, 100, 600);
  chassis.drive_distance(15,90);
  chassis.drive_with_voltage(5, 5);
  wait(650,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 800);/////////1000
  chassis.drive_distance(-40,90);//-35
  MatchLoad.set(false);
  chassis.DriveL.spin(reverse,0,pct);
  chassis.DriveR.spin(reverse,0,pct);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1000,msec);//2000
}
void RightSide_HalfAWP()
{
  chassis.set_coordinates(0, 0, 0);
  odom_constants();
  chassis.set_drive_exit_conditions(1.5, 100, 1050);//// 1101
  chassis.drive_distance(31,0);
  chassis.set_turn_exit_conditions(1, 100, 270);/////////300
  MatchLoad.set(true);
  chassis.turn_to_angle(90);
  chassis.set_turn_exit_conditions(1, 100, 300);/////////
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  wait(100,msec);//200
  chassis.set_drive_exit_conditions(1.5, 100, 600);
  chassis.drive_distance(10,90);
  chassis.drive_with_voltage(1, 1);//////////////////////
  wait(650,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 700);/////////1000
  chassis.drive_distance(-35,90);
  MatchLoad.set(false);
  chassis.drive_with_voltage(-2, -2);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1400,msec);//1100

  chassis.drive_with_voltage(12, 12);
  wait(150,msec); //200
  chassis.drive_with_voltage(12, -6);
  wait(200,msec);//300
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(1.5, 100, 800);//800
  chassis.turn_to_point(-25,6);///-25,6
  chassis.drive_to_point(-25,6);
  thread matchloadThread(matchloadTask);
  chassis.set_drive_exit_conditions(1.5, 100, 5000);
  chassis.drive_to_point(-25,-41);
  chassis.set_turn_exit_conditions(1, 100, 200);///250
  chassis.turn_to_angle(130);
  chassis.set_drive_exit_conditions(1.5, 100, 450);/////////////////////
  chassis.set_turn_exit_conditions(1, 100, 3000);
  chassis.drive_distance(-30,130);//////////////////////////////////////-24
  MatchLoad.set(false);
  CenterGoal.set(true);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 100, pct);//
  wait(1300,msec);//900
  chassis.drive_with_voltage(0, 0);
}
void RightSide_1LowGoal()
{
  odom_constants();
  chassis.drive_distance(25);
  MatchLoad.set(true);
  wait(600,msec);
  MatchLoad.set(false);
  chassis.drive_min_voltage = 4;
  chassis.drive_distance(3);
  chassis.drive_min_voltage = 0;
  chassis.turn_to_angle(90);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  thread matchloadThread(matchloadTask3);
  chassis.drive_distance(22);
  chassis.turn_to_point(42,0);
  chassis.drive_to_point(42,0);
  chassis.turn_to_angle(180);
  MatchLoad.set(true);
  chassis.set_drive_exit_conditions(1.5, 100, 600);
  chassis.drive_distance(15,180);
  chassis.drive_with_voltage(1, 1);
  wait(650,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 800);/////////1000
  chassis.drive_distance(-40,180);//-35
  MatchLoad.set(false);
  chassis.DriveL.spin(reverse,0,pct);
  chassis.DriveR.spin(reverse,0,pct);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1300,msec);//2000
}

void RightSideRush()
{
  odom_constants();
  chassis.set_coordinates(-4.12, -5.36, 295.16);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(1.5, 100, 1100);
  thread matchloadThread(matchloadTask2);
  chassis.drive_distance(31);
  chassis.drive_max_voltage = 8;
  chassis.turn_to_point(-42,25);
  chassis.drive_to_point(-42,25);
  chassis.drive_with_voltage(8,0);
  wait(300,msec);
  chassis.drive_with_voltage(0,0);
  wait(400,msec);
  chassis.drive_with_voltage(-8,0);
  wait(300,msec);
  chassis.drive_with_voltage(0,0);
  wait(300,msec);
  chassis.drive_distance(-20);
  chassis.turn_to_point(0,31);
  chassis.drive_to_point(0,31);
  chassis.turn_to_angle(95);
  chassis.set_drive_exit_conditions(1.5, 100, 600);///
  chassis.drive_distance(-30);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1000,msec);//2000
  ScoreMotor.spin(reverse, 20, pct);
  MatchLoad.set(true);
  chassis.set_drive_exit_conditions(1.5, 100, 1200);//// 600
  chassis.drive_distance(32,95);//40
  chassis.drive_with_voltage(1, 1);
  wait(650,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 900);/////////800
  chassis.drive_distance(-40,95);//-35
  MatchLoad.set(false);
  chassis.DriveL.spin(reverse,0,pct);
  chassis.DriveR.spin(reverse,0,pct);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1000,msec);//2000
}
void LeftSideRush()
{
  odom_constants();
  chassis.set_coordinates(-4.21, -30, 250.97);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(1.5, 100, 1100);
  thread matchloadThread(matchloadTask2);
  chassis.drive_distance(31);
  chassis.drive_max_voltage = 8;
  chassis.turn_to_point(-44,-59);
  chassis.drive_to_point(-44,-59);//-57
  chassis.drive_with_voltage(0,8);
  wait(250,msec);
  chassis.drive_with_voltage(0,0);
  wait(400,msec);
  chassis.drive_with_voltage(0,-8);
  wait(250,msec);
  chassis.drive_with_voltage(0,0);
  wait(300,msec);
  chassis.drive_distance(-20);
  chassis.turn_to_point(0,-68);
  chassis.drive_to_point(0,-68);
  chassis.turn_to_angle(92);
  chassis.set_drive_exit_conditions(1.5, 100, 600);///
  chassis.drive_distance(-33,92);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1000,msec);//2000
  ScoreMotor.spin(reverse, 20, pct);
  MatchLoad.set(true);
  chassis.set_drive_exit_conditions(1.5, 100, 1200);//// 600
  chassis.drive_distance(33,92);//40
  chassis.drive_with_voltage(1, 1);
  wait(650,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 900);/////////800
  chassis.drive_distance(-40,92);//-35
  MatchLoad.set(false);
  chassis.DriveL.spin(reverse,0,pct);
  chassis.DriveR.spin(reverse,0,pct);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1000,msec);//2000
}
void LeftSide_MiddleGoal()
{
  odom_constants();
  chassis.set_coordinates(-4.21, -30, 250.97);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(1.5, 100, 1100);
  thread matchloadThread(matchloadTask2);
  chassis.drive_distance(31);
  chassis.drive_max_voltage = 8;
  chassis.turn_to_point(-44,-59);
  chassis.drive_to_point(-44,-59);//-57
  chassis.drive_with_voltage(0,8);
  wait(250,msec);
  chassis.drive_with_voltage(0,0);
  wait(400,msec);
  chassis.drive_with_voltage(0,-8);
  wait(250,msec);
  chassis.drive_with_voltage(0,0);
  wait(300,msec);
  chassis.drive_distance(-22);
  chassis.turn_to_angle(135);
  chassis.drive_distance(-23);
  CenterGoal.set(true);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 100, pct);//
  wait(900,msec);//2000
  ScoreMotor.spin(reverse, 30, pct);//
  CenterGoal.set(false);
  chassis.set_drive_exit_conditions(1.5, 100, 1300);///5000
  chassis.turn_to_point(0,-67);
  chassis.drive_to_point(0,-67);
  chassis.turn_to_angle(92);
  chassis.set_drive_exit_conditions(1.5, 100, 600);///
  chassis.drive_distance(-33,92);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1000,msec);//2000
  ScoreMotor.spin(reverse, 20, pct);
  MatchLoad.set(true);
  chassis.set_drive_exit_conditions(1.5, 100, 1200);//// 600
  chassis.drive_distance(31,92);//40
  chassis.drive_with_voltage(1, 1);
  wait(650,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 900);/////////800
  chassis.drive_distance(-40,92);//-35
  MatchLoad.set(false);
  chassis.DriveL.spin(reverse,0,pct);
  chassis.DriveR.spin(reverse,0,pct);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(1000,msec);//2000
}

void Rutine(float dist1,float time1,float angle)
{
   chassis.set_turn_exit_conditions(1, 100, 270);/////////300
  MatchLoad.set(true);
  chassis.turn_to_angle(angle);
  chassis.set_turn_exit_conditions(1, 100, 300);/////////
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  wait(100,msec);//200
  chassis.set_drive_exit_conditions(1.5, 100, time1);
  chassis.drive_distance(dist1,angle);
  chassis.drive_with_voltage(1, 1);//////////////////////
  wait(1800,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 700);/////////1000
  chassis.drive_distance(-35,angle);
  MatchLoad.set(false);
  chassis.drive_with_voltage(-2, -2);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(2000,msec);//2000
  chassis.drive_distance(14);
}
void Rutine2(float dist1,float time1,float angle)
{
  chassis.set_turn_exit_conditions(1, 100, 270);/////////300
  MatchLoad.set(true);
  chassis.turn_to_angle(angle);
  chassis.set_turn_exit_conditions(1, 100, 300);/////////
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(reverse, 20, pct);
  wait(100,msec);//200
  chassis.set_drive_exit_conditions(1.5, 100, time1);
  chassis.drive_distance(dist1,angle);
  chassis.drive_with_voltage(1, 1);//////////////////////
  wait(1800,msec); //tal vez cambiar para no agarrar azul
  chassis.set_drive_exit_conditions(1.5, 100, 700);/////////1000
  chassis.drive_distance(-20,angle);//-35
  MatchLoad.set(false);

}
void Rutine3(float angle)
{
  chassis.set_drive_exit_conditions(1.5, 100, 700);/////////1000
  chassis.drive_distance(-15,angle);//-35
  MatchLoad.set(false);
  chassis.drive_with_voltage(-2, -2);
  IntakeMotor.spin(forward, 100, pct);
  ScoreMotor.spin(forward, 100, pct);
  wait(2000,msec);//2000
  chassis.drive_distance(14);
}

void Skills()
{
  chassis.set_coordinates(0, 0, 0);
  odom_constants();
  chassis.set_drive_exit_conditions(1.5, 100, 1050);//// 1101
  chassis.drive_distance(31,0);
  Rutine(10,600,90);
  chassis.set_drive_exit_conditions(1.5, 100, 5000);
  chassis.set_turn_exit_conditions(1, 100, 3000);/////////

  // chassis.turn_to_angle(180);
  chassis.turn_to_point(-5.96,-70);//-69
  // chassis.drive_distance(98);


  chassis.drive_to_point(-5.96,-70);//72

  Rutine(21,900,90);//10,600 TAL VEZ ANGULO 19
  chassis.set_drive_exit_conditions(1.5, 100, 5000);
  chassis.set_turn_exit_conditions(1, 100, 270);/////////
  chassis.turn_to_angle(40);
  chassis.drive_to_point(-18,-79);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-70,90);
  chassis.set_turn_exit_conditions(1, 100, 3000);/////////
  chassis.turn_to_angle(360);
  chassis.drive_distance(14); //- 8
  Rutine2(23,1000,270);
  chassis.set_drive_exit_conditions(1.5, 100, 5000);
  chassis.set_turn_exit_conditions(1, 100, 3000);
  chassis.turn_to_angle(360);
  chassis.drive_distance(96);
  chassis.turn_to_angle(270);
  Rutine3(270);
  Rutine2(23,1000,270);
  chassis.set_drive_exit_conditions(1.5, 100, 5000);
  chassis.set_turn_exit_conditions(1, 100, 3000);
  chassis.turn_to_angle(183);
  chassis.drive_distance(98);//97
  chassis.turn_to_angle(270);
  Rutine3(270);
  chassis.turn_to_angle(225);
  chassis.drive_distance(-23);//-18
  chassis.turn_to_angle(270);
  chassis.drive_max_voltage = 12;
  chassis.set_drive_exit_conditions(1.5, 100, 2000);
  chassis.drive_distance(-102);
  chassis.drive_with_voltage(8,8);
  wait(150,msec);
  chassis.drive_with_voltage(0,0);
  while( chassis.get_absolute_heading() > 0.0)
  {
    chassis.drive_with_voltage(0,-10);
    wait(20,msec);
  } 
  chassis.drive_with_voltage(0,0);







}
void drive_test(){

  //  chassis.drive_distance(48);
  //  chassis.drive_distance(-48);
  // chassis.turn_to_angle(180);

  // chassis.drive_distance(6);
  // chassis.drive_distance(12);
  // chassis.drive_distance(18);
  // chassis.drive_distance(-36,0.0,8.0,3);
  // chassis.turn_to_angle(5);
  // chassis.turn_to_angle(30);
  // chassis.turn_to_angle(90);
  // chassis.turn_to_angle(225);
  // chassis.turn_to_angle(0);


  odom_constants();

  // chassis.drive_to_point(0,24);
  // chassis.turn_to_point(24,24);
  // chassis.drive_to_point(24,24);;
  // chassis.drive_to_point(0,0);
  // chassis.turn_to_angle(0);

    chassis.drive_to_pose(0, 24, 0);
    chassis.drive_to_pose(0, 0, 0);
  // chassis.drive_to_point(0,24);
  //  chassis.turn_to_angle(180);
  //     chassis.turn_to_angle(180);
  // chassis.drive_to_point(0,0);
  // chassis.turn_to_angle(0);



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
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
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
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    //task::sleep(20);
  }
  //  chassis.drive_to_point(24,24);
  // chassis.drive_to_pose(0,24,0);
}

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