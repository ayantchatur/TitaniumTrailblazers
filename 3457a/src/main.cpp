#include "vex.h"
#define RED 1
#define BLUE 2
using namespace vex;

competition Competition;

int ALLIANCE_COLOR = RED;

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
TANK_TWO_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(Left1,Left2,Left3),

//Right Motors:
motor_group(Right1,Right2,Right3),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT14,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360/1.015,//360/1.0066,

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
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT16,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0.02,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT17,

//Sideways tracker diameter (reverse to make the direction switch):
2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
3.02

);

int current_auton_selection = 0;
bool auto_started = false;

void pre_auton() 
{
  vexcodeInit();
  default_constants();
  chassis.set_coordinates(0, 0, 0);
  ColorSortSensor.setLight(ledState::on);
  ColorSortSensor.setLightPower(100);
  Brain.Screen.setFont(vex::fontType::mono40);
  while(!auto_started)
  {

    if (Competition.isCompetitionSwitch()) 
    {
      if(ALLIANCE_COLOR == RED)
      {
        Brain.Screen.clearScreen(red);
        Brain.Screen.setFillColor(vex::color::red);
      }
      else
      {
        Brain.Screen.clearScreen(blue);
        Brain.Screen.setFillColor(vex::color::blue);
      }
      Brain.Screen.printAt(5, 40, "Battery: %d   %", Brain.Battery.capacity());
      switch(current_auton_selection)
      {
        case 0:
          Brain.Screen.printAt(5, 80, "SOLO AWP Signature");
          break;
        case 1:
          Brain.Screen.printAt(5, 80, "HALF SOLO AWP Sig");
          break;
        case 2:
          Brain.Screen.printAt(5, 80, "RIGHT SIDE 1 LOW");
          break;
        case 3:
          Brain.Screen.printAt(5, 80, "RIGHT SIDE RUSH");
          break;
        case 4:
          Brain.Screen.printAt(5, 80, "LEFT SIDE RUSH");
          break;
        case 5:
          Brain.Screen.printAt(5, 80, "LEFT SIDE MIDDLE GOAL");
          break;
        case 6:
          Brain.Screen.printAt(5, 80, "SKILLS");
          break;
        case 7:
          Brain.Screen.printAt(5, 80, "Auton 8");
          break;
      }
      if (Brain.Screen.pressing()) 
      {
        while(Brain.Screen.pressing())  
          wait(10,msec); 
        if (Brain.Screen.xPosition()> 480/2)  
            current_auton_selection++;
        else        
            current_auton_selection--;

        if (current_auton_selection > 7) current_auton_selection = 0;
        if (current_auton_selection < 0) current_auton_selection = 7;
      }
      wait(10,msec);
    }
  }
}
void autonomous(void) 
{
  chassis.drive_stop(hold);
  auto_started = true;
  switch(current_auton_selection)
  { 
    case 0:
      RightSideRush(); 
      //SoloAWPSignature();
      // Skills();
      break;
    case 1:         
       RightSide_HalfAWP();
      break;
    case 2:
       RightSide_1LowGoal();  
      break;
    case 3:
      RightSideRush(); 
      break;
    case 4:
      LeftSideRush();
      break;
    case 5:
      LeftSide_MiddleGoal();
      break;
    case 6:
      Skills();
      break;
    case 7:
      holonomic_odom_test();
      break;
 }
}

bool colorSort = true;

void usercontrol(void) 
{
  chassis.drive_stop(coast);
  Brain.Screen.setFillColor(vex::color::yellow);
  while (true) 
  {
      // Controller1.Screen.clearLine(1);
      // Controller1.Screen.setCursor(1, 1);
      // Controller1.Screen.print("colorSort: %s", colorSort ? "ON" : "OFF");

      // if (Controller1.ButtonB.pressing()) 
      // {
      // // Espera a que se suelte el botón para evitar múltiples activaciones rápidas
      // while (Controller1.ButtonB.pressing()) {
      //   wait(10, msec);
      // }
      // // Cambia el estado (toggle)
      // colorSort = !colorSort;

      // // Imprime el estado en la pantalla del control
  
      // }


    //Brain.Screen.clearScreen(yellow);
    double x = chassis.get_X_position();
    double y = chassis.get_Y_position();
    double heading = chassis.get_absolute_heading();
    // Imprimir en la consola de VEXcode
    printf("X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, heading);

    if(!IMU.isCalibrating())
    {
      Brain.Screen.setFillColor(vex::color::yellow);
      Brain.Screen.clearScreen(yellow);
      Brain.Screen.setPenColor(vex::color::black);
      Brain.Screen.setFont(vex::fontType::mono60);
      if(ALLIANCE_COLOR == RED)
      {
        Brain.Screen.setPenColor(vex::color::red);
        Brain.Screen.printAt(5,40, "RED ALLIANCE");
      }
      else
      {
        Brain.Screen.setPenColor(vex::color::blue);
        Brain.Screen.printAt(5,40, "BLUE ALLIANCE");       
      }
      Brain.Screen.setPenColor(vex::color::black);
      Brain.Screen.setFont(vex::fontType::mono40);
      Brain.Screen.printAt(5,80, "X:%.2f", chassis.get_X_position());
      Brain.Screen.printAt(320,80, "Y:%.2f", chassis.get_Y_position());
      Brain.Screen.printAt(5,120, "H:%.2f", chassis.get_absolute_heading());
    }
    // Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    // Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    chassis.control_tank();
    if (Controller1.ButtonR1.pressing())// && Controller1.ButtonL2.pressing()) 
    {
        IntakeMotor.spin(fwd, 0, pct);
        ScoreMotor.spin(fwd, 0, pct);
        if (Controller1.ButtonR1.pressing()) 
            Descore.set(false);
        else
            Descore.set(true);
    } 
    else if (Controller1.ButtonL1.pressing() && Controller1.ButtonR1.pressing()) 
    { 
      MatchLoad.set(true);
      IntakeMotor.spin(forward, 100, pct);
      // if(colorSort == true)
      // {
        // if(ColorSortSensor.hue() > 200 && ColorSortSensor.hue() < 250 && ALLIANCE_COLOR == RED && ColorSortSensor.isNearObject() )    
        //   ScoreMotor.spin(forward, 100, pct);
        // else if(ColorSortSensor.hue() > 0 && ColorSortSensor.hue() < 20 && ALLIANCE_COLOR == BLUE && ColorSortSensor.isNearObject())
        //   ScoreMotor.spin(forward, 100, pct);
        // else
        //   ScoreMotor.spin(reverse, 20, pct);
      // }
      // else
       ScoreMotor.spin(reverse, 20, pct);
 
    } 
    else if (Controller1.ButtonL1.pressing() && !Controller1.ButtonL2.pressing()) 
    {
        IntakeMotor.spin(fwd, 100, pct);
        ScoreMotor.spin(reverse, 20, pct);
    } 
    else if (Controller1.ButtonL2.pressing() && !Controller1.ButtonL1.pressing()) 
    {
      IntakeMotor.spin(reverse, 100,pct);
      ScoreMotor.spin(reverse, 100,pct);
    } 
    else if (Controller1.ButtonR1.pressing()) 
    {
      // if(colorSort == true)
      // {
        // if(ColorSortSensor.hue() > 200 && ColorSortSensor.hue() < 250 && ALLIANCE_COLOR == RED && ColorSortSensor.isNearObject() )// azul
        // {
        //   CenterGoal.set(true);
        //   //wait(50,msec);
        //   ScoreMotor.spin(reverse, 100,pct);
        // }
        // else if(ColorSortSensor.hue() > 0 && ColorSortSensor.hue() < 20 && ALLIANCE_COLOR == BLUE && ColorSortSensor.isNearObject() )
        // {
        //   CenterGoal.set(true);
        //   //wait(50,msec);
        //   ScoreMotor.spin(reverse, 100,pct);
        // }
        // else
        // {
        //   CenterGoal.set(false);
        //   ScoreMotor.spin(forward, 100,pct);    
        // }
      // }
      // else
      // {
        ScoreMotor.spin(forward, 100,pct); 
      // }
    
      IntakeMotor.spin(fwd, 100,pct);
    } 
    else if (Controller1.ButtonR2.pressing()) 
    {
      IntakeMotor.spin(forward, 100, pct);
      ScoreMotor.spin(reverse, 100, pct);
      CenterGoal.set(true);
    } 
    else 
    {
      IntakeMotor.spin(forward, 0, pct);
      ScoreMotor.spin(forward, 0,pct);
      CenterGoal.set(false);
      MatchLoad.set(false);
      Descore.set(true);
    }
    if (Controller1.ButtonLeft.pressing()) 
    {
        chassis.drive_with_voltage(12,12);
        wait(150,msec);
        chassis.drive_with_voltage(0,0);
        wait(300,msec);
        for (int i = 0; i <= 15; i++) 
        {
            Left1.spin(reverse, i * 9, pct);
            Left2.spin(reverse, i * 9, pct);
            Left3.spin(reverse, i * 9, pct);
            Right1.spin(reverse, i * 9, pct);
            Right2.spin(reverse, i * 9, pct);
            Right3.spin(reverse, i * 9, pct);
            wait(20,msec);
        }
        chassis.drive_with_voltage(0,0);
    }
    wait(20, msec); 

  }
}

int main() 
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) 
    wait(20, msec);
}
