#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;



// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
controller Controller1;

//The motor constructor takes motors as (port, ratio, reversed), so for example
motor Left1= motor(PORT11, ratio6_1, true);
motor Left2 = motor(PORT12, ratio6_1, true);
motor Left3 = motor(PORT13, ratio6_1, true);
motor Right1 = motor(PORT20, ratio6_1, false);
motor Right2 = motor(PORT19, ratio6_1, false);
motor Right3 = motor(PORT18, ratio6_1, false);
motor IntakeMotor = motor(PORT10, ratio6_1, false);
motor ScoreMotor = motor(PORT1, ratio6_1, true);

optical ColorSortSensor = optical(PORT9);
inertial IMU = inertial(PORT14);

digital_out CenterGoal = digital_out(Brain.ThreeWirePort.A);
digital_out MatchLoad = digital_out(Brain.ThreeWirePort.B);
digital_out Descore = digital_out(Brain.ThreeWirePort.C);

// line Arm_Limit = line(Brain.ThreeWirePort.H);


//Add your devices below, and don't forget to do the same in robot-config.h:


void vexcodeInit( void ) 
{
  IMU.calibrate();
  Brain.Screen.clearScreen();
  int barWidth = 400;          // ancho de la barra
  int barHeight = 100;         // altura de la barra
  int barX = (480 - barWidth)/2; // centrado horizontal
  int barY = (272 - barHeight)/2; // centrado vertical
  int totalTime = 2500;        // tiempo estimado de calibración en ms
  int steps = 100;             // cantidad de pasos de la barra

  for (int i = 0; i <= steps; i++) 
  {
      if (!IMU.isCalibrating()) break;
      Brain.Screen.setFillColor(vex::color::black);
      Brain.Screen.drawRectangle(barX, barY, barWidth, barHeight);
      int filledWidth = (i * barWidth) / steps;
      Brain.Screen.setFillColor(vex::color::green);
      Brain.Screen.drawRectangle(barX, barY, filledWidth, barHeight);
      vex::this_thread::sleep_for(totalTime / steps);
  }
  Brain.Screen.setFillColor(vex::color::yellow);
  Brain.Screen.clearScreen(yellow);
  Brain.Screen.setPenColor(vex::color::black);
  wait(500,msec);
  if (fabs(chassis.get_absolute_heading()) < 3 || fabs(chassis.get_absolute_heading() - 360) < 3) 
  {
  Controller1.rumble(".");
  Brain.Screen.printAt(10, 80, "IMU OK");
  } 
  else 
  {
    // Si está muy alejado de 0°, error → 3 vibraciones
    Controller1.rumble("...");
    Brain.Screen.printAt(10, 80, "IMU ERROR");
  }
  
  // nothing to initialize
}