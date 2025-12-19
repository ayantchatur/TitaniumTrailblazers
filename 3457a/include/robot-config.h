using namespace vex;

extern brain Brain;

extern controller Controller1;
//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;
// Declaración de motores
extern motor Left1;
extern motor Left2;
extern motor Left3;
extern motor Right1;
extern motor Right2;
extern motor Right3;
extern motor IntakeMotor;
extern motor ScoreMotor;
extern inertial IMU;
// Declaración de salidas digitales
extern digital_out CenterGoal;
extern digital_out MatchLoad;
extern digital_out Descore;
extern optical ColorSortSensor;

//Add your devices below, and don't forget to do the same in robot-config.cpp:


void  vexcodeInit( void );