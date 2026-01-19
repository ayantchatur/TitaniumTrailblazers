using namespace vex;

extern brain Brain;
//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:

extern controller Controller1;

// Sensors
extern inertial Inertial;
extern rotation ForwardTracker;

// LeftDrive
extern motor LeftDriveMotorA;
extern motor LeftDriveMotorB;
extern motor LeftDriveMotorC;
extern motor_group LeftDrive;

//RightDrive
extern motor RightDriveMotorA;
extern motor RightDriveMotorB;
extern motor RightDriveMotorC;
extern motor_group RightDrive;

// Other motors
extern motor IntakeMotor;
extern motor RampMotor;
extern motor ScoreMotor;

// Pneumatics
extern digital_out Descore;
extern digital_out IntakeTilt;
extern digital_out Matchload;

void  vexcodeInit( void );