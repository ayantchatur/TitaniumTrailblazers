#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

// Configuration functions
void default_constants();
void odom_constants();

// Basic movement tests
void drive_test();
void turn_test();
void swing_test();
void full_test();

// ODOM-based functions
void odom_test();
void odom_square_test();
void tank_odom_test();
void holonomic_odom_test();
void advanced_odom_demo();

// Competition autonomous
void custom_auton();

// PID Tuning
void tunepid();
void drive_pid_test();
void DrivePIDTest();
void TurnPIDTest();

// Odometry Tuning
void tuneodom();

void RightSide_HalfAWP();

void randomtest();
void LeftSide();

void leftauton();