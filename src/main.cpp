/*
 * Created by: Team 2381C (Colonel By Secondary School)
 * For:        VEX VRC Competition (Tower Takeover) 2019-2020
 * On:         January 25, 2020 (for Terrebonne Competition)
 * 
 * This project was created as code for the VEX VRC Tower Takeover competition in
 * the 2019-2020 competition season. It consists of the main and pid files. The main
 * focus of the programming aspect on the team is to program:
 *    - driver control (including macros); and
 *    - create an autonomous sequence driver pathway.
 * 
 * The challenges/problems solved through this project was to achieve both criteria
 * successfully, which means to ensure reliabilitym, simplicity and of course, make sure
 * that everything is working for the competition.
 */

// Including required .
#include "main.h"
#include "pros/api_legacy.h"
#include <array>
#include "pid.hpp"

// Defining ports for the Quadratic Encoder (for the tracking of wheel movements).
#define QUAD_TOP_PORT 1
#define QUAD_BOTTOM_PORT 2

// Declaring and initizalizing required motors and analog sensors.
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor leftBack(12);
pros::Motor leftFront(11);
pros::Motor rightBack(19);
pros::Motor rightFront(20);
pros::Motor leftIntake(18);
pros::Motor rightIntake(13);
pros::Motor lift(14);
pros::Motor center(16);
pros::ADIEncoder encoder(QUAD_TOP_PORT, QUAD_BOTTOM_PORT, false);
pros::ADIAnalogIn gyro('H');

// Global and constant delay.
int delay = 20;

// Defining PID Params for the angler and the drivebase.
std::array<double, 3> anglerPIDParams = {0.06, 0, 0};
std::array<double, 3> drivebasePIDParams = {0.3, 0, 0};

// Initializing the PIDs using the above paramenters, by calling the PID class.
PID *anglerPIDController = new PID(
    &anglerPIDParams[0],
    &anglerPIDParams[1],
    &anglerPIDParams[2]);

PID *drivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

// ENUMERATED variables used for when these variables are passed as parameters in functions.

// Possible DIRECTIONS
enum DIRECTION
{
  FORWARD,
  REVERSE,
  LEFT,
  RIGHT,
  LEFT_FORWARD,
  LEFT_BACK,
  RIGHT_FORWARD,
  RIGHT_BACK
};

// Possible starting POSITION for the AUTONOMOUS sequence.
enum AUTON_POS
{
  BIG_RED,
  BIG_BLUE,
  SMALL_RED,
  SMALL_BLUE
};

// Possible TURNS.
enum TURN
{
  N,
  E,
  S,
  W,
  NE,
  SE,
  SW,
  NW
};

// Define the starting AUTONOMOUS POSITION at the top, which can be changed match after match.
AUTON_POS startingPosition = BIG_RED;

/**
 * Calls the angler PID to stack the cubes. Essentially, this function creates a while loop
 * under which the program is to stack the cubes. For each iteration of the loop, a new value
 * is received from the angler PID controller, which is then passed into each motor, and the motors
 * are then moved accordingly.
 */
void autonStack()
{
  while (true)
  {
    
    
    double movFactor = anglerPIDController->update(1400, rightBack.get_position());

    pros::lcd::set_text(1, "Motor position: " + std::to_string(rightBack.get_position()));
    pros::lcd::set_text(2, "Motor speed: " + std::to_string(movFactor));

    // Using the transmission for the stacking of cubes.
    leftFront.move(-movFactor);
    leftBack.move(-movFactor);
    rightFront.move(movFactor);
    rightBack.move(movFactor);

    // Checking for the sentinel value (using the motor's encoder values) to determine if the operation
    // has been completed.
    if (rightBack.get_position() > 1200) 
    {
      leftFront.move(0);
      leftBack.move(0);
      rightFront.move(0);
      rightBack.move(0);
      // rightBack.tare_position();
      break;
    }

    // Delay as to not overload the motor.
    pros::delay(delay);
  }
  
  leftFront.move(20);
  leftBack.move(20);
  rightFront.move(-20);
  rightBack.move(-20);

  pros::delay(500);

  leftFront.move(0);
  leftBack.move(0);
  rightFront.move(0);
  rightBack.move(0);


  pros::delay(2000);

  leftFront.move(10);
  leftBack.move(10);
  rightFront.move(-10);
  rightBack.move(-10);

  pros::delay(800);

  leftFront.move(0);
  leftBack.move(0);
  rightFront.move(0);
  rightBack.move(0);

  leftFront.move(50);
  leftBack.move(-50);
  rightFront.move(-50);
  rightBack.move(50);

  leftIntake.move(-80);
  rightIntake.move(80);

  pros::delay(1000);

  leftFront.move(0);
  leftBack.move(0);
  rightFront.move(0);
  rightBack.move(0);

  leftIntake.move(0);
  rightIntake.move(0);
}

/**
 * For the testing of a manual (non-PID) based autonomous drivepath. This function
 * can be called to move the robot in any valid direction, with a predefined delay
 * and a given movemenet factor.
 * 
 * @param direction gets the direction to determine how the motors in the H-drive
 *                  configuration need to be moved.
 * @param delay     determines the delay after the motor movement, for the motor's
 *                  motion to fully complete, instead of being called instantaneously.
 * @param movFactor determines how much the motors are to be moved.
 */
void moveRobotManual(DIRECTION direction, int delay, int movFactor)
{
  // Motor coefficients for the H-Drive.
  int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;

  // Check the direction, and change the motor coefficient accordingly.
  switch (direction)
  {
  case FORWARD:
    cofRB = -1;
    cofLF = -1;
    break;

  case REVERSE:
    cofRF = -1;
    cofLB = -1;
    break;

  case LEFT:
    cofRB = -1;
    cofLB = -1;
    break;

  case RIGHT:
    cofRF = -1;
    cofLF = -1;
    break;
  }

  // Move the motors with the movement factor and multiplied by the coefficient which
  // determines whether the movement factor needs to be negative.
  leftBack.move(movFactor * cofLB);
  leftFront.move(movFactor * cofLF);
  rightBack.move(movFactor * cofRB);
  rightFront.move(movFactor * cofRF);

  // Delay to ensure that the movement is completed.
  pros::delay(delay);
}

/**
 * For the PID based autonomous drivepath. This function can be called to move the robot
 * in any valid direction, using a PID controller for the drive base. Essentially, the PID
 * params and PID controller defined at the top is called here to get values for the
 * drivebase's movement, and to ensure reliability of the movements. The concept for
 * this function and the loop is the same as the one written for the autonomous stacking
 * (which makes use of the angler PID).
 * 
 * @param direction     gets the direction to determine how the motors in the H-drive
 *                      configuration need to be moved.
 * @param envoderValue  used to determine how much the robot needs to be moved (by getting
 *                      the motor's position and comparing it to where it needs to be).
 */
void moveRobot(double encoderValue, DIRECTION direction)
{
  // Motor coefficients for the H-Drive.
  int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;

  // Check the direction, and change the motor coefficient accordingly.
  switch (direction)
  {
  case FORWARD:
    cofLB = -1;
    cofRF = -1;
    break;

  case REVERSE:
    cofLB = -1;
    cofRF = -1;
    break;

  case LEFT:
    cofLB = -1;
    cofRF = -1;
    break;

  case RIGHT:
    cofLB = -1;
    cofRF = -1;
    break;
  }

  while (true)
  {
    double movFactor = drivebasePIDController->update(encoderValue, rightBack.get_position());

    leftBack.move(movFactor * cofLB);
    leftFront.move(movFactor * cofLF);
    rightBack.move(movFactor * cofRB);
    rightFront.move(movFactor * cofRF);

    if (abs(rightBack.get_position() + 730) < 20)
    {
      rightBack.tare_position();
      break;
    }

    pros::delay(delay);
  }
}

/**
 * Called at the initialization of the competition. Turns on the brain's LCD.
 */
void initialize()
{
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Go 2381C!");
}

/**
 * Pre-built disabled function required for the competition.
 */
void disabled()
{
}

/**
 * Required competition-specific initialize code.
 */
void competition_initalize()
{
}

// Global variable that determines the mode for the goofy arm macros (the extent to which the goofy
// arms needs to be raised).
int mode = 0;

/**
 * This function is a macro for the raising of the foody arms. Essentially, it takes in the mode and
 * then raises the goofy arm to the required level as specified by the driver. The competitive
 * advantage of this function is that it ensures reliability during a competition match.
 */

// Global variable that determines whether the R2 button should act as a 'SHIFT' key or
// as the controller for the Goofy Arm macros.
int tower_counter = 0;
int origin = 0;
/**
 * Primary function containing the necessary code for the driver to drive and maneuver the
 * robot. It also contains the necessary event listeners to trigger the driver's movements
 * and enable macros as requested.
 */
void opcontrol()
{
  bool holdMove;
  bool macroEnabled = false;

  int hit = 0;

  while (true)
  {
    pros::delay(20);
    pros::lcd::set_text(4, "Motor pos: " + std::to_string(rightBack.get_position()));
    pros::lcd::set_text(5, "Goofy Position " + std::to_string(lift.get_position()));

    // Split acrade controls that control the drive base.
    leftFront.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    leftBack.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rightBack.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rightFront.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    center.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    // Keeping the motors at move_velocity(0) keeps the motor position locked.
    leftIntake.move_velocity(0);
    rightIntake.move_velocity(-0);
    leftIntake.move_velocity(-0);
    rightIntake.move_velocity(0);

    // brake_bold keeps the motors in brake mode so the motors will actively resist movement, are locked in position.
    leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    center.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   

    // Shift buttons R1 (for tray tilt) and R2 (for goofy arm).
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
      {
        leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      }
      leftFront.move(-0.5 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      leftBack.move(-0.5 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      rightBack.move(0.5 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      rightFront.move(0.5 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
      lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      lift.move(-master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    }

    // Intake controls using the L2 and L1 buttons.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
      leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      leftIntake.move_velocity(200);
      rightIntake.move_velocity(-200);
      pros::delay(20);
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
      leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      leftIntake.move_velocity(-80);
      rightIntake.move_velocity(80);
      pros::delay(20);
    }

    /*
		 * In the initalization (before the competition starts), the grippy tray will always be set up
		 * in a neutral position and the code will attempt to return the grippy arm to this neutral position
		 * unless a different position is required (either due to driver control or in the auton sequence).
		 */

    // Driver control and macros for the grippy arm.

    pros::delay(20);

    // Tower Macros.
    int control;
    //digital down buttons control the different tower heights 
    // if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    //   if(lift.get_position() != -330) {
    //     if(lift.get_position() < -360) {
    //       lift.move(30);
    //     }
        
    //     if(lift.get_position() > -300) {
    //       lift.move(-30);
    //     }
        
    //     if(lift.get_position() > -360 && lift.get_position() < -300) {
    //       lift.move(0);
    //     }
        
    //   }
      
    // }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      autonStack;
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      control = 0;
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      control = 1;
      rightBack.tare_position();
      origin = rightBack.get_position();
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      control = 2;
      rightBack.tare_position();
      origin = rightBack.get_position();
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        control = 3;
        rightBack.tare_position();
        origin = rightBack.get_position();
      }    

    pros::lcd::set_text(6, "origin" + std::to_string(origin));
     pros::lcd::set_text(8, "Motor Temperature:" + std::to_string(lift.get_temperature()));
    // Even numbers will toggle a separate macro using the A button.
    pros::lcd::set_text(7, "Control: " + std::to_string(control));

    if(control != 1 && control != 2) {
       lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
      if(control == 1) {

          if(lift.get_position() > -1900) {
           
            lift.move(-120);
          }
          else {
             lift.move(-5);
             control = 4;
          }
         
          if(rightBack.get_position() < origin + 415) {
            leftFront.move(-70);
            leftBack.move(-70);
            rightFront.move(70);
            rightBack.move(70);
          }
          

        }
        else if(control == 2) {
          if(lift.get_position() > -2400) {
            lift.move(-120);
          }
          else {
            lift.move(-5);
            control = 4;
          }

          
          if(rightBack.get_position() < 415) {
            leftFront.move(-70);
            leftBack.move(-70);
            rightFront.move(70);
            rightBack.move(70);
          }
          else {
            if(rightBack.get_position() < 415) {
              leftFront.move(0);
              leftBack.move(0);
              rightFront.move(0);
              rightBack.move(0);
            }
          }
        }
        else if(control == 3) {
          if(lift.get_position() < 30) {
            lift.move(40); 
          }
          else {
            lift.move(0);
            control = 3;
          }

           if(rightBack.get_position() > origin - 150) {
            leftFront.move(10);
            leftBack.move(10);
            rightFront.move(-10);
            rightBack.move(-10);
          }
          else {
            leftFront.move(0);
            leftBack.move(0);
            rightFront.move(0);
            rightBack.move(0);
            control = 4;
          }
        }
    
    
  }

  pros::delay(50);

  }

// Deploy function which will be called in autonomous to extend the tray of the robot.
void deploy()
{
  lift.move(-70);
  pros::delay(1600);
  lift.move(0);

  rightIntake.move(100);
  leftIntake.move(-100);
  pros::delay(500);
  rightIntake.move(0);
  leftIntake.move(0);

  leftFront.move(-30);
  leftBack.move(-30);
  rightBack.move(30);
  rightFront.move(30);
  pros::delay(500);
  leftFront.move(0);
  leftBack.move(0);
  rightBack.move(0);
  rightFront.move(0);
}

// Stops the drivebase by setting all motors to 0 velocity.
void stopDrivebase()
{
  moveRobotManual(FORWARD, 100, 0);
}

/**
 * Defines the drivepath for the big red goal zone. The motion are currently manually
 * profiled using the 'moveRobotManual' function.
 */

void cancerBigRed() {
  leftBack.move(60);
  leftFront.move(-60);
  rightBack.move(-60);
  rightFront.move(60);

  rightIntake.move(-186);
  leftIntake.move(186);

  pros::delay(1200);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  rightIntake.move(0);
  leftIntake.move(0);

  leftBack.move_velocity(-60);
  leftFront.move_velocity(60);
  rightBack.move_velocity(60);
  rightFront.move_velocity(-60);

  pros::delay(800);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  pros::delay(20);

  leftBack.move(-70);
  leftFront.move(70);
  rightBack.move(-70);
  rightFront.move(70);

  pros::delay(650);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);
  pros::delay(20);
  leftBack.move(60);
  leftFront.move(-60);
  rightBack.move(-60);
  rightFront.move(60);

  pros::delay(1300);

	leftBack.move(0);
	leftFront.move(0);
	rightBack.move(0);
	rightFront.move(0);

  rightIntake.move(186);
  leftIntake.move(-186);

  pros::delay(1500);



  rightIntake.move(0);
  leftIntake.move(0);

  leftBack.move(-60);
  leftFront.move(60);
  rightBack.move(60);
  rightFront.move(-60);

  pros::delay(2000);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

}

void attemptStack() {
   leftBack.move(60);
  leftFront.move(-60);
  rightBack.move(-60);
  rightFront.move(60);

  rightIntake.move(-186);
  leftIntake.move(186);

  pros::delay(1200);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  rightIntake.move(0);
  leftIntake.move(0);

  leftBack.move_velocity(-60);
  leftFront.move_velocity(60);
  rightBack.move_velocity(60);
  rightFront.move_velocity(-60);

  pros::delay(800);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  pros::delay(20);

  leftBack.move(-70);
  leftFront.move(70);
  rightBack.move(-70);
  rightFront.move(70);

  pros::delay(650);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);
  pros::delay(20);
  
  leftBack.move(60);
  leftFront.move(-60);
  rightBack.move(-60);
  rightFront.move(60);

  pros::delay(1300);

	leftBack.move(0);
	leftFront.move(0);
	rightBack.move(0);
	rightFront.move(0);

  leftFront.move(-30);
  leftBack.move(-30);
  rightBack.move(30);
  rightFront.move(30);

  pros::delay(1000);

  leftBack.move(0);
	leftFront.move(0);
	rightBack.move(0);
	rightFront.move(0);


  leftBack.move(-60);
  leftFront.move(60);
  rightBack.move(60);
  rightFront.move(-60);

  pros::delay(2000);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);
  
 
}

void bigRed()
{
  // Intake the first cube.
  rightIntake.move(-186 / 2);
  leftIntake.move(186 / 2);
  moveRobotManual(FORWARD, 1200, 60);

  stopDrivebase();
  rightIntake.move(0);
  leftIntake.move(0);

  // Move the robot backwards.
  moveRobotManual(REVERSE, 800, 60);
  stopDrivebase();

  // Turn the robot towards the goal zone (LEFT).
  moveRobotManual(LEFT, 650, 70);
  stopDrivebase();

  // Move the robot all the way towards the goal zone.
  moveRobotManual(FORWARD, 1300, 60);

  // Outtake the cube.
  rightIntake.move(186 / 2);
  leftIntake.move(-186 / 2);
  moveRobotManual(FORWARD, 1500, 0);

  rightIntake.move(0);
  leftIntake.move(0);

  // Drive away from the goal zone.
  moveRobotManual(REVERSE, 2000, 60);
  stopDrivebase();
}

/**
 * Defines the drivepath for the big blue goal zone. The motion are currently manually
 * profiled using the 'moveRobotManual' function.
 */
void bigBlue()
{
  // Intake the first cube.
  rightIntake.move(-186 / 2);
  leftIntake.move(186 / 2);
  moveRobotManual(FORWARD, 1200, 60);

  stopDrivebase();
  rightIntake.move(0);
  leftIntake.move(0);

  // Move the robot backwards.
  moveRobotManual(REVERSE, 800, 60);
  stopDrivebase();

  // Turn the robot towards the goal zone (RIGHT).
  moveRobotManual(RIGHT, 650, 70);
  stopDrivebase();

  // Move the robot all the way towards the goal zone.
  moveRobotManual(FORWARD, 1300, 60);

  // Outtake the cube.
  rightIntake.move(186 / 2);
  leftIntake.move(-186 / 2);
  moveRobotManual(FORWARD, 1500, 0);

  rightIntake.move(0);
  leftIntake.move(0);

  // Drive away from the goal zone.
  moveRobotManual(REVERSE, 2000, 60);
  stopDrivebase();
}

/**
 * Initiates the autonomous sequence during a competition match.
 */
void autonomous()
{

  autonStack();

  // pros::delay(2000);

  // switch (startingPosition)
  // {
  // case BIG_RED:
  //   cancerBigRed();
  //   break;
  // case BIG_BLUE:
  //   bigBlue();
  //   break;
  // }
}