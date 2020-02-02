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
#include "definitions.hpp"
#include "Autonomous Code/compAuton.cpp"

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
  RIGHT_BACK,
  TRANS_UP,
  TRANS_DOWN
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
AUTON_POS startingPosition = SMALL_RED;

/**
 * Calls the angler PID to stack the cubes. Essentially, this function creates a while loop
 * under which the program is to stack the cubes. For each iteration of the loop, a new value
 * is received from the angler PID controller, which is then passed into each motor, and the motors
 * are then moved accordingly.
 */

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

void moveRobotManual(DIRECTION direction, int delay, int movFactor, int intakeMove, int intakeSpeed)
{
  // Motor coefficients for the H-Drive.
  int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;
  int intakeR = 0, intakeL = 0;

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

  case TRANS_UP:
    cofLF = -1;
    cofLB = -1;
    break;

  case TRANS_DOWN:
    cofRB = -1;
    cofRF = -1;
    break;
  }

  if (intakeMove == 1 && intakeSpeed < 0)
  {
    intakeL = 1;
    intakeR = -1;
  }
  else if (intakeMove == 1 && intakeSpeed > 0)
  {
    intakeR = 1;
    intakeL = -1;
  }

  // Move the motors with the movement factor and multiplied by the coefficient which
  // determines whether the movement factor needs to be negative.
  leftBack.move(movFactor * cofLB);
  leftFront.move(movFactor * cofLF);
  rightBack.move(movFactor * cofRB);
  rightFront.move(movFactor * cofRF);

  leftIntake.move(intakeSpeed * intakeL);
  rightIntake.move(intakeSpeed * intakeR);
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

void stopDrivebase()
{
  moveRobotManual(FORWARD, 100, 0, 0, 0);
}

void moveRobot(double encoderValue, DIRECTION direction, int intakeMove, int intakeSpeed)
{

  //rightBack.tare_position();
  // Motor coefficients for the H-Drive.
  int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;
  int intakeR = 0, intakeL = 0;
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

  case TRANS_UP:
    cofLF = -1;
    cofLB = -1;
    break;

  case TRANS_DOWN:
    cofRB = -1;
    cofRF = -1;
    break;
  }

  if (intakeMove == 1 && intakeSpeed < 0)
  {
    intakeL = -1;
    intakeR = 1;
  }
  else if (intakeMove == 1 && intakeSpeed > 0)
  {
    intakeR = -1;
    intakeL = 1;
  }

  while (true)
  {
    double movFactor = drivebasePIDController->update(abs(encoderValue), abs(rightBack.get_position()));
    double adjustment;

    if(movFactor > 80) {
      movFactor = 55;
    }

    if (encoderValue < 0)
    {
      adjustment = movFactor;
    }
    else
    {
      adjustment = movFactor * -1;
    }


    pros::lcd::set_text(2, std::to_string(movFactor));

    leftBack.move(adjustment * cofLB);
    leftFront.move(adjustment * cofLF);
    rightBack.move(adjustment * cofRB);
    rightFront.move(adjustment * cofRF);

    leftIntake.move(intakeSpeed * intakeL);
    rightIntake.move(intakeSpeed * intakeR);

    if (rightBack.get_position() - 1100 < encoderValue)
    {
      pros::lcd::set_text(5, "Hi");

      rightBack.tare_position();

      leftFront.move(0);
      leftBack.move(0);
      rightFront.move(0);
      rightBack.move(0);

      pros::delay(20);
      return;
    }

    pros::lcd::set_text(1, std::to_string(rightBack.get_position()));
    pros::lcd::set_text(2, std::to_string(movFactor));
    pros::lcd::set_text(3, std::to_string(encoderValue));
    pros::delay(delay);
  }

}

void autonStack(double reset)
{
  pros::delay(500);
  
  //rightBack.tare_position();
  
  while (true)
  {

    double movFactor = anglerPIDController->update(reset + 1400, rightBack.get_position());

    pros::lcd::set_text(1, "Motor position: " + std::to_string(rightBack.get_position()));
    pros::lcd::set_text(2, "Motor speed: " + std::to_string(movFactor));

    // Using the transmission for the stacking of cubes.
    leftFront.move(-movFactor);
    leftBack.move(-movFactor);
    rightFront.move(movFactor);
    rightBack.move(movFactor);

    // Checking for the sentinel value (using the motor's encoder values) to determine if the operation
    // has been completed.
    if (rightBack.get_position() > 1280)
    {
      leftFront.move(0);
      leftBack.move(0);
      rightFront.move(0);
      rightBack.move(0);
      rightBack.tare_position();
      break;
    }

    // Delay as to not overload the motor.
    pros::delay(delay);
  }

  pros::delay(1000);

  moveRobotManual(TRANS_DOWN, 500, 20, 0, 0);
  stopDrivebase();
  moveRobotManual(REVERSE, 1000, 40, 1, -80);
  stopDrivebase();
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
 * Primary function containing the necessary code for the driver to drive and maneuver the
 * robot. It also contains the necessary event listeners to trigger the driver's movements
 * and enable macros as requested.
 */
void opcontrol()
{

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

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
      rightBack.tare_position();
      origin = rightBack.get_position();
      autonStack(origin);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
      control = 0;
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
      control = 1;
      rightBack.tare_position();
      origin = rightBack.get_position();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      control = 2;
      rightBack.tare_position();
      origin = rightBack.get_position();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
      control = 3;
      rightBack.tare_position();
      origin = rightBack.get_position();
    }

    if (control != 1 && control != 2)
    {
      lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    if (control == 1)
    {

      if (lift.get_position() > -1900)
      {

        lift.move(-140);
      }
      else
      {
        lift.move(-5);
        control = 4;
      }

      if (rightBack.get_position() < origin + 415)
      {
        leftFront.move(-80);
        leftBack.move(-80);
        rightFront.move(80);
        rightBack.move(80);
      }
    }
    else if (control == 2)
    {
      if (lift.get_position() > -2400)
      {
        lift.move(-140);
      }
      else
      {
        lift.move(-5);
        control = 4;
      }

      if (rightBack.get_position() < 415)
      {
        leftFront.move(-80);
        leftBack.move(-80);
        rightFront.move(80);
        rightBack.move(80);
      }
      else
      {
        if (rightBack.get_position() < 415)
        {
          leftFront.move(0);
          leftBack.move(0);
          rightFront.move(0);
          rightBack.move(0);
        }
      }
    }
    else if (control == 3)
    {
      if (lift.get_position() < 30)
      {
        lift.move(40);
      }
      else
      {
        lift.move(0);
        control = 3;
      }

      if (rightBack.get_position() > origin - 75)
      {
        leftFront.move(10);
        leftBack.move(10);
        rightFront.move(-10);
        rightBack.move(-10);
      }
      else
      {
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

void smallBlue()
{
  moveRobot(-3720, FORWARD, 1, 186 / 2);

  // BACK ROBOT AGAINST WALL
  deploy();
  
  moveRobot(0, REVERSE, 0, 0);

  // Move robot **SLIGHTLY** AWAY from the wall
  moveRobot(0, FORWARD, 0, 0);

  // TURN ROBOT **LEFT**
  moveRobot(0, LEFT, 0, 0);
  
  // MOVE ROBOT TO GOALZONE
  moveRobot(0, FORWARD, 0, 0);

  // STACK CUBES
  // autonStack(0);
}

void smallRed()
{
  moveRobot(-3640 - 900, FORWARD, 1, 160);

  // BACK ROBOT AGAINST WALL
  moveRobot(200 - 900, REVERSE, 0, 0);

  // Move robot **SLIGHTLY** AWAY from the wall
  moveRobot(0, FORWARD, 0, 0);

  // TURN ROBOT **RIGHT**
  moveRobot(0, RIGHT, 0, 0);
  
  // MOVE ROBOT TO GOALZONE
  moveRobot(0, FORWARD, 0, 0);

  // STACK CUBES
  // autonStack(rightBack.get_position());
}

/**
 * Defines the drivepath for the big red goal zone. The motion are currently manually
 * profiled using the 'moveRobotManual' function.
 */

void bigRed()
{
  // Intake the first cube.

  moveRobotManual(FORWARD, 1200, 60, 1, 186 / 2);

  stopDrivebase();
  rightIntake.move(0);
  leftIntake.move(0);

  // Move the robot backwards.
  moveRobotManual(REVERSE, 800, 60, 0, 0);
  stopDrivebase();

  // Turn the robot towards the goal zone (LEFT).
  moveRobotManual(LEFT, 650, 70, 0, 0);
  stopDrivebase();

  // Move the robot all the way towards the goal zone.
  moveRobotManual(FORWARD, 1300, 60, 0, 0);

  // Outtake the cube
  moveRobotManual(FORWARD, 1500, 0, 1, -186 / 2);

  rightIntake.move(0);
  leftIntake.move(0);

  // Drive away from the goal zone.
  moveRobotManual(REVERSE, 2000, 60, 0, 0);
  stopDrivebase();
}

/**
 * Defines the drivepath for the big blue goal zone. The motion are currently manually
 * profiled using the 'moveRobotManual' function.
 */
void bigBlue()
{
  // Intake the first cube.
  moveRobotManual(FORWARD, 1200, 60, 1, 186 / 2);

  stopDrivebase();

  // Move the robot backwards.
  moveRobotManual(REVERSE, 800, 60, 0, 0);
  stopDrivebase();

  // Turn the robot towards the goal zone (RIGHT).
  moveRobotManual(RIGHT, 650, 70, 0, 0);
  stopDrivebase();

  // Move the robot all the way towards the goal zone.
  moveRobotManual(FORWARD, 1300, 60, 0, 0);

  // Outtake the cube.
  moveRobotManual(FORWARD, 1500, 0, 1, -186 / 2);

  // Drive away from the goal zone.
  moveRobotManual(REVERSE, 2000, 60, 0, 0);
  stopDrivebase();
}

void autonomous()
{
  //deploy();
  //pros::delay(1000);

  switch (startingPosition)
  {
  case BIG_RED:
    bigRed();
    break;
  case BIG_BLUE:
    bigBlue();
    break;
  case SMALL_RED:
    smallRed();
    break;
  case SMALL_BLUE:
    smallBlue();
    break;
  }
}