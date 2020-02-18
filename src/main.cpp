/*
 * Created by: Team 2381C (Colonel By Secondary School)
 * For:        VEX VRC Competition (Tower Takeover) 2019-2020
 * On:         January 25, 2020 (for Terrebonne Competition)
 */

// Including required .
#include "main.h"
#include "pros/api_legacy.h"
#include <array>
#include "pid.hpp"
#include "definitions.hpp"
#include "Autonomous Code/compAuton.cpp"
#include "Autonomous Code/bigAuton.cpp"


// ENUMERATED variables used for when these variables are passed as parameters in functions.

// Possible starting POSITION for the AUTONOMOUS sequence.
enum AUTON_POS
{
  BIG_RED,
  BIG_BLUE,
  SMALL_RED,
  SMALL_BLUE,
  SAFE_AUTON
};

// Define the starting AUTONOMOUS POSITION at the top, which can be changed match after match.
AUTON_POS startingPosition = BIG_BLUE;

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

// Initializing the PIDs using the above paramenters, by calling the PID class.
PID *anglerPIDController = new PID(
    &anglerPIDParams[0],
    &anglerPIDParams[1],
    &anglerPIDParams[2]);

PID *drivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

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

  if (intakeMove == 1)
  {
    if (intakeSpeed < 0)
    {
      intakeL = 1;
      intakeR = -1;
    }
    else
    {
      intakeR = 1;
      intakeL = -1;
    }
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
  return;
}

/**
 * Manually stopping the drive base by setting all of the robots to have a delay of ZERO.
 * This functions simply makes a call to the pre-existing (defined above) function named
 * moveRobotManual(), and sets the parameters of this function to 0 to stop the movement.
 */
void stopDrivebase()
{
  moveRobotManual(FORWARD, 100, 0, 0, 0);
  return;
}

/**
 * This function moves the center motor of the H-drive to enable strafing. The purpose of
 * this is to manually move the H-drive during the autonomous sequence, as strafing will
 * make the planned drive pathway shorter.
 *
 * @param direction   determines which direction the robot will strafe (LEFT/RIGHT)
 * @param motorSpeed  sets the speed of the center motor
 * @param motorDelay  determines for how long the motors will be moving
 */
void strafeRobot(DIRECTION direction, int motorSpeed, int motorDelay)
{
  int speedCof = direction == LEFT ? -1 : 1;
  center.move(speedCof * motorSpeed);
  pros::delay(motorDelay);
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
void moveRobot(double encoderValue, DIRECTION direction, int intakeMove, int intakeSpeed)
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

  case RIGHT:
    cofRB = -1;
    cofLB = -1;
    break;

  case LEFT:
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

  if (intakeMove == 1)
  {
    intakeR = -1;
    intakeL = 1;
  }

  while (true)
  {
    double movFactor = drivebasePIDController->update(abs(encoderValue), abs(rightBack.get_position()));

    if (movFactor > 65 && abs(rightBack.get_position()) < 400)
    {
      movFactor = 60;
    }

    

    if(encoderValue == 3100 - 900) {
      if(movFactor < 20) {
        movFactor = 20;
      }
      else {
        movFactor * 0.65;
      }
    }

    if(encoderValue == 3200 + 900) {
      movFactor = 150;
    }

    if(encoderValue == -900 - 800) {
      movFactor = movFactor * 1.4;
    }

    if(abs(encoderValue) - 900 < 1000) {
      movFactor = 60;
    }

    if(direction == LEFT || direction == RIGHT) {
      movFactor = 70;
    }

    if(encoderValue == 1300) {
      movFactor = 50;
    }

    if (encoderValue < 0)
    {
      if (rightBack.get_position() - 1100 < encoderValue)
      {
        pros::lcd::set_text(5, "Hi");

        rightBack.tare_position();

        leftFront.move(0);
        leftBack.move(0);
        rightFront.move(0);
        rightBack.move(0);

        pros::delay(100);
        return;
      }
    }
    else
    {
      if (rightBack.get_position() + 1100 > encoderValue)
      {
        pros::lcd::set_text(5, "Hi");

        rightBack.tare_position();

        leftFront.move(0);
        leftBack.move(0);
        rightFront.move(0);
        rightBack.move(0);

        pros::delay(100);
        return;
      }
    }

    pros::lcd::set_text(2, std::to_string(movFactor));

    leftBack.move(movFactor * cofLB);
    leftFront.move(movFactor * cofLF);
    rightBack.move(movFactor * cofRB);
    rightFront.move(movFactor * cofRF);

    leftIntake.move(intakeSpeed * intakeL);
    rightIntake.move(intakeSpeed * intakeR);



    pros::lcd::set_text(1, std::to_string(rightBack.get_position()));
    pros::lcd::set_text(2, std::to_string(movFactor));
    pros::lcd::set_text(3, std::to_string(encoderValue));
    pros::delay(delay);
  }
}

/**
 * Calls the angler PID to stack the cubes. Essentially, this function creates a while loop
 * under which the program is to stack the cubes. For each iteration of the loop, a new value
 * is received from the angler PID controller, which is then passed into each motor, and the
 * motors are then moved accordingly.
 */
void autonStack(double reset)
{
  pros::delay(200);

  //rightBack.tare_position();

  while (true)
  {

    double movFactor = anglerPIDController->update(reset + 1400, rightBack.get_position());

     if(movFactor < 30) {
      movFactor = 30;
      pros::lcd::set_text(9, "const speed");
    }


    pros::lcd::set_text(1, "Motor position: " + std::to_string(rightBack.get_position()));
    pros::lcd::set_text(2, "Motor speed: " + std::to_string(movFactor));

    // Using the transmission for the stacking of cubes.
    leftFront.move(-movFactor);
    leftBack.move(-movFactor);
    rightFront.move(movFactor);
    rightBack.move(movFactor);

    if(abs(rightBack.get_position()) < abs(reset - (reset + 500))) {
      leftIntake.move(-25);
      rightIntake.move(25);
    }

    // Checking for the sentinel value (using the motor's encoder values) to determine if
    // the operation has been completed.
    if (rightBack.get_position() > 1450)
    {
      leftFront.move(0);
      leftBack.move(0);
      rightFront.move(0);
      rightBack.move(0);
      rightBack.tare_position();
      break;
    }


    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
      break;
    }

    // Delay as to not overload the motor.
    pros::delay(delay);
  }

  if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {


    pros::delay(700);

    moveRobotManual(REVERSE, 1000, 60, 1, -50);
    stopDrivebase();
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
 * Defines the drivepath for the small blue goal zone. The motions are currently
 * automatically profiled with the use of PIDs.
 * This sequence supports a MAXIMUM of: FOUR (4) cubes.
 */
void smallBlue()
{
  deploy();

  lift.move(17);
  /*
   * Move FORWARD while INTAKING. This will get the 4 cubes which are directly in front
   * of the robot from its starting position. By the end of this motion, the robot will
   * have 4 cubes in the tray (possible 5 based on the placement of the preload after
   * the robot deploys).
   */

  moveRobot(-3400 - 900, FORWARD, 1, 200);
  moveRobot(2840 + 900, REVERSE, 1, 100);


  /*
   * Make a 180 DEGREE TURN, so that the robot is now facing the opposite direction (or
   * facing its starting direction). This is done so that once the robot is moved with
   * its next motion, it will be directly in front of the small goalzone. The robot will
   * complete this motion (rotation) with a left turn, so that it's easier to align
   * in its next motion.
   */
  moveRobot(800 + 900, LEFT, 0, 0);

  /*
   * Move RIGHT by strafing using the H-drive's center wheel. This will allow the robot
   * to move into the right perimeter wall. In doing so, the robot will automatically
   * be aligned to be perfectly in front of the goalzone.
   */
  strafeRobot(RIGHT, 200, 1200);
  strafeRobot(RIGHT, 0, 0);

  /*
   * Move FORWARD, towards the goalzone. The robot will stop slightly away from the goal
   * zone, and will leave the right amount of room for the stacking process.
   */
  //moveRobot(-630 - 900, FORWARD, 0, 0);

  leftBack.move(60);
  leftFront.move(-60);
  rightBack.move(-60);
  rightFront.move(60);

  leftIntake.move(-30);
  rightIntake.move(30);

  pros::delay(630);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  pros::delay(200);

  rightBack.tare_position();

  // STACK the cubes. This is the end of the autonomous sequence.
  autonStack(rightBack.get_position());
}

/**
 * Defines the drivepath for the small red goal zone. The motions are currently
 * automatically profiled with the use of PIDs.
 * This sequence supports a MAXIMUM of: FOUR (4) cubes.
 */

void smallRed()
{
  deploy();

  lift.move(17);
  /*
   * Move FORWARD while INTAKING. This will get the 4 cubes which are directly in front
   * of the robot from its starting position. By the end of this motion, the robot will
   * have 4 cubes in the tray (possible 5 based on the placement of the preload after
   * the robot deploys).
   */

  moveRobot(-3400 - 900, FORWARD, 1, 200);
  moveRobot(2840 + 900, REVERSE, 1, 100);


  /*
   * Make a 180 DEGREE TURN, so that the robot is now facing the opposite direction (or
   * facing its starting direction). This is done so that once the robot is moved with
   * its next motion, it will be directly in front of the small goalzone. The robot will
   * complete this motion (rotation) with a left turn, so that it's easier to align
   * in its next motion.
   */
  moveRobot(-750 - 900, RIGHT, 0, 0);

  /*
   * Move RIGHT by strafing using the H-drive's center wheel. This will allow the robot
   * to move into the right perimeter wall. In doing so, the robot will automatically
   * be aligned to be perfectly in front of the goalzone.
   */
  strafeRobot(LEFT, 200, 2200);
  strafeRobot(LEFT, 0, 0);

  /*
   * Move FORWARD, towards the goalzone. The robot will stop slightly away from the goal
   * zone, and will leave the right amount of room for the stacking process.
   */
  //moveRobot(-630 - 900, FORWARD, 0, 0);
  //forwards();
  
  moveRobot(-600 - 900, FORWARD, 1, -30);
  pros::delay(200);

  rightBack.tare_position();

  // STACK the cubes. This is the end of the autonomous sequence.
  autonStack(rightBack.get_position());
}

/**
 * Defines the drivepath for the big red goal zone. The motions are currently manually
 * profiled using the 'moveRobotManual' function.
 * This sequence supports a MAXIMUM of: ONE (1) cube.
 */

void testRed() {
  //deploy();

  lift.move(17);

  moveRobot(-3100 - 900, FORWARD, 1, 200);
  pros::delay(100);
  moveRobot(400 + 900, LEFT, 0, 0);
  pros::delay(100);
  moveRobot(-650 - 900, FORWARD, 1, 200);
  pros::delay(200);
  moveRobot(-400 - 900, RIGHT, 1, 200);
  pros::delay(300);
  moveRobot(3200 + 900, REVERSE, 1, 50);
  pros::delay(100);
  // moveRobot(-800 - 900, RIGHT, 1, -25);

  // strafeRobot(LEFT, 200, 1200);
  // strafeRobot(LEFT, 0, 0);
  
  // rightBack.tare_position();
  // moveRobot(-300 - 900, FORWARD, 0, 0);


}

void bigRed3() {
  moveRobot(-1300 - 900, FORWARD, 1, 200);

  moveRobot(600 + 900, LEFT, 0, 0);
  moveRobot(-1600 - 900, FORWARD, 1, 200);
  moveRobot(500 + 900, LEFT, 1, 200);
  moveRobot(-800 -900, FORWARD, 1, -23);

  rightBack.tare_position();
  autonStack(rightBack.get_position());

}

void bigRed(int select) {
  if(select == 1) {
    oneBigRed();
  }
  else if(select == 2) {
    bigRed2();

    rightBack.tare_position();
    autonStack(rightBack.get_position());
  }
  else if(select == 3) {
    bigRed3();
  }
}

void bigBlue(int select) {
  if(select == 1) {
    oneBigBlue();
  }
  else if(select == 2) {
    bigBlue2();

    rightBack.tare_position();
    autonStack(rightBack.get_position());
  }
}

void autonomous() {
  bigRed(3);
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
    pros::lcd::set_text(6, " Motor LF " + std::to_string(leftFront.get_temperature()));
    pros::lcd::set_text(7, "Motor RF " + std::to_string(rightFront.get_temperature()));
    pros::lcd::set_text(3, "Goofy temp " + std::to_string(lift.get_temperature()));
    pros::lcd::set_text(8, "Motor RB " + std::to_string(rightBack.get_temperature()));
    pros::lcd::set_text(9, "Motor LB" + std::to_string(leftBack.get_temperature()));

    // Split acrade controls that control the drive base.
    leftFront.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    leftBack.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rightBack.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rightFront.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    center.move(-master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

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

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      rightBack.tare_position();
      autonStack(rightBack.get_position());
    }

    //Tower Macros
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
      toggle++;
      pros::delay(100);

    }

    if(toggle % 2 == 0 && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lift.move(17);
    }

    if (toggle % 2 != 0)
    {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
      {
        control++;

        stop = 4;
        start = 0;

        pros::delay(100);
      }
      else if (control > 2)
      {
        stop = 0;
        control = 0;
        start = 0;
      }
    }

    if(toggle%2 != 0) {
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && control != 0) {
        stop = 3;
        lift.move(-20);
        pros::lcd::set_text(8, "left toggled");
        start++;
      }
      else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && control != 0) {
        stop = 5;
        lift.move(20);
        start++;
      }


      if(start > 0 && !master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        lift.move(-10);
      }

      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        toggle++;
        stop = 6;
        control = 0;


      }

      if (control == 1 && stop == 4)
      {
        if (lift.get_position() >= -1500)
        {
          lift.move(-140);
        }
        else
        {
          lift.move(-10);
        }
      }

      if (control == 2 && stop == 4)
      {
        if (lift.get_position() >= -2300)
        {
          lift.move(-140);
        }
        else
        {
          lift.move(-10);
        }
      }

      if(stop == 0 && stop != 6) {
        if (lift.get_position() < 0)
        {
          lift.move(100);
        }
        else {
          lift.move(17);
        }


      }




    }

    pros::delay(50);
  }
}
