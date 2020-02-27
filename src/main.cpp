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

std::array<double,3> pidValues = {1,0,0};
PID* gripPid = new PID (&pidValues[0], &pidValues[1], &pidValues[2]);

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
// void strafeRobot(DIRECTION direction, int motorSpeed, int motorDelay)
// {
//   int speedCof = direction == LEFT ? -1 : 1;
//   center.move(speedCof * motorSpeed);
//   pros::delay(motorDelay);
// }

int resetCof(int cof)
{
  int newCof = 1;

  if (cof < 0)
  {
    newCof = -1;
  }

  return newCof;
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
  double cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;
  //double cofChange = 0.02;
  // int lastHeading = (int)inertial.get_heading();
  // int currentHeading = (int)inertial.get_heading();
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

    if (encoderValue == -3300 - 900)
    {
      if (movFactor < 20)
      {
        movFactor = 20;
      }
      else
      {
        movFactor * 0.6;
      }
    }

    if (encoderValue == -900 - 800)
    {
      movFactor = movFactor * 1.4;
    }

    if (abs(encoderValue) - 900 < 1000)
    {
      movFactor = 60;
    }

    if (direction == LEFT || direction == RIGHT)
    {
      movFactor = 70;
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

    // if (direction == FORWARD || direction == REVERSE)
    // {
    //   currentHeading = inertial.get_heading();
    //   if ((currentHeading - lastHeading) >= 2)
    //   {
    //     cofRF += cofChange;
    //     cofRB += cofChange;
    //     cofLF = resetCof(cofLF);
    //     cofLB = resetCof(cofLB);
    //   }
    //   else if ((lastHeading - currentHeading) >= 2)
    //   {
    //     cofRF = resetCof(cofRF);
    //     cofRB = resetCof(cofRB);
    //     cofLF += cofChange;
    //     cofLB += cofChange;
    //   }
    //   else
    //   {
    //     cofRF = resetCof(cofRF);
    //     cofRB = resetCof(cofRB);
    //     cofLF = resetCof(cofLF);
    //     cofLB = resetCof(cofLB);
    //   }
    // }

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

    if (movFactor < 30)
    {
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

    if (abs(rightBack.get_position()) < abs(reset - (reset + 700)))
    {
      leftIntake.move(-14);
      rightIntake.move(14);
    }

    // Checking for the sentinel value (using the motor's encoder values) to determine if
    // the operation has been completed.
    if (rightBack.get_position() > 1460)
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

  if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
  {

    pros::delay(800);

    moveRobotManual(TRANS_DOWN, 200, 80, 0, 0);
    moveRobotManual(REVERSE, 1000, 80, 1, -80);
    stopDrivebase();
  }
}

/**
 * Called at the initialization of the competition. Turns on the brain's LCD.
 */
void initialize()
{

  pros::lcd::initialize();
  inertial.reset();
}

/**
 * Defines the drivepath for the small blue goal zone. The motions are currently
 * automatically profiled with the use of PIDs.
 * This sequence supports a MAXIMUM of: FOUR (4) cubes.
 */

void perfectTurn(double encoder, int degree, DIRECTION direction)
{
  // Motor coefficients for the H-Drive.
  int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;
  int turnSwitch = 0;
  // Check the direction, and change the motor coefficient accordingly.
  switch (direction)
  {
  case RIGHT:
    cofRB = -1;
    cofLB = -1;
    break;

  case LEFT:
    cofRF = -1;
    cofLF = -1;
    break;
  }

  while (true)
  {
    pros::delay(20);

    double movFactor = drivebasePIDController->update(abs(encoder), abs(rightBack.get_position()));

    if (movFactor < 10)
    {
      movFactor = 10;
    }

    pros::lcd::set_text(2, std::to_string(inertial.get_heading()));
    pros::lcd::set_text(3, std::to_string(movFactor));

    leftBack.move(movFactor * cofLB * 4);
    leftFront.move(movFactor * cofLF * 4);
    rightBack.move(movFactor * cofRB * 4);
    rightFront.move(movFactor * cofRF * 4);


    if (inertial.get_heading() >= degree - 1 && inertial.get_heading() <= degree + 1)
    {
      pros::lcd::set_text(5, "Hi");

      rightBack.tare_position();

      leftFront.move(0);
      leftBack.move(0);
      rightFront.move(0);
      rightBack.move(0);

      break;
    }
  }
}

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

  moveRobot(-3200 - 900, FORWARD, 1, 200);
  moveRobot(3200 + 900, REVERSE, 1, 100);

  /*
   * Make a 180 DEGREE TURN, so that the robot is now facing the opposite direction (or
   * facing its starting direction). This is done so that once the robot is moved with
   * its next motion, it will be directly in front of the small goalzone. The robot will
   * complete this motion (rotation) with a left turn, so that it's easier to align
   * in its next motion.
   */
  moveRobot(-650 - 900, RIGHT, 0, 0);

  /*
   * Move RIGHT by strafing using the H-drive's center wheel. This will allow the robot
   * to move into the right perimeter wall. In doing so, the robot will automatically
   * be aligned to be perfectly in front of the goalzone.
   */
  // strafeRobot(LEFT, 200, 1800);
  // strafeRobot(LEFT, 0, 0);

  /*
   * Move FORWARD, towards the goalzone. The robot will stop slightly away from the goal
   * zone, and will leave the right amount of room for the stacking process.
   */
  //moveRobot(-630 - 900, FORWARD, 0, 0);
  //forwards();

  moveRobot(-650 - 900, FORWARD, 1, -40);
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

  moveRobot(-3300 - 900, FORWARD, 1, 200);
  moveRobot(3000 + 900, REVERSE, 1, 100);

  /*
   * Make a 180 DEGREE TURN, so that the robot is now facing the opposite direction (or
   * facing its starting direction). This is done so that once the robot is moved with
   * its next motion, it will be directly in front of the small goalzone. The robot will
   * complete this motion (rotation) with a left turn, so that it's easier to align
   * in its next motion.
   */
  moveRobot(700 + 900, LEFT, 0, 0);

  /*
   * Move RIGHT by strafing using the H-drive's center wheel. This will allow the robot
   * to move into the right perimeter wall. In doing so, the robot will automatically
   * be aligned to be perfectly in front of the goalzone.
   */
  // strafeRobot(RIGHT, 200, 1800);
  // strafeRobot(LEFT, 0, 0);

  /*
   * Move FORWARD, towards the goalzone. The robot will stop slightly away from the goal
   * zone, and will leave the right amount of room for the stacking process.
   */
  //moveRobot(-630 - 900, FORWARD, 0, 0);
  //forwards();

  moveRobot(-700 - 900, FORWARD, 1, -30);
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


void bigRed3()
{
  moveRobot(-1300 - 900, FORWARD, 1, 200);

  moveRobot(600 + 900, LEFT, 0, 0);
  moveRobot(-1600 - 900, FORWARD, 1, 200);
  moveRobot(500 + 900, LEFT, 1, 200);
  moveRobot(-800 - 900, FORWARD, 1, -35);

  rightBack.tare_position();
  autonStack(rightBack.get_position());
}

void bigRed4()
{
  deploy();
  pros::delay(300);

  lift.move(17);
  moveRobot(-1000 - 900, FORWARD, 1, 200);
  moveRobot(-700 - 900, RIGHT, 0, 0);
  moveRobot(-800 - 900, FORWARD, 1, 150);
  pros::delay(100);
  moveRobot(900 + 900, LEFT, 1, 100);
  pros::delay(100);
  moveRobot(-2280 - 900, FORWARD, 1, 200);
  moveRobot(390 + 900, LEFT, 1, 200);
  moveRobot(-500 - 900, FORWARD, 1, -44);

  rightBack.tare_position();
  autonStack(rightBack.get_position());
}

void bigBlue4() {
  //deploy();
  pros::delay(300);

  lift.move(17);
  moveRobot(-1000 - 900, FORWARD, 1, 200);
  moveRobot(670 + 900, LEFT, 0, 0);
  moveRobot(-1000 - 900, FORWARD, 1, 150);
  
  moveRobot(-1160 - 900, RIGHT, 1, 100);
  moveRobot(-2400 - 900, FORWARD, 1, 200);
  moveRobot( -450 - 900, RIGHT, 1, 200);
  moveRobot(-1210 - 900, FORWARD, 1, -27);

  rightBack.tare_position();
  autonStack(rightBack.get_position());
}

void bigRed(int select)
{
  if (select == 1)
  {
    oneBigRed();
  }
  else if (select == 2)
  {
    bigRed2();

    rightBack.tare_position();
    autonStack(rightBack.get_position());
  }
  else if (select == 3)
  {
    bigRed3();
  }
  else if(select = 4) {
    bigRed4();
  }
}

void bigBlue(int select)
{
  if (select == 1)
  {
    oneBigBlue();
  }
  else if (select == 2)
  {
    bigBlue2();

    rightBack.tare_position();
    autonStack(rightBack.get_position());
  }
  else if(select == 4) {
    bigBlue4();
  }
}

void callibrateIMU()
{
  inertial.reset();
  pros::delay(2400);
}

void autonomous()
{
  
  smallRed();
}

/**
 * Primary function containing the necessary code for the driver to drive and maneuver the
 * robot. It also contains the necessary event listeners to trigger the driver's movements
 * and enable macros as requested.
 */

int gripToggle = 0;

void opcontrol()
{

  int counter = 0;
  //Grip Max is going to be the position where you want it to go when it is locked
  while (true)
  {
    if(counter == 0 && grip.get_position() < 230) {
      grip.move(80);
    }

    if(grip.get_position() > 230) {
      counter++;
    }
    
    if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && counter != 0) {
      grip.move(4);
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      grip.move(-5);
      gripToggle++;
    }

    if(gripToggle > 0 && !master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      grip.move(-120);
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      grip.move(70);
      gripToggle = 0;
    }


    pros::delay(20);
    
    
    pros::lcd::set_text(4, "Motor pos: " + std::to_string(rightBack.get_position()));
    pros::lcd::set_text(5, "Goofy Position " + std::to_string(lift.get_position()));
    
    pros::lcd::set_text(2, "grip temperature: " + std::to_string(grip.get_temperature()));
    pros::lcd::set_text(1, "grip position: " + std::to_string(grip.get_position()));
    // pros::lcd::set_text(7, "Motor RF " + std::to_string(rightFront.get_temperature()));
    pros::lcd::set_text(3, "Goofy temp " + std::to_string(lift.get_temperature()));
    // pros::lcd::set_text(8, "Motor RB " + std::to_string(rightBack.get_temperature()));
    // pros::lcd::set_text(9, "Motor LB" + std::to_string(leftBack.get_temperature()));
    pros::lcd::set_text(6, "leftIntake:  " + std::to_string(leftIntake.get_temperature()));
    pros::lcd::set_text(7, "rigthIntake: " + std::to_string(rightIntake.get_temperature()));

    
    // Split acrade controls that control the drive base.
    leftFront.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    leftBack.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rightBack.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rightFront.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    
    // Keeping the motors at move_velocity(0) keeps the motor position locked.
    leftIntake.move_velocity(0);
    rightIntake.move_velocity(-0);
    leftIntake.move_velocity(-0);
    rightIntake.move_velocity(0);

    //center.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      leftIntake.move_velocity(55);
      rightIntake.move_velocity(-55);
    }

    if(leftIntake.get_temperature() > 45 || rightIntake.get_temperature() > 45) {
      if(rumble != 1 && rumble < 2 ) {
        master.rumble(".-");
        rumble++;
      }
    }

    if(leftIntake.get_temperature() > 40 || rightIntake.get_temperature() > 40) {
      if(rumble2 != 1 && rumble2 < 2) {
        master.rumble(".");
        rumble2++;
      }
    }

    
    // Shift buttons R1 (for tray tilt) and R2 (for goofy arm).
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
      leftFront.move(-0.6 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      leftBack.move(-0.6 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      rightBack.move(0.6 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      rightFront.move(0.6 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
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

    // Driver control and macros for the goofy arm.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
      toggle++;
      pros::delay(100);
    }

    if (toggle % 2 == 0 && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
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

    if (toggle % 2 != 0)
    {
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        control = 0;
        stop = 4;
      }

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && control != 0)
      {
        stop = 3;
        lift.move(-40);
        pros::lcd::set_text(2, "left toggled");
        start++;
      }
      else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && control != 0)
      {
        stop = 3;
        lift.move(20);
        pros::lcd::set_text(2, "right toggled");
        start++;
      }

      if (start > 0 && !master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
      {
        lift.move(-10);
      }

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
      {
        toggle++;
        stop = 6;
        control = 0;
      }

      if (control == 1 && stop == 4)
      {
        if (lift.get_position() >= -1550)
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

      if (control == 0 && stop != 6)
      {
        if (lift.get_position() < 0)
        {
          lift.move(100);
        }
        else
        {
          lift.move(17);
        }
      }
    }

    pros::delay(50);
  }
}
