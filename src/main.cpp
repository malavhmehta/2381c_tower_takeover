#include "main.h"
#include "pros/api_legacy.h"
#include <array>
#include "pid.hpp"

#define QUAD_TOP_PORT 1
#define QUAD_BOTTOM_PORT 2

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

int delay = 20;

std::array<double, 3> anglerPIDParams = {0.04 * 4, 0.000555 * 4, 0.7 * 3.5};
std::array<double, 3> drivebasePIDParams = {0.3, 0, 0};

PID *anglerPIDController = new PID(
    &anglerPIDParams[0],
    &anglerPIDParams[1],
    &anglerPIDParams[2]);

PID *drivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

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

enum AUTON_POS
{
  BIG_RED,
  BIG_BLUE,
  SMALL_RED,
  SMALL_BLUE
};

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

AUTON_POS startingPosition = BIG_RED;

void autonStack()
{
  while (true)
  {
    double movFactor = anglerPIDController->update(-720, rightBack.get_position());

    leftFront.move(-movFactor);
    leftBack.move(-movFactor);
    rightFront.move(movFactor);
    rightBack.move(movFactor);

    if (abs(rightBack.get_position() + 730) < 20)
    {
      rightBack.tare_position();
      break;
    }

    pros::delay(delay);
  }
}

void moveRobot(double encoderValue, DIRECTION direction)
{
  int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;

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

void initialize()
{
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Go 2381C!");
}

void disabled()
{
}

void competition_initalize()
{
}

void opcontrol()
{
  int angle = 0;
  int counter = 0;

  int analogSensitivity = 0.8;
  int encoderVal = encoder.get_value();

  while (encoder.get_value() < 1000)
  {
    // Move forward for 1000 ticks
    pros::lcd::set_text(1, std::to_string(encoderVal));
    pros::delay(50);
  }

  while (true)
  {
    pros::delay(delay * 2);

    angle = gyro.get_value();

    // Split arcade controls
    leftFront.move(-1*master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + analogSensitivity * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    leftBack.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) - analogSensitivity * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rightFront.move(-1*master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) - analogSensitivity * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rightBack.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + analogSensitivity * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

    // Stop intake
    leftIntake.move_velocity(0);
    rightIntake.move_velocity(0);

    leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    center.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Transmission drive moves angler
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
      lift.move(-master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    }

    // Intake controls
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)))
    {
      leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      leftIntake.move_velocity(200);
      rightIntake.move_velocity(-200);
      pros::delay(20);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))
    {
      leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      leftIntake.move_velocity(-200);
      rightIntake.move_velocity(200);
      pros::delay(20);
    }

    // Goofy controls
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
      center.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      center.move_velocity(50);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      center.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      center.move_velocity(-50);
    }
  }
}

void bigRed()
{
  leftBack.move(-60);
  leftFront.move(60);
  rightBack.move(60);
  rightFront.move(-60);

  rightIntake.move(-186 / 2);
  leftIntake.move(186 / 2);

  pros::delay(1200);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  rightIntake.move(0);
  leftIntake.move(0);

  leftBack.move_velocity(60);
  leftFront.move_velocity(-60);
  rightBack.move_velocity(-60);
  rightFront.move_velocity(60);

  pros::delay(800);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  pros::delay(20);

  leftBack.move(70);
  leftFront.move(-70);
  rightBack.move(70);
  rightFront.move(-70);

  pros::delay(650);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);
  pros::delay(20);
  leftBack.move(-60);
  leftFront.move(60);
  rightBack.move(60);
  rightFront.move(-60);

  pros::delay(1300);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  rightIntake.move(186 / 2);
  leftIntake.move(-186 / 2);

  pros::delay(1500);

  rightIntake.move(0);
  leftIntake.move(0);

  leftBack.move(60);
  leftFront.move(-60);
  rightBack.move(-60);
  rightFront.move(60);

  pros::delay(2000);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);
}

void bigBlue()
{
  leftBack.move(-60);
  leftFront.move(60);
  rightBack.move(60);
  rightFront.move(-60);

  rightIntake.move(-186 / 2);
  leftIntake.move(186 / 2);

  pros::delay(1200);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  rightIntake.move(0);
  leftIntake.move(0);

  leftBack.move_velocity(60);
  leftFront.move_velocity(-60);
  rightBack.move_velocity(-60);
  rightFront.move_velocity(60);

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
  leftBack.move(-60);
  leftFront.move(60);
  rightBack.move(60);
  rightFront.move(-60);

  pros::delay(1300);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  rightIntake.move(186 / 2);
  leftIntake.move(-186 / 2);

  pros::delay(1500);

  rightIntake.move(0);
  leftIntake.move(0);

  leftBack.move(60);
  leftFront.move(-60);
  rightBack.move(-60);
  rightFront.move(60);

  pros::delay(2000);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);
}

void autonomous()
{
  lift.move_velocity(-80);
  pros::delay(1800);

  //lift.move_velocity(0);
  //lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_velocity(0);
  lift.move_velocity(20);
  pros::delay(100);
  lift.move_velocity(0);

  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  leftFront.move_velocity(-90);
  leftBack.move_velocity(-90);
  rightBack.move_velocity(90);
  rightFront.move_velocity(90);

  pros::delay(500);

  leftFront.move_velocity(-0);
  leftBack.move_velocity(-0);
  rightBack.move_velocity(0);
  rightFront.move_velocity(0);
  lift.move_velocity(0);
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  // lift.move_velocity(-30);
  // pros::delay(800);

  // pros::delay(1500);
  // leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  // leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  // rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  // rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  leftFront.move_velocity(10);
  leftBack.move_velocity(10);
  rightBack.move_velocity(-10);
  rightFront.move_velocity(-10);
  lift.move_velocity(10);
  pros::delay(1000);
  lift.move_velocity(0);

  leftFront.move_velocity(-0);
  leftBack.move_velocity(-0);
  rightBack.move_velocity(0);
  rightFront.move_velocity(0);

  pros::delay(1200);

  bigRed();
}