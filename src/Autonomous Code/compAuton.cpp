

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

/**
 * Initiates the autonomous sequence during a competition match.
 */


