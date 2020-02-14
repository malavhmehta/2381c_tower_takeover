

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
  while(lift.get_position() > -1900) {
    lift.move(-200);
  } 

  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
    return;
  }

  leftIntake.move(-100);
  rightIntake.move(100);
  pros::delay(800);

  leftIntake.move(0);
  rightIntake.move(0);

  lift.move(180);
  pros::delay(600);
  lift.move(0);

  leftBack.move(50);
  leftFront.move(-50);
  rightBack.move(-50);
  rightFront.move(50);

  pros::delay(800);

  leftBack.move(0);
  leftFront.move(0);
  rightBack.move(0);
  rightFront.move(0);

  pros::delay(400);
  
  return;
}

/**
 * Initiates the autonomous sequence during a competition match.
 */


