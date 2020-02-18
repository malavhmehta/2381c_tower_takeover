void oneBigRed()
{
  
  deploy();
  
  oldBigRed();

}

/**
 * Defines the drivepath for the big blue goal zone. The motion are currently manually
 * profiled using the 'moveRobotManual' function.
 * This sequence supports a MAXIMUM of: ONE (1) cube.
 */

void oneBigBlue()
{

  deploy();

  oldBigBlue();

}


void bigRed2() {
    leftBack.move(60);
    leftFront.move(-60);
    rightBack.move(-60);
    rightFront.move(60);

    rightIntake.move(-150);
    leftIntake.move(150);

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

    pros::delay(900);

    leftBack.move(0);
    leftFront.move(0);
    rightBack.move(0);
    rightFront.move(0);

    pros::delay(20);

    leftBack.move(70);
    leftFront.move(-70);
    rightBack.move(70);
    rightFront.move(-70);
    
    leftIntake.move(-30);
    rightIntake.move(30);

    pros::delay(650);
    
    leftIntake.move(0);
    rightIntake.move(0);

    leftBack.move(0);
    leftFront.move(0);
    rightBack.move(0);
    rightFront.move(0);
    pros::delay(20);
    leftBack.move(70);
    leftFront.move(-70);
    rightBack.move(-70);
    rightFront.move(70);

    pros::delay(1600);

    leftBack.move(0);
    leftFront.move(0);
    rightBack.move(0);
    rightFront.move(0);
}

void bigBlue2() {
    leftBack.move(60);
    leftFront.move(-60);
    rightBack.move(-60);
    rightFront.move(60);

    rightIntake.move(-150);
    leftIntake.move(150);

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

    pros::delay(900);

    leftBack.move(0);
    leftFront.move(0);
    rightBack.move(0);
    rightFront.move(0);

    pros::delay(20);

    leftBack.move(-70);
    leftFront.move(70);
    rightBack.move(-70);
    rightFront.move(70);
    
    leftIntake.move(-30);
    rightIntake.move(30);

    pros::delay(650);
    
    leftIntake.move(0);
    rightIntake.move(0);

    leftBack.move(0);
    leftFront.move(0);
    rightBack.move(0);
    rightFront.move(0);
    pros::delay(20);
   
    leftBack.move(70);
    leftFront.move(-70);
    rightBack.move(-70);
    rightFront.move(70);

    pros::delay(1600);

    leftBack.move(0);
    leftFront.move(0);
    rightBack.move(0);
    rightFront.move(0);
}