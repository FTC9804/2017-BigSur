/*Code written by Steve, Etienne, & Marcus Fri. 2 Dec. 2016
* updated with new functions code

*   -Friday December 16, 2016: Afternoon --> introducing new code with functions, excluding shooting commands and cat ball
*
* FTC Team 9804 Bomb Squad
*
* Version 1: creates code for full movement, BLUE only
*
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "DistanceTestSpin3", group = "AutoWithFunctions")
//@Disabled
public class DistanceTestSpin extends Functions {

    public void runOpMode() throws InterruptedException {
        //motor configurations located in the hardware map

        //original configuration for motors, servos, and sensors
        Configure();

        //wait for the code to start to begin the autonomous program
        waitForStart();

        //calibrate gyro after waitForStart to prevent gyro drift from announcers talking
        calibrateGyro();

        drive(45, .3, 0);

        stopDrivingAndPause();

        spinMove(-88);

        stopDrivingAndPause();

        drive(20, .3, -88);

        stopDrivingAndPause();

        spinMove(0);

        stopDrivingAndPause();

        drive(10, .3, -5);

        stopDrivingAndPause();

        spinMove(-5);

        stopDrivingAndPause();

        drive(10, .3, -5);

        stopDrivingAndPause();

        spinMove(0);

        stopDrivingAndPause();

        driveToWhiteLine();

        stopDrivingAndPause();
    }

}
