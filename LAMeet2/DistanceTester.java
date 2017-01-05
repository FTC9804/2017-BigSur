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

@Autonomous(name = "Distance7", group = "AutoWithFunctions")
//@Disabled
public class DistanceTester extends Functions {

    public void runOpMode() throws InterruptedException {
        //motor configurations located in the hardware map

        //original configuration for motors, servos, and sensors
        Configure();

        //wait for the code to start to begin the autonomous program
        waitForStart();

        //calibrate gyro after waitForStart to prevent gyro drift from announcers talking
        calibrateGyro();

        drive(45, .4, 0);

        stopDrivingAndPause();

        pivot(-88);

        stopDrivingAndPause();

        drive(20, .4, -88);

        stopDrivingAndPause();

        pivot(0);

        stopDrivingAndPause();

        drive(10, .4, -5);

        stopDrivingAndPause();

        pivot(-5);

        stopDrivingAndPause();

        drive(10, .4, -5);

        stopDrivingAndPause();

        pivot(0);

        stopDrivingAndPause();

        driveToWhiteLine();

    }

}
