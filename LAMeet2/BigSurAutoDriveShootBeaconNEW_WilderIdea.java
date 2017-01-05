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


@Autonomous(name = "Any Alliance, Beacon Only", group = "AutoWithFunctions")
//@Disabled
public class BigSurAutoDriveShootBeaconNEW_WilderIdea extends FunctionsJAN3Good {

    public void runOpMode() throws InterruptedException {

        final int drive1 = 6;
        final double speed1 = .5;
        final double heading1 = 0;
        final double spinHeading1 = 45;

        //motor configurations located in the hardware map

        //original configuration for motors, servos, and sensors
        Configure();


        checkAutoAlliance();

        //wait for the code to start to begin the autonomous program
        waitForStart();

        //calibrate gyro after waitForStart to prevent gyro drift from announcers talking
        calibrateGyro();

        //drive initially forward to get into shooting range
        drive(drive1, speed1, heading1);

        stopDrivingAndPause();

        //spin move
        spinMove(45);

        stopDrivingAndPause();

        //function travelling infinite distance until white line is reached
        driveToWhiteLine();

        stopDrivingAndPause();

        proportionalLineFollowRIGHTSideOfLineRIGHTSensor(12, 0.4);

        //INSERT CHECK FOR BEACONS

        stopDriving();
        //END CODE
    }
}
