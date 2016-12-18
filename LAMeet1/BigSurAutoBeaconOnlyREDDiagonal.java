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

@Autonomous(name = "RED Beacon Only Diag", group = "AutoWithFunctions")
//@Disabled
public class BigSurAutoBeaconOnlyREDDiagonal extends Functions {

    public void runOpMode() throws InterruptedException {

        //motor configurations located in the hardware map

        //original configuration for motors, servos, and sensors
        Configure();

        //wait for the code to start to begin the autonomous program
        waitForStart();

        telemetry.addData("Code Starting.  Gyro beginning to calibrate", telemetryVariable);
        telemetry.update();

        //calibrate gyro after waitForStart to prevent gyro drift from announcers talking
        calibrateGyro();

        telemetry.addData("Calibration complete. starting drive 1", telemetryVariable);
        telemetry.update();

        shoot(.5, .2);

        stopDrivingAndPause();

        //drive initially forward to get into shooting range
        pivot(26);


        stopDrivingAndPause();


        telemetry.addData("drive to white line", telemetryVariable);
        telemetry.update();

        //drive to align with first beacon
        driveToWhiteLine();



        stopDrivingAndPause();


        telemetry.addData("spin move", telemetryVariable);
        telemetry.update();

        //90º counter clockwise to put ourselves in a line with beacons
        spinMove(0); //MAKE IT DRIVE HIGHER

        stopDrivingAndPause();

        pivot(-6);

        stopDrivingAndPause();

        driveBack(7, -.3, 0);

        stopDrivingAndPause();

        pivot(0);

        stopDrivingAndPause();

        pivot (currentHeading-2);

        driveToWhiteLine();

        stopDrivingAndPause();


        findAndPressBlueBeacon(3);

        telemetry.addData("find and press beacon",telemetryVariable);
        telemetry.update();

        stopDrivingAndPause();


        //drive until second line is reached
        driveToWhiteLine();

        telemetry.addData("drive to white line",telemetryVariable);
        telemetry.update();


        stopDrivingAndPause();

        pivot(0);


        telemetry.addData("find and press beacon", telemetryVariable);
        telemetry.update();

        //press second beacon to get lots of points
        findAndPressBlueBeacon(3);


        stopDrivingAndPause();

        telemetry.addData("end code", telemetryVariable);
        telemetry.update();

        stopDriving();
        //END CODE
    }
}