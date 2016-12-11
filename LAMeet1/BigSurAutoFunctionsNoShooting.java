/*Code written by Steve, Etienne, & Marcus Fri. 2 Dec. 2016
* updated with new functions code

*   -Sunday, December 10, 2016: Night --> introducing new code with functions, excluding shooting command
*
* FTC Team 9804 Bomb Squad
*
* Version 1: creates code for full movement
*
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BigSurAutoFunctionsNoShooting", group = "AutoWithFunctions")
//@Disabled
public class BigSurAutoFunctionsNoShooting extends Functions {

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

        //drive initially forward to get into shooting range
        drive(21, .5, 0);

        telemetry.addData("starting drive 2", telemetryVariable);
        telemetry.update();

        //drive more to place ourselves farther from the corner vortex
        drive(11, .5, 0);

        telemetry.addData("starting spin", telemetryVariable);
        telemetry.update();

        //spin move counter clockwise
        spinMove(90);

        telemetry.addData("starting drive", telemetryVariable);
        telemetry.update();

        //drive to align with first beacon
        drive(54, .5, 90);

        telemetry.addData("spin move", telemetryVariable);
        telemetry.update();


        //90º clockwise to put ourselves in a line with beacons
        spinMove(0);

        telemetry.addData("drive to white line", telemetryVariable);
        telemetry.update();

        drive(24, .5, 0);

        //function travelling infinite distance until white line is reached
        driveToWhiteLine();

        telemetry.addData("find and press beacon",telemetryVariable);
        telemetry.update();

        //activate beacon pressers for the first beacon
        findAndPressBeacon();

        telemetry.addData("drive to white line",telemetryVariable);
        telemetry.update();

        //drive until second line is reached
        driveToWhiteLine();

        telemetry.addData("find and press beacon", telemetryVariable);
        telemetry.update();

        //press second beacon to get lots of points
        findAndPressBeacon();
/*
        telemetry.addData("spin move", telemetryVariable);
        telemetry.update();

        //spin move to face ball in the center
        spinMove(-135);

        telemetry.addData("drive", telemetryVariable);
        telemetry.update();

        drive(100, .5, 0);
*/
        telemetry.addData("end code", telemetryVariable);
        telemetry.update();

        stopDriving();
        //END CODE
    }
}
