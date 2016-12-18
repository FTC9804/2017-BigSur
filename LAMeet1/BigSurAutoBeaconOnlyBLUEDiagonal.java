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

@Autonomous(name = "BLUE Beacon Only", group = "AutoWithFunctions")
//@Disabled
public class BigSurAutoBeaconOnlyBLUEDiagonal extends Functions {

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
        drive(4, .5, 0);


        stopDrivingAndPause();


        telemetry.addData("starting spin", telemetryVariable);
        telemetry.update();

        //spin move clockwise
        spinMove(-45);


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

        findAndPressBlueBeacon();

        telemetry.addData("find and press beacon",telemetryVariable);
        telemetry.update();

        stopDrivingAndPause();


        //drive until second line is reached
        driveToWhiteLine();

        telemetry.addData("drive to white line",telemetryVariable);
        telemetry.update();


        stopDrivingAndPause();


        telemetry.addData("find and press beacon", telemetryVariable);
        telemetry.update();

        //press second beacon to get lots of points
        findAndPressBlueBeacon();


        stopDrivingAndPause();

        telemetry.addData("end code", telemetryVariable);
        telemetry.update();

        stopDriving();
        //END CODE
    }
}
