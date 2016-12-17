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
public class BigSurAutoBeaconOnlyBLUE extends Functions {

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
        drive(32, .5, 0);


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
            stopDriving();
        }


        telemetry.addData("starting spin", telemetryVariable);
        telemetry.update();

        //spin move counter clockwise
        spinMove(90);


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
            stopDriving();
        }


        telemetry.addData("starting drive", telemetryVariable);
        telemetry.update();

        //drive to align with first beacon
        drive(54, .5, 90);


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
            stopDriving();
        }


        telemetry.addData("spin move", telemetryVariable);
        telemetry.update();

        //90º clockwise to put ourselves in a line with beacons
        spinMove(0);


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
            stopDriving();
        }


        telemetry.addData("drive to white line", telemetryVariable);
        telemetry.update();

        //function travelling infinite distance until white line is reached
        driveToWhiteLine();


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
            stopDriving();
        }


        telemetry.addData("find and press beacon",telemetryVariable);
        telemetry.update();

        //activate beacon pressers for the first beacon
        findAndPressBlueBeacon();


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
            stopDriving();
        }


        telemetry.addData("drive to white line",telemetryVariable);
        telemetry.update();

        //drive until second line is reached
        driveToWhiteLine();


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
            stopDriving();
        }


        telemetry.addData("find and press beacon", telemetryVariable);
        telemetry.update();

        //press second beacon to get lots of points
        findAndPressBlueBeacon();


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
            stopDriving();
        }


        telemetry.addData("end code", telemetryVariable);
        telemetry.update();

        stopDriving();
        //END CODE
    }
}
