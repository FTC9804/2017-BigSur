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

@Autonomous(name = "no encoders", group = "AutoWithFunctions")
//@Disabled
public class BigSurAutoRunNoEncoders extends Functions {

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

        while (this.opModeIsActive()) {

            driveForTimeNoEncoders(2);

            spinMove(90);

            driveForTimeNoEncoders(2);

            spinMove(180);

            driveForTimeNoEncoders(2);

            spinMove(270);

            driveForTimeNoEncoders(2);

            spinMove(0);

        }




        telemetry.addData("end code", telemetryVariable);
        telemetry.update();

        stopDriving();
        //END CODE
    }
}
