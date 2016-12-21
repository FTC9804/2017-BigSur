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

@Autonomous(name = "Shooter Auto Red", group = "AutoWithFunctions")
//@Disabled
public class shooterAutoRed extends Functions {

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

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo-timeOne<1.2)
        {
            leftMotor1.setPower(.4);
            leftMotor2.setPower(.4);
            rightMotor1.setPower(.4);
            rightMotor2.setPower(.4);
            timeTwo = this.getRuntime();
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

        shooter.setPower(-.5);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo-timeOne<5)
        {
            timeTwo= this.getRuntime();
        }



        intake.setPower(.95);
        elevator.setPower(.95);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo-timeOne<5)
        {
            timeTwo= this.getRuntime();
        }


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo-timeOne<.6)
        {
            leftMotor1.setPower(.4);
            leftMotor2.setPower(.4);
            rightMotor1.setPower(.4);
            rightMotor2.setPower(.4);
            timeTwo = this.getRuntime();
        }

    }
}
