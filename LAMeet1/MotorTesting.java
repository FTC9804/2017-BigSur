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

@Autonomous(name = "MotorTest", group = "AutoWithFunctions")
//@Disabled
public class MotorTesting extends Functions {

    public void runOpMode() throws InterruptedException {
        //motor configurations located in the hardware map

        //original configuration for motors, servos, and sensors
        Configure();

        //wait for the code to start to begin the autonomous program
        waitForStart();

        //calibrate gyro after waitForStart to prevent gyro drift from announcers talking
        calibrateGyro();

        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne<5)
        {
            timeTwo=this.getRuntime();
            leftMotor1.setPower(.5);
            leftMotor2.setPower(.5);
            rightMotor1.setPower(0);
            rightMotor2.setPower (0);
        }

        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne<5)
        {
            timeTwo=this.getRuntime();
            leftMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor1.setPower(.5);
            rightMotor2.setPower (.5);
        }

    }

}
