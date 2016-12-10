/*Code written by Steve, Etienne, & Marcus Fri. 2 Dec. 2016
* updated with new functions code

*   -Sunday, December 9, 2016: Late Afternoon --> introducing new code with functions
*
* FTC Team 9804 Bomb Squad
*
* Version 1: creates code for full movement
*
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import android.graphics.Color;

@Autonomous(name = "BigSurAutoFunctions", group = "AutoWithFunctions")
//@Disabled
public class BigSurAutoFunctions extends Functions {

    public void runOpMode() throws InterruptedException {

        //motor configurations located in the hardware map

        //original configuration for motors, servos, and sensors
        Configure();

        //wait for the code to start to begin the autonomous program
        waitForStart();

        //calibrate gyro after waitForStart to prevent gyro drift from announcers talking
        calibrateGyro();

        //drive initially forward to get into shooting range
        drive (21,.5,0);

        //launch elevator and shooting protocol to launch particles
        shootAndLift (12,3050,.95,.95);

        //stop shooting motors to conserve battery power for remainder of auto and teleop
        stopShooting();

        //drive more to place ourselves farther from the corner vortex
        drive (11,.5,0);

        //spin move counter clockwise
        spinMove(90);

        //drive to align with first beacon
        drive(54, .5, 90);

        //90º clockwise to put ourselves in a line with beacons
        spinMove(0);

        //function travelling infinite distance until white line is reached
        // *******ADD TIME CONSTRAINT ON THIS ACTION********
        driveToWhiteLine();

        //activate beacon pressers for the first beacon
        // **************RETRACT BEACON PRESSERS**********
        findAndPressBeacon();

        //drive until second line is reached
        driveToWhiteLine();

        //press second beacon to get lots of points
        findAndPressBeacon();

        //spin move to face ball in the center
        spinMove(-135);

        drive(100, .5, 0);

        stopDriving();
        //END CODE
    }
}
