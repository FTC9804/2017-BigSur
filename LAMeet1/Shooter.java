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

@Autonomous(name = "Shooter", group = "AutoWithFunctions")
//@Disabled
public class Shooter extends Functions {

    public void runOpMode() throws InterruptedException {

        //motor configurations located in the hardware map

        //original configuration for motors, servos, and sensors
        shooter = hardwareMap.dcMotor.get("m5");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //wait for the code to start to begin the autonomous program
        waitForStart();

        startShooting(.32);

        try {

            //launch elevator and shooting protocol to launch particles
            shootAndLiftTestingOptions( 2600, .95, .95);

        } catch (InterruptedException e) {

            throwingException= true;

            telemetry.addData("IndexOutOfBoundsException: " , throwingException);

        }

    }
}
