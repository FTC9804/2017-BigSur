/*Code written by Steve, Etienne, & Marcus Fri. 2 Dec. 2016
* updated with new functions code
*
Etienne Lunetta
*   -Sunday, December 4, 2016: 1:54 AM
*
* FTC Team 9804 Bomb Squad
* Made by the programmers of FTC Team 9804 Bomb Squad
*
* Version 1: creates code for full movement
*   Choose RED/BLUE alliance
*   Forward with intake
*   Shoot all balls
*   90º
*   FWD
*   90º
*   Check for line, beacon 1 operation
*   Check for line, beacon 2 operation
*   120º
*   FWD to knock off ball
*
*   EDITED SUNDAY, DECEMBER 4th, 2016
*       -Start
*       Drive forward 2 feet
*
* //
*  Steve Cox 10-28-16: Reflecting Data of TestBot
* +++++++++++++++++++++++++++++++TestBot Configuration Info+++++++++++++++++++++++++++++++
*
* SN            NAME                    PORT            CONFIG. NAME
* not yet available
* NOTE ABOUT THE CODE:
*  Every time you update a name/type or add a new motor/servo/sensor/etc,
*  add a line like the above template in order to have code organization.
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

@Autonomous(name = "BigSurAutoFunctions", group = "Auto")
//@Disabled
public class BigSurAutoFunctions extends Functions {

    public void runOpMode() throws InterruptedException {
        //motor configurations in the hardware map

        Configure();

        waitForStart();

        calibrateGyro();

        drive (21,.5,0);

        sleep (2525);

        shootAndLift (12,3050,.95,.95);

        stopShooting();

        drive (11,.5,0);

        spinMove(90);

        drive(54, .5, 90);

        spinMove(0);

        driveToWhiteLine();

        findAndPressBeacon();

        drive (30, .5, 0);

        driveToWhiteLine();

        findAndPressBeacon();

        spinMove(-135);

        drive(100, .5, 0);
    }
}
