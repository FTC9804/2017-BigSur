
package org.firstinspires.ftc.teamcode;

/**
 *
 * Created by Programmers of FTC Team 9804 Bomb Squad
 *
 * Created on Fri. Jan 6, '17 by Steve Cox
 *      Preliminary code to act as an auto chooser menu before auto begins
 *
 */


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class AutoChooserMenu extends LinearOpMode {

    double timeOne;
    double timeTwo;

    int telemetryVariable = 0;

    boolean choiceNotSelected = true;
    boolean allianceNotSelected = true;
    boolean weAreRed;

    boolean gamepad1IsNotOK = true;
    boolean gamepad2IsNotOK = true;


// F U N C T I O N S   F O R   A U T O   &   T E L E O P


    public void checkAutoMenu()
    {

        checkGamepads();

        checkAutoAlliance();

    }

    public void checkGamepads ()
    {
        //Check Gamepads
        while (gamepad1IsNotOK || !gamepad2IsNotOK) {

            if (gamepad1IsNotOK) {
                telemetry.addData("Start-A Gamepad1", telemetryVariable);
            }
            else {
                telemetry.addData("Gamepad1 is ok", telemetryVariable);
                gamepad1IsNotOK = false;
            }

            if (gamepad2IsNotOK) {
                telemetry.addData("Start-B Gamepad2", telemetryVariable);
            }
            else {
                telemetry.addData("Gamepad2 is ok", telemetryVariable);
                gamepad2IsOK = false;
            }
            telemetry.update;

    }

    public void checkAutoAlliance ()
    {
        telemetry.clearData;
        while (choiceNotSelected)   {
            if (allianceNotSelected){
                telemetry.addData("Choose Alliance Color", telemetryVariable);
                telemetry.update();
                if (gamepad1.x) {
                    weAreRed = false;
                    allianceNotSelected = false;
                }
                if (gamepad1.b) {
                    weAreRed = true;
                    allianceNotSelected = false;
                }

            }
            if (!allianceNotSelected) {
                telemetry.addData("Confirm your color choice", telemetryVariable);

                if (weAreRed) {
                    telemetry.addData("We are RED", telemetryVariable);
                }
                else {
                    telemetry.addData("We are BLUE", telemetryVariable);
                }

                telemetry.addData("Y is correct.  A is incorrect", telemetryVariable);

                if (gamepad1.y){
                    choiceNotSelected = false;
                }
                if (gamepad1.a) {
                    allianceNotSelected = true;
                }
            }
        }
    }

}
