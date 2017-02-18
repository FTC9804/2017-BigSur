//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by stevecox on 2/11/17.
 * Purpose of the code is to allow the ability to drive back over a specified distance.
 */

@Autonomous(name = "IT DriveBack", group = "MethodTesting")
//@Disabled
public class IndividualTestDriveBack extends FuctionsForILTNew {

    //OVERALL VARIABLES
    double timeOne, timeTwo, distance = 20, power = .5;
    int stepInMenu = 1, heading = 0;
    boolean notAllChosen = true, choiceNotSelected = true, goBack = false, distanceNotSelected = true, powerNotSelected = true, headingNotSelected = true;

    public void runOpMode() throws InterruptedException
    {




        /**
         *
         * Order of Auto Menu
         *
         * 1. Set distance to travel
         * 2. Set power level to move
         * 3. Set target Gyro Heading
         *
         */

        //Configure motors, servos and sensors
        Configure();

        while (notAllChosen) {

            goBack = false;

            if (stepInMenu == 1) {
                setDistanceToTravel();
            }

            delayShort();


            goBack = false;
            if (stepInMenu == 2) {
                setPowerLevel();
            }

            delayShort();


            goBack = false;
            if (stepInMenu == 3) {
                setTargetHeading();
            }


            if (!distanceNotSelected && !powerNotSelected && !headingNotSelected) {
                notAllChosen = false;
            } else {
                notAllChosen = true;
            }

        }

        telemetry.addData("HAVE A FANTASTIC DAY", telemetryVariable);
        telemetry.update();
        delayLong();


        //Wait until play button is pressed
        waitForStart();

        //calibrateGyro
        calibrateGyro();

        driveBack(distance, power, heading);
    }

    public void setDistanceToTravel ()
    {
        stepInMenu = 1;
        choiceNotSelected = true;
        distanceNotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 1)   {

            if (distanceNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set Target Distance", distance);
                telemetry.addLine("use dpad to adjust speed");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    distance += .5;
                }
                if (gamepad1.dpad_down) {
                    distance -= .5;
                }
                if (gamepad1.start) {
                    distanceNotSelected = false;
                }
            }

            if (!distanceNotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your target distance", distance);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y){
                    choiceNotSelected = false;
                    distanceNotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    distanceNotSelected = true;
                    delayShort();
                }
            }
            telemetry.update();
        }

        stepInMenu = 2;
        choiceNotSelected = true;

    }

    public void setPowerLevel ()
    {
        stepInMenu = 2;
        choiceNotSelected = true;
        powerNotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 2)   {

            if (powerNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set Power Level to Drive At", power);
                telemetry.addLine("use dpad to adjust power");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    power += .01;
                }
                if (gamepad1.dpad_down) {
                    power -= .01;
                }
                if (gamepad1.start) {
                    powerNotSelected = false;
                }
            }

            if (!powerNotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your target power level", power);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y){
                    choiceNotSelected = false;
                    powerNotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    powerNotSelected = true;
                    delayShort();
                }
            }

            if (gamepad1.back) {
                stepInMenu = stepInMenu - 1;
                delayShort();
                goBack = true;
            }

            telemetry.update();
        }

        if (goBack){
            stepInMenu = 1;
            powerNotSelected = true;
            delayShort();
        }
        else {
            goBack = false;
            stepInMenu = 3;
            delayShort();
        }
        choiceNotSelected = true;
        goBack = false;

    }

    public void setTargetHeading ()
    {
        stepInMenu = 3;
        choiceNotSelected = true;
        headingNotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 3)   {

            if (headingNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set heading", heading);
                telemetry.addLine("use dpad to adjust heading");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    heading += 1;
                }
                if (gamepad1.dpad_down) {
                    heading -= 1;
                }
                if (gamepad1.start) {
                    headingNotSelected = false;
                }
            }

            if (!headingNotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your target heading", heading);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y){
                    choiceNotSelected = false;
                    headingNotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    headingNotSelected = true;
                    delayShort();
                }
            }

            if (gamepad1.back) {
                stepInMenu = stepInMenu - 1;
                delayShort();
                goBack = true;
            }
            telemetry.update();
        }

        if (goBack){
            stepInMenu = 2;
            headingNotSelected = true;
            delayShort();
        }
        else {
            stepInMenu = 1;
            delayShort();
        }
        goBack = false;
        choiceNotSelected = true;

    }

    public void delayShort ()
    {
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < .25) {
            timeTwo = this.getRuntime();
        }
    }

    public void delayLong ()
    {
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            timeTwo = this.getRuntime();
        }
    }

}



