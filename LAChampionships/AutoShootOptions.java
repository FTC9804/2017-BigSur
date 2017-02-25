//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by stevecox on 2/11/17.
 * Purpose of the code is to allow the ability to drive back over a specified distance, shoot, and keep driving
 */

@Autonomous(name = "Auto Shoot Options", group = "AutoWithFunctions")
//@Disabled
public class AutoShootOptions extends FunctionsForLA {

    //OVERALL VARIABLES
    double timeOne, timeTwo, wait = 0, distance1 = 0, distance2 = 0, power = .5;
    int stepInMenu = 1, heading = 0;
    boolean notAllChosen = true, choiceNotSelected = true, goBack = false, waitTimeNotSelected = true, distance1NotSelected = true, distance2NotSelected = true;

    public void runOpMode() throws InterruptedException
    {




        /**
         *
         * Order of Auto Menu
         *
         * 1. Set wait time
         * 2. Set first distance
         * 3. Set second distance
         *
         */

        //Configure motors, servos and sensors
        Configure();

        while (notAllChosen) {

            goBack = false;
            if (stepInMenu == 1) {
                setWaitTime();
            }
            delayShort();


            goBack = false;
            if (stepInMenu == 2) {
                setDistance1ToTravel();
            }
            delayShort();


            goBack = false;
            if (stepInMenu == 3) {
                setDistance2ToTravel();
            }


            if (!waitTimeNotSelected && !distance1NotSelected && !distance2NotSelected) {
                notAllChosen = false;
            } else {
                notAllChosen = true;
            }

        }

        telemetry.addLine("HAVE A FANTASTIC DAY");
        telemetry.update();
        delayShort();


        //Wait until play button is pressed
        waitForStart();

        //calibrateGyro
        calibrateGyro();

        shooter.setPower(shooterPower);

        hood.setPosition(.6);

        driveBack(distance1, power, heading);

        shootAndLift(2700, .95);

        driveBack(distance2, power, heading);


    }

    public void setWaitTime()
    {
        stepInMenu = 1;
        choiceNotSelected = true;
        waitTimeNotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 1)   {

            if (waitTimeNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set wait time","%.1f", wait);
                telemetry.addLine("use dpad to adjust wait");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    wait += .0001;
                }
                if (gamepad1.dpad_down) {
                    wait -= .0001;
                }
                if (gamepad1.start) {
                    waitTimeNotSelected = false;
                }
            }

            if (!waitTimeNotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your wait time","%.1f", wait);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y){
                    choiceNotSelected = false;
                    waitTimeNotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    waitTimeNotSelected = true;
                    delayShort();
                }
            }
            telemetry.update();
        }

        stepInMenu = 2;
        choiceNotSelected = true;


    }

    public void setDistance1ToTravel ()
    {
        stepInMenu = 2;
        choiceNotSelected = true;
        distance1NotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 2)   {

            if (distance1NotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set Target Distance 1", "%.1f", distance1);
                telemetry.addLine("use dpad to adjust distance 1");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    distance1 += .0001;
                }
                if (gamepad1.dpad_down) {
                    distance1 -= .0001;
                }
                if (gamepad1.start) {
                    distance1NotSelected = false;
                }
            }

            if (!distance1NotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your target distance 1", "%.1f", distance1);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y) {
                    choiceNotSelected = false;
                    distance1NotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    distance1NotSelected = true;
                    delayShort();
                }

                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                    delayShort();
                    goBack = true;
                }

            }
                telemetry.update();
        }

        if (goBack){
            stepInMenu = 1;
            distance1NotSelected = true;
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

    public void setDistance2ToTravel ()
    {
        stepInMenu = 3;
        choiceNotSelected = true;
        distance2NotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 3)   {

            if (distance2NotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set Target Distance 2", "%.1f", distance2);
                telemetry.addLine("use dpad to adjust distance 2");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    distance2 += .0001;
                }
                if (gamepad1.dpad_down) {
                    distance2 -= .0001;
                }
                if (gamepad1.start) {
                    distance2NotSelected = false;
                }
            }

            if (!distance2NotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your target distance 2", "%.1f", distance2);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y) {
                    choiceNotSelected = false;
                    distance2NotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    distance2NotSelected = true;
                    delayShort();
                }

                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                    delayShort();
                    goBack = true;
                }

            }
            telemetry.update();
        }

        if (goBack){
            stepInMenu = 2;
            distance1NotSelected = true;
            delayShort();
        }
        else {
            goBack = false;
            stepInMenu = 1;
            delayShort();
        }
        choiceNotSelected = true;
        goBack = false;

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



