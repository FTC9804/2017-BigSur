//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

/**
 *
 * Created by stevecox on 2/25/17.
 *
 * Purpose of the code is to allow the ability to
 *  drive back over a specified distance,
 *  shoot, and keep driving
 *
 */

@Autonomous(name = "Auto Shoot Options", group = "AutoWithFunctions")
//@Disabled
public class AutoShootOptions extends FunctionsForLA {

    //OVERALL VARIABLES
    double timeOne, timeTwo, wait, distance1, distance2, power;
    int stepInMenu, heading;
    boolean notAllChosen, choiceNotSelected, goBack,
            waitTimeNotSelected, distance1NotSelected,
            distance2NotSelected , motorPowerNotSelected;

    public void runOpMode() throws InterruptedException
    {
        timeOne = 0;
        timeTwo =0;
        wait = 0;
        distance1 = 0;
        distance2 = 0;
        power = 0;
        stepInMenu = 1;
        heading = 0;
        notAllChosen = true;
        choiceNotSelected = true;
        goBack = false;
        waitTimeNotSelected = true;
        distance1NotSelected = true;
        distance2NotSelected = true;
        motorPowerNotSelected = true;


        /**
         *
         * Order of Auto Menu
         *
         * 1. Set wait time
         * 2. Set first distance
         * 3. Set second distance
         * 4. Set driving power
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
            delayShort();

            goBack = false;
            if (stepInMenu == 4) {
                setMotorPower();
            }

            if (!waitTimeNotSelected && !distance1NotSelected && !distance2NotSelected && !motorPowerNotSelected) {
                notAllChosen = false;
            } else {
                notAllChosen = true;
            }
        }

        telemetry.addLine("GET READY TO START");
        telemetry.addLine(" ");
        telemetry.addData("Delay time = ", "%.1f", wait);
        telemetry.addData("Distance 1 = ", "%.1f", distance1);
        telemetry.addData("Distance 2 = ", "%.1f", distance2);
        telemetry.addData("DrivePower = ", "%.2f", power);
        telemetry.update();

        //Wait until play button is pressed
        waitForStart();

        telemetry.addLine("Code starting");
        telemetry.update();

        //allows the wait time to include the gyro calibration to prevent any excess time wasted
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //calibrateGyro
        calibrateGyro();

        //the wait loop to prevent the code starting until the specified delay is finished
        while (timeTwo - timeOne < wait) {
            timeTwo = this.getRuntime();
        }

        //shooter given power to get the motor up to speed
        shooter.setPower(shooterPower);

        //hood set to scoring position desired for shooting
        hood.setPosition(.6);

        //drive back 1 using parameters specified by driver
        // driveBack (double distance, double speed, double targetHeading)
        driveBack(distance1, power, heading);

        //shoot using tested values of 2700 RPM
        // shootAndLift(double targetRPM, double intakeSpeed)
        shootAndLift(2700, .95, 5);

        //drive back 2 using parameters specified by driver
        // driveBack (double distance, double speed, double targetHeading)
        driveBack(distance2, power, heading);

        stopDriving();
        shooter.setPower(0);

    }

    //this is the first action called allowing the user
    // of Gamepad 1 to specify the time delay in seconds
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
                telemetry.addData("Set wait time in seconds","%.1f", wait);
                telemetry.addLine("This time includes gyro calibration period");
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
                wait = Range.clip(wait, 0, 30);
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

    //this is the second action called allowing the user
    // of Gamepad 1 to specify the first distance to travel in inches
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
                telemetry.addData("Set Target Distance 1 in inches", "%.1f", distance1);
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
                distance1 = Range.clip(distance1, 0, 100);
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

    //this is the third action called allowing the user
    // of Gamepad 1 to specify the second distance to travel in inches
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
                telemetry.addData("Set Target Distance 2 in inches", "%.1f", distance2);
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
                distance2 = Range.clip(distance2, 0, 100);
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
            stepInMenu = 4;
            delayShort();
        }
        choiceNotSelected = true;
        goBack = false;

    }

    //this is the fourth action called allowing the user
    // of Gamepad 1 to specify the driving motor power
    public void setMotorPower ()
    {
        stepInMenu = 4;
        choiceNotSelected = true;
        motorPowerNotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 4)   {

            if (motorPowerNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set Motor Power", "%.2f", power);
                telemetry.addLine("ONLY POSITIVE VALUES");
                telemetry.addLine("use dpad to adjust power");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    power += .00001;
                }
                if (gamepad1.dpad_down) {
                    power -= .00001;
                }
                if (gamepad1.start) {
                    motorPowerNotSelected = false;
                }
                power = Range.clip(power, 0, 1);
            }

            if (!motorPowerNotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your motor power", "%.2f", power);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y) {
                    choiceNotSelected = false;
                    motorPowerNotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    motorPowerNotSelected = true;
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
            stepInMenu = 3;
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


}


