//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by stevecox on 2/24/17.
 * Purpose of the code is to allow the ability to tune right beacon pusher servo
 */

@Autonomous(name = "right beacon", group = "ServoTuning")
//@Disabled
public class TuneRightBeacon extends FunctionsForLA {

    //OVERALL VARIABLES
    double timeOne, timeTwo, beaconStartPosition = 1, beaconEndPosition = 0;
    int stepInMenu = 1;
    boolean notAllChosen = true, choiceNotSelected = true, goBack = false, startPositionNotSelected = true, endPositionNotSelected = true;

    public void runOpMode() throws InterruptedException
    {

        /**
         *
         * Order of Auto Menu
         *
         * 1. Set start position
         * 2. Set end position
         *
         */

        //Configure motors, servos and sensors
        Configure();

        while (notAllChosen) {

            goBack = false;
            if (stepInMenu == 1) {
                setStartPosition();
            }
            delayShort();


            goBack = false;
            if (stepInMenu == 2) {
                setEndPosition();
            }
            delayShort();


            if (!startPositionNotSelected && !endPositionNotSelected) {
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

        beaconStartPosition = Range.clip(beaconStartPosition, 0, 1);
        beaconEndPosition = Range.clip(beaconEndPosition, 0, 1);
        telemetry.clearAll();
        telemetry.addData("Gamepad1 x for in position", beaconStartPosition);
        telemetry.addData("gamepad1 b for out position", beaconEndPosition);
        telemetry.update();

        while (this.opModeIsActive()) {
            if (gamepad1.x) {
                beaconPusherRight.setPosition(beaconStartPosition);
            }
            else if (gamepad1.b) {
                beaconPusherRight.setPosition(beaconEndPosition);
            }
        }
    }

    public void setStartPosition ()
    {
        stepInMenu = 1;
        choiceNotSelected = true;
        startPositionNotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 1)   {

            if (startPositionNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set start position", beaconStartPosition);
                telemetry.addLine("use dpad to adjust position");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    beaconStartPosition += .00001;
                }
                if (gamepad1.dpad_down) {
                    beaconStartPosition -= .00001;
                }
                if (gamepad1.start) {
                    startPositionNotSelected = false;
                }
            }

            if (!startPositionNotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your start position", beaconStartPosition);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y){
                    choiceNotSelected = false;
                    startPositionNotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    startPositionNotSelected = true;
                    delayShort();
                }
            }
            telemetry.update();
        }

        stepInMenu = 2;
        choiceNotSelected = true;

    }

    public void setEndPosition ()
    {
        stepInMenu = 2;
        choiceNotSelected = true;
        endPositionNotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 2)   {

            if (endPositionNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY, START to move on");
                telemetry.addData("Set end position", beaconEndPosition);
                telemetry.addLine("use dpad to adjust position");
                telemetry.update();
                if (gamepad1.dpad_up) {
                    beaconEndPosition += .00001;
                }
                if (gamepad1.dpad_down) {
                    beaconEndPosition -= .00001;
                }
                if (gamepad1.start) {
                    endPositionNotSelected = false;
                }
            }

            if (!endPositionNotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm your end position", beaconEndPosition);
                telemetry.addLine("Y is correct.  A is incorrect");
                telemetry.update();

                if (gamepad1.y){
                    choiceNotSelected = false;
                    endPositionNotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    endPositionNotSelected = true;
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
            endPositionNotSelected = true;
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
