
package org.firstinspires.ftc.teamcode;

/**
 *
 * Created by Programmers of FTC Team 9804 Bomb Squad
 *
 * Created on Fri. Jan 6, '17 by Steve Cox
 *      Preliminary code to act as an auto chooser menu before auto begins
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public abstract class AutoChooserMenu extends LinearOpMode {


    //OVERALL VARIABLES
    double timeOne;
    double timeTwo;
    int stepInMenu = 1;
    int telemetryVariable = 0;
    boolean notAllChosen = true;
    boolean choiceNotSelected = true;
    boolean goBack = false;


    //checkGamepads() variables
    boolean gamepad1IsNotOK = true;
    boolean gamepad2IsNotOK = true;

    //setAutoAlliance() variables
    boolean weAreRed;
    boolean allianceNotSelected = true;

    //setStartPosition() variables
    int startPosition;
    boolean startPositionNotSelected = true;

    //setTimeDelay() variables
    int timeDelay;
    boolean timeDelayNotSelected = true;

    /**
     *
     * Order of Auto Menu
     *
     * 00 checkGamepads()
     * 1. setAutoAlliance()
     * 2. setStartPosition()
     * 3. setTimeDelay()
     *
     */

// F U N C T I O N S   F O R   A U T O   &   T E L E O P


    public void checkAutoMenu()
    {

        //checkGamepads();

        telemetry.addData("USE GAMEPAD 1 FOR SELECTING OPTIONS", telemetryVariable);
        telemetry.update();
        delayLong();

        while (notAllChosen)
        {

            goBack = false;

            if (stepInMenu == 1){
                setAutoAlliance();
            }

            delayShort();



            goBack = false;
            if (stepInMenu == 2) {
                setStartPosition();
            }

            delayShort();


            goBack = false;
            if (stepInMenu == 3) {
                setTimeDelay();
            }


            if ( !allianceNotSelected && !startPositionNotSelected && !timeDelayNotSelected ) {
                notAllChosen = false;
            }
            else {
                notAllChosen = true;
            }

        }

        telemetry.addData("HAVE A FANTASTIC DAY", telemetryVariable);
        telemetry.update();
        delayLong();

    }

    public void checkGamepads ()
    {

        stepInMenu = 0;
        //Check Gamepads
        while ((gamepad1IsNotOK || !gamepad2IsNotOK) && stepInMenu == 0) {

            if (gamepad1IsNotOK) {
                telemetry.addData("Start-A Gamepad1", telemetryVariable);
            } else {
                telemetry.addData("Gamepad1 is ok", telemetryVariable);
                gamepad1IsNotOK = false;
            }

            if (gamepad2IsNotOK) {
                telemetry.addData("Start-B Gamepad2", telemetryVariable);
            } else {
                telemetry.addData("Gamepad2 is ok", telemetryVariable);
                gamepad2IsNotOK = false;
            }
            telemetry.update();

        }

        stepInMenu = 1;

    }

    public void setAutoAlliance ()
    {

        stepInMenu = 1;
        choiceNotSelected = true;
        allianceNotSelected = true;
        telemetry.clearAll();

        while (choiceNotSelected && stepInMenu == 1)   {

            if (allianceNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addData("Gamepad 1 ONLY, click BACK to go back a step", telemetryVariable);
                telemetry.addData("Choose Alliance Color, x= blue, b=red", telemetryVariable);
                telemetry.update();
                if (gamepad1.x) {
                    weAreRed = false;
                    allianceNotSelected = false;
                    delayShort();
                }
                if (gamepad1.b) {
                    weAreRed = true;
                    allianceNotSelected = false;
                    delayShort();
                }
            }

            if (!allianceNotSelected) {

                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addData("Gamepad 1 ONLY, click BACK to go back a step", telemetryVariable);
                telemetry.addData("Confirm your color choice", telemetryVariable);

                if (weAreRed) {
                    telemetry.addData("We are RED", telemetryVariable);
                }
                else {
                    telemetry.addData("We are BLUE", telemetryVariable);
                }

                telemetry.addData("Y is correct.  A is incorrect", telemetryVariable);

                telemetry.update();

                if (gamepad1.y){
                    choiceNotSelected = false;
                    allianceNotSelected = false;
                    delayShort();
                }
                if (gamepad1.a) {
                    allianceNotSelected = true;
                    delayShort();
                }
            }
        }

        stepInMenu = 2;
        choiceNotSelected = true;

    }

    public void setStartPosition ()
    {

        stepInMenu = 2;
        telemetry.clearAll();
        choiceNotSelected = true;
        startPositionNotSelected = true;

        while  (choiceNotSelected && stepInMenu == 2) {
            if (startPositionNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addData("Gamepad 1 ONLY, click BACK to go back a step", telemetryVariable);
                telemetry.addData("Choose start position", telemetryVariable);
                telemetry.addData("Y = ", 1);
                telemetry.addData("B = ", 2);
                telemetry.addData("A = ", 3);
                telemetry.addData("X = ", 4);
                telemetry.update();
                if (gamepad1.y) {
                    startPosition = 1;
                    startPositionNotSelected = false;
                    delayShort();
                }
                if (gamepad1.b) {
                    startPosition = 2;
                    startPositionNotSelected = false;
                    delayShort();


                }
                if (gamepad1.a) {
                    startPosition = 3;
                    startPositionNotSelected = false;
                    delayShort();

                }
                if (gamepad1.x) {
                    startPosition = 4;
                    startPositionNotSelected = false;
                    delayShort();
                }
                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                    delayShort();
                    goBack = true;
                }
                else {
                    goBack = false;
                }
            }

            if (!startPositionNotSelected) {
                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                    delayShort();
                    goBack = true;
                }
                else {
                    goBack = false;
                }
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addData("Gamepad 1 ONLY, click BACK to go back a step", telemetryVariable);
                telemetry.addData("Confirm your start position choice = ", startPosition);

                telemetry.addData("Y is correct.  A is incorrect", telemetryVariable);

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
        if (goBack){
            stepInMenu = 1;
            startPositionNotSelected = true;
            delayShort();
        }
        else {
            goBack = false;
            stepInMenu = 3;
            delayShort();
        }
        choiceNotSelected = true;


    }

    public void setTimeDelay ()
    {

        stepInMenu = 3;
        telemetry.clearAll();
        choiceNotSelected = true;

        while  (choiceNotSelected && stepInMenu == 3) {
            if (timeDelayNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addData("Gamepad 1 ONLY, click BACK to go back a step", telemetryVariable);
                telemetry.addData("Choose Time Delay", telemetryVariable);
                telemetry.addData("Y = ", 0);
                telemetry.addData("B = ", 5);
                telemetry.addData("A = ", 10);
                telemetry.addData("X = ", 15);
                telemetry.update();
                if (gamepad1.y) {
                    timeDelay = 0;
                    timeDelayNotSelected = false;
                 delayShort();

                }
                if (gamepad1.b) {
                    timeDelay = 5;
                    timeDelayNotSelected = false;
                  delayShort();

                }
                if (gamepad1.a) {
                    timeDelay = 10;
                    timeDelayNotSelected = false;
                 delayShort();

                }
                if (gamepad1.x) {
                    timeDelay = 15;
                    timeDelayNotSelected = false;
                    delayShort();
                }
                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                    delayShort();
                    goBack = true;
                }

            }

            if (!timeDelayNotSelected) {
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addData("Gamepad 1 ONLY, click BACK to go back a step", telemetryVariable);
                telemetry.addData("Confirm your time delay choice", timeDelay);

                telemetry.addData("Y is correct.  A is incorrect", telemetryVariable);

                telemetry.update();
                if (gamepad1.y){
                    choiceNotSelected = false;
                    timeDelayNotSelected = false;
                   delayShort();

                }
                if (gamepad1.a) {
                    timeDelayNotSelected = true;
                    delayShort();

                }
                if (gamepad1.back) {
                    choiceNotSelected = false;
                    delayShort();
                    goBack = true;
                }
            }


            telemetry.update();

        }

        if (goBack){
            stepInMenu = 2;
            timeDelayNotSelected = true;
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

        while (timeTwo - timeOne < 1.5) {
            timeTwo = this.getRuntime();
        }
    }

}
