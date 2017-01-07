
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

    int stepInMenu;
    int telemetryVariable = 0;
    boolean notAllChosen = true;
    boolean choiceNotSelected = true;

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

        checkGamepads();

        while (notAllChosen)
        {
            telemetry.addData("USE GAMEPAD 1 FOR SELECTING OPTIONS", telemetryVariable);
            telemetry.update();

            if (stepInMenu == 1){
                setAutoAlliance();
            }

            if (stepInMenu == 2) {
                setStartPosition();
            }

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
                    allianceNotSelected = false;
                }
                if (gamepad1.a) {
                    allianceNotSelected = true;
                }
            }
        }

        stepInMenu = 2;
    }

    public void setStartPosition ()
    {

        stepInMenu = 2;
        telemetry.clearAll();
        choiceNotSelected = true;
        telemetry.addData("Choose start position", telemetryVariable);
        telemetry.addData("Y = ", 1);
        telemetry.addData("B = ", 2);
        telemetry.addData("A = ", 3);
        telemetry.addData("X = ", 4);
        telemetry.update();

        while  (choiceNotSelected && stepInMenu == 2) {
            if (startPositionNotSelected){
                telemetry.addData("Choose Start Position Color", telemetryVariable);
                telemetry.update();
                if (gamepad1.y) {
                    startPosition = 1;
                    startPositionNotSelected = false;
                }
                if (gamepad1.b) {
                    startPosition = 2;
                    startPositionNotSelected = false;
                }
                if (gamepad1.a) {
                    startPosition = 3;
                    startPositionNotSelected = false;
                }
                if (gamepad1.x) {
                    startPosition = 4;
                    startPositionNotSelected = false;
                }
                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                }

            }

            if (!startPositionNotSelected) {
                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                }
                telemetry.addData("Confirm your start position choice", telemetryVariable);

                telemetry.addData("Start Position = ", startPosition);

                telemetry.addData("Y is correct.  A is incorrect", telemetryVariable);

                if (gamepad1.y){
                    choiceNotSelected = false;
                    startPositionNotSelected = false;
                }
                if (gamepad1.a) {
                    startPositionNotSelected = true;
                }
            }


            telemetry.update();

        }

        stepInMenu = 3;


    }

    public void setTimeDelay ()
    {

        stepInMenu = 3;
        telemetry.clearAll();
        choiceNotSelected = true;
        telemetry.addData("Set time delay", telemetryVariable);
        telemetry.addData("Y = ", 0);
        telemetry.addData("B = ", 5);
        telemetry.addData("A = ", 10);
        telemetry.addData("X = ", 15);
        telemetry.update();

        while  (choiceNotSelected && stepInMenu == 3) {
            if (timeDelayNotSelected){
                telemetry.addData("Choose Time Delay", telemetryVariable);
                telemetry.update();
                if (gamepad1.y) {
                    timeDelay = 0;
                    timeDelayNotSelected = false;
                }
                if (gamepad1.b) {
                    timeDelay = 5;
                    timeDelayNotSelected = false;
                }
                if (gamepad1.a) {
                    timeDelay = 10;
                    timeDelayNotSelected = false;
                }
                if (gamepad1.x) {
                    timeDelay = 15;
                    timeDelayNotSelected = false;
                }
                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                }

            }

            if (!timeDelayNotSelected) {
                telemetry.addData("Confirm your time delay choice", timeDelay);

                telemetry.addData("Time Delay = ", timeDelay);

                telemetry.addData("Y is correct.  A is incorrect", telemetryVariable);

                if (gamepad1.y){
                    choiceNotSelected = false;
                    timeDelayNotSelected = false;
                }
                if (gamepad1.a) {
                    timeDelayNotSelected = true;
                }
                if (gamepad1.back) {
                    stepInMenu = stepInMenu - 1;
                }
            }


            telemetry.update();

        }

    }
}
