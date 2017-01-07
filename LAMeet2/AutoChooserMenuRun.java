/*Code written by Steve
*
*   -Friday JAN 6. '17: night
*
* FTC Team 9804 Bomb Squad
*
* Version 1: creates code to test auto chooser
*
*
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoChooser", group = "AutoWithFunctions")
//@Disabled
public class AutoChooserMenuRun extends AutoChooserMenu {

    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (this.opModeIsActive()) {

            checkAutoMenu();

            telemetry.clearAll();
            telemetry.addData("WeAreRed", weAreRed);
            telemetry.addData("Start Position = ", startPosition);
            telemetry.addData("Time Delay = ", timeDelay);
            telemetry.update();

            notAllChosen = true;
            choiceNotSelected = true;
            allianceNotSelected = true;
            startPositionNotSelected = true;
            timeDelayNotSelected = true;

            delayLong();
            delayLong();
            delayLong();
            delayLong();

        }


        // E N D   C O D E
    }
}
