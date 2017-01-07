/*Code written by Steve, Etienne, & Marcus Fri. 2 Dec. 2016
* updated with new functions code

*   -Friday JAN 6. '17: Lunchtime/Afternoon --> code for driving backwards, shooting twice, continuing to drive backwards.
*
* FTC Team 9804 Bomb Squad
*
* Version 1: creates code for full movement, both alliances
*
*
* L I N E U P
*   aim for cap ball at the center vortex with front left of Big Sur on right edgeof 2nd tile
*   have alliance partner's particle within access for our intake system to sweep up
*
* M O V E M E N T
*   drive backwards 6 inches
*   run only intake to sweep in alliance partner’s particle
*   drive backwards 19 inches
*   shoot and lift at 2500 rpm
*   drive back 45 inches to hit cap ball
*
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
            
        }


        // E N D   C O D E
    }
}
