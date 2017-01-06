/*Code written by Steve, Etienne, & Marcus Fri. 2 Dec. 2016
* updated with new functions code

*   -Friday JAN 6. '17: Lunchtime/Afternoon --> code for driving backwards, shooting twice, continuing to drive backwards.
*
* FTC Team 9804 Bomb Squad
*
* Version 1: creates code for full movement, both alliances
*
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Backwards, 2 Shoot", group = "AutoWithFunctions")
//@Disabled
public class DriveBackwards2ShootBackwards extends FunctionsForMeet2 {

    public void runOpMode() throws InterruptedException {

        //motor configurations located in the hardware map

        Configure();

        waitForStart();

        calibrateGyro();


        driveBack(6 /*distance*/,
                -.5 /*speed*/,
                0 /*targetHeading*/);

        stopDrivingAndPause();


        runIntakeOnly(.95 /*intakeSpeed*/ ,
                        2 /*time*/ );

        stopDrivingAndPause();


        driveBack(19 /*distance*/ ,
                .5 /*speed*/,
                0/*targetHeading*/);
        stopDrivingAndPause();


        shootAndLift(20 /*time*/,
                    2500 /*targetRPM*/,
                    .95 /*elevatorSpeed*/,
                    .95 /*intakeSpeed*/);

        stopDrivingAndPause();


        driveBack(15 /*distance*/ ,
                -.5 /*speed*/ ,
                0 /*targetHeading*/ );

        stopDrivingAndPause();

        // E N D   C O D E
    }
}