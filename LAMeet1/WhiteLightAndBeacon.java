/*Code written by Steve, Etienne, & Marcus Fri. 2 Dec. 2016
* updated with new functions code

*   -Sunday, December 10, 2016: Night --> introducing new code with functions, excluding shooting command
*
* FTC Team 9804 Bomb Squad
*
* Version 1: creates code for full movement
*
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BigSurAutoFunctionsNoShooting", group = "AutoWithFunctions")
//@Disabled
public class WhiteLightAndBeacon extends Functions {

    public void runOpMode() throws InterruptedException {

        //motor configurations located in the hardware map

        //original configuration for motors, servos, and sensors
        Configure();

        //wait for the code to start to begin the autonomous program
        waitForStart();

        workshopWhiteLineBlueBeaconTesting();

        stopDriving();
        //END CODE
    }
}
