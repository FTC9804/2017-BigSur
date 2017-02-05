//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "V2RED", group = "AutoWithFunctions")

//@Disabled
public class V2RedAuto extends FuctionsForILTNew {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        //Set shooter to variable shooterPower
        shooter.setPower(shooterPower);

        //Drive back for 20 inches at .4 power
        driveBack(20, .4);

        //Shoot and Lift function: 2800 rpm, .95 intake
        shootAndLift(2800, .95);

        //Turn ccw .35 rotations at .25 power
        encoderTurnCounterClockwise(.35, .25);

        //Drive at -.5 power until right ods sees adequate white light
        driveToWhiteLineRight(-.5);

        //Drive back 7 inches at .8 power
        driveBack(7, .8);

        //Turn cw .22 rotations at .25 power
        encoderTurnClockwise(.22, .25);

        //Drive back, applying higher power to the right side until the left ods sees adequate white light
        driveToWhiteLineRightLeftSideFaster(-1);

        //Run press beacon side red method at -.15 power
        pressBeaconSideRed(-.15);

        //Turn cw .2 rotations at .25 power
        encoderTurnClockwise(.2, .25);

        //Run driveMoreLeft method for 24 inches at .8 power
        driveMoreLeft(24, .8);

        //Run press beacon side red method at .175 power
        pressBeaconSideRed(.175);
    }
}
