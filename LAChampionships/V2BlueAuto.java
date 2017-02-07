//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "V2BLUE", group = "AutoWithFunctions")

//@Disabled
public class V2BlueAuto extends FuctionsForILTNew {

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

        //Turn cw .35 rotations at .25 power
        encoderTurnClockwise(.78, .25);

        //Drive at -.8 power until left ods sees adequate white light
        driveToWhiteLineLeft(-.8);

        //Drive back 15 inches at .8 power
        driveBack(15, .8);

        //Turn ccw .15 rotations at .25 power
        encoderTurnCounterClockwise(.15, .25);

        //Drive back, applying higher power to the left side until the right ods sees adequate white light
        driveToWhiteLineLeftRightSideFaster(-1);

        //Run press beacon side blue method at -.15 power
        pressBeaconSideBlue(-.15);

        //Turn ccw .2 rotations at .25 power
        encoderTurnCounterClockwise(.2, .25);

        //Run driveMoreRight method for 24 inches at .8 power
        driveMoreRight(24, .8);

        //Run press beacon side blue method at .175 power
        pressBeaconSideBlue(.175);
    }

}
