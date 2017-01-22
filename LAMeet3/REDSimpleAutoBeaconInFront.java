

//Version 1.0 coded Jan. 15, 2017 by Rylan and Marcus.  Designed to test the new autonomous concept
//of driving forward, shooting, turning to 45, finding the white line, turning to 90
//and then driving forward and pressing the beacon.  In version 1.0 we solely tested driving
//forward, turning to 45, finding the white line, and turning to 90, which was accomplished.










//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "REDSimpleAutoBeaconInFront1.8", group = "AutoWithFunctions")
//@Disabled
public class REDSimpleAutoBeaconInFront extends FUNCTIONSDEC20 {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();


        //Calibrate the Gyro
        calibrateGyro();

        //Run shootAndLift function for 20 seconds at a goal of 2300 rpm, with intake and elevator speeds of .95
        //shootAndLift(20, 2300, .95, .95);

        drive(20, .2, 0);



        //Move 45 degrees clockwise.  Set parameter to 36 degrees clockwise to combat overshoot in the gyro
        //spinMove(-32);

        encoderTurnCounterClockwise(.54, .16);


        drive (17, .5, 0);



        //Run driveToOneWhiteLineRight function
        driveToOneWhiteLineLeft();


        //Move to 90 degrees clockwise.  Set parameter to 73 degrees clockwise to combat overshoot in the gyro
        //spinMove(-87);

        encoderTurnCounterClockwise(.53, .16);



        spinMove(86);



        //Run driveToTouch method, applying .25 power to the motors when doing so
        driveToTouch(0.19);



        //Run pressBeaconFrontBlue method
        pressBeaconFrontRed();



        driveBack(29, .5, 0);

        encoderTurnClockwise(.89, .13);


        drive (26, .5, 0);



        driveToOneWhiteLineLeft();


        drive(2.8, .25, 0);



        encoderTurnCounterClockwise(.56, .13);



        spinMoveSecond(86);



        //Run driveToTouch method, applying .25 power to the motors when doing so
        driveToTouch(0.15);


        //Run pressBeaconFrontBlue method
        pressBeaconFrontRed();

        beaconPusherLeft.setPosition(beaconPusherLeftExtendPosition);

        beaconPusherRight.setPosition(beaconPusherRightExtendPosition);

    }

}
