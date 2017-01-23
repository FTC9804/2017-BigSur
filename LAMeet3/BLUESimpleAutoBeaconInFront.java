


//Version 1.0 coded Jan. 15, 2017 by Rylan and Marcus.  Designed to test the new autonomous concept
//of driving forward, shooting, turning to 45, finding the white line, turning to 90
//and then driving forward and pressing the beacon.  In version 1.0 we solely tested driving
//forward, turning to 45, finding the white line, and turning to 90, which was accomplished.










//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUESimpleAutoBeaconInFront1.91", group = "AutoWithFunctions")
//@Disabled
public class BLUESimpleAutoBeaconInFront extends FUNCTIONSDEC20 {

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

        encoderTurnClockwise(.53, .23);


        //Set timeOne and timeTwo to the current run time

        drive (17, .5, 0);



        //Run driveToOneWhiteLineRight function
        driveToOneWhiteLineRight();


        drive(2, .25, 0);
        //Move to 90 degrees clockwise.  Set parameter to 73 degrees clockwise to combat overshoot in the gyro
        //spinMove(-87);

        encoderTurnClockwise(.43, .28);


        spinMove(-83);



        //Run driveToTouch method, applying .25 power to the motors when doing so
        driveToTouch(0.15);

        //Run pressBeaconFrontBlue method
        pressBeaconFrontBlue();



        driveBack(29, .5, 0);


        encoderTurnCounterClockwise(.85, .13);


        drive (26, .5, 0);


        driveToOneWhiteLineRight();


        drive(3, .25, 0);



        encoderTurnClockwise(.6, .13);


        spinMoveSecond(-83);


        //Run driveToTouch method, applying .25 power to the motors when doing so
        driveToTouch(0.15);


        //Run pressBeaconFrontBlue method
        pressBeaconFrontBlue();

        beaconPusherLeft.setPosition(beaconPusherLeftExtendPosition);

        beaconPusherRight.setPosition(beaconPusherRightExtendPosition);



    }

}
