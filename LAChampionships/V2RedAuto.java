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

        //calibrateGyro
        calibrateGyro();

        //public void drive (double distance, double speed, double targetHeading)
        drive (14, .6, 0, true);

        //spinMove (double desiredHeading)
        spinMove(35);

        //spinMove (double desiredHeading)
        spinMove(35);

        //public void drive (double distance, double speed, double targetHeading)
        drive(16, .8, 35, false);

        //public void driveToWhiteLineRight(double speed), .4 speed until right ods sees white light
        driveToWhiteLineLeft(.3, false);

        //public void drive (double distance, double speed, double targetHeading), drive 4 inches at .2 power at -35 deg heading
        drive (2, .2, 35, true);

        //spinMove (double desiredHeading) spin to 15 deg heading
        spinMove (14);

        //public void driveNoGyro (double distance, double speed), hit the wall, drive without a gyro for 20 inches at .25 power
        driveNoGyro(20, .25);

        //public void driveToWhiteLine (double speed, double targetHeading) drive at .3 power and -3 deg
        driveToWhiteLine(.3, 4);

        //public void drive (double distance, double speed, double targetHeading)
        drive (3, .2, 4, true);

        //public void pressBeaconSideBlue (double speed) find and press blue beacon at -.1 power
        pressBeaconSideRed(-.2, -4);

        //public void driveBack (double distance, double speed, double targetHeading), drive back 32 in. at .3 power and -4 deg
        driveBack(32, .3, -4);

        //public void driveToWhiteLine (double speed, double targetHeading), drive at -.3 power and -3 heading
        driveToWhiteLine(-.3, -4);

        //public void driveBack (double distance, double speed, double targetHeading), drive back 2 in. at .2 power and -4 deg
        driveBack(2, .2, -4);

        //public void pressBeaconSideBlue (double speed) find and press red beacon at .2 power and 4 heading
        pressBeaconSideRed (.2, 4);



    }

}
