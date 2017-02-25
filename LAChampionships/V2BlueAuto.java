//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Func;

@Autonomous(name = "V2BLUE", group = "AutoWithFunctions")

//@Disabled
public class V2BlueAuto extends FunctionsForLA {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne<1)
        {
            timeTwo=this.getRuntime();
            leftSideWheels.setPosition(.95);
            rightSideWheels.setPosition(.95);
        }

        //calibrateGyro
        calibrateGyro();

        //public void drive (double distance, double speed, double targetHeading)
        drive (14, .6, 0, true);

        //spinMove (double desiredHeading)
        spinMove(-37);

        //spinMove (double desiredHeading)
        spinMove(-37);

        //public void drive (double distance, double speed, double targetHeading)
        drive(16, .8, -37, false);

        //public void driveToWhiteLineRight(double speed), .4 speed until right ods sees white light
        driveToWhiteLineRight(.3, -37, true);


        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne<1)
        {
            timeTwo=this.getRuntime();
        }

        //spinMove (double desiredHeading) spin to 15 deg heading
        spinMove (-17);

        //public void driveNoGyro (double distance, double speed), hit the wall, drive without a gyro for 20 inches at .25 power
        driveNoGyro(20, .25);

        //public void driveToWhiteLine (double speed, double targetHeading) drive at .3 power and -3 deg
        driveToWhiteLine(.3, -4);

        //public void drive (double distance, double speed, double targetHeading)
        drive (3, .2, -4, true);

        //public void pressBeaconSideBlue (double speed) find and press blue beacon at -.1 power
        pressBeaconSideBlue (-.2, 4);

        //public void driveBack (double distance, double speed, double targetHeading), drive back 32 in. at .3 power and -4 deg
        driveBack(32, .3, 4);

        shooter.setPower(shooterPower);

        hood.setPosition(.6);

        //public void driveToWhiteLine (double speed, double targetHeading), drive at -.3 power and -3 heading
        driveToWhiteLine(-.3, 4);

        //public void driveBack (double distance, double speed, double targetHeading), drive back 2 in. at .2 power and -4 deg
        driveBack(2, .2, 4);

        //public void pressBeaconSideBlue (double speed) find and press red beacon at .2 power and 4 heading
        pressBeaconSideBlue (.2, -4);

        shootAndLift(2700, .95);

    }

}
