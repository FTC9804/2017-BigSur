//Version 2.0 coded Feb. 25, 2017 by Steve, Etienne and Marcus.
//Designed to execute the autonomous concept
//of pressing two beacons and shooting two balls
//When on the blue alliance

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "V2BLUE", group = "AutoWithFunctions")

//@Disabled
public class V2BlueAuto extends FunctionsForLA {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        //For 1 second, bring down leftSideWheels
        //and rightSideWheels to .95 position
        while (timeTwo-timeOne<1)
        {
            timeTwo=this.getRuntime();
            leftSideWheels.setPosition(.95);
            rightSideWheels.setPosition(.95);
        }

        //public void calibrateGyro()
        calibrateGyro();

        //public void drive (double distance, double speed, double targetHeading, boolean isStopAtEnd)
        drive (2, .6, 0, true);

        //public void spinMove (double desiredHeading)
        spinMove(-37);

        //public void spinMove (double desiredHeading)
        spinMove(-37);

        //public void drive (double distance, double speed, double targetHeading, boolean isStopAtEnd)
        drive(16, .8, -37, false);

        //public void driveToWhiteLineRight (double speed, double targetHeading, boolean isStopAtEnd)
        driveToWhiteLineRight(.3, -37, true);

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        //1 second wait
        while (timeTwo-timeOne<1)
        {
            timeTwo=this.getRuntime();
        }

        //spinMove (double desiredHeading)
        spinMove (-17);

        //public void driveNoGyro (double distance, double speed)
        driveNoGyro(20, .25);

        //public void driveToWhiteLine (double speed, double targetHeading)
        driveToWhiteLine(.3, -4);

        //public void drive (double distance, double speed, double targetHeading, boolean isStopAtEnd)
        drive (3, .2, -4, true);

        //public void pressBeaconSideBlue (double speed, double targetHeading)
        pressBeaconSideBlue (-.2, 4);

        //public void driveBack (double distance, double speed, double targetHeading)
        driveBack(32, .3, 4);

        //Set position of left and right Drawbridge to 1
        leftDrawbridge.setPosition(1);
        rightDrawbridge.setPosition(1);

        //Set shooter motor's power to variable shooterPower
        shooter.setPower(shooterPower);

        //Set position of hood to .6
        hood.setPosition(.6);

        //public void driveToWhiteLine (double speed, double targetHeading)
        driveToWhiteLine(-.3, 4);

        //public void driveBack (double distance, double speed, double targetHeading)
        driveBack(2, .2, 4);

        //public void pressBeaconSideBlue (double speed, double target heading)
        pressBeaconSideBlue (.2, -4);

        //public void shootAndLift (double targetRPM, double intakeSpeed)
        shootAndLift(2700, .95, 5);

    }

}
