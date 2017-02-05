

//Version 1.0 coded Jan. 15, 2017 by Rylan and Marcus.  Designed to test the new autonomous concept
//of driving forward, shooting, turning to 45, finding the white line, turning to 90
//and then driving forward and pressing the beacon.  In version 1.0 we solely tested driving
//forward, turning to 45, finding the white line, and turning to 90, which was accomplished.










//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "V2BLUEAUTO1.01", group = "AutoWithFunctions")
//@Disabled
public class V2BlueAuto extends FunctionsForILT {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        shooter.setPower(shooterPower);

        //Calibrate the Gyro
        //calibrateGyro();

        driveBack(20, .4);


        shootAndLift(10, 2800, .95, .95);

        encoderTurnClockwise(.78, .25);

        driveToWhiteLineLeft(-.8);

        driveBack(15, .8);

        encoderTurnCounterClockwise(.15, .25);

        driveToWhiteLineLeftRightSideFaster(-1);

        pressBeaconSideBlue(-.15);

        encoderTurnCounterClockwise(.15, .25);

        driveMoreRight(24, .8);

        pressBeaconSideBlue(.175);

        encoderTurnClockwise(.78, .25);

        drive(20, .8);


//        spinMove(-25);
//
//
//        driveMoreRightBack(40, .25, 0);
//
//
//        lineUpFasterRightBack();
//
//
//        pressBeaconFrontBlueNew(true);
//
//
//        driveMoreRight (29, .5, 0);
//
//
//        lineUpFasterRight();
//
//
//        pressBeaconFrontBlueNew(false);
//
//        spinMove(-18);
//
//        drive(33, .6);
    }

}
