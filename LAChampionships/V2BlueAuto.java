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

        calibrateGyro();

        driveBack(20, .4, 0);

        spinMove(75);

        spinMove(90);

        driveBack(20, .4, 0);

        driveToTouch(.1);

        pivot (3, .2, false);

        drive (25, .3, 185);

        pressBeaconSideBlue(.15);

    }

}
