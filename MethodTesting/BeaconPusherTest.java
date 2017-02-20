//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "bpt", group = "AutoWithFunctions")

//@Disabled
public class BeaconPusherTest extends FuctionsForILTNew {



    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        //calibrateGyro
        calibrateGyro();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_RETRACT_POSITION);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_EXTEND_POSITION);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_RETRACT_POSITION);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_EXTEND_POSITION);
            timeTwo = this.getRuntime();
        }
    }

}