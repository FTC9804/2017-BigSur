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

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherLeft.setPosition(beaconPusherLeftExtendPosition);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherRight.setPosition(beaconPusherRightExtendPosition);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherRight.setPosition(beaconPusherRightRetractPosition);
            timeTwo = this.getRuntime();
        }
    }

}