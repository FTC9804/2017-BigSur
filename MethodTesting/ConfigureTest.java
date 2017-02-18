//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "config", group = "AutoWithFunctions")

//@Disabled
public class ConfigureTest extends FuctionsForILTNew {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();
    }

}