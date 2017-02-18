//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SPINTEST", group = "AutoWithFunctions")

//@Disabled
public class SpinTest extends FuctionsForILTNew {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        //calibrateGyro
        calibrateGyro();

        spinMove(-70);

        spinMove(-85);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while (timeTwo - timeOne < 1) {
            timeTwo = this.getRuntime();
            telemetry.addData("gyro", gyro.getIntegratedZValue());
            telemetry.update();
        }

        while (gyro.getIntegratedZValue() < -87 || gyro.getIntegratedZValue()> -83) {
            spinMove(-85);
        }



//        while (gyro.getIntegratedZValue() < -87 || gyro.getIntegratedZValue()> -83)
//        {
//            spinMove( -85);
//        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while (timeTwo - timeOne < 10) {
            timeTwo = this.getRuntime();
            telemetry.addData("gyro", gyro.getIntegratedZValue());
            telemetry.update();
        }

    }

}