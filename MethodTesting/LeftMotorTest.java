//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "lmt", group = "AutoWithFunctions")

//@Disabled
public class LeftMotorTest extends FuctionsForILTNew {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        //calibrateGyro
        calibrateGyro();

        leftMotor1.setPower(.2);
        leftMotor2.setPower(.2);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while (timeTwo-timeOne < 10) {
            timeTwo = this.getRuntime();
        }

        leftMotor1.setPower(-.2);
        leftMotor2.setPower(-.2);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while (timeTwo-timeOne < 10) {
            timeTwo = this.getRuntime();
        }


    }

}