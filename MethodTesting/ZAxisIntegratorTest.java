//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "Z Axis Test", group = "AutoWithFunctions")

//@Disabled
public class ZAxisIntegratorTest extends FuctionsForILTNew {

    public void runOpMode() throws InterruptedException {


        //Configure motors, servos and sensors
        Configure();

        //Calibrate Gyro
        calibrateGyro();

        //Wait until play button is pressed
        waitForStart();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while (timeTwo-timeOne < 120) {
            timeTwo = this.getRuntime();
            currentHeading = gyro.getIntegratedZValue();
            telemetry.addData("Current Heading", currentHeading);
            gyro.resetZAxisIntegrator();
        }

        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

}