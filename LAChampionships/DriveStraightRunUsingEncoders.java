//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RunUsingEncoders", group = "RunModesTest")

//@Disabled
public class DriveStraightRunUsingEncoders extends FunctionsForLA {

    public void runOpMode() throws InterruptedException {

        inches = 60;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor1.setPower(.4);
        leftMotor2.setPower(.4);
        rightMotor1.setPower(.4);
        rightMotor2.setPower(.4);

        while (!gamepad1.a && this.opModeIsActive()) {

            telemetry.addData("LeftMotor1 Power", leftMotor1.getPower());
            telemetry.addData("LeftMotor2 Power", leftMotor2.getPower());
            telemetry.addData("RightMotor1 Power", rightMotor1.getPower());
            telemetry.addData("RightMotor2 Power", rightMotor2.getPower());
            telemetry.update();

        }

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopDriving();

    }

}
