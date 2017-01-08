package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Simple Auto 1.04", group = "AutoWithFunctions")
//@Disabled
public class SimpleAuto extends FUNCTIONSDEC20 {

    public void runOpMode() throws InterruptedException {
        Configure();

        waitForStart();

        calibrateGyro();

        drive(22, .4, 0);

        shootAndLift(20, 2300, .95, .95);

        drive(10, .4, 0);

        timeOne = this.getRuntime();

        timeTwo= this.getRuntime();

        while (timeTwo-timeOne<2)
        {
            timeTwo=this.getRuntime();
        }

        drive(8, .4, 0);




    }

    }