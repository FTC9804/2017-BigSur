package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Created by MarcusLeher on 28/01/2017.
 */

public class ShooterApparatus {

    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system

    Servo ballControl;

    Servo hood;       //position servo for 180º, adjust angle of shooter
    Servo turret;     //Continuous Rotation


    double shooterPower = 0.32; //initial power applied to the shooter

    double intakeSpeed;

    double turretAxis;          //value taken in by left turreting joystick.

    double turretRotationValue;       //value given to turret joystick
    final double TURRET_ROTATION_LEFT = 0.2;      //speed value of joystick moving to the left
    final double TURRET_ROTATION_RIGHT = 0.8;  //speed value of joystick moving to the right


    //rpm variables
    //shooter variables
    //encoder count variables
    double encoderClicksOne = 0;
    double encoderClicksTwo = 0;
    //rates per minute
    int rpm;
    //double array to compound the 5 most recent rpm values
    double[] averageRpmArray = {0, 0, 0, 0, 0};
    //int value so we know when we must calculate a new average rpm.  When this value is 5 we do so.
    int arrayCount = 0;
    //the value of the sum of the last 5 rpm values
    int totalRpm = 0;
    //the value of totalRpm divided by 5: the average of the last 5 rpm values
    int avgRpm = 0;
    //the value of the average of the last 5 rpm values, weighted to give preference to more recent values
    int weightedAvg = 0;
    //the weight given to the oldest value, incremented by .05 for each more recent value
    double baseWeight;
    //a boolean set to false unless outside force is currently being applied to the motor disturbing its rpm.  Reflective of how the motor will be disturbed when a ball is being shot in our actual robot.
    double tempWeightedAvg = 0;

    //time variables set to current run time throughout the code
    double timeOne;
    double timeTwo;

    //Gain to control rpm on the robot
    double rpmGain = .0000001;

    final double TURRET_INITIAL = 0.5;          //position of turret not moving

    final double HOOD_INITIAL = .12;          //initial hood position all the way at the bottom
    double hoodPositioning;  //hood positioning initially set

    //(not being used) The necesarry correction to power applied to motors when driving forward, based on the proximity between current and desired gyro headings
    double speedCorrection;

    //Placeholder Variable to display in telemetry
    int telemetryVariable = 0;

    HardwareMap hwMap  = null;

    Telemetry telemetry = null;

    Gamepad gamepad1;

    Gamepad gamepad2;

    ElapsedTime clock = new ElapsedTime();

    public ShooterApparatus() {
    }

    public void init(HardwareMap hwMapa, Telemetry telemetrya, ElapsedTime clocka, Gamepad gamepad1a, Gamepad gamepad2a) {
        hwMap = hwMapa;
        telemetry= telemetrya;
        clock = clocka;
        gamepad1 = gamepad1a;
        gamepad2 = gamepad2a;

        turret = hwMap.servo.get("s1");
        hood = hwMap.servo.get("s2");

        shooter = hwMap.dcMotor.get("m5");

        intake = hwMap.dcMotor.get("m6");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotor.Direction.FORWARD);

        clock.reset();

        //Servo initial positions
        turret.setPosition(TURRET_INITIAL);
        hood.setPosition(HOOD_INITIAL);

        //BALL CONTROL DECLARATION

    }

    public void shootAndLift (double time, double targetRPM, double elevatorSpeed, double intakeSpeed) throws InterruptedException
    {

        shooter.setPower(shooterPower);

        timeOne = clock.seconds();
        timeTwo = clock.seconds();

        while (timeTwo-timeOne<time) {

            //Current Run Time
            timeTwo = clock.seconds();
            //Current Encoder Clicks
            encoderClicksTwo = shooter.getCurrentPosition();


            //telemetry for shooting speed
            if (timeTwo - timeOne >= 0.1) {//if timeTwo and timeOne are more than .1 sec apart
                timeTwo = clock.seconds();//set time Two to curret runtime
                encoderClicksTwo = shooter.getCurrentPosition();//set encoderClicksTwo to the current position of the shooter motor
                rpm = (int) ((encoderClicksTwo - encoderClicksOne) / (timeTwo - timeOne) * (60 / 28)); //(clicks/seconds)(60seconds/1min)(1rev/28clicks)
                averageRpmArray[arrayCount] = rpm; //Set position arrayCount of averageRpmArray to current rpm
                timeOne = clock.seconds(); //set timeOne to current run time
                encoderClicksOne = shooter.getCurrentPosition(); //set encoderClicksOne to the current position of the shooter motor
                arrayCount++;//increment arrayCount by 1
            }


            if (arrayCount == 5) //if arrayCount equals 5
            {
                for (int i = 0; i < 5; i++) { //loop 5 times
                    totalRpm += averageRpmArray[i]; //increment totalRpm by the value at position i of averageRpmArray
                }
                avgRpm = (int) totalRpm / 5; //set avgRpm to totalRpm divided by five casted as an int
                baseWeight = .1; //Set base weight to .1
                for (int i = 0; i < 5; i++) { //Loop 5 times
                    weightedAvg += (int) averageRpmArray[i] * baseWeight; //Increment weightedAvg by the value of averageRpmArray at position i times baseWeight casted as an int
                    baseWeight += .05; //Increment base weight by .05
                }
                tempWeightedAvg = weightedAvg;

                if (weightedAvg + 50 > targetRPM) {
                    intake.setPower(intakeSpeed);
                } else {
                    intake.setPower(0);
                }


                //rpmGain now equal to .0000001 which is what we use in teleop.  may need to be adjusted
                shooterPower+= rpmGain * (targetRPM - weightedAvg);



                //MOVED THESE DOWN SO THAT THEY DONT INTERRUPT THE CODE WITH PROPORTIONAL CONTROL
                //Telemetry
                weightedAvg = 0; //set weightedAvg to 0
                arrayCount = 0; //set arrayCount to 0
                //set each value in averageRpmArray to 0
                averageRpmArray[0] = 0;
                averageRpmArray[1] = 0;
                averageRpmArray[2] = 0;
                averageRpmArray[3] = 0;
                averageRpmArray[4] = 0;
                //set totalRpm to 0;
                totalRpm = 0;

                shooter.setPower(shooterPower);
            }

            //telemetry for rpm and averages
            telemetry.addData("WeightedRPM: ", tempWeightedAvg);
            telemetry.addData("RPM : ", rpm);
            telemetry.addData("AvgRPM : ", avgRpm);
            telemetry.update();

        }
        shooter.setPower(0);
        intake.setPower(0);
        //driveNext();
    }

    public void getTimeOne ()
    {
        timeOne = clock.seconds();
    }

    public void teleOpShoot ()
    {
        //Current Run Time
        timeTwo = clock.seconds();
        //Current Encoder Clicks
        encoderClicksTwo = shooter.getCurrentPosition();

        //telemetry for shooting speed
        if (timeTwo - timeOne >= 0.1) {//if timeTwo and timeOne are more than .1 sec apart
            timeTwo = clock.seconds();//set time Two to curret runtime
            encoderClicksTwo = shooter.getCurrentPosition();//set encoderClicksTwo to the current position of the shooter motor
            rpm = (int) ((encoderClicksTwo - encoderClicksOne) / (timeTwo - timeOne) * (60 / 28)); //(clicks/seconds)(60seconds/1min)(1rev/28clicks)
            averageRpmArray[arrayCount] = rpm; //Set position arrayCount of averageRpmArray to current rpm
            timeOne = clock.seconds(); //set timeOne to current run time
            encoderClicksOne = shooter.getCurrentPosition(); //set encoderClicksOne to the current position of the shooter motor
            arrayCount++;//increment arrayCount by 1
        }


        if (arrayCount == 5) //if arrayCount equals 5
        {
            for (int i = 0; i < 5; i++) { //loop 5 times
                totalRpm += averageRpmArray[i]; //increment totalRpm by the value at position i of averageRpmArray
            }
            avgRpm = (int) totalRpm / 5; //set avgRpm to totalRpm divided by five casted as an int
            baseWeight = .1; //Set base weight to .1
            for (int i = 0; i < 5; i++) { //Loop 5 times
                weightedAvg += (int) averageRpmArray[i] * baseWeight; //Increment weightedAvg by the value of averageRpmArray at position i times baseWeight casted as an int
                baseWeight += .05; //Increment base weight by .05
            }
            tempWeightedAvg = weightedAvg;


            //Telemetry
            weightedAvg = 0; //set weightedAvg to 0
            arrayCount = 0; //set arrayCount to 0
            //set each value in averageRpmArray to 0
            averageRpmArray[0] = 0;
            averageRpmArray[1] = 0;
            averageRpmArray[2] = 0;
            averageRpmArray[3] = 0;
            averageRpmArray[4] = 0;
            //set totalRpm to 0;
            totalRpm = 0;


        }
        //telemetry for rpm and averages
        telemetry.addData("WeightedRPM: ", tempWeightedAvg);
        telemetry.addData("RPM : ", rpm);
        telemetry.addData("AvgRPM : ", avgRpm);

        if (gamepad2.right_trigger > 0.6)  //triggers act like an axis, so to make them behave like buttons
        {
            intakeSpeed = .95;
        } else {
            intakeSpeed = 0;
        }

//        if (gamepad1.right_bumper)
//        {
//            //BALL CONTROL UP
//        }
//        else
//        {
//            //BALL CONTROL DOWN
//        }


        //increment shooter motor power based on dpad commands
        shooterPower += rpmGain * (2400 - avgRpm);


        if (shooterPower > 1) {
            shooterPower = 1;
        }

        if (shooterPower < 0) {
            shooterPower = 0;
        }


        shooter.setPower(shooterPower);

        intake.setPower(intakeSpeed);
        telemetry.addData("shooter speed: ", shooterPower);

    }

    public void turretControl ()
    {
        //set turretAxis to the raw value of gamepad2.left_stick_x
        turretAxis = gamepad2.left_stick_x;

        if (turretAxis < -0.1) {  //0.1 to set off the dead zone. FUTURE -> set gain for precise adjustments
            turretRotationValue = TURRET_ROTATION_RIGHT;  //declared above for easy editing
        } else if (turretAxis > 0.1) {
            turretRotationValue = TURRET_ROTATION_LEFT;
        } else {
            turretRotationValue = 0.5;
        }

        turret.setPosition(turretRotationValue);
    }

    public void hoodControl ()
    {
        if (gamepad2.y) { //If y is being pressed, move the hood down
            hoodPositioning += .001;
        }
        if (gamepad2.a) { //If a is being pressed, move the hood up

            hoodPositioning -= .001;
        }
        //If the hoodPositioning is out of the possible servo range limits (both logically and physically), correct the values to
        //ensure the hoodPositioning value is in the correct range
        if (hoodPositioning > .35) {
            hoodPositioning = .35;
        }

        if (hoodPositioning < .09) {
            hoodPositioning = .09;
        }

        //Set the position of the hood to hoodPositioning
        hood.setPosition(hoodPositioning);
        telemetry.addData("hood" , hoodPositioning);
    }
}































