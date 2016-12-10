package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 *
 * Created by Programmers of FTC Team 9804 Bomb Squad
 *
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import android.graphics.Color;



public abstract class Functions extends LinearOpMode {
    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back
    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system
    DcMotor elevator;     //elevator loading system


    Servo hood;       //position servo for 180º, adjust angle of shooter
    Servo batterySideBeacon;
    Servo portSideBeacon;
    Servo turret;



    //encoder variables to adequately sense the lines
    final static double ENCODER_CPR = 1120;    //encoder counts per rotation (CPR)
    final static double GEAR_RATIO = 24 / 18;     //Gear ratio used in the test bot
    final static double WHEEL_DIAMETER = 4; //wheel diameter in inches


    double batterySideBeaconPositionInitial = 0.5;
    double batterySideBeaconPosition = 1;
    double portSideBeaconPositionInitial = 0.5;
    double portSideBeaconPosition = 1;


    //shooter variables;
    double shooterSpeed = 0.5;    //constant power applied to the shooter

    double turretSpeed=.5;


    double INCHES_TO_MOVE=0;
    double ROTATIONS=0;
    double COUNTS=0;

    ColorSensor colorSensor;

    OpticalDistanceSensor whiteLineSensor1;
    OpticalDistanceSensor whiteLineSensor2;

    double whiteThreshold = .8;
    boolean wls1light = false;
    boolean wls2light = false;

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

    double timeOne;

    double timeTwo;

    double timeRunningLoop;

    ModernRoboticsI2cGyro gyro;

    double currentHeading;

    double headingError;

    double initialHeading;

    double gyroGain=.005;

    double rpmGain = .00015;

    double speedCorrection;

    double turnSpeed;

    int telemetryVariable = 0;

    double [] movingWeightedAverage = {0, 0, 0, 0, 0};

    int loopCounter = 0;

    double movingAverage;

    double rpmError = 0;

    boolean throwingException=false;

// F U N C T I O N S   F O R   A U T O   &   T E L E O P


    public void drive (double distance, double speed, double targetHeading)
    {
        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = distance;
        ROTATIONS = distance/ (Math.PI*WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel



        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set the modes of the encoders to "STOP_AND_RESET_ENCODER" in order to give intial readings of 0
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //gives the target position for the motors to run to using the math from earlier in the code
        leftMotor1.setTargetPosition((int) COUNTS);
        leftMotor2.setTargetPosition((int) COUNTS);
        rightMotor1.setTargetPosition((int) COUNTS);
        rightMotor2.setTargetPosition((int) COUNTS);


        //set the motor mode to the "RUN_TO_POSITION" mode in order to allow the motor to continue moving until the desired encoder value is reached
        leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftMotor1.isBusy() && leftMotor2.isBusy() && rightMotor1.isBusy() && rightMotor2.isBusy()) {

            currentHeading = gyro.getIntegratedZValue();
            headingError = targetHeading - currentHeading;
            speedCorrection = headingError * gyroGain;
            //power of motors as .5
            leftMotor1.setPower(speed-speedCorrection);
            leftMotor2.setPower(speed-speedCorrection);
            rightMotor1.setPower(speed+speedCorrection);
            rightMotor2.setPower(speed+speedCorrection);

        }
        stopDriving();
        shooter.setPower(.5);


    }

    public void calibrateGyro () throws InterruptedException
    {
        gyro.calibrate();
        while (gyro.isCalibrating())
        {
            sleep(100);
        }
    }

    public void stopDriving ()
    {
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    public void stopShooting ()
    {
        intake.setPower(0);
        shooter.setPower(0);
        elevator.setPower(0);
    }

    public void spinMove (double desiredHeading)
    {
        initialHeading = gyro.getIntegratedZValue();
        if (desiredHeading>0)
        {
            do{
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * gyroGain;
                if (turnSpeed < 0.2) {
                    turnSpeed = 0.2;
                }
                if (turnSpeed > .95) {
                    turnSpeed = .95;
                }

                rightMotor1.setPower(turnSpeed);
                rightMotor2.setPower(turnSpeed);
                leftMotor1.setPower(-turnSpeed);
                leftMotor2.setPower(-turnSpeed);


            }
            while ((currentHeading-initialHeading)<desiredHeading);
        }
        else
        {
            do {
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * gyroGain;
                if (turnSpeed > -0.2) {
                    turnSpeed = -0.2;
                }
                if (turnSpeed < -.95) {
                    turnSpeed = -.95;
                }

                rightMotor1.setPower(turnSpeed);
                rightMotor2.setPower(turnSpeed);
                leftMotor1.setPower(-turnSpeed);
                leftMotor2.setPower(-turnSpeed);
            }
        while ((Math.abs(currentHeading-initialHeading))<-desiredHeading);
        }
        stopDriving();
    }

    public void Configure ()
    {
        rightMotor1 = hardwareMap.dcMotor.get("m1");//port 1 on robot and in the hardwaremap
        rightMotor2 = hardwareMap.dcMotor.get("m2");
        leftMotor1 = hardwareMap.dcMotor.get("m3");
        leftMotor2 = hardwareMap.dcMotor.get("m4");
        leftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter = hardwareMap.dcMotor.get("m5");

        intake = hardwareMap.dcMotor.get("m6");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator = hardwareMap.dcMotor.get("m7");

        turret = hardwareMap.servo.get("s1");

        batterySideBeacon = hardwareMap.servo.get("s3");
        portSideBeacon = hardwareMap.servo.get("s4");

        batterySideBeacon.setPosition(batterySideBeaconPositionInitial);
        portSideBeacon.setPosition(portSideBeaconPositionInitial);
        turret.setPosition(turretSpeed);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.get("gyro");

        whiteLineSensor1= hardwareMap.opticalDistanceSensor.get("ods1");
        whiteLineSensor2= hardwareMap.opticalDistanceSensor.get("ods2");
        whiteLineSensor1.enableLed(true);
        whiteLineSensor2.enableLed(true);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(false);



    }


    public void shootAndLift (double time, double targetRPM, double elevatorSpeed, double intakeSpeed) throws InterruptedException
    {

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        timeRunningLoop = this.getRuntime();

        while (this.opModeIsActive() && (this.getRuntime() - timeRunningLoop < time)) {
            loopCounter++;
            timeTwo = this.getRuntime();
            encoderClicksTwo = shooter.getCurrentPosition();
            telemetry.addData("Time Two", timeTwo);
            telemetry.addData("Time Difference", timeTwo - timeOne);
            if (timeTwo - timeOne >= 0.05) {//if timeTwo and timeOne are more than .1 sec apart

                timeTwo = this.getRuntime();//set time Two to curret runtime

                encoderClicksTwo = shooter.getCurrentPosition();//set encoderClicksTwo to the current position of the shooter motor

                rpm = (int) ((encoderClicksTwo - encoderClicksOne) / (timeTwo - timeOne) * (60 / 28)); //(clicks/seconds)(60seconds/1min)(1rev/28clicks)

                timeOne = this.getRuntime(); //set timeOne to current run time
                encoderClicksOne = shooter.getCurrentPosition(); //set encoderClicksOne to the current position of the shooter motor


                for (int i = 0; i < 5; i++) {
                    movingWeightedAverage[i] = movingWeightedAverage[i + 1];
                }
                movingWeightedAverage[5] = rpm;
            }

            if (loopCounter > 5) {
                baseWeight = .1; //Set base weight to .1
                for (int i = 0; i < 5; i++) { //Loop 5 times
                    totalRpm += (int) movingWeightedAverage[i] * baseWeight; //Increment weightedAvg by the value of averageRpmArray at position i times baseWeight casted as an int
                    baseWeight += .05; //Increment base weight by .05
                }
                movingAverage = totalRpm / 5.0;
                rpmError = targetRPM - movingAverage;
                shooterSpeed += rpmError * rpmGain;

                shooter.setPower(shooterSpeed);
                if (avgRpm > targetRPM - 200) {
                    elevator.setPower(elevatorSpeed);
                    intake.setPower(intakeSpeed);
                } else {
                    elevator.setPower(0);
                    intake.setPower(0);
                }
                telemetry.addData ("Moving weighted rpm avg:", movingAverage);
                telemetry.update();
            }
        }

        stopShooting();


    }

    public void driveToWhiteLine ()
    {
        wls1light = false;
        wls2light = false;
        stopDriving();
        leftMotor1.setPower(.15);
        rightMotor1.setPower(.15);
        leftMotor2.setPower(.15);
        rightMotor2.setPower(.15);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the motor(s) at .15 while op mode is active and not enough white light has been detected on a motor's ods
        do {

            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensor1.getLightDetected() >= whiteThreshold) {
                wls1light = true;
            }
            if (whiteLineSensor2.getLightDetected() >= whiteThreshold) {
                wls2light = true;
            }

            //If enough white light has not been detected, keep the power of the motor at .15; else set it to 0
            if (wls1light == false) {
                rightMotor1.setPower(.15);
                rightMotor2.setPower(.15);
            } else {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }


            //If enough white light has not been detected, keep the power of the motor at .15; else set it to 0
            if (wls2light == false) {
                leftMotor1.setPower(.15);
                leftMotor2.setPower(.15);
            } else {
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
            }

        }
        while ( (wls1light == false
                || wls2light == false)
                && this.opModeIsActive()
                && (timeTwo-timeOne < 10) );  //Repeat do loop until both odss have detected enough white light

    }

    public void findAndPressBeacon ()
    {

        while (colorSensor.blue()<.3)
        {
            leftMotor1.setPower(.4);
            leftMotor2.setPower(.4);
            rightMotor1.setPower(.4);
            rightMotor2.setPower(.4);
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 2)
        {
            timeTwo = this.getRuntime();

            batterySideBeacon.setPosition(batterySideBeaconPosition);
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 2)
        {
            timeTwo = this.getRuntime();

            batterySideBeacon.setPosition(0);
        }
        batterySideBeacon.setPosition(.5);
    }
}
