
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 *
 * Created by Programmers of FTC Team 9804 Bomb Squad
 *
 * edited on Fri. Dec. 16, '16
 *      Adjusting code for tournament on sunday
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
import com.qualcomm.robotcore.util.Range;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.I2cAddr;



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
    final static double GEAR_RATIO = 0.75;     //Gear ratio used in Big Sur in 24/18, so in code we multiply by 18/24
    final static double WHEEL_DIAMETER = 4; //wheel diameter in inches


    double batterySideBeaconPositionInitial = 0.5;
    double batterySideBeaconPositionExtend = 0;
    double batterySideBeaconPositionRetract = 1;
    double portSideBeaconPositionInitial = 0.5;
    double portSideBeaconPositionExtend = 1;
    double portSideBeaconPositionRetract = 0;


    //shooter variables;
    double shooterPower = 0.32;    //constant power applied to the shooter

    double turretSpeed=.5;


    double INCHES_TO_MOVE=0;
    double ROTATIONS=0;
    double COUNTS=0;

    ColorSensor pcolor;
    ColorSensor bcolor;

    OpticalDistanceSensor whiteLineSensorRight;
    OpticalDistanceSensor whiteLineSensorLeft;

    double whiteThreshold = .2;
    boolean wlsRightlight = false;
    boolean wlsLeftlight = false;

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

    double rpmGain = .000002;

    double speedCorrection;

    double turnSpeed;

    int telemetryVariable = 0;

    double [] movingWeightedAverage = {0, 0, 0, 0, 0};

    int loopCounter = 0;

    double movingAverage;

    double rpmError = 0;

    boolean throwingException=false;

    int mode = 0;

    double blueValue;
    double whiteValueRight;
    double whiteValueLeft;

    boolean choiceNotSelected = true;
    boolean allianceNotSelected = true;
    boolean weAreRed;


// F U N C T I O N S   F O R   A U T O   &   T E L E O P


    public void checkAutoAlliance ()
    {
        while (choiceNotSelected)   {
            if (allianceNotSelected){
                telemetry.addData("Choose Alliance Color", telemetryVariable);
                telemetry.update();
                if (gamepad1.x) {
                    weAreRed = false;
                    allianceNotSelected = false;
                }
                if (gamepad1.b) {
                    weAreRed = true;
                    allianceNotSelected = false;
                }

            }
            if (!allianceNotSelected) {
                telemetry.addData("Confirm your color choice, are we red?", weAreRed);
                telemetry.addData("Y is correct.  A is incorrect", telemetryVariable);
                if (gamepad1.y){
                    choiceNotSelected = false;
                }
                if (gamepad1.a) {
                    allianceNotSelected = true;
                }
            }
        }
    }

    public void beaconTurnOne ()
    {
        if (weAreRed)
        {
            spinMove(-90);
        }
        else
        {
            spinMove (90);
        }
    }

    public int driveTwo ()
    {
        if (weAreRed)
        {
            return -90;
        }
        else
        {
            return 90;
        }
    }

    public void workshopWhiteLineBlueBeaconTesting ()
    {
        while (this.opModeIsActive()) {

            telemetry.addData("Blue Value, pcolor = ", pcolor.blue());
            telemetry.addData("Blue Value, bcolor = ", bcolor.blue());

            telemetry.addData("WhiteValueRight RAW = ",whiteLineSensorRight.getRawLightDetected());
            telemetry.addData("WhiteValueLeft RAW = ", whiteLineSensorLeft.getRawLightDetected());

            telemetry.addData("WhiteValueRight STD = ",whiteLineSensorRight.getLightDetected());
            telemetry.addData("WhiteValueLeft STD = ", whiteLineSensorLeft.getLightDetected());

            telemetry.addData("WhiteThreshold", whiteThreshold);

            telemetry.update();
        }
    }

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

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initialHeading = gyro.getIntegratedZValue();
        if (desiredHeading>initialHeading)
        {
            do{
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * gyroGain;

                if (turnSpeed < 0.3) {
                    turnSpeed = 0.3;
                }
                if (turnSpeed > .95) {
                    turnSpeed = .95;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();

                rightMotor1.setPower(-turnSpeed);
                rightMotor2.setPower(-turnSpeed);
                leftMotor1.setPower(turnSpeed);
                leftMotor2.setPower(turnSpeed);


            }
            while ((currentHeading-initialHeading)<desiredHeading);
        }
        else
        {
            do {

                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * gyroGain;
                if (turnSpeed > -0.3) {
                    turnSpeed = -0.3;
                }
                if (turnSpeed < -.95) {
                    turnSpeed = -.95;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();

                rightMotor1.setPower(turnSpeed);
                rightMotor2.setPower(turnSpeed);
                leftMotor1.setPower(-turnSpeed);
                leftMotor2.setPower(-turnSpeed);
            }
        while (Math.abs(currentHeading-initialHeading)<=-Math.abs(headingError));
        }
        stopDriving();
    }

    public void Configure ()
    {
        rightMotor1 = hardwareMap.dcMotor.get("m1");//port 1 on robot and in the hardwaremap
        rightMotor2 = hardwareMap.dcMotor.get("m2");
        leftMotor1 = hardwareMap.dcMotor.get("m3");
        leftMotor2 = hardwareMap.dcMotor.get("m4");
        leftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.dcMotor.get("m5");

        intake = hardwareMap.dcMotor.get("m6");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator = hardwareMap.dcMotor.get("m7");

        turret = hardwareMap.servo.get("s1");

        batterySideBeacon = hardwareMap.servo.get("s3");
        portSideBeacon = hardwareMap.servo.get("s4");

        batterySideBeacon.setPosition(batterySideBeaconPositionInitial);
        portSideBeacon.setPosition(portSideBeaconPositionInitial);
        turret.setPosition(turretSpeed);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.get("gyro"); //I2C port 0

        whiteLineSensorRight= hardwareMap.opticalDistanceSensor.get("ods2");    //Analog import port 0
        whiteLineSensorLeft= hardwareMap.opticalDistanceSensor.get("ods1");     //Analog import port 4
        whiteLineSensorRight.enableLed(true);
        whiteLineSensorLeft.enableLed(true);

        pcolor = hardwareMap.colorSensor.get("pcolor");     //I2C port 1
        pcolor.setI2cAddress(I2cAddr.create7bit(0x1e));     //7 bit address needed b/c multiple color sensors
        bcolor = hardwareMap.colorSensor.get("bcolor");     //I2C port 2
        bcolor.setI2cAddress(I2cAddr.create7bit(0x26));     //CONVERSION FROM 8 BIT TO HEXADECIMAL
        pcolor.enableLed(false);
        bcolor.enableLed(false);



    }

    public void shootAndLiftTestingOptions ( double targetRPM, double elevatorSpeed, double intakeSpeed) throws InterruptedException
    {


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        timeRunningLoop = this.getRuntime();

        telemetry.addData("Input Gain (A,B,X,Y)", telemetryVariable);
        telemetry.update();
        while (!gamepad1.a&&!gamepad1.b&&!gamepad1.x&&!gamepad1.y) {


            //distance in INCHES from the center of the vortex basket (red or blue).
            if (gamepad1.a) {
                rpmGain = .000005;
            }
            if (gamepad1.b) {
                rpmGain = .000001;
            }
            if (gamepad1.x) {
                rpmGain = .0000015;
            }
            if (gamepad1.y) {
                rpmGain = .000002;
            }
        }

        telemetry.clearAll();
        telemetry.addData("Input RPM", telemetryVariable);
        telemetry.update();

        while (!gamepad1.dpad_up&&!gamepad1.dpad_down&&!gamepad1.dpad_left&&!gamepad1.dpad_right) {


            //distance in INCHES from the center of the vortex basket (red or blue).
            if (gamepad1.dpad_up) {
                targetRPM=2600;
            }
            if (gamepad1.dpad_right) {
                targetRPM=2800; //close-mid shot
            }
            if (gamepad1.dpad_down) {
                targetRPM=3000; //far-mid shot
            }
            if (gamepad1.dpad_left) {
                targetRPM=3200; //far shot
            }
        }

        telemetry.clearAll();
        telemetry.addData("Gain = ", rpmGain);
        telemetry.addData("TargetRPM = ", targetRPM);
        telemetry.update();


        while (this.opModeIsActive()) {


            loopCounter++;
            timeTwo = this.getRuntime();
            encoderClicksTwo = shooter.getCurrentPosition();
            telemetry.addData("Time Two", timeTwo);
            telemetry.addData("Time Difference", timeTwo - timeOne);
            if (timeTwo - timeOne >= 0.1) {//if timeTwo and timeOne are more than .1 sec apart

                timeTwo = this.getRuntime();//set time Two to curret runtime

                encoderClicksTwo = shooter.getCurrentPosition();//set encoderClicksTwo to the current position of the shooter motor

                rpm = (int) ((encoderClicksTwo - encoderClicksOne) / (timeTwo - timeOne) * (60 / 28)); //(clicks/seconds)(60seconds/1min)(1rev/28clicks)

                timeOne = this.getRuntime(); //set timeOne to current run time
                encoderClicksOne = shooter.getCurrentPosition(); //set encoderClicksOne to the current position of the shooter motor


                for (int i = 0; i < 4; i++) {
                    movingWeightedAverage[i] = movingWeightedAverage[i + 1];
                }
                movingWeightedAverage[4] = rpm;
                if (loopCounter > 5) {
                    baseWeight = .1; //Set base weight to .1
                    for (int i = 0; i < 5; i++) { //Loop 5 times
                        totalRpm += (int) movingWeightedAverage[i] * baseWeight; //Increment weightedAvg by the value of averageRpmArray at position i times baseWeight casted as an int
                        baseWeight += .05; //Increment base weight by .05
                    }
                    movingAverage = totalRpm;
                    rpmError = targetRPM - movingAverage;
                    shooterPower += rpmError * rpmGain;
                    Range.clip(shooterPower, .2, .5);
                    shooter.setPower(shooterPower);
                    if (movingAverage > targetRPM - 200) {
                        elevator.setPower(elevatorSpeed);
                        intake.setPower(intakeSpeed);
                    } else {
                        elevator.setPower(0);
                        intake.setPower(0);
                    }
                    telemetry.addData("Moving weighted rpm avg:", movingAverage);
                    telemetry.addData("Power", shooterPower);
                    telemetry.update();
                }
            }
        }

        stopShooting();
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
            if (timeTwo - timeOne >= 0.1) {//if timeTwo and timeOne are more than .1 sec apart

                timeTwo = this.getRuntime();//set time Two to curret runtime

                encoderClicksTwo = shooter.getCurrentPosition();//set encoderClicksTwo to the current position of the shooter motor

                rpm = (int) ((encoderClicksTwo - encoderClicksOne) / (timeTwo - timeOne) * (60 / 28)); //(clicks/seconds)(60seconds/1min)(1rev/28clicks)

                timeOne = this.getRuntime(); //set timeOne to current run time
                encoderClicksOne = shooter.getCurrentPosition(); //set encoderClicksOne to the current position of the shooter motor


                for (int i = 0; i < 4; i++) {
                    movingWeightedAverage[i] = movingWeightedAverage[i + 1];
                }
                movingWeightedAverage[4] = rpm;
                if (loopCounter > 5) {
                    baseWeight = .1; //Set base weight to .1
                    for (int i = 0; i < 5; i++) { //Loop 5 times
                        totalRpm += (int) movingWeightedAverage[i] * baseWeight; //Increment weightedAvg by the value of averageRpmArray at position i times baseWeight casted as an int
                        baseWeight += .05; //Increment base weight by .05
                    }
                    movingAverage = totalRpm;
                    rpmError = targetRPM - movingAverage;
                    shooterPower += rpmError * rpmGain;
                    Range.clip(shooterPower, .2, .5);
                    shooter.setPower(shooterPower);
                    if (movingAverage > targetRPM - 200) {
                        elevator.setPower(elevatorSpeed);
                        intake.setPower(intakeSpeed);
                    } else {
                        elevator.setPower(0);
                        intake.setPower(0);
                    }
                    telemetry.addData("Moving weighted rpm avg:", movingAverage);
                    telemetry.addData("Power", shooterPower);
                    telemetry.update();
                }
            }
        }

        stopShooting();
    }

    public void startShooting (double power)
    {
        shooter.setPower(power);
        sleep(2000);
    }

    public void driveToWhiteLine ()
    {


        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        wlsRightlight = false;
        wlsLeftlight = false;
        stopDriving();
        leftMotor1.setPower(.25);
        rightMotor1.setPower(.25);
        leftMotor2.setPower(.25);
        rightMotor2.setPower(.25);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {

            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorRight.getRawLightDetected() >= whiteThreshold) {
                wlsRightlight = true;
            }
            if (whiteLineSensorLeft.getRawLightDetected() >= whiteThreshold) {
                wlsLeftlight = true;
            }

            //If enough white light has not been detected, keep the power of the motor at .25; else set it to 0
            if (wlsRightlight == false) {
                rightMotor1.setPower(.25);
                rightMotor2.setPower(.25);
            } else {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }


            //If enough white light has not been detected, keep the power of the motor at .25; else set it to 0
            if (wlsLeftlight == false) {
                leftMotor1.setPower(.25);
                leftMotor2.setPower(.25);
            } else {
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
            }

        }
        while ( (wlsRightlight == false
                || wlsLeftlight == false)
                && this.opModeIsActive()
                && (timeTwo-timeOne < 30) );  //Repeat do loop until both odss have detected enough white light

              stopDriving();
            timeOne = this.getRuntime();
            timeTwo = this.getRuntime();
            while (timeTwo-timeOne < 2)
            {
                timeTwo = this.getRuntime();
            }


    }

    public void findAndPressBlueBeacon ()
    {



        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (pcolor.blue()<2.5)
        {
            telemetry.addData("pcolor Blue value:", pcolor.blue());
            telemetry.update();
            leftMotor1.setPower(.25);
            leftMotor2.setPower(.25);
            rightMotor1.setPower(.25);
            rightMotor2.setPower(.25);
        }

        stopDriving();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();

            portSideBeacon.setPosition(portSideBeaconPositionExtend);
        }


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();

            portSideBeacon.setPosition(.6);  //let servo push against beacon for a second, so slightly harder than rest position
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();

            portSideBeacon.setPosition(portSideBeaconPositionRetract);
        }
        portSideBeacon.setPosition(.5);

    }

    public void findAndPressRedBeacon ()
    {



        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (bcolor.red()<2.5)
        {
            telemetry.addData("bcolor RED value:", bcolor.red());
            telemetry.update();
            leftMotor1.setPower(.25);
            leftMotor2.setPower(.25);
            rightMotor1.setPower(.25);
            rightMotor2.setPower(.25);
        }

        stopDriving();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();

            batterySideBeacon.setPosition(batterySideBeaconPositionExtend);
        }


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();

            batterySideBeacon.setPosition(.4);  //let servo push against beacon for a second, so slightly harder than rest position
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();

            batterySideBeacon.setPosition(batterySideBeaconPositionRetract);
        }
        batterySideBeacon.setPosition(.5);

    }

    public void findAndPressBeacon ()
    {
        if (weAreRed) {
            findAndPressRedBeacon();
        }
        else {
            findAndPressBlueBeacon();
        }
    }

    public void shoot(double power)
    {
        timeRunningLoop = this.getRuntime();
        do{

            //set the shooter to power
            shooter.setPower(power);
            timeOne = this.getRuntime();
            timeTwo = this.getRuntime();

            //sleep function to allow the shooter to get up to speed
            while (timeTwo-timeOne < 2)
            {
                timeTwo = this.getRuntime();
            }

            intake.setPower(0.95); //cut off at 95% speed

            //sleep function to let the first ball pass through (1.5 seconds)
            timeOne = this.getRuntime();
            timeTwo = this.getRuntime();
            while (timeTwo-timeOne < 1.5)
            {
                timeTwo = this.getRuntime();
                elevator.setPower(0.95); //cut off at 95% speed
            }

            //stop shooting the balls
            elevator.setPower(0);

            //let the shooter get back up to speed
            timeOne = this.getRuntime();
            timeTwo = this.getRuntime();
            while (timeTwo-timeOne < 1)
            {
                timeTwo = this.getRuntime();
            }

            //shoot again
            timeOne = this.getRuntime();
            timeTwo = this.getRuntime();
            while (timeTwo-timeOne < 1.5)
            {
                timeTwo = this.getRuntime();
                elevator.setPower(0.95); //cut off at 95% speed
            }

            //stop the elevator again
            elevator.setPower(0);

        }while(
                this.opModeIsActive()
                && this.getRuntime() - timeRunningLoop < 7
                );

        stopShooting();

    }

    public void stopDrivingAndPause () {

        stopDriving();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1)
        {
            timeTwo = this.getRuntime();
        }
    }



}
