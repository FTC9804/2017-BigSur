
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



public abstract class FUNCTIONSDEC20 extends LinearOpMode {
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

    Servo ballControl;



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
    double shooterPower = 0.62;    //constant power applied to the shooter

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

    final double GYRO_GAIN =.0041;

    double straightGyroGain = .0005;

    double rpmGain = .0000035;

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

    boolean exitcode1 = false;


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
                telemetry.addData("Confirm your color choice", telemetryVariable);

                if (weAreRed) {
                    telemetry.addData("We are RED", telemetryVariable);
                }
                else {
                    telemetry.addData("We are BLUE", telemetryVariable);
                }

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

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftMotor1.getCurrentPosition()<COUNTS) {

            currentHeading = gyro.getIntegratedZValue();
            telemetry.addData("Current Heading = ", currentHeading);
            telemetry.update();
            headingError = targetHeading - currentHeading;
            speedCorrection = headingError * straightGyroGain;
            //power of motors as .5. MAKES NO SENSE + - SHOULD BE REVERSED //fix corrections
            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();


        }
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

    public void driveNext()
    {
        INCHES_TO_MOVE = 15;
        ROTATIONS = 15/ (Math.PI*WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftMotor1.getCurrentPosition()<COUNTS) {
            leftMotor1.setPower(.5);
            leftMotor2.setPower(.5);
            rightMotor1.setPower(.5);
            rightMotor2.setPower(.5);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    public void gyroTelemetry ()
    {
        telemetry.addData("Heading", gyro.getIntegratedZValue());
        telemetry.addData("Turn Speed", turnSpeed);
        //telemetry.addData("Motor power", leftMotor1.getPower());
        telemetry.update();
    }

    public void pivot (double desiredHeading)
    {
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initialHeading = gyro.getIntegratedZValue();

        if (desiredHeading < initialHeading)
        {
            do{
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = -headingError * GYRO_GAIN;

                if (turnSpeed < 0.4) {
                    turnSpeed = 0.4;
                }
                if (turnSpeed > .95) {
                    turnSpeed = .95;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();

                //MAKES NO SENSE JUST TRYING
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
                leftMotor1.setPower(turnSpeed);
                leftMotor2.setPower(turnSpeed);
                gyroTelemetry();

            }
            while (currentHeading > desiredHeading); //for clockwise heading you are going to a more positive number
        }
        else
        {
            do {

                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * GYRO_GAIN;

                if (turnSpeed < 0.4) {
                    turnSpeed = 0.4;
                }
                if (turnSpeed > .95) {
                    turnSpeed = .95;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();


                //WHAT THE HECK IS HAPPENING? SHOULDNT THESE BE OPPOSITE
                rightMotor1.setPower(turnSpeed);
                rightMotor2.setPower(turnSpeed);
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
                gyroTelemetry();
            }
            while (currentHeading < desiredHeading); //for counter-clockwise heading you are going to a more negative number
        }
        stopDriving();
    }

    public void driveBack (double distance, double speed, double targetHeading)
    {

        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = -distance;
        ROTATIONS = -distance / (Math.PI * WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel


        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set the modes of the encoders to "STOP_AND_RESET_ENCODER" in order to give intial readings of 0
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //gives the target position for the motors to run to using the math from earlier in the code
        leftMotor1.setTargetPosition((int) COUNTS);

        //set the motor mode to the "RUN_TO_POSITION" mode in order to allow the motor to continue moving until the desired encoder value is reached
        leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftMotor1.isBusy()) {


            currentHeading = gyro.getIntegratedZValue();
            telemetry.addData("Current Heading = ", currentHeading);
            telemetry.update();
            headingError = targetHeading - currentHeading;
            speedCorrection = headingError * straightGyroGain;
            //power of motors as .5. MAKES NO SENSE + - SHOULD BE REVERSED
            leftMotor1.setPower(speed + speedCorrection);
            leftMotor2.setPower(speed + speedCorrection);
            rightMotor1.setPower(speed - speedCorrection);
            rightMotor2.setPower(speed - speedCorrection);

        }
        stopDriving();
    }

    public void calibrateGyro () throws InterruptedException
    {
        gyro.calibrate();

        while (gyro.isCalibrating())
        {
            sleep(100);
            telemetry.addData("Gyro is not calibrated", telemetryVariable);
            telemetry.update();
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

        initialHeading = gyro.getIntegratedZValue();

        if (desiredHeading < initialHeading)
        {
            do{
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = -headingError * GYRO_GAIN;

                if (turnSpeed < 0.35) {
                    turnSpeed = 0.35;
                }
                if (turnSpeed > .82) {
                    turnSpeed = .82;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();

                //MAKES NO SENSE JUST TRYING
                rightMotor1.setPower(-turnSpeed);
                rightMotor2.setPower(-turnSpeed);
                leftMotor1.setPower(turnSpeed);
                leftMotor2.setPower(turnSpeed);
                gyroTelemetry();

            }
            while (currentHeading > desiredHeading); //for clockwise heading you are going to a more positive number
        }
        else
        {
            do {

                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * GYRO_GAIN;

                if (turnSpeed < .35) {
                    turnSpeed = 0.35;
                }
                if (turnSpeed > .82) {
                    turnSpeed = .82;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();


                //WHAT THE HECK IS HAPPENING? SHOULDNT THESE BE OPPOSITE
                rightMotor1.setPower(turnSpeed);
                rightMotor2.setPower(turnSpeed);
                leftMotor1.setPower(-turnSpeed);
                leftMotor2.setPower(-turnSpeed);
                gyroTelemetry();
            }
            while (currentHeading < desiredHeading); //for counter-clockwise heading you are going to a more negative number
        }
        stopDriving();
    }

    public void Configure ()
    {
        rightMotor1 = hardwareMap.dcMotor.get("m3");//port 1 on robot and in the hardwaremap
        rightMotor2 = hardwareMap.dcMotor.get("m4");
        leftMotor1 = hardwareMap.dcMotor.get("m1");
        leftMotor2 = hardwareMap.dcMotor.get("m2");
        leftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter = hardwareMap.dcMotor.get("m5");

        intake = hardwareMap.dcMotor.get("m6");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotor.Direction.FORWARD);

        elevator = hardwareMap.dcMotor.get("m7");

        turret = hardwareMap.servo.get("s1");

        ballControl = hardwareMap.servo.get("s5");

        hood = hardwareMap.servo.get("s2");
        batterySideBeacon = hardwareMap.servo.get("s3");
        portSideBeacon = hardwareMap.servo.get("s4");

        batterySideBeacon.setPosition(batterySideBeaconPositionInitial);
        portSideBeacon.setPosition(portSideBeaconPositionInitial);
        turret.setPosition(turretSpeed);
        ballControl.setPosition(0);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); //I2C port 0
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

    public void shootAndLift (double time, double targetRPM, double elevatorSpeed, double intakeSpeed) throws InterruptedException
    {

        shooter.setPower(shooterPower);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        timeRunningLoop = this.getRuntime();

        while (this.getRuntime()<40) {

            //Current Run Time
            timeTwo = this.getRuntime();
            //Current Encoder Clicks
            encoderClicksTwo = shooter.getCurrentPosition();


            //telemetry for shooting speed
            if (timeTwo - timeOne >= 0.1) {//if timeTwo and timeOne are more than .1 sec apart
                timeTwo = this.getRuntime();//set time Two to curret runtime
                encoderClicksTwo = shooter.getCurrentPosition();//set encoderClicksTwo to the current position of the shooter motor
                rpm = (int) ((encoderClicksTwo - encoderClicksOne) / (timeTwo - timeOne) * (60 / 28)); //(clicks/seconds)(60seconds/1min)(1rev/28clicks)
                averageRpmArray[arrayCount] = rpm; //Set position arrayCount of averageRpmArray to current rpm
                timeOne = this.getRuntime(); //set timeOne to current run time
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

                if (weightedAvg + 300 > targetRPM) {
                    ballControl.setPosition(60);
                    intake.setPower(intakeSpeed);
                    elevator.setPower(elevatorSpeed);
                } else {
                    ballControl.setPosition(0);
                    intake.setPower(0);
                    elevator.setPower(0);
                }

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

                if (weightedAvg<targetRPM)
                {
                    shooterPower+=rpmGain * (Math.abs(targetRPM-weightedAvg));
                }
                else
                {
                    shooterPower-=rpmGain * (Math.abs(targetRPM-weightedAvg));
                }
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
        elevator.setPower(0);
        driveNext();
    }

    public void driveToWhiteLine ()
    {


        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(.3);
        rightMotor1.setPower(.3);
        leftMotor2.setPower(.3);
        rightMotor2.setPower(.3);

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
            if (whiteLineSensorLeft.getRawLightDetected() >= whiteThreshold) {
                wlsRightlight = true;
            }
            if (whiteLineSensorRight.getRawLightDetected() >= whiteThreshold) {
                wlsLeftlight = true;
            }

            //If enough white light has not been detected, keep the power of the motor at .25; else set it to 0
            if (wlsRightlight == false) {
                rightMotor1.setPower(.3);
                rightMotor2.setPower(.3);
            } else {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }


            //If enough white light has not been detected, keep the power of the motor at .25; else set it to 0
            if (wlsLeftlight == false) {
                leftMotor1.setPower(.3);
                leftMotor2.setPower(.3);
            } else {
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
            }

        }
        while ( (wlsRightlight == false
                || wlsLeftlight == false)
                && this.opModeIsActive()
                && (timeTwo-timeOne < 4) );  //Repeat do loop until both odss have detected enough white light

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);


    }

    public void findAndPressBlueBeacon (double time)
    {
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (bcolor.blue()<2.5 && ((timeTwo-timeOne) < time))
        {
            timeTwo= this.getRuntime();
            telemetry.addData("bcolor Blue value:", bcolor.blue());
            telemetry.update();
            leftMotor1.setPower(.25);
            leftMotor2.setPower(.25);
            rightMotor1.setPower(.25);
            rightMotor2.setPower(.25);
        }

        if (timeTwo-timeOne>=time)
        {
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
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

    public void findAndPressRedBeacon (double time)
    {

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (pcolor.red()<2.5)
        {
            telemetry.addData("pcolor RED value:", pcolor.red());
            telemetry.update();
            leftMotor1.setPower(.25);
            leftMotor2.setPower(.25);
            rightMotor1.setPower(.25);
            rightMotor2.setPower(.25);
        }

        if (timeTwo-timeOne>=time)
        {
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
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

    public void findAndPressBeacon ()
    {
        if (weAreRed) {
            findAndPressRedBeacon(5);
        }
        else {
            findAndPressBlueBeacon(5);
        }
    }

    public void shoot(double power)
    {

        timeRunningLoop = this.getRuntime();

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
        while (timeTwo-timeOne < 1)
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
        intake.setPower(0);

        stopShooting();

    }

    public void stopDrivingAndPause ()
    {
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while (timeTwo - timeOne < 3)
        {
            timeTwo = this.getRuntime();
            telemetry.addData("Time", timeTwo);
        }
    }

    public void driveForTimeNoEncoders (double time)
    {

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < time){
            timeTwo = this.getRuntime();

            leftMotor1.setPower(0.5);
            leftMotor2.setPower(0.5);
            rightMotor1.setPower(0.5);
            rightMotor2.setPower(0.5);
        }

    }

}
