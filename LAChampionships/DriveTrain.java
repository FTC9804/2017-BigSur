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
 * Created by MarcusLeher on 27/01/2017.
 */

//NOT AN OP MODE

public class DriveTrain {
    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back

    TouchSensor touchSensor;

    Servo beaconPusherLeft;    //Servo on the left of the robot for beacon pushing
    Servo beaconPusherRight;   //Servo on the right of the robot for beacon pushing

    //Color sensor, located
    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;


    //Positions of the continuos rotation beacon pushers, based on wheter retracting or extending
    double beaconPusherLeftRetractPosition = 0.05;
    double beaconPusherLeftExtendPosition = .95;
    double beaconPusherRightRetractPosition = 0.95;
    double beaconPusherRightExtendPosition = 0.05;

    boolean push;



    double leftPower=0;
    double rightPower=0;

    double whitesCounter = 0;

    int loopCounter = 0;

    double speedCorrection;

    //Driving variables, initialized to 0
    double INCHES_TO_MOVE=0;  //Desired number of inches to drive
    double ROTATIONS=0;       //Wheel rotations necessary to drive the above amount of inches
    double COUNTS=0;          //Encoder counts necessary to drive the above amount of inches/rotations

    //encoder variables to adequately sense the lines
    final static double ENCODER_CPR = 1120;    //encoder counts per rotation (CPR)
    final static double GEAR_RATIO = 0.75;     //Gear ratio used in Big Sur in 24/18, so in code we multiply by 18/24
    final static double WHEEL_DIAMETER = 4; //wheel diameter in inches

    //Optical distance sensors to detect white light
    OpticalDistanceSensor whiteLineSensorRight;
    OpticalDistanceSensor whiteLineSensorLeft;

    double whiteThreshold = .4; //The threshold of white light necessary to set the below variables to true
    boolean wlsRightlight = false; //Boolean expressing whether the whiteLineSensorRight has seen a sufficient amount of white light
    boolean wlsLeftlight = false;  //Boolean expressing whether the whiteLineSensorLeft has seen a sufficient amount of white light

    int whitesCounterLeft;
    int whitesCounterRight;

    //time variables set to current run time throughout the code
    double timeOne = 0;
    double timeTwo = 0;

    //Gyro sensor declaration
    ModernRoboticsI2cGyro gyro;

    //measure gyro heading/position
    double currentHeading;

    //measure difference between current and desired gyro heading
    double headingError;

    //variable to measure the gyro heading at the beginning of a method
    double initialHeading;

    //Gain for the gyro when executing a spin move
    final double GYRO_GAIN =.0026;

    //Gain for the gyro when driving straight
    double straightGyroGain = .0005;

    //Speed applied to motors when executing a pivot or spinMove, based on both GYRO_GAIN and headingError
    double turnSpeed;

    //Placeholder Variable to display in telemetry
    int telemetryVariable = 0;

    HardwareMap hwMap  = null;

    Telemetry telemetry = null;

    Gamepad gamepad1;

    Gamepad gamepad2;

    ElapsedTime clock = new ElapsedTime();

    double joystick1ValueLeft;  //the raw value taken from the left joystick
    double joystick1ValueRight;  //the raw value taken from the right joystick

    double gain = 1;
    //drive gain to multiply the power by
    //toggle variables
    boolean halfGain = false;
//    boolean previousStatus = false;
//    boolean currentStatus = false;
    boolean justBumped = false;

    public DriveTrain() {
    }

    public void init(HardwareMap hwMapa, Telemetry telemetrya, ElapsedTime clocka, Gamepad gamepad1a, Gamepad gamepad2a) {
        hwMap = hwMapa;
        telemetry= telemetrya;
        clock = clocka;
        gamepad1 = gamepad1a;
        gamepad2 = gamepad2a;

        rightMotor1 = hwMap.dcMotor.get("m3");//port 1 on robot and in the hardwaremap
        rightMotor2 = hwMap.dcMotor.get("m4");
        leftMotor1 = hwMap.dcMotor.get("m1");
        leftMotor2 = hwMap.dcMotor.get("m2");
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);
        leftMotor1.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro"); //I2C port 0
        whiteLineSensorRight= hwMap.opticalDistanceSensor.get("ods2");    //Analog import port 0
        whiteLineSensorLeft= hwMap.opticalDistanceSensor.get("ods1");     //Analog import port 4
        whiteLineSensorRight.enableLed(true);
        whiteLineSensorLeft.enableLed(true);

        touchSensor = hwMap.touchSensor.get("touch");

        beaconPusherLeft = hwMap.servo.get("s3");
        beaconPusherRight = hwMap.servo.get("s4");

        colorSensorLeft = hwMap.colorSensor.get("color");     //I2C port 1
        colorSensorLeft.enableLed(false); //

        colorSensorRight = hwMap.colorSensor.get("color");     //I2C port 2
        colorSensorRight.enableLed(false); //

        clock.reset();

    }

    public void encoderTurnClockwise (double rotations, double speed)
    {
        //currentHeading = gyro.getIntegratedZValue();
        //headingError = Math.abs(desiredHeading - currentHeading);
        //encoderTurnArc = robotCircumference * (headingError / 360);
        COUNTS = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftMotor1.getCurrentPosition()<COUNTS) {
            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(-speed);
            rightMotor2.setPower(-speed);
            telemetry.addData("Current encoder position = ", leftMotor1.getCurrentPosition());
            telemetry.addData("Current rotations travelled = ", (leftMotor1.getCurrentPosition())/(ENCODER_CPR*GEAR_RATIO));
            telemetry.update();
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    public void encoderTurnCounterClockwise (double rotations, double speed)
    {
        //currentHeading = gyro.getIntegratedZValue();
        //headingError = Math.abs(desiredHeading - currentHeading);
        //encoderTurnArc = robotCircumference * (headingError / 360);
        COUNTS = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((Math.abs(rightMotor1.getCurrentPosition())<COUNTS)) {
            leftMotor1.setPower(-speed);
            leftMotor2.setPower(-speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);
            telemetry.addData("Current encoder position = ", rightMotor1.getCurrentPosition());
            telemetry.addData("Current rotations travelled = ", (rightMotor1.getCurrentPosition())/(ENCODER_CPR*GEAR_RATIO));
            telemetry.update();
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
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


    public void gyroTelemetry ()
    {
        telemetry.addData("Heading", gyro.getIntegratedZValue());
        telemetry.addData("Turn Speed", turnSpeed);
        //telemetry.addData("Motor power", leftMotor1.getPower());
        telemetry.update();
    }

    public void driveBack (double distance, double speed, double targetHeading)
    {
        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = distance;
        ROTATIONS = INCHES_TO_MOVE / (Math.PI * WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (Math.abs(leftMotor1.getCurrentPosition())<COUNTS) {

            currentHeading = gyro.getIntegratedZValue();
            telemetry.addData("Current Heading = ", currentHeading);
            telemetry.update();
            headingError = targetHeading - currentHeading;
            speedCorrection = headingError * straightGyroGain;

            leftMotor1.setPower(-speed);
            leftMotor2.setPower(-speed);
            rightMotor1.setPower(-speed);
            rightMotor2.setPower(-speed);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

    public void calibrateGyro () throws InterruptedException
    {
        gyro.calibrate();
        while (gyro.isCalibrating())
        {
            Thread.sleep(100);
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

                if (turnSpeed < 0.2) {
                    turnSpeed = 0.2;
                }
                if (turnSpeed > .82) {
                    turnSpeed = .82;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();

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

                if (turnSpeed < .2) {
                    turnSpeed = 0.2;
                }
                if (turnSpeed > .82) {
                    turnSpeed = .82;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();

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

    public void spinMoveSecond (double desiredHeading)
    {
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initialHeading = gyro.getIntegratedZValue();

        if (desiredHeading < initialHeading)
        {
            do{
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = -headingError * GYRO_GAIN;

                if (turnSpeed < 0.13) {
                    turnSpeed = 0.13;
                }
                if (turnSpeed > .82) {
                    turnSpeed = .82;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();

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

                if (turnSpeed < .13) {
                    turnSpeed = 0.13;
                }
                if (turnSpeed > .82) {
                    turnSpeed = .82;
                }

                telemetry.addData("Current Heading:",currentHeading);
                telemetry.addData("TurnSpeed: ",turnSpeed);
                telemetry.update();

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

    public void stopDrivingAndPause ()
    {

        stopDriving();
        timeOne = clock.seconds ();
        timeTwo = clock.seconds ();
        while (timeTwo - timeOne < 1.5)
        {
            timeTwo = clock.seconds ();
            telemetry.addData("Time", (timeTwo - timeOne));
        }
    }

    public void driveToTouch (double speed)
    {
        timeOne=clock.seconds ();
        timeTwo= clock.seconds ();

        while (!touchSensor.isPressed()&& (timeTwo-timeOne<4)){
            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);
            timeTwo=clock.seconds ();
        }

        stopDriving();
    }

    public void lineUpFasterLeft ()
    {
        whitesCounterLeft = 0;
        whitesCounterRight = 0;
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wlsRightlight=false;
        wlsLeftlight=false;

        timeOne = clock.seconds ();
        timeTwo = clock.seconds ();

        leftMotor1.setPower(.25);
        rightMotor1.setPower(.2);
        leftMotor2.setPower(.25);
        rightMotor2.setPower(.2);


        //Keep the motor(s) at .3 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getLightDetected()>=whiteThreshold) {
                whitesCounterLeft++;
                if (whitesCounterLeft>2) {
                    wlsLeftlight = true;
                }
            }
            if (whiteLineSensorRight.getLightDetected()>=whiteThreshold) {
                whitesCounterRight++;
                if (whitesCounterRight>2) {
                    wlsRightlight = true;
                }
            }
            // Display the light level while we are looking for the line
            //telemetry.addData("ODS 1 Light Level", whiteLineSensor1.getLightDetected());
            //telemetry.update();
            //idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            //telemetry.addData("ODS 2 Light Level", whiteLineSensor2.getLightDetected());
            //telemetry.update();

            //If enough white light has not been detected, keep the power of the motor at .3; else set it to 0
            if (wlsRightlight==false) {
                rightMotor1.setPower(.2);
                rightMotor2.setPower(.2);
            }
            else
            {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            //If enough white light has not been detected, keep the power of the motor at .3; else set it to 0
            if (wlsLeftlight==false) {
                leftMotor1.setPower(.25);
                leftMotor2.setPower(.25);
            }
            else
            {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            timeTwo=clock.seconds ();
        }
        while ((wlsLeftlight==false || wlsRightlight==false)&&((timeTwo-timeOne)<6));  //Repeat do loop until both odss have detected enough white light
        if (timeTwo-timeOne>6)
        {
            while (clock.seconds()<40)
            {
                timeTwo=clock.seconds ();
            }
        }

    }

    public void lineUpFasterRight ()
    {
        whitesCounterLeft = 0;
        whitesCounterRight = 0;
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wlsRightlight=false;
        wlsLeftlight=false;

        timeOne = clock.seconds ();
        timeTwo = clock.seconds ();

        leftMotor1.setPower(.2);
        rightMotor1.setPower(.25);
        leftMotor2.setPower(.2);
        rightMotor2.setPower(.25);


        //Keep the motor(s) at .3 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getLightDetected()>=whiteThreshold) {
                whitesCounterLeft++;
                if (whitesCounterLeft>2) {
                    wlsLeftlight = true;
                }
            }
            if (whiteLineSensorRight.getLightDetected()>=whiteThreshold) {
                whitesCounterRight++;
                if (whitesCounterRight>2) {
                    wlsRightlight = true;
                }
            }
            // Display the light level while we are looking for the line
            //telemetry.addData("ODS 1 Light Level", whiteLineSensor1.getLightDetected());
            //telemetry.update();
            //idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            //telemetry.addData("ODS 2 Light Level", whiteLineSensor2.getLightDetected());
            //telemetry.update();

            //If enough white light has not been detected, keep the power of the motor at .3; else set it to 0
            if (wlsRightlight==false) {
                rightMotor1.setPower(.25);
                rightMotor2.setPower(.25);
            }
            else
            {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            //If enough white light has not been detected, keep the power of the motor at .3; else set it to 0
            if (wlsLeftlight==false) {
                leftMotor1.setPower(.2);
                leftMotor2.setPower(.2);
            }
            else
            {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            timeTwo=clock.seconds ();
        }
        while ((wlsLeftlight==false || wlsRightlight==false)&&((timeTwo-timeOne)<6));  //Repeat do loop until both odss have detected enough white light
        if (timeTwo-timeOne>6)
        {
            while (clock.seconds()<40)
            {
                timeTwo=clock.seconds ();
            }
        }

    }

    public void driveToOneWhiteLineRight ()
    {
        whitesCounter = 0;


        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(.13);
        rightMotor1.setPower(.13);
        leftMotor2.setPower(.13);
        rightMotor2.setPower(.13);

        timeOne = clock.seconds ();
        timeTwo = clock.seconds ();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            loopCounter++;

            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = clock.seconds ();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorRight.getRawLightDetected() >= whiteThreshold) {
                whitesCounter++;
            }


            leftMotor1.setPower(.13);
            leftMotor2.setPower(.13);
            rightMotor1.setPower(.13);
            rightMotor2.setPower(.13);

        }
        while (whitesCounter<4 && (clock.seconds()<40) && (timeTwo-timeOne < 6));  //Repeat do loop until both odss have detected enough white light



        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

    public void driveToOneWhiteLineLeft ()
    {
        whitesCounter = 0;


        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(.13);
        rightMotor1.setPower(.13);
        leftMotor2.setPower(.13);
        rightMotor2.setPower(.13);

        timeOne = clock.seconds ();
        timeTwo = clock.seconds ();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            loopCounter++;

            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = clock.seconds ();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getRawLightDetected() >= whiteThreshold) {
                whitesCounter++;
            }


            leftMotor1.setPower(.13);
            leftMotor2.setPower(.13);
            rightMotor1.setPower(.13);
            rightMotor2.setPower(.13);

        }
        while (whitesCounter<4 && (clock.seconds()<40) && (timeTwo-timeOne < 6));  //Repeat do loop until both odss have detected enough white light

        if (timeTwo-timeOne>6)
        {
            while (clock.seconds()<40)
            {
                timeTwo = clock.seconds ();
            }
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    public void driveMoreLeft (double distance, double speed, double targetHeading)
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

            leftMotor1.setPower(speed+.05);
            leftMotor2.setPower(speed+.05);
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
    //checkrotations
    public void driveMoreRight (double distance, double speed, double targetHeading)
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

            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(speed+.05);
            rightMotor2.setPower(speed+.05);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();


        }
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

    public void driveMoreRightBack (double distance, double speed, double targetHeading)
    {
        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = distance;
        ROTATIONS = distance/ (Math.PI*WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(leftMotor1.getCurrentPosition())<COUNTS) {

            currentHeading = gyro.getIntegratedZValue();
            telemetry.addData("Current Heading = ", currentHeading);
            telemetry.update();
            headingError = targetHeading - currentHeading;
            speedCorrection = headingError * straightGyroGain;

            leftMotor1.setPower(-speed);
            leftMotor2.setPower(-speed);
            rightMotor1.setPower(-speed-.05);
            rightMotor2.setPower(-speed-.05);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();


        }
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

    public void driveMoreLeftBack (double distance, double speed, double targetHeading)
    {
        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = distance;
        ROTATIONS = distance/ (Math.PI*WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(leftMotor1.getCurrentPosition())<COUNTS) {

            currentHeading = gyro.getIntegratedZValue();
            telemetry.addData("Current Heading = ", currentHeading);
            telemetry.update();
            headingError = targetHeading - currentHeading;
            speedCorrection = headingError * straightGyroGain;

            leftMotor1.setPower(-speed-.05);
            leftMotor2.setPower(-speed-.05);
            rightMotor1.setPower(-speed);
            rightMotor2.setPower(-speed);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();


        }
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

    public void lineUpFasterLeftBack ()
    {
        whitesCounterLeft = 0;
        whitesCounterRight = 0;
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wlsRightlight=false;
        wlsLeftlight=false;

        timeOne = clock.seconds ();
        timeTwo = clock.seconds ();

        leftMotor1.setPower(-.25);
        rightMotor1.setPower(-.2);
        leftMotor2.setPower(-.25);
        rightMotor2.setPower(-.2);


        //Keep the motor(s) at .3 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getLightDetected()>=whiteThreshold) {
                whitesCounterLeft++;
                if (whitesCounterLeft>2) {
                    wlsLeftlight = true;
                }
            }
            if (whiteLineSensorRight.getLightDetected()>=whiteThreshold) {
                whitesCounterRight++;
                if (whitesCounterRight>2) {
                    wlsRightlight = true;
                }
            }
            // Display the light level while we are looking for the line
            //telemetry.addData("ODS 1 Light Level", whiteLineSensor1.getLightDetected());
            //telemetry.update();
            //idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            //telemetry.addData("ODS 2 Light Level", whiteLineSensor2.getLightDetected());
            //telemetry.update();

            //If enough white light has not been detected, keep the power of the motor at .3; else set it to 0
            if (wlsRightlight==false) {
                rightMotor1.setPower(-.2);
                rightMotor2.setPower(-.2);
            }
            else
            {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            //If enough white light has not been detected, keep the power of the motor at .3; else set it to 0
            if (wlsLeftlight==false) {
                leftMotor1.setPower(-.25);
                leftMotor2.setPower(-.25);
            }
            else
            {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            timeTwo=clock.seconds ();
        }
        while ((wlsLeftlight==false || wlsRightlight==false)&&((timeTwo-timeOne)<6));  //Repeat do loop until both odss have detected enough white light
        if (timeTwo-timeOne>6)
        {
            while (clock.seconds()<40)
            {
                timeTwo=clock.seconds ();
            }
        }

    }

    public void lineUpFasterRightBack ()
    {
        whitesCounterLeft = 0;
        whitesCounterRight = 0;
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wlsRightlight=false;
        wlsLeftlight=false;

        timeOne = clock.seconds ();
        timeTwo = clock.seconds ();

        leftMotor1.setPower(-.2);
        rightMotor1.setPower(-.25);
        leftMotor2.setPower(-.2);
        rightMotor2.setPower(-.25);


        //Keep the motor(s) at .3 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getLightDetected()>=whiteThreshold) {
                whitesCounterLeft++;
                if (whitesCounterLeft>2) {
                    wlsLeftlight = true;
                }
            }
            if (whiteLineSensorRight.getLightDetected()>=whiteThreshold) {
                whitesCounterRight++;
                if (whitesCounterRight>2) {
                    wlsRightlight = true;
                }
            }
            // Display the light level while we are looking for the line
            //telemetry.addData("ODS 1 Light Level", whiteLineSensor1.getLightDetected());
            //telemetry.update();
            //idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            //telemetry.addData("ODS 2 Light Level", whiteLineSensor2.getLightDetected());
            //telemetry.update();

            //If enough white light has not been detected, keep the power of the motor at .3; else set it to 0
            if (wlsRightlight==false) {
                rightMotor1.setPower(-.25);
                rightMotor2.setPower(-.25);
            }
            else
            {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            //If enough white light has not been detected, keep the power of the motor at .3; else set it to 0
            if (wlsLeftlight==false) {
                leftMotor1.setPower(-.2);
                leftMotor2.setPower(-.2);
            }
            else
            {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            timeTwo=clock.seconds ();
        }
        while ((wlsLeftlight==false || wlsRightlight==false)&&((timeTwo-timeOne)<6));  //Repeat do loop until both odss have detected enough white light
        if (timeTwo-timeOne>6)
        {
            while (clock.seconds()<40)
            {
                timeTwo=clock.seconds();
            }
        }

    }

    public void teleOpToggle ()
    {
        if (gamepad1.right_bumper) { //If gamepad1 right bumper than switch current value of half gain boolean
            justBumped = true;
        }
        if (!gamepad1.right_bumper&&justBumped)
        {
            halfGain= !halfGain;
            justBumped = false;
        }


        if (halfGain) //If we are on half gain then
        { //set the gain to .5 and show telemetry showing that
            gain = .5;
            telemetry.addData("On half gain (true/false)", halfGain);
        } else {
            gain = 1;
            telemetry.addData("On half gain (true/false)", halfGain);
        }
    }

    public void teleOpDrive ()
    {
        joystick1ValueLeft= gamepad1.left_stick_y; //set joystick1ValueLeft to the raw value of gamepad1.left_stick_y
        joystick1ValueRight = gamepad1.right_stick_y; //set joystick1ValueRight to the raw value of gamepad1.right_stick_y

        leftPower = .95 * (Math.pow(joystick1ValueLeft, 3)) * gain;
        rightPower = .95 * (Math.pow(joystick1ValueRight, 3)) * gain;

        if (joystick1ValueLeft < 0) {
            leftPower *= -Math.abs(leftPower);
        }
        if (joystick1ValueRight < 0) {
            rightPower *= -Math.abs(rightPower);
        }


        //set left motors and right motors to leftPower and rightPower, respectively
        leftMotor1.setPower(leftPower);
        leftMotor2.setPower(leftPower);


        rightMotor1.setPower(rightPower);
        rightMotor2.setPower(rightPower);
    }


    public void pressBeaconFrontRed (boolean goingForward)
    {
        timeOne = clock.seconds();
        push = false;
        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);

        while (timeTwo-timeOne<4) {
            if (colorSensorLeft.red() > 1.5 && colorSensorLeft.blue() < 1.5 && !push) {
                beaconPusherLeft.setPosition(beaconPusherLeftExtendPosition);
                push = true;
            }

            if (!push)
            {
                if (goingForward)
                {
                    leftMotor1.setPower(.2);
                    leftMotor2.setPower (.2);
                    rightMotor1.setPower(.2);
                    rightMotor2.setPower(.2);
                }
                else
                {
                    leftMotor1.setPower(-.2);
                    leftMotor2.setPower (-.2);
                    rightMotor1.setPower(-.2);
                    rightMotor2.setPower(-.2);
                }
            }

            telemetry.addData("Red color", colorSensorLeft.red());
            telemetry.update();

            timeTwo = clock.seconds();
        }

        if (timeTwo-timeOne>6)
        {
            while (clock.seconds()<40)
            {
                timeTwo = clock.seconds();
            }
        }

        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);

        telemetry.addData("Done", telemetryVariable);
    }

    public void pressBeaconFrontBlue (boolean goingForward) {
        timeOne = clock.seconds();
        push = false;
        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);

        while (timeTwo-timeOne<4) {
            if (colorSensorLeft.blue() > 1.5 && colorSensorLeft.red() < 1.5 && !push) {
                beaconPusherRight.setPosition(beaconPusherRightExtendPosition);
                push = true;
            }

            if (!push)
            {
                if (goingForward)
                {
                    leftMotor1.setPower(.2);
                    leftMotor2.setPower (.2);
                    rightMotor1.setPower(.2);
                    rightMotor2.setPower(.2);
                }
                else
                {
                    leftMotor1.setPower(-.2);
                    leftMotor2.setPower (-.2);
                    rightMotor1.setPower(-.2);
                    rightMotor2.setPower(-.2);
                }
            }

            telemetry.addData("Red color", colorSensorLeft.red());
            telemetry.update();

            timeTwo = clock.seconds();
        }

        if (timeTwo-timeOne>6)
        {
            while (clock.seconds()<40)
            {
                timeTwo = clock.seconds();
            }
        }

        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);

        telemetry.addData("Done", telemetryVariable);
    }

















}
