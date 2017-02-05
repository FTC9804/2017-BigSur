//package declaration
package org.firstinspires.ftc.teamcode;

//import statements
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


public abstract class FunctionsForILT extends LinearOpMode {
    //Variable Declarations

    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back
    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system

    double whitesCounter = 0;

    int loopCounter = 0;

    Servo hood;       //position servo for 180º, adjust angle of shooter
    Servo turret;     //Continuous Rotation

    Servo kicker;

    Servo ballControl;

    Servo leftIntake;
    Servo rightIntake;

    Servo beaconPusherLeft;    //Servo on the left of the robot for beacon pushing
    Servo beaconPusherRight;   //Servo on the right of the robot for beacon pushing



    //encoder variables to adequately sense the lines
    final static double ENCODER_CPR = 1120;    //encoder counts per rotation (CPR)
    final static double GEAR_RATIO = 0.75;     //Gear ratio used in Big Sur in 24/18, so in code we multiply by 18/24
    final static double WHEEL_DIAMETER = 4; //wheel diameter in inches


    double shooterPower = 0.85; //initial power applied to the shooter

    double turretPosition=.5;  //initial position set to the turret servo

    //Driving variables, initialized to 0
    double INCHES_TO_MOVE=0;  //Desired number of inches to drive
    double ROTATIONS=0;       //Wheel rotations necessary to drive the above amount of inches
    double COUNTS=0;          //Encoder counts necessary to drive the above amount of inches/rotations

    //Color sensor, located
    ColorSensor colorSensor;


    //Optical distance sensors to detect white light
    OpticalDistanceSensor whiteLineSensorLeft;
    OpticalDistanceSensor whiteLineSensorRight;

    double whiteThreshold = .4; //The threshold of white light necessary to set the below variables to true USED TO BE .4
    boolean wlsRightlight = false; //Boolean expressing whether the whiteLineSensorRight has seen a sufficient amount of white light
    boolean wlsLeftlight = false;  //Boolean expressing whether the whiteLineSensorLeft has seen a sufficient amount of white light

    int whitesCounterLeft;
    int whitesCounterRight;

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
    double timeRunningLoop;

    double robotCircumference = 16.125 * Math.PI;
    double encoderTurnArc = 0;

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

    //Gain to control rpm on the robot
    double rpmGain = .0000099;

    //(not being used) The necesarry correction to power applied to motors when driving forward, based on the proximity between current and desired gyro headings
    double speedCorrection;

    //Speed applied to motors when executing a pivot or spinMove, based on both GYRO_GAIN and headingError
    double turnSpeed;

    //Placeholder Variable to display in telemetry
    int telemetryVariable = 0;

    //Array of the last 5 rpm values
    //double [] movingWeightedAverage = {0, 0, 0, 0, 0};

    //boolean to express whether an exception is being thrown
    boolean throwingException=false;

    //To do, comment this
    //int mode = 0;

    //Variables for choosing alliance prior to auto
    boolean choiceNotSelected = true;  //Boolean exressing if we have not yet chose an alliance
    boolean allianceNotSelected = true; //Boolean exressing if we have not yet chose an alliance?
    boolean weAreRed; //Boolean set to true if we are the red alliance

    //Power applied to the left motor
    double leftPower;

    //Power applied to the right motor
    double rightPower;

    //Positions of the continuos rotation beacon pushers, based on wheter retracting or extending
    double beaconPusherLeftRetractPosition = .05;
    double beaconPusherLeftExtendPosition = .95;
    double beaconPusherRightRetractPosition = .95;
    double beaconPusherRightExtendPosition = .05;

    boolean pushOne;
    boolean pushTwo;

    boolean push;

    boolean whiteLineNotDetected = true;
    boolean beaconNotDetected = true;


// F U N C T I O N S   F O R   A U T O   &   T E L E O P



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

    public void drive (double distance, double speed)
    {
        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = distance;
        ROTATIONS = distance/ (Math.PI*WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftMotor1.getCurrentPosition()<COUNTS) {


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

    public void driveBack (double distance, double speed)
    {
        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = distance;
        ROTATIONS = INCHES_TO_MOVE / (Math.PI * WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (Math.abs(leftMotor1.getCurrentPosition())<COUNTS) {

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

    public void Configure ()
    {
        rightMotor1 = hardwareMap.dcMotor.get("m3");//port 1 on robot and in the hardwaremap
        rightMotor2 = hardwareMap.dcMotor.get("m4");
        leftMotor1 = hardwareMap.dcMotor.get("m1");
        leftMotor2 = hardwareMap.dcMotor.get("m2");
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);
        leftMotor1.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter = hardwareMap.dcMotor.get("m5");

        intake = hardwareMap.dcMotor.get("m6");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);


        turret = hardwareMap.servo.get("s1");
        hood = hardwareMap.servo.get("s2");
        beaconPusherLeft = hardwareMap.servo.get("s4");
        beaconPusherRight = hardwareMap.servo.get("s3");

        ballControl = hardwareMap.servo.get("s5");

        leftIntake = hardwareMap.servo.get("s6");
        rightIntake = hardwareMap.servo.get("s7");

        kicker = hardwareMap.servo.get("s8");

        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);
        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);
        turret.setPosition(turretPosition);

        kicker.setPosition(0);

        ballControl.setPosition(.7);

        hood.setPosition(1);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); //I2C port 0
        whiteLineSensorRight= hardwareMap.opticalDistanceSensor.get("ods2");    //Analog import port 0
        whiteLineSensorLeft= hardwareMap.opticalDistanceSensor.get("ods1");     //Analog import port 4
        whiteLineSensorRight.enableLed(true);
        whiteLineSensorLeft.enableLed(true);

        //colorSensorLeft = hardwareMap.colorSensor.get("colorLeft");     //I2C port 1
        // colorSensorLeft.enableLed(false); //

        //requires moving connection based on alliance color
        colorSensor = hardwareMap.colorSensor.get("color");     //I2C port 1
        colorSensor.enableLed(false); //


        // colorSensorRight = hardwareMap.colorSensor.get("colorRight");     //I2C port 2
        // colorSensorRight.enableLed(false); //


        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);
        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);

        pushOne=false;
        pushTwo=false;
        push=false;

        leftIntake.setPosition(.5);
        rightIntake.setPosition(.5);

    }

    public void shootAndLift (double time, double targetRPM, double elevatorSpeed, double intakeSpeed) throws InterruptedException
    {

        shooter.setPower(shooterPower);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while (timeTwo - timeOne < 1.5) {
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        timeRunningLoop = this.getRuntime();

        while (timeTwo<(timeRunningLoop+4)) {

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

                if (avgRpm + 50 > targetRPM) {
                    intake.setPower(intakeSpeed);
                } else {
                    intake.setPower(0);
                }


                // if (weightedAvg<targetRPM)
                // {
                //     shooterPower+=rpmGain * (Math.abs(targetRPM-weightedAvg));
                // }
                // else
                // {
                //     shooterPower-=rpmGain * (Math.abs(targetRPM-weightedAvg));
                // }
                //used with rpmGain = .0000035;


                //rpmGain now equal to .0000001 which is what we use in teleop.  may need to be adjusted
                shooterPower += rpmGain * (targetRPM - weightedAvg);



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

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 0.5)
        {
            timeTwo= this.getRuntime();
        }

        while (timeTwo<(timeRunningLoop+6))
        {
            timeTwo = this.getRuntime();
            intake.setPower(.95);
            kicker.setPosition(1);
            shooter.setPower (shooterPower);
        }

        stopShooting();


        kicker.setPosition(0);
        shooter.setPower(0);
        intake.setPower(0);
        //driveNext();
    }

    public void stopDrivingAndPause ()
    {

        stopDriving();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while (timeTwo - timeOne < 1.5)
        {
            timeTwo = this.getRuntime();
            telemetry.addData("Time", (timeTwo - timeOne));
        }
    }

    public void runIntakeOnly(double intakeSpeed, double time)
    {
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while ((timeTwo - timeOne) < time) {

            intake.setPower(intakeSpeed);

            timeTwo  = this.getRuntime();
        }

        intake.setPower(0);
    }

    public void lineUpFasterLeft ()
    {
        whitesCounterLeft = 0;
        whitesCounterRight = 0;
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wlsRightlight=false;
        wlsLeftlight=false;

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

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

            timeTwo=this.getRuntime();
        }
        while ((wlsLeftlight==false || wlsRightlight==false)&&((timeTwo-timeOne)<6));  //Repeat do loop until both odss have detected enough white light
        if (timeTwo-timeOne>6)
        {
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
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

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

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

            timeTwo=this.getRuntime();
        }
        while ((wlsLeftlight==false || wlsRightlight==false)&&((timeTwo-timeOne)<6));  //Repeat do loop until both odss have detected enough white light
        if (timeTwo-timeOne>6)
        {
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
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

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            loopCounter++;

            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorRight.getRawLightDetected() >= whiteThreshold) {
                whitesCounter++;
            }


            leftMotor1.setPower(.13);
            leftMotor2.setPower(.13);
            rightMotor1.setPower(.13);
            rightMotor2.setPower(.13);

        }
        while (whitesCounter<4 && this.opModeIsActive() && (timeTwo-timeOne < 6));  //Repeat do loop until both odss have detected enough white light



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

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            loopCounter++;

            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getRawLightDetected() >= whiteThreshold) {
                whitesCounter++;
            }


            leftMotor1.setPower(.13);
            leftMotor2.setPower(.13);
            rightMotor1.setPower(.13);
            rightMotor2.setPower(.13);

        }
        while (whitesCounter<4 && this.opModeIsActive() && (timeTwo-timeOne < 6));  //Repeat do loop until both odss have detected enough white light

        if (timeTwo-timeOne>6)
        {
            while (this.opModeIsActive())
            {
                timeTwo = this.getRuntime();
            }
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    public void driveToWhiteLineLeft(double speed)
    {

        whiteLineNotDetected = true;

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            loopCounter++;

            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            //telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getRawLightDetected() >= whiteThreshold) {
                whiteLineNotDetected = false;
            }



            leftMotor1.setPower(speed);
            rightMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor2.setPower(speed);


        }
        while (whiteLineNotDetected && this.opModeIsActive());  //Repeat do loop until both odss have detected enough white light

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    public void driveToWhiteLineLeftRightSideFaster(int plusOrNegative)
    {
        //for variable plusOrNegative put -1 for backwards and 1 for forwards
        whiteLineNotDetected = true;

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(plusOrNegative * .4);
        leftMotor2.setPower(plusOrNegative * .4);
        rightMotor1.setPower(plusOrNegative * .45);
        rightMotor2.setPower(plusOrNegative * .45);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            loopCounter++;

            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            //telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getRawLightDetected() >= whiteThreshold) {
                whiteLineNotDetected = false;
            }



            leftMotor1.setPower(plusOrNegative * .4);
            leftMotor2.setPower(plusOrNegative * .4);
            rightMotor1.setPower(plusOrNegative * .45);
            rightMotor2.setPower(plusOrNegative * .45);


        }
        while (whiteLineNotDetected && this.opModeIsActive());  //Repeat do loop until both odss have detected enough white light

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
    public void driveMoreRight (double distance, double speed/*, double targetHeading*/)
    {
//        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = distance;
        ROTATIONS = INCHES_TO_MOVE / (Math.PI * WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel


        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftMotor1.getCurrentPosition()<COUNTS) {

//            currentHeading = gyro.getIntegratedZValue();
//            telemetry.addData("Current Heading = ", currentHeading);
//            telemetry.update();
//            headingError = targetHeading - currentHeading;
//            speedCorrection = headingError * straightGyroGain;

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

    public void driveMoreRightBack (double distance, double speed /*, double targetHeading*/)
    {
//        currentHeading = gyro.getIntegratedZValue();
        INCHES_TO_MOVE = distance;
        ROTATIONS = distance/ (Math.PI*WHEEL_DIAMETER);
        COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(leftMotor1.getCurrentPosition())<COUNTS) {

//            currentHeading = gyro.getIntegratedZValue();
//            telemetry.addData("Current Heading = ", currentHeading);
//            telemetry.update();
////            headingError = targetHeading - currentHeading;
////            speedCorrection = headingError * straightGyroGain;

            leftMotor1.setPower(-speed);
            leftMotor2.setPower(-speed);
            rightMotor1.setPower(-speed-.05);
            rightMotor2.setPower(-speed-.05);
            telemetry.addData("Current dmrb", leftMotor1.getCurrentPosition());
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
            telemetry.addData("Current dmlb", leftMotor1.getCurrentPosition());
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

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

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

            telemetry.addData("Lining up", telemetryVariable);
            telemetry.update();
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

            timeTwo=this.getRuntime();
        }
        while ((wlsLeftlight==false || wlsRightlight==false)&&((timeTwo-timeOne)<6));  //Repeat do loop until both odss have detected enough white light
        if (timeTwo-timeOne>6)
        {
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
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

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

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

            telemetry.addData("Lining up", telemetryVariable);
            telemetry.update();
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

            timeTwo=this.getRuntime();
        }
        while ((wlsLeftlight==false || wlsRightlight==false)&&((timeTwo-timeOne)<6));  //Repeat do loop until both odss have detected enough white light
        if (timeTwo-timeOne>6)
        {
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }

    }

    public void pressBeaconFrontRedNew (boolean goingForward)
    {
        timeOne = this.getRuntime();
        push = false;
        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);

        while (timeTwo-timeOne<4) {
            if (colorSensor.red() > 1.5 && colorSensor.blue() < 1.5 && !push) {
                beaconPusherRight.setPosition(beaconPusherRightExtendPosition);
                push = true;
            }

            if (!push)
            {
                if (goingForward)
                {
                    leftMotor1.setPower(-.25);
                    leftMotor2.setPower (-.25);
                    rightMotor1.setPower(-.2);
                    rightMotor2.setPower(-.2);
                }
                else
                {
                    leftMotor1.setPower(.25);
                    leftMotor2.setPower (.25);
                    rightMotor1.setPower(.2);
                    rightMotor2.setPower(.2);
                }
            }

            telemetry.addData("Red color", colorSensor.red());
            telemetry.update();

            timeTwo = this.getRuntime();
        }

        if (timeTwo-timeOne>6)
        {
            while (this.getRuntime()<40)
            {
                timeTwo = this.getRuntime();;
            }
        }

        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);

        telemetry.addData("Done", telemetryVariable);
    }

    public void pressBeaconFrontBlueNew (boolean goingForward)
    {
        timeOne = this.getRuntime();
        push = false;
        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);

        while (timeTwo-timeOne<4) {
            if (colorSensor.blue() > 1.5 && colorSensor.red() < 1.5 && !push) {
                beaconPusherLeft.setPosition(beaconPusherLeftExtendPosition);
                push = true;
            }

            if (!push)
            {
                if (goingForward)
                {
                    leftMotor1.setPower(-.2);
                    leftMotor2.setPower (-.2);
                    rightMotor1.setPower(-.25);
                    rightMotor2.setPower(-.25);
                }
                else
                {
                    leftMotor1.setPower(.2);
                    leftMotor2.setPower (.2);
                    rightMotor1.setPower(.25);
                    rightMotor2.setPower(.25);
                }
            }

            telemetry.addData("Blue color", colorSensor.blue());
            telemetry.update();

            timeTwo = this.getRuntime();;
        }

        if (timeTwo-timeOne>6)
        {
            while (this.getRuntime()<40)
            {
                timeTwo = this.getRuntime();;
            }
        }

        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);

        telemetry.addData("Done", telemetryVariable);
    }

    public void pressBeaconSideBlue (double speed)
    {

        beaconNotDetected = true;
        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);

        do {

            telemetry.addData("Blue Value: ", colorSensor.blue());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met

            //If enough white light has been detected, set the ods boolean to true
            if (colorSensor.blue() > 1.5 && colorSensor.red() < 1.5) {
                beaconNotDetected = false;
            }

            leftMotor1.setPower(speed);
            rightMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor2.setPower(speed);


        } while (beaconNotDetected && this.opModeIsActive());

        stopDriving();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 2) {
            beaconPusherLeft.setPosition(beaconPusherLeftExtendPosition);
            timeTwo = this.getRuntime();
        }

        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);

    }



}
