//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to provide methods for the autonomous concept
//of shooting two balls and pressing two beacons



//package declaration
package org.firstinspires.ftc.teamcode;

//import statements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public abstract class FuctionsForILTNew extends LinearOpMode {

    //Variable Declarations

    //Motors
    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back
    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system
    DcMotor cap1;         //Cap ball motor
    DcMotor cap2;        //Cap ball motor

    TouchSensor touchSensor;

    boolean wlsRightlight = false;  //Boolean for whether right ods has seen white light, initially set to false
    boolean wlsLeftlight = false;   //Boolean for whether left ods has seen white light, initially set to false

    //Servos
    Servo hood;       //position servo for 180º, adjust angle of shooter
    Servo turret;     //Continuous Rotation
    Servo kicker;     //Servo to kick balls into the flywheel area
    Servo ballControl; //Servo to prevent unwanted balls from ascending
    Servo leftDrawbridge;  //Servo to keep intake folded into robot when desired on the left of the robot
    Servo rightDrawbridge;  //Servo to keep intake folded into robot when desired on the right of the robot
    Servo beaconPusherLeft;    //Servo on the left of the robot for beacon pushing
    Servo beaconPusherRight;   //Servo on the right of the robot for beacon pushing
    Servo capGrab1; //Servo to extend and retract cap ball grabbing arms
    Servo capGrab2; //Servo to extend and retract cap ball grabbing arms
    Servo leftSideWheels;
    Servo rightSideWheels;

    //encoder variables to adequately sense the lines
    final static double ENCODER_CPR = 1120;    //encoder counts per rotation (CPR)
    final static double GEAR_RATIO = 0.6;     //Gear ratio used in Big Sur in 30/18, so in code we multiply by 18/30
    final static double WHEEL_DIAMETER = 4; //wheel diameter in inches

    double shooterPower; //power applied to the shooter
    double turretPosition;  //position set to the turret servo

    //Driving variables
    double inches;  //Desired number of inches to drive
    double rotations;       //Wheel rotations necessary to drive the above amount of inches
    double counts;//Encoder counts necessary to drive the above amount of inches/rotations

    double turnSpeed = 0; //Turn speed outputted from gyro gain during spinMove method

    ColorSensor colorSensorRight; //Right color sensor for beacon autonomous
    ColorSensor colorSensorLeft; //Left color sensor for beacon autonomous

    //Optical distance sensors to detect white light
    OpticalDistanceSensor whiteLineSensorLeft; //left
    OpticalDistanceSensor whiteLineSensorRight; //right

    int loopCounter = 0; //Variable to count how many times a given loop has been entered

    final double WHITE_THRESHOLD = .25; //The threshold of white light necessary to set the below variables to true USED TO BE .4

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

    //time variables set to current run time throughout the code, typically set to this.getRunTime()
    double timeOne;
    double timeTwo;
    double timeRunningLoop;

    //Variables to count how many times white light has been seen in a given loop
    double whitesCount; //General
    double whiteCountLeft; //Specific to only left ods
    double whiteCountRight; //Specific to only right ods

    //Gain to control rpm on the robot
    final double RPM_GAIN = .0000125;

    //Positions of the beacon pushers, based on whether retracting or extending
    final double BEACON_PUSHER_LEFT_RETRACT_POSITION = .05; //left retract position
    final double BEACON_PUSHER_LEFT_EXTEND_POSITION = .95; //left extend position
    final double BEACON_PUSHER_RIGHT_RETRACT_POSITION = .95; //right retract position
    final double BEACON_PUSHER_RIGHT_EXTEND_POSITION = .05; //right extend position

    //Boolean that is true when a beacon has been extended and is false otherwise
    boolean push;

    //Boolean that is true when a white line has not been detected and is false otherwise
    boolean whiteLineNotDetected = true;

    //Boolean that is true when a beacon has not been detected and is false otherwise
    boolean beaconNotDetected = true;

    //Gyro sensor declaration
    ModernRoboticsI2cGyro gyro;

    //measure gyro heading/position
    double currentHeading;

    //measure difference between current and desired gyro heading
    double headingError;

    //variable to measure the gyro heading at the beginning of a method
    double initialHeading;

    //Gain for the gyro when executing a spin move
    final double GYRO_GAIN =.0099;

    //Gain for the gyro when driving straight
    double straightGyroGain = .025; //.007

    //Adjustment factor for motor powers when driving straight based on straightGyroGain
    double straightDriveAdjust = 0;

    // F U N C T I O N S   F O R   A U T O   &   T E L E O P

    //Given a number of wheel rotations and target motor speed,
    //the robot spins clockwise at the given speed for the given
    //amount of wheel rotations
    public void encoderTurnClockwise (double rotations, double speed)
    {
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //while loop that is entered until leftMotor1 reaches its target encoder count
        while (leftMotor1.getCurrentPosition()<counts) {
            //Set all driving powers
            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(-speed);
            rightMotor2.setPower(-speed);
            //Telemetry for robot rotations executed and encoder position
            telemetry.addData("Current encoder position = ", leftMotor1.getCurrentPosition());
            telemetry.addData("Current rotations travelled = ", (leftMotor1.getCurrentPosition())/(ENCODER_CPR*GEAR_RATIO));
            telemetry.update();
        }
        //Execute stopDriving method
        stopDriving();
    }

    //Given a number of wheel rotations and target motor speed,
    //the robot spins counter-clockwise at the given speed for the given
    //amount of wheel rotations
    public void encoderTurnCounterClockwise (double rotations, double speed)
    {
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set run mode of rightMotor1 to STOP_AND_RESET_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //while loop that is entered until rightMotor1 reaches its target encoder count
        while ((Math.abs(rightMotor1.getCurrentPosition())<counts)) {
            //Set all driving powers
            leftMotor1.setPower(-speed);
            leftMotor2.setPower(-speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);
            //Telemetry for robot rotations executed and encoder position
            telemetry.addData("Current encoder position = ", rightMotor1.getCurrentPosition());
            telemetry.addData("Current rotations travelled = ", (rightMotor1.getCurrentPosition())/(ENCODER_CPR*GEAR_RATIO));
            telemetry.update();
        }
        //Execute stopDriving method
        stopDriving();
    }

    //Drives straight and backwards for a provided distance, in inches
    //and at a given speed and a given gyro heading
    public void driveBack (double distance, double speed, double targetHeading)
    {
        //Set variable initialHeading to gyro's integrated z value
        initialHeading = gyro.getIntegratedZValue();
        //math to calculate total counts robot should travel
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set timeOne and timeTwo to this.getRuntime();
        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        //Execute loop while the absolute value of leftMotor1's current encoder position is under target encoeder count
        //And while time ellapsed in loop is less than 4
        while (Math.abs(leftMotor1.getCurrentPosition())<counts && (timeTwo-timeOne<4)) {
            //Set variable initialHeading to gyro's integrated z value
            currentHeading = gyro.getIntegratedZValue();
            //Calculate motor power adjustment factor based on proportional control in order to maintain
            //the desired gyro heading
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;
            //Set motor powers based on paramater speed and straightDriveAdjust
            leftMotor1.setPower(-speed+straightDriveAdjust);
            leftMotor2.setPower(-speed+straightDriveAdjust);
            rightMotor1.setPower(-speed-straightDriveAdjust);
            rightMotor2.setPower(-speed-straightDriveAdjust);
            //Telemetry for encoder position
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
            //Set timeTwo to this.getRuntime ()
            timeTwo= this.getRuntime();
        }
        //Safety timeout based on if the loop above executed in under 4 seconds
        //If it did not, do not execute the rest of the program
        if (timeTwo-timeOne>4)
        {
            while (this.opModeIsActive())
            {
                stopDriving();
                timeTwo=this.getRuntime();
            }
        }
        //Execute stopDriving method
        stopDriving();
    }

    //Sets all drive train motors to 0 power
    public void stopDriving ()
    {
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    //Sets intake and shooter to 0 power
    public void stopShooting ()
    {
        intake.setPower(0);
        shooter.setPower(0);
    }

    //Execute a robot spin using both sides of the drive train and the gyro
    public void spinMove (double desiredHeading) //desired gyro heading
    {
        //Set all drive train motor's run modes to RUN_WITHOUT_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set initialHeading to gyro's integrated Z value
        initialHeading = gyro.getIntegratedZValue();

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne= this.getRuntime();
        timeTwo=this.getRuntime();

        if (desiredHeading < initialHeading) //CLOCKWISE TURN
        {
            do{
                //Math to calculate turnSpeed of robot using proportional control
                //with turnSpeed lowering as robot reaches its heading target
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = -headingError * GYRO_GAIN;

                //Clip turnSpeed to be .4 through .8
                if (turnSpeed < 0.23) {
                    turnSpeed = 0.23;
                }
                if (turnSpeed > .8) {
                    turnSpeed = .8;
                }

                //turnSpeed = Range.clip(turnSpeed,.12,.5);

                //Set motor powers
                rightMotor1.setPower(-turnSpeed);
                rightMotor2.setPower(-turnSpeed);
                leftMotor1.setPower(turnSpeed);
                leftMotor2.setPower(turnSpeed);
                //Execute gyroTelemetry method
                gyroTelemetry();

                //Set timeTwo to this.getRuntime()
                timeTwo=this.getRuntime();


            }
            while (currentHeading > desiredHeading && (timeTwo-timeOne<6));
            //while current heading is greater than desired heading, and time elapsed in loop
            //is under 6 seconds
        }
        else //Counterclockwise turn
        {
            do {
                //Math to calculate turnSpeed of robot using proportional control
                //with turnSpeed lowering as robot reaches its heading target
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * GYRO_GAIN;

                //Clip turnSpeed to be .4 through .8
                if (turnSpeed < 0.23) {
                    turnSpeed = 0.23;
                }
                if (turnSpeed > .8) {
                    turnSpeed = .8;
                }

                //Set motor powers
                rightMotor1.setPower(turnSpeed);
                rightMotor2.setPower(turnSpeed);
                leftMotor1.setPower(-turnSpeed);
                leftMotor2.setPower(-turnSpeed);
                //Execute gyroTelemetry method
                gyroTelemetry();

                //Set timeTwo to this.getRuntime()
                timeTwo=this.getRuntime();
            }
            while (currentHeading < desiredHeading && (timeTwo-timeOne<6));
            //while current heading is less than desired heading, and time elapsed in loop
            //is under 6 seconds
        }
        //Safety timeout
        if (timeTwo-timeOne>6)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }

            telemetry.addLine("Timed out");
            telemetry.update();
        }
        //Execute stopDriving method
        stopDriving();
    }

    //Execute a robot spin using one side
    public void pivot (double time, double speed, boolean clockwise)
    {
        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        if (clockwise)
        {
            while (timeTwo-timeOne<time)
            {
                timeTwo=this.getRuntime();
                leftMotor1.setPower(speed);
                leftMotor2.setPower(speed);
            }
        }

        else
        {
            while (timeTwo-timeOne<time)
            {
                timeTwo=this.getRuntime();
                rightMotor1.setPower(speed);
                rightMotor2.setPower(speed);
            }
        }
    }

    //Provide telemetry related to the gyro sensor
    public void gyroTelemetry ()
    {
        //Current value of the gyro's integrated z value
        telemetry.addData("Heading", gyro.getIntegratedZValue());
        //Current value of the turnSpeed variable
        telemetry.addData("Turn Speed", turnSpeed);
        telemetry.update();
    }

    //Configures all hardware devices, and sets them to their initial
    //values, if necessary

    //WRITE IN ENCODER PORTS
    public void Configure ()
    {

        rightMotor1 = hardwareMap.dcMotor.get("m3"); //Motor Controller 4, port 2, XV78
        rightMotor2 = hardwareMap.dcMotor.get("m4"); //Motor Controller 4, port 1, XV78
        leftMotor1 = hardwareMap.dcMotor.get("m1"); //Motor Controller 1, port 2, UVQF
        leftMotor2 = hardwareMap.dcMotor.get("m2"); //Motor Controller 1, port 1, UVQF
        rightMotor1.setDirection(DcMotor.Direction.FORWARD); //Set direction of rightMotor1 to forward
        rightMotor2.setDirection(DcMotor.Direction.FORWARD); //Set direction of rightMotor2 to forward
        leftMotor1.setDirection(DcMotor.Direction.REVERSE); //Set direction of leftMotor1 to reverse
        leftMotor2.setDirection(DcMotor.Direction.REVERSE); //Set direction of leftMotor2 to reverse

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stop and reset encoder of leftMotor1

        shooter = hardwareMap.dcMotor.get("m5"); //Motor Controller 2, port 1, 9PCE
        shooter.setDirection(DcMotor.Direction.FORWARD); //Set shooter motor to forward direction

        intake = hardwareMap.dcMotor.get("m6"); //Motor Controller 2, port 2, 9PCE
        intake.setDirection(DcMotorSimple.Direction.REVERSE); //Set intake motor to forward direction

        turret = hardwareMap.servo.get("s1"); //Servo Controller 1, port 2, VSI1
        hood = hardwareMap.servo.get("s2"); //Servo Controller 1, port 1, VSI1

        touchSensor = hardwareMap.touchSensor.get("touch"); //Digital port 1

        beaconPusherLeft = hardwareMap.servo.get("s4"); //Servo Controller 3, port 2, VCT7
        beaconPusherRight = hardwareMap.servo.get("s3"); //Servo Controller 3, port 1, VCT7

        cap1 = hardwareMap.dcMotor.get("m7"); //Motor Controller 3, port 1, VF7F
        cap2 = hardwareMap.dcMotor.get("m8"); //Motor Controller 3, port 2, VF7F

        cap1.setDirection(DcMotor.Direction.FORWARD); //Set cap1 motor to forward setting
        cap2.setDirection(DcMotor.Direction.FORWARD); //Set cap2 motor to reverse setting

        ballControl = hardwareMap.servo.get("s5"); //Servo Controller 1, port 4, VSI1

        leftDrawbridge = hardwareMap.servo.get("s6"); //Servo Controller 3, port 3, VCT7
        rightDrawbridge = hardwareMap.servo.get("s7"); //Servo Controller 3, port 4, VCT7

        kicker = hardwareMap.servo.get("s8"); //Servo Controller 1, port 3, VSI1

        capGrab1 = hardwareMap.servo.get("s9"); //Servo Controller 3, port 5, VCT7
        capGrab2 = hardwareMap.servo.get("s10"); //Servo Controller 3, port 6, VCT7

        capGrab1.setDirection(Servo.Direction.FORWARD); //Set capGrab1 motor to forward position
        capGrab2.setDirection(Servo.Direction.REVERSE); //Set capGrab2 motor to reverse position

        leftSideWheels= hardwareMap.servo.get("s11"); //Servo Controller 1, port 6, VSI1;
        rightSideWheels= hardwareMap.servo.get("s12"); //Servo Controller 1, port 5, VSI1;
        leftSideWheels.setPosition(.5);
        rightSideWheels.setPosition(.5);

        beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_RETRACT_POSITION); //Set beaconPusherLeft to beaconPusherLeftRetractPosition
        beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_RETRACT_POSITION); //Set beaconPusherRight to beaconPusherRightRetractPosition
        turret.setPosition(turretPosition); //Set turret to turretPosition
        kicker.setPosition(0); //Set kicker to 0
        ballControl.setPosition(0); //Set ballControl to .75
        hood.setPosition(1); //Set hood to 1
        leftDrawbridge.setPosition(.5); //Set leftDrawbridge to .5
        rightDrawbridge.setPosition(.5); //Set rightDrawbridge to .5
        capGrab1.setPosition(.5); //Set capGrab1 to .5
        capGrab2.setPosition(.5); ////Set capGrab2 to .5

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); //I2C port 0

        whiteLineSensorRight= hardwareMap.opticalDistanceSensor.get("ods2");    //Analog import port 0
        whiteLineSensorLeft= hardwareMap.opticalDistanceSensor.get("ods1");     //Analog import port 4
        whiteLineSensorRight.enableLed(true); //Set enableLed of whiteLineSensorRight to true
        whiteLineSensorLeft.enableLed(true); //Set enableLed of whiteLineSensorLeft to true

        I2cAddr i2cColorLeft = I2cAddr.create8bit(0x5c);
        I2cAddr i2cColorRight = I2cAddr.create8bit(0x3c);


        //requires moving connection based on alliance color
        colorSensorRight = hardwareMap.colorSensor.get("colorright");     //I2C port 2
        colorSensorRight.setI2cAddress(i2cColorRight);
        colorSensorRight.enableLed(false); //Set enableLed of colorSensorRight to false

        colorSensorLeft = hardwareMap.colorSensor.get("colorleft");     //I2C port 5
        colorSensorLeft.setI2cAddress(i2cColorLeft);
        colorSensorLeft.enableLed(false); //Set enableLed of colorSensorLeft to false

        push=false; //Set push variable to false

        leftDrawbridge.setPosition(.5); //Set leftDrawbridge position to .5
        rightDrawbridge.setPosition(.5); //Set rightDrawbridge positon to .5

        //Set gyro variables to 0
        currentHeading=0;
        initialHeading=0;
        headingError=0;
        turnSpeed=0;

        //Set initial shooterPower to .95
        shooterPower = .95;

        //Set variable turretPosition to .5
        turretPosition=.5;

        //Initialize encoder variables to 0
        inches=0;
        rotations=0;
        counts=0;

        //Set white line variables to 0
        whitesCount=0;
        whiteCountLeft=0;
        whiteCountRight=0;
    }

    //Calibrate the gyro sensor
    public void calibrateGyro () throws InterruptedException
    {
        gyro.calibrate();

        while (gyro.isCalibrating())
        {
            sleep(500);
            telemetry.addLine("Gyro is not calibrated");
            telemetry.update();
            leftSideWheels.setPosition(.95);
            rightSideWheels.setPosition(.95);
        }

        leftSideWheels.setPosition(.5);
        rightSideWheels.setPosition(.5);
    }

    //Runs the flywheel shooter, attempting to maintain a constant
    //rpm of the shooter, and also a constant speed of the intake
    //motor.  If the rpm is not close enough to the target value,
    //the intake will not run as too ensure balls are given the
    //best opportunity to score.
    public void shootAndLift (double targetRPM, double intakeSpeed) throws InterruptedException
    {
        shooter.setPower(shooterPower);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
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

                if (avgRpm > targetRPM) {
                    intake.setPower(intakeSpeed);
                    ballControl.setPosition(.24);
                } else {
                    intake.setPower(0);
                }

                //rpmGain now equal to .0000001 which is what we use in teleop.  may need to be adjusted
                shooterPower += RPM_GAIN * (targetRPM - weightedAvg);

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
            kicker.setPosition(.7);
            shooter.setPower (shooterPower);
        }

        stopShooting();

        kicker.setPosition(0);
        shooter.setPower(0);
        intake.setPower(0);
        //driveNext();
    }


    //drive forward at a given distance, speed and gyro heading
    public void drive (double distance, double speed, double targetHeading, boolean isStopAtEnd)
    {
        initialHeading = gyro.getIntegratedZValue();
        inches = distance;
        rotations = distance/ (Math.PI*WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timeOne = this.getRuntime();
        timeTwo= this.getRuntime();

        while (leftMotor1.getCurrentPosition()<counts && (timeTwo-timeOne < 4)) {
            currentHeading = gyro.getIntegratedZValue();
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;
            leftMotor1.setPower(speed + straightDriveAdjust);
            leftMotor2.setPower(speed + straightDriveAdjust);
            rightMotor1.setPower(speed - straightDriveAdjust);
            rightMotor2.setPower(speed - straightDriveAdjust);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
            timeTwo = this.getRuntime();
        }

        if (timeTwo-timeOne>4)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }
        if (isStopAtEnd) {
            stopDriving();
        }

    }

    //drive forward at a given distance, speed, but without considering gyro heading
    public void driveNoGyro (double distance, double speed)
    {

        inches = distance;
        rotations = distance/ (Math.PI*WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timeOne = this.getRuntime();
        timeTwo= this.getRuntime();

        while (leftMotor1.getCurrentPosition()<counts && (timeTwo-timeOne<3)) {
            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
            timeTwo = this.getRuntime();
        }

        if (timeTwo-timeOne>3)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

    //drive at a given speed until the touch sensor is pressed
    public void driveToTouch (double speed)
    {
        timeOne=this.getRuntime();
        timeTwo= this.getRuntime();

        while (!touchSensor.isPressed()&& (timeTwo-timeOne<4)){
            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);
            timeTwo=this.getRuntime();
        }

        stopDriving();
    }

    //Drive at a given speed until the left ods sees adequate white light
    public void driveToWhiteLineLeft(double speed, double targetHeading, boolean isStopAtEnd)
    {
        whitesCount= 0;

        whiteLineNotDetected = true;

        initialHeading = gyro.getIntegratedZValue();

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            currentHeading = gyro.getIntegratedZValue();
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            loopCounter++;

            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            //telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getRawLightDetected() >= WHITE_THRESHOLD) {
                whiteLineNotDetected = false;
                whitesCount++;
            }

            if (timeTwo-timeOne>4)
            {
                stopDriving();
                while (this.opModeIsActive())
                {
                    timeTwo=this.getRuntime();
                }
            }

            leftMotor1.setPower(speed+straightDriveAdjust);
            rightMotor1.setPower(speed-straightDriveAdjust);
            leftMotor2.setPower(speed+straightDriveAdjust);
            rightMotor2.setPower(speed-straightDriveAdjust);



        }
        while (whiteLineNotDetected && this.opModeIsActive() && (timeTwo-timeOne<3) && whitesCount<3);  //Repeat do loop until both odss have detected enough white light


        if (timeTwo-timeOne>3)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }

        if (isStopAtEnd)
        {
            stopDriving();
        }
    }

    //Drive at a given speed until both ods sensors see adequate white light
    public void driveToWhiteLine (double speed, double targetHeading)
    {
        whiteCountLeft=0;
        whiteCountRight=0;

        wlsRightlight = false;
        wlsLeftlight = false;

        initialHeading = gyro.getIntegratedZValue();

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the motor(s) at .25 while op mode is active and not enough white light has been detected on a motor's ods
        do {
            currentHeading = gyro.getIntegratedZValue();
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorLeft.getRawLightDetected() >= WHITE_THRESHOLD) {
                wlsLeftlight = true;
                whiteCountLeft++;
            }
            if (whiteLineSensorRight.getRawLightDetected() >= WHITE_THRESHOLD) {
                wlsRightlight = true;
                whiteCountRight++;
            }

            //If enough white light has not been detected, keep the power of the motor at .25; else set it to 0
            if (wlsRightlight == false) {
                rightMotor1.setPower(speed - straightDriveAdjust);
                rightMotor2.setPower(speed - straightDriveAdjust);
            } else {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }


            //If enough white light has not been detected, keep the power of the motor at .25; else set it to 0
            if (wlsLeftlight == false) {
                leftMotor1.setPower(speed + straightDriveAdjust);
                leftMotor2.setPower(speed + straightDriveAdjust);
            } else {
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
            }

        }
        while ( (wlsRightlight == false
                || wlsLeftlight == false)
                && this.opModeIsActive()
                && (timeTwo-timeOne < 8));  //Repeat do loop until both odss have detected enough white light

        if (timeTwo-timeOne>8)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);


    }

    //Drive at a given speed until the right ods sees adequate white light
    public void driveToWhiteLineRight(double speed, double targetHeading, boolean isStopAtEnd)
    {
        whitesCount = 0;

        whiteLineNotDetected = true;

        initialHeading = gyro.getIntegratedZValue();

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

            currentHeading = gyro.getIntegratedZValue();
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            //telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean to true
            if (whiteLineSensorRight.getRawLightDetected() >= WHITE_THRESHOLD) {
                whiteLineNotDetected = false;
                whitesCount++;
            }

            if (timeTwo-timeOne>4)
            {
                stopDriving();
                while (this.opModeIsActive())
                {
                    timeTwo=this.getRuntime();
                }
            }

            leftMotor1.setPower(speed+straightDriveAdjust);
            rightMotor1.setPower(speed-straightDriveAdjust);
            leftMotor2.setPower(speed+straightDriveAdjust);
            rightMotor2.setPower(speed-straightDriveAdjust);

        }
        while (whiteLineNotDetected && this.opModeIsActive() && (timeTwo-timeOne<3) && whitesCount<3);  //Repeat do loop until both odss have detected enough white light

        if (timeTwo-timeOne>3)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }

        if (isStopAtEnd)
        {
            stopDriving();
        }

    }

    //Drive forward or back, applying higher power to the right side until the left ods sees adequate white light
    public void driveToWhiteLineRightLeftSideFaster(int plusOrNegative)
    {
        //for variable plusOrNegative put -1 for backwards and 1 for forwards
        whiteLineNotDetected = true;

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(plusOrNegative * .4);
        leftMotor2.setPower(plusOrNegative * .4);
        rightMotor1.setPower(plusOrNegative * .35);
        rightMotor2.setPower(plusOrNegative * .35);

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
            if (whiteLineSensorRight.getRawLightDetected() >= WHITE_THRESHOLD) {
                whiteLineNotDetected = false;
            }



            leftMotor1.setPower(plusOrNegative * .4);
            leftMotor2.setPower(plusOrNegative * .4);
            rightMotor1.setPower(plusOrNegative * .35);
            rightMotor2.setPower(plusOrNegative * .35);


        }
        while (whiteLineNotDetected && this.opModeIsActive());  //Repeat do loop until both odss have detected enough white light

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    //Drive forward or back, applying higher power to the left side until the right ods sees adequate white light
    public void driveToWhiteLineLeftRightSideFaster(int plusOrNegative)
    {
        //for variable plusOrNegative put -1 for backwards and 1 for forwards
        whiteLineNotDetected = true;

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor1.setPower(plusOrNegative * .35);
        leftMotor2.setPower(plusOrNegative * .35);
        rightMotor1.setPower(plusOrNegative * .4);
        rightMotor2.setPower(plusOrNegative * .4);

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
            if (whiteLineSensorLeft.getRawLightDetected() >= WHITE_THRESHOLD) {
                whiteLineNotDetected = false;
            }



            leftMotor1.setPower(plusOrNegative * .35);
            leftMotor2.setPower(plusOrNegative * .35);
            rightMotor1.setPower(plusOrNegative * .4);
            rightMotor2.setPower(plusOrNegative * .4);


        }
        while (whiteLineNotDetected && this.opModeIsActive());  //Repeat do loop until both odss have detected enough white light

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
    }

    //Drive forward for a given distance and speed, but adjusting to give the left side
    //of the drive train more power
    public void driveMoreLeft (double distance, double speed)
    {
        inches = distance;
        rotations = distance/ (Math.PI*WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (leftMotor1.getCurrentPosition()<counts) {

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

    //Drive forward for a given distance and speed, but adjusting to give the right side
    //of the drive train more power
    public void driveMoreRight (double distance, double speed)
    {

        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel


        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (leftMotor1.getCurrentPosition()<counts) {

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

    //Run method to drive at a given speed until adequate blue light is detected
    //at which time the appropriate beacon pusher extends and retracts to push
    //the beacon
    public void pressBeaconSideBlue (double speed, double targetHeading)
    {
        initialHeading = gyro.getIntegratedZValue();

        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        beaconNotDetected = true;
        beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_RETRACT_POSITION);

        do {
            currentHeading = gyro.getIntegratedZValue();
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            telemetry.addData("Blue Value: ", colorSensorRight.blue());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met

            //If enough white light has been detected, set the ods boolean to true
            if (colorSensorRight.blue() >= 2 && colorSensorRight.red() < 2) {
                beaconNotDetected = false;
            }

            leftMotor1.setPower(speed + straightDriveAdjust);
            leftMotor2.setPower(speed + straightDriveAdjust);
            rightMotor1.setPower(speed - straightDriveAdjust);
            rightMotor2.setPower(speed - straightDriveAdjust);

            timeTwo=this.getRuntime();

        } while (beaconNotDetected && this.opModeIsActive() && (timeTwo-timeOne < 12));

        stopDriving();

        if (timeTwo-timeOne>12)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_EXTEND_POSITION);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();


        while (timeTwo - timeOne < 1) {
            beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_RETRACT_POSITION);
            timeTwo = this.getRuntime();
        }

    }

    //Run method to drive at a given speed until adequate red light is detected
    //at which time the appropriate beacon pusher extends and retracts to push
    //the beacon
    public void pressBeaconSideRed (double speed, double targetHeading)
    {
        initialHeading = gyro.getIntegratedZValue();

        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        beaconNotDetected = true;
        beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_RETRACT_POSITION);

        do {
            currentHeading = gyro.getIntegratedZValue();
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            telemetry.addData("Red Value: ", colorSensorLeft.red());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met

            //If enough white light has been detected, set the ods boolean to true
            if (colorSensorLeft.red() >= 2 && colorSensorLeft.blue() < 2) { //change csensor to left
                beaconNotDetected = false;
            }

            leftMotor1.setPower(speed + straightDriveAdjust);
            leftMotor2.setPower(speed + straightDriveAdjust);
            rightMotor1.setPower(speed - straightDriveAdjust);
            rightMotor2.setPower(speed - straightDriveAdjust);

            timeTwo=this.getRuntime();

        } while (beaconNotDetected && this.opModeIsActive() && (timeTwo-timeOne < 12));

        stopDriving();

        if (timeTwo-timeOne>12)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }


        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 3) {
            beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_EXTEND_POSITION);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 1) {
            beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_RETRACT_POSITION);
            timeTwo = this.getRuntime();
        }

    }

}
