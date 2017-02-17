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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public abstract class FunctionsForLA extends LinearOpMode {

    //Variable Declarations

    //Motors
    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back
    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system
    DcMotor cap1;
    DcMotor cap2;


    TouchSensor touchSensor;

    boolean wlsRightlight = false;
    boolean wlsLeftlight = false;

    //Servos
    Servo hood;       //position servo for 180º, adjust angle of shooter
    Servo turret;     //Continuous Rotation
    Servo kicker;
    Servo ballControl;
    Servo leftDrawbridge;
    Servo rightDrawbridge;
    Servo beaconPusherLeft;    //Servo on the left of the robot for beacon pushing
    Servo beaconPusherRight;   //Servo on the right of the robot for beacon pushing
    Servo capGrab1;
    Servo capGrab2;

    boolean telemetryVariable = true;


    //encoder variables to adequately sense the lines
    final static double ENCODER_CPR = 1120;    //encoder counts per rotation (CPR)
    final static double GEAR_RATIO = 0.6;     //Gear ratio used in Big Sur in 30/18, so in code we multiply by 18/30
    final static double WHEEL_DIAMETER = 4; //wheel diameter in inches

    double shooterPower = 0.95; //initial power applied to the shooter
    double turretPosition=.5;  //initial position set to the turret servo

    //Driving variables, initialized to 0
    double inches=0;  //Desired number of inches to drive
    double rotations=0;       //Wheel rotations necessary to drive the above amount of inches
    double counts=0;//Encoder counts necessary to drive the above amount of inches/rotations

    double turnSpeed = 0;

    //Color sensor, located
    ColorSensor colorSensor;

    //Optical distance sensors to detect white light
    OpticalDistanceSensor whiteLineSensorLeft;
    OpticalDistanceSensor whiteLineSensorRight;

    int loopCounter = 0; //Variable to count how many times a given loop has been entered

    double whiteThreshold = .25; //The threshold of white light necessary to set the below variables to true USED TO BE .4

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

    double whitesCount = 0;

    double whiteCountLeft = 0;
    double whiteCountRight = 0;



    //Gain to control rpm on the robot
    double rpmGain = .0000125;

    //Positions of the beacon pushers, based on whether retracting or extending
    double beaconPusherLeftRetractPosition = .05;
    double beaconPusherLeftExtendPosition = .95;
    double beaconPusherRightRetractPosition = .95;
    double beaconPusherRightExtendPosition = .05;

    boolean pushOne;
    boolean pushTwo;

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
    final double GYRO_GAIN =.0078;

    //Gain for the gyro when driving straight
    double straightGyroGain = .006;

    double straightDriveAdjust = 0;

    // F U N C T I O N S   F O R   A U T O   &   T E L E O P

    //Given a number of wheel rotations and target motor speed,
    //the robot spins clockwise at the given speed for the given
    //amount of wheel rotations
    public void encoderTurnClockwise (double rotations, double speed)
    {
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftMotor1.getCurrentPosition()<counts) {
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

    //Given a number of wheel rotations and target motor speed,
    //the robot spins counter-clockwise at the given speed for the given
    //amount of wheel rotations
    public void encoderTurnCounterClockwise (double rotations, double speed)
    {
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((Math.abs(rightMotor1.getCurrentPosition())<counts)) {
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

    //Drives straight and backwards for a provided distance, in inches
    //and at a given speed
    public void driveBack (double distance, double speed, double targetHeading)
    {
        initialHeading = gyro.getIntegratedZValue();
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();


        while (Math.abs(leftMotor1.getCurrentPosition())<counts && (timeTwo-timeOne<4)) {
            currentHeading = gyro.getIntegratedZValue();
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;
            leftMotor1.setPower(-speed+straightDriveAdjust);
            leftMotor2.setPower(-speed+straightDriveAdjust);
            rightMotor1.setPower(-speed-straightDriveAdjust);
            rightMotor2.setPower(-speed-straightDriveAdjust);
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
            timeTwo= this.getRuntime();
        }
        if (timeTwo-timeOne>4)
        {
            while (this.opModeIsActive())
            {
                stopDriving();
                timeTwo=this.getRuntime();
            }
        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
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

    public void spinMove (double desiredHeading)
    {
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initialHeading = gyro.getIntegratedZValue();

        timeOne= this.getRuntime();
        timeTwo=this.getRuntime();

        if (desiredHeading < initialHeading) //CLOCKWISE TURN
        {
            do{
                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = -headingError * GYRO_GAIN;

                if (turnSpeed < 0.2) {
                    turnSpeed = 0.2;
                }
                if (turnSpeed > .8) {
                    turnSpeed = .8;
                }

                //turnSpeed = Range.clip(turnSpeed,.12,.5);

                rightMotor1.setPower(-turnSpeed);
                rightMotor2.setPower(-turnSpeed);
                leftMotor1.setPower(turnSpeed);
                leftMotor2.setPower(turnSpeed);
                gyroTelemetry();

                timeTwo=this.getRuntime();


            }
            while (currentHeading > desiredHeading && (timeTwo-timeOne<30)); //for clockwise heading you are going to a more negatoive number
        }
        else //CCW TURN
        {
            do {

                currentHeading= gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * GYRO_GAIN;

                if (turnSpeed < 0.2) {
                    turnSpeed = 0.2;
                }
                if (turnSpeed > .8) {
                    turnSpeed = .8;
                }


                rightMotor1.setPower(turnSpeed);
                rightMotor2.setPower(turnSpeed);
                leftMotor1.setPower(-turnSpeed);
                leftMotor2.setPower(-turnSpeed);
                gyroTelemetry();
            }
            while (currentHeading < desiredHeading && (timeTwo-timeOne<30)); //for counter-clockwise heading you are going to a more negative number
        }
        if (timeTwo-timeOne>4)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }

            telemetry.addData("Timed out", telemetryVariable);
            telemetry.update();
        }
        stopDriving();
    }

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

    public void gyroTelemetry ()
    {
        telemetry.addData("Heading", gyro.getIntegratedZValue());
        telemetry.addData("Turn Speed", turnSpeed);
        telemetry.update();
    }

    //Configures all hardware devices, and sets them to their initial
    //values, if necessary
    public void Configure ()
    {
        rightMotor1 = hardwareMap.dcMotor.get("m3"); //Motor Controller 4, port 2, XV78
        rightMotor2 = hardwareMap.dcMotor.get("m4"); //Motor Controller 4, port 1, XV78
        leftMotor1 = hardwareMap.dcMotor.get("m1"); //Motor Controller 1, port 2, UVQF
        leftMotor2 = hardwareMap.dcMotor.get("m2"); //Motor Controller 1, port 1, UVQF
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);
        leftMotor1.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter = hardwareMap.dcMotor.get("m5"); //Motor Controller 2, port 1, 9PCE
        shooter.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.dcMotor.get("m6"); //Motor Controller 2, port 2, 9PCE
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        turret = hardwareMap.servo.get("s1"); //Servo Controller 1, port 2, VSI1
        hood = hardwareMap.servo.get("s2"); //Servo Controller 1, port 1, VSI1

        touchSensor = hardwareMap.touchSensor.get("touch"); //Digital port 1

        beaconPusherLeft = hardwareMap.servo.get("s4"); //Servo Controller 3, port 1, VCT7
        beaconPusherRight = hardwareMap.servo.get("s3"); //Servo Controller 3, port 2, VCT7

        cap1 = hardwareMap.dcMotor.get("m7"); //Motor Controller 3, port 1, VF7F
        cap2 = hardwareMap.dcMotor.get("m8"); //Motor Controller 3, port 2, VF7F

        cap1.setDirection(DcMotor.Direction.FORWARD);
        cap2.setDirection(DcMotor.Direction.REVERSE);

        ballControl = hardwareMap.servo.get("s5"); //Servo Controller 1, port 4, VSI1

        leftDrawbridge = hardwareMap.servo.get("s6"); //Servo Controller 3, port 3, VCT7
        rightDrawbridge = hardwareMap.servo.get("s7"); //Servo Controller 3, port 4, VCT7

        kicker = hardwareMap.servo.get("s8"); //Servo Controller 1, port 3, VSI1

        capGrab1 = hardwareMap.servo.get("s9"); //Servo Controller 3, port 5, VCT7
        capGrab2 = hardwareMap.servo.get("s10"); //Servo Controller 3, port 6, VCT7

        capGrab1.setDirection(Servo.Direction.FORWARD);
        capGrab2.setDirection(Servo.Direction.REVERSE);

        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);
        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);
        turret.setPosition(turretPosition);
        kicker.setPosition(0);
        ballControl.setPosition(.75);
        hood.setPosition(1);
        leftDrawbridge.setPosition(.5);
        rightDrawbridge.setPosition(.5);
        capGrab1.setPosition(.5);
        capGrab2.setPosition(.5);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); //I2C port 0



        whiteLineSensorRight= hardwareMap.opticalDistanceSensor.get("ods2");    //Analog import port 0
        whiteLineSensorLeft= hardwareMap.opticalDistanceSensor.get("ods1");     //Analog import port 4
        whiteLineSensorRight.enableLed(true);
        whiteLineSensorLeft.enableLed(true);

        //requires moving connection based on alliance color
        colorSensor = hardwareMap.colorSensor.get("color");     //I2C port 1
        colorSensor.enableLed(false); //

        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);
        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);

        pushOne=false;
        pushTwo=false;
        push=false;

        leftDrawbridge.setPosition(.5);
        rightDrawbridge.setPosition(.5);
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
                } else {
                    intake.setPower(0);
                }

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

    public void drive (double distance, double speed, double targetHeading)
    {
        initialHeading = gyro.getIntegratedZValue();
        inches = distance;
        rotations = distance/ (Math.PI*WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        stopDriving();

    }

    public void driveNoGyro (double distance, double speed)
    {

        inches = distance;
        rotations = distance/ (Math.PI*WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;  //math to calculate total counts robot should travel

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    public void driveToWhiteLineLeft(double speed)
    {
        whitesCount= 0;

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

            leftMotor1.setPower(speed);
            rightMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor2.setPower(speed);



        }
        while (whiteLineNotDetected && this.opModeIsActive() && (timeTwo-timeOne<4) && whitesCount<3);  //Repeat do loop until both odss have detected enough white light


        if (timeTwo-timeOne>4)
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
            if (whiteLineSensorLeft.getRawLightDetected() >= whiteThreshold) {
                wlsLeftlight = true;
                whiteCountLeft++;
            }
            if (whiteLineSensorRight.getRawLightDetected() >= whiteThreshold) {
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
                && (timeTwo-timeOne < 4)
                && whiteCountLeft<3
                && whiteCountRight<3);  //Repeat do loop until both odss have detected enough white light

        if (timeTwo-timeOne>4)
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
    public void driveToWhiteLineRight(double speed)
    {
        whitesCount = 0;

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
            if (whiteLineSensorRight.getRawLightDetected() >= whiteThreshold) {
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

            leftMotor1.setPower(speed);
            rightMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor2.setPower(speed);


        }
        while (whiteLineNotDetected && this.opModeIsActive() && (timeTwo-timeOne<4) && whitesCount<3);  //Repeat do loop until both odss have detected enough white light

        if (timeTwo-timeOne>4)
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
            if (whiteLineSensorRight.getRawLightDetected() >= whiteThreshold) {
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
            if (whiteLineSensorLeft.getRawLightDetected() >= whiteThreshold) {
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
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    public void pressBeaconSideBlue (double speed)
    {

        beaconNotDetected = true;
        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);

        do {

            telemetry.addData("Blue Value: ", colorSensor.blue());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met

            //If enough white light has been detected, set the ods boolean to true
            if (colorSensor.blue() >= 1 && colorSensor.red() < 1.5) {
                beaconNotDetected = false;
            }

//            if (speed>0)
//            {
//                leftMotor1.setPower(speed);
//                leftMotor2.setPower(speed);
//                rightMotor1.setPower(speed+.05);
//                rightMotor2.setPower(speed+.05);
//            }
//            else
//            {
//                leftMotor1.setPower(speed+ .05);
//                leftMotor2.setPower(speed + .05);
//                rightMotor1.setPower(speed);
//                rightMotor2.setPower(speed);
//            }

            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);

            timeTwo=this.getRuntime();

        } while (beaconNotDetected && this.opModeIsActive() && (timeTwo-timeOne < 4));

        stopDriving();

        if (timeTwo-timeOne>4)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 2) {
            beaconPusherLeft.setPosition(beaconPusherLeftExtendPosition);
            timeTwo = this.getRuntime();
        }

        beaconPusherLeft.setPosition(beaconPusherLeftRetractPosition);

    }

    //Run method to drive at a given speed until adequate red light is detected
    //at which time the appropriate beacon pusher extends and retracts to push
    //the beacon
    public void pressBeaconSideRed (double speed)
    {
        beaconNotDetected = true;
        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);

        do {

            telemetry.addData("Red Value: ", colorSensor.red());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met

            //If enough white light has been detected, set the ods boolean to true
            if (colorSensor.red() >= 1 && colorSensor.blue() < 1.5) {
                beaconNotDetected = false;
            }

//            if (speed>0)
//            {
//                leftMotor1.setPower(speed+.05);
//                leftMotor2.setPower(speed+.05);
//                rightMotor1.setPower(speed);
//                rightMotor2.setPower(speed);
//            }
//            else
//            {
//                leftMotor1.setPower(speed-.05);
//                leftMotor2.setPower(speed-.05);
//                rightMotor1.setPower(speed);
//                rightMotor2.setPower(speed);
//            }


            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);

            timeTwo=this.getRuntime();

        } while (beaconNotDetected && this.opModeIsActive() && (timeTwo-timeOne < 4));

        if (timeTwo-timeOne>4)
        {
            stopDriving();
            while (this.opModeIsActive())
            {
                timeTwo=this.getRuntime();
            }
        }

        stopDriving();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 2) {
            beaconPusherRight.setPosition(beaconPusherRightExtendPosition);
            timeTwo = this.getRuntime();
        }

        beaconPusherRight.setPosition(beaconPusherRightRetractPosition);

    }

}
