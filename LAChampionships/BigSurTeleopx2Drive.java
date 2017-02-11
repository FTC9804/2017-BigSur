//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
// Designed to run the teleop phase of the match, and to
//allow our drivers and coaches to shoot balls into the
//center vortex and press beacons.

//package declaration
package org.firstinspires.ftc.teamcode;

//import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "TeleOpV7Variable", group = "LA Championships")
//@Disabled
public class BigSurTeleopx2Drive extends OpMode {

    //Variable declarations

    TouchSensor beamBreak; //Touch sensor to detect whether the robot currently is possessing a particle

    DigitalChannel ledShootGreen; //led to shine green light if rpm is adequate
    DigitalChannel ledDontShootRed; //led to shine red light if rpm is not adequate
    DigitalChannel ledBallBlue; //led to shine blue light if beamBreak detects a ball in the robot

    //booleans to detect whether led lights are on
    boolean greenLEDIsOn = true;
    boolean redLEDIsOn = true;
    boolean blueLEDIsOn = true;


    //Motors
    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back
    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system
    DcMotor encode; //Hypothetical Motor

    //Servos
    Servo turret;
    Servo hood;
    Servo beaconPusherLeft;
    Servo beaconPusherRight;
    Servo ballControl;
    Servo leftDrawbridge;
    Servo rightDrawbridge;
    Servo kicker;

    //Variable to temporarily store the weighted average of the rpm of the shooter
    double tempWeightedAvg;

    //Gain to control rpm of shooter
    double rpmGain =                    .000000125;
    double desiredRPMGain =             .000000125;
    double rpmGainExtremeValuesChange = .00000025;
    double rpmGainCloseValuesChange =   .000000125;
    double rpmGainFarSteady =               .000000125;
    double rpmGainMidSteady =               .00000015;
    double rpmGainNearSteady =              .000000175;


    //Target rpm of shooter to be maintained
    double shootRPMFar = 3000;
    double shootRPMMid = 2700;
    double shootRPMNear = 2500;
    double targetRPM = shootRPMMid;
    double deltaRPM;
    boolean farModeEngaged = false;
    boolean midModeEngaged = true;
    boolean nearModeEngaged = false;

    double pastShootingRPM;


    //Positions relating to the beacon pushers
    double beaconPusherRightPosition;
    double beaconPusherLeftPosition;

    //Turret variables
    double turretAxis;          //value taken in by left turret joystick.
    final double TURRET_INITIAL = 0.5;          //position of turret not moving
    double turretRotationValue;       //value given to turret joystick
    final double TURRET_ROTATION_LEFT = 0.2;      //speed value of joystick moving to the left
    final double TURRET_ROTATION_RIGHT = 0.8;  //speed value of joystick moving to the right

    //shooter variables;
    double shooterSpeed = 0.95;    //Power applied to shooter in init method

    //intake variables
    double intakeSpeed;          //power given to the intake system

    //values for the gears of the turret
    final double SMALL_GEAR_TEETH = 32.0;
    final double BIG_GEAR_TEETH = 89.0;
    final double GEAR_RATIO = SMALL_GEAR_TEETH/BIG_GEAR_TEETH;

    //hood variables
    final double HOOD_INITIAL = 1;          //initial hood position all the way at the bottom
    double hoodPositionFar = .25;
    double hoodPositionMid = .125;
    double hoodPositionNear = 1;
    double hoodPositioning = hoodPositionMid;  //hood positioning initially set

    double joystick1ValueLeft;  //the raw value taken from the left joystick
    double joystick1ValueRight;  //the raw value taken from the right joystick

    int initialEncoderCount;  //the number of counts of the encoder on the turret initially
    int currentEncoderPosition; //the current number of counts of the encoder on the turret

    double countsPerRotation = 1440.0; //counts of the turret encoder per revolution

    double countDelta; //the absolute value of the difference between currentEncoderPosition and initialEncoderCount

    double rotations; //the amount of 360 degree rotations the turret has completed since the beginning of the match


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

    //Run time variables
    double timeOne=0;
    double timeTwo=0;



    /* Initialize standard Hardware interfaces */
    public void init() { //use hardwaremap here instead of hwmap or ahwmap provided in sample code

        //configuration of the LED indicators
        //These LEDs are placed on the outside of the robot in the view of the drivers so that they accurately know
        //if the RPM is within an acceptable range
        ledShootGreen = hardwareMap.digitalChannel.get("led2");
        ledDontShootRed = hardwareMap.digitalChannel.get("led1");
        ledBallBlue = hardwareMap.digitalChannel.get("led3");
        ledShootGreen.setMode(DigitalChannelController.Mode.OUTPUT);        //the LEDs will be given a logical
        ledDontShootRed.setMode(DigitalChannelController.Mode.OUTPUT);       //output signal to turn on/off
        ledBallBlue.setMode(DigitalChannelController.Mode.OUTPUT);
        ledShootGreen.setState(greenLEDIsOn);          //LEDs are initialized to "ON"
        ledDontShootRed.setState(redLEDIsOn);
        ledBallBlue.setState(blueLEDIsOn);

        //beamBreak initialization
        beamBreak = hardwareMap.touchSensor.get("bb");

        //motor configurations in the hardware map
        rightMotor1 = hardwareMap.dcMotor.get("m3");//port 1 on robot and in the hardwaremap
        rightMotor2 = hardwareMap.dcMotor.get("m4");//port 2
        leftMotor1 = hardwareMap.dcMotor.get("m1");
        leftMotor2 = hardwareMap.dcMotor.get("m2");
        shooter = hardwareMap.dcMotor.get("m5");
        intake = hardwareMap.dcMotor.get("m6");
        encode = hardwareMap.dcMotor.get("m8");

        //Stop and reset encoder, declare intention to run using encoder
        encode.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encode.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //encoder count for turret at the init phase
        initialEncoderCount = encode.getCurrentPosition();

        //encoder counts for shooter at init phase
        encoderClicksOne=shooter.getCurrentPosition();
        encoderClicksTwo=shooter.getCurrentPosition();

        //Motor directions
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        //Servo configurations
        turret = hardwareMap.servo.get("s1");
        hood = hardwareMap.servo.get("s2");
        beaconPusherRight = hardwareMap.servo.get("s3");
        beaconPusherLeft = hardwareMap.servo.get("s4");
        ballControl = hardwareMap.servo.get("s5");
        leftDrawbridge = hardwareMap.servo.get("s6");
        rightDrawbridge = hardwareMap.servo.get("s7");
        kicker = hardwareMap.servo.get("s8");


        //Servo initial positions
        turret.setPosition(TURRET_INITIAL);
        hood.setPosition(HOOD_INITIAL);
        beaconPusherRight.setPosition(1);
        beaconPusherLeft.setPosition(0);
        ballControl.setPosition(0);
        leftDrawbridge.setPosition(.5);
        rightDrawbridge.setPosition(.5);
        kicker.setPosition(0);

        //Shooter initially running at shooterSpeed power
        shooter.setPower(shooterSpeed);
    }


    @Override
    public void loop() {

        //assign joystick values
        joystick1ValueLeft= gamepad1.left_stick_y; //set joystick1ValueLeft to the raw value of gamepad1.left_stick_y
        joystick1ValueRight = gamepad1.right_stick_y; //set joystick1ValueRight to the raw value of gamepad1.right_stick_y

        //Set motor powers, multiplying by .95
        leftMotor1.setPower(.95*joystick1ValueLeft);
        leftMotor2.setPower(.95*joystick1ValueLeft);
        rightMotor1.setPower(.95*joystick1ValueRight);
        rightMotor2.setPower(.95*joystick1ValueRight);

        //*****************
        // T U R R E T  @@
        //*****************

        //set turretAxis to the raw value of gamepad2.left_stick_x
        turretAxis = gamepad2.left_stick_x;

        //Assign turretRotationValues based on the value of turretAxis.  If turretAxis is significantly positive, rotate to the left.
        //If turretAxis is significantly negative, rotate to the right.  If neither, keep the turret still by setting its position to .5
        if (turretAxis < -0.1) {  //0.1 to set off the dead zone. FUTURE -> set gain for precise adjustments
            turretRotationValue = TURRET_ROTATION_RIGHT;  //declared above for easy editing
        } else if (turretAxis > 0.1) {
            turretRotationValue = TURRET_ROTATION_LEFT;
        } else {
            turretRotationValue = 0.5;
        }

        //*****************
        // S H O O T I N G **
        //*****************

        //Current Run Time
        timeTwo = this.getRuntime ();
        //Current Encoder Clicks
        encoderClicksTwo = shooter.getCurrentPosition();

        //Telemetry for time variables
        telemetry.addData("Time Two", timeTwo);
        telemetry.addData("Time Difference", timeTwo-timeOne);

        //telemetry for shooting speed
        if (timeTwo - timeOne >= 0.1)
        {//if timeTwo and timeOne are more than .1 sec apart
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
            avgRpm = totalRpm / 5; //set avgRpm to totalRpm divided by five casted as an int
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

        //telemetry for rpm
        telemetry.addData("RPM : ", rpm);
        telemetry.addData("AvgRPM : ", avgRpm);

        //***************************************************
        //E L E V A T O R  L O A D E R  &  I N T A K E **
        //***************************************************

        //set intake servo powers
        if (gamepad2.x)
        {
            rightDrawbridge.setPosition(.95);
            leftDrawbridge.setPosition(.05);
        }
        else if (gamepad2.b)
        {
            leftDrawbridge.setPosition(.95);
            rightDrawbridge.setPosition(.05);
        }
        else {
            leftDrawbridge.setPosition(.5);
            rightDrawbridge.setPosition(.5);
        }

        //set intake speeds
        if (gamepad2.right_trigger > 0.6) { //triggers act like an axis, so to make them behave like buttons
            // we set them to active when they reach beyond a certain value: .6.  If the right trigger's
            //value is above .6, activate elevator (going up) and intake (coming in)
            intakeSpeed = 1.0;
        }
        else if (gamepad2.left_trigger>.6)
        {
            intakeSpeed=-1;
        }
        else
        {
            intakeSpeed=0;
        }

        //Ball control positions
        if (gamepad2.left_bumper)
        {
            ballControl.setPosition(.95);
        }
        else
        {
            ballControl.setPosition(0);
        }

        //Kicker positions
        if (gamepad2.right_bumper)
        {
            kicker.setPosition(.95);
        }
        else
        {
            kicker.setPosition(0);
        }

        /******************
        //SET ALL POWERS @@
        //*****************/


//        //Set the elevator and intake's speed to the values specified above
//        if (gamepad2.dpad_up) {
//            pastShootingRPM = targetRPM;
//            targetRPM = shootRPMFar;
//            hoodPositioning = hoodPositionFar;
//            farModeEngaged = true;
//            midModeEngaged = false;
//            nearModeEngaged = false;
//        }
//        else if (gamepad2.dpad_left) {
//            pastShootingRPM = targetRPM;
//            targetRPM = shootRPMMid;
//            hoodPositioning = hoodPositionMid;
//            farModeEngaged = false;
//            midModeEngaged = true;
//            nearModeEngaged = false;
//        }
//        else if (gamepad2.dpad_down) {
//            pastShootingRPM = targetRPM;
//            targetRPM = shootRPMNear;
//            hoodPositioning = hoodPositionNear;
//            farModeEngaged = false;
//            midModeEngaged = false;
//            nearModeEngaged = true;
//        }
//
//
//        if (pastShootingRPM != targetRPM){
//            if (pastShootingRPM == shootRPMNear && targetRPM == shootRPMMid) {
//                rpmGain = rpmGainCloseValuesChange;
//                telemetry.addLine("RPM Close Change");
//            }
//            if (pastShootingRPM == shootRPMNear && targetRPM == shootRPMFar) {
//                rpmGain = rpmGainExtremeValuesChange;
//                telemetry.addLine("RPM Extreme Change");
//            }
//            if (pastShootingRPM == shootRPMMid && targetRPM == shootRPMNear) {
//                rpmGain = rpmGainCloseValuesChange;
//                telemetry.addLine("RPM Close Change");
//            }
//            if (pastShootingRPM == shootRPMMid && targetRPM == shootRPMFar) {
//                rpmGain = rpmGainCloseValuesChange;
//                telemetry.addLine("RPM Close Change");
//            }
//            if (pastShootingRPM == shootRPMFar && targetRPM == shootRPMNear) {
//                rpmGain = rpmGainExtremeValuesChange;
//                telemetry.addLine("RPM Extreme Change");
//            }
//            if (pastShootingRPM == shootRPMFar && targetRPM == shootRPMMid) {
//                rpmGain = rpmGainCloseValuesChange;
//                telemetry.addLine("RPM Close Change");
//            }
//        }
//
//        if (avgRpm < targetRPM + 50 && avgRpm > targetRPM - 50) {
//            pastShootingRPM = targetRPM;
//
//            if (farModeEngaged) {
//                rpmGain = rpmGainFarSteady;
//                telemetry.addLine("RPM Far Steady");
//            }
//            if (midModeEngaged) {
//                rpmGain = rpmGainMidSteady;
//                telemetry.addLine("RPM Mid Steady");
//            }
//            if (nearModeEngaged) {
//                rpmGain = rpmGainNearSteady;
//                telemetry.addLine("RPM Near Steady");
//            }
//
//        }

        if (gamepad2.dpad_up) {
            rpmGain += rpmGain * .1;
        } else if (gamepad2.dpad_down) {
            rpmGain -= rpmGain * .1;
        } else if (gamepad2.dpad_right) {
            rpmGain += rpmGain * .25;
        } else if (gamepad2.dpad_left) {
            rpmGain -= rpmGain * .25;
        }



//        //linear adjustment of RPM Gain
//        if (gamepad2.dpad_up) {
//            targetRPM = shootRPMFar;
//            hoodPositioning = hoodPositionFar;
//        }
//        else if (gamepad2.dpad_left) {
//            targetRPM = shootRPMMid;
//            hoodPositioning = hoodPositionMid;
//        }
//        else if (gamepad2.dpad_down) {
//            targetRPM = shootRPMNear;
//            hoodPositioning = hoodPositionNear;
//        }
//
//        deltaRPM = Math.abs(avgRpm - targetRPM);
//        rpmGain = (desiredRPMGain/150)*deltaRPM + desiredRPMGain;

        shooterSpeed += rpmGain * (targetRPM-avgRpm);

        shooterSpeed = Range.clip(shooterSpeed,0,1);

        shooter.setPower(shooterSpeed);

        intake.setPower(intakeSpeed);
        telemetry.addData("Shooter Motor Power: ", shooterSpeed);
        telemetry.addData("RPM Gain", rpmGain);

        //***************************************************
        // L E D   N O T I F I C A T I O N S
        //***************************************************

        //LED Notifications
        if (avgRpm < (targetRPM +100)  && avgRpm > (targetRPM -100)) {
            ledDontShootRed.setState(!redLEDIsOn);
            ledShootGreen.setState(greenLEDIsOn);
        }
        else {
            ledDontShootRed.setState(redLEDIsOn);
            ledShootGreen.setState(!greenLEDIsOn);
        }
        if (beamBreak.isPressed()) {
            telemetry.addLine("No Ball");
            ledBallBlue.setState(!blueLEDIsOn);
        }
        else {
            telemetry.addLine("Ball");
            ledBallBlue.setState(blueLEDIsOn);
        }

        //*****************
        // H O O D @@
        //*****************

        //increase/decrease hood positioning at a slow rate to allow fine tune adjustment because the code will cycle approx. 100-150 times a second

        if (gamepad2.y) { //If y is being pressed, move the hood down
            hoodPositioning -= .005;
        }
        if (gamepad2.a) { //If a is being pressed, move the hood up
            hoodPositioning += .005;
        }
        //If the hoodPositioning is out of the possible servo range limits (both logically and physically), correct the values to
        //ensure the hoodPositioning value is in the correct range
        hoodPositioning = Range.clip(hoodPositioning, 0.04, 1);

        //Set the position of the hood to hoodPositioning
        hood.setPosition(hoodPositioning);
        telemetry.addData("hood" , hoodPositioning);

        //*****************
        // B E A C O N @@
        //*****************

        //Move both port side and battery side beacons based on actions on the driver gamepad
        //.1 and .9  for testing
        if (gamepad1.x) {
            beaconPusherLeftPosition = .9;
        }
        else {
            beaconPusherLeftPosition = .1;
        }

        if (gamepad1.b) {
            beaconPusherRightPosition = .1;
        }
        else
        {
            beaconPusherRightPosition = .9;
        }

        //set the position of each beacon to its respective position determined above
        beaconPusherLeft.setPosition(beaconPusherLeftPosition);
        beaconPusherRight.setPosition(beaconPusherRightPosition);

        telemetry.update(); //update telemetry
    }
}
