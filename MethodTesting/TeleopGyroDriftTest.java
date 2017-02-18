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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


@TeleOp(name = "TeleopGyroDriftTest", group = "LA Championships")
//@Disabled
public class TeleopGyroDriftTest extends OpMode {

    //Variable declarations

    TouchSensor beamBreak; //Touch sensor to detect whether the robot currently is possessing a particle

    DigitalChannel ledShootGreen; //led to shine green light if rpm is adequate
    DigitalChannel ledDontShootRed; //led to shine red light if rpm is not adequate
    DigitalChannel ledBallBlue; //led to shine blue light if beamBreak detects a ball in the robot

    //booleans to detect whether led lights are on
    boolean greenLEDIsOn = true;
    boolean redLEDIsOn = true;
    boolean blueLEDIsOn = true;

    ModernRoboticsI2cGyro gyro;


    //Motors
    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back
    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system
    DcMotor cap1;
    DcMotor cap2;


    //Servos
    Servo turret;
    Servo hood;
    Servo beaconPusherLeft;
    Servo beaconPusherRight;
    Servo ballControl;
    Servo leftDrawbridge;
    Servo rightDrawbridge;
    Servo kicker;
    Servo capGrab1;
    Servo capGrab2;

    ServoController servoControllerNoCap;

    //Variable to temporarily store the weighted average of the rpm of the shooter
    double tempWeightedAvg;

    //Gain to control rpm of shooter
    double rpmGain = .000000125;
    double desiredRPMGain = .000000125;
    double rpmGainExtremeValuesChange = .00000025;
    double rpmGainCloseValuesChange = .000000125;
    double rpmGainFarSteady = .000000125;
    double rpmGainMidSteady = .00000015;
    double rpmGainNearSteady = .000000175;


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

    double capMotorValue = 0;

    boolean capBallState = false;

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
    final double GEAR_RATIO = SMALL_GEAR_TEETH / BIG_GEAR_TEETH;

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

    double capGrabValueLeft = .5;
    double capGrabValueRight = .5;


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
    double timeOne = 0;
    double timeTwo = 0;


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
        beamBreak = hardwareMap.touchSensor.get("bb"); //Digital port 0

        //motor configurations in the hardware map
        rightMotor1 = hardwareMap.dcMotor.get("m3");//port 1 on robot and in the hardwaremap
        rightMotor2 = hardwareMap.dcMotor.get("m4");//port 2
        leftMotor1 = hardwareMap.dcMotor.get("m1");
        leftMotor2 = hardwareMap.dcMotor.get("m2");
        shooter = hardwareMap.dcMotor.get("m5");
        intake = hardwareMap.dcMotor.get("m6");
        cap1 = hardwareMap.dcMotor.get("m7");
        cap2 = hardwareMap.dcMotor.get("m8");


        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); //I2C port 0

        //Stop and reset encoder, declare intention to run using encoder
        cap2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //encoder count for turret at the init phase
        //initialEncoderCount = cap2.getCurrentPosition();

        //encoder counts for shooter at init phase
        encoderClicksOne = shooter.getCurrentPosition();
        encoderClicksTwo = shooter.getCurrentPosition();

        //Motor directions
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        cap1.setDirection(DcMotor.Direction.FORWARD);
        cap2.setDirection(DcMotor.Direction.REVERSE);



        //Servo configurations
        turret = hardwareMap.servo.get("s1");
        hood = hardwareMap.servo.get("s2");
        beaconPusherRight = hardwareMap.servo.get("s3");
        beaconPusherLeft = hardwareMap.servo.get("s4");
        ballControl = hardwareMap.servo.get("s5");
        leftDrawbridge = hardwareMap.servo.get("s6");
        rightDrawbridge = hardwareMap.servo.get("s7");
        kicker = hardwareMap.servo.get("s8");
        capGrab1 = hardwareMap.servo.get("s9");
        capGrab2 = hardwareMap.servo.get("s10");

        capGrab1.setDirection(Servo.Direction.FORWARD);
        capGrab2.setDirection(Servo.Direction.REVERSE);

        servoControllerNoCap = hardwareMap.servoController.get("Servo Controller 1");

        //Servo initial positions
        turret.setPosition(TURRET_INITIAL);
        hood.setPosition(HOOD_INITIAL);
        beaconPusherRight.setPosition(1);
        beaconPusherLeft.setPosition(0);
        ballControl.setPosition(0);
        leftDrawbridge.setPosition(.5);
        rightDrawbridge.setPosition(.5);
        kicker.setPosition(0);
        capGrab1.setPosition(.5);
        capGrab2.setPosition(.5);
    }


    @Override
    public void loop() {
        telemetry.addData("Gyro heading", gyro.getIntegratedZValue());
        if (gamepad1.y) {
            gyro.resetZAxisIntegrator();
        }
    }



}
