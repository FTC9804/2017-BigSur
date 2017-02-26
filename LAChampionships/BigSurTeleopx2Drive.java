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


@TeleOp(name = "TeleOpV7VariableGunnerIntake", group = "LA Championships")
//@Disabled
public class BigSurTeleopx2Drive extends OpMode {

    //Variable declarations

    TouchSensor beamBreak; //Touch sensor to detect whether the robot currently is possessing a particle

    DigitalChannel ledShootGreen; //led to shine green light if rpm is adequate
    DigitalChannel ledDontShootRed; //led to shine red light if rpm is not adequate
    DigitalChannel ledBallBlue; //led to shine blue light if beamBreak detects a ball in the robot

    //booleans to detect whether led lights are on, initially set to true
    boolean greenLEDIsOn = true;
    boolean redLEDIsOn = true;
    boolean blueLEDIsOn = true;

    //base capGrab servo position value, which capGrab is set to 
    //during end game
    double capGrabValue = .1;


    //Motors
    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back
    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system
    DcMotor cap1;         //cap ball lift motor
    DcMotor cap2;         //cap ball lift motor


    //Servos
    Servo turret;               //Servo to change direction of shooting
    Servo hood;                 //Servo to adjust angle of ball departure
    Servo beaconPusherLeft;     //Servo to push beacons from robot side left
    Servo beaconPusherRight;    //Servo to push beacons from robot side right
    Servo ballControl;          //Servo to prevent balls from shooting if unwanted
    Servo leftDrawbridge;       //Servo on robot side left to lift and raise intake
    Servo rightDrawbridge;      //Servo on robot side right to lift and raise intake
    Servo kicker;               //Servo to lift the last particle in a series of shots
    Servo capGrab;              //Servo to stabilize control of the cap ball during lift
    Servo leftSideWheels;       //Servo to bring down wheels on the left side of the robot
    Servo rightSideWheels;      //Servo to bring down wheels on the right side of the robot

    //ServoController involving the Servos that are not used during cap ball phase
    //and are thus deactivated during cap ball phase to save battery power
    ServoController servoControllerNoCap;

    //Variable to temporarily store the weighted average of the rpm of the shooter
    double tempWeightedAvg;

    //Gain to control rpm of shooter
    double rpmGain = .000001;

    //Target rpm of shooter to be maintained -- gotten from driver practice
    double targetRPM = 2700;

    //initial value given to the cap motors -- manipulated during teleop
    double capMotorValue = 0;

    //booleans to choose what state you are in: cap or regular mode,
    // this is a toggle for the drivers
    boolean capBallState = false;
    boolean previousStatus = false;
    boolean currentStatus = false;

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
    final double SMALL_GEAR_TEETH = 32.0;   //small gear of the turret used to drive the larger gear
    final double BIG_GEAR_TEETH = 89.0;     //large gear of the turret (lazy susan apparatus) being driven
    final double GEAR_RATIO = SMALL_GEAR_TEETH / BIG_GEAR_TEETH;    //because it is small driving large, we divide small by large

    //hood variables
    final double HOOD_INITIAL = 1;          //initial hood position all the way at the bottom
    double hoodPositionMid = .6;  //hood used for longer range shots
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
    double timeOne = 0;
    double timeTwo = 0;


    /* Initialize standard Hardware interfaces */
    public void init() { //use hardwaremap here instead of hwmap or ahwmap provided in sample code

        //configuration of the LED indicators
        //These LEDs are placed on the outside of the robot in the view of the drivers so that they accurately know
        //if the RPM is within an acceptable range
        ledDontShootRed = hardwareMap.digitalChannel.get("led1");
        ledShootGreen = hardwareMap.digitalChannel.get("led2");
        ledBallBlue = hardwareMap.digitalChannel.get("led3");
        ledDontShootRed.setMode(DigitalChannelController.Mode.OUTPUT);
        ledShootGreen.setMode(DigitalChannelController.Mode.OUTPUT);        //the LEDs will be given a logical
        ledBallBlue.setMode(DigitalChannelController.Mode.OUTPUT);          //output signal to turn on/off
        ledDontShootRed.setState(redLEDIsOn);
        ledShootGreen.setState(greenLEDIsOn);                               //LEDs are initialized to "ON"
        ledBallBlue.setState(blueLEDIsOn);


        //beamBreak initialization
        beamBreak = hardwareMap.touchSensor.get("bb");


        //motor configurations in the hardware map
        leftMotor1 = hardwareMap.dcMotor.get("m1"); //Motor Controller 1, port 2, UVQF
        leftMotor2 = hardwareMap.dcMotor.get("m2"); //Motor Controller 1, port 1, UVQF
        rightMotor1 = hardwareMap.dcMotor.get("m3"); //Motor Controller 4, port 2, XV78
        rightMotor2 = hardwareMap.dcMotor.get("m4"); //Motor Controller 4, port 1, XV78
        shooter = hardwareMap.dcMotor.get("m5"); //Motor Controller 2, port 1, 9PCE
        intake = hardwareMap.dcMotor.get("m6"); //Motor Controller 2, port 2, 9PCE
        cap1 = hardwareMap.dcMotor.get("m7"); //Motor Controller 3, port 1, VF7F
        cap2 = hardwareMap.dcMotor.get("m8"); //Motor Controller 3, port 2, VF7F


        //Stop and reset encoder, declare intention to run without encoder PID control
        cap2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //encoder count for turret at the init phase
        //initialEncoderCount = cap2.getCurrentPosition();


        //encoder counts for shooter at init phase
        encoderClicksOne = shooter.getCurrentPosition();
        encoderClicksTwo = shooter.getCurrentPosition();


        //Motor directions: set forward/reverse
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        cap1.setDirection(DcMotor.Direction.FORWARD);
        cap2.setDirection(DcMotor.Direction.FORWARD);


        //Servo configurations
        turret = hardwareMap.servo.get("s1"); //Servo Controller 1, port 2, VSI1
        hood = hardwareMap.servo.get("s2"); //Servo Controller 1, port 1, VSI1
        beaconPusherRight = hardwareMap.servo.get("s3"); //Servo Controller 3, port 1, VCT7
        beaconPusherLeft = hardwareMap.servo.get("s4"); //Servo Controller 3, port 2, VCT7
        ballControl = hardwareMap.servo.get("s5"); //Servo Controller 1, port 4, VSI1
        leftDrawbridge = hardwareMap.servo.get("s6"); //Servo Controller 3, port 3, VCT7
        rightDrawbridge = hardwareMap.servo.get("s7"); //Servo Controller 3, port 4, VCT7
        kicker = hardwareMap.servo.get("s8"); //Servo Controller 1, port 3, VSI1
        capGrab = hardwareMap.servo.get("s9"); //Servo Controller 3, Port 5, VCT7
        leftSideWheels= hardwareMap.servo.get("s11"); //Servo Controller 1, port 6, VSI1;
        rightSideWheels= hardwareMap.servo.get("s12"); //Servo Controller 1, port 5, VSI1;


        //servo controller named so we can
        servoControllerNoCap = hardwareMap.servoController.get("Servo Controller 1");
        //Servo initial positions based on individual tunings for every servo
        turret.setPosition(TURRET_INITIAL);
        hood.setPosition(HOOD_INITIAL);
        beaconPusherRight.setPosition(1);
        beaconPusherLeft.setPosition(0);
        ballControl.setPosition(0);
        leftDrawbridge.setPosition(.5); //drawbridges and side wheels are continuous rotation servos,
        rightDrawbridge.setPosition(.5);// so we init them to 0.5 so they do not immediately start moving
        leftSideWheels.setPosition(.5);
        rightSideWheels.setPosition(.5);
        kicker.setPosition(0);
        capGrab.setDirection(Servo.Direction.FORWARD);
        capGrab.setPosition(1);


        //Shooter initially running at shooterSpeed power
        shooter.setPower(shooterSpeed);


    }


    @Override
    public void loop() {

        //code for toggling the use of the cap ball mode.
        // To prevent accidental activation during Start-A and Start-B,
        // the gunner has to hold down both the start and back buttons.
        // This has the added benefit of forcing the gunner to be very
        // intentional in his application of the mode
        previousStatus = currentStatus;
        currentStatus = (gamepad2.start && gamepad2.back);

        //switching cap ball state
        if (currentStatus && !previousStatus)
        {
            capBallState=!capBallState;
        }

        if (!capBallState) {

            //we disable this pwm during cap ball mode, so we always make sure
            // it is enabled during the standard mode
            servoControllerNoCap.pwmEnable();



            //*****************
            // S H O O T I N G **
            //*****************

            //Current Run Time
            timeTwo = this.getRuntime();
            //Current Encoder Clicks
            encoderClicksTwo = shooter.getCurrentPosition();

            //Telemetry for time variables
            telemetry.addData("Time Two", timeTwo);
            telemetry.addData("Time Difference", timeTwo - timeOne);

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

            //telemetry for rpm: we use Avg RPM for calculations to RPM proportional control,
            // but we also want to show the cycle by cycle RPM for troubleshooting
            telemetry.addData("RPM : ", rpm);
            telemetry.addData("AvgRPM : ", avgRpm);



            //***************************************************
            //E L E V A T O R  L O A D E R  &  I N T A K E **
            //***************************************************

            //set drawbridge servo powers: CR servos so we default to .5 and then move
            if (gamepad2.x) {           //raise intake using drawbridge
                rightDrawbridge.setPosition(.95);
                leftDrawbridge.setPosition(.05);
            } else if (gamepad2.b) {    //lowers intake using drawbridge
                leftDrawbridge.setPosition(.95);
                rightDrawbridge.setPosition(.05);
            } else {
                leftDrawbridge.setPosition(.5);
                rightDrawbridge.setPosition(.5);
            }

            //set intake speeds
            //Triggers act like an axis from 0->1, so to make them behave like buttons
            // we set them to active when they reach beyond a certain value: .6.  If the right trigger's
            // value is above .6, activate elevator (going up) and intake (coming in)
            if (gamepad2.right_trigger > 0.6) {
                intakeSpeed = 1.0;
            } else if (gamepad2.left_trigger > .6) {
                intakeSpeed = -1;
            } else {
                intakeSpeed = 0;
            }

            //Ball control positions: tuned from individual testing
            //Meant to always be blocking until gunner allows clear passage of particles
            if (gamepad2.left_bumper) {
                ballControl.setPosition(.24);
            } else {
                ballControl.setPosition(0);
            }

            //Kicker positions: tuned from individual testing
            //Meant to always be down until gunner enables kicker to shoot last particle
            if (gamepad2.right_bumper) {
                kicker.setPosition(.72);
            } else {
                kicker.setPosition(0);
            }



            //******************
            //S E T  P O W E R S @@
            //*****************/


            //The ability to adjust RPM is essential so that we don't
            if (gamepad2.dpad_down) {
                targetRPM -= 1;
            }
            else if (gamepad2.dpad_up) {
                targetRPM += 1;
            }

            //display RPM to phones so that the drive coach can give the shooting command
            telemetry.addData("Target RPM = ", targetRPM);

            //proportional control to correct shooting power based on the current error
            shooterSpeed += rpmGain * (targetRPM - avgRpm);

            //clip the range from .1->1 because we don't want the motor to go to 0 until the cap ball mode
            shooterSpeed = Range.clip(shooterSpeed, 0.1, 1);

            //set shooter and intake power
            shooter.setPower(shooterSpeed);
            intake.setPower(intakeSpeed);

            //show relevant values for shooting: power, gain, hood angle
            telemetry.addData("Shooter Motor Power: ", shooterSpeed);
            telemetry.addData("RPM Gain", rpmGain);
            telemetry.addData("Hood pos", hood.getPosition());



            //***************************************************
            // L E D   N O T I F I C A T I O N S
            //***************************************************

            //LED Notifications for shooting

            //if the RPM is within +/- 100 of target, we want the green light to tell drivers to shoot
            if (avgRpm < (targetRPM + 100) && avgRpm > (targetRPM - 100)) {
                ledDontShootRed.setState(!redLEDIsOn);
                ledShootGreen.setState(greenLEDIsOn);
            } else {
                ledDontShootRed.setState(redLEDIsOn);
                ledShootGreen.setState(!greenLEDIsOn);
            }

            //blue LED activated when our beam break sensor (acting as a touch sensor) is
            // tripped to show a ball is present
            if (beamBreak.isPressed()) {
                telemetry.addLine("No Ball");
                ledBallBlue.setState(!blueLEDIsOn);
            } else {
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
            hoodPositioning = Range.clip(hoodPositioning, 0.6, 1);

            //Set the position of the hood to hoodPositioning
            hood.setPosition(hoodPositioning);
            telemetry.addData("hood", hoodPositioning);


        }



        //*****************
        // T U R R E T  @@
        //*****************

        //we want turret control throughout the match regardless of state
        // we want to have the turret for shooting and for moving out of the way of cap ball mechanism

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

        //set position of turret
        turret.setPosition(turretRotationValue);



        //*****************
        // D R I V I N G  @@
        //*****************

        //assign joystick values
        joystick1ValueLeft = gamepad1.left_stick_y; //set joystick1ValueLeft to the raw value of gamepad1.left_stick_y
        joystick1ValueRight = gamepad1.right_stick_y; //set joystick1ValueRight to the raw value of gamepad1.right_stick_y

        //we use x^2 adjustment for code, and we cut down by 95% so that we don't overdo the motor control
        leftMotor1.setPower(.95 * Math.abs(joystick1ValueLeft) * joystick1ValueLeft);
        leftMotor2.setPower(.95 * Math.abs(joystick1ValueLeft) * joystick1ValueLeft);
        rightMotor1.setPower(.95 * Math.abs(joystick1ValueRight) * joystick1ValueRight);
        rightMotor2.setPower(.95 * Math.abs(joystick1ValueRight) * joystick1ValueRight);



        //*****************
        // B E A C O N @@
        //*****************

        //Move both port side and battery side beacons based on actions on the driver gamepad
        //.1 and .9  for testing
        //default is that they stay inside before being pushed outwards
        if (gamepad1.x) {
            beaconPusherLeftPosition = .9;
        } else {
            beaconPusherLeftPosition = .1;
        }
        if (gamepad1.b) {
            beaconPusherRightPosition = .1;
        } else {
            beaconPusherRightPosition = .9;
        }

        //set the position of each beacon to its respective position determined above
        beaconPusherLeft.setPosition(beaconPusherLeftPosition);
        beaconPusherRight.setPosition(beaconPusherRightPosition);



        //*****************
        // C A P   B A L L @@
        //*****************

        //cap ball is only operational when the state is activated so that we do not
        // entangle with other robots.  we also have specific power allocations during this time, in order 
        // to conserve power to operate our continuous lift system

        if (capBallState) {

            //disable whole servo controller that is not used during the cap ball period
            servoControllerNoCap.pwmDisable();

            //shooter and intake are cut to 0 power to save power
            shooter.setPower(0);
            intake.setPower(0);

            //cap motor is controlled by the gunner joystick for fine control movement
            capMotorValue = gamepad2.right_stick_y;
            cap1.setPower(capMotorValue);
            cap2.setPower(capMotorValue);

            //used to secure the cap ball on the rising mechanism to prevent slipping out
            if (gamepad2.left_bumper) {
                capGrabValue-=.04;
            }
            else {
                capGrabValue+=.04*gamepad2.left_trigger;
            }

            //clipping servo to relevant and tuned values
            capGrabValue = Range.clip(capGrabValue, 0.08, .95);

            //telemetry to show drivers the value of the servo
            telemetry.addData("Cap Grab Servo Value", capGrabValue);

            //set the servo position for holding cap ball
            capGrab.setPosition(capGrabValue);

        }
        
        //sending telemetry to the phones at the end of every cycle so that all
        // information is available at the same time. This is the only time in the
        // looping code that it is updated so the drivers can see everything at once
        telemetry.update(); //update telemetry

    }


}
