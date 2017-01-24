/*Code written by Steve on Friday Nov. 18, 2016.
* edited by Steve and Marcus on Saturday Nov. 19, 2016
* edited by Steve and Etienne on Saturday Nov. 26, 2016
* edited by Marcus Sunday Nov. 27, 2016
* edited by Marcus Wednesday Nov. 30, 2916
* edited by Etienne Lunetta Thursday Dec. 1, 2016
* code examined by Steve and Etienne on Friday 2 Dec. 2016
* reedited Etienne Lunetta Sunday December 4 @ 2:01 AM
* edited by steve and marcus on friday january 13 @ 5:50 in the afternoon
*
*  Purpose: Preliminary coding of the In N Out Project to create initial robot movement
*
*
* FTC Team 9804 Bomb Squad
* Made by the programmers of FTC Team 9804 Bomb Squad
*
* Version 1: logarithmic driving, turreting, shooting, intake, hood, storage
* Version 2: Increased comments to show extent of code and additional gain options commented in for future use.
* Version 3: Adjust forward/reverse setting and hood positioning to reflect physical attributes of the robot.
* Version 4: Add more comments and ensure consistency and that java conventions are being adhered to.  Also adjust
* the accepted range of hood servo values to ensure that servos do not get burned out.
* Version 5: Add set rpms and hood angles for specific shot ranges, letting the gunner control these functions using the dpad.
* Version 6.0: Fixed errors in past code regarding the elevator and turret and multiple speed declarations, deleted unnecessary spaces
* version 7.0: fixed errors resulting from our last tournament and testing
*
*
*/


/*
   Tracking Shortcuts:
   ** = check again
   @@ = all good
*/




package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "TeleOpV7", group = "InNOut Testing")
//@Disabled
public class BigSurTeleopx2Drive extends OpMode {




    //motors declared
    DcMotor rightMotor1;   //right drive motor front
    DcMotor leftMotor1;    //left drive motor front
    DcMotor rightMotor2;   //right drive motor back
    DcMotor leftMotor2;    //left drive motor back
    DcMotor shooter;      //shooting flywheel
    DcMotor intake;       //intake system
    DcMotor elevator;     //elevator loading system




    DcMotor encode; //Hypothetical Motor


    double tempWeightedAvg;

    double rpmGain = .0000001;


    int mode;




    //servos declared
    Servo turret;       //continuous rotation servo, adjust direction of shooter
    Servo hood;       //position servo for 180º, adjust angle of shooter
    Servo batterySideBeacon;
    Servo portSideBeacon;




    double batterySideBeaconPosition;
    double portSideBeaconPosition;








    //Driving variables
    double joystick1ValueLeftYAxis;       //value taken in by left driving joystick
    double joystick1ValueRightYAxis;   //value taken in by right driving joystick
    double leftPower;           //power given to left motors
    double rightPower;       //power given to right motors




    double gain = 1;
    //drive gain to multiply the power by
    //toggle variables
    boolean halfGain = false;
//    boolean previousStatus = false;
//    boolean currentStatus = false;




    //Turret variables
    double turretAxis;          //value taken in by left turreting joystick.
    final double TURRET_INITIAL = 0.5;          //position of turret not moving
    double turretRotationValue;       //value given to turret joystick
    final double TURRET_ROTATION_LEFT = 0.2;      //speed value of joystick moving to the left
    final double TURRET_ROTATION_RIGHT = 0.8;  //speed value of joystick moving to the right




    //shooter variables;
    double shooterSpeed = 0.235;    // 0.23 is for 1850 RPM, 0.24 is for 2500 RPM, 0.235 is for 2100
    double elevatorSpeed;        //power given to the loading elevator




    //intake variables
    double intakeSpeed;          //power given to the intake system




    //values for the gears of the turret
    final double SMALL_GEAR_TEETH = 16.0;
    final double BIG_GEAR_TEETH = 72.0;
    final double GEAR_RATIO = SMALL_GEAR_TEETH/BIG_GEAR_TEETH;




    //hood variables
    final double HOOD_INITIAL = .12;          //initial hood position all the way at the bottom
    double hoodPositioning;  //hood positioning initially set




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




    double timeOne=0;




    double timeTwo=0;








    //Standard rpms and hood angles for various shots
    //Hood angle and rpm for near side short range shot, about 20 inches (from flywheel to center of goal)
    double twentyInchHoodAngle = .09;
    double twentyInchRPM = 2550;
    //Hood angle and rpm for near side mid range shot, about 39 inches
    double thirtyNineInchHoodAngle = .128;
    double thirtyNineInchRPM = 2720;
    //Hood angle and rpm for near side far range shot, about 54 inches
    double fiftyFourInchHoodAngle = .176;
    double fiftyFourInchRPM=2800;
    //Hood angle and rpm for far side far range shot, about 88 inches
    double eightyEightInchHoodAngle = .211;
    double eightyEightInchRPM = 3250;




    double driveGain = .000065;//gain for proportional driving NOT TRUE EDITS NEED TO BE MADE



    //boolean shooterOff=false;








    /* Initialize standard Hardware interfaces */
    public void init() { //use hardwaremap here instead of hwmap or ahwmap provided in sample code








        //motor configurations in the hardware map
        rightMotor1 = hardwareMap.dcMotor.get("m3");//port 1 on robot and in the hardwaremap
        rightMotor2 = hardwareMap.dcMotor.get("m4");//port 2
        leftMotor1 = hardwareMap.dcMotor.get("m1");
        leftMotor2 = hardwareMap.dcMotor.get("m2");
        shooter = hardwareMap.dcMotor.get("m5");
        intake = hardwareMap.dcMotor.get("m6");
        elevator = hardwareMap.dcMotor.get("m7");
        encode = hardwareMap.dcMotor.get("m8");








        //Stop and reset encoder, declare intention to run using encoder
        encode.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encode.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        //encoder count for turret at the init phase
        initialEncoderCount = encode.getCurrentPosition();




        //encoder counts for shooter at init phase
        encoderClicksOne=shooter.getCurrentPosition();
        encoderClicksTwo=shooter.getCurrentPosition();




        // right side set to FORWARD, left set to REVERSE using our custom west coast driving gearbox
        // Set shooter, intake an elevator to FORWARD, REVERSE, and FORWARD, respectively based on their orientations on the robot

        //WE GIVE RIGHT SIDE FORWARD VALUES AND LEFT REVERSE VALUES BECAUSE JOYSTICK VALUES (UP/DOWN) ARE OPPOSITE (UP-> -1; DOWN -> 1)

        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD);






        //Servo configurations
        turret = hardwareMap.servo.get("s1");
        hood = hardwareMap.servo.get("s2");
        batterySideBeacon = hardwareMap.servo.get("s3");
        portSideBeacon = hardwareMap.servo.get("s4");




        //Servo initial positions
        turret.setPosition(TURRET_INITIAL);
        hood.setPosition(HOOD_INITIAL);
        batterySideBeacon.setPosition(.5);
        portSideBeacon.setPosition(.5);




        //Shooter initially running at shooterSpeed power
        shooter.setPower(shooterSpeed);

    }




    @Override
    public void loop() {




        //***************************************************
        //  A S S I G N  J O Y S T I C K  V A L U E S @@
        //***************************************************

        //maybe change
        joystick1ValueLeft= gamepad1.left_stick_y; //set joystick1ValueLeft to the raw value of gamepad1.left_stick_y
        joystick1ValueRight = gamepad1.right_stick_y; //set joystick1ValueRight to the raw value of gamepad1.right_stick_y


        //toggle gain


        // Understood dysfunctional toggle code and rewrote with 1 boolean as opposed to 3 to achieve the same outcome
//        previousStatus = currentStatus;
//        currentStatus = gamepad1.right_bumper;


        //********************************
        //  A S S I G N  G A I N  M O D E S @@
        //********************************


        if (gamepad1.right_bumper) { //If gamepad1 right bumper than switch current value of half gain boolean


            halfGain = !halfGain;
        }


        if (halfGain) //If we are on half gain then
        { //set the gain to .5 and show telemetry showing that
            gain = .5;
            telemetry.addData("On half gain (true/false)", halfGain);
        } else {


            gain = 1;
            telemetry.addData("On half gain (true/false)", halfGain);
        }


//        if (gamepad1.left_bumper) { //If gamepad1 right bumper than switch current value of half gain boolean
//
//
//            shooterOff = !shooterOff;
//            telemetry.addData("On shooterOff (true/false)", shooterOff);
//        }

        //set leftPower and rightPower to .95 * the value of joystick1ValueLeft and joystick1ValueRight cubed, respectively, in order to
        //allow fine control for driving
        leftPower = .95 * (Math.pow(joystick1ValueLeft, 3)) * gain;
        rightPower = .95 * (Math.pow(joystick1ValueRight, 3)) * gain;


        //POSSIBLE DRIVING GAIN OPTIONS TO TEST
        //leftPower = Math.sin((Math.PI/2)*joystick1ValueLeft);
        //rightPower = Math.sin((Math.PI/2)*joystick2ValueRight);
        //leftPower = .95 * (Math.pow (joystick1ValueLeft,3));
        //rightPower = .95 * (Math.pow (joystick1ValueLeft,3));
        //leftPower = Math.pow(joystick1ValueLeft, exponent);
        //rightPower = Math.pow(joystick2ValueRight,exponent);
        //leftPower = 1- Math.pow(1-joystick1ValueLeft*joystick1ValueLeft, .5);
        //rightPower = 1- Math.pow(1-joystick2ValueRight*joystick2ValueRight, .5);
        //leftPower = ((Math.pow(base, joystick1ValueLeft)-1)/(base-1));
        //rightPower = ((Math.pow(base,joystick2ValueRight)-1)/(base-1));
        //leftPower = .64 * Math.tan(joystick1ValueLeft);
        //rightPower = .64 * Math.tan(joystick2ValueRight);


        //*****************
        // D R I V I N G **  ASSUME THIS WORKS, ANOTHER VERSION WILL HAVE SIMPLE DRIVE CODE
        //*****************


        //ensures the value of the joystick, if negative, will result in a negative value of power


        if (joystick1ValueLeftYAxis < 0) {
            leftPower *= -Math.abs(leftPower);
        }
        if (joystick1ValueRightYAxis < 0) {
            rightPower *= -Math.abs(rightPower);
        }


        //set left motors and right motors to leftPower and rightPower, respectively
        leftMotor1.setPower(leftPower);
        leftMotor2.setPower(leftPower);


        rightMotor1.setPower(rightPower);
        rightMotor2.setPower(rightPower);




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


        //**********************************
        // T U R R E T   R O T A T I O N S  @@
        //**********************************




        currentEncoderPosition = encode.getCurrentPosition();  //set current Encoder position
        telemetry.addData("Turret Position" , currentEncoderPosition); //telemetry
        countDelta = (double) Math.abs(currentEncoderPosition - initialEncoderCount); //set countDelta
        rotations =   (countDelta / countsPerRotation) * (GEAR_RATIO); //set rotations to countDelta/countsPerRotation * GEAR_RATIO
        //send telemetry concerning the status of the turret
        if (rotations < 3) {
            turret.setPosition(turretRotationValue);
            telemetry.addData("Turret OK", rotations);
        }
        else if (rotations >= 3 && rotations < 5) {
            turret.setPosition(turretRotationValue);
            telemetry.addData("TURRET IN DANGER", rotations);
        }
        else {
            turret.setPosition(.5);
            telemetry.addData("ABORT TURRET", rotations);
        }
        telemetry.addData("Turett countDelta: ", countDelta);


        //*****************
        // S H O O T I N G **
        //*****************



        //Current Run Time
        timeTwo = this.getRuntime();
        //Current Encoder Clicks
        encoderClicksTwo = shooter.getCurrentPosition();




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
            avgRpm = (int) totalRpm / 5; //set avgRpm to totalRpm divided by five casted as an int
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
        //telemetry for rpm and averages
        telemetry.addData("WeightedRPM: ", tempWeightedAvg);
        telemetry.addData("RPM : ", rpm);
        telemetry.addData("AvgRPM : ", avgRpm);


        //***************************************************
        //E L E V A T O R  L O A D E R  &  I N T A K E **
        //***************************************************




        if (gamepad2.right_trigger > 0.6) { //triggers act like an axis, so to make them behave like buttons
            // we set them to active when they reach beyond a certain value: .6.  If the right trigger's
            //value is above .6, activate elevator (going up) and intake (coming in)
            elevatorSpeed = 1.0;
            intakeSpeed = 1.0;
        } else if (gamepad2.right_bumper) { //If the right bumper is pressed, activate elevator (going down) and intake (going out)
            elevatorSpeed = -1.0;
            intakeSpeed = -1.0;
        } else if ((gamepad2.left_trigger > 0.6) && (!(gamepad2.right_trigger > 0.6) && !(gamepad2.right_bumper))) { //If the left trigger's value is beyond .6,
            //but the right trigger is not above .6 and the right bumper is not being pressed, keep the elevator still and activate the intake (coming in)
            intakeSpeed = 1.0;
            elevatorSpeed = 0.0;
        } else if ((gamepad2.left_bumper) && (!(gamepad2.right_trigger > 0.6) && !(gamepad2.right_bumper)))//If the left bumper is being pressed,
        //but the right trigger is not above .6 and the right bumper is not being pressed, keep the elevator still and activate the intake (going out)
        {
            intakeSpeed = -1.0;
            elevatorSpeed = 0.0;
        } else {//If none of the above conditions are true, stop running both the intake and the elevator
            intakeSpeed = 0.0;
            elevatorSpeed = 0.0;
        }




        //increment shooter motor power based on dpad commands
            shooterSpeed+= rpmGain * (2000-avgRpm);



        //*****************
        //SET ALL POWERS @@
        //*****************

        //Set the elevator and intake's speed to the values specified above



//    if (shooterOff)
//    {
//        shooterSpeed = 0;
//    }

//        //no longer used because of Range.clip right below
//        if (shooterSpeed>1)
//        {
//            shooterSpeed=1;
//        }
//        if (shooterSpeed<0)
//        {
//            shooterSpeed=0;
//        }

        shooterSpeed = Range.clip(shooterSpeed,0,1);


        shooter.setPower(shooterSpeed);

        intake.setPower(intakeSpeed);
        telemetry.addData("shooter speed: ", shooterSpeed);

        elevator.setPower(elevatorSpeed);


        //*****************
        // H O O D @@
        //*****************








        //increase/decrease hood positioning at a slow rate to allow fine tune adjustment because the code will cycle approx. 100-150 times a second




        if (gamepad2.y) { //If y is being pressed, move the hood down
            mode=0;
            hoodPositioning += .001;
        }
        if (gamepad2.a) { //If a is being pressed, move the hood up
            mode=0;
            hoodPositioning -= .001;
        }
        //If the hoodPositioning is out of the possible servo range limits (both logically and physically), correct the values to
        //ensure the hoodPositioning value is in the correct range
        if (hoodPositioning > .35) {
            hoodPositioning = .35;
        }








        if (hoodPositioning < .09) {
            hoodPositioning = .09;
        }




        //Set the position of the hood to hoodPositioning
        hood.setPosition(hoodPositioning);
        telemetry.addData("hood" , hoodPositioning);




        //*****************
        // B E A C O N @@
        //*****************




        //Move both port side and battery side beacons based on actions on the dpad*/
        if (gamepad1.x) {
            portSideBeaconPosition = .2;
        }
        else if (gamepad1.b) {
            portSideBeaconPosition = .8;
        }
        else
        {
            portSideBeaconPosition = .5;
        }








        if (gamepad1.a) {
            batterySideBeaconPosition = .2;
        }
        else if (gamepad1.y) { //If a is being pressed, move the hood up
            batterySideBeaconPosition = .8;
        }
        else
        {
            batterySideBeaconPosition=.5;
        }
        //set the position of each beacon to its respective position determined above
        portSideBeacon.setPosition(portSideBeaconPosition);
        batterySideBeacon.setPosition(batterySideBeaconPosition);








        telemetry.update(); //update telemetry
    }
//
//    @Override
//    public void stop() {
//
//        do {
//
//            shooterSpeed -= .001;
//            shooter.setPower(shooterSpeed);
//            telemetry.addData("shooterSpeed = ", shooterSpeed);
//            telemetry.update();
//            timeOne = this.getRuntime();
//
//        } while (shooterSpeed > 0);
//
//        shooter.setPower(0);
//
//    }
}
