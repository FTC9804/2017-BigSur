

//Version 1.0 coded Jan. 15, 2017 by Rylan and Marcus.  Designed to test the new autonomous concept
//of driving forward, shooting, turning to 45, finding the white line, turning to 90
//and then driving forward and pressing the beacon.  In version 1.0 we solely tested driving
//forward, turning to 45, finding the white line, and turning to 90, which was accomplished.


//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "REDSimpleAutoBeaconInFront1.204", group = "AutoWithFunctions")
//@Disabled
public class REDSimpleAutoBeaconInFront extends FUNCTIONSDEC20 {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();


        //Calibrate the Gyro
        calibrateGyro();

        //Drive Forward 10 inches at .4 speed, maintaining a gyro heading of 0
        drive(10, .4, 0);

        //Run shootAndLift function for 20 seconds at a goal of 2300 rpm, with intake and elevator speeds of .95
        //shootAndLift(20, 2300, .95, .95);

        //Move 45 degrees counterclockwise.  Set parameter to 36 degrees counterclockwise to combat overshoot in the gyro
        spinMove(36);


        //Set timeOne and timeTwo to the current run time
        timeTwo=this.getRuntime();
        timeOne=this.getRuntime();

        //Wait three seconds
        while (timeTwo - timeOne <3)
        {
            timeTwo=this.getRuntime();
        }

        //Run driveToOneWhiteLineRight function
        driveToOneWhiteLineLeft();

        //Set timeOne and timeTwo to the current run time
        timeTwo=this.getRuntime();
        timeOne=this.getRuntime();

        //Run driveToOneWhiteLineRight function
        while (timeTwo - timeOne <3)
        {
            timeTwo=this.getRuntime();
        }

        //Set timeOne and timeTwo to the current run time
        timeTwo=this.getRuntime();
        timeOne=this.getRuntime();

        //Run driveToOneWhiteLineRight function
        while (timeTwo - timeOne <3)
        {
            timeTwo=this.getRuntime();
        }


        //Move to 90 degrees counterclockwise.  Set parameter to 73 degrees counterclockwise to combat overshoot in the gyro
        spinMove(73);






        //Run driveToTouch method, applying .25 power to the motors when doing so
        //driveToTouch(0.12);

        //Run pressBeaconFrontRed method
        //pressBeaconFrontRed();








    }

}
