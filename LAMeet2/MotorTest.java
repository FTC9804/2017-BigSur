/*Code written by Steve, Etienne, & Marcus Fri. 2 Dec. 2016
* updated with new functions code

*   -Friday December 16, 2016: Afternoon --> introducing new code with functions, excluding shooting commands and cat ball
*
* FTC Team 9804 Bomb Squad
*
* Version 1: creates code for full movement, BLUE only
*
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MotorTest3.94", group = "AutoWithFunctions")
//@Disabled
public class MotorTest extends FUNCTIONSDEC20 {


    public void runOpMode() throws InterruptedException, NullPointerException {
        Configure();

        //wait for the code to start to begin the autonomous program
        waitForStart();

        //calibrate gyro after waitForStart to prevent gyro drift from announcers talking
        calibrateGyro();



        drive(20, .5, 0);

        shootAndLift(10, 2500, .95, .95);

        drive(15, .5, -90);

        spinMove(-81);

        
//        timeOne = this.getRuntime();
//
//        do {
//            timeTwo= this.getRuntime();
//        }
//        while (timeTwo-timeOne<2);
//
//        spinMove(0);

//        spinMove(-13);
//
//       drive(20, .5, -8);
//
//        spinMove(-5);
//
//        driveToWhiteLine();





    }

}
