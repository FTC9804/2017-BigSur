/* V2 created for the new Drogon robot with adaptations to the driving with the
 * redesigned west coast drive and updated chassis.
 * Separate codes for red and blue
 */


//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "V2REDAUTO", group = "AutoWithFunctions")
//@Disabled
public class V2RedAuto extends FunctionsForILT {

    public void runOpMode() throws InterruptedException {
//Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        //Calibrate the Gyro
        calibrateGyro();


        driveBack(20, .2);


        shootAndLift(10, 2025, .95, .95);


        spinMove(25);


        driveMoreLeftBack(40, .25, 0);


        lineUpFasterLeftBack();


        pressBeaconFrontRedNew(true);


        driveMoreRight (29, .5, 0);


        lineUpFasterRight();


        pressBeaconFrontBlueNew(false);

        spinMove(-18);


        drive(33, .6);
    }
}
