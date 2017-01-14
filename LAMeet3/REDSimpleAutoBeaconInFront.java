package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "REDSimpleAutoBeaconInFront", group = "AutoWithFunctions")
//@Disabled
public class REDSimpleAutoBeaconInFront extends FunctionsForMeet3 {

    public void runOpMode() throws InterruptedException {
        Configure();

        waitForStart();

        calibrateGyro();

        /*Driving code*/

        drive(22, .4, 0);

        shootAndLift(20, 2300, .95, .95);

        spinMove(40);

        driveToOneWhiteLine();

        spinMove(85);
        
        driveToTouch(0.25);

        pressBeaconFrontRed();



    }

}