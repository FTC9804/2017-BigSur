//Version 1.0 coded Feb. 4, 2017 by Steve, Etienne and Marcus.
//Designed to test the autonomous concept
//of shooting two balls and pressing two beacons

//package declaration
package org.firstinspires.ftc.teamcode;

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "DriveShoot", group = "AutoWithFunctions")

//@Disabled
public class DriveShoot extends FunctionsForLA {

    public void runOpMode() throws InterruptedException {



        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        timeTwo = this.getRuntime();

        timeOne= this.getRuntime();

        while (timeTwo-timeOne<2)
        {
            timeTwo=this.getRuntime();
        }

        calibrateGyro();


        //Set shooter to variable shooterPower
        shooter.setPower(shooterPower);

        //Drive back for 30 inches at .4 power
        driveBack(30, .4, 0);

        //Shoot and Lift function: 2800 rpm, .95 intake
        shootAndLift(3000, .95, 5);

        driveBack(10, .4, 0);
    }
}
