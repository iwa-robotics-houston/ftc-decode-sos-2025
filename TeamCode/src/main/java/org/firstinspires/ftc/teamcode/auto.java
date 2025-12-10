package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name = "auto", group = "OpMode")
public class auto extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        // Create robot instance (loads all motors & servos)
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;


        // DRIVE FORWARD 2 SECONDS
        //driveAll(0.3);   // 30% power forward
        //sleep(2000);

        // STOP
        //driveAll(0);
        //sleep(1000);


        // DRIVE BACKWARD 2 SECONDS
        driveAll(-0.5);
        sleep(1000);
        fireINtheHole(0.6);
        sleep(2000);
        strafeleft(0.3);


        // FINAL STOP
        driveAll(0);
    }

    /**
     * Helper method to power every drive motor on the robot
     */
    private void driveAll(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);





    }

    private void fireINtheHole(double fire) {
        robot.flywheel1.setVelocity(fire);
        robot.flywheel2.setVelocity(fire);
    }

    private void strafeleft(double totheleft) {


        robot.leftFrontDrive.setPower(-totheleft);
        robot.rightFrontDrive.setPower(totheleft);
        robot.leftBackDrive.setPower(-totheleft);
        robot.rightBackDrive.setPower(totheleft);
    }
}

