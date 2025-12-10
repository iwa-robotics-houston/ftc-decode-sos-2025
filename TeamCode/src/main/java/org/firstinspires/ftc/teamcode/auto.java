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
        driveAll(-0.3);
        sleep(2000);

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


    }


    /*private void fireINtheHole(double fire) {
        robot.flywheel1.setVelocity(fire);
        robot.flywheel2.setVelocity(fire);*/


