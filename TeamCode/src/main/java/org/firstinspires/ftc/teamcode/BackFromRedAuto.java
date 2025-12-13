package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// THIS IS THE AUTONOMOUS FOR WHEN THE ROBOT STARTS AT THE RED GOAL AND BACKS UP TO SHOOT
@Autonomous(name = "BackFromRed", group = "OpMode")
public class BackFromRedAuto extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {

        // Initialize robot
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // back up to line up shot
        driveAll(-0.6);   // backward at 60% power
        sleep(1000);      // move for 1 second
        driveAll(0);      // stop


        // fire two artifacts using velocity trigger
        fireSequence(1285, 2);  // 1285 ticks/sec target velocity, 2 balls

        // strafe right after all shots
        strafeRight(0.4); // 40% power RIGHT
        sleep(2000);     // strafe for 2 seconds
        driveAll(0);     // stop movement


        telemetry.addData("Status", "ugghghghgh");
        telemetry.update();
    }


    // Fires each ball only when flywheel is up to speed
    private void fireSequence(double targetVelocity, int shots) {

        // Start flywheel motors spinning toward target
        robot.flywheel1.setVelocity(-targetVelocity);
        robot.flywheel2.setVelocity(-targetVelocity);
        //code from some website.  TRUST IT.  IMPORTANT!!
        //&& = adn
        for (int i = 0; i < shots && opModeIsActive(); i++) {

            // Wait until flywheel is up to speed
            while (opModeIsActive() && getAvgFlywheel() < targetVelocity * 0.99) {
                telemetry.addData("Flywheel Avg", getAvgFlywheel());
                telemetry.addData("Shots Fired", i);
                telemetry.update();
                sleep(10);  // tiny delay
            }

            // Feed one artifact
            feedOnce();

            // Short pause to allow flywheel to recover
            sleep(120);
        }

        // Stop flywheel after all shots
        robot.flywheel1.setVelocity(0);
        robot.flywheel2.setVelocity(0);
    }

    // avg flywheel velocity
    private double getAvgFlywheel() {
        return (Math.abs(robot.flywheel1.getVelocity()) +
                Math.abs(robot.flywheel2.getVelocity())) / 2.0;
    }

    // feed one artifact
    private void feedOnce() {

        // Start feeding motors
        robot.hotwheelsback.setPower(1);
        robot.rollitbackbottom.setPower(-1);
        robot.rollitbacktop.setPower(-1);

        sleep(1000);

        // Stop feeding motors
        robot.hotwheelsback.setPower(0);
        robot.rollitbackbottom.setPower(0);
        robot.rollitbacktop.setPower(0);
    }

    // drivetrain helpers
    private void driveAll(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    // strafe to the right
    private void strafeRight(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightBackDrive.setPower(power);
    }

}
