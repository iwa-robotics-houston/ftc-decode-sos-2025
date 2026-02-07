package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "fromAfarBlue", group = "OpMode")
public class fromAfarBlue extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Back up to line up shot
        strafeRight(0.45);
        sleep(1000);
        driveAll(0);

        turnLeft(0.275); //original .275
        sleep(340); // original 360
        driveAll(0);

        // Drive forward a little bit
        driveAll(0.3);
        sleep(200);  // Adjust this time to control how far forward
        driveAll(0);

        // Fire first two balls
        fireSequence(1525, 2, 750);

        // Stage third ball (just position it, don't feed yet)
        advanceThirdBall(580);

        // Fire third ball
        fireSequence(1525, 1, 450);

        // Drive forward to ball collection area
        driveAll(0.5);
        sleep(900);  // Adjust time based on distance to balls
        driveAll(0);

        // Turn to face the balls
        turnLeft(0.3);
        sleep(400);  // Adjust to get correct angle
        driveAll(0);

        // Intake three balls
        intakeThreeBalls(2500);  // Run intake for 2.5 seconds to collect 3 balls

        // Back up
        driveAll(-0.5);
        sleep(400);  // Same distance as forward drive
        driveAll(0);

        telemetry.addData("Status", "Finished Auto");
        telemetry.update();
    }

    // Fire shots at target velocity
    private void fireSequence(double targetVelocity, int shots, int delayBetweenShotsMs) {

        final double tolerance = 0.99;  // 99% of target
        final double spinupTimeout = 1.5; // seconds

        // Start flywheel
        robot.flywheel1.setVelocity(-targetVelocity);
        robot.flywheel2.setVelocity(-targetVelocity);

        for (int i = 0; i < shots && opModeIsActive(); i++) {

            // Wait for flywheel to reach target speed (or timeout)
            ElapsedTime spinupTimer = new ElapsedTime();
            spinupTimer.reset();
            while (opModeIsActive() && getAvgFlywheel() < targetVelocity * tolerance && spinupTimer.seconds() < spinupTimeout) {
                telemetry.addData("Shot", i + 1);
                telemetry.addData("Flywheel Avg", getAvgFlywheel());
                telemetry.addData("Target Vel", targetVelocity);
                telemetry.update();
                sleep(20);
            }

            // Feed the ball
            feedOnce();

            // Stage next ball if applicable
            if (i < shots - 1) {
                startIntake();
                sleep(delayBetweenShotsMs); // allow ball to move into position
            }
        }

        // Stop intake and flywheel after all shots
        stopIntake();
        robot.flywheel1.setVelocity(0);
        robot.flywheel2.setVelocity(0);
    }

    // Stage third ball - just move it into position, DON'T feed into flywheel yet
    private void advanceThirdBall(int stageTimeMs) {
        startIntake();
        // DON'T run the back rollers here - that feeds it into the flywheel
        // Just run intake to position the ball

        sleep(stageTimeMs);

        stopIntake();
    }

    // Intake three balls
    private void intakeThreeBalls(int intakeTimeMs) {
        startIntake();

        sleep(intakeTimeMs);

        stopIntake();
    }

    // Average flywheel velocity
    private double getAvgFlywheel() {
        return (Math.abs(robot.flywheel1.getVelocity()) +
                Math.abs(robot.flywheel2.getVelocity())) / 2.0;
    }

    // Feed one ball through shooter
    private void feedOnce() {
        robot.hotwheelsback.setPower(1);
        robot.rollitbackbottom.setPower(-1);
        robot.rollitbacktop.setPower(-1);

        sleep(1200);

        robot.hotwheelsback.setPower(0);
        robot.rollitbackbottom.setPower(0);
        robot.rollitbacktop.setPower(0);
    }

    // Start intake
    private void startIntake() {
        robot.rollerIntake.setPower(1);
        robot.hotwheelsfront.setPower(1);
        robot.hotwheelsback.setPower(1);
    }

    // Stop intake
    private void stopIntake() {
        robot.rollerIntake.setPower(0);
        robot.hotwheelsfront.setPower(0);
        robot.hotwheelsback.setPower(0);
    }

    // Drivetrain helpers
    private void driveAll(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    private void strafeLeft(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(-power);
    }

    private void turnLeft(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    private void strafeRight(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightBackDrive.setPower(power);
    }
}


// i need you to take this code and mirror the direction so it works on the red side
// i need you to take this code and mirror the direction so it works on the red side