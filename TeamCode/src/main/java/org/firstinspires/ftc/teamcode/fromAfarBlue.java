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
        strafeRight(0.4);
        sleep(1000);
        driveAll(0);

        turnLeft(0.28);
        sleep(270);  // Reduced from 300 to turn less
        driveAll(0);

        // Fire first two balls
        fireSequence(1505, 2, 500);

        // Stage third ball (just position it, don't feed yet)
        advanceThirdBall(600);  // Reduced time to just stage, not feed

        // Fire third ball
        fireSequence(1505, 1, 250);

        // Strafe left and stop
        strafeLeft(0.4);
        sleep(2000);
        driveAll(0);

        telemetry.addData("Status", "Finished Auto");
        telemetry.update();
    }

    // Fire shots at target velocity
    private void fireSequence(double targetVelocity, int shots, int delayBetweenShotsMs) {

        final double tolerance = 0.97;  // 97% of target
        final double spinupTimeout = 2.0; // seconds for initial spinup
        final double recoveryTimeout = 1.0; // seconds for recovery between shots

        // Start flywheel
        robot.flywheel1.setVelocity(-targetVelocity);
        robot.flywheel2.setVelocity(-targetVelocity);

        for (int i = 0; i < shots && opModeIsActive(); i++) {
            double currentTimeout = (i == 0) ? spinupTimeout : recoveryTimeout;

            // Wait for flywheel to reach target speed (or timeout)
            ElapsedTime spinupTimer = new ElapsedTime();
            spinupTimer.reset();

            while (opModeIsActive() && spinupTimer.seconds() < currentTimeout) {
                double avgVel = getAvgFlywheel();
                telemetry.addData("Shot", i + 1);
                telemetry.addData("Flywheel Avg", avgVel);
                telemetry.addData("Target Vel", targetVelocity);
                telemetry.update();

                // Break when speed is reached
                if (avgVel >= targetVelocity * tolerance) {
                    break;
                }
                sleep(20);
            }

            // Feed the ball
            feedOnce();

            // Stage next ball if applicable
            if (i < shots - 1) {
                // Re-set velocity to maintain speed
                robot.flywheel1.setVelocity(-targetVelocity);
                robot.flywheel2.setVelocity(-targetVelocity);

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

        sleep(1000);  // Reduced from 1200 to minimize flywheel slowdown

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