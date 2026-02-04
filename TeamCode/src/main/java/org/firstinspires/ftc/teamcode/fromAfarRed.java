package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "fromAfarRed", group = "OpMode")
public class fromAfarRed extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Back up to line up shot
        strafeLeft(0.4);
        sleep(1000);
        driveAll(0);

        turnRight(0.28);
        sleep(350);
        driveAll(0);

        // Fire first two balls
        fireSequence(1505, 2, 500); // 250ms delay between first two shots

        // Third ball to flywheel
        advanceThirdBall(900);

        // Strafe left and stop
        strafeRight(0.4);
        sleep(2000);
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

    // Stage third ball
    private void advanceThirdBall(int stageTimeMs) {
        startIntake();
        robot.rollitbackbottom.setPower(-1);
        robot.rollitbacktop.setPower(-1);

        sleep(stageTimeMs);

        stopIntake();
        robot.rollitbackbottom.setPower(0);
        robot.rollitbacktop.setPower(0);
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

    private void strafeRight(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightBackDrive.setPower(power);
    }

    private void turnRight(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.rightBackDrive.setPower(-power);
    }
}
