package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BackFromBlue", group = "OpMode")
public class BackFromBlueAuto extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Back up to line up shot
        driveAll(-0.6);
        sleep(1000);
        driveAll(0);

        // Fire first two balls smoothly
        fireSequenceSmooth(1310, 2);

        // Stage third ball
        advanceThirdBallSmooth(900);

        // Fire third ball
        fireSequenceSmooth(1310, 1);

        // Strafe left and stop
        strafeLeft(0.4);
        sleep(2000);
        driveAll(0);

        telemetry.addData("Status", "Finished Auto");
        telemetry.update();
    }

    // flywheel keeps spinning, balls fed one by one
    private void fireSequenceSmooth(double targetVelocity, int shots) {

        final double spinupTimeout = 1.5; // seconds
        final double percentReady = 0.97; // 97% of target for READY

        // Start flywheel and keep it running
        robot.flywheel1.setVelocity(-targetVelocity);
        robot.flywheel2.setVelocity(-targetVelocity);

        for (int i = 0; i < shots && opModeIsActive(); i++) {

            // Slight delay for consistent launch
            ElapsedTime spinupTimer = new ElapsedTime();
            spinupTimer.reset();
            while (opModeIsActive() && getAvgFlywheel() < targetVelocity * percentReady && spinupTimer.seconds() < spinupTimeout) {
                telemetry.addData("Shot", i + 1);
                telemetry.addData("Flywheel Avg", getAvgFlywheel());
                telemetry.addData("Target Vel", targetVelocity);
                telemetry.update();
                sleep(30);
            }

            // Start intake to feed ball smoothly while flywheel keeps running
            startIntake();
            sleep(300); // slight buffer before ball hits flywheel

            // Fire ball
            feedOnceSmooth();

            // Keep intake moving just enough to stage next ball (if any)
            if (i < shots - 1) {
                startIntake(); // keeps balls moving up
            }

            sleep(150); // small delay between shots for consistency
        }

        // Stop intake after all shots, keep flywheel off
        stopIntake();
        robot.flywheel1.setVelocity(0);
        robot.flywheel2.setVelocity(0);
    }

    // Pulls third ball into flywheel
    private void advanceThirdBallSmooth(int stageTimeMs) {
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

    // Feed one ball smoothly
    private void feedOnceSmooth() {
        robot.hotwheelsback.setPower(1);
        robot.rollitbackbottom.setPower(-1);
        robot.rollitbacktop.setPower(-1);

        sleep(1000); // feed duration

        robot.hotwheelsback.setPower(0);
        robot.rollitbackbottom.setPower(0);
        robot.rollitbacktop.setPower(0);
    }

    // Start intake wheels for smooth staging
    private void startIntake() {
        robot.rollerIntake.setPower(1);
        robot.hotwheelsfront.setPower(1);
        robot.hotwheelsback.setPower(1);
    }

    // Stop intake wheels
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
}