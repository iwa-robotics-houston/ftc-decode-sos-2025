package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// THIS IS THE AUTONOMOUS FOR WHEN THE ROBOT STARTS AGAINST THE WALL
// AND DRIVES FORWARD -> TURNS LEFT -> SHOOTS INTO BLUE GOAL
@Autonomous(name = "FBlueAuto", group = "OpMode")
public class FBlue extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {

        // Initialize robot
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Drive forward away from wall
        driveAll(0.6);     // forward at 60%
        sleep(2000);        // move 0.9 sec, probably should adjust
        driveAll(0);

        // Turn slightly left to aim at blue goal
        turnLeft(0.28);     // 40% power turn
        sleep(380);        // adjust based on your robot turn rate
        driveAll(0);

        // Drive forward again
        //driveAll(0.6);     // forward at 60%
        //sleep(1100);        // move 0.9 sec, probably should adjust
        //driveAll(0);

        // Fire two shots using velocity triggering
        fireSequence(1320, 2, 500);

        advanceThirdBall(900);

        //fireSequence(1310, 1);


        telemetry.addData("Status", "Finished Auto :)");
        telemetry.update();
    }


    // Shooting system
    private void fireSequence(double targetVelocity, int shots, int delayBetweenShotsMs) {

        final double tolerance = 0.99;
        final double spinupTimeout = 1.5;

        robot.flywheel1.setVelocity(-targetVelocity);
        robot.flywheel2.setVelocity(-targetVelocity);

        for (int i = 0; i < shots && opModeIsActive(); i++) {

            ElapsedTime spinupTimer = new ElapsedTime();
            spinupTimer.reset();
            while (opModeIsActive() && getAvgFlywheel() < targetVelocity * tolerance && spinupTimer.seconds() < spinupTimeout) {
                telemetry.addData("Shot", i + 1);
                telemetry.addData("Flywheel Avg", getAvgFlywheel());
                telemetry.addData("Target Vel", targetVelocity);
                telemetry.update();
                sleep(20);
            }

            feedOnce();

            if (i < shots - 1) {
                startIntake();
                sleep(delayBetweenShotsMs);
            }
        }

        stopIntake();
        robot.flywheel1.setVelocity(0);
        robot.flywheel2.setVelocity(0);
    }


    private double getAvgFlywheel() {
        return (Math.abs(robot.flywheel1.getVelocity()) +
                Math.abs(robot.flywheel2.getVelocity())) / 2.0;
    }

    private void feedOnce() {
        robot.hotwheelsback.setPower(1);
        robot.rollitbackbottom.setPower(-1);
        robot.rollitbacktop.setPower(-1);

        sleep(1500);

        robot.hotwheelsback.setPower(0);
        robot.rollitbackbottom.setPower(0);
        robot.rollitbacktop.setPower(0);
    }


    private void advanceThirdBall(int stageTimeMs) {
        startIntake();
        robot.rollitbackbottom.setPower(-1);
        robot.rollitbacktop.setPower(-1);

        sleep(stageTimeMs);

        stopIntake();
        robot.rollitbackbottom.setPower(0);
        robot.rollitbacktop.setPower(0);
    }

    private void startIntake() {
        robot.rollerIntake.setPower(1);
        robot.hotwheelsfront.setPower(1);
        robot.hotwheelsback.setPower(1);
    }

    private void stopIntake() {
        robot.rollerIntake.setPower(0);
        robot.hotwheelsfront.setPower(0);
        robot.hotwheelsback.setPower(0);
    }
    // Movement helpers
    private void driveAll(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    // Turn left
    private void turnLeft(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }


}