package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueFarPGPAuto", group = "OpMode")
public class BlueFarPGPAuto extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Drive forward away from wall
        driveAll(0.6);
        sleep(2000);
        driveAll(0);

        // Turn slightly left to aim at blue goal
        turnLeft(0.28);
        sleep(380);
        driveAll(0);

        // Shoot first two balls
        fireSequence(1320, 2);


        // Go collect more balls


        // Drive backwards away from goal
        driveAll(-0.55);
        sleep(1200);
        driveAll(0);

        // Turn left toward ball stack PGP
        turnLeft(0.35);
        sleep(600);
        driveAll(0);

        // Drive forward to pick up balls
        driveAll(0.4);
        sleep(2000);
        driveAll(0);

        // Intake balls



        // Return to shooting position


        // Drive backwards to original line
        driveAll(-0.55);
        sleep(2000);
        driveAll(0);

        // Turn right to face goal again
        turnRight(0.45);
        sleep(600);
        driveAll(0);

        // Drive forward to shooting spot
        driveAll(0.5);
        sleep(1200);
        driveAll(0);

        // Shoot collected balls
        fireSequence(1320, 2);

        telemetry.addData("Status", "Finished Auto :)");
        telemetry.update();
    }

    // Shooting system
    private void fireSequence(double targetVelocity, int shots) {

        robot.flywheel1.setVelocity(-targetVelocity);
        robot.flywheel2.setVelocity(-targetVelocity);

        for (int i = 0; i < shots && opModeIsActive(); i++) {

            while (opModeIsActive() && getAvgFlywheel() < targetVelocity * 0.99) {
                telemetry.addData("Flywheel Avg", getAvgFlywheel());
                telemetry.addData("Shots Fired", i);
                telemetry.update();
                sleep(10);
            }

            feedOnce();
            sleep(120);
        }

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

    // Movement helpers
    private void driveAll(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    private void turnLeft(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    private void turnRight(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.rightBackDrive.setPower(-power);
    }
}
// unable to find a hardware device with name "frontLeft" and type DC Motor