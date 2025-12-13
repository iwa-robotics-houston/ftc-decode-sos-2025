package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// THIS IS THE AUTONOMOUS FOR WHEN THE ROBOT STARTS AGAINST THE WALL
// AND DRIVES FORWARD -> TURNS LEFT -> SHOOTS INTO BLUE GOAL
@Autonomous(name = "ForwardBlueAuto", group = "OpMode")
public class ForwardBlueAuto extends LinearOpMode {

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
        sleep(1000);        //move 0.9 sec, probably should adjust
        driveAll(0);

        // Turn slightly left to aim at blue goal
        turnLeft(0.4);     // 40% power turn
        sleep(380);        // adjust based on your robot turn rate
        driveAll(0);

        // Drive forward again
        driveAll(0.6);     // forward at 60%
        sleep(900);        // move 0.9 sec, probably should adjust
        driveAll(0);

        // Fire two shots using velocity triggering
        fireSequence(1320, 2);

        strafeLeft(0.4); // 40% power left
        sleep(2000);     // strafe for 2 seconds
        driveAll(0);     // stop movement


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

    // Turn left
    private void turnLeft(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }
    private void strafeLeft(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(-power);
    }


}