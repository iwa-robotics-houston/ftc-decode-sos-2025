package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// THIS IS THE AUTONOMOUS FOR WHEN THE ROBOT STARTS AGAINST THE WALL
// AND DRIVES FORWARD -> TURNS RIGHT -> SHOOTS INTO RED GOAL
@Autonomous(name = "ForwardRedAuto", group = "OpMode")
public class FRedAuto extends LinearOpMode {

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
        driveAll(0.6);
        sleep(2000);  // 900
        driveAll(0);


        // Turn slightly RIGHT for red goal
        turnRight(0.28);      // ← mirrored from the blue side
        sleep(380);
        driveAll(0);


        // Drive forward more to get into shooting position
        //driveAll(0.6);
        //sleep(300);
        //driveAll(0);


        // Fire two velocity-triggered shots
        fireSequence(1310, 2);

        strafeRight(0.4); // 40% power RIGHT
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


    // Turn right
    private void turnRight(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.rightBackDrive.setPower(-power);
    }
    private void strafeRight(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightBackDrive.setPower(power);
    }

}