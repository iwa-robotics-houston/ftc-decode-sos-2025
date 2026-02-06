package org.firstinspires.ftc.teamcode;

/*
 Limelight AprilTag TeleOp with PID Lock-On & Goal-Specific Offset (strafe only)
*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name = "AprilTag TeleOp PID Goal Lock", group = "LinearOpMode")
public class AprilTagTeleop extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private RevBlinkinLedDriver blinkin;

    // Lock-on toggle
    private boolean lockOn = false;

    // PID constants
    private double kp = 0.05;        // proportional gain
    private double ki = 0.002;       // integral gain
    private double kd = 0.01;        // derivative gain
    private double integral = 0;
    private double prevError = 0;
    private double dt = 0.02;        // loop time in seconds (~50Hz)

    private double lockThreshold = 0.5; // degrees to consider centered
    private double maxLateral = 0.5;    // max strafe speed

    private double targetVelocity = 0;
    private RevBlinkinLedDriver.BlinkinPattern readyColor = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    // Goal selection
    private double targetTX = 0; // dynamic target offset
    private String goalColor = "RED"; // default goal

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // default pipeline
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "Jesuz");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double tolerance = 40; // flywheel tolerance

        while (opModeIsActive()) {

            // -------- GOAL SELECTION --------
            if (gamepad1.b) {
                goalColor = "RED";
                targetTX = -1.2;
            } else if (gamepad1.x) {
                goalColor = "BLUE";
                targetTX = 1.2;
            }

            // -------- TOGGLE LOCK-ON --------
            if (gamepad1.a) {
                lockOn = !lockOn;
                integral = 0;  // reset integral
                prevError = 0;
                sleep(250); // debounce
            }

            // -------- DRIVER INPUTS --------
            double axial = -gamepad1.left_stick_y;   // forward/back
            double lateral = gamepad1.left_stick_x;  // strafe
            double yaw = gamepad1.right_stick_x;     // rotate

            boolean locked = false;

            // -------- PID LOCK-ON STRAFING --------
            if (lockOn && limelight.getLatestResult() != null && limelight.getLatestResult().isValid()) {
                double tx = limelight.getLatestResult().getTx(); // horizontal offset

                double error = tx - targetTX;

                // Deadzone for locking
                if (Math.abs(error) < lockThreshold) {
                    lateral = 0;
                    locked = true;
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    integral = 0; // reset integral when locked
                } else {
                    locked = false;

                    // PID calculations
                    integral += error * dt;
                    double derivative = (error - prevError) / dt;
                    lateral = kp * error + ki * integral + kd * derivative;

                    // Clamp lateral speed
                    lateral = Math.max(Math.min(lateral, maxLateral), -maxLateral);

                    prevError = error;
                }

                // Disable forward/backward while locked
                axial = 0;

                telemetry.addData("Lock-On Mode", "Active");
                telemetry.addData("TX", tx);
                telemetry.addData("Target TX", targetTX);
                telemetry.addData("Locked onto tag?", locked ? "YES" : "NO");
                telemetry.addData("Goal Color", goalColor);

            } else {
                telemetry.addData("Lock-On Mode", "Inactive");
                telemetry.addData("Locked onto tag?", "N/A");
            }

            // -------- DRIVE CALCULATION (mecanum) --------
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 0.85) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightBackDrive.setPower(rightBackPower);

            // -------- INTAKE --------
            if (gamepad2.left_trigger > 0) {
                robot.rollerIntake.setPower(-1);
                robot.hotwheelsfront.setPower(-1);
                robot.hotwheelsback.setPower(-1);
            } else if (gamepad2.left_bumper) {
                robot.rollerIntake.setPower(1);
                robot.hotwheelsfront.setPower(1);
                robot.hotwheelsback.setPower(1);
                robot.rollitbackbottom.setPower(-1);
            } else {
                robot.rollerIntake.setPower(0);
                robot.hotwheelsfront.setPower(0);
                robot.hotwheelsback.setPower(0);
            }

            // -------- SHOOTER MODE SELECT --------
            if (gamepad2.x) {
                targetVelocity = 1310;
                readyColor = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
            } else if (gamepad2.y) {
                targetVelocity = 1500;
                readyColor = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
            }

            // -------- SHOOTER CONTROL --------
            if (gamepad2.right_trigger > 0) {
                robot.flywheel1.setVelocity(-targetVelocity);
                robot.flywheel2.setVelocity(-targetVelocity);
                robot.rollitbackbottom.setPower(0);
                robot.rollitbacktop.setPower(0);
            } else if (gamepad2.right_bumper) {
                robot.flywheel1.setVelocity(targetVelocity);
                robot.flywheel2.setVelocity(targetVelocity);
                robot.rollitbackbottom.setPower(1);
                robot.rollitbacktop.setPower(1);
            } else {
                robot.flywheel1.setVelocity(0);
                robot.flywheel2.setVelocity(0);
                robot.rollitbackbottom.setPower(0);
                robot.rollitbacktop.setPower(0);

                if (!lockOn || !locked) {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            }

            // -------- BLINKIN READY INDICATOR WHEN SHOOTER AT SPEED --------
            if (targetVelocity > 0 && (gamepad2.right_trigger > 0 || gamepad2.right_bumper)) {
                double flywheelVel = Math.abs(robot.flywheel1.getVelocity());
                if (flywheelVel >= targetVelocity - tolerance && !locked) {
                    blinkin.setPattern(readyColor);
                } else if (!locked) {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            }

            // -------- TELEMETRY --------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();

            sleep((long)(dt*1000)); // maintain approx loop timing
        }
    }
}
