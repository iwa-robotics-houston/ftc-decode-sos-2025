package org.firstinspires.ftc.teamcode;

/*
 Limelight AprilTag
*/

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//she was a fairyyy yea yea 🧚🧚🧚🧚
@TeleOp(name = "TeleOp", group = "LinearOpMode")
public class Main2 extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private RevBlinkinLedDriver blinkin;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
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

        double targetVelocity = 0;
        RevBlinkinLedDriver.BlinkinPattern readyColor = RevBlinkinLedDriver.BlinkinPattern.BLACK;

        while (opModeIsActive()) {

            // DRIVE COPY ALL THIS FOR NEXT SEASON
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

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

            // INTAKE
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

            // SHOOTER MODE SELECT
            if (gamepad2.x) {
                targetVelocity = 1315;
                readyColor = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN; // purple
            } else if (gamepad2.y) {
                targetVelocity = 1505;
                readyColor = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED; // green
            }

            double tolerance = 40;

            // SHOOTER CONTROL
            if (gamepad2.right_trigger > 0) {
                robot.flywheel1.setVelocity(-targetVelocity);
                robot.flywheel2.setVelocity(-targetVelocity);
                robot.rollitbackbottom.setPower(0);
                robot.rollitbacktop.setPower(-1);

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
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            // BLINKIN READY INDICATOR
            if (targetVelocity > 0 && (gamepad2.right_trigger > 0 || gamepad2.right_bumper)) {
                double flywheelVel = Math.abs(robot.flywheel1.getVelocity());

                if (flywheelVel >= targetVelocity - tolerance) {
                    blinkin.setPattern(readyColor);
                } else {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            } else {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Flywheel 1 Vel", robot.flywheel1.getVelocity());
            telemetry.addData("Flywheel 2 Vel", robot.flywheel2.getVelocity());
            telemetry.update();
        }
    }
}