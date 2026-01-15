package org.firstinspires.ftc.teamcode;

/*
 Limelight AprilTag
*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// LIMELIGHT IMPORTS
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOp", group = "LinearOpMode")
public class StarterBotTeleop extends LinearOpMode {

    // LIMELIGHT VARIABLES
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        // LIMELIGHT INIT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        /*
         Pipeline reference:
         0 = Blue AprilTag (ex: Tag 20)
         1 = Red  AprilTag (ex: Tag 24)
        */
        limelight.pipelineSwitch(0);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // MAIN LOOP
        while (opModeIsActive()) {

            /*
            // LIMELIGHT LOOP
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                Pose2D botPose = llResult.getBotpose_MT2();

                telemetry.addData("LL Tx", llResult.getTx());
                telemetry.addData("LL Ty", llResult.getTy());
                telemetry.addData("LL Ta", llResult.getTa());

                telemetry.addData("LL X (m)", botPose.getX());
                telemetry.addData("LL Y (m)", botPose.getY());
                telemetry.addData("LL Heading (deg)", botPose.getHeading());
            } else {
                telemetry.addLine("Limelight: No valid target");
            }

            // Driver pipeline toggle
            if (gamepad1.x) {
                limelight.pipelineSwitch(0); // Blue
            } else if (gamepad1.b) {
                limelight.pipelineSwitch(1); // Red
            }

             */

            // DRIVE CONTROLS
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
            double intakePower = 1;
            double compliantWheel = 1;

            if (gamepad2.left_trigger > 0) {
                robot.rollerIntake.setPower(-intakePower);
                robot.hotwheelsfront.setPower(-compliantWheel);
                robot.hotwheelsback.setPower(-compliantWheel);
            } else if (gamepad2.left_bumper) {
                robot.rollerIntake.setPower(intakePower);
                robot.hotwheelsfront.setPower(compliantWheel);
                robot.hotwheelsback.setPower(compliantWheel);
            } else {
                robot.rollerIntake.setPower(0);
                robot.hotwheelsfront.setPower(0);
                robot.hotwheelsback.setPower(0);
            }

            // FLYWHEELS / LAUNCH
            double ticksPerRotation = 5600;

            if (gamepad2.right_trigger > 0) {
                robot.flywheel1.setVelocity(-ticksPerRotation);
                robot.flywheel2.setVelocity(-ticksPerRotation);
                robot.rollitbackbottom.setPower(0);
                robot.rollitbacktop.setPower(0);
            } else if (gamepad2.right_bumper) {
                robot.flywheel1.setVelocity(ticksPerRotation);
                robot.flywheel2.setVelocity(ticksPerRotation);
                robot.rollitbackbottom.setPower(1);
                robot.rollitbacktop.setPower(1);
            } else {
                robot.flywheel1.setVelocity(0);
                robot.flywheel2.setVelocity(0);
                robot.rollitbackbottom.setPower(0);
                robot.rollitbacktop.setPower(0);
            }

            // TELEMETRY
            telemetry.addData("Run Time", runtime.toString());

            telemetry.addData("Flywheel 1 Vel", robot.flywheel1.getVelocity());
            telemetry.addData("Flywheel 2 Vel", robot.flywheel2.getVelocity());

            telemetry.addData("Drive FL / FR", "%.2f / %.2f",
                    robot.leftFrontDrive.getPower(),
                    robot.rightFrontDrive.getPower());

            telemetry.addData("Drive BL / BR", "%.2f / %.2f",
                    robot.leftBackDrive.getPower(),
                    robot.rightBackDrive.getPower());

            telemetry.update();
        }
    }
}
