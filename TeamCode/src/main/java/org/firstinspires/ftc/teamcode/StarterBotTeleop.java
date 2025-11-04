package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/** * This is the primary TeleOp for the 2025-2026 DECODE season.
 *
 * CONTROLS:
 * Gamepad 1 (Driver):
 *  - Left Stick:  Strafe (Left/Right) and Drive (Forward/Backward)
 *  - Right Stick: Rotate (Turn Left/Right)
 *
 * Gamepad 2 (Operator):
 *  - Left Trigger:  Run full intake system to bring a ball IN.
 *  - Right Trigger: Run full launch system to move a ball UP and OUT.
 *  - Right Bumper:  Run expel system to move a ball BACK and OUT.
 */
@TeleOp(name = "TeleOp", group = "SOS TestTeleOp")
public class StarterBotTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        // INITIALIZATION
        Robot robot = new Robot(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press START to begin.");
        telemetry.update();

        // Wait for the game to start (driver presses START on the Driver Station)
        waitForStart();
        runtime.reset();

        // MAIN CONTROL LOOP
        while (opModeIsActive()) {

            // Gamepad 1: Drivetrain Controls
            double max;
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral + yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightBackDrive.setPower(rightBackPower);

            // Gamepad 2

            double ticksPerRotation = 2800; // Adjust this value for your flywheel
            double launchVelocity = gamepad2.right_trigger * ticksPerRotation;
            double feedPower = gamepad2.right_trigger;

            // LAUNCH MODE
            // Pressing the right trigger runs all motors needed to move the ball UP and OUT.
            if (gamepad2.right_trigger > 0) {
                // Activate launching motors
                robot.flywheel1.setVelocity(launchVelocity);
                robot.hotwheelBack.setPower(feedPower);
                robot.hotwheelBottom.setPower(feedPower);
                robot.rollitbackTop.setPower(feedPower);

                // Ensure other systems are off
                robot.rollerIntake.setPower(0);
                robot.hotwheelFront.setPower(0);
            }
            // EXPEL MODE
            // Pressing the right bumper reverses all motors needed to expel the ball
            else if (gamepad2.right_bumper) {
                // Activate expel motors (in reverse)
                robot.rollerIntake.setPower(-1.0);
                robot.hotwheelFront.setPower(-1.0);
                robot.hotwheelBack.setPower(-1.0);

                // Ensure other systems are off
                robot.flywheel1.setVelocity(0);
                robot.hotwheelBottom.setPower(0);
                robot.rollitbackTop.setPower(0);
            }
            // INTAKE MODE
            // Pressing the left trigger runs all motors needed to bring a ball in
            else if (gamepad2.left_trigger > 0) {
                // Activate intake motors
                robot.rollerIntake.setPower(1.0);
                robot.hotwheelFront.setPower(1.0);
                robot.hotwheelBack.setPower(1.0);

                // Ensure other systems are off
                robot.flywheel1.setVelocity(0);
                robot.hotwheelBottom.setPower(0);
                robot.rollitbackTop.setPower(0);
            }
            // IDLE MODE
            // If no buttons are pressed, turn everything off
            else {
                robot.flywheel1.setVelocity(0);
                robot.hotwheelBack.setPower(0);
                robot.hotwheelBottom.setPower(0);
                robot.rollitbackTop.setPower(0);
                robot.rollerIntake.setPower(0);
                robot.hotwheelFront.setPower(0);
            }


            // --- TELEMETRY ---
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("--- Drivetrain ---", "");
            telemetry.addData("Front Wheels", "Left: (%.2f), Right: (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("Back Wheels", "Left: (%.2f), Right: (%.2f)", leftBackPower, rightBackPower);
            telemetry.addData("--- Mechanisms ---", "");
            telemetry.addData("Flywheel Velocity", "%.2f", robot.flywheel1.getVelocity());
            telemetry.addData("Intake Power", "%.2f", robot.rollerIntake.getPower());
            telemetry.addData("Hotwheel Front Power", "%.2f", robot.hotwheelFront.getPower());
            telemetry.update();
        }
    }
}
