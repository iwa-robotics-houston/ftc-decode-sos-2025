package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the primary TeleOp for the 2025-2026 DECODE season.
 *
 * CONTROLS:
 * Gamepad 1 (Driver):
 *  - Left Stick:  Strafe (Left/Right) and Drive (Forward/Backward)
 *  - Right Stick: Rotate (Turn Left/Right)
 *
 * Gamepad 2 (Operator):
 *  - Left Trigger:  Run roller intake forward (collect)
 *  - Left Bumper:   Run roller intake backward (expel)
 *  - Right Trigger: Spin up flywheel and run feeder system forward (LAUNCH)
 *  - Right Bumper:  Spin up flywheel and run feeder system in reverse (unjam)
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
        // This loop runs continuously after START is pressed, until STOP is pressed.
        while (opModeIsActive()) {

            // Gamepad 1: Drivetrain Controls
            double max;

            // POV Mode uses left joystick to go forward/backward and strafe, and right joystick to rotate.
            // This is the standard Mecanum drive control scheme.
            double axial   = -gamepad1.left_stick_y;  // Forward/Backward. Pushing stick forward gives a negative value.
            double lateral =  gamepad1.left_stick_x;  // Strafe Left/Right
            double yaw     =  gamepad1.right_stick_x; // Rotation

            // Combine the joystick requests for each axis-motion to determine each wheel's power
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion direction, just at a lower speed.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to the wheels
            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightBackDrive.setPower(rightBackPower);


            // Gamepad 2: Mechanism Controls
            double ticksPerRotation = 2800; // Example value
            double launchVelocity = gamepad2.right_trigger * ticksPerRotation;

            // LAUNCH MODE
            // Pressing the right trigger spins up the flywheel AND runs the intake and boot feeders.
            if (gamepad2.right_trigger > 0) {
                // Launcher active
                robot.flywheel1.setVelocity(launchVelocity);
                robot.boot.setPower(gamepad2.right_trigger);
                robot.boot2.setPower(gamepad2.right_trigger);
                // Also run the main intake to help feed
                robot.rollerIntake.setPower(gamepad2.right_trigger);
            }
            // REVERSE/UNJAM MODE
            // Pressing the right bumper reverses ALL motors in the system.
            else if (gamepad2.right_bumper) {
                // Reverse mode active
                robot.flywheel1.setVelocity(0); // Flywheel should not spin in reverse
                robot.boot.setPower(-1);
                robot.boot2.setPower(-1);
                robot.rollerIntake.setPower(-1);
            }
            // INTAKE ONLY MODE
            // Pressing the left trigger ONLY runs the roller intake.
            else if (gamepad2.left_trigger > 0) {
                // Intake only mode active
                robot.rollerIntake.setPower(1.0);
                // Ensure other motors are off
                robot.flywheel1.setVelocity(0);
                robot.boot.setPower(0);
                robot.boot2.setPower(0);
            }
            // IDLE MODE
            // If no buttons are pressed, turn everything off.
            else {
                robot.flywheel1.setVelocity(0);
                robot.boot.setPower(0);
                robot.boot2.setPower(0);
                robot.rollerIntake.setPower(0);
            }


            // TELEMETRY
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("--- Drivetrain ---", "");
            telemetry.addData("Front Wheels", "Left: (%.2f), Right: (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("Back Wheels", "Left: (%.2f), Right: (%.2f)", leftBackPower, rightBackPower);
            telemetry.addData("--- Launcher ---", "");
            telemetry.addData("Flywheel Target Velocity", robot.flywheel1.getVelocity());
            telemetry.addData("Boot Power", robot.boot.getPower());
            telemetry.update();
        }
    }
}
