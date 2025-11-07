

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "LinearOpMode")
// TeleOp for claw bot with Mecanum drive
public class Main2 extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        //wait for the game to start (driver presses START)
        telemetry.addData("status", "initialized");
        //telemetry.addData("Servo Power", robot.intakeServo1.getPower());
        //telemetry.addData("Servo Position", wristServo.getPosition());
        telemetry.update();
        waitForStart();
        runtime.reset();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            boolean armLocked = false;


            //For flywheel functions: launching artifact
            double ticksPerRotation = 2800;
            double IshowSpeed = gamepad2.right_trigger;
            //robot.flywheel2.setVelocity(IshowSpeed * ticksPerRotation);


            /*/for automatic intake:
            double orbitPerRotation = 2792.83;
            double IshowSpeed2 = gamepad2.right_trigger;
            robot.rollerIntake.setVelocity(orbitPerRotation * IshowSpeed2);
            */
            //Set power values

            double intakePower = 1;
            double compliantWheel = 1;
            double launchPower = (IshowSpeed * ticksPerRotation);
            double pollie = 1;
            double rollie = 1;
            double hottie = 1;

            //operate
            if (gamepad2.left_trigger > 0) {
                robot.rollerIntake.setPower(-intakePower);
                robot.hotwheelsfront.setPower(compliantWheel);
                //robot.hotwheelsback.setPower(hottie);
            } else if (gamepad2.left_bumper) {
                robot.rollerIntake.setPower(intakePower);
                robot.hotwheelsfront.setPower(-compliantWheel);
            } else {
                robot.rollerIntake.setPower(0);
                robot.hotwheelsfront.setPower(0);
            }


            if (gamepad2.right_trigger > 0) {
                robot.flywheel1.setVelocity(-launchPower);
                //robot.rollitbackbottom.setPower(pollie);
                //robot.rollitbacktop.setPower(rollie);
                //robot.hotwheelsback.setPower(hottie);
            }

            else if (gamepad2.right_bumper) {
                robot.flywheel1.setVelocity(launchPower);
                //robot.rollitbackbottom.setPower(-pollie);
                //robot.hotwheelsback.setPower(hottie);
                //robot.rollitbacktop.setPower(-rollie);}
            }
            else {
                robot.flywheel1.setVelocity(0);
                //robot.rollitbackbottom.setPower(0);
                //robot.hotwheelsback.setPower(0);
                //robot.rollitbacktop.setPower(0);}
            }

            //combine the joystick requests for each axis-motion to determine each wheel's power
            //set up a variable for each drive wheel to save the power level for telemetry

            //POV Mode uses left joystick to go forward & strafe, and right joystick to rotate
            //Joystick controls

            double axial = -gamepad1.left_stick_y; //note: pushing stick forward gives negative value
            //NOTE: I am making sure they are not in inverse.
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            //normalize the values so no wheel power exceeds 100%
            //this ensures that the robot maintains the desired motion
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            //send calculated power to wheels
            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightBackDrive.setPower(rightBackPower);


            // Auto Intake
            // Example: Timed Intake Cycle (opens and closes every X seconds)
            //Timed cycle: Alternates between open and closed states at a set time interval.
            double currentTime = robot.timer.seconds();
            /*
            if (currentTime - robot.lastIntakeTime > robot.intakeInterval) {
                robot.isArmClawOpen = !robot.isArmClawOpen;
                robot.lastIntakeTime = currentTime;  // Reset timer
            }


            }





             */

            //show the elapsed game time and wheel power.
            telemetry.addData("status", "Run Time:" + runtime);
            telemetry.addData("Front left/Right", "%4.2f,%4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/Right", "%4.2f,%4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("Wrist Pos", "%4.2f", wristServo.getPosition());
            //telemetry.addData("Claw Power", "%4.2f", robot.intakeServo1.getPower());
            //telemetry.addData("robot.arm Pos", robot.arm1.getCurrentPosition());
            telemetry.update();
        }
    }
}



