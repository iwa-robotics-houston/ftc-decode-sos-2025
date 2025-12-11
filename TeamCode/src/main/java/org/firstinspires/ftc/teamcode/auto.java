package org.firstinspires.ftc.teamcode;

import static com.sun.tools.doclint.Entity.and;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Timer;


@Autonomous(name = "auto", group = "OpMode")
public class auto extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        // Create robot instance (loads all motors & servos)
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;


        // DRIVE FORWARD 2 SECONDS
        //driveAll(0.3);   // 30% power forward
        //sleep(2000);

        // STOP
        //driveAll(0);
        //sleep(1000);


        // DRIVE BACKWARD 2 SECONDS
        driveAll(-0.6);
        sleep(3500);
        driveAll(0);
        fireINtheHole(5600);
        sleep(3000);
        backitup(1);
        sleep(2000);
        fireINtheHole(0);
        strafeleft(0.3);
        sleep(1000);
        driveAll(0);
    }

    /**
     * Helper method to power every drive motor on the robot
     */
    private void driveAll(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);





    }
    private void fireINtheHole(double ticksperRotation) {


        robot.flywheel1.setVelocity(-ticksperRotation);
        robot.flywheel2.setVelocity(-ticksperRotation);
    }
        private void backitup(double power) {
            robot.hotwheelsback.setPower(-power);
            robot.rollitbackbottom.setPower(power);
            robot.rollitbacktop.setPower(-power);


        }
    private void strafeleft(double totheleft) {


        robot.leftFrontDrive.setPower(-totheleft);
        robot.rightFrontDrive.setPower(totheleft);
        robot.leftBackDrive.setPower(-totheleft);
        robot.rightBackDrive.setPower(totheleft);
    }

}





