package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
Contains robot build, state, and transformation functions.
*/

public class Robot {

    // motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;
    public DcMotorEx flywheel1;

    // servos
    public CRServo hotwheelFront;
    public CRServo hotwheelBack;
    public CRServo rollitbackBottom;
    public CRServo rollitbackTop;
    public DcMotor rollerIntake;

    public Robot(HardwareMap hardwareMap) {
        // init hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        flywheel1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        rollerIntake = hardwareMap.get(DcMotor.class, "imHungy");

        hotwheelFront = hardwareMap.get(CRServo.class, "hotwheelFront");
        hotwheelBack = hardwareMap.get(CRServo.class, "hotwheelBack");

        rollitbackBottom = hardwareMap.get(CRServo.class, "rollitbackBottom");
        rollitbackTop = hardwareMap.get(CRServo.class, "rollitbacktop");


        // configure drive motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Flywheel 1
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);

        // Configure other motors and servos
        rollerIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        hotwheelFront.setDirection(DcMotorSimple.Direction.FORWARD);
        hotwheelBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rollitbackBottom.setDirection(DcMotorSimple.Direction.FORWARD); // Set direction for the new parts
        rollitbackTop.setDirection(DcMotorSimple.Direction.FORWARD);  // Set direction for the new parts
    }
}
