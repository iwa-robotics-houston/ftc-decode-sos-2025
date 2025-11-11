package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
Contains robot build, state, and transformation functions
*/

public class Robot {

    // motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;
    public DcMotorEx flywheel1;
    public DcMotorEx flywheel2;

    public DcMotorEx rollitbackBottom;

    // servos
    public CRServo hotwheelFront;
    public CRServo hotwheelBack;

    public CRServo rollitbackTop;
    public DcMotor rollerIntake;

    public Robot(HardwareMap hardwareMap) {
        // Init hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        rollerIntake = hardwareMap.get(DcMotor.class, "rollerIntake");

        hotwheelFront = hardwareMap.get(CRServo.class, "hotwheelFront");
        hotwheelBack = hardwareMap.get(CRServo.class, "hotwheelBack");

        rollitbackBottom = hardwareMap.get(DcMotorEx.class, "rollitbackBottom");
        rollitbackTop = hardwareMap.get(CRServo.class, "rollitbackTop");


        // Configure drive motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Flywheels
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        // Configure other motors and servos
        rollerIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        hotwheelFront.setDirection(DcMotorSimple.Direction.FORWARD);
        hotwheelBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rollitbackBottom.setDirection(DcMotorEx.Direction.FORWARD); // Set direction for the new parts
        rollitbackTop.setDirection(DcMotorSimple.Direction.FORWARD);  // Set direction for the new parts
    }
}
