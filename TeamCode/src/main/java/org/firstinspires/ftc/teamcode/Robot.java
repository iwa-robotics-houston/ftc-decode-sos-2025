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
    public DcMotor flywheel1;
    public DcMotor flywheel2;
    public DcMotor rollitbackBottom;

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

        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        rollerIntake = hardwareMap.get(DcMotor.class, "rollerIntake");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

        hotwheelFront = hardwareMap.get(CRServo.class, "hotwheelFront");
        hotwheelBack = hardwareMap.get(CRServo.class, "hotwheelBack");

        rollitbackBottom = hardwareMap.get(DcMotor.class, "rollitbackBottom");
        rollitbackTop = hardwareMap.get(CRServo.class, "rollitbackTop");


        // Configure drive motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Flywheels
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);

        // Configure other motors and servos
        rollerIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        hotwheelFront.setDirection(DcMotorSimple.Direction.REVERSE);
        hotwheelBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rollitbackBottom.setDirection(DcMotorEx.Direction.REVERSE); // Set direction for the new parts
        rollitbackTop.setDirection(DcMotorSimple.Direction.FORWARD);  // Set direction for the new parts
    }
}
