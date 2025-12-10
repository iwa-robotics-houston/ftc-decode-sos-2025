package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Robot {
    // dimensions
    final public double drivetrainDiagonal = 17; // in
    public boolean isArmClawOpen;
    public boolean isMiniClawOpen;
    double cpr = 537.7; // clicks
    double wheelCirc = 11.9; // in
    static final double vertSlideWheelCirc = Math.PI * 1.5;

    static final double armSlideWheelCirc = Math.PI * 1.5;
    // to correct movement lengths
    static final double drivetrainMultiplier = 1.5;

    // state
    public boolean rightBumperPrev = false;
    public ElapsedTime timer = new ElapsedTime();  // Create a timer instance
    public double lastIntakeTime = 0;
    public double intakeInterval = 2.0;  // Adjust timing as needed (seconds)


    // motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;

    public DcMotorEx flywheel1;

    public DcMotorEx flywheel2;

    public CRServo hotwheelsfront;

    public CRServo hotwheelsback;

    public DcMotor rollerIntake;


    //servos
    public DcMotor rollitbackbottom;

    public CRServo rollitbacktop;
    public Robot(HardwareMap hardwareMap) {
        // init hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");


        rollerIntake = hardwareMap.get(DcMotor.class, "rollerIntake");

        hotwheelsfront = hardwareMap.get(CRServo.class, "hotwheelFront");
        hotwheelsback = hardwareMap.get(CRServo.class, "hotwheelBack");
        rollitbackbottom = hardwareMap.get(DcMotor.class, "rollitbackBottom");
        rollitbacktop = hardwareMap.get(CRServo.class, "rollitbackTop");


        // configure drive motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Flywheel 1
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        rollitbackbottom.setDirection(DcMotor.Direction.FORWARD);
        rollitbacktop.setDirection(CRServo.Direction.REVERSE);

        // Configure encoders
        //Encoder code
        //flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //automatic intake!
        rollerIntake.setDirection(DcMotor.Direction.REVERSE);
        hotwheelsfront.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    }