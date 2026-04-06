package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "tuningFlywheel", group = "LinearOpMode")
public class tuningFlywheel extends OpMode {

    //Robot robot = new Robot(hardwareMap);

    public DcMotorEx flywheel1;

    public DcMotorEx flywheel2;
    public double highVelocity = 1500;

    public double lowVelocity = 900;

    double cuTargetVelcoity2 = highVelocity;

    double F = 0;

    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};

    int stepIndex = 1;


    @Override
    public void init() {
        //hardware map
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        //directions
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        //Run using encoders
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        //PIDF coefficients
        flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("init complete");


    }


    @Override
    public void loop() {
        //get our gamepad commands
        //set target vleocity
        //update etelemetry
        if (gamepad1.yWasPressed()){
            if (cuTargetVelcoity2 == highVelocity){
                cuTargetVelcoity2 = lowVelocity;
            } else { cuTargetVelcoity2 = highVelocity; }

        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;

        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel1.setVelocity(cuTargetVelcoity2);
        flywheel2.setVelocity(cuTargetVelcoity2);

        double velocity1 = flywheel1.getVelocity();
        double velocity2 = flywheel2.getVelocity();
        double TargetVelocity = (velocity1 + velocity2) / 2.0;

        double error = cuTargetVelcoity2 -TargetVelocity;

        telemetry.addData("Target Velocity", cuTargetVelcoity2);
        telemetry.addData("Current Velocity, %.2f", TargetVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("---------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);


    }
}
