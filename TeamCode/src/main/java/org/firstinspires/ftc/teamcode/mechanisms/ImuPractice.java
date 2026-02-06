package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class ImuPractice extends OpMode {

    TestBenchIMU bench = new TestBenchIMU();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {

        double heading = bench.getHeading(AngleUnit.DEGREES);

        double y = -gamepad1.left_stick_y;  // forward
        double x = gamepad1.left_stick_x;   // strafe

        // Field-centric rotation
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        telemetry.addData("Heading (deg)", bench.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Field X", rotX);
        telemetry.addData("Field Y", rotY);

        // For now just spin your test motor based on rotY
        bench.setMotor(rotY);
    }
}