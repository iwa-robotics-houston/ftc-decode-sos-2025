package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Apriltagstest", group = "OpMode")
public class Apriltags extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevHubOrientationOnRobot = new RevHubOrientationOnRobot(com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(RevHubOrientationOnRobot));
    }

    @Override

    public void start() {
        limelight.start();



    }


    @Override

    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult LLResult = limelight.getLatestResult();
        if (LLResult != null && LLResult.isValid()) {
            Pose3D botPose = LLResult.getBotpose();
            telemetry.addData("tx", LLResult.getTx());
            telemetry.addData("ty", LLResult.getTy());
            telemetry.addData("ta", LLResult.getTa());



        }



    }
}
