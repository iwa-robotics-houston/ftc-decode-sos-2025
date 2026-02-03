package org.firstinspires.ftc.teamcode.pedroPathing;



import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.00);
            /*Note to self for tuning for newbies.  if your speed goes too fast! check the weight by m
            measuring your weight first and then measuring your weight with the roboot and subtract
            total weight by your weight! :)  total - your weight = robot weight!
             */
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD);
            //.yVelocity(82.67515770571744)

            //.xVelocity(85.09062101056136);
            //Forward veloctiy : 85.09062101056136\
            //Forward vleocity 102.75026814984247
            //Strafe veloctiy: 71.56640154382777
            //82.15165315036616



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    private static Object multiplier;
    private static Object velocity;
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("frontLeft")
            .strafeEncoder_HardwareMapName("backRight")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD


                    )
            )


            //.forwardEncoderDirection(Encoder.REVERSE)
            //potential forward value: 0.003144865909788695
            //.strafeEncoderDirection(Encoder.REVERSE);
            .forwardTicksToInches(0.003144865909788695)
            .strafeTicksToInches(0.002780111075841615)
            //0.09151578291380659

            // 0.002747749553064749
            // 0.002780111075841615


            .forwardPodY(-1)
            .strafePodX(-6.50);
            //(-7, -1)  (-7, 1)

            /*also very important do not put this ------> ";" next to all the following commands
            please only use that symbol once youre done with the constants.  I wish yall luck newbies😘
             */

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();


    }
}
