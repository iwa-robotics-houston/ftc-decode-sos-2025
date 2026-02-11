package org.firstinspires.ftc.teamcode.pedroPathing;



import static com.sun.tools.javac.code.Lint.LintCategory.PATH;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.00)
            .forwardZeroPowerAcceleration(-53.377519196560364)
            .lateralZeroPowerAcceleration(-83.93407971504556)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.01));

            //-51.12524709535485
            //-53.377519196560364
            //-50.452787535754894
            //Lateral/strafe
            //-85.39521976747464
            //-90.91036473209661
            //-77.85728007406766

            //65
            //-84.6650549622675
            //-84.48754623127735
            //-89.4689939877809
            //-90.68147976240056
            //-104.6493394218943

            //54
            //-85.43610073794079
            //-88.86476093437331
            //-83.93407971504556
            //-87.12870335621824
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
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)

            .xVelocity(101.97251337366836)
            .yVelocity(72.10190712744952);
            //Forward veloctiy : 85.09062101056136\
            //Forward vleocity 102.75026814984247
            //101.97251337366836
            //101.97251337366836
            //101.74081921170222
            //Strafe veloctiy: 71.56640154382777
            //STRAFE velocti:  67.45599256907278
            //Strafe velocity: 72.10190712744952
            //73.02420103674652
            //70.44138898580954






    /*public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            2.0,
            2.0);*/



   /*public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            4.0,
            10,
            1
    );*/
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
                //.pathConstraints(pathConstraints)

                .mecanumDrivetrain(driveConstants)
                .build();


    }
}
