package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Auto", group = "OpMode")
public class Auto extends OpMode {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor rollerIntake = null;

    @Override
    public void init() {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        rollerIntake = hardwareMap.get(DcMotor.class, "rollerIntake");

        /*
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)

        double axial = 0;
        double lateral = 0;
        double yaw = 0;
        drive(axial, lateral, yaw);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        telemetry.addLine("Please work");
        telemetry.addLine("PLEASE");
    }

    // Thanks to FTC16072 for sharing this code!!
    void drive( double axial, double lateral, double yaw){
        // This calculates the power needed for each wheel based on the amount of axial,
        // strafe, lateral, and yaw

        // Variables needed to know how much engine power is needed.
        // The numbers here can be recorded when playing the teleop mode
        double forward = 0.19; // SOS - RECORD VALUES
        double backward = -0.24;

        // FORWARD

        frontLeftDrive.setPower(forward);
        frontRightDrive.setPower(forward);
        backLeftDrive.setPower(forward);
        backRightDrive.setPower(forward);

        sleep(2000);

        // STOP

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        sleep(2000);

        // BACKWARDS

        frontLeftDrive.setPower(backward);
        frontRightDrive.setPower(backward);
        backLeftDrive.setPower(backward);
        backRightDrive.setPower(backward);

        sleep(2000);



    }
}