
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Color Sense Auto", group = "Autonomous")
public class LimonCamera extends LinearOpMode {

    private ColorSensor colorSensor;
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            int red = colorSensor.red();
            int blue = colorSensor.blue();

            telemetry.addData("Red", red);
            telemetry.addData("Blue", blue);
            telemetry.update();

            if (red > blue) {
                // RED detected → turn left
                turnLeft();
            } else if (blue > red) {
                // BLUE detected → turn right
                turnRight();
            } else {
                // No clear color → stop
                stopMotors();
            }

            sleep(1000);
            stopMotors();
        }
    }

    private void turnLeft() {
        leftDrive.setPower(-0.4);
        rightDrive.setPower(0.4);
    }

    private void turnRight() {
        leftDrive.setPower(0.4);
        rightDrive.setPower(-0.4);
    }

    private void stopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
