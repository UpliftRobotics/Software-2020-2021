package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.TelemetryOutput;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;


@Disabled
@TeleOp(name = "TestTeleop", group = "Opmodes")
public class TestTeleop extends ULLinearOpMode {

    // declare robot and variables.
    Robot robot;
    Odometry odom;

    // declare joystick values
    double rightX;
    double leftY;
    double leftX;

    DcMotor shooter1;
    DcMotor shooter2;
    DcMotor intake;

    @Override
    public void runOpMode() {

        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        while(opModeIsActive()) {

            Log.i("Thread", "OG THREAD Working");

            if(!opModeIsActive()) {
                odom.updateValid = false;
            }

//            odom.positionUpdate();

            // initialize the gamepad stick values to the three needed axes
            leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
            rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
            leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

            shooter1 = hardwareMap.get(DcMotor.class, "shooter_1");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter_2");

            intake = hardwareMap.get(DcMotor.class, "intake");

            // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.

            // find the angle of the left joystick
            double joystickAngle = Math.toDegrees(Math.atan2(leftY, leftX));

            // find the magnitude, or hypotenuse of the left joystick and scale it down by dividing by the max
            double magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

            // find the turnValue directly from the rightX input value (scaled for smoothness)
            double turnValue = 0.75 * rightX;

            // set the powers using the 2 specific equations and clip the result
            MovementFunctions.driveTowards(magnitude, joystickAngle, turnValue, robot);

            // add telemetry data for the encoders
            TelemetryOutput.printFullTelemetry(telemetry, robot);

            if (gamepad2.a) {
                shooter1.setPower(1);
                shooter2.setPower(1);

            }
            if (gamepad2.b) {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }
            intake.setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
        }

    }
}


