package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.TelemetryOutput;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends ULLinearOpMode {

    // Declare robot and variables.
    Robot robot;
    Odometry odom;

    double rightX;
    double leftY;
    double leftX;


    @Override
    public void runOpMode() {

        waitForStart();
        robot = new Robot();
        odom = new Odometry(robot);

        while(opModeIsActive()) {

            odom.positionUpdate();

            // initialize the gamepad stick values to the three needed axes
            leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
            rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
            leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

            // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.

            // find the angle of the left joystick
            double joystickAngle = Math.toDegrees(Math.atan2(leftY, leftX));

            // find the magnitude, or hypotenuse of the left joystick and scale it down by dividing by the max it could be
            double magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

            // find the turnValue directly from the rightX input value (halved for smoothness)
            double turnValue = 0.75 * rightX;

            // set the powers using the 2 specific equations and clip the result
            robot.drive(magnitude, joystickAngle, turnValue);

            // add telemetry data for the encoders
            TelemetryOutput.printFullTelemetry(telemetry, odom);

        }

    }
}
