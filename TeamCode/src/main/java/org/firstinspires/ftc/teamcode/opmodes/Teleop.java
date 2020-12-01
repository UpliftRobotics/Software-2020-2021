package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.TelemetryOutput;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends ULLinearOpMode {

    // declare robot and variables.
    Robot robot;
    Odometry odom;

    // declare joystick values
    double rightX;
    double leftY;
    double leftX;


    @Override
    public void runOpMode() {

        robot = new Robot();
        odom = robot.odometry;
        odom.worldXPosition = Double.parseDouble(ReadWriteFile.readFile(odom.odometryFileWorldX).trim());
        odom.worldYPosition = Double.parseDouble(ReadWriteFile.readFile(odom.odometryFileWorldY).trim());
        odom.worldAngle = Double.parseDouble(ReadWriteFile.readFile(odom.odometryFileWorldAngle).trim());

        waitForStart();

        while(opModeIsActive()) {
            // initialize the gamepad stick values to the three needed axes
            leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
            rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
            leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

            // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.

            // find the angle of the left joystick
            double joystickAngle = Math.toDegrees(MathFunctions.atan2UL(leftY, leftX));

            // find the magnitude, or hypotenuse of the left joystick and scale it down by dividing by the max
            double magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

            // find the turnValue directly from the rightX input value (scaled for smoothness)
            double turnValue = 0.75 * rightX;

            // set the powers using the 2 specific equations and clip the result
            MovementFunctions.driveTowards(magnitude, joystickAngle, turnValue, robot);

        }

        odom.stopUpdateThread();

    }
}
