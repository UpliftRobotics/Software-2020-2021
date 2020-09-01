package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.toolkit.ULOpMode;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends ULOpMode {

    // Declare robot and variables.
    Robot robot;
    double rightX;
    double leftY;
    double leftX;

    @Override
    public void init() {
        robot = new Robot();
    }

    @Override
    public void loop() {

        robot.globalCoordinatePositionUpdate();

        // initialize the gamepad stick values to the three needed axes
        leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        rightX = Range.clip(gamepad1. right_stick_x, -1, 1);
        leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

        // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.

        // find the angle of the left joystick
        double joystickAngle = Math.atan2(leftY , leftX);

        // find the magnitude, or hypotenuse of the left joystick and scale it down by dividing by the max it could be
        double magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

        // find the turnValue directly from the rightX input value (halved for smoothness)
        double turnValue = 0.75 * rightX;

        // set the powers using the 2 specific equations and clip the result
        robot.slideDirection(magnitude, joystickAngle, turnValue);

        // add telemetry data for the encoders
        telemetry.addData("Left Encoder pos:\t", robot.getLeftTicks()/robot.COUNTS_PER_INCH);
        telemetry.addData("Right Encoder pos:\t", robot.getRightTicks()/robot.COUNTS_PER_INCH);
        telemetry.addData("Center Encoder pos:\t", robot.getCenterTicks()/robot.COUNTS_PER_INCH);
        telemetry.addData("WorldX:\t", robot.worldXPosition);
        telemetry.addData("WorldY:\t", robot.worldYPosition);
        telemetry.addData("WorldOrientationAngle\t", robot.getWorldAngle_rad());
        telemetry.update();

    }
}
