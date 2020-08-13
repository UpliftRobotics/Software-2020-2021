package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends ULOpMode {

    // Declare robot and variables.
    Robot robot;
    OdometryGlobalPositionUpdate od;
    double rightX;
    double leftY;
    double leftX;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
        od = new OdometryGlobalPositionUpdate(robot.leftFront, robot.rightFront, robot.leftBack, OdometryGlobalPositionUpdate.oneRotationTicks, 10);
        robot = new Robot();
    }

    @Override
    public void loop() {

        // initialize the gamepad stick values to the three needed axes
        leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        rightX = Range.clip(gamepad1. right_stick_x, -1, 1);
        leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

        // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.

        // find the angle of the left joystick
        double joystickAngle = Math.atan2(leftY , leftX);

        // find the magnitude, or hypotenuse of the left joystick and scale it down by dividing by the max it could be
        double magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

        // find the turnValue directly from the rightX input value
        double turnValue = 0.50 * rightX;

        // set the powers using the 2 specific equations and clip the result
        robot.leftFront.setPower(Range.clip((Math.sin(joystickAngle + (0.25 * Math.PI)) * magnitude) + turnValue, -1, 1));
        robot.rightFront.setPower(Range.clip((Math.sin(joystickAngle - (0.25 * Math.PI)) * magnitude) - turnValue, -1, 1));
        robot.leftBack.setPower(Range.clip((Math.sin(joystickAngle - (0.25 * Math.PI)) * magnitude) + turnValue, -1, 1));
        robot.rightBack.setPower(Range.clip((Math.sin(joystickAngle + (0.25 * Math.PI)) * magnitude) - turnValue, -1, 1));

        // add telemetry data for the encoders
        telemetry.addData("Left Encoder pos:\t", od.getLeftTicks());
        telemetry.addData("Right Encoder pos:\t", od.getRightTicks());
        telemetry.addData("Center Encoder pos:\t", od.getCenterTicks());
        telemetry.update();

    }
}
