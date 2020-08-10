package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends ULOpMode {

    // Declare robot and variables.
    Robot robot;
    double rightX;
    double leftY;
    double leftX;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
        robot = new Robot();
    }

    @Override
    public void loop() {

        // initialize the gamepad stick values to the three needed axes
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1. right_stick_x;
        leftX = gamepad1.left_stick_x;

        /*
        // clip the input variables
        Range.clip(leftX, -1, 1);
        Range.clip(leftY, -1, 1);
        Range.clip(rightX, -1, 1);

        // calculate power needed for each motor using the gamepad values
        leftFrontPower = rightX + leftX + leftY;
        rightFrontPower = rightX - leftX - leftY;
        leftBackPower = rightX - leftX + leftY;
        rightBackPower = rightX + leftX - leftY;

        // set the restricted powers for the respective motors
        robot.leftFront.setPower(Range.clip(leftFrontPower,-1,1));
        robot.rightFront.setPower(Range.clip(rightFrontPower,-1,1));
        robot.leftBack.setPower(Range.clip(leftBackPower,-1,1));
        robot.rightBack.setPower(Range.clip(rightBackPower,-1,1));
        */

        // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.

        // find the angle of the left joystick
        double joystickAngle = Math.atan2(leftX , leftY);

        // find the magnitude, or hypotenuse of the left joystick and scale it to less than one
        double magnitude = Range.clip(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), 0, 1);

        // set the powers using the 2 specific equations
        robot.leftFront.setPower(Math.sin(joystickAngle + (0.25 * Math.PI)) * magnitude);
        robot.rightFront.setPower(Math.sin(joystickAngle - (0.25 * Math.PI)) * magnitude);
        robot.leftBack.setPower(Math.sin(joystickAngle - (0.25 * Math.PI)) * magnitude);
        robot.rightBack.setPower(Math.sin(joystickAngle + (0.25 * Math.PI)) * magnitude);

        // display odometry telemetry data onto the phone
//        telemetry.addData("Left Encoder Position\t", robot.leftEncoderMotor.getCurrentPosition());
//        telemetry.addData("Right Encoder Position\t", robot.rightEncoderMotor.getCurrentPosition());
//        telemetry.addData("Center Encoder Position\t", robot.centerEncoderMotor.getCurrentPosition());
//        telemetry.update();

    }
}
