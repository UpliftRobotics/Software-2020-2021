package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends ULOpMode {

    // Declare robot and variables.
    Robot robot;
    double leftY;
    double rightX;
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
        leftY = -gamepad1.left_stick_y;
        rightX = gamepad1. right_stick_x;
        leftX = gamepad1.left_stick_x;

        // calculate power needed for each motor using the gamepad values
        leftFrontPower = leftY + leftX + rightX;
        rightFrontPower = leftY - leftX - rightX;
        leftBackPower = leftY - leftX + rightX;
        rightBackPower = leftY + leftX - rightX;

        // set the restricted powers for the respective motors
        robot.leftFront.setPower(Range.clip(leftFrontPower,-1,1));
        robot.rightFront.setPower(Range.clip(rightFrontPower,-1,1));
        robot.leftBack.setPower(Range.clip(leftBackPower,-1,1));
        robot.rightBack.setPower(Range.clip(rightBackPower,-1,1));

        // display odometry telemetry data onto the phone
//        telemetry.addData("Left Encoder Position\t", robot.leftEncoderMotor.getCurrentPosition());
////        telemetry.addData("Right Encoder Position\t", robot.rightEncoderMotor.getCurrentPosition());
////        telemetry.addData("Center Encoder Position\t", robot.centerEncoderMotor.getCurrentPosition());
////        telemetry.update();

    }
}
