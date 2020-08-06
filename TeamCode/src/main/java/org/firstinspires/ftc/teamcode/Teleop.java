package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends OpMode {

    // Declare robot and variables.
    Robot robot = new Robot();
    double leftY;
    double rightX;
    double leftX;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        leftY = -gamepad1.left_stick_y;
        rightX = gamepad1. right_stick_x;
        leftX = gamepad1.left_stick_x;

        leftFrontPower = leftY + leftX + rightX;
        rightFrontPower = leftY - leftX - rightX;
        leftBackPower = leftY - leftX + rightX;
        rightBackPower = leftY + leftX - rightX;

        robot.leftFront.setPower(Range.clip(leftFrontPower,-1,1));
        robot.rightFront.setPower(Range.clip(rightFrontPower,-1,1));
        robot.leftBack.setPower(Range.clip(leftBackPower,-1,1));
        robot.rightBack.setPower(Range.clip(rightBackPower,-1,1));

    }
}
