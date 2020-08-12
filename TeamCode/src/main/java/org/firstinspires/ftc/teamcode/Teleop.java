package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends OpMode {
    Robot robot = new Robot();
    double leftY;
    double rightX;
    double leftX;
    double fl;
    double fr;
    double bl;
    double br;

        @Override
        public void init() {

        }

        @Override
        public void loop() {
           leftY = -gamepad1.left_stick_y;
           rightX = gamepad1. right_stick_x;
           leftX = gamepad1.left_stick_x;

            fl = leftY + leftX + rightX;
            fr = leftY - leftX - rightX;
            bl = leftY - leftX + rightX;
            br = leftY + leftX - rightX;

            robot.leftFront.setPower(Range.clip(fl,-1,1));
            robot.rightFront.setPower(Range.clip(fr,-1,1));
            robot.leftBack.setPower(Range.clip(bl,-1,1));
            robot.rightBack.setPower(Range.clip(br,-1,1));







        }
}
