package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
@Disabled
@TeleOp(name = "servoTest",group = "test")
public class ServoTest extends ULLinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        waitForStart();
        robot.robotStatus = "Program Running...";
        while (opModeIsActive()) {
            //            if(gamepad2.dpad_up){
//                robot.constant += 0.05;
//            }
//            if(gamepad2.dpad_down){
//                robot.constant -= 0.05;
//            }
//            if(gamepad2.y){
//                robot.wobble.setPosition(robot.constant);
//            }
//
            // flicker servo tester
            if(gamepad1.dpad_up){
                robot.constant += 0.05;
            }
            if(gamepad1.dpad_down){
                robot.constant -= 0.05;
            }
            if(gamepad1.y){
                robot.flicker.setPosition(robot.constant);
            }


        }
    }
}
