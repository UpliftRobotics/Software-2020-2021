package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

@TeleOp(name = "GamepadTester", group = "OpModes")
public class GamepadTester extends ULLinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Gamepad1 Left Joystick Y:   ", gamepad1.left_stick_y);
            telemetry.addData("Gamepad1 Left Joystick X:   ", gamepad1.left_stick_x);
            telemetry.addData("Gamepad1 Right Joystick Y:   ", gamepad1.right_stick_y);
            telemetry.addData("Gamepad1 Right Joystick X:   ", gamepad1.right_stick_x);
            telemetry.update();
        }

    }
}