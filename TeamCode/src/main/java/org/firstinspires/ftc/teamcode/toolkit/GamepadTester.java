package org.firstinspires.ftc.teamcode.toolkit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "GamepadTester", group = "OpModes")
public class GamepadTester extends ULOpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot();
    }

    @Override
    public void loop() {
        telemetry.addData("Gamepad1 Left Joystick Y:   ", gamepad1.left_stick_y);
        telemetry.addData("Gamepad1 Left Joystick X:   ", gamepad1.left_stick_x);
        telemetry.addData("Gamepad1 Right Joystick Y:   ", gamepad1.right_stick_y);
        telemetry.addData("Gamepad1 Right Joystick X:   ", gamepad1.right_stick_x);
        telemetry.update();
    }
}