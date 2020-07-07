package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.MovementVars;


public class MyOpMode extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        OdometryMain.goToPosition(0, 0, 0.7, Math.toRadians(90), 0.3);
    }
}
