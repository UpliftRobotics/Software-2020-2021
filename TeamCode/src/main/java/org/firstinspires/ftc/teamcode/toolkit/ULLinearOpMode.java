package org.firstinspires.ftc.teamcode.toolkit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ULLinearOpMode extends LinearOpMode {
    private static ULLinearOpMode instance = null;

    public ULLinearOpMode() {
        super();
        instance = this;
    }

    public static ULLinearOpMode getInstance() {
        return instance;
    }
}