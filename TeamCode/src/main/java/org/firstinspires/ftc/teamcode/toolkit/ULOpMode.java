package org.firstinspires.ftc.teamcode.toolkit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class ULOpMode extends OpMode {
    private static ULOpMode instance = null;

    public ULOpMode() {
        super();
        instance = this;
    }

    public static ULOpMode getInstance() {
        return instance;
    }
}
