package org.firstinspires.ftc.teamcode.toolkit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;

public class TelemetryOutput {
    public static void printFullTelemetry(Telemetry telemetry, Odometry odom) {
        telemetry.addData("Left Encoder pos:\t", odom.getLeftTicks() / Robot.COUNTS_PER_INCH);
        telemetry.addData("Right Encoder pos:\t", odom.getRightTicks() / Robot.COUNTS_PER_INCH);
        telemetry.addData("Center Encoder pos:\t", odom.getCenterTicks() / Robot.COUNTS_PER_INCH);
        telemetry.addData("LeftDelta:\t", odom.deltaLeftDistance);
        telemetry.addData("RightDelta:\t", odom.deltaRightDistance );
        telemetry.addData("WorldX:\t", MathFunctions.truncate(odom.worldXPosition));
        telemetry.addData("WorldY:\t", MathFunctions.truncate(odom.worldYPosition));
        telemetry.addData("WorldOrientationAngle\t", Math.toDegrees(odom.worldAngle));
        telemetry.update();
    }

    public static void printWorldData(Telemetry telemetry, Odometry odom) {
        telemetry.addData("WorldX:\t", MathFunctions.truncate(odom.worldXPosition));
        telemetry.addData("WorldY:\t", MathFunctions.truncate(odom.worldYPosition));
        telemetry.addData("WorldOrientationAngle\t", Math.toDegrees(odom.worldAngle));
        telemetry.update();
    }

}
