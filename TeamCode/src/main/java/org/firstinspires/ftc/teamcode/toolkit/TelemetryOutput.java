package org.firstinspires.ftc.teamcode.toolkit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;

public class TelemetryOutput {

    public static void printFullTelemetry(Telemetry telemetry, Robot robot) {
        telemetry.addData("WorldX:\t", MathFunctions.truncate(robot.odometry.worldXPosition));
        telemetry.addData("WorldY:\t", MathFunctions.truncate(robot.odometry.worldYPosition));
        telemetry.addData("WorldOrientationAngle\t", robot.odometry.worldAngle);
        telemetry.addData("Left Encoder pos:\t", robot.odometry.getLeftTicks() / Robot.COUNTS_PER_INCH);
        telemetry.addData("Right Encoder pos:\t", robot.odometry.getRightTicks() / Robot.COUNTS_PER_INCH);
        telemetry.addData("Center Encoder pos:\t", robot.odometry.getCenterTicks() / Robot.COUNTS_PER_INCH);
        telemetry.addData("Ring Count:\t", robot.detector.ringCount);
        telemetry.update();
    }

    public static void printWorldData(Telemetry telemetry, Robot robot) {
        telemetry.addData("Robot Status:\t", robot.robotStatus);
        telemetry.addData("WorldX:\t", MathFunctions.truncate(robot.odometry.worldXPosition));
        telemetry.addData("WorldY:\t", MathFunctions.truncate(robot.odometry.worldYPosition));
        telemetry.addData("WorldAngle\t", robot.odometry.worldAngle);
        telemetry.addData("Ring Count:\t", robot.detector.ringCount);
        telemetry.addData("Servo Val", robot.wobble.getPosition());
        telemetry.update();
    }

}
