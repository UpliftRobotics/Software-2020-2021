package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import org.firstinspires.ftc.teamcode.toolkit.ULOpMode;

import java.util.ArrayList;

@Autonomous(name = "PathAuto", group = "OpModes")
public class Auto extends ULLinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot();

        waitForStart();
        // declare and initialize an empty list of CurvePoints
        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        // add each CurvePoint to the ArrayList
        allPoints.add(new CurvePoint(0, 24, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2,5));

        // tell the robot to map out the path and follow it
        robot.followCurve(allPoints, Math.toRadians(90));
    }
}
