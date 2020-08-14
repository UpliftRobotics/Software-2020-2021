package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import org.firstinspires.ftc.teamcode.toolkit.ULOpMode;

import java.util.ArrayList;

@Autonomous(name = "PathAuto", group = "OpModes")
public class Auto extends ULLinearOpMode {

    Robot robot = new Robot();
    OdometryGlobalPositionUpdate od = new OdometryGlobalPositionUpdate(robot, 76.5954101038, 100);

    @Override
    public void runOpMode() {
        waitForStart();
        // declare and initialize an empty list of CurvePoints
        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        // add each CurvePoint to the ArrayList
        allPoints.add(new CurvePoint(2.258823529454284, 44.04705882435854, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2));
        allPoints.add(new CurvePoint(9.31764705899892, 43.200000000813176, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2));
        allPoints.add(new CurvePoint(15.247058823816415, 38.11764705954104, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2));
        allPoints.add(new CurvePoint(20.61176470627034, 33.0352941182689, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2));
        allPoints.add(new CurvePoint(25.976470588724265, 28.235294118178548, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2));
        allPoints.add(new CurvePoint(31.90588235354176, 24.000000000451767, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2));
        allPoints.add(new CurvePoint(38.68235294190461, 21.17647058863391, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2));
        allPoints.add(new CurvePoint(45.74117647144925, 19.764705882724982, 0.7, 0.5, 2.0, 5.0, 0.5235987755982988, 0.2));

        // tell the robot to map out the path and follow it
        robot.followCurve(allPoints, Math.toRadians(90));
    }
}
