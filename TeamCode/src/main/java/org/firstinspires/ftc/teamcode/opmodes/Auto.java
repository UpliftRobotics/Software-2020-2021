package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.Point;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "PathAuto", group = "OpModes")
public class Auto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {
        robot = new Robot();
        odom = new Odometry(robot);
        odom.setStartPosition(new Point(0, 0), 0);

        waitForStart();

        // declare and initialize an empty list of CurvePoints
        ArrayList<PathPoint> allPoints = new ArrayList<>();

        allPoints.add(new PathPoint(0, 72, 0.7, 3, 5));
        allPoints.add(new PathPoint(-36, 72, 0.7, 3, 5));
        allPoints.add(new PathPoint(15, 120, 0.7, 3, 5));
        allPoints.add(new PathPoint(52, 105, 0.7, 3, 5));
        allPoints.add(new PathPoint(-20, 30, 0.7, 3, 5));
        allPoints.add(new PathPoint(-36, 15, 0.7, 3, 5));
        allPoints.add(new PathPoint(21, 72, 0.7, 3, 5));
        allPoints.add(new PathPoint(0, 0, 0.7, 3, 5));
        //follow the path
        odom.followPath(allPoints);
    }
}
