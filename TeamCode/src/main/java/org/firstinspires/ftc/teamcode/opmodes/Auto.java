package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
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

        waitForStart();

        // declare and initialize an empty list of CurvePoints
        ArrayList<PathPoint> allPoints = new ArrayList<>();

        allPoints.add(new PathPoint(-24, 32, 0.5, 3));

        //follow the path
        odom.followPath(allPoints);
    }
}
