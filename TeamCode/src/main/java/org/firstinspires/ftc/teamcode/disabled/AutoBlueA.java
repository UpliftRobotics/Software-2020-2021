package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "AutoBlueA", group = "OpModes")

public class AutoBlueA extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {
        robot = new Robot ();
        odom = robot.odometry;

        waitForStart();

        ArrayList<PathPoint> allPoints = new ArrayList<>();

        allPoints.add(new PathPoint(0, 76, 0.7, 4, 5));
        allPoints.add(new PathPoint(-30, 76, 0.7, 4, 5));
        allPoints.add(new PathPoint(0, 76, 0.7, 4, 5));
        //allPoints.add(new PathPoint(-38, 72, 0.5, 2, 5));

        odom.followPath(allPoints);

        odom.stopUpdateThread();
    }
}



