package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "AutoRedC", group = "OpModes")

public class AutoRedC extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode(){
        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        ArrayList<PathPoint> allPoints = new ArrayList<>();

        allPoints.add(new PathPoint(0, 124, 0.7, 2, 5));
        //allPoints.add(new PathPoint(42, 124, 0.5, 2, 5));

        odom.followPath(allPoints);

        odom.stopUpdateThread();
    }

}
