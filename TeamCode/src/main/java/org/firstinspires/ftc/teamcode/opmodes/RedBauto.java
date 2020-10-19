package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "RedBauto", group = "OpModes")

public class RedBauto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode(){
        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        ArrayList<PathPoint> allPoints = new ArrayList<>();

        allPoints.add(new PathPoint(0, 86, 0.7, 2, 5));
        //allPoints.add(new PathPoint(20, 86, 0.3, 2, 5));

        odom.followPath(allPoints);
    }
}
