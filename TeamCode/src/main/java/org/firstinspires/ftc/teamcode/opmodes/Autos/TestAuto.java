package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "TestAuto", group = "OpModes")
public class TestAuto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {
        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        // declare and initialize an empty list of PathPoints
        ArrayList<PathPoint> allPoints = new ArrayList<>();


        allPoints.add(new PathPoint(0, 72, 1.0, 4, 5));
        allPoints.add(new PathPoint(-38, 72, 1.0, 6, 5));

        //allPoints.add(new PathPoint(0, -30, 0.5, , 10));



        //follow the path
        odom.followPath(allPoints);

        odom.stopUpdateThread();
    }
}
