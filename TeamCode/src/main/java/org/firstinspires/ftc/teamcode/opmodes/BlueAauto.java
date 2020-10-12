package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "BlueAauto", group = "OpModes")

public class BlueAauto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {
        robot = new Robot ();
        odom = new Odometry (robot);

        waitForStart();

        ArrayList<PathPoint> allPoints = new ArrayList<>();

        allPoints.add(new PathPoint(0, 72, 0.7, 4, 5));
        allPoints.add(new PathPoint(-38, 72, 0.7, 4, 5));

        odom.followPath(allPoints);
    }
}



