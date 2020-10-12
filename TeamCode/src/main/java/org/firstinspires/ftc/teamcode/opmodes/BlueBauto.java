package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;


@Autonomous(name = "BlueBauto", group= "Opmodes" )

public class BlueBauto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode(){
        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        ArrayList<PathPoint> allPoints = new ArrayList<>();

        allPoints.add(new PathPoint(0, 72, 1.0, 4, 5));
    }
}
