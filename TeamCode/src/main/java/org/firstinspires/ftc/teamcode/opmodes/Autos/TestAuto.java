package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "TestAuto", group = "OpModes")
public class TestAuto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {
        robot = new Robot();
        odom = robot.odometry;

        waitForStart();

        // declare and initialize an empty list of PathPoints
        ArrayList<PathPoint> path = new ArrayList<>();


        path.add(new PathPoint(0, 72, 1.0, 4, 5));


        // follow the path designated earlier in the program (only if the path list was filled)
        if(!path.isEmpty()) {
            odom.followPath(path);
            path.clear();
        }

        odom.stopUpdateThread();
    }
}
