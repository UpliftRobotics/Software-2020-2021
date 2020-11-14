package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "AutoBluePark", group = "OpModes")
public class AutoBluePark extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        // create empty path list
        ArrayList<PathPoint> path = new ArrayList<>();

        // fill path with points
        path.add(new PathPoint(0, 76, 0.7, 4, 5));


        // follow the path designated earlier in the program (only if the path list was filled)
        if(!path.isEmpty()) {
            odom.followPath(path);
            path.clear();
        }


        odom.stopUpdateThread();

    }

}
