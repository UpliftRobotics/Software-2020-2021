package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "BlueVisionAuto", group = "OpModes")
public class BlueVisionAuto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        // create empty path list
        ArrayList<PathPoint> path = new ArrayList<>();

        // fill the path with the correct points, dependent on the number of rings detected
        if(robot.detector.ringCount == 1) {
            path.clear();
            odom.setStartPosition(53, 8);
            path.add(new PathPoint(53, 108, 0.7, 4, 5));
            path.add(new PathPoint(34, 108, 0.7, 4, 5));
        } else if(robot.detector.ringCount == 4) {
            path.clear();
            odom.setStartPosition(53, 8);
            path.add(new PathPoint(53, 132, 0.7, 4, 5));
            path.add(new PathPoint(10, 132, 0.7, 4, 5));
        } else {
            path.clear();
            odom.setStartPosition(53, 8);
            path.add(new PathPoint(53, 84, 0.7, 4, 5));
            path.add(new PathPoint(12, 84, 0.7, 4, 5));
        }

        telemetry.addData("Number of Rings Detected", robot.detector.ringCount);
        telemetry.update();

        // follow the path designated earlier in the program (only if the path list was filled)
        if(!path.isEmpty()) {
            odom.followPath(path);
            path.clear();
        }

        odom.stopUpdateThread();

    }

}
