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
        if(robot.detectorAuto.ringCount == 0) {
            path.clear();
            odom.setStartPosition(53, 8);
            path.add(new PathPoint(53, 84, 0.7, 2, 5));
            path.add(new PathPoint(10, 84, 0.7, 2, 5));
        } else if(robot.detectorAuto.ringCount == 1) {
            path.clear();
            odom.setStartPosition(53, 8);
            path.add(new PathPoint(53, 108, 0.7, 2, 5));
            path.add(new PathPoint(34, 108, 0.7, 2, 5));
        } else if(robot.detectorAuto.ringCount == 4) {
            path.clear();
            odom.setStartPosition(53, 8);
            path.add(new PathPoint(53, 132, 0.7, 2, 5));
            path.add(new PathPoint(10, 132, 0.7, 2, 5));
        } else {
            // do nothing, something is VERY WRONG!
        }

        telemetry.addData("Top Rectangle Raw Value", (int) robot.detectorAuto.TopMatRaw);
        telemetry.addData("Top Rectangle Percent", Math.round(robot.detectorAuto.TopMatValue * 100) + "%");
        telemetry.addData("Bottom Rectangle Raw Value", (int) robot.detectorAuto.BottomMatRaw);
        telemetry.addData("Bottom Rectangle Percent", Math.round(robot.detectorAuto.BottomMatValue * 100) + "%");
        telemetry.addData("Number of Rings Detected", robot.detectorAuto.ringCount);
        telemetry.update();

        // follow the path designated earlier in the program (only if the path list was filled)
        if(!path.isEmpty()) {
            odom.followPath(path);
            path.clear();
        }

        odom.stopUpdateThread();

    }
}
