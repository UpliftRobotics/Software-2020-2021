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

        waitForStart();

        // create empty path list
        ArrayList<PathPoint> path = new ArrayList<>();

        // fill the path with the correct points, dependent on the number of rings detected
        if(robot.detector.ringCount == 0) {
            path.clear();
            path.add(new PathPoint(53, 132, 0.7, 2, 5));
            path.add(new PathPoint(53, 60, 0.7, 2, 5));
            path.add(new PathPoint(10, 60, 0.7, 2, 5));
        } else if(robot.detector.ringCount == 1) {
            path.clear();
            path.add(new PathPoint(53, 132, 0.7, 2, 5));
            path.add(new PathPoint(53, 36, 0.7, 2, 5));
            path.add(new PathPoint(34, 36, 0.7, 2, 5));
        } else if(robot.detector.ringCount == 4) {
            path.clear();
            path.add(new PathPoint(53, 132, 0.7, 2, 5));
            path.add(new PathPoint(53, 12, 0.7, 2, 5));
            path.add(new PathPoint(10, 12, 0.7, 2, 5));
        } else {
            // do nothing, something is VERY WRONG!
        }

        telemetry.addData("Top Rectangle Raw Value", (int) robot.detector.TopMatRaw);
        telemetry.addData("Top Rectangle Percent", Math.round(robot.detector.TopMatValue * 100) + "%");
        telemetry.addData("Bottom Rectangle Raw Value", (int) robot.detector.BottomMatRaw);
        telemetry.addData("Bottom Rectangle Percent", Math.round(robot.detector.BottomMatValue * 100) + "%");
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
