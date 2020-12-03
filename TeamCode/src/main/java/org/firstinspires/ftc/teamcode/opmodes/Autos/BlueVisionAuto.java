package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.io.File;
import java.util.ArrayList;

@Autonomous(name = "BlueVisionAuto", group = "OpModes")
public class BlueVisionAuto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {

        robot = new Robot();
        odom = robot.odometry;


        waitForStart();

        robot.robotStatus = "Program Running...";

        // pull ring count from the detector class immediately at start
        int ringNum = robot.detector.ringCount;

        // create empty path list
        ArrayList<PathPoint> path = new ArrayList<>();

        // set the initial position of the robot
        odom.setStartPosition(53, 8, 0);

        // fill the path with the correct points, dependent on the number of rings detected
        if(ringNum == 0) {
            path.clear();
            path.add(new PathPoint(53, 84, 0.7, 0.5));
            path.add(new PathPoint(12, 84, 0.7, 0.5));
        } else if(ringNum == 1) {
            path.clear();
            path.add(new PathPoint(53, 108, 0.7, 0.5));
            path.add(new PathPoint(34, 108, 0.7, 0.5));
        } else if(ringNum == 4) {
            path.clear();
            path.add(new PathPoint(53, 132, 0.7, 0.5));
            path.add(new PathPoint(10, 132, 0.7, 0.5));
        } else {
            // detection did not work, so just park on the line
            path.clear();
            path.add(new PathPoint(53, 84, 0.7, 0.5));
        }

        // follow the path designated earlier in the program (only if the path list was filled)
        if(!path.isEmpty()) {
            odom.followPath(path, MovementFunctions.SLIDE_WITHOUT_TURNS);
            path.clear();
        }

        robot.robotStatus = "Program Stopping...";
        odom.stopUpdateThread();
        odom.writePositionToFiles();

    }

}