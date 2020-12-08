package org.firstinspires.ftc.teamcode.toolkit;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

public class AutoFunctions {
    public static ArrayList<PathPoint> createWobblePath(double speed, double errorDistance, Robot robot) {

        // pull ring count from the detector class immediately at start
        int ringNum = robot.detector.ringCount;

        // create empty path list
        ArrayList<PathPoint> path = new ArrayList<>();

        // fill the path with the correct points, dependent on the number of rings detected
        if(ringNum == 0) {
            path.clear();
            path.add(new PathPoint(53, 84, speed, errorDistance));
            path.add(new PathPoint(36, 84, speed, errorDistance));
        } else if(ringNum == 1) {
            path.clear();
            path.add(new PathPoint(53, 108, speed, errorDistance));
            path.add(new PathPoint(60, 108, speed, errorDistance));
        } else if(ringNum == 4) {
            path.clear();
            path.add(new PathPoint(53, 132, speed, errorDistance));
            path.add(new PathPoint(36, 132, speed, errorDistance));
        } else {
            // detection did not work, so just park on the line
            path.clear();
            path.add(new PathPoint(53, 84, speed, errorDistance));
        }

        return path;
    }


}
