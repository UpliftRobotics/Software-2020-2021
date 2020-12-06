package org.firstinspires.ftc.teamcode.opmodes.Autos;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

public class ShootingAuto extends ULLinearOpMode {
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
        path.clear();
        path.add(new PathPoint(40, 78, 0.7, 0.5));
        if(!path.isEmpty()) {
            odom.followPath(path, MovementFunctions.SLIDE_WITHOUT_TURNS);
            path.clear();
        }
        MovementFunctions.turnTo(-15,0.5,0, robot);
        TeleOpFunctions.shooterOn(1,1000,robot);
        TeleOpFunctions.shooterOff(robot);
        MovementFunctions.turnTo(0,0.5,0, robot);
        TeleOpFunctions.shooterOn(1,1000,robot);
        TeleOpFunctions.shooterOff(robot);
        MovementFunctions.turnTo(15,0.5,0, robot);
        TeleOpFunctions.shooterOn(1,1000,robot);
        TeleOpFunctions.shooterOff(robot);
        odom.goToPosition(40,82,0.7,0,0.5,MovementFunctions.SLIDE_WITHOUT_TURNS);
        robot.robotStatus = "Program Stopping...";
        odom.stopUpdateThread();
        odom.writePositionToFiles();

    }

}

