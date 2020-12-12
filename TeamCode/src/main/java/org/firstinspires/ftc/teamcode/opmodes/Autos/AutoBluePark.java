package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "AutoBluePark", group = "OpModes")
public class AutoBluePark extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {

        robot = new Robot();
        odom = robot.odometry;

        waitForStart();

        TeleOpFunctions.releaseLatch(robot);

        robot.robotStatus = "Program Running...";

        // create empty path list
        ArrayList<PathPoint> path = new ArrayList<>();

        // set the initial position
        odom.setStartPosition(29, 0.5, 0);

        // fill path with points
        path.add(new PathPoint(43, 24.5, 0.7, 0.5));
        path.add(new PathPoint(43, 72, 0.7, 0.5));


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
