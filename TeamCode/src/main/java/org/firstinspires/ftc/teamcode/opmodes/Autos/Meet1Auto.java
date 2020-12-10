package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.AutoFunctions;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "Meet 1 Auto", group = "OpModes")
public class Meet1Auto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {

        robot = new Robot();
        odom = robot.odometry;


        waitForStart();

        robot.robotStatus = "Program Running...";

        // create empty path list
        ArrayList<PathPoint> wobblePath;

        // set the initial position of the robot
        odom.setStartPosition(33, 8, 0);
        wobblePath = AutoFunctions.createWobblePath(0.7, 0.5, robot);
        if(!wobblePath.isEmpty()) {
            odom.followPath(wobblePath, MovementFunctions.SLIDE_WITHOUT_TURNS);
            wobblePath.clear();
        }

        // turn the robot in order to drop off the wobble goal
        MovementFunctions.turnTo(-180, 0.7, MovementFunctions.COUNTER_CLOCKWISE, robot);

        // put down servo
//        TeleOpFunctions.dropWobble(robot);

        // turn a little to get rid of wobble goal
        MovementFunctions.turn(30, 0.7, robot);

        // reset angle quickly
        MovementFunctions.turnTo(0, 0.7, 0, robot);

        // go to shooting position
        odom.goToPosition(40, 78, 0.7, 0, 0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);
        MovementFunctions.turnTo(-15,0.5,0, robot);
        TeleOpFunctions.shooterOn(1,1000, robot);
//        TeleOpFunctions.flickRing(robot);
        MovementFunctions.turnTo(0,0.5,0, robot);
//        TeleOpFunctions.flickRing(robot);
        MovementFunctions.turnTo(15,0.5,0, robot);
//        TeleOpFunctions.flickRing(robot);
        TeleOpFunctions.shooterOff(robot);

        // park on line
        odom.goToPosition(40,82,0.7,0,0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);

        robot.robotStatus = "Program Stopping...";
        odom.stopUpdateThread();
        odom.writePositionToFiles();

    }

}

