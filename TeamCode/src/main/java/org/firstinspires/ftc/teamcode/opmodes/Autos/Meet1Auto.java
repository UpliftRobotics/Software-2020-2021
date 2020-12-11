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

        odom.setStartPosition(36, 8, 0);

        // shift to the right
        odom.goToPosition(60, 8, 0.7, 0, 0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);

        // go to shooting position
        odom.goToPosition(60, 76, 0.7, 0, 0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);

        // put the transfer system up
        TeleOpFunctions.transferUp(robot);

        // turn on shooter
        TeleOpFunctions.shooterOn(1, robot);

        // turn towards the left powershot target
        MovementFunctions.turnTo(-15,0.5,0, robot);

        // flick the ring into the shooter
        TeleOpFunctions.flickRing(robot);

        // turn towards the center powershot target
        MovementFunctions.turnTo(0,0.5,0, robot);

        // flick the ring into the shooter
        TeleOpFunctions.flickRing(robot);

        // turn towards the right powershot target
        MovementFunctions.turnTo(15,0.5,0, robot);

        // flick the ring into the shooter
        TeleOpFunctions.flickRing(robot);

        // turn back towards center and shoot twice, IN CASE ANY RINGS ARE STUCK
        MovementFunctions.turnTo(0,0.5,0, robot);
        TeleOpFunctions.flickRing(robot);
        TeleOpFunctions.flickRing(robot);

        // turn off the shooter
        TeleOpFunctions.shooterOff(robot);

        // put the transfer system down
        TeleOpFunctions.transferDown(robot);

        // park on line
        odom.goToPosition(60,84,0.7,0,0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);

        robot.robotStatus = "Program Stopping...";
        odom.stopUpdateThread();
        odom.writePositionToFiles();

    }

}

