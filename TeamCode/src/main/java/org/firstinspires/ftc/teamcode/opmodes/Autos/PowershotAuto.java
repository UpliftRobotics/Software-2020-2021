package org.firstinspires.ftc.teamcode.opmodes.Autos;

import android.content.res.Resources;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.AutoFunctions;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "Meet1 Powershot Auto", group = "OpModes")
public class PowershotAuto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {

        robot = new Robot();
        odom = robot.odometry;


        waitForStart();

        robot.robotStatus = "Program Running...";

        odom.setStartPosition(29, 0.5, 0);

        // shift to the right/forward
        odom.goToPosition(43, 24.5, 0.7, 0, 0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);

        // go to the left powershot shooting position
        odom.goToPosition(43, 56, 0.7, 0, 0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);
        MovementFunctions.turnTo(0, 0.7, 0, robot);

        // turn on shooter
        TeleOpFunctions.shooterOn(1, robot);
        sleep(2000);

        // put the transfer system up
        TeleOpFunctions.transferUp(robot);

        // flick the 1st ring into the shooter
        TeleOpFunctions.flickRing(robot);

        // go to center powershot shooting position
        odom.goToPosition(50.5, 56, 0.7, 0, 0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);
        MovementFunctions.turnTo(0, 0, 0, robot);

        // flick the 2nd ring into the shooter
        TeleOpFunctions.flickRing(robot);

        // go to the right powershot shooting position
        odom.goToPosition(58, 58, 0.7, 0, 0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);
        MovementFunctions.turnTo(0, 0, 0, robot);

        // flick the ring into the shooter
        TeleOpFunctions.flickRing(robot);

        // turn back towards center and shoot twice, IN CASE ANY RINGS ARE STUCK
        MovementFunctions.turnTo(0,0.5,0, robot);
        TeleOpFunctions.flickRing(robot);
        TeleOpFunctions.flickRing(robot);

        // turn off the shooter
        TeleOpFunctions.shooterOff(robot);

        // put the transfer system down
//        TeleOpFunctions.transferDown(robot);

        // park on line
        odom.goToPosition(53,72,0.7,0,0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);

        robot.robotStatus = "Program Stopping...";
        odom.stopUpdateThread();
        odom.writePositionToFiles();

    }

}

