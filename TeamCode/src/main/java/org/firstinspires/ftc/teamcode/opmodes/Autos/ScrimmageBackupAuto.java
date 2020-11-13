package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "Scrimmage Backup Auto", group = "OpModes")
public class ScrimmageBackupAuto extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        // park the robot on the white line
        robot.drive(0.7, 0, 0);
        Thread.sleep(1000);
        robot.stopMotors();

        odom.stopUpdateThread();

    }

}

