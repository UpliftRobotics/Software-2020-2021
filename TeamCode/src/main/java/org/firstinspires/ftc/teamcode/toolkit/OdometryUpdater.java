package org.firstinspires.ftc.teamcode.toolkit;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;

public class OdometryUpdater implements Runnable{

    Robot robot;
    Odometry odom;

    public OdometryUpdater(Robot robot, Odometry odom){
        this.robot = robot;
        this.odom = odom;
    }

    @Override
    public void run() {
        while(true) {
            odom.positionUpdate();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                System.out.println("Couldn't sleep...");
                e.printStackTrace();
            }
        }
    }
}
