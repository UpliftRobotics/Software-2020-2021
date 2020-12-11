package org.firstinspires.ftc.teamcode.toolkit;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;


public class TeleOpFunctions {
    public static void shooterOn(double speed, Robot robot) {
        robot.shooter1.setPower(speed);
        robot.shooter2.setPower(speed);
    }

    public static void shooterOn(double speed, long time, Robot robot) {
        robot.shooter1.setPower(speed);
        robot.shooter2.setPower(speed);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static void shooterOff(Robot robot) {
        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);
    }

    public static void flickRing(Robot robot) {
        robot.flicker.setPosition(0.5);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.flicker.setPosition(0.2);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
 }

    public static void shoot(Robot robot) {
        transferUp(robot);
        shooterOn(1, robot);
        for (int i = 0; i < 3; i++) {
           flickRing(robot);
           try {
               Thread.sleep(100);
           } catch (InterruptedException e) {
               e.printStackTrace();
           }
        }
        shooterOff(robot);
        transferDown(robot);
    }



    public static void dropWobble (Robot robot){
        robot.wobble.setPosition(0); // CHANGE VALUE
    }

    public static void pickupWobble (Robot robot){
        robot.wobble.setPosition(0.5); // CHANGE VALUE
    }

    public static void transferUp(Robot robot) {
        while(robot.transfer.getCurrentPosition() > -730) {
            robot.transfer.setPower(-0.05);
        }
    }

    public static void transferDown(Robot robot) {
        while(robot.transfer.getCurrentPosition() < 0) {
            robot.transfer.setPower(0.05);
        }
    }

}
