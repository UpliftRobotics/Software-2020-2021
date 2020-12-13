package org.firstinspires.ftc.teamcode.toolkit;

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
        shooterOff(robot);
    }

    public static void shooterOff(Robot robot) {
        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);
    }

    public static void flickRing(Robot robot) {
        robot.flicker.setPosition(0.15);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.flicker.setPosition(0.5);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
 }

    public static void shoot(Robot robot) {
        shooterOn(1, robot);
        if(!robot.ULwait((long)2000)) {
            return;
        }
        transferUp(robot);
        for (int i = 0; i < 3; i++) {
           flickRing(robot);
           if(!robot.ULwait((long)650)) {
                return;
           }
        }
        shooterOff(robot);
        transferDown(robot);
    }

    public static void releaseLatch(Robot robot){
        robot.latch.setPosition(0.32);
    }

    public static void dropWobble (Robot robot){
        robot.wobble.setPosition(0.5); // CHANGE VALUE
    }

    public static void pickupWobble (Robot robot){
        robot.wobble.setPosition(0.9); // CHANGE VALUE
    }

    public static void transferUp(Robot robot) {
        if(robot.transfer.getCurrentPosition() > robot.transferUpHeight) {
            while(robot.transfer.getCurrentPosition() > robot.transferUpHeight) {
                robot.transfer.setPower(-0.3);
            }
        }else if(robot.transfer.getCurrentPosition() < robot.transferUpHeight) {
            while(robot.transfer.getCurrentPosition() < robot.transferUpHeight) {
                robot.transfer.setPower(0.3);
            }
        }
        robot.transfer.setPower(0);

    }

    public static void transferDown(Robot robot) {
        while(robot.transfer.getCurrentPosition() < robot.transferUpHeight - robot.offset) {
            robot.transfer.setPower(0.3);
        }
        robot.transfer.setPower(0);
    }

}
