package org.firstinspires.ftc.teamcode.toolkit;
import org.firstinspires.ftc.teamcode.Robot;


public class TeleOpFunctions{
    public static void shooterOn(double speed, Robot robot) {
        robot.shooter1.setPower(speed);
        robot.shooter2.setPower(speed);
    }

    public static void shooterOn(double speed, long time, Robot robot){
        robot.shooter1.setPower(speed);
        robot.shooter2.setPower(speed);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static void shooterOff(Robot robot){
        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);
    }

    public static void flickRing(Robot robot) {
        robot.flicker.setPosition(0.5);
        robot.flicker.setPosition(0);
    }

    public static void shoot(Robot robot) {
        // put transfer here
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
       // put transfer here
    }

    public static void dropWobble(Robot robot) {
        robot.wobble.setPosition(0); // CHANGE VALUE
    }

    public static void pickupWobble(Robot robot) {
        robot.wobble.setPosition(0.5); // CHANGE VALUE
    }

}
