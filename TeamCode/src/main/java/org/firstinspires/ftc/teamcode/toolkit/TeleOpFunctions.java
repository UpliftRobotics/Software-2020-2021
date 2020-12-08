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
}
