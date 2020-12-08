package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "Shooter Velocity Measurer", group = "Hardware Testers")
public class ShooterVelocity extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    private static double ticksPerShooterWheelRotation = 720; // (CHANGE THIS)
    private static double shooterWheelRadius = 0.05; // in meters (CHANGE THIS)

    @Override
    public void runOpMode() {
        robot = new Robot();
        odom = robot.odometry;

        waitForStart();

        ArrayList<Double> times = new ArrayList<>();
        ArrayList<Integer> ticks1 = new ArrayList<>();
        ArrayList<Integer> ticks2 = new ArrayList<>();
        ArrayList<Double> angularVelocities1 = new ArrayList<>();
        ArrayList<Double> angularVelocities2 = new ArrayList<>();

        ElapsedTime timer = new ElapsedTime();

        double totalTimeSec = 0;
        double timeElapsedSec = timer.milliseconds() / 1000;

        TeleOpFunctions.shooterOn(1, robot);

        while (totalTimeSec < 5) {

            timeElapsedSec = timer.milliseconds() / 1000;

            if(timeElapsedSec > 0.05) {
                totalTimeSec += timeElapsedSec;
                times.add(totalTimeSec);
                int shooter1Ticks = getShooter1Ticks();
                int shooter2Ticks = getShooter2Ticks();
                ticks1.add(shooter1Ticks);
                ticks2.add(shooter2Ticks);
                angularVelocities1.add(shooter1Ticks / timeElapsedSec);
                angularVelocities2.add(shooter2Ticks / timeElapsedSec);
                timer.reset();
            }
        }

        TeleOpFunctions.shooterOff(robot);

        Log.i("Thread", "STOPPED");
        robot.robotStatus = "LOADING DATA... DO NOT PRESS STOP!";

        String timeStr = "";
        for(int i = 0; i < times.size(); i++) {
            timeStr += (times.get(i) + ", ");
        }

        String angularVelocity1Str = "";
        for(int i = 0; i < angularVelocities1.size(); i++) {
            angularVelocity1Str += (angularVelocities1.get(i) + ", ");
        }

        String angularVelocity2Str = "";
        for(int i = 0; i < angularVelocities2.size(); i++) {
            angularVelocity2Str += (angularVelocities2.get(i) + ", ");
        }

        Log.w("Times", timeStr);
        Log.w("Shooter 1 Ang Velocity", angularVelocity1Str);
        Log.w("Shooter 2 Ang Velocity", angularVelocity2Str);

        odom.updateValid = false;

        stop();

        Log.i("STOP PROGRAM", "STOP");

    }

    private int getShooter1Ticks() {
        return robot.shooter1.getCurrentPosition();
    }

    private int getShooter2Ticks() {
        return robot.shooter2.getCurrentPosition();
    }


}
