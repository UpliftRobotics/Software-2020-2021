package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import java.util.ArrayList;

@Autonomous(name = "Shooter Velocity Measurer", group = "Hardware Testers")
public class ShooterVelocity extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    private static double ticksPerShooterWheelRotation = 28;

    @Override
    public void runOpMode() {
        robot = new Robot();
        odom = robot.odometry;

        waitForStart();

        ArrayList<Double> times = new ArrayList<>();
        ArrayList<Double> angularVelocities1 = new ArrayList<>();

        double initialTime = System.currentTimeMillis() / 1000.0;
        double currentElapsedSec = 0.0;
        double previousElapsedSec = 0.0;
        int previousTicks = 0;


        TeleOpFunctions.shooterOn(1, robot);

        while (currentElapsedSec < 8) {

            currentElapsedSec = (System.currentTimeMillis() / 1000.0) - initialTime;
            double deltaTime = currentElapsedSec - previousElapsedSec;
            previousElapsedSec = currentElapsedSec;
            int currentTicks = getShooter1Ticks();
            int deltaTicks = currentTicks - previousTicks;
            previousTicks = currentTicks;

            times.add(currentElapsedSec);
            angularVelocities1.add(((deltaTicks / deltaTime) * 60) / (ticksPerShooterWheelRotation));

            if(currentElapsedSec > 5) {
                TeleOpFunctions.shooterOff(robot);
            }

        }

        robot.robotStatus = "LOADING DATA... DO NOT PRESS STOP!";

        String timeStr = "";
        for(int i = 0; i < times.size(); i++) {
            timeStr += (times.get(i) + "\n");
        }

        String angularVelocity1Str = "";
        for(int i = 0; i < angularVelocities1.size(); i++) {
            angularVelocity1Str += (angularVelocities1.get(i) + "\n");
        }

        Log.w("Times", timeStr);
        Log.w("Shooter 1 Ang Velocity", angularVelocity1Str);

        odom.stopUpdateThread();

        stop();

    }

    private int getShooter1Ticks() {
        return robot.shooter1.getCurrentPosition();
    }


}
