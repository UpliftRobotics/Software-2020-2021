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

        double initialTime = System.currentTimeMillis();
        double currentElapsedMilli = 0.0;
        double previousElapsedMilli = 0.0;
        int previousTicks = 0;


        TeleOpFunctions.shooterOn(1, robot);

        while (currentElapsedMilli < 8000) {

            currentElapsedMilli = System.currentTimeMillis() - initialTime;
            double deltaTime = currentElapsedMilli - previousElapsedMilli;
            int currentTicks = getShooter1Ticks();
            int deltaTicks = currentTicks - previousTicks;

            if(deltaTime > 10) {
                times.add(currentElapsedMilli);
                angularVelocities1.add(((deltaTicks / deltaTime) * 60000.0) / (ticksPerShooterWheelRotation));
                previousElapsedMilli = currentElapsedMilli;
                previousTicks = currentTicks;
            }

            if(currentElapsedMilli > 5000 && currentElapsedMilli < 5100) {
                TeleOpFunctions.shooterOff(robot);
            }


        }

        robot.robotStatus = "LOADING DATA... DO NOT PRESS STOP!";

        for(int i = 0; i < times.size(); i++) {
            Log.i("Time", times.get(i) + "");
        }

        for(int i = 0; i < angularVelocities1.size(); i++) {
            Log.d("Velocity", angularVelocities1.get(i) + "");
        }

        odom.stopUpdateThread();
    }

    private int getShooter1Ticks() {
        return robot.shooter1.getCurrentPosition();
    }


}
