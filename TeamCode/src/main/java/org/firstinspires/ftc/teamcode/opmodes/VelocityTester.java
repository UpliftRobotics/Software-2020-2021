package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;

@Autonomous
public class VelocityTester extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        PathPoint pt = new PathPoint(0, 60, 0.7, 2, 10);

        double yPosition = pt.y;
        double movementSpeed = pt.moveSpeed;
        double allowedDistError = pt.errorDistance;

        double initialYDistanceToPoint = yPosition - odom.worldYPosition;
        double finalYDistanceToPoint = initialYDistanceToPoint;

        double approachZone = allowedDistError * 5;

        ArrayList<Double> velocities = new ArrayList<>();
        ArrayList<Double> times = new ArrayList<>();
        ArrayList<Double> distances = new ArrayList<>();

        ElapsedTime timer = new ElapsedTime();

        double totalTimeSec = 0;

        telemetry.addData("WorldAngle", odom.worldAngle);

        while (finalYDistanceToPoint > allowedDistError) {
            finalYDistanceToPoint = yPosition - odom.worldYPosition;
            //if it enters the approach zone
            if (finalYDistanceToPoint <= approachZone) {
                robot.drive(MathFunctions.slowApproach(movementSpeed, finalYDistanceToPoint, approachZone), 90, 0);
                //if it is not in the approach zone
            } else {
                robot.drive(movementSpeed, 90, 0);
            }
            double timeElapsedSec = timer.milliseconds() / 1000;
            double deltaDistance = initialYDistanceToPoint - finalYDistanceToPoint;

            if(timeElapsedSec > 0.1) {
                totalTimeSec += timeElapsedSec;
                times.add(totalTimeSec);
                distances.add(deltaDistance);
                velocities.add(deltaDistance / timeElapsedSec);
                initialYDistanceToPoint = finalYDistanceToPoint;
                timer.reset();
            }
        }

        odom.stopMotors();

        Log.i("Thread", "STOPPED");

        String velocityStr = "";
        for(int i = 0; i < velocities.size(); i++) {
            velocityStr += (velocities.get(i) + ", ");
        }

        String timeStr = "";
        for(int i = 0; i < velocities.size(); i++) {
            timeStr += (times.get(i) + ", ");
        }

        String distanceStr = "";
        for(int i = 0; i < velocities.size(); i++) {
            distanceStr += (distances.get(i) + ", ");
        }

        Log.w("Velocity", velocityStr);
        Log.w("Times", timeStr);
        Log.w("Distances", distanceStr);

        odom.updateValid = false;

        stop();

        Log.i("STOP PROGRAM", "STOP");

    }

}
