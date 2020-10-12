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
    public void runOpMode() {
        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();

        PathPoint pt = new PathPoint(0, 120, 0.7, 2, 10);

        double yPosition = pt.y;
        double movementSpeed = pt.moveSpeed;
        double allowedDistError = pt.errorDistance;

        double yDistanceToPoint = yPosition - odom.worldYPosition;
        double approachZone = allowedDistError * 5;

        ArrayList<Double> velocities = new ArrayList<>();
        ArrayList<Double> times = new ArrayList<>();
        ArrayList<Double> distances = new ArrayList<>();

        ElapsedTime timer = new ElapsedTime();

        while (yDistanceToPoint > allowedDistError) {
            timer.reset();
            //if it enters the approach zone
            if (yDistanceToPoint <= approachZone) {
                robot.drive(MathFunctions.slowApproach(movementSpeed, yDistanceToPoint, approachZone), 0, 0);
                //if it is not in the approach zone
            } else {
                robot.drive(movementSpeed, 0, 0);
            }
            yDistanceToPoint = yPosition - odom.worldYPosition;
            double timeElapsedSec = timer.milliseconds() / 1000;
            times.add(timeElapsedSec);
            distances.add(yDistanceToPoint);
            velocities.add(yDistanceToPoint / timeElapsedSec);
        }

        odom.stopMotors();

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

        Log.println(1, "Velocity", velocityStr);
        Log.println(1, "Times", timeStr);
        Log.println(1, "Distances", distanceStr);

        odom.updateValid = false;

    }

}
