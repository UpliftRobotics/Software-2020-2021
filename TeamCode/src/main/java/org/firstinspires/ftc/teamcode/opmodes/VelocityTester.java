package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous
public class VelocityTester extends ULLinearOpMode {

    Robot robot;
    Odometry odom;

    @Override
    public void runOpMode() {
        robot = new Robot();
        odom = robot.odometry;

        waitForStart();

        PathPoint pt = new PathPoint(0, 60, 1, 4);

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

        while (finalYDistanceToPoint > allowedDistError) {
            finalYDistanceToPoint = yPosition - odom.worldYPosition;
            //if it enters the approach zone
            if (finalYDistanceToPoint <= approachZone) {
                MovementFunctions.driveTowards(MathFunctions.slowApproach(movementSpeed, finalYDistanceToPoint, approachZone), 0, 0, robot);
                //if it is not in the approach zone
            } else {
                MovementFunctions.driveTowards(movementSpeed, 0, 0, robot);
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

        MovementFunctions.stopMotors(robot);

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
