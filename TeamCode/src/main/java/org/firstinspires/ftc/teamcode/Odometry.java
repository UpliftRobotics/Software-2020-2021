package org.firstinspires.ftc.teamcode;


import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;
import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.Point;

import java.util.ArrayList;

public class Odometry {

    private Robot robot;
    private PositionUpdateThread posRun;
    public boolean updateValid;

    // initialize starting position and orientation
    public double worldXPosition = 0;
    public double worldYPosition = 0;
    public double worldAngle = 0;

    // declare and initialize needed variables to calculate position
    private double initialLeftDistance = 0;
    private double initialRightDistance = 0;
    private double initialCenterDistance = 0;
    private double initialAngle = 0;
    private double finalLeftDistance = 0;
    private double finalRightDistance = 0;
    private double finalCenterDistance = 0;
    private double finalAngle = 0;
    public double deltaLeftDistance;
    public double deltaRightDistance;
    public double deltaCenterDistance;
    private double deltaAngle;
    private double deltaHorizontal;
    private double changeInRobotOrientation;
    private double robotOrientationRadians;


    // class constructor for Odometry
    public Odometry(Robot robot) {
        this.robot = robot;
//        positionUpdate();
        posRun = new PositionUpdateThread();
        updateValid = true;
        posRun.start();
    }

    // getter method for the left encoder ticks
    public int getLeftTicks() {
        return robot.leftFront.getCurrentPosition();
    }

    // getter method for the right encoder ticks
    public int getRightTicks() {
        return robot.rightFront.getCurrentPosition();
    }

    // getter method for the center encoder ticks
    public int getCenterTicks() {
        return robot.leftBack.getCurrentPosition();
    }

    public void setStartPosition(Point pt, double angle) {
        worldXPosition += pt.x;
        worldYPosition += pt.y;
        worldAngle += angle; // in degrees
    }

    // method to update the robot's position
    public void positionUpdate() {

        finalLeftDistance = (getLeftTicks() / Robot.COUNTS_PER_INCH);
        finalRightDistance = (getRightTicks() / Robot.COUNTS_PER_INCH);
        finalCenterDistance = (getCenterTicks() / Robot.COUNTS_PER_INCH);

        deltaLeftDistance = finalLeftDistance - initialLeftDistance;
        deltaRightDistance = finalRightDistance - initialRightDistance;
        deltaCenterDistance = finalCenterDistance - initialCenterDistance;
        changeInRobotOrientation = Math.toDegrees((deltaLeftDistance - deltaRightDistance) / (Robot.robotEncoderWheelDistance));
        deltaHorizontal = deltaCenterDistance - (changeInRobotOrientation * Robot.horizontalEncoderInchesPerDegreeOffset);

        worldAngle = MathFunctions.AngleRestrictions(worldAngle + changeInRobotOrientation);

        worldXPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(Math.toRadians(worldAngle))) + (deltaHorizontal * Math.cos(Math.toRadians(worldAngle)));

        worldYPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(Math.toRadians(worldAngle))) - (deltaHorizontal * Math.sin(Math.toRadians(worldAngle)));

        initialLeftDistance = finalLeftDistance;
        initialRightDistance = finalRightDistance;
        initialCenterDistance = finalCenterDistance;

    }

    // method to go to a given point
    public void goToPosition(double xPosition, double yPosition, double movementSpeed, double preferredAngle, double allowedDistError, double allowedAngleError) {
//        positionUpdate();
        double xDistanceToPoint = xPosition - worldXPosition;
        double yDistanceToPoint = yPosition - worldYPosition;
        double distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
        double relativeAngle = Math.toDegrees(Math.atan2(yDistanceToPoint, xDistanceToPoint));
        double approachZone = allowedDistError * 5;

        while (distanceToPoint > allowedDistError) {
            //if it enters the approach zone
            if (distanceToPoint <= approachZone) {
                robot.drive(MathFunctions.slowApproach(movementSpeed, distanceToPoint, approachZone), relativeAngle, 0);
                //if it is not in the approach zone
            } else {
                robot.drive(movementSpeed, relativeAngle, 0);
            }

//            positionUpdate();
            xDistanceToPoint = xPosition - worldXPosition;
            yDistanceToPoint = yPosition - worldYPosition;
            distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
            relativeAngle = Math.toDegrees(Math.atan2(yDistanceToPoint, xDistanceToPoint));
        }

//        positionUpdate();

        stopMotors();

//        if(worldAngle > MathFunctions.AngleRestrictions(preferredAngle + allowedAngleError)) {
//            while(worldAngle > MathFunctions.AngleRestrictions(preferredAngle + (allowedAngleError / 4))) {
//                robot.spin(-0.2);
//                positionUpdate();
//            }
//            stopMotors();
//        } else if(worldAngle < MathFunctions.AngleRestrictions(preferredAngle - allowedAngleError)) {
//            while(worldAngle < MathFunctions.AngleRestrictions(preferredAngle - (allowedAngleError / 4))) {
//                robot.spin(0.2);
//                positionUpdate();
//            }
//            stopMotors();
//        }

//        positionUpdate();

        return;
    }

    public double getDistanceToPoint(PathPoint pt) {
        double xPosition = pt.x;
        double yPosition = pt.y;
        double xDistanceToPoint = xPosition - worldXPosition;
        double yDistanceToPoint = yPosition - worldYPosition;
        double distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
        return distanceToPoint;
    }

    public void followPath(ArrayList<PathPoint> path) {
        // tell the robot to map out the path and follow it
        for (PathPoint pt : path) {
            goToPosition(pt.x, pt.y, pt.moveSpeed, 0, pt.errorDistance, pt.errorAngle);
        }
    }

    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void stopUpdateThread() {
        updateValid = false;
    }

    private class PositionUpdateThread extends Thread {

        @Override
        public void run() {
            while(updateValid) {
                positionUpdate();
//                Log.i("Thread", "THREAD WORKING");
                try {
                    Thread.sleep(10);
//                    Log.i("Thread", "THREAD SLEEPING... SHHHHHHH");
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            }
        }

    }

}
