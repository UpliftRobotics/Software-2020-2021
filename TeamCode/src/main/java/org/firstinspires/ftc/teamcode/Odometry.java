package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.Point;

import java.util.ArrayList;

public class Odometry {

    private Robot robot;
    private MovementFunctions moveFun;
    private PositionUpdateThread posRun;
    public boolean updateValid;

    // initialize starting position and orientation
    public double worldXPosition = 0;
    public double worldYPosition = 0;
    public double rawAngle = 0;
    public double worldAngle = 0;

    // declare and initialize needed variables to calculate position
    private double initialLeftDistance = 0;
    private double initialRightDistance = 0;
    private double initialCenterDistance = 0;
    private double finalLeftDistance = 0;
    private double finalRightDistance = 0;
    private double finalCenterDistance = 0;
    public double deltaLeftDistance;
    public double deltaRightDistance;
    public double deltaCenterDistance;
    private double deltaHorizontal;
    private double changeInRobotOrientation;


    // class constructor for Odometry
    public Odometry(Robot robot) {
        this.robot = robot;
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
        return robot.rightBack.getCurrentPosition();
    }

    // overloaded method to set initial position and angle
    public void setStartPosition(Point pt, double angle) {
        worldXPosition += pt.x;
        worldYPosition += pt.y;
        worldAngle += angle; // in degrees
    }

    // overloaded method to set the initial position (not angle)
    public void setStartPosition(double x, double y) {
        worldXPosition += x;
        worldYPosition += y;
    }

    // method to update the robot's position
    public void positionUpdate() {

        finalLeftDistance = (getLeftTicks() / Robot.COUNTS_PER_INCH);
        finalRightDistance = (getRightTicks() / Robot.COUNTS_PER_INCH);
        finalCenterDistance = -getCenterTicks() / Robot.COUNTS_PER_INCH;

        deltaLeftDistance = finalLeftDistance - initialLeftDistance;
        deltaRightDistance = finalRightDistance - initialRightDistance;
        deltaCenterDistance = finalCenterDistance - initialCenterDistance;
        changeInRobotOrientation = Math.toDegrees((deltaLeftDistance - deltaRightDistance) / (Robot.robotEncoderWheelDistance));
        deltaHorizontal = deltaCenterDistance - (changeInRobotOrientation * Robot.horizontalEncoderInchesPerDegreeOffset);

        rawAngle = worldAngle + changeInRobotOrientation;
        worldAngle = MathFunctions.angleRestrictions(rawAngle);

        worldXPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(Math.toRadians(worldAngle))) + (deltaHorizontal * Math.cos(Math.toRadians(worldAngle)));

        worldYPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(Math.toRadians(worldAngle))) - (deltaHorizontal * Math.sin(Math.toRadians(worldAngle)));

        initialLeftDistance = finalLeftDistance;
        initialRightDistance = finalRightDistance;
        initialCenterDistance = finalCenterDistance;

    }

    // method to go to a given point
    public void goToPosition(double xPosition, double yPosition, double movementSpeed, double preferredAngle, double allowedDistError, double allowedAngleError) {
        double xDistanceToPoint = xPosition - worldXPosition;
        double yDistanceToPoint = yPosition - worldYPosition;
        double distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
        double relativeAngle = Math.toDegrees(Math.atan2(yDistanceToPoint, xDistanceToPoint));
//        double approachZone = allowedDistError * 5;

        while (distanceToPoint > allowedDistError) {
            //if it enters the approach zone
//            if (distanceToPoint <= approachZone) {
//                robot.drive(movementSpeed, relativeAngle, 0);
//                //if it is not in the approach zone
//            } else {
            MovementFunctions.driveTowards(movementSpeed, relativeAngle, 0, robot);


            xDistanceToPoint = xPosition - worldXPosition;
            yDistanceToPoint = yPosition - worldYPosition;
            distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
            relativeAngle = Math.toDegrees(Math.atan2(yDistanceToPoint, xDistanceToPoint));
        }

        MovementFunctions.stopMotors(robot);

        return;
    }

    public void followPath(ArrayList<PathPoint> path) {
        // tell the robot to map out the path and follow it
        for (PathPoint pt : path) {
            goToPosition(pt.x, pt.y, pt.moveSpeed, 0, pt.errorDistance, pt.errorAngle);
        }
    }

    public double getDistanceToPoint(PathPoint pt) {
        double xPosition = pt.x;
        double yPosition = pt.y;
        double xDistanceToPoint = xPosition - worldXPosition;
        double yDistanceToPoint = yPosition - worldYPosition;
        double distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
        return distanceToPoint;
    }

    public void stopUpdateThread() {
        updateValid = false;
    }

    private class PositionUpdateThread extends Thread {

        @Override
        public void run() {
            while(updateValid) {
                positionUpdate();
                try {
                    Thread.sleep(10);
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            }
        }

    }

}
