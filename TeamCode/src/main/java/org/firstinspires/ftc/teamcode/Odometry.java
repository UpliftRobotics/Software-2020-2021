package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;
import org.firstinspires.ftc.teamcode.toolkit.Point;

import java.util.ArrayList;

/**
 * <h1>Odometry</h1>
 * The Odometry class is used to update the position of the robot and follow a specified path.
 * <p>
 *
 * @author  Uplift Robotics FTC Team 18172
 * @version Meet 1
 * @since   2020-09-12
 */
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


    /**
     * This is the constructor for the Odometry class, which takes in the specified Robot instance
     * that created the Odometry object. The PositionUpdateThread is started in this constructor.
     * @param robot This is the Robot instance that created the Odometry object
     */
    public Odometry(Robot robot) {
        this.robot = robot;
        posRun = new PositionUpdateThread();
        updateValid = true;
        posRun.start();
    }

    /**
     * This is a getter method for the left encoder ticks.
     * @return int This returns the value of the left encoder's position (in ticks)
     */
    public int getLeftTicks() {
        return robot.leftFront.getCurrentPosition();
    }

    /**
     * This is a getter method for the right encoder ticks.
     * @return int This returns the value of the right encoder's position (in ticks)
     */
    public int getRightTicks() {
        return robot.rightFront.getCurrentPosition();
    }

    /**
     * This is a getter method for the center encoder ticks.
     * @return int This returns the value of the center encoder's position (in ticks)
     */
    public int getCenterTicks() {
        return robot.rightBack.getCurrentPosition();
    }

    /**
     * This is an overloaded (1) setter method for the robot's initial position and angle
     * @param pt This is a Point object that holds the x and y values that will be added to the
     *           worldXPosition and worldYPosition. (It is usually used for initial position.)
     * @param angle This is a double value of the angle that will be added to the worldAngle of the
     *              robot. (It is usually used for initial angle.)
     */
    public void setStartPosition(Point pt, double angle) {
        worldXPosition += pt.x;
        worldYPosition += pt.y;
        worldAngle += angle; // in degrees
    }

    /**
     * This is an overloaded (2) setter method for the robot's initial position only
     * @param x This is a double value that holds the x value that will be added to the
     *          worldXPosition (It is usually used for initial x position.)
     * @param y This is a double value that holds the y value that will be added to the
     *          worldYPosition (It is usually used for initial y position.)
     */
    public void setStartPosition(double x, double y) {
        worldXPosition += x;
        worldYPosition += y;
    }

    /**
     * This is a method to update the global position and angle of the robot. This method uses the
     * odometry wheel values (encoders) to mathematically calculate the robot's position on the
     * field. This method is primarily used in the PositionUpdateThread, which calls the method
     * after a set amount of time intervals. The method adds degrees to the angle or x/y amounts to
     * the world position variables (worldXPosition, worldYPosition, worldAngle).
     */
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

    /**
     * This is a method to go to a certain point. The robot drives towards the point, routinely
     * adjusting course, and eventually stopping within a certain allowed distance error.
     * @param xPosition This is the double value of the x position of the target point.
     * @param yPosition This is the double value of the y position of the target point.
     * @param movementSpeed This is the movement speed value, from [-1,1], that will be inputted
     *                      into the driveTowards() method, or manipulated for slowing down.
     * @param preferredAngle This is the preferred angle measure, in degrees [-180,180] that the
     *                       robot should stay facing during its approach to the point. (Usually a
     *                       value of 0)
     * @param allowedDistError This is the allowed distance error from the target point that the
     *                         robot should stop within.
     * @param allowedAngleError This is the amount of angle drift that the robot can handle without
     *                          correcting itself to stay at the preferredAngle.
     */
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

    /**
     * This is a method to follow the path specified in the Auto class.
     * @param path This is an ArrayList of type PathPoint which holds the target points of the
     *             robot and does NOT include the starting position.
     */
    public void followPath(ArrayList<PathPoint> path) {
        // tell the robot to map out the path and follow it
        for (PathPoint pt : path) {
            goToPosition(pt.x, pt.y, pt.moveSpeed, 0, pt.errorDistance, pt.errorAngle);
        }
    }

    /**
     * This method calculates the distance of the robot to a certain point and can be very useful
     * in debugging.
     * @param pt This is a PathPoint value that is the reference point (target).
     * @return
     */
    public double getDistanceToPoint(PathPoint pt) {
        double xPosition = pt.x;
        double yPosition = pt.y;
        double xDistanceToPoint = xPosition - worldXPosition;
        double yDistanceToPoint = yPosition - worldYPosition;
        double distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
        return distanceToPoint;
    }

    /**
     * This method is used to stop the PositionUpdateThread safely by setting the updateValid
     * boolean to false, breaking the PositionUpdateThread out of the while loop.
     */
    public void stopUpdateThread() {
        updateValid = false;
    }

    /**
     * This is a nested class that holds the thread for updating the position of the robot by
     * calling the positionUpdate() method repeatedly every __ milliseconds.
     */
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
