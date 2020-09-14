package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.toolkit.OdometryUpdater;
import org.firstinspires.ftc.teamcode.toolkit.PathPoint;

import java.util.ArrayList;

public class Odometry {

    private Robot robot;

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

    // class constructor for Odometry
    public Odometry(Robot robot) {
        this.robot = robot;
        positionUpdate();
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

    // method to update the robot's position
    public void positionUpdate(){

        finalLeftDistance = (getLeftTicks() / Robot.COUNTS_PER_INCH);
        finalRightDistance = (getRightTicks() / Robot.COUNTS_PER_INCH);
        finalCenterDistance = (getCenterTicks() / Robot.COUNTS_PER_INCH);
        finalAngle = robot.imu.getAngularOrientation().firstAngle;

        deltaLeftDistance = finalLeftDistance - initialLeftDistance;
        deltaRightDistance = finalRightDistance - initialRightDistance;
        deltaCenterDistance = finalCenterDistance - initialCenterDistance;
        deltaAngle = finalAngle - initialAngle;
        deltaHorizontal = deltaCenterDistance + (deltaAngle * Robot.horizontalEncoderInchesPerDegreeOffset);

        worldAngle += deltaAngle;

        worldXPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(Math.toRadians(worldAngle))) + (deltaHorizontal * Math.cos(Math.toRadians(worldAngle)));

        worldYPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(Math.toRadians(worldAngle))) - (deltaHorizontal * Math.sin(Math.toRadians(worldAngle)));

        initialLeftDistance = finalLeftDistance;
        initialRightDistance = finalRightDistance;
        initialCenterDistance = finalCenterDistance;
        initialAngle = finalAngle;

    }

    //method to get the distance away from point (not used in robot, but can be used in another class if printing value)
    public double getDistanceToPoint(double xPosition, double yPosition,double movementSpeed,double preferredAngle,double turnSpeed) {
        return Math.hypot(xPosition - worldXPosition, yPosition - worldYPosition);
    }

    public void goToPosition(double xPosition, double yPosition,double movementSpeed,double preferredAngle, double allowDistanceError){
        positionUpdate();
        double xDistanceToPoint = xPosition - worldXPosition;
        double yDistanceToPoint = yPosition - worldYPosition;
        double distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);

        while(distanceToPoint > allowDistanceError) {
            positionUpdate();
            xDistanceToPoint = xPosition - worldXPosition;
            yDistanceToPoint = yPosition - worldYPosition;
            double relativeAngle = Math.toDegrees(Math.atan2(yDistanceToPoint, xDistanceToPoint));
            robot.drive(movementSpeed, relativeAngle, 0);
            distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
        }

        stopMotors();

        return;
    }

    public void followPath(ArrayList<PathPoint> path) {
        // tell the robot to map out the path and follow it
        for(PathPoint pt : path) {
            goToPosition(pt.x, pt.y, pt.moveSpeed, 0, pt.errorDistance);
        }
    }

    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

}
