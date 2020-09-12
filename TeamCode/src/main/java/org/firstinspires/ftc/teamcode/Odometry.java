package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class Odometry {

    private Robot robot;

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

    public Odometry(Robot robot) {
        this.robot = robot;
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
    public void globalCoordinatePositionUpdate(){

        finalLeftDistance = (getLeftTicks() / Robot.COUNTS_PER_INCH);
        finalRightDistance = (getRightTicks() / Robot.COUNTS_PER_INCH);
        finalCenterDistance = (getCenterTicks() / Robot.COUNTS_PER_INCH);
        finalAngle = robot.imu.getAngularOrientation().firstAngle;

        deltaLeftDistance = finalLeftDistance - initialLeftDistance;
        deltaRightDistance = finalRightDistance - initialRightDistance;
        deltaCenterDistance = finalCenterDistance - initialCenterDistance;
        deltaAngle = finalAngle - initialAngle;
        deltaHorizontal = deltaCenterDistance + (deltaAngle * Robot.horizontalEncoderInchesPerDegreeOffset);

        robot.worldAngle += deltaAngle;

        robot.worldXPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(Math.toRadians(robot.worldAngle))) + (deltaHorizontal * Math.cos(Math.toRadians(robot.worldAngle)));

        robot.worldYPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(Math.toRadians(robot.worldAngle))) - (deltaHorizontal * Math.sin(Math.toRadians(robot.worldAngle)));

        initialLeftDistance = finalLeftDistance;
        initialRightDistance = finalRightDistance;
        initialCenterDistance = finalCenterDistance;
        initialAngle = finalAngle;

    }

    //method to get the distance away from point (not used in robot, but can be used in another class if printing value)
    public double getDistanceToPoint(double xPosition, double yPosition,double movementSpeed,double preferredAngle,double turnSpeed) {
        globalCoordinatePositionUpdate();
        double distanceToPoint = Math.hypot(xPosition - robot.worldXPosition, yPosition - robot.worldYPosition);
        return distanceToPoint;
    }

    // method to slide to an area (for TeleOp, turnVal changes and the robot can turn; for Auto, turnVal is always 0 as diagonal sliding is used only)
    public void slideDirection(double speedVal, double angle, double turnVal) {
        // NOTE: Set turnVal to 0 if you are not turning at all. (ie. Auto)
        // set the powers using the 2 specific equations and clip the result
        robot.leftFront.setPower(Range.clip((Math.sin(Math.toRadians(angle) + (0.25 * Math.PI)) * speedVal + turnVal), -1, 1));
        robot.rightFront.setPower(Range.clip((Math.sin(Math.toRadians(angle) - (0.25 * Math.PI)) * speedVal - turnVal), -1, 1));
        robot.leftBack.setPower(Range.clip((Math.sin(Math.toRadians(angle) - (0.25 * Math.PI)) * speedVal + turnVal), -1, 1));
        robot.rightBack.setPower(Range.clip((Math.sin(Math.toRadians(angle) + (0.25 * Math.PI)) * speedVal - turnVal), -1, 1));
    }

    public void goToPosition(double xPosition, double yPosition,double movementSpeed,double preferredAngle, double allowDistanceError){
        globalCoordinatePositionUpdate();
        double xDistanceToPoint = xPosition - robot.worldXPosition;
        double yDistanceToPoint = yPosition - robot.worldYPosition;
        double distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);

        while(distanceToPoint > allowDistanceError) {
            globalCoordinatePositionUpdate();
            xDistanceToPoint = xPosition - robot.worldXPosition;
            yDistanceToPoint = yPosition - robot.worldYPosition;
            double relativeAngle = Math.toDegrees(Math.atan2(yDistanceToPoint, xDistanceToPoint));
            slideDirection(movementSpeed, relativeAngle, 0);
            distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
        }

        stopMotors();

        return;
    }

    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }
}
