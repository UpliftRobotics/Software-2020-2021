package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;
import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.Point;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import org.firstinspires.ftc.teamcode.toolkit.ULOpMode;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.toolkit.MathFunctions.lineCircleIntersect;

public class Robot {

    // Declare HardwareMap and hardware devices
    public HardwareMap hardwareMap;

    public DcMotor leftFront; // links to Left Encoder Motor
    public DcMotor leftBack; // links to Center Encoder Motor
    public DcMotor rightFront; // links to Right Encoder Motor
    public DcMotor rightBack;

    public BNO055IMU imu;

    public double worldXPosition = 0;
    public double worldYPosition = 0;
    public double worldAngle = 0;

    static final double oneRotationTicks = 720;
    static final double wheelRadius = 19/25.4; // in meters (change this later)
    static final double wheelCircumference = wheelRadius*2*Math.PI; // inches
    static final double COUNTS_PER_INCH = 720*4/wheelCircumference;
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
    private double horizontalChange;
    private int sleepTime;
    private double robotEncoderWheelDistance = 14;
    private double horizontalEncoderInchesPerDegreeOffset = 0.02386;

    // access files created and written to in the calibration program
//    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
//    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    public Robot() {
        hardwareMap = ULLinearOpMode.getInstance().hardwareMap;
        worldXPosition = 0;
        worldYPosition = 0;
        worldAngle = 0;

        leftFront = hardwareMap.get(DcMotor.class,"lf_motor");//Declares two left motors
        leftBack = hardwareMap.get(DcMotor.class,"lb_motor");
        rightFront = hardwareMap.get(DcMotor.class,"rf_motor"); //Declares two right motors
        rightBack = hardwareMap.get(DcMotor.class,"rb_motor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Note: may need to change direction of some motors

    }

    // method to update the robot's position
    public void globalCoordinatePositionUpdate(){
        
        finalLeftDistance = (getLeftTicks() / COUNTS_PER_INCH);
        finalRightDistance = (getRightTicks() / COUNTS_PER_INCH);
        finalCenterDistance = (getCenterTicks() / COUNTS_PER_INCH);
        finalAngle = imu.getAngularOrientation().firstAngle;

        deltaLeftDistance = finalLeftDistance - initialLeftDistance;
        deltaRightDistance = finalRightDistance - initialRightDistance;
        deltaCenterDistance = finalCenterDistance - initialCenterDistance;
        deltaAngle = finalAngle - initialAngle;

        worldAngle += deltaAngle;

        horizontalChange = deltaCenterDistance + (deltaAngle * horizontalEncoderInchesPerDegreeOffset);

        worldXPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(Math.toRadians(worldAngle))) + (horizontalChange * Math.cos(Math.toRadians(worldAngle)));

        worldYPosition += ((((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(Math.toRadians(worldAngle))) - (horizontalChange * Math.sin(Math.toRadians(worldAngle)));

        initialLeftDistance = finalLeftDistance;
        initialRightDistance = finalRightDistance;
        initialCenterDistance = finalCenterDistance;
        initialAngle = finalAngle;

    }

    // getter method for the left encoder ticks
    public int getLeftTicks() {
        return leftFront.getCurrentPosition();

    }

    // getter method for the right encoder ticks
    public int getRightTicks() {

        return rightFront.getCurrentPosition();

    }

    // getter method for the center encoder ticks
    public int getCenterTicks() {

        return leftBack.getCurrentPosition();

    }

    public double getXPos() {
        return worldXPosition;
    }

    public double getYPos() {
        return worldYPosition;
    }

    public double getWorldAngle() {
        return worldAngle;
    }

    //last update time
    private long lastUpdateTime = 0;

    public void slideDirection(double speedVal, double angle, double turnVal) {
        // set the powers using the 2 specific equations and clip the result
        leftFront.setPower(Range.clip((Math.sin(angle + (0.25 * Math.PI)) * speedVal) + turnVal, -1, 1));
        rightFront.setPower(Range.clip((Math.sin(angle - (0.25 * Math.PI)) * speedVal) - turnVal, -1, 1));
        leftBack.setPower(Range.clip((Math.sin(angle - (0.25 * Math.PI)) * speedVal) + turnVal, -1, 1));
        rightBack.setPower(Range.clip((Math.sin(angle + (0.25 * Math.PI)) * speedVal) - turnVal, -1, 1));
    }

    public void stopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void goToPosition(double xPosition, double yPosition,double movementSpeed,double preferredAngle,double turnSpeed){
        globalCoordinatePositionUpdate();
        double xDistanceToPoint = xPosition - worldXPosition;
        double yDistanceToPoint = yPosition - worldYPosition;
        double distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);

        while(distanceToPoint > 5) {
            double relativeAngle = Math.toDegrees(Math.atan2(yDistanceToPoint, xDistanceToPoint));
            slideDirection(0.7, relativeAngle, 0);
            xDistanceToPoint = xPosition - worldXPosition;
            yDistanceToPoint = yPosition - worldYPosition;
            distanceToPoint = Math.hypot(xDistanceToPoint, yDistanceToPoint);
        }

        stopMotors();

        return;
    }

    public double getDistanceToPoint(double xPosition, double yPosition,double movementSpeed,double preferredAngle,double turnSpeed) {
        globalCoordinatePositionUpdate();
        double distanceToPoint = Math.hypot(xPosition - worldXPosition, yPosition - worldYPosition);
        return distanceToPoint;
    }

    public CurvePoint getFollowPointPath(CurvePoint startLine, CurvePoint endLine, Point robotLocation, double followRadius) {
        CurvePoint followPt = new CurvePoint(startLine);

        ArrayList<Point> intersections = lineCircleIntersect(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

        double closestAngle = Double.MAX_VALUE;

        for(Point thisIntersection : intersections) {

            double angle = Math.toDegrees(Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition));
            double deltaAngle = Math.abs(MathFunctions.AngleRestrictions(angle - worldAngle));

            if(deltaAngle < closestAngle) {
                closestAngle = deltaAngle;
                followPt.setPoint(thisIntersection);
            }
        }

        return followPt;

    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {

        for(int i = 0; i < allPoints.size() - 1; i++) {

            CurvePoint followPt = getFollowPointPath(allPoints.get(i), allPoints.get(i + 1), new Point(worldXPosition, worldYPosition), allPoints.get(i).followDistance);

            goToPosition(followPt.x, followPt.y, followPt.moveSpeed, followAngle, followPt.turnSpeed);

        }
        stopMotors();
    }
}