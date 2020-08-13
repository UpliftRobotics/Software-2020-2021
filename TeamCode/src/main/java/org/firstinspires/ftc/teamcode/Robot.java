package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;
import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.toolkit.MathFunctions.lineCircleIntersect;
import static org.firstinspires.ftc.teamcode.toolkit.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.toolkit.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.toolkit.MovementVars.movement_y;

public class Robot {

    // Declare HardwareMap and hardware devices

    public HardwareMap hardwareMap;

    public DcMotor leftFront; //Left Encoder Motor
    public DcMotor leftBack; // Center Encoder Motor
    public DcMotor rightFront; //Right Encoder Motor
    public DcMotor rightBack;

    public BNO055IMU imu;


    public static double xSpeed = 0;
    public static double ySpeed = 0;
    public static double turnSpeed = 0;

    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;

    public Robot() {
        hardwareMap = ULOpMode.getInstance().hardwareMap;
        worldXPosition = 0;
        worldYPosition = 0;
        worldAngle_rad = Math.toRadians(0);

        leftFront = hardwareMap.get(DcMotor.class,"lf_motor");//Declares two left motors
        leftBack = hardwareMap.get(DcMotor.class,"lb_motor");
        rightFront = hardwareMap.get(DcMotor.class,"rf_motor"); //Declares two right motors
        rightBack = hardwareMap.get(DcMotor.class,"rb_motor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");


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



    public double getXPos() {
        return worldXPosition;
    }

    public double getYPos() {
        return worldYPosition;
    }


    public double getWorldAngle_rad() {
        return worldAngle_rad;
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

        double distanceToPoint = Math.hypot(xPosition - worldXPosition, yPosition - worldYPosition);

        while(distanceToPoint > 5) {
            // hypotenuse of the triangle is the distance
            distanceToPoint = Math.hypot(xPosition - worldXPosition, yPosition - worldYPosition);
            // arctan is the direction
            double absoluteAngle = Math.atan2(yPosition - worldYPosition, xPosition - worldXPosition);
            // if angle is above pi and below negative pi
            double relativeAngle = absoluteAngle - MathFunctions.AngleRestrictions(worldAngle_rad - Math.toRadians(90));
//          // because I subtract the xposition and yposiion inputed by the current position of the robot
//          double relativeXToPoint = Math.cos(relativeAngle)*distanceToPoint;
//          double relativeYToPoint = Math.sin(relativeAngle)*distanceToPoint;
//          // speeds of the x and y power
//          double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint));
//          double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint));
//          // speeds of x and y
//          movement_x = movementXPower * movementSpeed;
//          movement_y = movementYPower * movementSpeed;
            // add or subtract from the current robot position and get the relative angle of it
            double relativeTurnAngle = relativeAngle - Math.toRadians(180) + preferredAngle;
            // adjust turn speed throughout the curve
            movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;

            slideDirection(1, relativeTurnAngle, movement_turn);
        }

    }

    public CurvePoint getFollowPointPath(CurvePoint startLine, CurvePoint endLine, Point robotLocation, double followRadius) {
        CurvePoint followPt = new CurvePoint(startLine);

        ArrayList<Point> intersections = lineCircleIntersect(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

        double closestAngle = Double.MAX_VALUE;

        for(Point thisIntersection : intersections) {
            double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
            double deltaAngle = Math.abs(MathFunctions.AngleRestrictions(angle - worldAngle_rad));

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