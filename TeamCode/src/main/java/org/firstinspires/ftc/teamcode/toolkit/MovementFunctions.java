package org.firstinspires.ftc.teamcode.toolkit;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

public class MovementFunctions {

    // method to move a certain direction at a given speed
    public static void driveTowards(double speedVal, double angle, double turnVal, Robot robot) {
        robot.leftFront.setPower(Range.clip((Math.sin(Math.toRadians(angle) + (0.25 * Math.PI)) * speedVal + turnVal), -1, 1));
        robot.rightFront.setPower(Range.clip((Math.sin(Math.toRadians(angle) - (0.25 * Math.PI)) * speedVal - turnVal), -1, 1));
        robot.leftBack.setPower(Range.clip((Math.sin(Math.toRadians(angle) - (0.25 * Math.PI)) * speedVal + turnVal), -1, 1));
        robot.rightBack.setPower(Range.clip((Math.sin(Math.toRadians(angle) + (0.25 * Math.PI)) * speedVal - turnVal), -1, 1));
    }

    // method to constantly spin ( [+] for clockwise and [-] for counter-clockwise )
    public static void spin(double speed, Robot robot) {
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(-speed);
    }

    // method to turn a certain number of degrees ( [+] for clockwise and [-] for counter-clockwise )
    public static void turn(double degrees, double speed, Robot robot) {
        double initialAngle = robot.odometry.rawAngle;
        // if turning counter-clockwise
        if(degrees < 0) {
            while(robot.odometry.worldAngle > (initialAngle + degrees)) {
                if(Math.abs(robot.odometry.worldAngle - initialAngle) < Math.abs(0.2 * degrees)) {
                    MovementFunctions.spin((-1 * Math.abs(speed / 2)), robot);
                } else {
                    MovementFunctions.spin((-1 * Math.abs(speed)), robot);
                }
            }
        // if turning clockwise
        } else if(degrees > 0) {
            while(robot.odometry.worldAngle < (initialAngle + degrees)) {
                if(Math.abs(robot.odometry.worldAngle - initialAngle) < Math.abs(0.2 * degrees)) {
                    MovementFunctions.spin(Math.abs(speed / 2), robot);
                } else {
                    MovementFunctions.spin(Math.abs(speed), robot);
                }
            }
        } else {
            // either a value of 0 was passed into the method, or some null/NA value [do nothing]
        }
    }

    // method to turn TO a certain angle (within the angle restrictions), with either the shortest path (technique 0) or through a specified direction in the direction indicator (clockwise for 1, counter-clockwise for 2)
    public static void turnTo(double targetAngle, double speed, int directionIndex, Robot robot) {
        double initialAngle = robot.odometry.worldAngle;
        // clockwise turn
        if(directionIndex == 1) {
            MovementFunctions.turn(MathFunctions.angleRestrictions(targetAngle - initialAngle + 360), speed, robot);
        // counter-clockwise turn
        } else if(directionIndex == 2) {
            MovementFunctions.turn(MathFunctions.angleRestrictions(targetAngle - initialAngle - 360), speed, robot);
        // Shortest technique
        } else {
            MovementFunctions.turn(MathFunctions.angleRestrictions(targetAngle - initialAngle), speed, robot);
        }
    }


    // method to stop all motors
    public static void stopMotors(Robot robot) {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

}
