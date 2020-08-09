package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;
import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.Point;
import java.util.ArrayList;
import static org.firstinspires.ftc.teamcode.toolkit.MathFunctions.lineCircleIntersect;
import static org.firstinspires.ftc.teamcode.toolkit.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.toolkit.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.toolkit.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.Robot.worldYPosition;

public class OdometryMain {
        // instantiate new Robot object (Object-oriented programming)
        Robot robot = new Robot();

        Orientation angles;

        public void init() {
            //initialize the sensors in an if statement
        }

        public static void goToPosition(double xPosition, double yPosition,double movementSpeed,double preferredAngle,double turnSpeed){
            // hypotenuse of the triangle is the distance
            double distanceToPoint = Math.hypot(xPosition - worldXPosition, yPosition - worldYPosition);
            // arctan is the direction
            double absoluteAngle = Math.atan2(yPosition - worldYPosition, xPosition - worldXPosition);
            // if angle is above pi and below negative pi
            double relativeAngle = absoluteAngle - MathFunctions.AngleRestrictions(worldAngle_rad- Math.toRadians(90));
            // because I subtract the xposition and yposiion inputed by the current position of the robot
            double relativeXToPoint = Math.cos(relativeAngle)*distanceToPoint;
            double relativeYToPoint = Math.sin(relativeAngle)*distanceToPoint;
            // speeds of the x and y power
            double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint));
            // speeds of x and y
            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;
            // add or subtract from the current robot position and get the relative angle of it
            double relativeTurnAngle = relativeAngle - Math.toRadians(180) + preferredAngle;
            // adjust turn speed throughout the curve
            movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;

            // stop the robot when it is nearly there (will coast the rest of the way)
            if(distanceToPoint < 10){
                movement_turn = 0;
                movement_y = 0;
                movement_x = 0;
            }

        }

        public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
            CurvePoint followPt = new CurvePoint(pathPoints.get(0));

            for(int i = 0; i < pathPoints.size() - 1; i++) {
                CurvePoint startLine = pathPoints.get(i);
                CurvePoint endLine = pathPoints.get(i + 1);

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
            }

            return followPt;

        }

        public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {

            CurvePoint followPt = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);

            goToPosition(followPt.x, followPt.y, followPt.moveSpeed, followAngle, followPt.turnSpeed);
        }

}
