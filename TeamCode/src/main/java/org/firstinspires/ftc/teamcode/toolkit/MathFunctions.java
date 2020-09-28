package org.firstinspires.ftc.teamcode.toolkit;

import java.util.ArrayList;

public class MathFunctions {

    // https://www.desmos.com/calculator/s4qbf5gnoy

    public static double slowApproach(double moveSpeed, double distToTarget, double approachZoneRadius) {
        double val = distToTarget / (approachZoneRadius);
        moveSpeed = (2 * (1 - (1 / (1 + val)))) * moveSpeed;
        return moveSpeed;
    }

    public static double truncate(double val) {
        return (((int)(val * 1000)) / 1000.0);
    }

    public static double AngleRestrictions(double angle) {
        while (angle < -180) {
            angle += 2 * 180;
        }
        while (angle > 180) {
            angle -= 2 * 180;
        }
        return angle;
    }

    // old method for circle-line intersection (UNUSED)
//    public static ArrayList<Point> lineCircleIntersect(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
//        if((Math.abs(linePoint1.y - linePoint2.y)) < 0.003) {
//            linePoint1.y = linePoint2.y + 0.003;
//        }
//
//        if((Math.abs(linePoint1.x - linePoint2.x)) < 0.003) {
//            linePoint1.x = linePoint2.x + 0.003;
//        }
//
//        //create ArrayList of the points
//        ArrayList<Point> allPoints = new ArrayList<>();
//
//        double slope1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
//
//        double x1 = linePoint1.x - circleCenter.x;
//        double y1 = linePoint1.y - circleCenter.y;
//
//        // Use Quadratic formula to solve for x in (mx + b)^2 = r^2 - x^2
//        // A = m^2 + 1
//        double A = Math.pow(slope1, 2) + 1.0;
//
//        // B = 2mb = 2m(y - mx) = 2my - 2m^2x
//        double B = (2.0 * slope1 * y1) - (2.0 * Math.pow(slope1, 2) * x1);
//
//        // C = b^2 - r^2 = (y - mx)^2 - r^2 = y^2 - 2ymx + m^2x^2 - r^2
//        double C = Math.pow(y1, 2) - (2.0 * y1 * slope1 * x1) + (Math.pow(slope1, 2) * Math.pow(x1, 2)) - Math.pow(radius, 2);
//
//        try {
//            // root1 = (-B + sqrt(B^2 - 4AC)) / 2A
//            double xRoot1 = (-B + Math.sqrt(Math.pow(B, 2) - (4 * A * C))) / (2 * A);
//
//            // use pt slope: y - y1 = slope(x - x1)
//            // solve for y
//            double yRoot1 = (slope1 * (xRoot1 - x1)) + y1;
//
//            //add offset that was ignored earlier
//            xRoot1 += circleCenter.x;
//            yRoot1 += circleCenter.y;
//
//            double minX;
//            if(linePoint1.x < linePoint2.x) {
//                minX = linePoint1.x;
//            } else {
//                minX = linePoint2.x;
//            }
//
//            double maxX;
//            if(linePoint1.x > linePoint2.x) {
//                maxX = linePoint1.x;
//            } else {
//                maxX = linePoint2.x;
//            }
//
//
//            if(xRoot1 > minX && xRoot1 < maxX) {
//                allPoints.add(new Point(xRoot1, yRoot1));
//            }
//
//            // root2 = (-B - sqrt(B^2 - 4AC)) / 2A
//            double xRoot2 = (-B - Math.sqrt(Math.pow(B, 2) - (4 * A * C))) / (2 * A);
//
//            // use pt slope: y - y1 = slope(x - x1)
//            // solve for y
//            double yRoot2 = (slope1 * (xRoot2 - x1)) + y1;
//
//            //add offset that was ignored earlier
//            xRoot2 += circleCenter.x;
//            yRoot2 += circleCenter.y;
//
//            if(xRoot2 > minX && xRoot2 < maxX) {
//                allPoints.add(new Point(xRoot2, yRoot2));
//            }
//
//        } catch (Exception ex) {
//
//            ex.printStackTrace();
//
//        }
//
//        return allPoints;
//
//    }

}
