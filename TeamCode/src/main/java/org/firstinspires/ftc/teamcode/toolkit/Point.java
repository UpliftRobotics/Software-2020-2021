package org.firstinspires.ftc.teamcode.toolkit;

public class Point {
    public double x;
    public double y;

    public Point() {
        x = 0;
        y = 0;
    }

    public Point(double xCoord, double yCoord) {
        x = xCoord;
        y = yCoord;
    }

    public Point(Point p) {
        x = p.x;
        y = p.y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
