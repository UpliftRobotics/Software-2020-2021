package org.firstinspires.ftc.teamcode.toolkit;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRad;
    public double slowDownTurnAmount;
    public double error;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double pointLength, double slowDownTurnRad, double slowDownTurnAmount, double error) {

        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = pointLength;
        this.slowDownTurnRad = slowDownTurnRad;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.error = error;
    }

    public CurvePoint(CurvePoint thisPoint) {
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        pointLength = thisPoint.pointLength;
        slowDownTurnRad = thisPoint.slowDownTurnRad;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        error = thisPoint.error;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public void setPoint(Point point) {
        x = point.x;
        y = point.y;
    }
}
