package org.firstinspires.ftc.teamcode;

public class MathFunctions {

    public static double AngleRestrictions(double Angle) {
        while (Angle < -Math.PI) {
            Angle += 2 * Math.PI;
        }
        while (Angle > Math.PI) {
            Angle -= 2 * Math.PI;
        }
        return Angle;
    }
}
