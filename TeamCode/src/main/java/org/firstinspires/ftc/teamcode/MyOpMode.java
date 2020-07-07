package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.OdometryMain.followCurve;


public class MyOpMode extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 50, 20, Math.toRadians(30), 1.0));
        allPoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 50, 20, Math.toRadians(30), 1.0));
        allPoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 50, 20, Math.toRadians(30), 1.0));
        allPoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 50, 20, Math.toRadians(30), 1.0));
        allPoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 50, 20, Math.toRadians(30), 1.0));
        allPoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 50, 20, Math.toRadians(30), 1.0));

        followCurve(allPoints, Math.toRadians(90));
    }
}
