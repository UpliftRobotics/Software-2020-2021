package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import org.firstinspires.ftc.teamcode.toolkit.ULOpMode;

import java.util.ArrayList;

@Autonomous(name = "TestAuto", group = "OpModes")
public class TestAuto extends ULOpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot();
        robot.globalCoordinatePositionUpdate();
    }

    @Override
    public void loop() {
        //set up Text files through control hub
        telemetry.addData("worldx", robot.getXPos());
        telemetry.addData("worldy", robot.getYPos());
        telemetry.update();

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        // add each CurvePoint to the ArrayList
        allPoints.add(new CurvePoint(5, 0, .7, 0.5, 50, 5.0, 0, 0));
        allPoints.add(new CurvePoint(5, 5, .7, 0.5, 50, 5.0, 0, 0));
        allPoints.add(new CurvePoint(0, 5, .7, 0.5, 50, 5.0, 0, 0));
        allPoints.add(new CurvePoint(0, 0, .7, 0.5, 50, 5.0, 0, 0));

        // tell the robot to map out the path and follow it
        for (CurvePoint target : allPoints) {
            robot.goToPosition(target.x, target.y, target.moveSpeed, 0, 0);
        }

        robot.stopMotors();
        requestOpModeStop();

    }

}
