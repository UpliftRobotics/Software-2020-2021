package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "TestAuto", group = "OpModes")
public class TestAuto extends ULLinearOpMode {

    Robot robot;
    OdometryGlobalPositionUpdate od;

    @Override
    public void runOpMode() {
        robot = new Robot();
        od = new OdometryGlobalPositionUpdate(robot, 76.5954101038, 10);



        waitForStart();
        // declare and initialize an empty list of CurvePoints

        while(opModeIsActive()) {
            ArrayList<CurvePoint> allPoints = new ArrayList<>();

            // add each CurvePoint to the ArrayList
            // allPoints.add(new CurvePoint(0, 0, 0.3, 0.5, 50, 5.0, 0.5235987755982988, 0.2));
            allPoints.add(new CurvePoint(10, 0, 0.3, 0.5, 50, 5.0, 0.5235987755982988, 0.2));


            // tell the robot to map out the path and follow it
            for (CurvePoint target : allPoints) {
                robot.goToPosition(target.x, target.y, target.moveSpeed, 0, target.turnSpeed);
            }
            robot.stopMotors();
        }
    }

}
