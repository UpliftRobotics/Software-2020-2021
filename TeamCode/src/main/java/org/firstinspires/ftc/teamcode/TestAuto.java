package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import org.firstinspires.ftc.teamcode.toolkit.ULOpMode;

import java.util.ArrayList;

@Autonomous(name = "TestAuto", group = "OpModes")
public class TestAuto extends ULLinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.globalCoordinatePositionUpdate();

        waitForStart();

        //set up Text files through control hub


        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        // add each CurvePoint to the ArrayList

        allPoints.add(new CurvePoint(-111, 0, .7, 0.5, 5, 5.0, 0, 0));

//        // tell the robot to map out the path and follow it
//        for (CurvePoint target : allPoints) {
//            //robot.goToPosition(target.x, target.y, target.moveSpeed, 0, 0);
//        }

        telemetry.addData("distanceToTargetPoint", robot.getDistanceToPoint(-111, 0, .7, 0, 0));
        telemetry.update();

        robot.stopMotors();

    }

}
