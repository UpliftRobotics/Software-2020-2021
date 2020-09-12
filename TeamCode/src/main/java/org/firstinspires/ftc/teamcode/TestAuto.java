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
        robot.goToPosition(0, 24, 0.5, 0, 0,2);
        robot.goToPosition(24, 24, 0.5, 0, 0,2);


        //set up Text files through control hub


//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//
//        // add each CurvePoint to the ArrayList
//
//        allPoints.add(new CurvePoint(0, 24, .7, 0.5, 5, 5.0, 0, 0));
//
////        // tell the robot to map out the path and follow it
//        for (CurvePoint target : allPoints) {
//            robot.goToPosition(target.x, target.y, target.moveSpeed, 0, 0);
//        }
//        robot.stopMotors();
//


    }

}
