package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends ULLinearOpMode {

    // Declare robot and variables.
    Robot robot;
    double rightXDirection;
    double leftY;
    double leftX;


    @Override
    public void runOpMode() {

        waitForStart();
        robot = new Robot();



        while(opModeIsActive()) {
            robot.globalCoordinatePositionUpdate();


            // initialize the gamepad stick values to the three needed axes
            leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
            rightXDirection = Range.clip(gamepad1.right_stick_x, -1, 1);
            leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

            // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.

            // find the angle of the left joystick
            double joystickAngle = Math.atan2(leftY, leftX);

            // find the magnitude, or hypotenuse of the left joystick and scale it down by dividing by the max it could be
            double magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

            // find the turnValue directly from the rightXDirection input value (halved for smoothness)
            double turnValue = 0.75 * rightXDirection;

            // set the powers using the 2 specific equations and clip the result
            robot.slideDirection(magnitude, joystickAngle, turnValue);

            // add telemetry data for the encoders
            telemetry.addData("Left Encoder pos:\t", robot.getLeftTicks() / Robot.COUNTS_PER_INCH);
            telemetry.addData("Right Encoder pos:\t", robot.getRightTicks() / Robot.COUNTS_PER_INCH);
            telemetry.addData("Center Encoder pos:\t", robot.getCenterTicks() / Robot.COUNTS_PER_INCH);
            telemetry.addData("LeftDelta:\t", robot.deltaLeftDistance);
            telemetry.addData("RightDelta:\t", robot.deltaRightDistance );
            telemetry.addData("WorldX:\t", MathFunctions.truncate(robot.worldXPosition));
            telemetry.addData("WorldY:\t", MathFunctions.truncate(robot.worldYPosition));
            telemetry.addData("WorldOrientationAngle\t", Math.toDegrees(robot.worldAngle));
            telemetry.update();

        }

    }
}
