package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.toolkit.MathFunctions;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Teleop extends ULLinearOpMode {

    // declare robot and variables.
    Robot robot;
    Odometry odom;


    // declare joystick values
    double rightX;
    double leftY;
    double leftX;
    @Override
    public void runOpMode() {
        robot = new Robot();
        odom = robot.odometry;

        odom.readPositionFiles();

        waitForStart();

        robot.robotStatus = "Program Running...";

        while (opModeIsActive()) {
            // initialize the gamepad stick values to the three needed axes
            leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
            rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
            leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

            // set powers for intake and transfer
            robot.intake.setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
            robot.transfer.setPower(Range.clip(gamepad2.left_stick_y / 4, -0.25, 0.25));


            // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.
            // find the angle of the left joystick
            double joystickAngle = Math.toDegrees(MathFunctions.atan2UL(leftY, leftX));
            // find the magnitude, or hypotenuse of the left joystick and scale it down by dividing by the max
            double magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
            // find the turnValue directly from the rightX input value (scaled for smoothness)
            double turnValue = 0.75 * rightX;
            // set the powers using the 2 specific equations and clip the result
            MovementFunctions.driveTowards(magnitude, joystickAngle, turnValue, robot);

            //BUTTONS
            //GAMEPAD 1(DRIVER)
            if (gamepad1.a){
                // go to shooting position (for high goal)
                odom.goToPosition(26.512,44.841,0.7,0,0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);

            }
            if(gamepad1.b){
                // go to powershot shooting position
                odom.goToPosition(50.5,56,0.7,0,0.5, MovementFunctions.SLIDE_WITHOUT_TURNS);
            }

            if(gamepad1.x) {
                robot.latch.setPosition(0.22);
            }
            if(gamepad1.y) {
                robot.latch.setPosition(0.32);
            }

            // GAMEPAD 2(OPERATOR)
            if (gamepad2.a) {
                TeleOpFunctions.shooterOn(1, robot);
            }

            if (gamepad2.b) {
                TeleOpFunctions.shooterOff(robot);
            }

            if(gamepad2.right_bumper){
                TeleOpFunctions.flickRing(robot);
            }

            if (gamepad2.y) {
                TeleOpFunctions.shoot(robot);
            }

            if(gamepad2.x) {
                robot.transferUpHeight = robot.transfer.getCurrentPosition();
            }

            if(gamepad2.dpad_left) {
                TeleOpFunctions.dropWobble(robot);
            }

            if(gamepad2.dpad_right) {
                TeleOpFunctions.pickupWobble(robot);
            }

            // CANCEL button for goToPosition
            // Click Left Dpad on Gamepad 1 to cancel a goToPosition method

        }
        robot.robotStatus = "Program Stopping...";
        odom.stopUpdateThread();
        odom.clearPositionFiles();

    }

}
