package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;

import static org.firstinspires.ftc.teamcode.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.Robot.worldYPosition;

public class OdometryMain {

        Orientation angles;

        static final double oneRotationTicks = 800;
        static final double wheelRadius = 0.025; // in meters (change this later)
        static final double wheelDistanceApart = 0.09144 + .016 * 2.0; // in meters (change this later)
        private int leftEncoderPos = 0;
        private int centerEncoderPos = 0;
        private int rightEncoderPos = 0;
        private double deltaLeftDistance = 0;
        private double deltaRightDistance = 0;
        private double deltaCenterDistance = 0;
        private double x = 0;
        private double y = 0;
        private double theta = 0;


        private DcMotor leftEncoderMotor = null;
        private DcMotor rightEncoderMotor = null;
        private DcMotor centerEncoderMotor = null;


        // set up the hardware map
        public void init() {
            //initialize the sensors in an if statement
        }

        public int getLeftTicks() {
            return leftEncoderMotor.getCurrentPosition() - leftEncoderPos;
        }

        public int getRightTicks() {
            return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
        }

        public int getCenterTicks() {
            return centerEncoderMotor.getCurrentPosition() - centerEncoderPos;
        }

        public void updatePosition() {
            deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
            deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
            deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
            x += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
            y += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
            theta += (deltaLeftDistance - deltaRightDistance) / wheelDistanceApart;
            // resetTicks();
        }


        public static void goToPosition(double xPosition, double yPosition,double movementSpeed,double preferredAngle,double turnSpeed){
                //hypotenuse of the triangle is the distance
                double distanceToPoint = Math.hypot(xPosition - worldXPosition, yPosition - worldYPosition);
                // arctan is the direction
                double absoluteAngle = Math.atan2(yPosition - worldYPosition, xPosition - worldXPosition);
                //if angle is above pi and below negative pi
                double relativeAngle = absoluteAngle - MathFunctions.AngleRestrictions(worldAngle_rad- Math.toRadians(90));
                // because I subtract the xposition and yposiion inputed by the current position of the robot
                double relativeXToPoint = Math.cos(relativeAngle)*distanceToPoint;
                double relativeYToPoint = Math.sin(relativeAngle)*distanceToPoint;
                //speeds of the x and y power
                double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint));
                double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint));
                //speeds of x and y
                movement_x = movementXPower * movementSpeed;
                movement_y = movementYPower * movementSpeed;
                // add or subtract from the current robot position and get the relative angle of it
                double relativeTurnAngle = relativeAngle - Math.toRadians(180) + preferredAngle;
                //adjust turn speed throughout the curve
                movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;

                if(distanceToPoint < 10){
                    movement_turn = 0;
                    movement_y = 0;
                    movement_x = 0;
                }


        }

}
