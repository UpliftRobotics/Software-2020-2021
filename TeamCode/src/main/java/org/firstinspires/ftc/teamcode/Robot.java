package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Robot {

    public Robot() {
        worldXPosition = 0;
        worldYPosition = 0;
        worldAngle_rad = Math.toRadians(0);


        DcMotor leftMotor; //Declares two left motors
        DcMotor leftMotor1;
        DcMotor rightMotor; //Declares two right motors
        DcMotor rightMotor1;

    }

    //the actual speed the robot is moving
    public static double xSpeed = 0;
    public static double ySpeed = 0;
    public static double turnSpeed = 0;

    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;

    public double getXPos() {
        return worldXPosition;
    }

    public double getYPos() {
        return worldYPosition;
    }


    public double getWorldAngle_rad() {
        return worldAngle_rad;
    }


    //last update time
    private long lastUpdateTime = 0;

    /**
     * Calculates the change in position of the robot
     */
    public void update() {
        //get the current time
        long currentTimeMillis = System.currentTimeMillis();
        //get the elapsed time
        double elapsedTime = (currentTimeMillis - lastUpdateTime) / 1000.0;
        //remember the lastUpdateTime
        lastUpdateTime = currentTimeMillis;
        if (elapsedTime > 1) {
            return;
        }

        //increment the positions
        
    }
}