package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.File;

public class OdometryGlobalPositionUpdate implements Runnable {
    // create boolean that becomes true the instant the program begins
    private boolean isRunning = true;

    // declare and init Robot and variables/constants
    Robot robot = new Robot();

    static final double oneRotationTicks = 800;
    static final double wheelRadius = 0.025; // in meters (change this later)

    private double deltaLeftDistance = 0;
    private double deltaRightDistance = 0;
    private double deltaCenterDistance = 0;
    private int sleepTime;
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    // access files created and written to in the calibration program
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    // constructor for this class that initializes the encoders, delay, and wheel constants
    public OdometryGlobalPositionUpdate(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, double COUNTS_PER_INCH, int threadSleepDelay){
        this.robot.leftFront = leftFront;
        this.robot.rightFront = rightFront;
        this.robot.leftBack = leftBack;
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    // method to update the robot's position
    private void globalCoordinatePositionUpdate(){
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        robot.worldXPosition += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(robot.worldAngle_rad);
        robot.worldYPosition += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(robot.worldAngle_rad);
        robot.worldAngle_rad += (deltaLeftDistance - deltaRightDistance) / robotEncoderWheelDistance;
        //resetTicks();
    }

    // getter method for the left encoder ticks
    public int getLeftTicks() {
        // left encoder wheel
        return -robot.leftFront.getCurrentPosition();
    }

    // getter method for the right encoder ticks
    public int getRightTicks() {
        // right encoder wheel
        return robot.rightFront.getCurrentPosition();
    }

    // getter method for the center encoder ticks
    public int getCenterTicks() {
        // center encoder wheel
        return robot.leftBack.getCurrentPosition();
    }

    // method to "stop" the program by setting the boolean isRunning to false;
    public void stop(){
        isRunning = false;
    }


    // method that will run when the program is played; stops when the boolean isRunning becomes false
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

} // end of class OdometryGlobalPositionUpdate
