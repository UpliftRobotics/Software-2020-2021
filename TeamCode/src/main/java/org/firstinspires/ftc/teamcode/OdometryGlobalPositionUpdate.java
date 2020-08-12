package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.File;

public class OdometryGlobalPositionUpdate implements Runnable {
    private boolean isRunning = true;

    Robot robot = new Robot();
    static final double oneRotationTicks = 800;
    static final double wheelRadius = 0.025; // in meters (change this later)

    private int leftEncoderPos = 0;
    private int centerEncoderPos = 0;
    private int rightEncoderPos = 0;
    private double deltaLeftDistance = 0;
    private double deltaRightDistance = 0;
    private double deltaCenterDistance = 0;
    private double x = 0;
    private double y = 0;
    private double theta = 0;
    private int sleepTime;
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");


    public OdometryGlobalPositionUpdate(DcMotor leftEncoderMotor, DcMotor rightEncoderMotor, DcMotor centerEncoderMotor, double COUNTS_PER_INCH, int threadSleepDelay){
        this.robot.leftEncoderMotor = leftEncoderMotor;
        this.robot.rightEncoderMotor = rightEncoderMotor;
        this.robot.centerEncoderMotor = centerEncoderMotor;
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }
    private void globalCoordinatePositionUpdate(){
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        Robot.worldXPosition += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        Robot.worldYPosition += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        Robot.worldAngle_rad += (deltaLeftDistance - deltaRightDistance) / robotEncoderWheelDistance;
        // resetTicks();


    }

    public int getLeftTicks() {
        return robot.leftEncoderMotor.getCurrentPosition() - leftEncoderPos;
    }

    public int getRightTicks() {
        return robot.rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
    }

    public int getCenterTicks() {
        return robot.centerEncoderMotor.getCurrentPosition() - centerEncoderPos;
    }

    public void stop(){ isRunning = false; }


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






}
