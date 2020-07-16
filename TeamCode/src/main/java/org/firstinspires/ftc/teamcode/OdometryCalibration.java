package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.File;

@TeleOp(name = "OdometryCalibration", group = "Odometry")
public class OdometryCalibration extends LinearOpMode {
    Robot robot = new Robot();

    final double PIVOT_SPEED = 0.5;
    //CHANGE WHEN ROBOT READY
    final double COUNTS_PER_INCH = 0.1;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        while(getZAngle() < 90 && opModeIsActive()){
            setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);

            if(getZAngle() < 60) {
                    setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
                }else{
                    setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
                }

                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            }
            setPowerAll(0, 0, 0, 0);
            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive()){
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            }
        double angle = getZAngle();


        double encoderDifference = Math.abs(robot.leftEncoderMotor.getCurrentPosition()) + (Math.abs(robot.rightEncoderMotor.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = robot.centerEncoderMotor.getCurrentPosition()/Math.toRadians(getZAngle());

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -robot.leftEncoderMotor.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.rightEncoderMotor.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.centerEncoderMotor.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }
    private double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }
    private void setPowerAll(double rf, double rb, double lf, double lb){
        robot.rightMotor.setPower(rf);
        robot.rightMotor1.setPower(rb);
        robot.leftMotor.setPower(lf);
        robot.rightMotor1.setPower(lb);
    }

}
