package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.toolkit.ULOpMode;

/*
    Note: this class was inspired by the odometry calibration program made by Wizard.exe, FTC Team 9794
*/

@TeleOp(name = "OdometryCalibration", group = "Odometry")
public class OdometryCalibration extends ULOpMode {
    Robot robot = new Robot();
    OdometryGlobalPositionUpdate od = new OdometryGlobalPositionUpdate(robot, OdometryGlobalPositionUpdate.oneRotationTicks, 10);

    // declare and init class variables/constants
    final double PIVOT_SPEED = 0.5;
    final double COUNTS_PER_INCH = 76.595410103; // CHANGE WHEN ROBOT READY
    ElapsedTime timer = new ElapsedTime();
    double horizontalTickOffset = 0;

    // Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
//    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
//    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void init() {

    }

    // declare and initialize instance of the Robot class (Object-oriented programming)

    @Override
    public void loop() {
        // add necessary parameters for the REVhub imu
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

        // Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();


        while(getZAngle() < 90 ){
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
            while(timer.milliseconds() < 1000){
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            }
        double angle = getZAngle();


        double encoderDifference = Math.abs(od.getLeftTicks()) + (Math.abs(od.getRightTicks()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = od.getCenterTicks()/Math.toRadians(getZAngle());


            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", od.getLeftTicks());
            telemetry.addData("Vertical Right Position", od.getRightTicks());
            telemetry.addData("Horizontal Position", od.getCenterTicks());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }

    private double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }

    private void setPowerAll(double rf, double rb, double lf, double lb){
        robot.rightFront.setPower(rf);
        robot.rightBack.setPower(rb);
        robot.leftFront.setPower(lf);
        robot.rightBack.setPower(lb);
    }

}
