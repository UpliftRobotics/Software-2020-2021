package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

/*
    Note: this class was inspired by the odometry calibration program made by Wizard.exe, FTC Team 9794
*/

@TeleOp(name = "OdometryCalibration", group = "Odometry")
public class OdometryCalibration extends ULLinearOpMode {
    Robot robot;
    Odometry odom;

    // declare and init class variables/constants
    final double PIVOT_SPEED = 0.5;
    ElapsedTime timer = new ElapsedTime();
    double horizontalTickOffset = 0;

    // Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
//    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
//    File horizontalTickOffsetFile = AppUtil.getInstance().getSetfhtingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() {
        robot = new Robot();
        odom = new Odometry(robot);

        waitForStart();


        // add necessary parameters for the REVhub imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        // Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();


        while (getZAngle() < 90) {
            setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);

            if (getZAngle() < 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            } else {
                setPowerAll(-PIVOT_SPEED / 2, -PIVOT_SPEED / 2, PIVOT_SPEED / 2, PIVOT_SPEED / 2);
            }
            double angle = getZAngle();


            double encoderDifference = Math.abs(odom.getLeftTicks()) + (Math.abs(odom.getRightTicks()));

            double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

            double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * Robot.COUNTS_PER_INCH);

            horizontalTickOffset = odom.getCenterTicks() / Math.toRadians(getZAngle());


            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", odom.getLeftTicks());
            telemetry.addData("Vertical Right Position", odom.getRightTicks());
            telemetry.addData("Horizontal Position", robot.leftBack.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);
            telemetry.addData("Wheel distance", wheelBaseSeparation);
            telemetry.update();
        }


        setPowerAll(0, 0, 0, 0);
        stop();
        timer.reset();
//            while(timer.milliseconds() < 1000){
//                telemetry.addData("IMU Angle", getZAngle());
//                telemetry.update();
//            }
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
