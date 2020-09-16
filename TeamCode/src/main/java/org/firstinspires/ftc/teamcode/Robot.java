package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;

public class Robot {

    // Declare HardwareMap and hardware devices
    public HardwareMap hardwareMap;

    //declare all pieces of hardware on the robot
    public DcMotor leftFront; // links to Left Encoder Motor
    public DcMotor leftBack; // links to Center Encoder Motor
    public DcMotor rightFront; // links to Right Encoder Motor
    public DcMotor rightBack;

    public BNO055IMU imu;

    // values specific to the drivetrain
    public static double oneRotationTicks = 720;
    public static double wheelRadius = 19/25.4; // in meters (change this later)
    public static double wheelCircumference = wheelRadius * (2 * Math.PI); // inches
    public static double COUNTS_PER_INCH = (720 * 4) / wheelCircumference;
    public static double robotEncoderWheelDistance = 14;   //unknown
    public static double horizontalEncoderInchesPerDegreeOffset = 0.02386;

//    // access files created and written to in the calibration program
//    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
//    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    public Robot() {
        //create the hardware map
        hardwareMap = ULLinearOpMode.getInstance().hardwareMap;

        //initialize the motors into the hardware map
        leftFront = hardwareMap.get(DcMotor.class,"lf_motor");//Declares two left motors
        leftBack = hardwareMap.get(DcMotor.class,"lb_motor");
        rightFront = hardwareMap.get(DcMotor.class,"rf_motor"); //Declares two right motors
        rightBack = hardwareMap.get(DcMotor.class,"rb_motor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //setup imu (gyro)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        //setup the motors
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // method to move a certain direction at a given speed
    public void drive(double speedVal, double angle, double turnVal) {
        leftFront.setPower(Range.clip((Math.sin(Math.toRadians(angle) + (0.25 * Math.PI)) * speedVal + turnVal), -1, 1));
        rightFront.setPower(Range.clip((Math.sin(Math.toRadians(angle) - (0.25 * Math.PI)) * speedVal - turnVal), -1, 1));
        leftBack.setPower(Range.clip((Math.sin(Math.toRadians(angle) - (0.25 * Math.PI)) * speedVal + turnVal), -1, 1));
        rightBack.setPower(Range.clip((Math.sin(Math.toRadians(angle) + (0.25 * Math.PI)) * speedVal - turnVal), -1, 1));
    }

    // method to spin at a certain speed (clockwise if positive, counter-clockwise if negative)
    public void spin(double turnVal) {
        leftFront.setPower(turnVal);
        rightFront.setPower(-turnVal);
        leftBack.setPower(turnVal);
        rightBack.setPower(-turnVal);
    }

}