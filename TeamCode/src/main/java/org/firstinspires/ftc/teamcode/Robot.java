package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class Robot {

    public  HardwareMap hardwareMap;

    public DcMotor leftMotor; //Declares two left motors
    public DcMotor leftMotor1;
    public DcMotor rightMotor; //Declares two right motors
    public DcMotor rightMotor1;


    public DcMotor leftEncoderMotor;
    public DcMotor rightEncoderMotor;
    public DcMotor centerEncoderMotor;
    public BNO055IMU imu;


    public static double xSpeed = 0;
    public static double ySpeed = 0;
    public static double turnSpeed = 0;

    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;

    public Robot() {
        worldXPosition = 0;
        worldYPosition = 0;
        worldAngle_rad = Math.toRadians(0);


        leftMotor = hardwareMap.get(DcMotor.class,"lf_motor");//Declares two left motors
        leftMotor1 = hardwareMap.get(DcMotor.class,"lb_motor");
        rightMotor = hardwareMap.get(DcMotor.class,"rf_motor"); //Declares two right motors
        rightMotor1 = hardwareMap.get(DcMotor.class,"rb_motor");


        leftEncoderMotor = hardwareMap.get(DcMotor.class,"l_encoder");
        rightEncoderMotor = hardwareMap.get(DcMotor.class,"r_encoder") ;
        centerEncoderMotor = hardwareMap.get(DcMotor.class,"c_encoder");

        imu = hardwareMap.get(BNO055IMU.class, "imu");


        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //may need to change direction of some motors

    }



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
}