package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    // Declare HardwareMap and hardware devices

    public HardwareMap hardwareMap;

    public DcMotor leftFront; //Declares two left motors
    public DcMotor leftBack;
    public DcMotor rightFront; //Declares two right motors
    public DcMotor rightBack;

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

        leftFront = hardwareMap.get(DcMotor.class,"lf_motor");//Declares two left motors
        leftBack = hardwareMap.get(DcMotor.class,"lb_motor");
        rightFront = hardwareMap.get(DcMotor.class,"rf_motor"); //Declares two right motors
        rightBack = hardwareMap.get(DcMotor.class,"rb_motor");


        leftEncoderMotor = hardwareMap.get(DcMotor.class,"l_encoder");
        rightEncoderMotor = hardwareMap.get(DcMotor.class,"r_encoder") ;
        centerEncoderMotor = hardwareMap.get(DcMotor.class,"c_encoder");

        imu = hardwareMap.get(BNO055IMU.class, "imu");


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Note: may need to change direction of some motors

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