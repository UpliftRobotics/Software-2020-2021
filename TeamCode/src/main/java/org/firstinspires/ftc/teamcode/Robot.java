package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.toolkit.MovementFunctions;
import org.firstinspires.ftc.teamcode.toolkit.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit.RingDetector;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

/**
 * <h1>Robot</h1>
 * The Robot class is where all hardware aspects of the robot are declared and initialized. Also,
 * specific constants and settings are added here.
 * <p>
 *
 * @author  Uplift Robotics FTC Team 18172
 * @version Meet 1
 * @since   4/27/2020
 */
public class Robot {

    public ULLinearOpMode opMode;
    // Declare the odometry object
    public Odometry odometry;

    // Declare HardwareMap and hardware devices
    public HardwareMap hardwareMap;

    //declare all pieces of hardware on the robot
    public DcMotor leftFront; // links to Left Encoder Motor
    public DcMotor leftBack; // links to Center Encoder Motor
    public DcMotor rightFront; // links to Right Encoder Motor
    public DcMotor rightBack;

    public DcMotor shooter1;
    public DcMotor shooter2;
    public DcMotor intake;
    public DcMotor transfer;
    public Servo wobble;
    public Servo flicker;
    public Servo latch;

    public DistanceSensor inDistSensor;
    public BNO055IMU imu;

    public OpenCvCamera camera;
    WebcamName webcamName;
    public RingDetector detector = new RingDetector();

    public String robotStatus;
    public int telemetryType;
    public static final int FULL_TELEMETRY = 0;
    public static final int WORLD_TELEMETRY = 1;
    public static final int CLASS_SPECIFIC_TELEMETRY = 2;
    public double constant = 0.5;
    public int transferUpHeight = -735;
    public int offset = -735;



    // values specific to the drivetrain
    public static double oneRotationTicks = 720;
    public static double wheelRadius = 19/25.4; // in meters (change thi later)
    public static double wheelCircumference = wheelRadius * (2 * Math.PI); // inches
    public static double COUNTS_PER_INCH = (720 * 4) / wheelCircumference;
    public static double robotEncoderWheelDistance = 15.5255;
    public static double horizontalEncoderInchesPerDegreeOffset = -2.055 / COUNTS_PER_INCH;


    /**
     * This is the Robot constructor that initializes all hardware and major software aspects of
     * the robot. The most important parts of this constructor include its creation of the hardware map
     * and instantiation of the Odometry object.
     */
    public Robot() {
        robotStatus = "Init Loading...";

        opMode = ULLinearOpMode.getInstance();

        //create the hardware map
        hardwareMap = opMode.hardwareMap;

        //initialize the motors into the hardware map
        leftFront = hardwareMap.get(DcMotor.class,"lf_motor");//Declares two left motors
        leftBack = hardwareMap.get(DcMotor.class,"lb_motor");
        rightFront = hardwareMap.get(DcMotor.class,"rf_motor"); //Declares two right motors
        rightBack = hardwareMap.get(DcMotor.class,"rb_motor");

        wobble = hardwareMap.get(Servo.class,"wobble");
        flicker = hardwareMap.get(Servo.class,"flicker");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        latch = hardwareMap.get(Servo.class, "latch");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter_1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter_2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        inDistSensor = hardwareMap.get(DistanceSensor.class,"d1");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //setup imu (gyro)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

//        webcamName= hardwareMap.get(WebcamName.class,"webcam");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        camera.openCameraDevice();
//        camera.setPipeline(detector);
//        camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        odometry = new Odometry(this);

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

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robotStatus = "Init Complete - Ready to Start!";
        telemetryType = WORLD_TELEMETRY;

    }

    public void safeDisable() {
        MovementFunctions.stopMotors(this);
        intake.setPower(0);
        shooter1.setPower(0);
        shooter2.setPower(0);
        transfer.setPower(0);
    }

    public boolean ULwait(Long milliseconds) {
        long initialTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - initialTime < milliseconds) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(opMode.gamepad1.dpad_left) {
                safeDisable();
                return false;
            }
        }
        return true;
    }

}