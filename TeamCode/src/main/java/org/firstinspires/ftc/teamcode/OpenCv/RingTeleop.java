package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "RingTeleop", group = "OpenCv")
public class RingTeleop extends LinearOpMode {
    private OpenCvCamera camera;
    private TeleopRingDetector detector = new TeleopRingDetector();
    WebcamName webcamName;


    @Override
    public void runOpMode() throws InterruptedException {
        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (!isStarted()) {
            telemetry.addData("Top Rectangle Raw Value", (int) detector.TopMatRaw);
            telemetry.addData("Top Rectangle Percent", Math.round(detector.TopMatValue * 100) + "%");
            telemetry.addData("Bottom Rectangle Raw Value", (int) detector.BottomMatRaw);
            telemetry.addData("Bottom Rectangle Percent", Math.round(detector.BottomMatValue * 100) + "%");
            telemetry.addData("position", detector.ringCount);
            telemetry.update();

        }


    }
}
