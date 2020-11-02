package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RingDetector", group = "OpenCv")
public class RingAutonomous extends LinearOpMode {
    private OpenCvCamera camera;
    private RingDetector detector = new RingDetector();
    WebcamName webcamName;


    @Override
    public void runOpMode() throws InterruptedException {
        webcamName= hardwareMap.get(WebcamName .class,"webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        while (!isStarted()){
            telemetry.addData("Main Rectangle Raw Value", (int) detector.mainMatRaw);
            telemetry.addData("Main Rectangle Percent", Math.round(detector.mainMatValue * 100) + "%");
            telemetry.update();
//            telemetry.addData("top", detector.TopTotal);
//            telemetry.addData("topcenter", detector.centerTopTotal);
//            telemetry.addData("bottomcenter", detector.centerBottomTotal);
//            telemetry.addData("bottom", detector.bottomTotal);
//            telemetry.addData("postiton", detector.ringCount);
//            telemetry.update();

        }

        
    }
}
