package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "RingDetector", group = "OpenCv")
public class RingAutonomous extends ULLinearOpMode {
    private OpenCvInternalCamera phonecam;
    private RingDetector detector;
    private String position;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        phonecam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,cameraMonitorViewId);
        phonecam.openCameraDevice();
        phonecam.setPipeline(detector);
        phonecam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        while (!isStarted()){
            position = detector.position;
            telemetry.addData("position",position);
        }

        
    }
}
