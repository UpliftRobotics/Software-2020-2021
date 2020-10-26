package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.toolkit.ULLinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "RingDetector", group = "OpenCv")
public class RingAutonomous extends ULLinearOpMode {
    private OpenCvCamera camera;
    private RingDetector detector;
    private String position;
    Robot robot=new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.webcamName, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        while (!isStarted()){
            position = detector.position;
            telemetry.addData("position",position);
            telemetry.addData("top", detector.TopTotal);
            telemetry.addData("topcenter", detector.centerTopTotal);
            telemetry.addData("bottomcenter", detector.centerBottomTotal);
            telemetry.addData("bottom", detector.bottomTotal);



        }

        
    }
}
