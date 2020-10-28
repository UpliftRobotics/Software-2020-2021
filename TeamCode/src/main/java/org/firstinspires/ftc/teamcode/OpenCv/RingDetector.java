package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public double bottomTotal;
    public double centerBottomTotal;
    public double centerTopTotal;
    public double TopTotal;
    public RingDetector() {

    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matBottom = workingMatrix.submat(120, 150, 80, 120);
        Mat matCenterBottom = workingMatrix.submat(150, 180, 80, 120);
        Mat matCenterTop = workingMatrix.submat(180, 210, 80, 120);
        Mat matTop = workingMatrix.submat(210, 240, 80, 120);

        Imgproc.rectangle(workingMatrix,new Rect(10,120,40,30),new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix,new Rect(10,150,40,30),new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix,new Rect(10,180,40,30),new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix,new Rect(10,210,40,30),new Scalar(0,255,0));

         bottomTotal = Core.sumElems(matBottom).val[2];
         centerBottomTotal = Core.sumElems(matCenterBottom).val[2];
         centerTopTotal = Core.sumElems(matCenterTop).val[2];
         TopTotal = Core.sumElems(matTop).val[2];


        return workingMatrix;
    }




}
