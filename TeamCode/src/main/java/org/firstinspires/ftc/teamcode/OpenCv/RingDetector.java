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
    String ringCount = "";
    public RingDetector() {

    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_BGR2BGRA);

        Mat matBottom = workingMatrix.submat(120, 180, 140, 180);
        Mat matCenterBottom = workingMatrix.submat(150, 210, 140,180 );
//        Mat matCenterTop = workingMatrix.submat(180, 240, 140, 180);
//        Mat matTop = workingMatrix.submat(210, 270, 140, 180);

        Imgproc.rectangle(workingMatrix,new Rect(140,120,40,60),new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix,new Rect(140,150,40,60),new Scalar(0,255,0));
//        Imgproc.rectangle(workingMatrix,new Rect(140,180,40,60),new Scalar(0,255,0));
//        Imgproc.rectangle(workingMatrix,new Rect(140,210,40,60),new Scalar(0,255,0));

         bottomTotal = Core.sumElems(matBottom).val[2];
//         centerBottomTotal = Core.sumElems(matCenterBottom).val[2];
//         centerTopTotal = Core.sumElems(matCenterTop).val[2];
//         TopTotal = Core.sumElems(matTop).val[2];
//
//         if(bottomTotal< centerBottomTotal && bottomTotal<centerTopTotal && bottomTotal< TopTotal){
//             ringCount =  "1";
//         }
//         if(bott\){
//             ringCount = "4";
//
//         } else{
//            ringCount = "0";
//        }


        return workingMatrix;
    }




}
