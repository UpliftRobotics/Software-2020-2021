package org.firstinspires.ftc.teamcode.toolkit;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Disabled
public class RingDetectorTeleop extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public Rect topRect = new Rect(new Point(120, 90), new Point(200, 120));
    public Rect btRect = new Rect(new Point(120, 200), new Point(200, 230));
    public double TopMatRaw;
    public double TopMatValue;
    public double BottomMatRaw;
    public double BottomMatValue;
    public int ringCount = -1;

    public RingDetectorTeleop() {

    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(10, 50, 70);
        Scalar highHSV = new Scalar(50, 255, 255);

        Core.inRange(workingMatrix, lowHSV, highHSV, workingMatrix);

        Mat topMat = workingMatrix.submat(topRect);
        Mat btMat = workingMatrix.submat(btRect);

        Imgproc.rectangle(workingMatrix, topRect, new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, btRect, new Scalar(0, 255, 0));


        TopMatRaw = Core.sumElems(topMat).val[0];

        TopMatValue = ( Core.sumElems(topMat).val[0] / topRect.area() ) / 255;

        BottomMatRaw = Core.sumElems(btMat).val[0];

        BottomMatValue = ( Core.sumElems(btMat).val[0] / btRect.area() ) / 255;

        if(BottomMatValue > 0.5 && TopMatValue > 0.5){
            ringCount =  4;
        } else if(BottomMatValue > 0.5 && TopMatValue < 0.5){
            ringCount = 1;
        } else {
            ringCount = 0;
        }

        return workingMatrix;
    }
}