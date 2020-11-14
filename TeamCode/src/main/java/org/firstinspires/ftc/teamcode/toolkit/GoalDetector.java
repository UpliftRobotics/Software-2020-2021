package org.firstinspires.ftc.teamcode.toolkit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class GoalDetector extends OpenCvPipeline {
    public Telemetry telemetry;

    @Override
    public Mat processFrame(Mat input) {

        //Converting the source image to hsv and then binary
        input = zoomMat(input, 2);
        double totalMatArea = input.width() * input.height();
        Mat gray = input.clone();

        Imgproc.blur(gray, gray, new Size(2.0, 2.0), new Point(-1, -1));

        Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(0, 50, 70);
        Scalar highHSV = new Scalar(10, 255, 255);

        Core.inRange(gray, lowHSV, highHSV, gray);

        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        //Finding Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        int maxIndex = 0;

        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea && area < (0.25 * totalMatArea)) {
                maxArea = Imgproc.contourArea(contours.get(i));
                maxIndex = i;
            }
        }

        MatOfPoint largestContour = contours.get(maxIndex);

        // Transform the contour to a different format
        org.opencv.core.Point[] points = largestContour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(largestContour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        drawRectOnObject(rotatedRectFitToContour, input);


        //Drawing the Contours
//        Scalar color = new Scalar(0, 255, 0);
//        Imgproc.drawContours(input, contours, -1, color, 2, Imgproc.LINE_8,
//                hierarchy, 2, new Point() ) ;

        return gray;
    }

    static void drawRectOnObject(RotatedRect rect, Mat drawOn)
    {
        org.opencv.core.Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], new Scalar(255, 0, 0), 2);
        }
    }

    public Mat zoomMat(Mat inputMat, double zoomAmount) {
        double width = inputMat.width();
        double height = inputMat.height();

        double zoomWidth = width / zoomAmount;
        double zoomHeight = height / zoomAmount;

        Range columnRange = new Range((int)(0.5 * (width - zoomWidth)), (int)(width - (0.5 * (width - zoomWidth))));
        Range rowRange = new Range((int)(0.5 * (height - zoomHeight)), (int)(height - (0.5 * (height - zoomHeight))));

        return new Mat(inputMat, rowRange, columnRange);
    }
}
