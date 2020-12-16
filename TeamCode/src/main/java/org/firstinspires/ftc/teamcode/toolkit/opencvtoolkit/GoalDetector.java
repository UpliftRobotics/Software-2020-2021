package org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit;

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

    @Override
    public Mat processFrame(Mat input) {

//        input = zoomMat(input, 2, new Point(80, 60));

        double totalMatArea = input.width() * input.height();

        Mat gray = input.clone();
        Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(0, 50, 70);
        Scalar highHSV = new Scalar(5, 255, 255);

        Core.inRange(gray, lowHSV, highHSV, gray);

        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        //Finding Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0), 2);

        List<MatOfPoint> prunedContours = pruneContours(contours, totalMatArea);

        Imgproc.drawContours(input, prunedContours, -1, new Scalar(0, 255, 0), 2);

        if(!prunedContours.isEmpty()) {
            ArrayList<Double> rectRatioArray = new ArrayList<>(prunedContours.size());
            for(MatOfPoint contour : prunedContours) {
                rectRatioArray.add(findRectangleRatio(contour, input));
            }

            String arrayStr = "";
            for(Double ratio : rectRatioArray) {
                if(ratio < 0.16 && ratio > 0.1) {
                    arrayStr += ratio + " ";
                }
            }


        } else {

        }

        return input;
    }

    static void drawRectOnObject(RotatedRect rect, Mat drawOn)
    {
        Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], new Scalar(0, 255, 255), 2);
        }
    }

    public List<MatOfPoint> pruneContours(List<MatOfPoint> contours, double totalMatArea) {
        List<MatOfPoint> prunedContours = new ArrayList<>();

        for(int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if(area < (0.75 * totalMatArea)) {
                prunedContours.add(contours.get(i));
            }
        }

        return prunedContours;
    }

    public Mat zoomMat(Mat inputMat, double zoomAmount, Point pt) {
        double width = inputMat.width();
        double height = inputMat.height();

        double zoomWidth = width / zoomAmount;
        double zoomHeight = height / zoomAmount;

        Range columnRange = new Range((int)(pt.x - (0.5 * zoomWidth)), (int)(pt.x + (0.5 * zoomWidth)));
        Range rowRange = new Range((int)(pt.y - (0.5 * zoomHeight)), (int)(pt.y + (0.5 * zoomHeight)));

        return new Mat(inputMat, rowRange, columnRange);
    }

    public static int findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        int maxIndex = 0;

        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea) {
                maxArea = Imgproc.contourArea(contours.get(i));
                maxIndex = i;
            }
        }

        return maxIndex;
    }

    public static double findRectangleRatio(MatOfPoint contour, Mat input) {
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        double rectWidth1 = rotatedRectFitToContour.size.width;
        double rectHeight1 = rotatedRectFitToContour.size.height;

        if (rectWidth1 < rectHeight1) {
            double temp = rectHeight1;
            rectHeight1 = rectWidth1;
            rectWidth1 = temp;
        }

        double rectRatio1 = rectHeight1 / rectWidth1;
        if(rectRatio1 < 0.16 && rectRatio1 > 0.1) {
            drawRectOnObject(rotatedRectFitToContour, input);
        }
        return rectRatio1;
    }
}
