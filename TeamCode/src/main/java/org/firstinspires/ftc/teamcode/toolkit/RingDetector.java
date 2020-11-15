package org.firstinspires.ftc.teamcode.toolkit;

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

public class RingDetector extends OpenCvPipeline {

    public int ringCount;

    @Override
    public Mat processFrame(Mat input) {

        //Converting the source image to hsv and then binary
        input = zoomMat(input, 2);
        double totalMatArea = input.width() * input.height();
        Mat zoomedMat = input.clone();

        Imgproc.blur(zoomedMat, zoomedMat, new Size(2.0, 2.0), new Point(-1, -1));
        Imgproc.cvtColor(zoomedMat, zoomedMat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(0, 50, 70);
        Scalar highHSV = new Scalar(30, 255, 255);

        Core.inRange(zoomedMat, lowHSV, highHSV, zoomedMat);

        // convert Mat to binary
        Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
        Imgproc.threshold(zoomedMat, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        //Finding Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        int maxIndex = 0;

        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea && area < (0.5 * totalMatArea)) {
                maxArea = Imgproc.contourArea(contours.get(i));
                maxIndex = i;
            }
        }

        MatOfPoint largestContour = contours.get(maxIndex);

        //check to make sure the rectangle is large enough
        if(Imgproc.contourArea(largestContour) > 200) {

            // Transform the contour to a different format
            Point[] points = largestContour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(largestContour.toArray());

            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRectOnObject(rotatedRectFitToContour, input);

            double rectWidth = rotatedRectFitToContour.size.width;
            double rectHeight = rotatedRectFitToContour.size.height;

            if (rectWidth < rectHeight) {
                double temp = rectHeight;
                rectHeight = rectWidth;
                rectWidth = temp;
            }

            double rectRatio = rectHeight / rectWidth;

            ringCount = -1;

            // Determine the number of rings detected
            // if rectangle height : width ratio is between 2.5/5 and 4/5, then there are 4 rings (5 x 3 inches)
            if (rectRatio > 0.5 && rectRatio < 0.8) {
                ringCount = 4;
            }
            // if height : width ratio is between 0.5/5 and 2/5, then there is 1 ring (5 x 0.75 inches)
            else if (rectRatio > 0.1 && rectRatio < 0.4) {
                ringCount = 1;
            }

        } else {
            ringCount = 0;
        }

        return input;

    }

    static void drawRectOnObject(RotatedRect rect, Mat drawOn)
    {
        Point[] points = new Point[4];
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
