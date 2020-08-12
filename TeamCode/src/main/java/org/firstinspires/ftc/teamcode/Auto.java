package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.toolkit.CurvePoint;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.OdometryMain.followCurve;

//@Autonomous(name = "PathAuto", group = "OpModes")
public class Auto extends OpMode {

    Robot robot;
    @Override
    public void init() {
        robot = new Robot();
    }

    @Override
    public void loop() {
        // declare and initialize an empty list of CurvePoints
        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        // add each CurvePoint to the ArrayList
        allPoints.add(new CurvePoint(16.658823529725343, 129.03529412007597, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(24.000000000451767, 129.03529412007597, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(31.341176471178187, 129.03529412007597, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(33.0352941182689, 121.97647059053132, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(33.0352941182689, 114.63529411980491, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(32.752941177087116, 107.57647059026027, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(32.752941177087116, 100.23529411953385, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(32.752941177087116, 92.61176470762564, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(33.317647059450685, 85.552941178081, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(33.317647059450685, 78.21176470735458, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(33.60000000063247, 71.15294117780994, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(33.60000000063247, 63.81176470708352, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(33.882352941814254, 56.75294117753888, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(40.65882353017711, 54.4941176480846, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(47.71764705972175, 54.21176470690281, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(54.77647058926638, 53.64705882453924, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(62.40000000117459, 53.64705882453924, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(70.0235294130828, 53.64705882453924, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(77.36470588380922, 53.08235294217567, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(84.70588235453565, 52.5176470598121, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(92.61176470762564, 52.5176470598121, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(99.95294117835206, 52.5176470598121, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(107.57647059026027, 52.235294118630314, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(110.9647058844417, 58.44705882462959, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(110.9647058844417, 65.78823529535602, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(110.9647058844417, 73.12941176608244, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(111.52941176680527, 80.18823529562708, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(108.14117647262384, 86.40000000162635, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(100.80000000189742, 86.68235294280814, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(93.74117647235278, 88.09411764871707, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(86.68235294280814, 89.78823529580778, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(79.62352941326351, 91.20000000171672, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(72.28235294253709, 92.04705882526207, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(64.65882353062888, 92.04705882526207, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(57.035294118720664, 91.76470588408029, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(49.97647058917603, 91.4823529428985, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(42.63529411844961, 91.4823529428985, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(41.22352941254068, 98.54117647244313, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(41.22352941254068, 105.88235294316955, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(41.22352941254068, 113.22352941389597, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(40.941176471358894, 120.28235294344061, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(40.09411764781354, 127.34117647298525, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(36.141176471268544, 133.27058823780274, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(29.082352941723904, 134.96470588489345, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));
        allPoints.add(new CurvePoint(21.74117647099748, 135.81176470843883, 0.7, 0.3, 5.0, 5.0, 0.08726646259971647, 5.0));

        // tell the robot to map out the path and follow it
        followCurve(allPoints, Math.toRadians(90));
    }
}
