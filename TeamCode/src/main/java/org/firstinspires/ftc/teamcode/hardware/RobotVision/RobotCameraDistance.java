package org.firstinspires.ftc.teamcode.hardware.RobotVision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

public class RobotCameraDistance {

    private static final Scalar LOWER_BOUND_RED = new Scalar(0, 100, 100);
    private static final Scalar UPPER_BOUND_RED = new Scalar(10, 255, 255);
    private static final Scalar LOWER_BOUND_YELLOW = new Scalar(20, 100, 100);
    private static final Scalar UPPER_BOUND_YELLOW = new Scalar(30, 255, 255);
    private static final Scalar LOWER_BOUND_BLUE = new Scalar(110, 100, 100);
    private static final Scalar UPPER_BOUND_BLUE = new Scalar(130, 255, 255);

    private OpenCvCamera webcam;
    private String detectedColor;
    private double[] polarCoordinates = new double[2];
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private Scalar lowerBound;
    private Scalar upperBound;
    private RotatedRect currentRect;

    public void initialize(HardwareMap hardwareMap, String color) {
        this.detectedColor = color;
        setColorBounds(color);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new ColorDetectionPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public double[] getPolarCoordinates() {
        return polarCoordinates;
    }

    private void setColorBounds(String color) {
        switch (color.toLowerCase()) {
            case "red":
                lowerBound = LOWER_BOUND_RED;
                upperBound = UPPER_BOUND_RED;
                break;
            case "yellow":
                lowerBound = LOWER_BOUND_YELLOW;
                upperBound = UPPER_BOUND_YELLOW;
                break;
            case "blue":
                lowerBound = LOWER_BOUND_BLUE;
                upperBound = UPPER_BOUND_BLUE;
                break;
            default:
                throw new IllegalArgumentException("Unsupported color: " + color);
        }
    }

    public void updateCurrentRectExcludingCurrent() {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        RotatedRect largestRotatedRect = null;
        double maxArea = 0;
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
            double area = rotatedRect.size.area();
            if (area > maxArea && (currentRect == null || !isSameRect(rotatedRect, currentRect))) {
                maxArea = area;
                largestRotatedRect = rotatedRect;
            }
        }

        if (largestRotatedRect != null) {
            currentRect = largestRotatedRect;
        }
    }

    private class ColorDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, lowerBound, upperBound, mask);

            updateCurrentRectExcludingCurrent();

            if (currentRect != null) {
                Point rectCenter = currentRect.center;
                Point imageCenter = new Point(input.width() / 2.0, input.height() / 2.0);
                double distance = Math.sqrt(Math.pow(rectCenter.x - imageCenter.x, 2) + Math.pow(rectCenter.y - imageCenter.y, 2));
                double angle = Math.toDegrees(Math.atan2(rectCenter.y - imageCenter.y, rectCenter.x - imageCenter.x));
                if (angle < 0) {
                    angle += 360;
                }
                polarCoordinates[0] = distance;
                polarCoordinates[1] = angle;

                Point[] vertices = new Point[4];
                currentRect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                Imgproc.circle(input, rectCenter, 5, new Scalar(0, 255, 0), -1);
                Imgproc.putText(input, "Distance: " + distance, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
                Imgproc.putText(input, "Angle: " + angle, new Point(10, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
            }
            return input;
        }
    }

    private boolean isSameRect(RotatedRect rect1, RotatedRect rect2) {
        double distanceThreshold = 10.0;
        double areaThreshold = 0.1;
        double centerDistance = Math.sqrt(Math.pow(rect1.center.x - rect2.center.x, 2) + Math.pow(rect1.center.y - rect2.center.y, 2));
        double areaDifference = Math.abs(rect1.size.area() - rect2.size.area()) / rect1.size.area();
        return centerDistance < distanceThreshold && areaDifference < areaThreshold;
    }
}
