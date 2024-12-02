package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

public class RobotCameraDistance {
    private OpenCvCamera webcam;
    private String detectedColor;
    private double[] polarCoordinates = new double[2];  // 数组形式，0为距离，1为角度
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private Scalar lowerBound;
    private Scalar upperBound;

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
                // 处理摄像头打开错误
            }
        });
    }

    public double[] getPolarCoordinates() {
        return polarCoordinates;
    }

    private void setColorBounds(String color) {
        switch (color.toLowerCase()) {
            case "red":
                lowerBound = new Scalar(0, 100, 100);
                upperBound = new Scalar(10, 255, 255);
                break;
            case "yellow":
                lowerBound = new Scalar(20, 100, 100);
                upperBound = new Scalar(30, 255, 255);
                break;
            case "blue":
                lowerBound = new Scalar(110, 100, 100);
                upperBound = new Scalar(130, 255, 255);
                break;
            default:
                throw new IllegalArgumentException("Unsupported color: " + color);
        }
    }

    private class ColorDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // 转换为HSV颜色空间
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // 创建颜色掩码
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // 查找轮廓
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // 查找最大矩形
            double maxArea = 0;
            Rect largestRect = null;
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double area = rect.area();
                if (area > maxArea) {
                    maxArea = area;
                    largestRect = rect;
                }
            }

            // 计算距离和角度
            if (largestRect != null) {
                Point rectCenter = new Point(largestRect.x + largestRect.width / 2.0, largestRect.y + largestRect.height / 2.0);
                Point imageCenter = new Point(input.width() / 2.0, input.height() / 2.0);
                double distance = Math.sqrt(Math.pow(rectCenter.x - imageCenter.x, 2) + Math.pow(rectCenter.y - imageCenter.y, 2));
                double angle = Math.toDegrees(Math.atan2(rectCenter.y - imageCenter.y, rectCenter.x - imageCenter.x));

                polarCoordinates[0] = distance;
                polarCoordinates[1] = angle;
            }

            return input;
        }
    }
}
