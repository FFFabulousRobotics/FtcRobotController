package org.firstinspires.ftc.teamcode.hardware.RobotVision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
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

    // 定义用于颜色检测的HSV颜色范围常量
    private static final Scalar LOWER_BOUND_RED = new Scalar(0, 100, 100);
    private static final Scalar UPPER_BOUND_RED = new Scalar(10, 255, 255);
    private static final Scalar LOWER_BOUND_YELLOW = new Scalar(20, 100, 100);
    private static final Scalar UPPER_BOUND_YELLOW = new Scalar(30, 255, 255);
    private static final Scalar LOWER_BOUND_BLUE = new Scalar(110, 100, 100);
    private static final Scalar UPPER_BOUND_BLUE = new Scalar(130, 255, 255);

    private OpenCvCamera webcam;  // 用于摄像头操作的OpenCvCamera对象
    private String detectedColor;  // 用于存储检测到的颜色
    private double[] polarCoordinates = new double[2];  // 数组形式，0为距离，1为角度
    private Mat hsv = new Mat();  // 用于存储HSV颜色空间图像
    private Mat mask = new Mat();  // 用于存储颜色掩码
    private Mat hierarchy = new Mat();  // 用于存储轮廓层级信息
    private Mat sharpened = new Mat();  // 用于存储锐化后的图像
    private Scalar lowerBound;  // 用于存储颜色下界
    private Scalar upperBound;  // 用于存储颜色上界

    /**
     * 初始化摄像头并设置颜色检测管道。
     *
     * @param hardwareMap 硬件映射对象
     * @param color 要检测的颜色
     */
    public void initialize(HardwareMap hardwareMap, String color) {
        this.detectedColor = color;
        setColorBounds(color);  // 设置颜色范围

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new ColorDetectionPipeline());  // 设置颜色检测管道

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // 开始视频流，设置分辨率和旋转角度
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // 处理摄像头打开错误
            }
        });
    }

    /**
     * 获取极坐标信息，包括距离和角度。
     *
     * @return 极坐标数组，0为距离，1为角度
     */
    public double[] getPolarCoordinates() {
        return polarCoordinates;
    }

    /**
     * 根据颜色名称设置颜色范围。
     *
     * @param color 颜色名称
     */
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

    private class ColorDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // 复制输入图像用于锐化处理
            Mat src = input.clone();
            Mat blurImg = new Mat();

            // 应用高斯模糊减少噪声
            Imgproc.GaussianBlur(src, blurImg, new Size(0, 0), 25);

            // 创建锐化效果
            Core.addWeighted(src, 1.5, blurImg, -0.5, 0, sharpened);

            // 将图像转换为HSV颜色空间
            Imgproc.cvtColor(sharpened, hsv, Imgproc.COLOR_RGB2HSV);

            // 根据颜色范围创建掩码
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // 查找图像中的轮廓
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // 查找最大矩形
            double maxArea = 0;
            RotatedRect largestRotatedRect = null;
            for (MatOfPoint contour : contours) {
                // 将轮廓转换为2f点集
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                // 查找最小面积的包围矩形
                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
                double area = rotatedRect.size.area();
                if (area > maxArea) {
                    maxArea = area;
                    largestRotatedRect = rotatedRect;
                }
            }

            // 计算距离和角度
            if (largestRotatedRect != null) {
                Point rectCenter = largestRotatedRect.center;
                Point imageCenter = new Point(input.width() / 2.0, input.height() / 2.0);
                double distance = Math.sqrt(Math.pow(rectCenter.x - imageCenter.x, 2) + Math.pow(rectCenter.y - imageCenter.y, 2));
                double angle = Math.toDegrees(Math.atan2(rectCenter.y - imageCenter.y, rectCenter.x - imageCenter.x));

                // 使角度范围在0-360度之间
                if (angle < 0) {
                    angle += 360;
                }

                polarCoordinates[0] = distance;
                polarCoordinates[1] = angle;

                // 绘制最大旋转矩形
                Point[] vertices = new Point[4];
                largestRotatedRect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                // 在图像上绘制最大矩形和中心点
                Imgproc.circle(input, rectCenter, 5, new Scalar(0, 255, 0), -1);
                Imgproc.putText(input, "Distance: " + distance, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
                Imgproc.putText(input, "Angle: " + angle, new Point(10, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
            }

            return input;
        }
    }
}
