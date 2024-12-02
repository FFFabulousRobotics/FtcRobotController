package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

public class RobotVisionAngle {
    private OpenCvCamera webcam;
    private double detectedAngle = 0;  // 角度变量

    public void initialize(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new CenterAnglePipeline());

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

    public double getDetectedAngle() {
        return detectedAngle;
    }

    private class CenterAnglePipeline extends OpenCvPipeline {
        Mat gray = new Mat();
        Mat blurred = new Mat();
        Mat edges = new Mat();
        Mat hierarchy = new Mat();
        MatOfPoint2f approxCurve = new MatOfPoint2f();

        @Override
        public Mat processFrame(Mat input) {
            // 转换为灰度图像
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

            // 高斯模糊，减少噪声
            Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);

            // 边缘检测
            Imgproc.Canny(blurred, edges, 50, 150);

            // 轮廓检测
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // 查找最大矩形
            double maxArea = 0;
            Rect largestRect = null;
            for (MatOfPoint contour : contours) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double peri = Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, 0.02 * peri, true);

                if (approxCurve.total() == 4) {
                    Rect rect = Imgproc.boundingRect(contour);
                    double area = rect.area();
                    if (area > maxArea) {
                        maxArea = area;
                        largestRect = rect;
                    }
                }
            }

            // 计算偏角
            if (largestRect != null) {
                Point rectCenter = new Point(largestRect.x + largestRect.width / 2.0, largestRect.y + largestRect.height / 2.0);
                Point imageCenter = new Point(input.width() / 2.0, input.height() / 2.0);
                double angle = Math.toDegrees(Math.atan2(rectCenter.y - imageCenter.y, rectCenter.x - imageCenter.x));

                // 调整角度方向：竖直为0度，向左为正，向右为负
                if (angle < -90) {
                    angle = 90;
                } else if (angle > 90) {
                    angle = -90;
                }

                // 四舍五入到最近的15度区间
                int roundedAngle = (int) Math.round(angle / 15.0) * 15;
                if (roundedAngle == -90) {
                    roundedAngle = 90;
                }
                detectedAngle = roundedAngle;
            }

            return input;
        }
    }
}
