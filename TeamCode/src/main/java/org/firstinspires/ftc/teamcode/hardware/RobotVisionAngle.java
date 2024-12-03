package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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
        Mat lines = new Mat();
        Mat sharp = new Mat();
        Mat kernel = new Mat(3, 3, CvType.CV_16SC1);

        @Override
        public Mat processFrame(Mat input) {
            // 转换为灰度图像
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
            // 高斯模糊，减少噪声
            Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);
            // 锐化内核
            kernel.put(0, 0, 0, -1, 0);
            kernel.put(1, 0, -1, 5, -1);
            kernel.put(2, 0, 0, -1, 0);
            // 锐化处理
            Imgproc.filter2D(blurred, sharp, gray.depth(), kernel);
            // 获取图像中心区域 (3/4的画面)
            Rect centerRect = new Rect(input.width() / 8, input.height() / 8, input.width() * 3 / 4, input.height() * 3 / 4);
            Mat centerMat = sharp.submat(centerRect);
            // 边缘检测
            Imgproc.Canny(centerMat, edges, 50, 150);
            // 霍夫直线变换检测线条
            Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 50, 50, 10);

            double angle = 0;
            if (lines.rows() > 0) {
                for (int i = 0; i < lines.rows(); i++) {
                    double[] line = lines.get(i, 0);
                    double dx = line[2] - line[0];
                    double dy = line[3] - line[1];
                    angle = Math.toDegrees(Math.atan2(dy, dx));
                    if (angle < 0) {
                        angle += 180;
                    }
                    angle = Math.abs(angle - 90);
                    break; // 考虑简单起见只取第一条检测到的线条
                }
            }

            // 四舍五入到最接近的10位数
            detectedAngle = Math.round(angle / 10.0) * 10;

            // 获取图像中心点和矩形中心点
            Point imageCenter = new Point(input.width() / 2.0, input.height() / 2.0);
            Point rectCenter = new Point(centerRect.x + centerRect.width / 2.0, centerRect.y + centerRect.height / 2.0);

            // 计算距离和角度
            double distance = Math.sqrt(Math.pow(rectCenter.x - imageCenter.x, 2) + Math.pow(rectCenter.y - imageCenter.y, 2));
            double rectAngle = Math.toDegrees(Math.atan2(rectCenter.y - imageCenter.y, rectCenter.x - imageCenter.x));

            // 将角度存储到polarCoordinates中
            double[] polarCoordinates = new double[2];
            polarCoordinates[0] = distance;
            polarCoordinates[1] = rectAngle;

            // 在图像上绘制最大矩形和中心点
            Imgproc.rectangle(input, centerRect, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, rectCenter, 5, new Scalar(0, 255, 0), -1);
            Imgproc.putText(input, "Distance: " + distance, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
            Imgproc.putText(input, "Angle: " + rectAngle, new Point(10, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);

            return input;
        }
    }
}
