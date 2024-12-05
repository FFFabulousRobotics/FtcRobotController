package org.firstinspires.ftc.teamcode.hardware.RobotVision;

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
    private double detectedAngle = 0;

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
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
            Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);
            kernel.put(0, 0, 0, -1, 0);
            kernel.put(1, 0, -1, 5, -1);
            kernel.put(2, 0, 0, -1, 0);
            Imgproc.filter2D(blurred, sharp, gray.depth(), kernel);
            Rect centerRect = new Rect(input.width() / 8, input.height() / 8, input.width() * 3 / 4, input.height() * 3 / 4);
            Mat centerMat = sharp.submat(centerRect);
            Imgproc.Canny(centerMat, edges, 50, 150);
            Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 50, 50, 10);

            double angle = 0;
            if (lines.rows() > 0) {
                for (int i = 0; i < lines.rows(); i++) {
                    double[] line = lines.get(i, 0);
                    double dx = line[2] - line[0];
                    double dy = line[3] - line[1];
                    angle = Math.toDegrees(Math.atan2(dy, dx));
                    break;
                }
            }

            detectedAngle = Math.round(angle / 10.0) * 10;

            Point imageCenter = new Point(input.width() / 2.0, input.height() / 2.0);
            Point rectCenter = new Point(centerRect.x + centerRect.width / 2.0, centerRect.y + centerRect.height / 2.0);

            double distance = Math.sqrt(Math.pow(rectCenter.x - imageCenter.x, 2) + Math.pow(rectCenter.y - imageCenter.y, 2));
            double rectAngle = Math.toDegrees(Math.atan2(rectCenter.y - imageCenter.y, rectCenter.x - imageCenter.x));

            if (rectAngle > 180) {
                rectAngle -= 360;
            }

            double[] polarCoordinates = new double[2];
            polarCoordinates[0] = distance;
            polarCoordinates[1] = rectAngle;

            Imgproc.rectangle(input, centerRect, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, rectCenter, 5, new Scalar(0, 255, 0), -1);
            Imgproc.putText(input, "Distance: " + distance, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
            Imgproc.putText(input, "Angle: " + rectAngle, new Point(10, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);

            return input;
        }
    }
}
