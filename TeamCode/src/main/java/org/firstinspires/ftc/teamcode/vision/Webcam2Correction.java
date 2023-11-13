package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.VisionProcessor;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.Calib3d;
public class Webcam2Correction implements VisionProcessor {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat K = new Mat();
    Mat D = new Mat();


    public Webcam2Correction(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {




    }



    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Core.rotate(frame, frame, Core.ROTATE_90_COUNTERCLOCKWISE);
        Calib3d.fisheye_distortPoints(frame, frame, K, D);
        frame.copyTo(frame);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }


}
