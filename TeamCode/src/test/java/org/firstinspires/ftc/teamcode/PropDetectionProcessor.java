package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class PropDetectionProcessor implements VisionProcessor {

    private static final String FILE_NAME = "/sdcard/FIRST/tflitemodels/ssd_pixel_meta_one.tflite";
    private static final String[] LABELS = {
            "p",
    };

    List<Recognition> currentRecognitions;
    List<Double> angles;


    public int thresholdRight = 20;
    public int thresholdLeft = 20;
    public int thresholdMiddle = 20;
    public TfodProcessor ODproc;
    boolean isMODE_DEBUG = false;
    Telemetry telemetry;

    public void Init(Telemetry telemetry, boolean MODE_DEBUG){
        this.telemetry = telemetry;
        isMODE_DEBUG = MODE_DEBUG;
        ODproc = new TfodProcessor.Builder()
                .setIsModelQuantized(true)
                .setModelInputSize(320)
                .setIsModelTensorFlow2(true)
                .setMaxNumRecognitions(4)
                .setModelFileName(FILE_NAME)
                .setModelLabels(LABELS)
                .build();

        ODproc.setMinResultConfidence(0.75f);

    }

    public double GetDriveDirection(){
        if (currentRecognitions != null){
            if (currentRecognitions.size() >= 2){
                angles.set(0,currentRecognitions.get(0).estimateAngleToObject(AngleUnit.DEGREES));
                angles.set(1,currentRecognitions.get(1).estimateAngleToObject(AngleUnit.DEGREES));
            }
            return 0;
        }
        return 0;
    }

    public void telemetryTfod() {

        currentRecognitions = ODproc.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (isMODE_DEBUG){
                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }

        }

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}