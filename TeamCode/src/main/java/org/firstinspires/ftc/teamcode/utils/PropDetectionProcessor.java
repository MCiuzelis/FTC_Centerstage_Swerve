package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.List;

public class PropDetectionProcessor extends Thread implements VisionProcessor {

    private static final String FILE_NAME = "/sdcard/FIRST/tflitemodels/ssd_prop_meta_step_max.tflite";
    private static final String[] LABELS = {
            "b", "r",
    };


    public TfodProcessor ODproc;
    Telemetry telemetry;
    public POSITION propPosition = POSITION.UNKNOWN;
    public COLOR propColor = COLOR.UNKNOWN;
    boolean stopThread = false;



    public enum POSITION{
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    }

    public enum COLOR{
        RED,
        BLUE,
        UNKNOWN
    }

    public void Init(Telemetry telemetry){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ODproc = new TfodProcessor.Builder()
                .setIsModelQuantized(true)
                .setModelInputSize(320)
                .setIsModelTensorFlow2(true)
                .setMaxNumRecognitions(4)
                .setModelFileName(FILE_NAME)
                .setModelLabels(LABELS)
                .build();
        ODproc.setMinResultConfidence(0.81f);


        //telemetryTfod();
    }

    @Override
    public void run(){
        while (!stopThread) {
            telemetryTfod();
            //telemetry.addLine("waiting");
            telemetry.update();
        }
    }


    public void stopThread(){
        stopThread = true;
    }


    public POSITION getResults(){
        return propPosition;
    }



    public void telemetryTfod() {

        List<Recognition> currentRecognitions = ODproc.getRecognitions();

        if (currentRecognitions.size() != 0){
            int useRecognition = 0;
            for (int i = 0; i < currentRecognitions.size(); i++){
                if (currentRecognitions.get(i).getWidth() > 400 || currentRecognitions.get(i).getHeight() > 200){
                    useRecognition++;
                }
            }

            if (currentRecognitions.size() > useRecognition) {
                telemetry.addData("usig recognition", useRecognition);

                Recognition propDetection = currentRecognitions.get(useRecognition);
                if (propDetection.getLabel().equals("r")) {
                    propColor = COLOR.RED;
                } else if (propDetection.getLabel().equals("b")) {
                    propColor = COLOR.BLUE;
                }
                telemetry.addData("Angle", propDetection.estimateAngleToObject(AngleUnit.DEGREES));
                if (propDetection.estimateAngleToObject(AngleUnit.DEGREES) * -1 > 5) {
                    propPosition = POSITION.LEFT;
                } else if (propDetection.estimateAngleToObject(AngleUnit.DEGREES) * -1 < -5) {
                    propPosition = POSITION.RIGHT;
                } else {
                    propPosition = POSITION.MIDDLE;
                }
            }
        }
        else{
            telemetry.addLine("nothing detected");
        }

        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }
    }



    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        telemetryTfod();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}