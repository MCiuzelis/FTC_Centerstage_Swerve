package org.firstinspires.ftc.teamcode.hardware;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PropDetectionPipeline extends OpenCvPipeline {
    public int areaLeft, areaMid, areaRight;

    //DEFAULT VALUES
    private ColorResult colorResult = ColorResult.UNKNOWN;
    private PositionResult positionResult = PositionResult.UNKNOWN;
    Mat region1_submat, region2_submat, region3_submat;
    Mat YCrCbImage = new Mat();
    Mat Cb = new Mat();
    Mat Cr = new Mat();
    private Mat mainMat = new Mat();

    public enum ColorResult {
        RED,
        BLUE,
        UNKNOWN
    }
    public enum PositionResult {
        LEFT,
        RIGHT,
        MIDDLE,
        UNKNOWN
    }

    @Override
    public void init(Mat firstFrame){
        inputToCb(firstFrame);
        processFrame(firstFrame);
    }

    public Mat processFrame(Mat inputFrame) {
        inputToCb(inputFrame);

        mainMat = getColorResult(Cr, Cb);

        int x1 = 20, y1 = 98, width1 = 75, height1 = 75;
        int x2 = 130, y2 = 98, width2 = 75, height2 = 75;
        int x3 = 240, y3 = 98, width3 = 75, height3 = 75;
        Point LEFT1 = new Point(x1, y1);
        Point LEFT2 = new Point(x2, y2);
        Point LEFT3 = new Point(x3, y3);
        Rect roiRect1 = new Rect(x1, y1, width1, height1);
        Rect roiRect2 = new Rect(x2, y2, width2, height2);
        Rect roiRect3 = new Rect(x3, y3, width3, height3);

        region1_submat = mainMat.submat(roiRect1);
        region2_submat = mainMat.submat(roiRect2);
        region3_submat = mainMat.submat(roiRect3);

        Imgproc.rectangle(inputFrame, LEFT1, new Point(LEFT1.x + width1, LEFT1.y + width1), new Scalar(255, 0,0));
        Imgproc.rectangle(inputFrame, LEFT2, new Point(LEFT2.x + width2, LEFT2.y + width2), new Scalar(255, 0,0));
        Imgproc.rectangle(inputFrame, LEFT3, new Point(LEFT3.x + width3, LEFT3.y + width3), new Scalar(255, 0,0));

        areaLeft = (int)Core.mean(region1_submat).val[0];
        areaRight = (int)Core.mean(region2_submat).val[0];
        areaMid = (int)Core.mean(region3_submat).val[0];

        if (areaLeft > areaMid && areaLeft > areaRight){
            positionResult = PositionResult.LEFT;
            Imgproc.rectangle(inputFrame, LEFT1, new Point(LEFT1.x + width1, LEFT1.y + width1), new Scalar(0, 255,0));
        } else if (areaRight > areaLeft && areaRight > areaMid) {
            positionResult = PositionResult.MIDDLE;
            Imgproc.rectangle(inputFrame, LEFT2, new Point(LEFT2.x + width2, LEFT2.y + width2), new Scalar(0, 255,0));
        } else if (areaMid > areaLeft & areaMid > areaRight) {
            positionResult = PositionResult.RIGHT;
            Imgproc.rectangle(inputFrame, LEFT3, new Point(LEFT3.x + width3, LEFT3.y + width3), new Scalar(0, 255,0));
        }
//        areas.add(areaLeft);
//        areas.add(areaRight);
//        areas.add(areaMid);
//
//        int maxIndex = areas.indexOf(Collections.max(areas));
//
//        switch (maxIndex) {
//            case 0:
//                positionResult = PositionResult.LEFT;
//                break;
//            case 1:
//                positionResult = PositionResult.RIGHT;
//                break;
//            case 2:
//                positionResult = PositionResult.MIDDLE;
//                break;
//        }
        return inputFrame;
    }

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCbImage, Imgproc.COLOR_RGB2YCrCb);
        YCrCbImage.copyTo(mainMat);
        Core.extractChannel(YCrCbImage, Cb, 2);
        Core.extractChannel(YCrCbImage, Cr, 1);
    }



    public Mat getColorResult(Mat Cr, Mat Cb){

        int redPixels = (int)Core.mean(Cr).val[0];
        int bluePixels = (int)Core.mean(Cb).val[0];


        // Determine the dominant color
        if (redPixels > bluePixels) {
            colorResult = ColorResult.RED;
            return Cr;
        } else if (bluePixels > redPixels) {
            colorResult = ColorResult.BLUE;
            return Cb;
        } else {
            colorResult = ColorResult.UNKNOWN;
            return YCrCbImage;
        }
    }

    public ColorResult getColorResult() {
        return colorResult;
    }
    public PositionResult getPositionResult(){
        return positionResult;
    }
}