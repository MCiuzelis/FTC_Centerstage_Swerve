package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.text.DecimalFormat;

import java.util.ArrayList;

public class AprilTagData{

    private static final DecimalFormat df = new DecimalFormat("0.00");
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private boolean isModeDebug;

    private AprilTagProcessor tagProc = null;
    private VisionPortal visionPortal;
    public void Init(HardwareMap hw, Telemetry telemetry, boolean DEBUG_MODE){
        this.hardwareMap = hw;
        this.telemetry = telemetry;
        this.isModeDebug = DEBUG_MODE;

        tagProc = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProc)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public ArrayList<AprilTagDetection> GetTags(){
        if (tagProc.getDetections() != null) {
            if (tagProc.getDetections().size() > 0) {
                if (isModeDebug) {
                    for (AprilTagDetection tag : tagProc.getDetections()) {
                        telemetry.addData(String.format("Distance to marker with id of %s", tag.id), df.format(tag.ftcPose.range) + " mm");
                    }
                    telemetry.addData("Average solve time being", tagProc.getPerTagAvgPoseSolveTime() + " ms");
                    telemetry.addData("Camera State", visionPortal.getCameraState());
                    telemetry.update();
                    return tagProc.getDetections();
                } else {
                    return tagProc.getDetections();
                }
            }
        }
        else{
            telemetry.addLine(":(");
        }

        //telemetry.update();

        return null;

    }
}

