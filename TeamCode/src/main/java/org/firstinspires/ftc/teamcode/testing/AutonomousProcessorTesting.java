package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Disabled
@Config
@TeleOp(name = "Processor test", group = "Test")
public class AutonomousProcessorTesting extends LinearOpMode {
    VisionPortal.Builder portalBuilder;
    //PixelDetectionProcessor odPixelProcessor = new PixelDetectionProcessor();
    PropDetectionProcessor odPropProcessor = new PropDetectionProcessor();

    //RobotHardware robot = new RobotHardware();
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //robot.initialiseHardware(hardwareMap, telemetry);
        //odPixelProcessor.Init(telemetry);
        odPropProcessor.Init(telemetry);

        BuildPortal();

        odPropProcessor.start();




        waitForStart();

        while (opModeIsActive()){

            //telemetry.addData("Drive Direction", odPixelProcessor.GetDriveDirection());
            //odPropProcessor.telemetryTfod();
            telemetry.addData("Prop position", odPropProcessor.propPosition);
            telemetry.update();
        }

    }



    public void BuildPortal(){

        //TfodProcessor pixTFOD = odPixelProcessor.ODproc;
        TfodProcessor propTFOD = odPropProcessor.ODproc;
        portalBuilder = new VisionPortal.Builder();
        portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //portalBuilder.addProcessor(pixTFOD);
        portalBuilder.addProcessor(propTFOD);
        //portalBuilder.setCameraResolution(new Size((int) cameraResolutionVector.getX(), (int) cameraResolutionVector.getY()));
        portalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);  // YUY2 -> MJPEG
        portalBuilder.enableLiveView(true);
        portalBuilder.setAutoStopLiveView(true);

        portal = portalBuilder.build();

        //portal.setProcessorEnabled(pixTFOD, true);
        portal.setProcessorEnabled(propTFOD, true);
    }
}