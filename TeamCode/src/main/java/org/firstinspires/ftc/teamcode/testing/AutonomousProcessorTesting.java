package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.utils.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.Trajectories;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Disabled
@Photon
@Config
@TeleOp(name = "camera test")

public class AutonomousProcessorTesting extends CommandOpMode {
    public static double profileAccelerationConstraint = 19;
    public static double maxWheelVelocity = 20;

    public static double maxAngularVelocity = 6;
    public static double maxAngularAcceleration = 9;

    public static double TranslationKp = 0.001;
    public static double TranslationKi = 0;
    public static double TranslationKd = 0.0001;

    public static double RotationKp = 0.5;
    public static double RotationKi = 0.0005;
    public static double RotationKd = 0.1;


    public static double base = 10;
    public static double width = 10;

    public static double rotationKv = 0.19;
    public static double rotationKa = 0;
    public static double translationKv = 0.013;
    public static double translationKa = 0;

    boolean isOpModeStarted = false;
    boolean autoStartingCloseToBackBoard;

    double odometryTickToInchRatio =  ((((1+(46d/17d))) * (1+(46d/17))) * 28) / ((25d  / 18) * 2 * Math.PI * 1.5f);

//    ElapsedTime headingTimer = new ElapsedTime();
//    LowPassFilter headingFilter = new LowPassFilter(0.9);
//    double prevTime = 0;
//    double prevHeading = 0;


    VisionPortal.Builder portalBuilder;
    PropDetectionProcessor odPropProcessor = new PropDetectionProcessor();
    VisionPortal portal;

    TfodProcessor propTFOD;
    TrajectorySequence trSequence;
    com.acmerobotics.roadrunner.geometry.Pose2d robotPosition = new com.acmerobotics.roadrunner.geometry.Pose2d(0, 0, 0);
    TrajectoryAccelerationConstraint accelerationConstraint;
    SwerveVelocityConstraint swerveVelocityConstraint;
    TrajectorySequenceRunner trajectorySequenceRunner;
    TrajectoryFollower trajectoryFollower;
    DrivetrainSubsystem swerve;
    RobotHardware hardware;
    DriveSignal impulse;
    ArmSubsystem arm;
    CalibrationTransfer file;
    Trajectories trajectories;





    @Override
    public void initialize() {

        hardware = new RobotHardware(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware.initialiseHardware(telemetry);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        file = new CalibrationTransfer(telemetry);
        impulse = new DriveSignal();

        swerve = new DrivetrainSubsystem(hardware, telemetry, false, false);

        trajectoryFollower = new HolonomicPIDVAFollower(new PIDCoefficients(TranslationKp, TranslationKi, TranslationKd), new PIDCoefficients(TranslationKp, TranslationKi, TranslationKd),new PIDCoefficients(RotationKp, RotationKi, RotationKd)) ;
        swerveVelocityConstraint = new SwerveVelocityConstraint(maxWheelVelocity, width, base);
        accelerationConstraint = new ProfileAccelerationConstraint(profileAccelerationConstraint);
        arm = new ArmSubsystem(hardware, telemetry, true);
        swerve.resetAllEncoders();


        odPropProcessor.Init(telemetry);
        BuildPortal();
        odPropProcessor.start();

    }

    @Override
    public void run(){
        telemetry.addData("prop", odPropProcessor.propPosition);
    }


    public void BuildPortal(){
        propTFOD = odPropProcessor.ODproc;
        portalBuilder = new VisionPortal.Builder();
        portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        portalBuilder.addProcessor(propTFOD);
        portalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);  // YUY2 -> MJPEG
        portalBuilder.enableLiveView(true);
        portalBuilder.setAutoStopLiveView(true);

        portal = portalBuilder.build();
        portal.setProcessorEnabled(propTFOD, true);
    }
}