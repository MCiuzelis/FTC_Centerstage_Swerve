package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Constants.clawPickupPos;
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
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Config
@Autonomous(name = "😈")

public class MainAutonomous extends CommandOpMode {
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

//    public static double RotationKp = 1.25;
//    public static double RotationKi = 0.0005;
//    public static double RotationKd = 2;

    public static double base = 10;
    public static double width = 10;

    public static double rotationKv = 0.19;
    public static double rotationKa = 0;
    public static double translationKv = 0.013;
    public static double translationKa = 0;

    boolean isOpModeStarted = false;
    boolean autoStartingCloseToBackBoard;

    double odometryTickToInchRatio =  ((((1+(46d/17d))) * (1+(46d/17))) * 28) / ((25d  / 18) * 2 * Math.PI * 1.5f);


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

        swerve = new DrivetrainSubsystem(hardware, telemetry, false);

        trajectoryFollower = new HolonomicPIDVAFollower(new PIDCoefficients(TranslationKp, TranslationKi, TranslationKd), new PIDCoefficients(TranslationKp, TranslationKi, TranslationKd),new PIDCoefficients(RotationKp, RotationKi, RotationKd)) ;
        swerveVelocityConstraint = new SwerveVelocityConstraint(maxWheelVelocity, width, base);
        accelerationConstraint = new ProfileAccelerationConstraint(profileAccelerationConstraint);
        arm = new ArmSubsystem(hardware, telemetry, true);
        swerve.resetAllEncoders();

        telemetry.addLine("press triangle 🔺 if robot close to backboard");
        telemetry.addLine("press cross ❌ if robot far from backboard");
        telemetry.update();

        autoStartingCloseToBackBoard = true;
        while (!this.isStopRequested()) {
            if (gamepad1.triangle) break;
            else if (gamepad1.cross){
                autoStartingCloseToBackBoard = false;
                break;
            }
        }
        telemetry.addData("auto starting close to backboard: ", autoStartingCloseToBackBoard);
        telemetry.update();

        trajectories = new Trajectories(arm, swerveVelocityConstraint, accelerationConstraint);

        telemetry.addData("auto starting close to backboard: ", autoStartingCloseToBackBoard);
        telemetry.addData("Prop position", odPropProcessor.propPosition);
        telemetry.addLine("ready for start");
        telemetry.update();

        odPropProcessor.Init(telemetry);
        BuildPortal();
    }



    @Override
    public void run() {

        if (!isOpModeStarted){
            odPropProcessor.start();
            hardware.clawAngleServo.setPosition(clawPickupPos);

            while (odPropProcessor.getResults() == PropDetectionProcessor.POSITION.UNKNOWN && !isStopRequested())idle();

            odPropProcessor.stopThread();
            StopProcessors();

            if (autoStartingCloseToBackBoard) trSequence = trajectories.generateTrajectoryCloseToBackboard(odPropProcessor.getResults(), odPropProcessor.propColor);
            else trSequence = trajectories.generateTrajectoryFarFromBackboard(odPropProcessor.getResults(), odPropProcessor.propColor);

            trajectorySequenceRunner = new TrajectorySequenceRunner(trajectoryFollower, new PIDCoefficients(RotationKp, RotationKi, RotationKd));
            trajectorySequenceRunner.followTrajectorySequenceAsync(trSequence);

            hardware.startIMUThread(this);
            isOpModeStarted = true;
        }


        super.run();
        swerve.updateOdometryFromMotorEncoders();

        robotPosition = new com.acmerobotics.roadrunner.geometry.Pose2d(
                swerve.robotPosition.getX() / odometryTickToInchRatio,
                swerve.robotPosition.getY() / odometryTickToInchRatio,
                hardware.imuAngle.getRadians());


        ChassisSpeeds robotVelocity = swerve.getChassisSpeedFromEncoders();
        com.acmerobotics.roadrunner.geometry.Pose2d currentRobotVelocity = new com.acmerobotics.roadrunner.geometry.Pose2d(
                robotVelocity.vyMetersPerSecond / odometryTickToInchRatio,
                -robotVelocity.vxMetersPerSecond / odometryTickToInchRatio,
                0);

        impulse = trajectorySequenceRunner.update(robotPosition, currentRobotVelocity);
        Pose2d correction = getCorrection(impulse);


        telemetry.addData("turn Correction", correction.getHeading());
        telemetry.addData("xCorrection", correction.getX());
        telemetry.addData("yCorrection", correction.getY());

        if (impulse != null) {
            telemetry.addData("signal vel", impulse.getVel().getHeading());
            telemetry.addData("signal accel", impulse.getAccel().getHeading());
            swerve.drive(correction);
        } else {
            swerve.drive();
        }


        if (isStopRequested() || trajectories.stopOpMode) {
            while (!file.hasWrote){
                file.PushCalibrationData(hardware.imuAngle.getRadians(), swerve.getAllModuleAngleRads());
            }
            sleep(100);
            requestOpModeStop();
        }
        telemetry.update();
        hardware.clearBulkCache();
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


    public void StopProcessors(){
        portal.setProcessorEnabled(propTFOD, false);
        portal.close();
    }




    private double getHeadingCorrection (DriveSignal signal){
        if (signal != null) {
            return signal.getVel().getHeading() * rotationKv + signal.getAccel().getHeading() * rotationKa;
        }
        else return 0;
    }



    private Pose2d getCorrection (DriveSignal signal){
        if (signal != null) {
            Translation2d driveVelocityCorrection = new Translation2d(-signal.getVel().getY(), signal.getVel().getX());
            Translation2d driveAccelerationCorrection = new Translation2d(-signal.getAccel().getY(), signal.getVel().getX());
            driveVelocityCorrection = driveVelocityCorrection.times(translationKv);
            driveAccelerationCorrection = driveAccelerationCorrection.times(translationKa);

            double headingCorrection = signal.getVel().getHeading() * rotationKv + signal.getAccel().getHeading() * rotationKa;
            return new Pose2d(driveVelocityCorrection.plus(driveAccelerationCorrection), new Rotation2d(headingCorrection));
        }
        else return new Pose2d();
    }
}