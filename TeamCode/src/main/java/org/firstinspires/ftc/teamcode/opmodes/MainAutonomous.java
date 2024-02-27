package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.utils.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.TrajectoriesNew;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Photon
@Config
@Autonomous(name = "😈")

public class MainAutonomous extends CommandOpMode {
    public static double translationKv = 0.039;
    public static double translationKa = 0.006;
    public static double translationKp = 0;
    public static double translationKi = 0;
    public static double translationKd = 0;
    public static double rotationKv = 0.325;
    public static double rotationKa = 0.026;
    public static double rotationKp = 1.2;
    public static double rotationKi = 0;
    public static double rotationKd = 0.003;
    public static double lowPassGain = 0.7;

    double odometryTickToInchRatio =  ((((1+(46d/17d))) * (1+(46d/17))) * 28) / ((33d / 62 * 52 / 18) * 2 * Math.PI * 1.5d);


    double prevHeadingVelocity = 0;
    boolean isOpModeStarted = false;
    boolean autoStartingCloseToBackBoard = true;

    ElapsedTime headingTimer = new ElapsedTime();
    double prevTime = 0;
    double prevHeading = 0;


    VisionPortal.Builder portalBuilder;
    PropDetectionProcessor odPropProcessor = new PropDetectionProcessor();
    VisionPortal portal;
    TfodProcessor propTFOD;

    TrajectorySequence trSequence;
    TrajectorySequenceRunner trajectorySequenceRunner;
    TrajectoryFollower trajectoryFollower;
    DrivetrainSubsystem swerve;
    RobotHardware hardware;
    //ArmSubsystem arm;
    CalibrationTransfer file;
    TrajectoriesNew trajectories;



    @Override
    public void initialize() {
        hardware = new RobotHardware(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware.initialiseHardware(telemetry);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        file = new CalibrationTransfer(telemetry);
        swerve = new DrivetrainSubsystem(hardware, telemetry, false, true);
        //arm = new ArmSubsystem(hardware, telemetry, true);
        trajectories = new TrajectoriesNew(telemetry);

        swerve.resetAllEncoders();
        swerve.resetImuOffset();

        trajectoryFollower = new HolonomicPIDVAFollower(new PIDCoefficients(translationKp, translationKi, translationKd), new PIDCoefficients(translationKp, translationKi, translationKd),new PIDCoefficients(rotationKp, rotationKi, rotationKd));
        trajectorySequenceRunner = new TrajectorySequenceRunner(trajectoryFollower, new PIDCoefficients(rotationKp, rotationKi, rotationKd));


        telemetry.addLine("press triangle 🔺 if robot close to backboard");
        telemetry.addLine("press cross ❌ if robot far from backboard");
        telemetry.update();

        while (!this.isStopRequested()) {
            if (gamepad1.triangle) {
                autoStartingCloseToBackBoard = true;
                break;
            }
            else if (gamepad1.cross){
                autoStartingCloseToBackBoard = false;
                break;
            }
        }

        telemetry.addData("auto starting close to backboard: ", autoStartingCloseToBackBoard);
        telemetry.addData("Prop position", odPropProcessor.propPosition);
        telemetry.addLine("ready for start");
        telemetry.update();

//        odPropProcessor.Init(telemetry);
//        BuildPortal();
//        odPropProcessor.start();
    }



    @Override
    public void run() {
        if (!isOpModeStarted){
//            while (odPropProcessor.getResults() == PropDetectionProcessor.POSITION.UNKNOWN && !isStopRequested())idle();
//            StopProcessors();
////            odPropProcessor.stopThread();
////            if (autoStartingCloseToBackBoard) trSequence = trajectories.buildTwoPixelBackboardAuto(odPropProcessor.getResults(), odPropProcessor.propColor);
////            else trSequence = trajectories.generateTrajectoryFarFromBackboard(odPropProcessor.getResults(), odPropProcessor.propColor);
            trSequence = trajectories.buildTwoPixelBackboardAuto(PropDetectionProcessor.POSITION.MIDDLE, PropDetectionProcessor.COLOR.BLUE);
////
//            trSequence = trajectories.buildTwoPixelBackboardAuto(odPropProcessor.getResults(), odPropProcessor.propColor);
            trajectorySequenceRunner.followTrajectorySequenceAsync(trSequence);

            hardware.startIMUThread(this);
            isOpModeStarted = true;
        }

        hardware.clearBulkCache();
        swerve.updateModuleAngles();
        swerve.updateOdometryFromMotorEncoders();

        double imuAngle = hardware.imuAngle.getRadians();
        Pose2d robotPosition = new Pose2d(
                swerve.robotPosition.getX() / odometryTickToInchRatio,
                swerve.robotPosition.getY() / odometryTickToInchRatio,
                imuAngle);

        double robotVel =  getHeadingVelocity(imuAngle);
        ChassisSpeeds robotVelocity = swerve.getChassisSpeedFromEncoders();

        Pose2d currentRobotVelocity = new Pose2d(
                robotVelocity.vyMetersPerSecond / odometryTickToInchRatio,
                -robotVelocity.vxMetersPerSecond / odometryTickToInchRatio,
                robotVel);

        DriveSignal impulse = trajectorySequenceRunner.update(robotPosition, currentRobotVelocity);
        Pose2d correction = getCorrection(impulse);

        if (impulse != null) {
            telemetry.addData("heading angle", Math.toDegrees(imuAngle));
            telemetry.addData("heading vel", robotVel);
            telemetry.addData("robot position X", robotPosition.getX());
            telemetry.addData("robot position X", robotPosition.getY());
            telemetry.addData("robot velocity X", currentRobotVelocity.getX());
            telemetry.addData("robot velocity Y", currentRobotVelocity.getY());
            swerve.setGamepadInput(correction);
        }
        else swerve.setGamepadInput();
//        if (isStopRequested() || trajectories.stopOpMode) {
//            while (!file.hasWrote){
//                file.PushCalibrationData(hardware.imuAngle.getRadians(), swerve.getAllModuleAngleRads());
//            }
//            sleep(100);
//            requestOpModeStop();
//        }
        swerve.drive();
        telemetry.update();
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

    private Pose2d getCorrection (DriveSignal signal) {
        if (signal != null) {
            double velocityX = signal.getVel().getY() * translationKv;
            double velocityY = signal.getVel().getX() * translationKv;
            double accelerationX = signal.getAccel().getY() * translationKa;
            double accelerationY = signal.getAccel().getX() * translationKa;
            double headingCorrection = signal.getVel().getHeading() * rotationKv + signal.getAccel().getHeading() * rotationKa;

            return new Pose2d(velocityX + accelerationX, velocityY + accelerationY, headingCorrection);
        } else return new Pose2d();
    }

    private double getHeadingVelocity(double imuAngle){
        double angleDela = imuAngle - prevHeading;
        double currentTime = headingTimer.seconds();
        double dt = currentTime - prevTime;

        prevHeading = imuAngle;
        prevTime = currentTime;
        double output = (angleDela / dt) * (1 - lowPassGain) + lowPassGain * prevHeadingVelocity;
        prevHeadingVelocity = output;
        return output;
    }
}