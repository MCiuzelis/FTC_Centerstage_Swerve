package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Config
@Autonomous(name = "😈")

public class MainAutonomous extends CommandOpMode {
    VisionPortal.Builder portalBuilder;
    PropDetectionProcessor odPropProcessor = new PropDetectionProcessor();

    private VisionPortal portal;
    public static double profileAccelerationConstraint = 19;
    public static double maxWheelVelocity = 20;

    public static double maxAngularVelocity = 6;
    public static double maxAngularAcceleration = 9;

    public static double TranslationKp = 0.008;
    public static double TranslationKi = 0;
    public static double TranslationKd = 0.0005;

    public static double RotationKp = 1.25;
    public static double RotationKi = 0.0005;
    public static double RotationKd = 2;

    public static double base = 10;
    public static double width = 10;

    public static double rotationKv = 0.167;
    public static double rotationKa = 0;
    public static double translationKv = 0.013;
    public static double translationKa = 0;

    public boolean isOpModeStarted = false;

    Thread armThread;
    double odometryTickToInchRatio =  ((((1+(46d/17d))) * (1+(46d/17))) * 28) / ((25d  / 18) * 2 * Math.PI * 1.5f);


    TfodProcessor propTFOD;
    TrajectorySequence trSequence;
    Pose2d robotPosition = new Pose2d(0,0,0);
    TrajectoryAccelerationConstraint accelerationConstraint;
    SwerveVelocityConstraint swerveVelocityConstraint;
    TrajectorySequenceRunner trajectorySequenceRunner;
    TrajectoryFollower trajectoryFollower;
    DrivetrainSubsystem swerve;
    RobotHardware hardware;
    DriveSignal impulse;
    ArmSubsystem arm;
    CalibrationTransfer file;





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

        odPropProcessor.Init(telemetry);
        BuildPortal();
        odPropProcessor.start();
        while (odPropProcessor.getResults() == PropDetectionProcessor.POSITION.UNKNOWN && !isStopRequested())idle();

        odPropProcessor.stopThread();
        odPropProcessor.interrupt();
        sleep(1000);
        StopProcessors();


        telemetry.clearAll();
        telemetry.addData("Prop position", odPropProcessor.propPosition);
        telemetry.addLine("ready for start");
        telemetry.update();






        if (odPropProcessor.getResults() == PropDetectionProcessor.POSITION.RIGHT){
            trSequence = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, accelerationConstraint,maxAngularVelocity,maxAngularAcceleration)
                    .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                    .forward(28)
                    .turn(Math.toRadians(-90))
                    .strafeRight(3)
                    .addDisplacementMarker(()->arm.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                    .strafeLeft(22)
                    .waitSeconds(0.5)
                    //.addDisplacementMarker(()->arm.update(ArmSubsystem.ARM_TARGET_POSITION.HIGHPOS))
                    //.strafeRight(15)

                    .build();
        }
        else if (odPropProcessor.getResults() == PropDetectionProcessor.POSITION.LEFT){
            trSequence = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, accelerationConstraint,maxAngularVelocity,maxAngularAcceleration)
                    .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                    .forward(28)
                    .turn(Math.toRadians(-90))
                    .strafeLeft(21)
                    .addDisplacementMarker(()->arm.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))

                    .waitSeconds(0.5)

                    .strafeLeft(10)
                    .build();
        }
        else{
            trSequence = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, accelerationConstraint,maxAngularVelocity,maxAngularAcceleration)
                    .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                    .forward(27.5)
                    .addDisplacementMarker(()->arm.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                    .back(8)
                    .waitSeconds(0.5)
                    .build();
        }




        trajectorySequenceRunner = new TrajectorySequenceRunner(trajectoryFollower, new PIDCoefficients(RotationKp, RotationKi, RotationKd));

        armThread = new Thread(arm);
        register(arm);

    }


    @Override
    public void run() {

        if (isOpModeStarted ==false){
            trajectorySequenceRunner.followTrajectorySequenceAsync(trSequence);
            swerve.resetAllEncoders();
            armThread.start();

            hardware.startIMUThread(this);
            isOpModeStarted = true;
        }



        hardware.clearBulkCache();
        swerve.periodic();

        robotPosition = new Pose2d(
                swerve.robotPosition.getX() / odometryTickToInchRatio,
                swerve.robotPosition.getY() / odometryTickToInchRatio,
                hardware.imuAngle.getRadians());

//            robotPosition = new Pose2d(swerve.robotPosition.getX() / odometryTickToInchRatio,
//                                       swerve.robotPosition.getY() / odometryTickToInchRatio,
//                                          hardware.imuAngle.getRadians());

        telemetry.addData("Robot position", robotPosition);

        Pose2d currentRobotVelocity = new Pose2d(
                swerve.getChassisSpeedFromEncoders().vyMetersPerSecond / odometryTickToInchRatio,
                -swerve.getChassisSpeedFromEncoders().vxMetersPerSecond / odometryTickToInchRatio,
                0);

        impulse = trajectorySequenceRunner.update(robotPosition, currentRobotVelocity);

        double turnCorrection = getHeadingCorrection(impulse);
        Vector2d driveCorrection = getTranslationCorrection(impulse);

        telemetry.addData("turn Correction", turnCorrection);
        if (impulse != null) {
            telemetry.addData("signal vel", impulse.getVel().getHeading());
            telemetry.addData("signal accel", impulse.getAccel().getHeading());
        }

        telemetry.addData("driveCorrection", driveCorrection);
        telemetry.addData("xCorrection", driveCorrection.getX());
        telemetry.addData("yCorrection", driveCorrection.getY());


        if (impulse != null) {
            swerve.drive(driveCorrection, turnCorrection);
        } else {
            swerve.drive(new com.arcrobotics.ftclib.geometry.Vector2d(0, 0), 0);
            swerve.stopAllMotors();
        }


        if (isStopRequested()) {
            while (!file.hasWrote){
                file.PushCalibrationData(hardware.imuAngle.getRadians(), swerve.getAllModuleAngleRads());
            }
        }
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
        if (signal != null) return signal.getVel().getHeading() * rotationKv;
        else return 0;
    }



    private Vector2d getTranslationCorrection (DriveSignal signal){
        if (signal != null) {
            Vector2d driveVelocityCorrectionVector = new Vector2d(-signal.getVel().getY(), signal.getVel().getX());
            driveVelocityCorrectionVector = driveVelocityCorrectionVector.scale(translationKv);
            return driveVelocityCorrectionVector;
        }
        else return new Vector2d(0, 0);
    }
}