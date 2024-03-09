package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.hardware.Localizer;


@Photon
@Config
@Autonomous(name = "AutonomousTrackballTesting")

public class AutonomousTrackballTesting extends CommandOpMode {
    public static double maxProfileAcceleration = 6.5;
    public static double maxTranslationalVelocity = 6.5;

    public static double maxAngularVelocity = 2.5;
    public static double maxAngularAcceleration = 2.5;

    public static double TranslationKp = 0.017;
    public static double TranslationKi = 0;
    public static double TranslationKd = 0;
    public static double translationKv = 0.0089;
    public static double translationKa = 0.0086;

    public static double RotationKp = 0.11;
    public static double RotationKi = 0;
    public static double RotationKd = 0;
    public static double rotationKv = 0.34;
    public static double rotationKa = 0.026;

    Pose2d startPose =  new Pose2d();

    boolean isOpModeStarted = false;

    RobotHardware hardware;
    DrivetrainSubsystem swerve;
    Localizer localizer;

    TrajectoryAccelerationConstraint accelerationConstraint;
    TrajectoryVelocityConstraint trajectoryVelocityConstraint;
    TrajectorySequenceRunner trajectorySequenceRunner;
    TrajectoryFollower trajectoryFollower;



    @Override
    public void initialize() {
        trajectoryFollower = new HolonomicPIDVAFollower(new PIDCoefficients(TranslationKp, TranslationKi, TranslationKd), new PIDCoefficients(TranslationKp, TranslationKi, TranslationKd),new PIDCoefficients(RotationKp, RotationKi, RotationKd)) ;
        accelerationConstraint = new ProfileAccelerationConstraint(maxProfileAcceleration);
        trajectoryVelocityConstraint = new TranslationalVelocityConstraint(maxTranslationalVelocity);
        trajectorySequenceRunner = new TrajectorySequenceRunner(trajectoryFollower, new PIDCoefficients(RotationKp, RotationKi, RotationKd));

        hardware = new RobotHardware(hardwareMap);
        hardware.initialiseHardware(telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        localizer = new Localizer(hardware, telemetry, true);
        swerve = new DrivetrainSubsystem(hardware, telemetry, false, true);

        swerve.resetAllEncoders();
        swerve.resetImuOffset();
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }



    @Override
    public void run() {
        if (!isOpModeStarted){
            TrajectorySequence trSequence = new TrajectorySequenceBuilder(startPose, trajectoryVelocityConstraint, accelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                    .forward(37)
                    .waitSeconds(1.5)
                    .back(10)
                    .waitSeconds(1.5)
                    .turn(Math.toRadians(-90))
                    .waitSeconds(1.5)
                    .back(30)
                    .build();
            trajectorySequenceRunner.followTrajectorySequenceAsync(trSequence);
            localizer.reset();

            hardware.startIMUThread(this);
            isOpModeStarted = true;
        }

        hardware.clearBulkCache();
        swerve.updateModuleAngles();
        CommandScheduler.getInstance().run();

        localizer.updateOdometry();
        Pose2d robotPosition = localizer.getRobotCentricPosition();
        Pose2d robotVelocity = localizer.getRobotCentricVelocity();
        Pose2d fieldCentricPosition = localizer.getFieldCentricPosition();

        DriveSignal driveSignal = trajectorySequenceRunner.update(fieldCentricPosition, robotVelocity);
        Pose2d correction = getCorrection(driveSignal);
        swerve.setGamepadInput(correction);
        swerve.drive();

        telemetry.addData("positionX", robotPosition.getX());
        telemetry.addData("positionY", robotPosition.getY());
        telemetry.addData("fieldCentric X", fieldCentricPosition.getX());
        telemetry.addData("fieldCentric Y", fieldCentricPosition.getY());
        telemetry.addData("velocity", robotVelocity);
        telemetry.addData("turn Correction", correction.getHeading());
        telemetry.addData("xCorrection", correction.getX());
        telemetry.addData("yCorrection", correction.getY());

        if (driveSignal != null) {
            telemetry.addData("signal vel", driveSignal.getVel().getHeading());
            telemetry.addData("signal accel", driveSignal.getAccel().getHeading());
        }
        else{
            telemetry.addData("signal vel", 0);
            telemetry.addData("signal accel", 0);
        }
        telemetry.update();
    }




    private Pose2d getCorrection (DriveSignal signal) {
        if (signal != null) {
            Vector2d driveVelocityCorrection = new Vector2d(signal.getVel().getY(), signal.getVel().getX());
            Vector2d driveAccelerationCorrection = new Vector2d(signal.getAccel().getY(), signal.getVel().getX());
            driveVelocityCorrection = driveVelocityCorrection.times(translationKv);
            driveAccelerationCorrection = driveAccelerationCorrection.times(translationKa);

            double headingCorrection = signal.getVel().getHeading() * rotationKv + signal.getAccel().getHeading() * rotationKa;
            return new Pose2d(driveVelocityCorrection.plus(driveAccelerationCorrection), headingCorrection);
        } else return new Pose2d();
    }
}