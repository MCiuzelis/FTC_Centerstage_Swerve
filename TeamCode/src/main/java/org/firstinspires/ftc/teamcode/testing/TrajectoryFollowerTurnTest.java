package org.firstinspires.ftc.teamcode.testing;

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
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceRunner;

@Disabled
@Config
@Autonomous(name = "CTurn Test", group = "OpMode:Test")
public class TrajectoryFollowerTurnTest extends OpMode {

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



    TrajectorySequence trSequence;
    Pose2d robotPosition = new Pose2d(0,0,0);
    TrajectoryAccelerationConstraint accelerationConstraint;
    SwerveVelocityConstraint swerveVelocityConstraint;
    TrajectorySequenceRunner trajectorySequenceRunner;
    TrajectoryFollower trajectoryFollower;
    DrivetrainSubsystem swerve;
    RobotHardware hardware = new RobotHardware(hardwareMap);
    DriveSignal impulse = new DriveSignal();
    ArmSubsystem arm;

    Thread armThread;



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware.initialiseHardware(telemetry);
        //hardware.startIMUThread(this);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        swerve = new DrivetrainSubsystem(hardware, telemetry, false);

        trajectoryFollower = new HolonomicPIDVAFollower(new PIDCoefficients(TranslationKp, TranslationKi, TranslationKd), new PIDCoefficients(TranslationKp, TranslationKi, TranslationKd),new PIDCoefficients(RotationKp, RotationKi, RotationKd)) ;
        swerveVelocityConstraint = new SwerveVelocityConstraint(maxWheelVelocity, width, base);
        accelerationConstraint = new ProfileAccelerationConstraint(profileAccelerationConstraint);
        arm = new ArmSubsystem(hardware, telemetry, true);
        swerve.resetAllEncoders();

        trSequence = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, accelerationConstraint,maxAngularVelocity,maxAngularAcceleration)
                .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                //.addTemporalMarker(()->arm.update(ArmSubsystem.ARM_TARGET_POSITION.HIGHPOS))
                //.addDisplacementMarker(()->CommandScheduler.getInstance().schedule(new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN)))
                //.waitSeconds(1)
                .forward(20)
                .turn(Math.toRadians(90))
                .strafeRight(20)
                //.back(40)
                .waitSeconds(30)
                .build();

        trajectorySequenceRunner = new TrajectorySequenceRunner(trajectoryFollower, new PIDCoefficients(RotationKp, RotationKi, RotationKd));

        armThread = new Thread(arm);

    }

    @Override
    public void start() {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trSequence);
        swerve.resetAllEncoders();

        armThread.start();

//
//        Thread swerveThread = new Thread(swerve);
//        swerveThread.start();
    }

//    private static DriveSignal normalizeVector(DriveSignal signal) {
//
//        Pose2d heading = signal.component2();
//        double targetX = signal.component1().getX();
//        double targetY = signal.component1().getY();
//        double distance = Math.sqrt(targetX * targetX + targetY * targetY);
//
//        if (distance == 0) {
//            return new DriveSignal(new Pose2d(0,0), new Pose2d(0)); // Return (0, 0) for zero-length vector
//        }
//
//        double scaleFactor = Math.min(1.0 / distance, 1.0);
//
//        double directionalX = targetX * scaleFactor;
//        double directionalY = targetY * scaleFactor;
//
//        return new DriveSignal(new Pose2d(directionalX,directionalY));
//    }



    private double getHeadingCorrection (DriveSignal signal){
        if (signal != null) {
            return signal.getVel().getHeading() * rotationKv;
        }
        else{
            return 0;
        }
    }



    private double getHeadingCorrectionWithAcceleration (DriveSignal signal){
        return  signal.getVel().getHeading() * rotationKv + signal.getAccel().getHeading() * rotationKa;
    }


    private Vector2d getTranslationCorrection (DriveSignal signal){
        if (signal != null) {
            Vector2d driveVelocityCorrectionVector = new Vector2d(-signal.getVel().getY(), signal.getVel().getX());
            driveVelocityCorrectionVector = driveVelocityCorrectionVector.scale(translationKv);
            return driveVelocityCorrectionVector;
        }
        else{
            return new Vector2d(0, 0);
        }
    }



    private Vector2d getTranslationCorrectionWithAcceleration (DriveSignal signal){

        double xCorrection = signal.getVel().getX() * translationKv + signal.getAccel().getX() * translationKa;
        double yCorrection = signal.getVel().getY() * translationKv + signal.getAccel().getY() * translationKa;
        return new Vector2d(yCorrection, xCorrection);
    }



    @Override
    public void loop() {
        hardware.updateIMUStupidMonkeyMethod();
        hardware.clearBulkCache();
        swerve.periodic();
        //arm.periodic();



        robotPosition = new Pose2d(swerve.robotPosition.getX() / ((((1+(46d/17d))) * (1+(46d/17))) * 28) * (25d  / 18) * 2 * Math.PI * 1.5f, swerve.robotPosition.getY() / ((((1+(46d/17))) * (1+(46d/17))) * 28) * (25d  / 18) * 2 * Math.PI * 1.5f, hardware.imuAngle.getRadians());
        telemetry.addData("Robot position", robotPosition);

        Pose2d currentRobotVelocity = new Pose2d(
                                                    swerve.getChassisSpeedFromEncoders().vyMetersPerSecond / ((((1+(46d/17d))) * (1+(46d/17))) * 28) * (25d  / 18) * 2 * Math.PI * 1.5d,
                                                    -swerve.getChassisSpeedFromEncoders().vxMetersPerSecond / ((((1+(46d/17))) * (1+(46d/17))) * 28) * (25d  / 18) * 2 * Math.PI * 1.5d,
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



        if (impulse != null){
            swerve.drive(driveCorrection, turnCorrection);
        }

        else{
            swerve.drive(new com.arcrobotics.ftclib.geometry.Vector2d(0,0), 0);
            swerve.stopAllMotors();
        }

        telemetry.update();
    }
}

