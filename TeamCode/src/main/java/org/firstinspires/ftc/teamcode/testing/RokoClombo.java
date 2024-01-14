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
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceRunner;

@Disabled
@Config
@Autonomous(name = "RokoExample", group = "OpMode:Test")
public class RokoClombo extends OpMode {

    public static double profileAccelerationConstraint = 6;
    public static double waitSeconds = 2;
    public static double maxWheelVelocity = 8;

    public static double maxAngularVelocity = 1;
    public static double maxAngularAcceleration = 1;

    public static double PIDVAKpx = 0.065;
    public static double PIDVAKpy = -0.065;

    public static double PIDVAKix = 0.1;
    public static double PIDVAKiy = -0.1;

    public static double PIDVAKdx = -1;
    public static double PIDVAKdy = -1;

    public static double PIDVAKpth = 0;
    public static double PIDVAKith = 0;
    public static double PIDVAKdth = 0;

    private com.arcrobotics.ftclib.geometry.Vector2d normalizedVector;

    TrajectorySequence trSequence;
    Pose2d robotPosition = new Pose2d(0,0,0);
    TrajectoryAccelerationConstraint accelerationConstraint;
    SwerveVelocityConstraint swerveVelocityConstraint;
    TrajectorySequenceRunner trajectorySequenceRunner;
    TrajectoryFollower trajectoryFollower;
    DrivetrainSubsystem swerve;
    RobotHardware hardware = new RobotHardware(hardwareMap);
    DriveSignal impulse = new DriveSignal();
    public static double base = 10;
    public static double width = 10;
    public static double stupidX = 0.01;
    public static double stupidY = 0.01;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware.initialiseHardware(telemetry);
        //hardware.startIMUThread(this);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        swerve = new DrivetrainSubsystem(hardware, telemetry, false);

        trajectoryFollower = new HolonomicPIDVAFollower(new PIDCoefficients(PIDVAKpx,PIDVAKix,PIDVAKdx), new PIDCoefficients(PIDVAKpy,PIDVAKiy,PIDVAKdy),new PIDCoefficients(PIDVAKpth,PIDVAKith,PIDVAKdth)) ;
        swerveVelocityConstraint = new SwerveVelocityConstraint(maxWheelVelocity, width, base);
        accelerationConstraint = new ProfileAccelerationConstraint(profileAccelerationConstraint);
        swerve.resetAllEncoders();

        trSequence = new TrajectorySequenceBuilder(robotPosition,swerveVelocityConstraint, accelerationConstraint,maxAngularVelocity,maxAngularAcceleration)
                .setTurnConstraint(maxAngularVelocity,maxAngularAcceleration)
                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(0 , 10))
                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(10 , 0))

                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(0 , -10))

                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(-10 , 0))
                .waitSeconds(1)
                .build();

        trajectorySequenceRunner = new TrajectorySequenceRunner(trajectoryFollower, new PIDCoefficients(PIDVAKpth,PIDVAKith,PIDVAKdth));

    }

    @Override
    public void start() {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trSequence);

        swerve.resetAllEncoders();
    }

    private static DriveSignal normalizeVector(DriveSignal signal) {
        Pose2d heading = signal.component2();
        double targetX = signal.component1().getX();
        double targetY = signal.component1().getY();
        double distance = Math.sqrt(targetX * targetX + targetY * targetY);

        if (distance == 0) {
            return new DriveSignal(new Pose2d(0,0), new Pose2d(0)); // Return (0, 0) for zero-length vector
        }

        double scaleFactor = Math.min(1.0 / distance, 1.0);

        double directionalX = targetX * scaleFactor;
        double directionalY = targetY * scaleFactor;

        return new DriveSignal(new Pose2d(directionalX,directionalY));
    }

    @Override
    public void loop() {
        hardware.updateIMUStupidMonkeyMethod();

        hardware.clearBulkCache();
        swerve.periodic();

        double ticksToInchRatio = ((((1+(46d/17))) * (1+(46d/17))) * 28) * (25d  / 18) * 2 * Math.PI * 1.5f;

        robotPosition = new Pose2d(swerve.robotPosition.getX() / ticksToInchRatio, swerve.robotPosition.getY() / ticksToInchRatio, swerve.robotPosition.getRotation().getRadians());
        telemetry.addData("Robot position", robotPosition);
        telemetry.addData("Robot position(swerve)", swerve.robotPosition);

        ChassisSpeeds currentVelocity = swerve.getChassisSpeedFromEncoders();
        impulse = trajectorySequenceRunner.update(robotPosition, new Pose2d(currentVelocity.vyMetersPerSecond, currentVelocity.vxMetersPerSecond));


        telemetry.addData("Impulse", impulse);
        normalizedVector = new com.arcrobotics.ftclib.geometry.Vector2d(normalizeVector(impulse).component1().getY(), normalizeVector(impulse).component1().getX());

        telemetry.addData("Turning vecor", impulse.component2().headingVec());
        telemetry.addData("Turning idk what2", impulse.component2().component2());
        telemetry.addData("Turning idk what3", impulse.component2().component3());
        telemetry.addData("Turning idk what", impulse.component2().getHeading());


        if (impulse != null){
            swerve.drive(normalizedVector, impulse.component2().getHeading());
        }
//        if (impulse != null){
//            if (!trajectorySequenceRunner.isBusy()){
//                swerve.drive(new com.arcrobotics.ftclib.geometry.Vector2d(0,0), 0);
//                swerve.stopAllMotors();
//            }
//        }
        else{
            swerve.drive(new com.arcrobotics.ftclib.geometry.Vector2d(0,0), 0);
            swerve.stopAllMotors();
            telemetry.addLine("Halt");
        }
        telemetry.addData("Impulse normal", normalizedVector);
        telemetry.addData("X", normalizedVector.getX());
        telemetry.addData("Y", normalizedVector.getY());


        telemetry.update();
    }
}

