package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
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
@Autonomous(name = "Squared Eight Test", group = "OpMode:Test")
public class TrajectoryFollowerSquared8Test extends OpMode {

    public static double profileAccelerationConstraint = 6;
    public static double waitSeconds = 2;
    public static double maxWheelVelocity = 8;
    public static double PIDVAKpx = 0.065;
    public static double PIDVAKpy = -0.065;

    public static double PIDVAKix = 0.1;
    public static double PIDVAKiy = -0.1;

    public static double PIDVAKdx = -1;
    public static double PIDVAKdy = -1;

    public static double PIDVAKpth = 0.0000;

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


        hardware.initialiseHardware(telemetry);
        //hardware.startIMUThread(this);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        swerve = new DrivetrainSubsystem(hardware, telemetry, false);

        trajectoryFollower = new HolonomicPIDVAFollower(new PIDCoefficients(PIDVAKpx,PIDVAKix,PIDVAKdx), new PIDCoefficients(PIDVAKpy,PIDVAKiy,PIDVAKdy),new PIDCoefficients(PIDVAKpth,0,0)) ;
        swerveVelocityConstraint = new SwerveVelocityConstraint(maxWheelVelocity, width, base);
        accelerationConstraint = new ProfileAccelerationConstraint(profileAccelerationConstraint);
        swerve.resetAllEncoders();

        trSequence = new TrajectorySequenceBuilder(robotPosition,swerveVelocityConstraint, accelerationConstraint,180,180)
//
                .strafeLeft(60, swerveVelocityConstraint, accelerationConstraint)
                .waitSeconds(2)
                .back(60, swerveVelocityConstraint, accelerationConstraint)
                .waitSeconds(2)
                .strafeRight(60, swerveVelocityConstraint, accelerationConstraint)
                .waitSeconds(2)
                .forward(60, swerveVelocityConstraint, accelerationConstraint)
                .waitSeconds(2)
                .strafeRight(60, swerveVelocityConstraint, accelerationConstraint)
                .waitSeconds(2)
                .back(60, swerveVelocityConstraint, accelerationConstraint)
                .waitSeconds(2)
                .strafeLeft(60, swerveVelocityConstraint, accelerationConstraint)
                .waitSeconds(2)
                .forward(60, swerveVelocityConstraint, accelerationConstraint)

                .build();

        trajectorySequenceRunner = new TrajectorySequenceRunner(trajectoryFollower, new PIDCoefficients(PIDVAKpth,0,0));

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


        hardware.clearBulkCache();
        swerve.periodic();


        robotPosition = new Pose2d(swerve.robotPosition.getX() / ((((1+(46/17))) * (1+(46/17))) * 28) * (25  / 18) * 2 * Math.PI * 1.5f, swerve.robotPosition.getY() / ((((1+(46/17))) * (1+(46/17))) * 28) * (25  / 18) * 2 * Math.PI * 1.5f, swerve.robotPosition.getHeading());
        telemetry.addData("Robot position", robotPosition);
        telemetry.addData("Robot position(swerve)", swerve.robotPosition);

        impulse = trajectorySequenceRunner.update(robotPosition, new Pose2d(stupidX, stupidY,0));

        telemetry.addData("Impulse", impulse);
        normalizedVector = new com.arcrobotics.ftclib.geometry.Vector2d(normalizeVector(impulse).component1().getY(), normalizeVector(impulse).component1().getX());

        if (impulse != null){
            swerve.drive(normalizedVector, 0);
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

