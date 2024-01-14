package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.hardware.Constants.backLocation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.frontLeftLocation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.frontRightLocation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.constraint.SwerveDriveKinematicsConstraint;
import com.arcrobotics.ftclib.trajectory.constraint.TrajectoryConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;

@Disabled
@Config
@TeleOp(name = "Trajectories", group = "OpMode")
public class TrajectoryTestOpMode extends OpMode {
    RobotHardware robot = new RobotHardware(hardwareMap);
    DrivetrainSubsystem drivetrainSubsystem;
    public static double maxVelocity = 0;
    public static double maxAcceleration = 0;
    Trajectory trajectory;
    ElapsedTime timer = new ElapsedTime();


    public static double Kp = 0;

    @Override
    public void init() {

        robot.initialiseHardware(telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrainSubsystem.resetAllEncoders();



        Pose2d sideStart = new Pose2d(0, 0,
                Rotation2d.fromDegrees(0));
        Pose2d crossScale = new Pose2d(0.5, 0.5,
                Rotation2d.fromDegrees(0));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(0, 0.5));


        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLocation);
        TrajectoryConstraint SwerveDriveKinematicsConstraint = new SwerveDriveKinematicsConstraint(kinematics, 0);
        config.addConstraint(SwerveDriveKinematicsConstraint);
        config.setReversed(false);

        trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
    }


    @Override
    public void start(){
        timer.reset();
    }

    @Override
    public void loop(){
        double currentX = 0;
        double currentY = 0;

        Pose2d goal = trajectory.sample(timer.seconds()).poseMeters;

        double correctionX = (goal.getX() - currentX) * Kp;
        double correctionY = (goal.getY() - currentY) * Kp;

        Vector2d drive = new Vector2d(correctionX, correctionY);

        drivetrainSubsystem.drive(drive, 0);

        if (gamepad1.left_bumper) timer.reset();
    }
}