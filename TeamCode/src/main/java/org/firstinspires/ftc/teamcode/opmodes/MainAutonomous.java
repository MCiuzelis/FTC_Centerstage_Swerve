package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;

@Autonomous(name = "MainAutonomous", group = "OpMode")
public class MainAutonomous extends OpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(robot, telemetry, false);
    ElapsedTime timer = new ElapsedTime();

    String teamPropLocation = "center";


    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void start(){
        timer.reset();

        switch (teamPropLocation) {
            case "center":
                driveForSeconds(0, 100, 0.4);
            case "left":
                driveForSeconds(45, 100, 0.4);
            case "right":
                driveForSeconds(-45, 100, 0.4);
        }
    }

    @Override
    public void loop() {
    }


    void driveForSeconds (double angleDegrees, double timeMilliseconds, double speedMultiplier){
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.milliseconds() < timeMilliseconds){
            drivetrain.Drive(new Vector2d(0, 0.4), 0);
        }
        drivetrain.stop();
    }

    public void generateTrajectory() {

        Pose2d sideStart = new Pose2d(1.54, 23.23,
                Rotation2d.fromDegrees(-180));
        Pose2d crossScale = new Pose2d(23.7, 6.8,
                Rotation2d.fromDegrees(-160));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
        interiorWaypoints.add(new Translation2d(14.54, 23.23));
        interiorWaypoints.add(new Translation2d(21.04, 18.23));

        TrajectoryConfig config = new TrajectoryConfig(12, 12);
        config.setReversed(true);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
    }
}