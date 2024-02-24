package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

public class TrajectoriesNew {
    Telemetry telemetry;
    ArmSubsystem armSubsystem;

    public static double angularVelocityConstraint = 0;
    public static double angularAccelerationConstraint = 0;
    public static double translationalVelocityConstraint = 0;
    public static double translationalAccelerationConstraint = 0;

    Pose2d blueStartingPosition = new Pose2d();
    Pose2d redStartingPosition = new Pose2d();

    TrajectoryVelocityConstraint trajectoryVelocityConstraint = new TranslationalVelocityConstraint(translationalVelocityConstraint);
    TrajectoryAccelerationConstraint trajectoryAccelerationConstraint = new ProfileAccelerationConstraint(translationalAccelerationConstraint);


    public boolean stopOpMode = false;


    public TrajectoriesNew(ArmSubsystem armSubsystem, Telemetry telemetry) {
        this.armSubsystem = armSubsystem;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    public TrajectorySequence buildTwoPixelBackboardAuto(PropDetectionProcessor.POSITION position, PropDetectionProcessor.COLOR color) {

        double inverter;
        Pose2d startingPosition = new Pose2d();
        if (color == PropDetectionProcessor.COLOR.BLUE) {
            inverter = 1;
            startingPosition = blueStartingPosition;
        }
        else if (color == PropDetectionProcessor.COLOR.RED) {
            inverter = -1;
            startingPosition = redStartingPosition;
        }




        if (color == PropDetectionProcessor.COLOR.BLUE) {
            switch (position) {
                case RIGHT:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .build();

                case LEFT:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .build();

                case MIDDLE:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .addTemporalMarker(()-> buildBackboardCycle(new Pose2d(0,0,0)))
                            .build();

                default:
                    throw new RuntimeException("bad");
            }
        } else return null;
    }


    public TrajectorySequence buildBackboardCycle (Pose2d startingPosition){
        return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                .build();
    }
}