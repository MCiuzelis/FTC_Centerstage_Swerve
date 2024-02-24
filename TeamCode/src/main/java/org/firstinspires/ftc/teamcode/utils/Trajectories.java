package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.opmodes.MainAutonomous.maxAngularAcceleration;
import static org.firstinspires.ftc.teamcode.opmodes.MainAutonomous.maxAngularVelocity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.SetArmToStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Set;

public class Trajectories {
    ArmSubsystem armSubsystem;
    Pose2d robotPosition = new Pose2d();
    SwerveVelocityConstraint swerveVelocityConstraint;
    TrajectoryAccelerationConstraint trajectoryAccelerationConstraint;
    public boolean stopOpMode = false;



    public Trajectories(ArmSubsystem arm, SwerveVelocityConstraint swerveVelocityConstraint, TrajectoryAccelerationConstraint trajectoryAccelerationConstraint) {
        armSubsystem = arm;
        this.swerveVelocityConstraint = swerveVelocityConstraint;
        this.trajectoryAccelerationConstraint = trajectoryAccelerationConstraint;
    }




    public TrajectorySequence generateTrajectoryCloseToBackboard(PropDetectionProcessor.POSITION position, PropDetectionProcessor.COLOR color){
        TrajectorySequence trajectory;

        double inverter;
        if (color == PropDetectionProcessor.COLOR.BLUE) inverter = 1;
        else if (color == PropDetectionProcessor.COLOR.RED) inverter = -1;
        else throw new RuntimeException("prop colour not specified, inverter = 0");


        if (color == PropDetectionProcessor.COLOR.BLUE){
                switch (position) {
                    case RIGHT:
                        trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                                .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                                .forward(27.5)
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(24, 0))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(32, 0))
                                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                                .lineTo(new Vector2d(65, 0))
                                .waitSeconds(0.1)
                                .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(60, 0))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                                .waitSeconds(5)
                                .addDisplacementMarker(() -> stopOpMode = true)
                                .build();
                        break;


                    case LEFT:
                        trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                                .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                                .forward(27)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(46, 0))
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(47.5, 0))
                                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(63, 0))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(58, 0))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                                .waitSeconds(5)
                                .addDisplacementMarker(() -> stopOpMode = true)
                                .build();
                        break;


                    case MIDDLE:
                        trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                                .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                                .forward(28)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                                .waitSeconds(0.5)
                                .back(3.5)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                                .lineTo(new Vector2d(63, 0))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(58, 0))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                                .waitSeconds(3)
                                .addDisplacementMarker(() -> stopOpMode = true)
                                .build();
                        break;

                    default:
                        throw new IllegalStateException("Unexpected trajectory value exception: " + position);
                }
        }
        else {
            switch (position) {
                case LEFT:
                    trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                            .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                            .forward(27.5)
                            .turn(Math.toRadians(90))
                            .lineTo(new Vector2d(24, 0))
                            .waitSeconds(0.5)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                            .waitSeconds(0.5)
                            .lineTo(new Vector2d(32, 0))
                            .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .lineTo(new Vector2d(65, 0))
                            .waitSeconds(0.1)
                            .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                            .waitSeconds(0.5)
                            .lineTo(new Vector2d(60, 0))
                            .waitSeconds(0.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(5)
                            .addDisplacementMarker(() -> stopOpMode = true)
                            .build();
                    break;


                case RIGHT:
                    trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                            .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                            .forward(27)
                            .turn(Math.toRadians(90))
                            .waitSeconds(0.5)
                            .lineTo(new Vector2d(46, 0))
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                            .waitSeconds(0.5)
                            .lineTo(new Vector2d(47.5, 0))
                            .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(63, 0))
                            .waitSeconds(0.5)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(58, 0))
                            .waitSeconds(0.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(5)
                            .addDisplacementMarker(() -> stopOpMode = true)
                            .build();
                    break;


                case MIDDLE:
                    trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                            .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                            .forward(28)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                            .waitSeconds(0.5)
                            .back(3.5)
                            .turn(Math.toRadians(90))
                            .waitSeconds(0.5)
                            .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .lineTo(new Vector2d(63, 0))
                            .waitSeconds(0.5)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(58, 0))
                            .waitSeconds(0.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(3)
                            .addDisplacementMarker(() -> stopOpMode = true)
                            .build();
                    break;

                default:
                    throw new IllegalStateException("Unexpected trajectory value exception: " + position);
            }
        }
        return trajectory;
    }









    public TrajectorySequence generateTrajectoryFarFromBackboard(PropDetectionProcessor.POSITION position, PropDetectionProcessor.COLOR color){
        TrajectorySequence trajectory;

        double inverter;
        if (color == PropDetectionProcessor.COLOR.BLUE) inverter = 1;
        else if (color == PropDetectionProcessor.COLOR.RED) inverter = -1;
        else throw new RuntimeException("prop colour not specified, inverter = 0");


        switch (position){
            case RIGHT:
                trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint,maxAngularVelocity,maxAngularAcceleration)
                        .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                        .forward(27)
                        .turn(Math.toRadians(-90))
                        .addDisplacementMarker(()->armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                        .waitSeconds(0.5)
                        .lineTo(new Vector2d(32, 0))
                        .addDisplacementMarker(()-> stopOpMode = true)
                        .build();
                break;




            case LEFT:
                trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint,maxAngularVelocity,maxAngularAcceleration)
                        .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                        .forward(26.75)
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d(27.5, 0))
                        .addDisplacementMarker(()->armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                        .waitSeconds(0.5)
                        .lineTo(new Vector2d(32, 0))
                        .addDisplacementMarker(()-> stopOpMode = true)
                        .build();
                break;




            case MIDDLE:
                trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint,maxAngularVelocity,maxAngularAcceleration)
                        .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                        .forward(25.5)
                        .addDisplacementMarker(()->armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                        .waitSeconds(0.5)
                        .back(5)
                        .addDisplacementMarker(()-> stopOpMode = true)
                        .build();
                break;



            default:
                throw new IllegalStateException("Unexpected trajectory value exception: " + position);
        }
        return trajectory;
    }
}