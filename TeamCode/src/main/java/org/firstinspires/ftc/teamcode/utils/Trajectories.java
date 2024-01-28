package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.opmodes.MainAutonomous.maxAngularAcceleration;
import static org.firstinspires.ftc.teamcode.opmodes.MainAutonomous.maxAngularVelocity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

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
        //(0, 28) -> left
        //(28, 0) -> forward
        //(28, 0) -> back
        //(0, 28) -> right
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
                                .forward(28)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(31, 0))
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.LOWPOS))
                                .lineTo(new Vector2d(56, 0))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                                .waitSeconds(2)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.TRANSFER))
                                .waitSeconds(1.5)
                                .lineTo(new Vector2d(49, 0))

                                .waitSeconds(1)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.PICKUP))
                                .addDisplacementMarker(() -> stopOpMode = true)
                                .build();
                        break;


                    case LEFT:
                        trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                                .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                                .forward(25)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(46, 0))
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(47, 0))
                                .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.LOWPOS))
                                .waitSeconds(2)
                                .lineTo(new Vector2d(53, 0))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                                .waitSeconds(2)
                                .lineTo(new Vector2d(51.5, 0))
                                .waitSeconds(2)
                                .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.TRANSFER))
                                .waitSeconds(1)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.PICKUP))
                                .waitSeconds(1)
                                .addDisplacementMarker(() -> stopOpMode = true)
                                .build();
                        break;


                    case MIDDLE:
                        trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                                .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                                .forward(26)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                                .waitSeconds(0.5)
                                .back(3.5)
                                .turn(Math.toRadians(-90 * inverter))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.LOWPOS))
                                .lineTo(new Vector2d(51.5, 0))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.BOTH_OPEN))
                                .waitSeconds(2)
                                .lineTo(new Vector2d(49, 0))
                                .waitSeconds(2)
                                .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.TRANSFER))
                                .waitSeconds(2)
                                .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.PICKUP))
                                .waitSeconds(2)
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
                            .forward(27)
                            .turn(Math.toRadians(90))
                            .waitSeconds(0.5)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                            .waitSeconds(0.5)
                            .lineTo(new Vector2d(31, 0))
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.LOWPOS))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(56, 0))
                            .waitSeconds(0.5)
                            .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                            .waitSeconds(2)
                            .lineTo(new Vector2d(54, 0))
                            .waitSeconds(1)
                            .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.TRANSFER))
                            .waitSeconds(1)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.PICKUP))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(58, 0))
                            .addDisplacementMarker(() -> stopOpMode = true)
                            .build();
                    break;


                case RIGHT:
                    trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                            .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                            .forward(23)
                            .turn(Math.toRadians(90))
                            .waitSeconds(0.5)
                            .lineTo(new Vector2d(44.5, 10))
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                            .waitSeconds(0.5)
                            .lineTo(new Vector2d(45.75, 10))
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.LOWPOS))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(50, 10))
                            .waitSeconds(0.5)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(49, 10))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.TRANSFER))
                            .waitSeconds(2)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.PICKUP))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(51, 10))
                            .addDisplacementMarker(() -> stopOpMode = true)
                            .build();
                    break;


                case MIDDLE:
                    trajectory = new TrajectorySequenceBuilder(robotPosition, swerveVelocityConstraint, trajectoryAccelerationConstraint, maxAngularVelocity, maxAngularAcceleration)
                            .setTurnConstraint(maxAngularVelocity, maxAngularAcceleration)
                            .forward(26)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.LEFT_OPENED))
                            .waitSeconds(0.5)
                            .back(3.5)
                            .turn(Math.toRadians(-90 * inverter))
                            .waitSeconds(0.5)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.LOWPOS))
                            .lineTo(new Vector2d(51, 0))
                            .waitSeconds(0.5)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.CLAW_STATE.RIGHT_OPEN))
                            .waitSeconds(1)
                            .addTemporalMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.TRANSFER))
                            .waitSeconds(2)
                            .lineTo(new Vector2d(49.5, 0))
                            .waitSeconds(2)
                            .addDisplacementMarker(() -> armSubsystem.update(ArmSubsystem.AXON_STATE.PICKUP))
                            .waitSeconds(1)
                            .lineTo(new Vector2d(52, 0))
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