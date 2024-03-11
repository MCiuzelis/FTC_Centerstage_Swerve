package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SetArmToStateCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

public class TrajectoriesNewNew {
    Telemetry telemetry;
    ArmSubsystem armSubsystem;

    public static double angularVelocityConstraint = 3;
    public static double angularAccelerationConstraint = 3;
    public static double translationalVelocityConstraint = 6;
    public static double translationalAccelerationConstraint = 6;

    Pose2d blueStartingPosition = new Pose2d();
    Pose2d redStartingPosition = new Pose2d();

    TrajectoryVelocityConstraint trajectoryVelocityConstraint = new TranslationalVelocityConstraint(translationalVelocityConstraint);
    TrajectoryAccelerationConstraint trajectoryAccelerationConstraint = new ProfileAccelerationConstraint(translationalAccelerationConstraint);


    public boolean stopOpMode = false;


    public TrajectoriesNewNew(ArmSubsystem armSubsystem, Telemetry telemetry) {
        this.armSubsystem = armSubsystem;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public TrajectoriesNewNew(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    public TrajectorySequence buildTwoPixelBackboardAuto(PropDetectionProcessor.POSITION position, PropDetectionProcessor.COLOR color) {

        double inverter;
        Pose2d startingPosition = new Pose2d();
        if (color == PropDetectionProcessor.COLOR.BLUE) {
            inverter = 1;
            startingPosition = blueStartingPosition;
        } else if (color == PropDetectionProcessor.COLOR.RED) {
            inverter = -1;
            startingPosition = redStartingPosition;
        }


        if (color == PropDetectionProcessor.COLOR.BLUE) {
            switch (position) {
                case RIGHT:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                            .forward(10)
                            .turn(Math.toRadians(-90))
                            .forward(2.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                            .waitSeconds(1)
                            .back(4)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_CLOSED)))
                            .back(16)
                            .strafeLeft(0.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.LEFT_OPENED)))
                            .waitSeconds(0.2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(()->armSubsystem.offsetLift(-50))))

                            .forward(2)
                            .strafeRight(4)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.TRANSFER)))
                            .strafeRight(7)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(200)
                            .addTemporalMarker(() -> stopOpMode = true)
                            .build();

                case LEFT:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                            .forward(10)
                            .turn(Math.toRadians(-90))
                            .back(6.25)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                            .waitSeconds(0.5)
                            .back(2)
                            .waitSeconds(0.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .waitSeconds(1)
                            .back(5)
                            .strafeRight(4.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.LEFT_OPENED)))
                            .waitSeconds(0.2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(()->armSubsystem.offsetLift(-50))))

                            .forward(2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.TRANSFER)))
                            .strafeRight(7 )
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(200)
                            .addTemporalMarker(() -> stopOpMode = true)
                            .build();

                case MIDDLE:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                            .forward(10.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                            .waitSeconds(0.5)
                            .back(3.5)
                            .waitSeconds(1)
                            .turn(Math.toRadians(-90))
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_CLOSED)))
                            .back(16)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.LEFT_OPENED)))
                            .waitSeconds(0.2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(()->armSubsystem.offsetLift(-50))))

                            .forward(2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.TRANSFER)))
                            .strafeRight(8.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(200)
                            .addTemporalMarker(() -> stopOpMode = true)
                            .build();

                default:
                    throw new RuntimeException("bad");
            }
        }




        //RED
        else {
            switch (position) {
                case LEFT:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                            .forward(8.5)
                            .turn(Math.toRadians(90))
                            .forward(1)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                            .waitSeconds(1)
                            .back(3)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_CLOSED)))
                            .back(11)
                            //.strafeRight(0.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.LEFT_OPENED)))
                            .waitSeconds(0.2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(()->armSubsystem.offsetLift(-50))))

                            .forward(3)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.TRANSFER)))
                            .strafeLeft(11)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(200)
                            .addTemporalMarker(() -> stopOpMode = true)
                            .build();


                case RIGHT:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                            .forward(10)
                            .turn(Math.toRadians(90))
                            .back(7.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                            .waitSeconds(0.5)
                            .back(2)
                            .waitSeconds(0.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .waitSeconds(1)
                            .back(2.25)
                            .strafeLeft(4.5)
                            .back(1.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.LEFT_OPENED)))
                            .waitSeconds(0.2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(()->armSubsystem.offsetLift(-50))))

                            .forward(2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.TRANSFER)))
                            .strafeLeft(7 )
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(200)
                            .addTemporalMarker(() -> stopOpMode = true)
                            .build();

                case MIDDLE:
                    return new TrajectorySequenceBuilder(startingPosition, trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                            .forward(10.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                            .waitSeconds(0.5)
                            .back(3.5)
                            .waitSeconds(1)
                            .turn(Math.toRadians(90))
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)))
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_CLOSED)))
                            .back(15.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.LEFT_OPENED)))
                            .waitSeconds(0.2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(()->armSubsystem.offsetLift(-50))))

                            .forward(2)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.TRANSFER)))
                            .strafeLeft(8.5)
                            .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)))
                            .waitSeconds(200)
                            .addTemporalMarker(() -> stopOpMode = true)
                            .build();

                default: return null;
            }
        }
    }




    public TrajectorySequence buildTwoPixelFarAuto(PropDetectionProcessor.POSITION position, PropDetectionProcessor.COLOR color) {
        switch (position) {
            case RIGHT:
                return new TrajectorySequenceBuilder(new Pose2d(), trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                        .forward(9)
                        .turn(Math.toRadians(-90))
                        .forward(1.5)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                        .waitSeconds(1)
                        .back(3)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_CLOSED)))
                        .addTemporalMarker(() -> stopOpMode = true)
                        .build();

            case LEFT:
                return new TrajectorySequenceBuilder(new Pose2d(), trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                        .forward(9)
                        .turn(Math.toRadians(90))
                        .forward(1.5)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                        .waitSeconds(1)
                        .back(3)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_CLOSED)))
                        .addTemporalMarker(() -> stopOpMode = true)
                        .build();

            case MIDDLE:
                return new TrajectorySequenceBuilder(new Pose2d(), trajectoryVelocityConstraint, trajectoryAccelerationConstraint, angularVelocityConstraint, angularAccelerationConstraint)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)))
                        .forward(10.25)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_OPEN)))
                        .waitSeconds(0.5)
                        .back(3)
                        .waitSeconds(1)
                        .addTemporalMarker(() -> CommandScheduler.getInstance().schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.RIGHT_CLOSED)))
                        .waitSeconds(200)
                        .addTemporalMarker(() -> stopOpMode = true)
                        .build();
            default:
                throw new RuntimeException("bad");
        }
    }
}