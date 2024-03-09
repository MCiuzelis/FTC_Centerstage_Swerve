package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetAxonAngleCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetClawAngleCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetSlideHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;

public class SetArmToStateCommand extends CommandBase {
    ArmState targetState;
    ArmSubsystem arm;

    BooleanSupplier moveToTransfer = ()-> !(arm.areSlidesDown() && arm.areAxonsCloseToTransferPos());
    BooleanSupplier isArmAtPickup = ()-> arm.areAxonsAtPickup();

    BooleanSupplier leftPixelInClaw = ()-> arm.leftPixelInClaw();
    BooleanSupplier rightPixelInClaw = ()-> arm.rightPixelInClaw();

    public SetArmToStateCommand(ArmSubsystem arm, ArmState state){
        targetState = state;
        this.arm = arm;
        addRequirements(arm);
    }


    @Override
    public void initialize(){
        switch (targetState){
            case PICKUP:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> arm.isArmUp = false),

                                new ConditionalCommand(
                                    new SequentialCommandGroup(
                                        new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.PICKUP),
                                        new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.PICKUP),
                                        new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.PICKUP),
                                        new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN)
                                    ),


                                    new SequentialCommandGroup(
                                            new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.PICKUP),
                                            new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.UPPER_TRANSFER),
                                            new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.TRANSFER),

                                            new ConditionalCommand(
                                                    new WaitCommand(600),
                                                    new WaitCommand(50),
                                                    moveToTransfer
                                            ),

                                            new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.PICKUP),
                                            new WaitCommand(50),
                                            new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.PICKUP),
                                            new WaitCommand(50),
                                            new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN)
                                    ),
                                    isArmAtPickup
                            ),


                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                        new WaitUntilCommand(leftPixelInClaw),
                                        new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.LEFT_CLOSED),
                                        new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.LEFT, true))
                                    ),
                                    new SequentialCommandGroup(
                                            new WaitUntilCommand(rightPixelInClaw),
                                            new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.RIGHT_CLOSED),
                                            new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.RIGHT, true))
                                    )
                            ),

                            new InstantCommand(()-> arm.rumble = true),
                            new SetArmToStateCommand(arm, ArmState.TRANSFER)
                        )
                );
                break;

            case TRANSFER:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.LEFT, false)),
                            new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.RIGHT, false)),
                            new InstantCommand(()-> arm.isArmUp = false),


                            new ConditionalCommand(
                                new SequentialCommandGroup(
                                    new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN),
                                    new WaitCommand(80),
                                    new InstantCommand(()-> arm.offsetLift(-90)),
                                    new WaitCommand(80)
                                ),
                                new SequentialCommandGroup(
                                    new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED),
                                    new WaitCommand(120)
                                ),
                                moveToTransfer
                            ),

                            new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.TRANSFER),
                            new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.TRANSFER),
                            new WaitUntilCommand(arm::areAxonsCloseToTransferPos),
                            new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.PICKUP)
                        )
                );
                break;

            case LOW:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> arm.isArmUp = true),
                                new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.LEFT, false)),
                                new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.RIGHT, false)),
                                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED),
                                new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.LOWPOS),
                                new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.LOWPOS),
                                new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.LOW)
                        ));
                break;

            case MID:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> arm.isArmUp = true),
                                new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.LEFT, false)),
                                new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.RIGHT, false)),
                                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED),
                                new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.MIDPOS),
                                new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.MIDPOS),
                                new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.MID)
                        ));
                break;

            case HIGH:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> arm.isArmUp = true),
                                new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.LEFT, false)),
                                new InstantCommand(()-> arm.changeClawLockState(ArmSubsystem.CLAW.RIGHT, false)),
                                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED),
                                new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.HIGHPOS),
                                new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.HIGHPOS),
                                new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.HIGH)
                        ));
                break;
        }
    }



    public enum ArmState{
        PICKUP,
        TRANSFER,
        LOW,
        MID,
        HIGH
    }
}